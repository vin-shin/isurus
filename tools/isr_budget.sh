#!/usr/bin/env bash
# Control-ISR timing budget, measured on the target.
#
# The 30 kHz ISR has a hard 1/PWM_FREQ_HZ deadline and every feature added to
# it spends from the same account, so "did that cost anything?" has to be
# answerable with a number rather than an opinion. This runs the real ISR on
# the real silicon and reports where the budget went.
#
# Usage:  tools/isr_budget.sh [samples]        default 240
#
# What it does NOT do is energise the bridge. The ISR body is gated on
# g_foc.enabled, which this sets; the gate driver outputs are a separate thing
# (MotorPwm_EnableOutputs) and are deliberately left alone. The ISR executes
# exactly the same instructions either way, so the timing is honest while the
# motor stays inert and no current flows.
#
# Two details that are easy to get wrong and produce believable nonsense:
#
#   - The angle must be swept. sinf/cosf and the CORDIC both cost slightly
#     different amounts in different quadrants, and a stationary rotor only
#     ever shows one of them. g_foc.elec_offset walks the electrical angle
#     through a full turn without moving anything.
#
#   - The core must be HALTED to read a coherent sample. A running 30 kHz ISR
#     rewrites these words underneath a multi-word SWD read, and with 20 pole
#     pairs one encoder LSB of jitter is 0.22 degrees electrical - enough to
#     make consistent values look inconsistent.

set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ELF="$ROOT/build/Debug/makolongfin2.elf"
SAMPLES="${1:-240}"

NM="${NM:-arm-none-eabi-nm}"
command -v "$NM" >/dev/null 2>&1 || \
  NM="/c/Program Files (x86)/GNU Arm Embedded Toolchain/10 2021.10/bin/arm-none-eabi-nm"

[ -f "$ELF" ] || { echo "No ELF at $ELF - run 'ninja -C build/Debug' first." >&2; exit 1; }

# Addresses come from the ELF every run, never from a constant in this file.
# A code-size change moves BSS, and a stale address does not error - it reads
# some other variable and looks exactly like the firmware ignoring you.
sym() {
  local a
  a="$("$NM" "$ELF" | awk -v s="$1" '$3 == s { print $1 }')"
  [ -n "$a" ] || { echo "Symbol '$1' not found - rebuild?" >&2; exit 1; }
  echo "$((0x$a))"
}

FOC=$(sym g_foc)
# Offsets into FocState_t. The struct is a dense chain of 4-byte fields from
# offset 0; tools/foc_dash.sh pins the same layout at +108/+112/+116.
ENA=$(printf  "0x%x" $((FOC+112)))   # enabled
ISRMAX=$(printf "0x%x" $((FOC+108))) # isr_max, whole ISR
EOFF=$(printf "0x%x" $((FOC+84)))    # elec_offset
CTRLCYC=$(printf "0x%x" $(sym g_isr_ctrl_cyc))
CTRLMAX=$(printf "0x%x" $(sym g_isr_ctrl_max))
ENCCYC=$(printf  "0x%x" $(sym g_isr_enc_cyc))
ENCMAX=$(printf  "0x%x" $(sym g_isr_enc_max))

TMP="$(mktemp -d)"
CFG="$TMP/budget.cfg"
trap 'rm -rf "$TMP"' EXIT INT TERM

STEP=$(( 32768 / SAMPLES ))
[ "$STEP" -lt 1 ] && STEP=1

cat > "$CFG" <<EOF
source [find interface/stlink.cfg]
transport select hla_swd
source [find target/stm32g4x.cfg]
reset_config none
adapter speed 4000
init

mww $ENA 1
sleep 50
mww $CTRLMAX 0
mww $ENCMAX 0
mww $ISRMAX 0

for {set o 0} {\$o < 32768} {incr o $STEP} {
    mww $EOFF \$o
    sleep 3
    halt
    echo "S [expr {[read_memory $CTRLCYC 32 1]}] [expr {[read_memory $ENCCYC 32 1]}]"
    resume
}

mww $EOFF 0
sleep 20
halt
echo "M [expr {[read_memory $CTRLMAX 32 1]}] [expr {[read_memory $ENCMAX 32 1]}] [expr {[read_memory $ISRMAX 32 1]}]"
resume
mww $ENA 0
shutdown
EOF

openocd -s "${OPENOCD_SCRIPTS:-C:/msys64/mingw64/share/openocd/scripts}" -d0 -f "$CFG" 2>&1 \
  | grep -E "^[SM] " > "$TMP/raw.txt" || true

[ -s "$TMP/raw.txt" ] || { echo "No samples - is the board attached?" >&2; exit 1; }

# 128 cycles = 1 us: HSI16 -> PLLN=16, PLLM=1, PLLR=2 -> SYSCLK 128 MHz.
# The deadline is 1/PWM_FREQ_HZ, read from the header rather than written out,
# because a switching-frequency change must move this report with it.
PWM_HZ=$(awk '/#define +PWM_FREQ_HZ/ { gsub(/[^0-9]/,"",$3); print $3 }' "$ROOT/Core/Inc/motor_pwm.h")

python - "$TMP/raw.txt" "$PWM_HZ" <<'PY'
import sys
rows = [l.split() for l in open(sys.argv[1])]
pwm  = int(sys.argv[2])
CPU  = 128e6
budget = CPU / pwm                       # cycles available per control period

S = [(int(r[1],0), int(r[2],0)) for r in rows if r[0] == 'S']
M = [(int(r[1],0), int(r[2],0), int(r[3],0)) for r in rows if r[0] == 'M']

def stat(v):
    v = sorted(v); n = len(v)
    return v[0], v[n//2], v[min(n-1, int(n*0.95))], v[-1]

print(f"\n  {len(S)} samples across a full electrical turn, "
      f"{pwm/1000:.0f} kHz period = {budget:.0f} cycles ({1e6/pwm:.2f} us)\n")
print(f"  {'stage':<22}{'min':>8}{'med':>8}{'p95':>8}{'max':>8}   {'max us':>8}")
print(f"  {'-'*62}")
for name, vals in (("control (FOC+pos)", [a for a,_ in S]), ("encoder read", [b for _,b in S])):
    lo, md, p95, hi = stat(vals)
    print(f"  {name:<22}{lo:>8}{md:>8}{p95:>8}{hi:>8}   {hi/128.0:>8.2f}")

if M:
    cm, em, im = M[0]
    print(f"\n  latched worst case over the whole sweep:")
    print(f"    g_isr_ctrl_max  {cm:>6} cyc  {cm/128.0:6.2f} us")
    print(f"    g_isr_enc_max   {em:>6} cyc  {em/128.0:6.2f} us")
    print(f"    g_foc.isr_max   {im:>6} cyc  {im/128.0:6.2f} us   "
          f"{100.0*im/budget:.1f}% of budget")
    head = budget - im
    print(f"\n    headroom        {head:>6.0f} cyc  {head/128.0:6.2f} us")
PY
