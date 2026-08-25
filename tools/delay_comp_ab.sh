#!/usr/bin/env bash
# A/B the transport-delay compensation against the d axis.
#
# The claim 1a makes is narrow and testable: using one angle for both Park
# transforms leaves the applied vector lagging the rotor, and that lag leaks
# q-axis command into d. So with id_ref held at zero, |id| while spinning is
# the error signal. Turn the compensation off and it should get worse.
#
# THIS SPINS THE MOTOR. It energises the power stage, runs the velocity loop
# at a commanded speed, and only stops when it is done or you Ctrl-C. Clear the
# bench first.
#
# Usage:  tools/delay_comp_ab.sh --run [vel_dps] [rounds]
#
#   vel_dps  mechanical deg/s, default 3600 = LIM_VEL_MAX_DPS = 10 rev/s.
#            With 20 pole pairs that is 200 Hz electrical, which is as fast as
#            this bench goes and where the effect is largest.
#   rounds   on/off pairs, default 3. It alternates rather than measuring one
#            block of each, so a drift in temperature or supply cannot be
#            mistaken for the effect.
#
# Without --run it prints this header and exits, because a script that spins a
# motor as a side effect of being curious about its options is a bad script.

set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ELF="$ROOT/build/Debug/makolongfin2.elf"

if [ "${1:-}" != "--run" ]; then
  # Print the comment header above, stopping at the first line of actual code,
  # so this cannot drift out of range when the header is edited.
  awk 'NR>1 { if (!/^#/) exit; sub(/^# ?/, ""); print }' "$0"
  exit 1
fi
shift
VEL="${1:-3600}"
ROUNDS="${2:-3}"

NM="${NM:-arm-none-eabi-nm}"
command -v "$NM" >/dev/null 2>&1 || \
  NM="/c/Program Files (x86)/GNU Arm Embedded Toolchain/10 2021.10/bin/arm-none-eabi-nm"

[ -f "$ELF" ] || { echo "No ELF at $ELF - run 'ninja -C build/Debug' first." >&2; exit 1; }

sym() {
  local a
  a="$("$NM" "$ELF" | awk -v s="$1" '$3 == s { print $1 }')"
  [ -n "$a" ] || { echo "Symbol '$1' not found - rebuild?" >&2; exit 1; }
  echo "$((0x$a))"
}

FOC=$(sym g_foc); POS=$(sym g_pos); CMD=$(sym g_cmd); FLT=$(sym g_faulted)

IDA=$(printf   "0x%x" $((FOC+0)))     # id_ref, float
IQA=$(printf   "0x%x" $((FOC+4)))     # iq_ref, float
ENA=$(printf   "0x%x" $((FOC+112)))   # g_foc.enabled
IDMA=$(printf  "0x%x" $((FOC+116)))   # id_ma  mirror
IQMA=$(printf  "0x%x" $((FOC+120)))   # iq_ma  mirror
DC=$(printf    "0x%x" $((FOC+196)))   # delay_comp flag
OM=$(printf    "0x%x" $((FOC+200)))   # omega_e, tenths of rad/s
ADV=$(printf   "0x%x" $((FOC+204)))   # applied advance, tenths of a degree
PENA=$(printf  "0x%x" $((POS+0)))
PMODE=$(printf "0x%x" $((POS+92)))
PVCMD=$(printf "0x%x" $((POS+100)))   # vel_cmd_dps
CMDA=$(printf  "0x%x" $CMD)
FLTA=$(printf  "0x%x" $FLT)

TMP="$(mktemp -d)"
CFG="$TMP/ab.cfg"; SAFECFG="$TMP/safe.cfg"
OCD_PID=""; CLEANED=0

# Killing OpenOCD does NOT stop the motor - the firmware keeps running whatever
# was last commanded. Same reattach-on-exit as tools/foc_dash.sh, and for the
# same reason: without it, Ctrl-C leaves the motor spinning with no way to stop
# it short of a power cycle.
cat > "$SAFECFG" <<EOF
source [find interface/stlink.cfg]
transport select hla_swd
source [find target/stm32g4x.cfg]
reset_config none
adapter speed 4000
init
mww $PVCMD 0
mww $PMODE 0
mww $PENA 0
mww $IQA 0
mww $IDA 0
mww $ENA 0
mww $DC 1
mww [expr {$CMDA+20}] 0
mww [expr {$CMDA+16}] 0
mww $CMDA 1
shutdown
EOF

cleanup() {
  [ "$CLEANED" = "1" ] && return
  CLEANED=1
  if [ -n "$OCD_PID" ]; then kill "$OCD_PID" 2>/dev/null || true; wait "$OCD_PID" 2>/dev/null || true; fi
  sleep 1
  echo "stopping motor..." >&2
  if openocd -s "${OPENOCD_SCRIPTS:-C:/msys64/mingw64/share/openocd/scripts}" -d0 -f "$SAFECFG" >/dev/null 2>&1; then
    echo "power stage safe." >&2
  else
    echo "!! COULD NOT REATTACH - POWER DOWN THE BOARD MANUALLY !!" >&2
  fi
  rm -rf "$TMP"
}
trap cleanup EXIT INT TERM

echo "Spinning at $VEL deg/s for $ROUNDS on/off rounds. Ctrl-C stops it."
sleep 1

{
cat <<EOF
source [find interface/stlink.cfg]
transport select hla_swd
source [find target/stm32g4x.cfg]
reset_config none
adapter speed 4000
init

# Bring the stage up in the order main.c enforces - duty, then outputs, then
# gates - then clear any latched fault and hand control to FOC.
mww [expr {$CMDA+4}] 0
mww [expr {$CMDA+8}] 0
mww [expr {$CMDA+12}] 0
mww [expr {$CMDA+16}] 1
mww $CMDA 1
sleep 200
mww [expr {$CMDA+20}] 1
mww $CMDA 1
sleep 200
mww $FLTA 0
mww $IDA 0
mww $ENA 1
sleep 100

# Velocity mode. id_ref stays at zero throughout - that is the whole point,
# because id is then pure error and anything non-zero in it is the loop
# failing to put the vector where it meant to.
mww $PMODE 2
mww $PVCMD $VEL
mww $PENA 1
sleep 1500
EOF

for r in $(seq 1 "$ROUNDS"); do
  for dc in 1 0; do
    echo "mww $DC $dc"
    echo "sleep 400"          # let the loop settle after the step change
    for i in $(seq 1 40); do
      echo "echo \"D $dc [expr {[read_memory $IDMA 32 1]}] [expr {[read_memory $IQMA 32 1]}] [expr {[read_memory $OM 32 1]}] [expr {[read_memory $ADV 32 1]}]\""
      echo "sleep 12"
    done
  done
done

cat <<EOF
mww $DC 1
mww $PVCMD 0
mww $PMODE 0
mww $PENA 0
sleep 800
mww $ENA 0
mww [expr {$CMDA+20}] 0
mww [expr {$CMDA+16}] 0
mww $CMDA 1
shutdown
EOF
} > "$CFG"

openocd -s "${OPENOCD_SCRIPTS:-C:/msys64/mingw64/share/openocd/scripts}" -d0 -f "$CFG" 2>&1 \
  | grep -E "^D " > "$TMP/raw.txt" || true

[ -s "$TMP/raw.txt" ] || { echo "No samples collected." >&2; exit 1; }

python - "$TMP/raw.txt" <<'PY'
import sys, math
def s32(v): return v - (1 << 32) if v & 0x80000000 else v
on, off = [], []
om, adv = [], []
for line in open(sys.argv[1]):
    p = line.split()
    dc = int(p[1]); idm = s32(int(p[2], 16)); iqm = s32(int(p[3], 16))
    (on if dc else off).append((idm, iqm))
    if dc:
        om.append(s32(int(p[4], 16)) / 10.0)
        adv.append(s32(int(p[5], 16)) / 10.0)

def stats(v):
    d = [a for a, _ in v]; q = [b for _, b in v]
    n = len(d)
    mean = sum(d) / n
    rms  = math.sqrt(sum(x * x for x in d) / n)
    return mean, rms, max(abs(x) for x in d), sum(q) / n, n

print()
if om:
    print(f"  spinning at {sum(om)/len(om):7.1f} elec rad/s "
          f"({sum(om)/len(om)/(2*math.pi):5.1f} Hz electrical), "
          f"advance applied {sum(adv)/len(adv):.2f} deg")
print(f"\n  id_ref = 0 throughout, so every mA of id is error.\n")
print(f"  {'delay_comp':<12}{'mean id':>10}{'rms id':>10}{'peak |id|':>11}{'mean iq':>10}{'n':>6}")
print(f"  {'-'*59}")
res = {}
for name, v in (("ON", on), ("OFF", off)):
    if not v: continue
    mean, rms, pk, iq, n = stats(v)
    res[name] = rms
    print(f"  {name:<12}{mean:>9.0f} {rms:>9.1f} {pk:>10.0f} {iq:>9.0f} {n:>6}")
if "ON" in res and "OFF" in res and res["OFF"] > 0:
    ch = 100.0 * (res["ON"] - res["OFF"]) / res["OFF"]
    print(f"\n  rms id with compensation ON is {ch:+.1f}% vs OFF "
          f"({'better' if ch < 0 else 'WORSE - investigate'})")
PY
