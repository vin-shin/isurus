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
#   rounds   on/off pairs, default 6. It alternates rather than measuring one
#            block of each, so a drift in temperature or supply cannot be
#            mistaken for the effect, and each round is compared against its
#            own neighbour rather than against a pooled average.
#   n        samples per block, default 300.
#
# Sample sizing is not arbitrary. id at id_ref = 0 sits on a noise floor of
# ~148 mA pp from the ADC and PWM ripple (see POS_OUT_LPF_HZ in position.h),
# and the effect being looked for is a few percent of the rms on top of it.
# The relative standard error of an rms estimate is about 1/sqrt(2n), so 40
# samples per block resolves nothing finer than ~11% and even 120 leaves 6.5% -
# the same size as the effect. 300 per block over 6 rounds puts 1800 samples
# behind each condition, or ~1.7%, which is enough to separate them. The
# report prints the paired per-round spread so this is checkable rather than
# asserted.
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
ROUNDS="${2:-6}"
NSAMP="${3:-300}"

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
    echo "sleep 400"          # let the loop settle after the flag change
    for i in $(seq 1 "$NSAMP"); do
      # 6 ms apart. The mirrors refresh at 1 kHz and the electrical period is
      # 5 ms at 200 Hz, so consecutive samples are independent rather than
      # repeatedly catching the same point of the same ripple cycle.
      echo "echo \"D $r $dc [expr {[read_memory $IDMA 32 1]}] [expr {[read_memory $IQMA 32 1]}] [expr {[read_memory $OM 32 1]}] [expr {[read_memory $ADV 32 1]}]\""
      echo "sleep 6"
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

rounds = {}          # (round, flag) -> list of id
iqs    = {}
om, adv = [], []
for line in open(sys.argv[1]):
    p = line.split()
    r = int(p[1]); dc = int(p[2])
    rounds.setdefault((r, dc), []).append(s32(int(p[3], 16)))
    iqs.setdefault(dc, []).append(s32(int(p[4], 16)))
    if dc:
        om.append(s32(int(p[5], 16)) / 10.0)
        adv.append(s32(int(p[6], 16)) / 10.0)

rms  = lambda v: math.sqrt(sum(x * x for x in v) / len(v))
mean = lambda v: sum(v) / len(v)

print()
if om:
    print(f"  spinning at {mean(om):7.1f} elec rad/s "
          f"({mean(om)/(2*math.pi):5.1f} Hz electrical), "
          f"advance applied {mean(adv):.2f} deg")
print("\n  id_ref = 0 throughout, so every mA of id is error.")
print("  Note mean id is nulled by the d-axis integrator either way - the")
print("  angle error shows up in the rms, not the DC value.\n")

rn = sorted({r for r, _ in rounds})
print(f"  {'round':<8}{'rms id ON':>11}{'rms id OFF':>12}{'delta %':>10}")
print(f"  {'-'*41}")
deltas = []
for r in rn:
    a, b = rounds.get((r, 1)), rounds.get((r, 0))
    if not a or not b: continue
    ra, rb = rms(a), rms(b)
    d = 100.0 * (ra - rb) / rb
    deltas.append(d)
    print(f"  {r:<8}{ra:>11.1f}{rb:>12.1f}{d:>+10.1f}")

allon  = [x for (r, f), v in rounds.items() if f == 1 for x in v]
alloff = [x for (r, f), v in rounds.items() if f == 0 for x in v]
print(f"\n  {'pooled':<8}{rms(allon):>11.1f}{rms(alloff):>12.1f}"
      f"{100.0*(rms(allon)-rms(alloff))/rms(alloff):>+10.1f}")
print(f"  mean id      {mean(allon):>+7.0f} mA (ON)   {mean(alloff):>+7.0f} mA (OFF)")
print(f"  mean iq      {mean(iqs[1]):>+7.0f} mA (ON)   {mean(iqs[0]):>+7.0f} mA (OFF)")
print(f"  samples      {len(allon):>7} (ON)   {len(alloff):>7} (OFF)")

# Two error bars, because they answer different questions.
#
# The POOLED one is analytic: the relative standard error of an rms estimate
# from n independent samples is 1/sqrt(2n), so the ratio of two carries
# sqrt(1/2n_on + 1/2n_off). This is the right primary statistic - it uses all
# the samples instead of collapsing each round to a single number, which is
# what makes a handful of rounds unable to resolve anything.
#
# The PER-ROUND spread is the cross-check. If it is much wider than the
# analytic figure, the samples are not as independent as assumed (aliasing
# against the ripple) or something drifted during the run, and the analytic
# bar is then optimistic. Disagreement between the two is the signal to
# distrust the result, so both are printed.
pooled = 100.0 * (rms(allon) - rms(alloff)) / rms(alloff)
se_an  = 100.0 * math.sqrt(1.0 / (2 * len(allon)) + 1.0 / (2 * len(alloff)))
print(f"\n  pooled delta     {pooled:+.1f}% +/- {se_an:.1f}% (analytic, 1 se)")

if len(deltas) >= 2:
    m  = mean(deltas)
    sd = math.sqrt(sum((d - m) ** 2 for d in deltas) / (len(deltas) - 1))
    se = sd / math.sqrt(len(deltas))
    print(f"  per-round delta  {m:+.1f}% +/- {se:.1f}% (empirical, {len(deltas)} rounds)")
    if se > 2.5 * se_an:
        print("  -> per-round spread far exceeds the analytic error: samples are")
        print("     correlated or the bench drifted. Trust the wider bar.")

t = abs(pooled) / se_an if se_an > 0 else 0.0
print(f"\n  |t| = {t:.1f} against the analytic error")
if t < 2.0:
    print("  Not resolved. This test is a weak probe by construction: the d-axis")
    print("  integrator nulls steady-state id whether or not the angle is right,")
    print("  so only the ripple carries the effect. A step in iq_ref is the")
    print("  transient the compensation actually improves - see phase 1b.")
elif pooled < 0:
    print("  Compensation reduces d-axis error.")
else:
    print("  Compensation made it WORSE - check the sign of omega_e before")
    print("  trusting anything downstream of this.")
PY
