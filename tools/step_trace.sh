#!/usr/bin/env bash
# Step iq_ref on a spinning rotor and capture the current loop's answer at the
# ISR rate, with the decoupling feedforward on and off.
#
# This is the test the steady-state A/B could not do. A step is where the
# integrator has NOT yet absorbed the disturbance, so what the feedforward is
# worth shows up directly: the d axis should barely move if the cross-coupling
# is being cancelled, and should be visibly kicked if it is not.
#
# THIS SPINS THE MOTOR, freely and under torque control - it accelerates until
# back-EMF meets the modulation ceiling, which on this bench is about 600 rpm.
# Clear the bench first.
#
# Usage:  tools/step_trace.sh --run [iq_lo_ma] [iq_hi_ma] [reps] [flag]
#
# flag selects which correction is toggled: "decouple" (default) or
# "delay_comp". The OTHER one is left at its default of on, because the
# question being asked is what each is worth in the loop as it actually
# ships, not in isolation from the rest of it.
#
# reps defaults to 12. One step per condition is one sample of a noisy thing:
# rotor position, ripple phase and speed all differ between trials, so the
# report gives mean +/- spread across reps rather than a single number.
#
# Method, and why it is arranged this way:
#
#   - The position loop is stood DOWN and iq_ref driven directly. Going
#     through MODE_TORQUE would put the 150 Hz output filter in front of the
#     step and there would be no step left to measure.
#   - The rotor free-spins up on iq_lo first. The mechanical time constant is
#     hundreds of ms and the current loop settles in about one, so during the
#     capture the speed is effectively constant - which is what makes the
#     comparison between the two flag states fair.
#   - The step is fired by the FIRMWARE, not from here: write g_step_iq_ma,
#     then g_step_req, and the control ISR arms the trace and applies the new
#     setpoint a fixed 60 ticks later. Doing it with two SWD writes does not
#     work - OpenOCD command latency is milliseconds and varies, which is the
#     same order as the whole 17 ms window, so the edge lands anywhere. That
#     was measured, not assumed: it came out at sample 487 of 512.

set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ELF="$ROOT/build/Debug/makolongfin2.elf"

if [ "${1:-}" != "--run" ]; then
  awk 'NR>1 { if (!/^#/) exit; sub(/^# ?/, ""); print }' "$0"
  exit 1
fi
shift
IQ_LO_MA="${1:-400}"
IQ_HI_MA="${2:-1200}"
REPS="${3:-12}"
FLAG="${4:-decouple}"

NM="${NM:-arm-none-eabi-nm}"
command -v "$NM" >/dev/null 2>&1 || \
  NM="/c/Program Files (x86)/GNU Arm Embedded Toolchain/10 2021.10/bin/arm-none-eabi-nm"
[ -f "$ELF" ] || { echo "No ELF at $ELF - run 'ninja -C build/Debug' first." >&2; exit 1; }

sym() {
  local a; a="$("$NM" "$ELF" | awk -v s="$1" '$3 == s { print $1 }')"
  [ -n "$a" ] || { echo "Symbol '$1' not found - rebuild?" >&2; exit 1; }
  echo "$((0x$a))"
}
fbits() { python -c "import struct,sys;print(struct.unpack('<I',struct.pack('<f',float(sys.argv[1])))[0])" "$1"; }

FOC=$(sym g_foc); POS=$(sym g_pos); CMD=$(sym g_cmd); FLT=$(sym g_faulted); TRC=$(sym g_trace)

IDA=$(printf "0x%x" $((FOC+0)));   IQA=$(printf "0x%x" $((FOC+4)))
ENA=$(printf "0x%x" $((FOC+112))); DC=$(printf "0x%x" $((FOC+196)))
OM=$(printf "0x%x" $((FOC+200)));  DEC=$(printf "0x%x" $((FOC+208)))
VQFF=$(printf "0x%x" $((FOC+220)))
TCOUNT=$(printf "0x%x" $((TRC+8)))
VBUSU=$(printf "0x%x" $((FOC+176)))   # vbus_used_mv
STEPIQ=$(printf "0x%x" $(sym g_step_iq_ma))
STEPRQ=$(printf "0x%x" $(sym g_step_req))
PENA=$(printf "0x%x" $((POS+0)));  PMODE=$(printf "0x%x" $((POS+92)))
CMDA=$(printf "0x%x" $CMD);        FLTA=$(printf "0x%x" $FLT)
TARM=$(printf "0x%x" $((TRC+0)));  TDONE=$(printf "0x%x" $((TRC+4)))
TDECIM=$(printf "0x%x" $((TRC+12))); TBUF=$((TRC+20))

ADV=$(printf "0x%x" $((FOC+204)))     # theta_adv_deg_x10

case "$FLAG" in
  decouple)   FLAGA="$DEC" ;;
  delay_comp) FLAGA="$DC"  ;;
  *) echo "flag must be 'decouple' or 'delay_comp', got '$FLAG'" >&2; exit 1 ;;
esac

IQ_LO=$(fbits "$(python -c "print($IQ_LO_MA/1000.0)")")
IQ_HI=$(fbits "$(python -c "print($IQ_HI_MA/1000.0)")")

TMP="$(mktemp -d)"; CFG="$TMP/step.cfg"; SAFECFG="$TMP/safe.cfg"; CLEANED=0
cat > "$SAFECFG" <<EOF
source [find interface/stlink.cfg]
transport select hla_swd
source [find target/stm32g4x.cfg]
reset_config none
adapter speed 4000
init
mww $PMODE 0
mww $PENA 0
mww $IQA 0
mww $IDA 0
mww $ENA 0
mww $DEC 1
mww $DC 1
mww $TARM 0
mww [expr {$CMDA+20}] 0
mww [expr {$CMDA+16}] 0
mww $CMDA 1
shutdown
EOF
cleanup() {
  [ "$CLEANED" = "1" ] && return; CLEANED=1
  sleep 1; echo "stopping motor..." >&2
  if openocd -s "${OPENOCD_SCRIPTS:-C:/msys64/mingw64/share/openocd/scripts}" -d0 -f "$SAFECFG" >/dev/null 2>&1
  then echo "power stage safe." >&2
  else echo "!! COULD NOT REATTACH - POWER DOWN THE BOARD MANUALLY !!" >&2; fi
  rm -rf "$TMP"
}
trap cleanup EXIT INT TERM

echo "Spinning up on ${IQ_LO_MA} mA, stepping to ${IQ_HI_MA} mA, toggling ${FLAG}. Ctrl-C stops it."
sleep 1

{
cat <<EOF
source [find interface/stlink.cfg]
transport select hla_swd
source [find target/stm32g4x.cfg]
reset_config none
adapter speed 4000
init
mww [expr {$CMDA+4}] 0
mww [expr {$CMDA+8}] 0
mww [expr {$CMDA+12}] 0
mww [expr {$CMDA+16}] 1
mww $CMDA 1
sleep 200
mww [expr {$CMDA+20}] 1
mww $CMDA 1
sleep 200
# g_faulted is a derived mirror now - writing it clears nothing. The real
# entry is g_cmd.clear_fault, which re-runs the self-test. See drive.h.
mww [expr {$CMDA+36}] 1
mww $CMDA 1
sleep 300
mww $PMODE 0
mww $PENA 0
mww $IDA 0
mww $ENA 1
mww $TDECIM 1
sleep 100
EOF

for dc in 1 0; do
  # Spin up once per condition, then step repeatedly from the same base
  # current. Re-accelerating between every rep would drift the speed and make
  # the reps incomparable.
  echo "mww $FLAGA $dc"
  echo "mww $IQA $IQ_LO"
  echo "sleep 2500"
  for rep in $(seq 1 "$REPS"); do
    echo "mww $IQA $IQ_LO"
    echo "sleep 500"
    echo "mww $STEPIQ $IQ_HI_MA"
    echo "mww $STEPRQ 1"
    echo "sleep 60"
    echo "echo \"META $dc $rep [expr {[read_memory $OM 32 1]}] [expr {[read_memory $VQFF 32 1]}] [expr {[read_memory $TDONE 32 1]}] [expr {[read_memory $TCOUNT 32 1]}] [expr {[read_memory $VBUSU 32 1]}] [expr {[read_memory $ADV 32 1]}]\""
    for i in $(seq 0 15); do
      echo "echo \"T $dc $rep $i [read_memory [expr {$TBUF + $i*256}] 16 128]\""
    done
  done
  echo "mww $IQA $IQ_LO"
  echo "sleep 400"
done

cat <<EOF
mww $DEC 1
mww $DC 1
mww $IQA 0
mww $IDA 0
sleep 300
mww $ENA 0
mww [expr {$CMDA+20}] 0
mww [expr {$CMDA+16}] 0
mww $CMDA 1
shutdown
EOF
} > "$CFG"

openocd -s "${OPENOCD_SCRIPTS:-C:/msys64/mingw64/share/openocd/scripts}" -d0 -f "$CFG" 2>&1 \
  | grep -E "^(T|META) " > "$TMP/raw.txt" || true
[ -s "$TMP/raw.txt" ] || { echo "No trace captured." >&2; exit 1; }

cp "$TMP/raw.txt" "${STEP_TRACE_RAW:-$ROOT/step_trace_raw.txt}"
python "$ROOT/tools/step_trace_analyze.py" "$TMP/raw.txt" "$IQ_LO_MA" "$IQ_HI_MA" "$FLAG"
