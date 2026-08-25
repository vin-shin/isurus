#!/usr/bin/env bash
# Standstill noise A/B. Holds position and cycles the two anti-whistle filters
# so you can pick a setting by ear.
#
# Why this exists as its own tool: judging the noise needs the servo energised
# and holding, which means something has to own the ST-Link - and only one
# process can. pos_dash.sh occupies it for the whole run, so there is no way to
# change a filter live while listening. This holds the shaft still and does the
# sweeping itself.
#
# The rotor does not move. It is held at wherever it was when you started.
#
# Usage:  tools/pos_listen.sh [seconds_per_setting]      (default 5)
#
# What you are listening for: the position loop's PID runs at 1 kHz while the
# current loop consumes its output at the PWM rate, so any dither in the PID
# output reaches the motor as a 1 kHz staircase - right in the audible band.
# The dominant source at standstill is kd multiplying the velocity estimate's
# quantisation noise. Both knobs below attack that:
#
#   vel_filt   how hard the velocity estimate is filtered before kd sees it.
#              This is the big lever. Lower = quieter, but it adds phase lag
#              to the damping term, so overshoot creeps up.
#   out_lpf    a low-pass on the current command itself, at the full PWM rate.
#              Rounds off the staircase edges whatever made them.
#
# Anything still audible with BOTH at their most aggressive is not the position
# loop: the current loop alone measures ~148 mA pp of ripple at zero command
# from ADC noise and PWM ripple. The switching tone itself is no longer a
# candidate - at 30 kHz it is above hearing.

set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ELF="$ROOT/build/Debug/makolongfin2.elf"
HOLD="${1:-5}"

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

POS=$(sym g_pos); FOC=$(sym g_foc); CMD=$(sym g_cmd); FLT=$(sym g_faulted)
PENA=$(printf "0x%x" $((POS+0)));  PZER=$(printf "0x%x" $((POS+36)))
PLPF=$(printf "0x%x" $((POS+80))); PVF=$(printf "0x%x" $((POS+84)))
PMODE=$(printf "0x%x" $((POS+92)))   # MOTION_MODE_POSITION == 3
PBASE=$(printf "0x%x" $POS)
IDA=$(printf "0x%x" $((FOC+0)));   IQA=$(printf "0x%x" $((FOC+4)))
FENA=$(printf "0x%x" $((FOC+112)))
CMDA=$(printf "0x%x" $CMD);        FLTA=$(printf "0x%x" $FLT)

CFG="$(mktemp)"; SAFECFG="$(mktemp)"; OCD_PID=""; CLEANED=0

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
mww $FENA 0
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
  echo "disengaging servo..." >&2
  if openocd -s "${OPENOCD_SCRIPTS:-C:/msys64/mingw64/share/openocd/scripts}" -d0 -f "$SAFECFG" >/dev/null 2>&1; then
    echo "power stage safe." >&2
  else
    echo "!! COULD NOT REATTACH - POWER DOWN THE BOARD MANUALLY !!" >&2
  fi
  rm -f "$CFG" "$SAFECFG"
}
trap cleanup EXIT INT TERM

echo "This energises the motor and HOLDS it still. Ctrl-C to stop."
echo "Listen during each step; the quietest one wins."
sleep 1

cat > "$CFG" <<EOF
source [find interface/stlink.cfg]
transport select hla_swd
source [find target/stm32g4x.cfg]
reset_config none
adapter speed 4000
init
proc s32 {v} { if {\$v > 0x7FFFFFFF} { return [expr {\$v - 0x100000000}] } ; return \$v }
proc mwi {a v} { mww \$a [expr {\$v & 0xFFFFFFFF}] }

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
mww $IDA 0
mww $IQA 0
mww $FENA 1
sleep 200
# Hold position explicitly; see the note in pos_dash.sh - the mode is sticky
# and another tool may have left it at IDLE.
mwi $PMODE 3
mww $PENA 1
sleep 200
mww $PZER 1
sleep 400

# {out_lpf_hz  vel_filt_x1000  label}
set sweep {
       0  200 "everything off      (worst case - the original behaviour)"
       0  100 "vel filter only"
     300  200 "output filter only"
     300  100 "BOTH  (current defaults)"
     150   50 "both, aggressive"
      80   30 "both, very aggressive (watch for overshoot later)"
}

puts ""
foreach {lpf vf lbl} \$sweep {
    mwi $PLPF \$lpf
    mwi $PVF  \$vf
    sleep 600
    # measure while you listen
    set rmn 99999 ; set rmx -99999 ; set omn 99999 ; set omx -99999
    set vmn 99999 ; set vmx -99999
    for {set i 0} {\$i < 200} {incr i} {
        set p [read_memory $PBASE 32 23]
        set o [s32 [lindex \$p 16]] ; set r [s32 [lindex \$p 22]] ; set v [s32 [lindex \$p 13]]
        if {\$r < \$rmn} {set rmn \$r} ; if {\$r > \$rmx} {set rmx \$r}
        if {\$o < \$omn} {set omn \$o} ; if {\$o > \$omx} {set omx \$o}
        if {\$v < \$vmn} {set vmn \$v} ; if {\$v > \$vmx} {set vmx \$v}
    }
    puts [format "  out_lpf %3d Hz  vel_filt %.2f   ripple %3d mA pp   vel noise %3d d/s   %s" \\
          \$lpf [expr {\$vf/1000.0}] [expr {\$omx-\$omn}] [expr {\$vmx-\$vmn}] \$lbl]
    flush stdout
    sleep [expr {$HOLD * 1000}]
}
puts ""
puts "Sweep done. To keep a setting, put it in Core/Inc/position.h:"
puts "   POS_OUT_LPF_HZ   and   POS_VEL_ALPHA"
puts "then rebuild and reflash."
mww $PENA 0
mww $IQA 0
mww $FENA 0
mww [expr {$CMDA+20}] 0
mww [expr {$CMDA+16}] 0
mww $CMDA 1
shutdown
EOF

openocd -s "${OPENOCD_SCRIPTS:-C:/msys64/mingw64/share/openocd/scripts}" -d0 -f "$CFG" &
OCD_PID=$!
wait "$OCD_PID"
