#!/usr/bin/env bash
# Live position-control dashboard. Runs until Ctrl-C.
#
# The outer loop's job is visible in four places at once:
#   - a mechanical dial carrying two needles. The rotor is the INNER needle
#     (=== with an O tip); the target is an OUTER tick (* on the rim). They sit
#     on different radii on purpose - drawn on the same one, the rotor paints
#     over the target exactly when the loop is tracking, which is when you most
#     want to see that the two agree.
#   - a multi-turn ruler, because the dial alone cannot tell +10 from +370
#   - a scrolling trace of the position error, which is where overshoot,
#     ringing and steady-state droop actually show up
#   - the current the loop is asking the FOC for, against its clamp
#
# Same constraints as foc_dash.sh: all rendering happens inside OpenOCD's Tcl
# because piping would block-buffer and destroy the redraw, and jimtcl has no
# math functions, so the trig is an integer sine table with quadrant symmetry.
#
# Usage:  tools/pos_dash.sh [interval_ms]          just watch
#         tools/pos_dash.sh --demo [interval_ms]   run a move sequence
#
# --demo energises the stage, zeroes the position at wherever the rotor is
# sitting, then works through twelve moves: fine steps at the encoder's
# resolution limit, quarter and half turns, a full sweep, and multi-turn runs
# at three different slew rates. Every move is commanded relative to that zero,
# so it is safe from any rotor angle. Each completed move is scored for settle
# time and peak error, and the last three scores stay on screen.

set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ELF="$ROOT/build/Debug/makolongfin2.elf"
DEMO=0
if [ "${1:-}" = "--demo" ]; then DEMO=1; shift; fi
INTERVAL="${1:-60}"

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

POS=$(sym g_pos)
FOC=$(sym g_foc)
CS=$(sym g_cs)
FLT=$(sym g_faulted)
CMD=$(sym g_cmd)

# g_pos is 20 contiguous 32-bit words: commands at 0..36, telemetry at 40..76.
POSA=$(printf "0x%x" $POS)
PENA=$(printf "0x%x" $((POS+0)))     # enabled
PCMD=$(printf "0x%x" $((POS+4)))     # cmd_deg_x10
PKP=$(printf  "0x%x" $((POS+8)))
PKI=$(printf  "0x%x" $((POS+12)))
PKD=$(printf  "0x%x" $((POS+16)))
PVMX=$(printf "0x%x" $((POS+20)))    # vel_max_dps
PACC=$(printf "0x%x" $((POS+24)))    # accel_max_dps2
PJRK=$(printf "0x%x" $((POS+28)))    # jerk_max_dps3
PIMX=$(printf "0x%x" $((POS+32)))    # iq_max_ma
PZER=$(printf "0x%x" $((POS+36)))    # zero_here
PMODE=$(printf "0x%x" $((POS+92)))   # mode: MOTION_MODE_POSITION == 3

FENA=$(printf "0x%x" $((FOC+112)))
FISR=$(printf "0x%x" $((FOC+108)))
IDA=$(printf "0x%x" $((FOC+0)))
IQA=$(printf "0x%x" $((FOC+4)))
CSA=$(printf "0x%x" $CS)
FLTA=$(printf "0x%x" $FLT)
CMDA=$(printf "0x%x" $CMD)

CFG="$(mktemp)"
SAFECFG="$(mktemp)"
OCD_PID=""
ARMED=0
CLEANED=0

# Killing OpenOCD does NOT stop the motor - and a position loop is worse than a
# bare torque command here, because it will actively fight anything that moves
# the shaft, indefinitely. So the teardown reattaches with a second short
# OpenOCD pass whose only job is to disengage the servo and open the gates.
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
  # Ctrl-C fires INT and then EXIT; only let the reattach run once.
  [ "$CLEANED" = "1" ] && return
  CLEANED=1
  if [ -n "$OCD_PID" ]; then
    kill "$OCD_PID" 2>/dev/null || true
    wait "$OCD_PID" 2>/dev/null || true
  fi
  if [ "$ARMED" = "1" ]; then
    sleep 1        # let the ST-Link be released before reattaching
    echo "disengaging servo..." >&2
    if openocd -s "${OPENOCD_SCRIPTS:-C:/msys64/mingw64/share/openocd/scripts}" -d0 -f "$SAFECFG" >/dev/null 2>&1; then
      echo "power stage safe." >&2
    else
      echo "!! COULD NOT REATTACH - POWER DOWN THE BOARD MANUALLY !!" >&2
    fi
  fi
  printf '\033[?25h\033[0m\n'
  rm -f "$CFG" "$SAFECFG"
}
trap cleanup EXIT INT TERM

if [ "$DEMO" = "1" ]; then
  ARMED=1
  echo "--demo energises the power stage and moves the motor. Ctrl-C stops it."
  sleep 1
fi

cat > "$CFG" <<EOF
source [find interface/stlink.cfg]
transport select hla_swd
source [find target/stm32g4x.cfg]
reset_config none
adapter speed 4000
init

set ESC "\033"

set SINT {
    0 175 349 523 698 872 1045 1219 1392 1564 1736 1908 2079
    2250 2419 2588 2756 2924 3090 3256 3420 3584 3746 3907 4067 4226
    4384 4540 4695 4848 5000 5150 5299 5446 5592 5736 5878 6018 6157
    6293 6428 6561 6691 6820 6947 7071 7193 7314 7431 7547 7660 7771
    7880 7986 8090 8192 8290 8387 8480 8572 8660 8746 8829 8910 8988
    9063 9135 9205 9272 9336 9397 9455 9511 9563 9613 9659 9703 9744
    9781 9816 9848 9877 9903 9925 9945 9962 9976 9986 9994 9998 10000
}
proc isin {deg} {
    global SINT
    set d [expr {((\$deg % 360) + 360) % 360}]
    if {\$d <= 90}  { return [lindex \$SINT \$d] }
    if {\$d <= 180} { return [lindex \$SINT [expr {180 - \$d}]] }
    if {\$d <= 270} { return [expr {-[lindex \$SINT [expr {\$d - 180}]]}] }
    return [expr {-[lindex \$SINT [expr {360 - \$d}]]}]
}
proc icos {deg} { return [isin [expr {\$deg + 90}]] }
proc rdiv {n d} {
    if {\$n >= 0} { return [expr {(\$n + \$d/2) / \$d}] }
    return [expr {-((-\$n + \$d/2) / \$d)}]
}
proc s32 {v} { if {\$v > 0x7FFFFFFF} { return [expr {\$v - 0x100000000}] } ; return \$v }
# mww takes an unsigned word, so a signed command goes out as two's complement.
proc mwi {addr v} { mww \$addr [expr {\$v & 0xFFFFFFFF}] }

# ---- small canvas helpers -------------------------------------------------
proc blank {w h} {
    set rows {}
    for {set y 0} {\$y < \$h} {incr y} {
        set row {}
        for {set x 0} {\$x < \$w} {incr x} { lappend row " " }
        lappend rows \$row
    }
    return \$rows
}
proc plot {gv w h x y ch} {
    upvar \$gv g
    if {\$x < 0 || \$x >= \$w || \$y < 0 || \$y >= \$h} { return }
    set row [lindex \$g \$y]
    lset row \$x \$ch
    lset g \$y \$row
}
proc puttext {gv w h x y s} {
    upvar \$gv g
    set n [string length \$s]
    for {set i 0} {\$i < \$n} {incr i} {
        plot g \$w \$h [expr {\$x + \$i}] \$y [string index \$s \$i]
    }
}
# One radial point on an ellipse, at percent-of-radius k.
# k is per-mille of full radius (1000 = on the rim). isin/icos are scaled by
# 10000, so the divisor is 1e4 * 1e3.
proc rx_at {cx rx deg k} { return [expr {\$cx + [rdiv [expr {\$rx * [icos \$deg] * \$k}] 10000000]}] }
proc ry_at {cy ry deg k} { return [expr {\$cy - [rdiv [expr {\$ry * [isin \$deg] * \$k}] 10000000]}] }

proc colorof {ch} {
    switch -- \$ch {
        "." { return "0;90" }
        "+" { return "0;90" }
        "-" { return "0;90" }
        "|" { return "0;90" }
        "=" { return "1;96" }
        "O" { return "1;93" }
        "#" { return "1;92" }
        "*" { return "1;95" }
        "T" { return "1;95" }
        "o" { return "1;93" }
        default { return "0;37" }
    }
}
proc bar {val lo hi width} {
    global ESC
    set span [expr {\$hi - \$lo}]
    if {\$span <= 0} { set span 1 }
    set n [expr {((\$val - \$lo) * \$width) / \$span}]
    if {\$n < 0} { set n 0 }
    if {\$n > \$width} { set n \$width }
    set mid [expr {\$width / 2}]
    set out ""
    for {set i 0} {\$i < \$width} {incr i} {
        if {\$i == \$mid} {
            if {\$i < \$n} { append out "\${ESC}\[1;92m#\${ESC}\[0m" } else { append out "\${ESC}\[90m|\${ESC}\[0m" }
        } elseif {\$i < \$n} {
            append out "\${ESC}\[1;92m#\${ESC}\[0m"
        } else {
            append out "\${ESC}\[90m.\${ESC}\[0m"
        }
    }
    return \$out
}

set TW 74
set TH 15
set TRW 41
set hist {}
set vhist {}
set thist {}
set DEMO $DEMO

# Move schedule for --demo, as {frames_to_hold  target_deg_x10  vel_max_dps
# label}. Everything is relative to the zero taken at start-up, so any rotor
# angle is a safe place to begin from.
#
# Twenty moves, arranged so each group isolates one variable:
#   fine       2/5/10 deg, down where cogging alone could hold the rotor
#   S vs trap  the SAME +/-90 move with jerk on, then off - compare the
#              velocity trace shapes back to back
#   gentle/snappy   same move, only the acceleration limit changes
#   dither     rapid +/-45 reversals, which is where a sloppy loop rings
#   fast/crawl 2 turns at 720 deg/s against 1 turn at 90 deg/s
#   push it    a plain hold - grab the shaft and feel it resist
set demo_seq {
    18      0 360 3600 36000 "hold zero"
    16     20 360 3600 36000 "+2 deg fine"
    16     50 360 3600 36000 "+5 deg fine"
    16    100 360 3600 36000 "+10 deg fine"
    16      0 360 3600 36000 "home"
    24    900 360 3600 36000 "+90 S-curve"
    24      0 360 3600     0 "home trapezoid"
    24    900 360 3600     0 "+90 trapezoid"
    24      0 360 3600 36000 "home S-curve"
    28   1800 360 1200 12000 "+180 gentle"
    28      0 360 9000 90000 "home snappy"
    14    450 480 6000 60000 "+45 dither"
    14      0 480 6000 60000 "-45 dither"
    14    450 480 6000 60000 "+45 dither"
    14      0 480 6000 60000 "-45 dither"
    40   7200 720 7200 72000 "+2 turns fast"
    40      0 720 7200 72000 "home fast"
    78   3600   90 1200 12000 "+1 turn crawl"
    46      0 360 3600 36000 "home"
    26      0 360 3600 36000 "holding - push it"
}
set demo_i 0
set demo_left 0
set demo_lbl ""
set demo_tgt 0
set demo_n 0
set demo_total [expr {[llength \$demo_seq] / 6}]
set mv_frames 0
set mv_peak 0
set mv_lastbad -1
set scores {}

# Bring the stage up in the order main.c enforces - duty, then outputs, then
# gates - hand the current loop to FOC, and only then engage the servo. The
# servo engages holding wherever the rotor already is, so nothing jumps; the
# zero_here that follows makes that position the origin for the demo moves.
if {$DEMO} {
    mww [expr {$CMDA+4}] 0
    mww [expr {$CMDA+8}] 0
    mww [expr {$CMDA+12}] 0
    mww [expr {$CMDA+16}] 1
    mww $CMDA 1
    sleep 200
    mww [expr {$CMDA+20}] 1
    mww $CMDA 1
    sleep 200
    # Clear a latched fault through the state machine, not by poking
    # g_faulted. That word is now a derived mirror written only by
    # Drive_Enter, so writing 0 to it does nothing at all - the drive stays in
    # FAULT and refuses to arm. g_cmd.clear_fault is the one real entry, and
    # it re-runs the self-test on the way out. See drive.h.
    mww [expr {$CMDA+36}] 1
    mww $CMDA 1
    sleep 300
    mww $IDA 0
    mww $IQA 0
    mww $FENA 1
    sleep 200
    # Gains are set explicitly rather than inherited, so the demo shows the
    # same behaviour regardless of what the flashed defaults happen to be.
    mwi $PKP 12000
    mwi $PKI 30000
    mwi $PKD 120
    mwi $PIMX 1000
    # Select position control EXPLICITLY rather than inheriting whatever the
    # last tool left behind. This script predates the mode selector, and
    # motor_ctl.py and dial.py both leave g_pos.mode at IDLE on the way out -
    # so the demo would bring the bridge up, enable the servo, and then run all
    # twenty moves commanding zero current. It only appeared to work straight
    # after a reset, where the mode defaults to position.
    mwi $PMODE 3
    mww $PENA 1
    sleep 100
    mww $PZER 1
    sleep 200
}

puts -nonewline "\${ESC}\[?25l\${ESC}\[2J"

while {1} {
    set p   [read_memory $POSA 32 20]
    set fen [read_memory $FENA 32 1]
    set isr [read_memory $FISR 32 1]
    set flt [read_memory $FLTA 32 1]
    set cs  [read_memory $CSA 32 14]

    set pen    [lindex \$p 0]
    set cmddx  [s32 [lindex \$p 1]]
    set vmax   [s32 [lindex \$p 5]]
    set amax   [s32 [lindex \$p 6]]
    set jmax   [s32 [lindex \$p 7]]
    set iqmax  [s32 [lindex \$p 8]]
    set posdx  [s32 [lindex \$p 10]]
    set tgtdx  [s32 [lindex \$p 11]]
    set errdx  [s32 [lindex \$p 12]]
    set veldps [s32 [lindex \$p 13]]
    set tveld  [s32 [lindex \$p 14]]
    set taccd  [s32 [lindex \$p 15]]
    set iqma   [s32 [lindex \$p 16]]
    set integ  [s32 [lindex \$p 17]]
    set turns  [s32 [lindex \$p 18]]
    set vbus   [lindex \$cs 13]
    if {\$iqmax <= 0} { set iqmax 1 }

    lappend hist \$errdx
    if {[llength \$hist] > \$TRW} { set hist [lrange \$hist end-[expr {\$TRW-1}] end] }
    lappend vhist \$veldps
    lappend thist \$tveld
    if {[llength \$vhist] > \$TRW} { set vhist [lrange \$vhist end-[expr {\$TRW-1}] end] }
    if {[llength \$thist] > \$TRW} { set thist [lrange \$thist end-[expr {\$TRW-1}] end] }

    # ---- demo sequencer ---------------------------------------------------
    # Each move is scored while it runs: peak absolute error, and the last
    # frame at which the error was still outside 0.5 deg, which is the settle
    # time. Scores are banked as the move ends so three stay on screen.
    if {\$DEMO} {
        if {\$demo_left <= 0} {
            if {\$demo_n > 0} {
                lappend scores [format "%s %.2fs pk%.1f" \$demo_lbl \\
                    [expr {(\$mv_lastbad + 1) * $INTERVAL / 1000.0}] \\
                    [expr {\$mv_peak / 10.0}]]
                if {[llength \$scores] > 3} { set scores [lrange \$scores end-2 end] }
            }
            set demo_left [lindex \$demo_seq \$demo_i]
            set demo_tgt  [lindex \$demo_seq [expr {\$demo_i+1}]]
            mwi $PVMX [lindex \$demo_seq [expr {\$demo_i+2}]]
            mwi $PACC [lindex \$demo_seq [expr {\$demo_i+3}]]
            mwi $PJRK [lindex \$demo_seq [expr {\$demo_i+4}]]
            mwi $PCMD \$demo_tgt
            set demo_lbl [lindex \$demo_seq [expr {\$demo_i+5}]]
            set demo_i [expr {(\$demo_i + 6) % [llength \$demo_seq]}]
            incr demo_n
            if {\$demo_n > \$demo_total} { set demo_n 1 }
            set mv_frames 0 ; set mv_peak 0 ; set mv_lastbad -1
        }
        if {abs(\$errdx) > \$mv_peak} { set mv_peak [expr {abs(\$errdx)}] }
        if {abs(\$errdx) > 5} { set mv_lastbad \$mv_frames }
        incr mv_frames
        incr demo_left -1
    }

    # A jerk limit of 0 makes the profile a plain trapezoid; the demo
    # switches between the two mid-run, so say which is active.
    if {\$jmax > 0} { set pmode "S-curve" } else { set pmode "trapez." }
    # ---- header -----------------------------------------------------------
    set out "\${ESC}\[H"
    if {\$DEMO} {
        append out [format "  \${ESC}\[1;97mPOSITION  Isurus\${ESC}\[0m   \${ESC}\[1;95mDEMO %2d/%-2d %-18s\${ESC}\[0m\${ESC}\[90mctrl-c to quit\${ESC}\[0m\r\n" \\
                    \$demo_n \$demo_total \$demo_lbl]
    } else {
        append out "  \${ESC}\[1;97mPOSITION  Isurus\${ESC}\[0m                                \${ESC}\[90mctrl-c to quit\${ESC}\[0m\r\n"
    }
    if {\$flt} {
        append out "  \${ESC}\[1;101;97m OVERCURRENT TRIP \${ESC}\[0m\r\n"
    } elseif {\$pen && \$fen} {
        append out [format "  \${ESC}\[1;92m* servo on\${ESC}\[0m  cmd \${ESC}\[1;96m%+8.1f\${ESC}\[0m deg  %4d d/s %5d d/s2 %s  %d.%02d A  %5.2f V  isr %4.1f us\r\n" \\
                    [expr {\$cmddx/10.0}] \$vmax \$amax \$pmode [expr {\$iqmax/1000}] [expr {(\$iqmax%1000)/10}] \\
                    [expr {\$vbus/1000.0}] [expr {\$isr/128.0}]]
    } elseif {\$fen} {
        append out [format "  \${ESC}\[93m- servo off\${ESC}\[0m  (FOC torque mode)                     vbus %5.2f V\r\n" [expr {\$vbus/1000.0}]]
    } else {
        append out [format "  \${ESC}\[90m- drive disabled\${ESC}\[0m                                  vbus %5.2f V\r\n" [expr {\$vbus/1000.0}]]
    }
    append out "\r\n"

    # ---- dial + error trace, side by side ---------------------------------
    set g [blank \$TW \$TH]
    set cx 15 ; set cy 6 ; set rx 13 ; set ry 5
    set mdeg [expr {((\$posdx / 10) % 360 + 360) % 360}]
    set tdeg [expr {((\$tgtdx / 10) % 360 + 360) % 360}]

    # Draw the dial CLOCKWISE-positive, so the needle turns the way the
    # shaft does. isin/icos are the usual maths convention and ry_at
    # negates screen y, so an increasing angle plots anticlockwise - and
    # this encoder counts UP as the shaft turns clockwise. Rendered
    # directly, a clockwise shaft drove the needle backwards. Negating
    # the plot angle is display-only: position, velocity and every number
    # below keep the encoder's own sign, which is the sign the control
    # loop is built around.
    #
    # The rim dots and the quadrant ticks are symmetric under negation
    # ({0,90,180,270} maps onto itself), so only the needles change.
    set mdraw [expr {-\$mdeg}]
    set tdraw [expr {-\$tdeg}]

    # rim, with a heavier tick at each quadrant so the dial has a reference
    for {set t 0} {\$t < 360} {incr t 5} {
        plot g \$TW \$TH [rx_at \$cx \$rx \$t 1000] [ry_at \$cy \$ry \$t 1000] "."
    }
    foreach t {0 90 180 270} {
        plot g \$TW \$TH [rx_at \$cx \$rx \$t 1000] [ry_at \$cy \$ry \$t 1000] "+"
    }

    # Target: an OUTER tick sitting on the rim, radius 86%..100%. Kept clear of
    # the rotor needle's radii so the two can never overwrite each other.
    for {set k 860} {\$k <= 1000} {incr k 70} {
        plot g \$TW \$TH [rx_at \$cx \$rx \$tdraw \$k] [ry_at \$cy \$ry \$tdraw \$k] "*"
    }
    plot g \$TW \$TH [rx_at \$cx \$rx \$tdraw 1000] [ry_at \$cy \$ry \$tdraw 1000] "T"

    # Rotor: the INNER needle, radius 15%..66%, with an O at 74%.
    for {set k 150} {\$k <= 660} {incr k 60} {
        plot g \$TW \$TH [rx_at \$cx \$rx \$mdraw \$k] [ry_at \$cy \$ry \$mdraw \$k] "="
    }
    plot g \$TW \$TH \$cx \$cy "+"
    plot g \$TW \$TH [rx_at \$cx \$rx \$mdraw 740] [ry_at \$cy \$ry \$mdraw 740] "O"

    puttext g \$TW \$TH 2 0 "mech angle  CW+"
    puttext g \$TW \$TH 2 13 "O rotor   T target"

    # Error trace: newest column at the right, zero on the centre line. The
    # vertical scale is the largest error still in the window, so a settled
    # loop zooms in on its own residual instead of showing a flat line.
    set ex 32 ; set ecy 4
    set escale 50
    foreach v \$hist { if {abs(\$v) > \$escale} { set escale [expr {abs(\$v)}] } }
    for {set x 0} {\$x < \$TRW} {incr x} { plot g \$TW \$TH [expr {\$ex + \$x}] \$ecy "-" }
    set n [llength \$hist]
    for {set i 0} {\$i < \$n} {incr i} {
        set v [lindex \$hist \$i]
        plot g \$TW \$TH [expr {\$ex + \$i}] [expr {\$ecy - [rdiv [expr {\$v * 3}] \$escale]}] "="
    }
    puttext g \$TW \$TH 32 0 [format "error trace   full scale +/-%.1f deg" [expr {\$escale/10.0}]]

    # Velocity trace. Two series share one axis: the profile the firmware
    # generated (=) and what the rotor actually did (o). This is where the
    # S-curve reads as a shape rather than a number - the profile should ease
    # up, hold flat, and ease back down to zero, with the rotor tracking it
    # instead of being yanked into motion and then caught at the far end.
    set vcy 11
    set vscale 60
    foreach v \$vhist { if {abs(\$v) > \$vscale} { set vscale [expr {abs(\$v)}] } }
    foreach v \$thist { if {abs(\$v) > \$vscale} { set vscale [expr {abs(\$v)}] } }
    for {set x 0} {\$x < \$TRW} {incr x} { plot g \$TW \$TH [expr {\$ex + \$x}] \$vcy "-" }
    set n [llength \$thist]
    for {set i 0} {\$i < \$n} {incr i} {
        plot g \$TW \$TH [expr {\$ex + \$i}] [expr {\$vcy - [rdiv [expr {[lindex \$thist \$i] * 3}] \$vscale]}] "="
    }
    set n [llength \$vhist]
    for {set i 0} {\$i < \$n} {incr i} {
        plot g \$TW \$TH [expr {\$ex + \$i}] [expr {\$vcy - [rdiv [expr {[lindex \$vhist \$i] * 3}] \$vscale]}] "o"
    }
    puttext g \$TW \$TH 32 7 [format "velocity   = profile   o rotor   +/-%d d/s" \$vscale]

    # Multi-turn ruler. The dial wraps every revolution, so on a two-turn move
    # it says nothing useful; this spans +/-2.5 turns and shows absolute travel.
    set ruler_lo -9000 ; set ruler_hi 9000 ; set rw \$TRW
    puttext g \$TW \$TH 32 13 "-2.5 turn |         0         | +2.5 turn"
    for {set x 0} {\$x < \$rw} {incr x} { plot g \$TW \$TH [expr {\$ex + \$x}] 14 "-" }
    plot g \$TW \$TH [expr {\$ex + \$rw/2}] 14 "|"
    set rt [expr {\$ex + ((\$tgtdx - \$ruler_lo) * (\$rw-1)) / (\$ruler_hi - \$ruler_lo)}]
    set rp [expr {\$ex + ((\$posdx - \$ruler_lo) * (\$rw-1)) / (\$ruler_hi - \$ruler_lo)}]
    plot g \$TW \$TH \$rt 14 "T"
    plot g \$TW \$TH \$rp 14 "O"

    for {set y 0} {\$y < \$TH} {incr y} {
        set row [lindex \$g \$y]
        set cur ""
        set line "  "
        for {set x 0} {\$x < \$TW} {incr x} {
            set ch [lindex \$row \$x]
            if {\$ch eq " "} { append line " " ; continue }
            set c [colorof \$ch]
            if {\$c ne \$cur} { append line "\${ESC}\[\${c}m" ; set cur \$c }
            append line \$ch
        }
        append out \$line "\${ESC}\[0m\r\n"
    }

    # ---- numbers ----------------------------------------------------------
    append out "\r\n"
    append out [format "   \${ESC}\[90mpos\${ESC}\[0m %+9.1f deg   \${ESC}\[90mturn\${ESC}\[0m %+3d      \${ESC}\[90mtgt\${ESC}\[0m %+9.1f deg\r\n" \\
                [expr {\$posdx/10.0}] \$turns [expr {\$tgtdx/10.0}]]
    append out [format "   \${ESC}\[90merr\${ESC}\[0m %+9.1f deg   %s\r\n" [expr {\$errdx/10.0}] [bar \$errdx [expr {-\$escale}] \$escale 32]]
    append out [format "   \${ESC}\[90mvel\${ESC}\[0m %+9d d/s   %s\r\n" \$veldps [bar \$veldps [expr {-\$vmax}] \$vmax 32]]
    append out [format "   \${ESC}\[90mprf\${ESC}\[0m %+9d d/s   %s  \${ESC}\[90macc\${ESC}\[0m %+7d d/s2\r\n" \$tveld [bar \$tveld [expr {-\$vmax}] \$vmax 32] \$taccd]
    append out [format "   \${ESC}\[90miq \${ESC}\[0m %+9d mA    %s\r\n" \$iqma [bar \$iqma [expr {-\$iqmax}] \$iqmax 32]]
    append out [format "   \${ESC}\[90mint\${ESC}\[0m %+9d mA\r\n" \$integ]
    if {\$DEMO} {
        append out [format "\r\n   \${ESC}\[90mthis move\${ESC}\[0m  pk %.1f deg   \${ESC}\[90mdone\${ESC}\[0m  %s\r\n" \\
                    [expr {\$mv_peak/10.0}] [join \$scores "   "]]
    }
    append out "\${ESC}\[J"
    puts -nonewline \$out
    flush stdout

    sleep $INTERVAL
}
EOF

openocd -s "${OPENOCD_SCRIPTS:-C:/msys64/mingw64/share/openocd/scripts}" -d0 -f "$CFG" &
OCD_PID=$!
wait "$OCD_PID"
