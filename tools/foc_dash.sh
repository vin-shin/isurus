#!/usr/bin/env bash
# Live FOC dashboard. Runs until Ctrl-C.
#
# Three views of the same control loop:
#   - the electrical angle as a rotating needle (what Park/inverse-Park see)
#   - the current vector in the ROTOR frame, which is the whole point of FOC:
#     a well-tuned loop parks it on the +q axis with id at zero
#   - scrolling id/iq traces against their setpoints
#
# All rendering happens inside OpenOCD's Tcl; piping would block-buffer and
# destroy the redraw. jimtcl here has no math functions, so the trig is an
# integer sine table with quadrant symmetry.
#
# Usage:  tools/foc_dash.sh [interval_ms]          just watch
#         tools/foc_dash.sh --demo [interval_ms]   drive a torque sequence
#
# --demo cycles iq_ref through 0 -> +0.5 -> +1.0 -> 0 -> -0.5 -> -1.0 -> 0 and
# repeats, so the motor spins up, stops and reverses while you watch the dq
# vector swing between +q and -q. It drives and renders from one process
# because only one thing can hold the ST-Link at a time.

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

FOC=$(sym g_foc)
CS=$(sym g_cs)
ENC=$(sym g_enc)
FLT=$(sym g_faulted)

MIR=$(printf "0x%x" $((FOC+116)))    # 14 int32 mirrors
ENA=$(printf "0x%x" $((FOC+112)))
ISR=$(printf "0x%x" $((FOC+108)))
CSA=$(printf "0x%x" $CS)
ENCA=$(printf "0x%x" $ENC)
FLTA=$(printf "0x%x" $FLT)
IQA=$(printf "0x%x" $((FOC+4)))
IDA=$(printf "0x%x" $((FOC+0)))
CMDA=$(printf "0x%x" $(sym g_cmd))

# The outer motion loop owns g_foc.iq_ref whenever it is enabled - it rewrites
# it on every control tick. This dashboard drives iq_ref directly, so the two
# cannot both run: without standing the position loop down first, every value
# written here is overwritten within 33 us and the demo does nothing at all.
POSB=$(sym g_pos)
PENA=$(printf "0x%x" $((POSB+0)))
PMODE=$(printf "0x%x" $((POSB+92)))

CFG="$(mktemp)"
SAFECFG="$(mktemp)"
OCD_PID=""
ARMED=0          # 1 once --demo has energised the power stage
CLEANED=0        # cleanup is trapped on INT *and* EXIT; only let it run once

# Killing OpenOCD does NOT stop the motor - the firmware keeps running whatever
# was last commanded. So once the render process is gone we reattach with a
# second short OpenOCD pass whose only job is to put the power stage back in a
# safe state. Without this, Ctrl-C during a demo leaves the motor spinning with
# no way to stop it short of a power cycle.
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
mww [expr {$CMDA+20}] 0
mww [expr {$CMDA+16}] 0
mww $CMDA 1
shutdown
EOF

cleanup() {
  # Ctrl-C fires INT and then EXIT. Re-running the reattach would find the
  # stage already safe, fail to take the ST-Link, and print a false alarm.
  [ "$CLEANED" = "1" ] && return
  CLEANED=1
  if [ -n "$OCD_PID" ]; then
    kill "$OCD_PID" 2>/dev/null || true
    wait "$OCD_PID" 2>/dev/null || true
  fi
  if [ "$ARMED" = "1" ]; then
    sleep 1        # let the ST-Link be released before reattaching
    echo "stopping motor..." >&2
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
  echo "--demo energises the power stage and spins the motor. Ctrl-C stops it."
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
proc colorof {ch} {
    switch -- \$ch {
        "." { return "0;90" }
        "+" { return "0;90" }
        "=" { return "1;96" }
        "O" { return "1;93" }
        "#" { return "1;92" }
        "*" { return "1;95" }
        default { return "0;37" }
    }
}

# ---- bar meter ------------------------------------------------------------
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

set TW 46
set TH 13
set hist_id {}
set hist_iq {}
set frame 0
set DEMO $DEMO

# iq_ref schedule for --demo, as {frames_to_hold  float_bits  label}.
set demo_seq {
    25 0x00000000 "settle"
    45 0x3F000000 "+0.5 A"
    45 0x3F800000 "+1.0 A"
    25 0x00000000 "coast"
    45 0xBF000000 "-0.5 A  reverse"
    45 0xBF800000 "-1.0 A  reverse"
    25 0x00000000 "coast"
}
set demo_i 0
set demo_left 0
set demo_lbl ""

# --demo only sets iq_ref, which does nothing on a disabled bridge. Bring the
# stage up in the order main.c enforces - duty, then outputs, then gates - then
# clear any latched fault and hand control to FOC.
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
    mww $FLTA 0
    # Stand the outer loop down first, so the iq_ref written below is
    # ours and stays ours.
    mww $PMODE 0
    mww $PENA 0
    sleep 100
    mww $IDA 0
    mww $IQA 0
    mww $ENA 1
    sleep 200
}

puts -nonewline "\${ESC}\[?25l\${ESC}\[2J"

while {1} {
    set m   [read_memory $MIR 32 14]
    set ena [read_memory $ENA 32 1]
    set isr [read_memory $ISR 32 1]
    set flt [read_memory $FLTA 32 1]
    set cs  [read_memory $CSA 32 14]
    set mech [expr {[read_memory [expr {$ENCA + 4}] 32 1] / 100.0}]

    set id     [s32 [lindex \$m 0]]
    set iq     [s32 [lindex \$m 1]]
    set duty_u [s32 [lindex \$m 6]]
    set duty_v [s32 [lindex \$m 7]]
    set duty_w [s32 [lindex \$m 8]]
    set iqref  [s32 [lindex \$m 9]]
    set idref  [s32 [lindex \$m 10]]
    set edeg   [expr {[s32 [lindex \$m 11]] / 10}]
    set vmaxpm [s32 [lindex \$m 12]]
    set vmagpm [s32 [lindex \$m 13]]
    set vbus   [lindex \$cs 13]

    lappend hist_id \$id
    lappend hist_iq \$iq
    if {[llength \$hist_id] > 44} { set hist_id [lrange \$hist_id end-43 end] }
    if {[llength \$hist_iq] > 44} { set hist_iq [lrange \$hist_iq end-43 end] }

    # ---- demo sequencer ---------------------------------------------------
    if {\$DEMO} {
        if {\$demo_left <= 0} {
            set demo_left [lindex \$demo_seq \$demo_i]
            write_memory $IQA 32 [list [lindex \$demo_seq [expr {\$demo_i+1}]]]
            set demo_lbl [lindex \$demo_seq [expr {\$demo_i+2}]]
            set demo_i [expr {(\$demo_i + 3) % [llength \$demo_seq]}]
        }
        incr demo_left -1
    }

    set out "\${ESC}\[H"
    if {\$DEMO} {
        append out [format "  \${ESC}\[1;97mFOC  makolongfin2\${ESC}\[0m  \${ESC}\[1;95mDEMO %-16s\${ESC}\[0m\${ESC}\[90mctrl-c to quit\${ESC}\[0m
" \$demo_lbl]
    } else {
    append out "  \${ESC}\[1;97mFOC  makolongfin2\${ESC}\[0m    \${ESC}\[90mctrl-c to quit\${ESC}\[0m\r\n"
    }
    if {\$flt} {
        append out "  \${ESC}\[1;101;97m OVERCURRENT TRIP \${ESC}\[0m\r\n"
    } elseif {\$ena} {
        append out [format "  \${ESC}\[1;92m* closed loop\${ESC}\[0m   iq_ref \${ESC}\[1;96m%+6d\${ESC}\[0m mA   vbus %5.2f V   isr %4.1f us\r\n" \\
                    \$iqref [expr {\$vbus/1000.0}] [expr {\$isr/128.0}]]
    } else {
        append out [format "  \${ESC}\[90m- FOC disabled\${ESC}\[0m                        vbus %5.2f V\r\n" [expr {\$vbus/1000.0}]]
    }
    append out "\r\n"

    # ---- electrical angle dial + dq vector, side by side ------------------
    set g [blank \$TW \$TH]
    set cx 11 ; set cy 6 ; set rx 9 ; set ry 5
    for {set t 0} {\$t < 360} {incr t 6} {
        plot g \$TW \$TH [expr {\$cx + [rdiv [expr {\$rx * [icos \$t]}] 10000]}] \\
                        [expr {\$cy - [rdiv [expr {\$ry * [isin \$t]}] 10000]}] "."
    }
    for {set k 20} {\$k <= 90} {incr k 10} {
        plot g \$TW \$TH [expr {\$cx + [rdiv [expr {\$rx * [icos \$edeg] * \$k}] 1000000]}] \\
                        [expr {\$cy - [rdiv [expr {\$ry * [isin \$edeg] * \$k}] 1000000]}] "="
    }
    plot g \$TW \$TH \$cx \$cy "+"
    plot g \$TW \$TH [expr {\$cx + [rdiv [expr {\$rx * [icos \$edeg]}] 10000]}] \\
                    [expr {\$cy - [rdiv [expr {\$ry * [isin \$edeg]}] 10000]}] "O"
    puttext g \$TW \$TH 2 0 "elec angle"

    # dq plane: id horizontal, iq vertical. A healthy loop sits on +q.
    set dx 34 ; set dy 6 ; set drx 9 ; set dry 5
    set scale 1500
    foreach v [list \$id \$iq \$iqref] { if {abs(\$v) > \$scale} { set scale [expr {abs(\$v)}] } }
    for {set x -9} {\$x <= 9} {incr x} { plot g \$TW \$TH [expr {\$dx + \$x}] \$dy "." }
    for {set y -5} {\$y <= 5} {incr y} { plot g \$TW \$TH \$dx [expr {\$dy + \$y}] "." }
    puttext g \$TW \$TH 29 0 "dq current"
    # setpoint marker
    set sy [expr {\$dy - (\$iqref * \$dry) / \$scale}]
    plot g \$TW \$TH \$dx \$sy "*"
    # actual vector
    set px [expr {\$dx + (\$id * \$drx) / \$scale}]
    set py [expr {\$dy - (\$iq * \$dry) / \$scale}]
    plot g \$TW \$TH \$px \$py "O"

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

    append out "\r\n"
    append out [format "   \${ESC}\[90mid\${ESC}\[0m %+6d mA  %s\r\n" \$id [bar \$id [expr {-\$scale}] \$scale 28]]
    append out [format "   \${ESC}\[90miq\${ESC}\[0m %+6d mA  %s\r\n" \$iq [bar \$iq [expr {-\$scale}] \$scale 28]]
    append out [format "   \${ESC}\[90mV \${ESC}\[0m %5d/%4d   %s\r\n" \$vmagpm \$vmaxpm [bar \$vmagpm 0 \$vmaxpm 28]]
    append out "\r\n"
    append out [format "   \${ESC}\[90mduty\${ESC}\[0m U %4d  V %4d  W %4d      \${ESC}\[90mmech\${ESC}\[0m %6.2f deg   \${ESC}\[90melec\${ESC}\[0m %3d deg\r\n" \\
                \$duty_u \$duty_v \$duty_w \$mech \$edeg]
    append out "\${ESC}\[J"
    puts -nonewline \$out
    flush stdout

    sleep $INTERVAL
}
EOF

openocd -s "${OPENOCD_SCRIPTS:-C:/msys64/mingw64/share/openocd/scripts}" -d0 -f "$CFG" &
OCD_PID=$!
wait "$OCD_PID"
