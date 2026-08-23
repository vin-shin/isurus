#!/usr/bin/env bash
# Live ASCII dial showing the encoder angle. Runs until Ctrl-C.
#
# Reads the g_enc telemetry block out of RAM over SWD while the target runs
# free, and draws a needle on a circle. All rendering happens inside OpenOCD's
# Tcl so nothing is piped — a pipe would block-buffer and destroy the redraw.
#
# Note this build of jimtcl has no math functions, so the trig is an integer
# sine table with quadrant symmetry.
#
# Usage:  tools/encoder_dial.sh [interval_ms]     (default 40 => ~25 fps)

set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ELF="$ROOT/build/Debug/makolongfin2.elf"
INTERVAL="${1:-40}"

NM="${NM:-arm-none-eabi-nm}"
command -v "$NM" >/dev/null 2>&1 || \
  NM="/c/Program Files (x86)/GNU Arm Embedded Toolchain/10 2021.10/bin/arm-none-eabi-nm"

[ -f "$ELF" ] || { echo "No ELF at $ELF - run 'ninja -C build/Debug' first." >&2; exit 1; }

ADDR="$("$NM" "$ELF" | awk '$3 == "g_enc" { print "0x" $1 }')"
[ -n "$ADDR" ] || { echo "Symbol 'g_enc' not found in $ELF - rebuild?" >&2; exit 1; }
CSADDR="$("$NM" "$ELF" | awk '$3 == "g_cs" { print "0x" $1 }')"
[ -n "$CSADDR" ] || { echo "Symbol 'g_cs' not found in $ELF - rebuild?" >&2; exit 1; }
FADDR="$("$NM" "$ELF" | awk '$3 == "g_faulted" { print "0x" $1 }')"
TADDR="$("$NM" "$ELF" | awk '$3 == "g_oc_trips" { print "0x" $1 }')"
PADDR="$("$NM" "$ELF" | awk '$3 == "g_oc_peak" { print "0x" $1 }')"

CFG="$(mktemp)"
# Restore the cursor whatever happens - Ctrl-C included. Deliberately not
# exec'ing openocd, so this trap still runs.
OCD_PID=""
cleanup() {
  # Kill our openocd explicitly. If the wrapper is killed directly (a timeout,
  # or anything that does not signal the whole process group) openocd would
  # otherwise survive and keep holding the ST-Link, which then looks exactly
  # like the wedged-probe failure in HARDWARE_NOTES.md section 2.
  if [ -n "$OCD_PID" ]; then kill "$OCD_PID" 2>/dev/null || true; fi
  printf '\033[?25h\033[0m\n'
  rm -f "$CFG"
}
trap cleanup EXIT INT TERM

cat > "$CFG" <<EOF
source [find interface/stlink.cfg]
transport select hla_swd
source [find target/stm32g4x.cfg]
reset_config none
adapter speed 4000
init

set ESC "\033"

# sin(deg)*10000 for 0..90; other quadrants by symmetry.
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

# Rounded division that behaves for negative numerators (jimtcl / floors).
proc rdiv {n d} {
    if {\$n >= 0} { return [expr {(\$n + \$d/2) / \$d}] }
    return [expr {-((-\$n + \$d/2) / \$d)}]
}

set W 52
set H 21
set CX 25
set CY 10
set RX 20
set RY 9

proc blankgrid {} {
    global W H
    set rows {}
    for {set y 0} {\$y < \$H} {incr y} {
        set row {}
        for {set x 0} {\$x < \$W} {incr x} { lappend row " " }
        lappend rows \$row
    }
    return \$rows
}

proc plot {gridvar x y ch} {
    upvar \$gridvar g
    global W H
    if {\$x < 0 || \$x >= \$W || \$y < 0 || \$y >= \$H} { return }
    set row [lindex \$g \$y]
    lset row \$x \$ch
    lset g \$y \$row
}

proc puttext {gridvar x y s} {
    upvar \$gridvar g
    set n [string length \$s]
    for {set i 0} {\$i < \$n} {incr i} {
        plot g [expr {\$x + \$i}] \$y [string index \$s \$i]
    }
}

# Character -> colour. Grouped into runs at render time.
proc colorof {ch} {
    switch -- \$ch {
        "." { return "0;90" }
        "+" { return "1;37" }
        "=" { return "1;96" }
        "O" { return "1;93" }
        default { return "0;90" }
    }
}

proc render {g} {
    global ESC W H
    set out ""
    for {set y 0} {\$y < \$H} {incr y} {
        set row [lindex \$g \$y]
        set cur ""
        set line ""
        for {set x 0} {\$x < \$W} {incr x} {
            set ch [lindex \$row \$x]
            if {\$ch eq " "} {
                append line " "
                continue
            }
            set c [colorof \$ch]
            if {\$c ne \$cur} {
                append line "\${ESC}\[\${c}m"
                set cur \$c
            }
            append line \$ch
        }
        if {\$cur ne ""} { append line "\${ESC}\[0m" }
        append out \$line "\r\n"
    }
    return \$out
}

# Scrolling current-waveform graph.
set GW 46
set GH 11
set hist_u {}
set hist_w {}

# Hide cursor, clear screen once.
puts -nonewline "\${ESC}\[?25l\${ESC}\[2J"

while {1} {
    set d      [read_memory $ADDR 32 6]
    set raw    [lindex \$d 0]
    set degx   [lindex \$d 1]
    set reads  [lindex \$d 2]
    set errs   [lindex \$d 3]
    set rate   [lindex \$d 4]
    set frames [lindex \$d 5]

    set adeg [expr {\$degx / 100}]
    # Screen angle. The encoder counts one way and the maths convention
    # (0 at east, positive CCW) counts the other, so mirror it - otherwise
    # turning the shaft CW draws CCW.
    set ddeg [expr {(360 - \$adeg) % 360}]

    set g [blankgrid]

    # Rim
    for {set t 0} {\$t < 360} {incr t 2} {
        set x [expr {\$CX + [rdiv [expr {\$RX * [icos \$t]}] 10000]}]
        set y [expr {\$CY - [rdiv [expr {\$RY * [isin \$t]}] 10000]}]
        plot g \$x \$y "."
    }

    # Cardinal labels
    puttext g [expr {\$CX + \$RX + 2}] \$CY "0"
    puttext g [expr {\$CX - 1}] [expr {\$CY - \$RY - 1}] "270"
    puttext g [expr {\$CX - \$RX - 5}] \$CY "180"
    puttext g [expr {\$CX - 1}] [expr {\$CY + \$RY + 1}] "90"

    # Needle
    for {set k 12} {\$k <= 88} {incr k 2} {
        set x [expr {\$CX + [rdiv [expr {\$RX * [icos \$ddeg] * \$k}] 1000000]}]
        set y [expr {\$CY - [rdiv [expr {\$RY * [isin \$ddeg] * \$k}] 1000000]}]
        plot g \$x \$y "="
    }
    plot g \$CX \$CY "+"

    # Tip
    set tx [expr {\$CX + [rdiv [expr {\$RX * [icos \$ddeg]}] 10000]}]
    set ty [expr {\$CY - [rdiv [expr {\$RY * [isin \$ddeg]}] 10000]}]
    plot g \$tx \$ty "O"

    # Frame: home, draw, clear to end of screen.
    set out "\${ESC}\[H"
    append out "\${ESC}\[1;97m  encoder A1333 / SPI1\${ESC}\[0m   \${ESC}\[90mctrl-c to quit\${ESC}\[0m\r\n"
    set flt  [read_memory $FADDR 32 1]
    set trips [read_memory $TADDR 32 1]
    set opk  [read_memory $PADDR 32 1]
    if {\$opk > 0x7FFFFFFF} { set opk [expr {\$opk - 0x100000000}] }
    if {\$flt} {
        append out [format "  \${ESC}\[1;101;97m OVERCURRENT TRIP - DRIVE LATCHED OFF \${ESC}\[0m \${ESC}\[1;91mpeak %d mA, %d trip(s)\${ESC}\[0m\r\n" \$opk \$trips]
    } else {
        append out [format "  \${ESC}\[1;92m* running\${ESC}\[0m   \${ESC}\[90mpeak %d mA, %d trip(s)\${ESC}\[0m\r\n" \$opk \$trips]
    }
    append out "\r\n"
    append out [render \$g]
    append out "\r\n"
    append out [format "   %sangle%s %s%7.2f deg%s    %sraw%s %5d    %srx2%s 0x%04X\r\n" \\
                "\${ESC}\[90m" "\${ESC}\[0m" "\${ESC}\[1;96m" [expr {\$degx/100.0}] "\${ESC}\[0m" \\
                "\${ESC}\[90m" "\${ESC}\[0m" \$raw \\
                "\${ESC}\[90m" "\${ESC}\[0m" [expr {(\$frames >> 16) & 0xFFFF}]]
    append out [format "   %srate%s  %6d Hz    %sreads%s %9d    %serrors%s %d\r\n" \\
                "\${ESC}\[90m" "\${ESC}\[0m" \$rate \\
                "\${ESC}\[90m" "\${ESC}\[0m" \$reads \\
                "\${ESC}\[90m" "\${ESC}\[0m" \$errs]

    # Live phase currents, U and W.
    set cs [read_memory $CSADDR 32 12]
    proc s32 {v} { if {\$v > 0x7FFFFFFF} { return [expr {\$v - 0x100000000}] } ; return \$v }
    set iu [s32 [lindex \$cs 4]]
    set iw [s32 [lindex \$cs 5]]
    append out [format "   \${ESC}\[90mI_U\${ESC}\[0m %s%6d mA\${ESC}\[0m   \${ESC}\[90mI_W\${ESC}\[0m %s%6d mA\${ESC}\[0m   \${ESC}\[90merr\${ESC}\[0m %d
" \
                [expr {abs(\$iu) > 1000 ? "\${ESC}\[1;91m" : "\${ESC}\[1;92m"}] \$iu \
                [expr {abs(\$iw) > 1000 ? "\${ESC}\[1;91m" : "\${ESC}\[1;92m"}] \$iw \
                [lindex \$cs 9]]
    # --- scrolling I_U / I_W waveform ---
    lappend hist_u \$iu
    lappend hist_w \$iw
    if {[llength \$hist_u] > \$GW} { set hist_u [lrange \$hist_u end-[expr {\$GW-1}] end] }
    if {[llength \$hist_w] > \$GW} { set hist_w [lrange \$hist_w end-[expr {\$GW-1}] end] }

    # Autoscale to the largest excursion on screen, rounded up to 500 mA.
    set scale 500
    foreach v [concat \$hist_u \$hist_w] { if {abs(\$v) > \$scale} { set scale [expr {abs(\$v)}] } }
    set scale [expr {((\$scale/500)+1)*500}]

    set gg {}
    for {set r 0} {\$r < \$GH} {incr r} {
        set grow {}
        for {set c 0} {\$c < \$GW} {incr c} { lappend grow " " }
        lappend gg \$grow
    }
    set zr [expr {(\$GH-1)/2}]
    set grow [lindex \$gg \$zr]
    for {set c 0} {\$c < \$GW} {incr c} { lset grow \$c "." }
    lset gg \$zr \$grow

    set n [llength \$hist_u]
    for {set c 0} {\$c < \$n} {incr c} {
        set col [expr {\$GW - \$n + \$c}]
        set v [lindex \$hist_u \$c]
        set r [expr {((\$scale - \$v) * (\$GH-1)) / (2*\$scale)}]
        if {\$r < 0} { set r 0 }
        if {\$r >= \$GH} { set r [expr {\$GH-1}] }
        set grow [lindex \$gg \$r]; lset grow \$col "o"; lset gg \$r \$grow
        set v [lindex \$hist_w \$c]
        set r [expr {((\$scale - \$v) * (\$GH-1)) / (2*\$scale)}]
        if {\$r < 0} { set r 0 }
        if {\$r >= \$GH} { set r [expr {\$GH-1}] }
        set grow [lindex \$gg \$r]
        if {[lindex \$grow \$col] eq "o"} { lset grow \$col "8" } else { lset grow \$col "*" }
        lset gg \$r \$grow
    }

    append out "\r\n"
    for {set r 0} {\$r < \$GH} {incr r} {
        if {\$r == 0} { set lbl [format "%+5d" \$scale] } elseif {\$r == \$zr} { set lbl "    0" } elseif {\$r == \$GH-1} { set lbl [format "%+5d" [expr {-\$scale}]] } else { set lbl "     " }
        append out "  \${ESC}\[90m\$lbl\${ESC}\[0m "
        set grow [lindex \$gg \$r]
        set cur ""
        for {set c 0} {\$c < \$GW} {incr c} {
            set ch [lindex \$grow \$c]
            if {\$ch eq " "} { append out " " ; continue }
            set cc "0;90"
            if {\$ch eq "o"} { set cc "1;96" }
            if {\$ch eq "*"} { set cc "1;95" }
            if {\$ch eq "8"} { set cc "1;93" }
            if {\$cc ne \$cur} { append out "\${ESC}\[\${cc}m" ; set cur \$cc }
            append out \$ch
        }
        append out "\${ESC}\[0m\r\n"
    }
    append out "        \${ESC}\[1;96mo I_U\${ESC}\[0m  \${ESC}\[1;95m* I_W\${ESC}\[0m  \${ESC}\[1;93m8 both\${ESC}\[0m  \${ESC}\[90mmA, autoscaled\${ESC}\[0m\r\n"
    append out "\${ESC}\[J"
    puts -nonewline \$out
    flush stdout

    sleep $INTERVAL
}
EOF

openocd -s "${OPENOCD_SCRIPTS:-C:/msys64/mingw64/share/openocd/scripts}" -d0 -f "$CFG" &
OCD_PID=$!
wait "$OCD_PID"
