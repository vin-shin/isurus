#!/usr/bin/env bash
# Live encoder angle readout over SWD.
#
# The bare ST-Link/V2 on this bench has no virtual COM port, so the UART
# printout on PB6 is unreachable without a USB-serial adapter. This reads the
# telemetry struct straight out of RAM over SWD instead, while the target runs
# free — no halting, no breakpoints, nothing to perturb the loop.
#
# g_enc is one contiguous block of 32-bit words, so a whole sample is a single
# SWD transaction. Its address is resolved from the ELF at run time, so this
# keeps working after the firmware layout changes.
#
# Usage:  tools/watch_encoder.sh [samples] [interval_ms]
#   e.g.  tools/watch_encoder.sh 200 20     # 200 samples at 50 Hz

set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ELF="$ROOT/build/Debug/makolongfin2.elf"
SAMPLES="${1:-100}"
INTERVAL="${2:-20}"

NM="${NM:-arm-none-eabi-nm}"
command -v "$NM" >/dev/null 2>&1 || \
  NM="/c/Program Files (x86)/GNU Arm Embedded Toolchain/10 2021.10/bin/arm-none-eabi-nm"

[ -f "$ELF" ] || { echo "No ELF at $ELF - run 'ninja -C build/Debug' first." >&2; exit 1; }

ADDR="$("$NM" "$ELF" | awk '$3 == "g_enc" { print "0x" $1 }')"
[ -n "$ADDR" ] || { echo "Symbol 'g_enc' not found in $ELF - rebuild?" >&2; exit 1; }

CFG="$(mktemp)"
trap 'rm -f "$CFG"' EXIT

cat > "$CFG" <<EOF
source [find interface/stlink.cfg]
transport select hla_swd
source [find target/stm32g4x.cfg]
reset_config none
adapter speed 4000
init
echo "   deg      raw     rx1     rx2      reads  err     Hz"
for {set i 0} {\$i < $SAMPLES} {incr i} {
    set d [read_memory $ADDR 32 6]
    set raw    [lindex \$d 0]
    set degx   [lindex \$d 1]
    set reads  [lindex \$d 2]
    set errs   [lindex \$d 3]
    set rate   [lindex \$d 4]
    set frames [lindex \$d 5]
    echo [format "%7.2f  %7d  0x%04X  0x%04X  %9d  %3d  %5d" \\
          [expr {\$degx/100.0}] \$raw \\
          [expr {\$frames & 0xFFFF}] [expr {(\$frames >> 16) & 0xFFFF}] \\
          \$reads \$errs \$rate]
    sleep $INTERVAL
}
exit
EOF

exec openocd -s "${OPENOCD_SCRIPTS:-C:/msys64/mingw64/share/openocd/scripts}" \
     -f "$CFG" 2>&1 | grep -vE "^Info|^Open On|^Licensed|^For bug|^\s*http|^Warn"
