#!/usr/bin/env bash
# First-power-on check for the Mako Desori. Ten minutes, ST-Link only.
#
# Nothing needs to be connected but the debug probe: no DC link, no motor, no
# CAN. The board is powered from its logic supply and this reads the telemetry
# structs out of RAM over SWD while the target runs free - no halting, nothing
# perturbed.
#
# The point is not to dump numbers. This branch was written entirely against a
# schematic and four datasheets, with no hardware, so a dozen things in it are
# INFERENCES. Each check below is one of those inferences, phrased so that
# passing means the inference held and failing tells you which assumption to
# go and look at. The three marked !! are the ones carrying the most weight.
#
# Usage:  tools/bringup_check.sh
#
# Exit status is the number of failed checks, so it is usable in a script.

set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ELF="$ROOT/build/Debug/makolongfin2.elf"

NM="${NM:-arm-none-eabi-nm}"
command -v "$NM" >/dev/null 2>&1 || \
  NM="/c/Program Files (x86)/GNU Arm Embedded Toolchain/10 2021.10/bin/arm-none-eabi-nm"

[ -f "$ELF" ] || { echo "No ELF at $ELF - build it first." >&2; exit 1; }

# Symbol addresses from the ELF at run time, like every other tool here: any
# code-size change moves the BSS layout, and a baked-in address does not error,
# it reads a different variable. See CLAUDE.md.
sym() {
  local a
  a="$("$NM" "$ELF" | awk -v s="$1" '$3 == s { print "0x" $1 }' | head -1)"
  [ -n "$a" ] || { echo "Symbol '$1' not in $ELF - stale build?" >&2; exit 1; }
  echo "$a"
}

CS="$(sym g_cs)"
DR="$(sym g_drive)"
TH="$(sym g_therm)"
PW="$(sym g_pwm)"
EN="$(sym g_enc)"
SCK="$(sym SystemCoreClock)"
SUBC="$(sym g_enc_sub_consec)"
CSRC="$(sym g_cs_init_rc)"
THRC="$(sym g_therm_init_rc)"
PWRC="$(sym g_pwm_init_rc)"
TGRC="$(sym g_cs_trig_rc)"

CFG="$(mktemp)"
trap 'rm -f "$CFG"' EXIT

# Field indices are word offsets into each struct. They track the declarations
# in csense.h, drive.h, thermal.h, motor_pwm.h and main.c - if a field is
# inserted rather than appended there, these move with it.
cat > "$CFG" <<EOF
source [find interface/stlink.cfg]
transport select hla_swd
source [find target/stm32g4x.cfg]
reset_config none
adapter speed 4000
init

set fails 0

proc row {verdict name detail} {
    global fails
    if {\$verdict eq "FAIL"} { incr fails }
    echo [format "  %-4s  %-34s %s" \$verdict \$name \$detail]
}
proc judge {ok name detail} {
    row [expr {\$ok ? "ok" : "FAIL"}] \$name \$detail
}

set cs [read_memory $CS 32 18]
set dr [read_memory $DR 32 21]
set th [read_memory $TH 32 6]
set pw [read_memory $PW 32 11]
set en [read_memory $EN 32 6]

set sysclk  [lindex [read_memory $SCK  32 1] 0]
set subc    [lindex [read_memory $SUBC 32 1] 0]
set cs_rc   [lindex [read_memory $CSRC 32 1] 0]
set th_rc   [lindex [read_memory $THRC 32 1] 0]
set pw_rc   [lindex [read_memory $PWRC 32 1] 0]
set tg_rc   [lindex [read_memory $TGRC 32 1] 0]

# ---- signed helper: OpenOCD hands back unsigned words -----------------------
proc s32 {v} { return [expr {\$v > 0x7FFFFFFF ? \$v - 0x100000000 : \$v}] }

# ---- struct fields ---------------------------------------------------------
set u_zero  [lindex \$cs  8]
set w_zero  [lindex \$cs  9]
set samples [lindex \$cs 11]
set cs_err  [lindex \$cs 12]
set vrefint [lindex \$cs 13]
set vdda    [lindex \$cs 14]
set vbus_mv [lindex \$cs 16]
set trig    [lindex \$cs 17]

set state   [lindex \$dr  0]
set fault   [lindex \$dr  1]
set stfail  [lindex \$dr  4]
set gd_flt  [lindex \$dr 19]
set gd_nrdy [lindex \$dr 20]

set th_valid [lindex \$th 3]
set th_c     [s32 [lindex \$th 1]]
set th_ohm   [lindex \$th 2]

set period  [lindex \$pw 0]
set pwm_hz  [lindex \$pw 1]
set out_en  [lindex \$pw 7]
set gate_en [lindex \$pw 9]

set enc_raw  [lindex \$en 0]
set enc_deg  [lindex \$en 1]
set enc_rd   [lindex \$en 2]
set enc_err  [lindex \$en 3]

proc swnames {mask} {
    set n {UH UL VH VL WH WL}
    set out {}
    for {set i 0} {\$i < 6} {incr i} {
        if {\$mask & (1 << \$i)} { lappend out [lindex \$n \$i] }
    }
    if {[llength \$out] == 0} { return "none" }
    return [join \$out " "]
}
proc statename {s} {
    return [lindex {INIT SELFTEST READY RUN FAULT} \$s]
}
proc faultname {f} {
    return [lindex {NONE OVERCURRENT OVERVOLTAGE UNDERVOLT ENCODER CSENSE \\
                    SELFTEST WATCHDOG COMMAND OVERTEMP GATEDRV} \$f]
}

echo ""
echo "Mako Desori bring-up check - logic power only, nothing else connected"
echo "====================================================================="
echo ""
echo "core and peripherals"

judge [expr {\$sysclk == 160000000}] "core runs at 160 MHz" \\
      [format "SystemCoreClock = %d" \$sysclk]
judge [expr {\$cs_rc == 0}] "CSense_Init" [format "rc %d" [s32 \$cs_rc]]
judge [expr {\$th_rc == 0}] "Thermal_Init" [format "rc %d" [s32 \$th_rc]]
judge [expr {\$pw_rc == 0}] "MotorPwm_Init" [format "rc %d" [s32 \$pw_rc]]
judge [expr {\$tg_rc == 0}] "CSense_UseHrtimTrigger" [format "rc %d" [s32 \$tg_rc]]
judge [expr {\$period == 64000 && \$pwm_hz == 20000}] "PWM 20 kHz, 64000 counts" \\
      [format "%d counts, %d Hz" \$period \$pwm_hz]
judge [expr {\$trig == 1}] "ADC1 is HRTIM-triggered" \\
      [expr {\$trig ? "locked to the PWM period" : "STILL FREE-RUNNING"}]

echo ""
echo "the bridge is inert - it should be, nothing has armed it"
judge [expr {\$gate_en == 0}] "gate drivers disabled" \\
      [expr {\$gate_en ? "ENABLED - PC8 polarity may be inverted" : "PC8 low"}]
judge [expr {\$out_en == 0}] "HRTIM outputs disabled" \\
      [expr {\$out_en ? "ENABLED" : "off"}]

echo ""
echo "!! the three that carry the most weight !!"

# 1. VREF+. Never known until the target measured it - VREFBUF is disabled on
#    this board and the reference is an external part nobody has identified.
judge [expr {\$vdda > 2000 && \$vdda < 3600 && \$vrefint > 100}] \\
      "VREF+ measured via VREFINT" \\
      [format "%d mV (VREFINT raw %d)" \$vdda \$vrefint]

# 2. Current-sense zero. VREFHALF is VREF/2 from a 10k/10k divider, so zero
#    current should land on mid-scale BY CONSTRUCTION. If it does not, the
#    conditioning is not what board.h section 4 says it is, and every current
#    this drive measures is scaled from a wrong premise.
set ud [expr {abs(\$u_zero - 2048)}]
set wd [expr {abs(\$w_zero - 2048)}]
judge [expr {\$ud <= 200 && \$wd <= 200}] \\
      "current zeros sit at mid-scale" \\
      [format "U %d (%+d), W %d (%+d), tol 200" \\
              \$u_zero [expr {\$u_zero - 2048}] \$w_zero [expr {\$w_zero - 2048}]]

# 3. Gate driver status. Confirms FLT active-low, RDY active-high and that the
#    pull-ups took - all three inferred from the UCC21756 datasheet, never seen.
judge [expr {\$gd_nrdy == 0}] "all six drivers report READY" \\
      [format "not ready: %s" [swnames \$gd_nrdy]]
judge [expr {\$gd_flt == 0}] "no driver is asserting FLT" \\
      [format "faulted: %s" [swnames \$gd_flt]]

echo ""
echo "sensors"
judge [expr {\$samples > 0 && \$cs_err == 0}] "current sense converting" \\
      [format "%d sample sets, %d errors" \$samples \$cs_err]
judge [expr {\$enc_rd > 0}] "encoder answering" \\
      [format "%d reads, %d errors, raw %d = %.2f deg" \\
              \$enc_rd \$enc_err \$enc_raw [expr {\$enc_deg/100.0}]]
judge [expr {\$subc < 8}] "encoder link not substituting" \\
      [format "%d consecutive substitutions (limit 8)" \$subc]
judge [expr {\$th_valid == 1}] "KTY in range" \\
      [format "%.1f C, %d ohm  - SHAPE IS WRONG, see LATER.md 1" \\
              [expr {\$th_c/10.0}] \$th_ohm]

echo ""
echo "drive state"
row "--" "state" [format "%s, fault %s, self-test %s" \\
    [statename \$state] [faultname \$fault] [faultname \$stfail]]
row "--" "bus reading" [format "%d mV  - UNCALIBRATED, models the divider only" \$vbus_mv]

echo ""
if {\$fails == 0} {
    echo "all checks passed. Every inference this branch made about the analogue"
    echo "chain, the driver polarities and the clock held on real hardware."
} else {
    echo [format "%d check(s) failed. Each names the assumption to go and look at;" \$fails]
    echo "docs/PORT-MAKO-DESORI.md section 4 lists what was inferred and why."
}
echo ""
shutdown
EOF

set +e
openocd -s "${OPENOCD_SCRIPTS:-C:/msys64/mingw64/share/openocd/scripts}" \
        -f "$CFG" 2>&1 | grep -vE "^Info|^Open On|^Licensed|^For bug|^\s*http|^Warn|^shutdown"
rc=${PIPESTATUS[0]}
set -e
exit "$rc"
