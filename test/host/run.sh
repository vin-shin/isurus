#!/usr/bin/env bash
# Build and run the host tests for foc.c.
#
#   test/host/run.sh              build and run
#   test/host/run.sh --mutants    also verify the tests can FAIL
#
# foc.c is compiled here byte-for-byte as it ships on the target. The two
# things it needs from the MCU - the CORDIC and fastmath.h's VSQRT - are
# supplied by shim/, which goes ahead of Core/Inc on the include path. Nothing
# in Core/ is modified or conditionally compiled for the host, because a
# harness that needs the file under test edited is testing a different program.
#
# --mutants is the part that matters most. A passing test suite proves nothing
# until it has been seen to fail, so this plants the two bugs this project
# actually shipped and checks the suite goes red:
#
#   1. the predecessor project's angle conversion, off by exactly pi
#   2. the old anti-windup that scaled the integrators by k
#
# Both of those cost real bench time to find. Both are one command now.

set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
OUT="${TMPDIR:-/tmp}/makolongfin_hosttest"
mkdir -p "$OUT"

CC="${CC:-gcc}"
command -v "$CC" >/dev/null 2>&1 || CC=/c/msys64/mingw64/bin/gcc
command -v "$CC" >/dev/null 2>&1 || {
  echo "No host C compiler. On this machine: pacman -S mingw-w64-x86_64-gcc" >&2
  exit 1
}

SRC=( "$ROOT/test/host/test_foc.c" "$ROOT/test/host/pmsm.c"
      "$ROOT/test/host/cordic_model.c" "$ROOT/Core/Src/ident.c" )
INC=( -I"$ROOT/test/host/shim" -I"$ROOT/Core/Inc" -I"$ROOT/test/host" )

# -Werror here too: the tests are code, and they were already caught once
# indenting two statements under one if.
build() { "$CC" -std=gnu11 -O2 -g -Wall -Wextra -Werror "${INC[@]}" "${SRC[@]}" "$1" -o "$2" -lm; }

build "$ROOT/Core/Src/foc.c" "$OUT/test_foc.exe"
"$OUT/test_foc.exe"
rc=$?

# Also build the CSV dumper, so `tools/viz.py compare` has something to plot
# against a bench capture. Same harness (sim.h) as the assertions above, so a
# plot cannot show behaviour the tests never exercised.
"$CC" -std=gnu11 -O2 -g -Wall -Wextra -Werror "${INC[@]}"       "$ROOT/test/host/sim_dump.c" "$ROOT/test/host/pmsm.c"       "$ROOT/test/host/cordic_model.c" "$ROOT/Core/Src/foc.c"       -o "$OUT/sim_dump.exe" -lm
echo "sim_dump: $OUT/sim_dump.exe"

# ---- drive.c: the state machine and the fault paths ----------------------
#
# Separate binary rather than more tests in test_foc.exe, because the two need
# different stubs: foc.c wants a CORDIC and nothing else, drive.c wants the
# gate driver, both sensors and a clock. Linking one set of stubs into both
# would make each suite depend on scaffolding the other needs.
DSRC=( "$ROOT/test/host/test_drive.c" "$ROOT/test/host/drive_stubs.c"
       "$ROOT/test/host/cordic_model.c" "$ROOT/Core/Src/foc.c" )
dbuild() { "$CC" -std=gnu11 -O2 -g -Wall -Wextra -Werror "${INC[@]}" "${DSRC[@]}" "$1" -o "$2" -lm; }

dbuild "$ROOT/Core/Src/drive.c" "$OUT/test_drive.exe"
"$OUT/test_drive.exe"
drc=$?
[ "$rc" -eq 0 ] || drc=$rc
rc=$drc

if [ "${1:-}" = "--mutants" ]; then
  echo "verifying the tests can fail"
  echo "---------------------------"

  # 1. Angle conversion off by pi - the predecessor project's version.
  sed 's|CORDIC->WDATA = (uint32_t)counts << 17;|CORDIC->WDATA = (uint32_t)(((int32_t)counts - 16384) << 17);|' \
      "$ROOT/Core/Src/foc.c" > "$OUT/mut_pi.c"
  # 2. Anti-windup by scaling instead of back-calculation.
  sed -e 's|f->id_integ -= (vd_cmd - f->vd);|f->id_integ *= k;|' \
      -e 's|f->iq_integ -= (vq_cmd - f->vq);|f->iq_integ *= k;|' \
      "$ROOT/Core/Src/foc.c" > "$OUT/mut_aw.c"

  # ---- drive.c mutants --------------------------------------------------
  #
  # Each is a bug this state machine could plausibly have, and two of them it
  # actually used to: re-arming straight to READY after a clear, and the
  # undervoltage case latching instead of retrying.
  #
  # 3. Clearing a fault goes straight back to READY without re-running the
  #    checks. This is the behaviour drive.h records as removed.
  sed 's|^  Drive_SelfTest();$|  Drive_Enter(DRIVE_READY);|'       "$ROOT/Core/Src/drive.c" > "$OUT/mut_clear.c"
  # 4. The bridge is not safed on the way into FAULT - cause still latches, so
  #    only an assertion about the OUTPUTS can catch this one.
  sed 's|^  Drive_SafeState();$||' "$ROOT/Core/Src/drive.c" > "$OUT/mut_nosafe.c"
  # 5. A dead MISO line reads 0xFFFF; drop the check that notices.
  sed 's|else if (raw == 0xFFFFU)               { bad = DRIVE_FAULT_ENCODER; }||'       "$ROOT/Core/Src/drive.c" > "$OUT/mut_miso.c"
  # 6. Latch on an undervoltage bus instead of retrying, so the drive never
  #    comes up on its own when the supply arrives.
  sed 's|    return;   /\* stay in SELFTEST; Drive_Step retries \*/|    ;|'       "$ROOT/Core/Src/drive.c" > "$OUT/mut_uvlatch.c"

  # 7. Saturation CLEARS the plausibility accumulator instead of holding it,
  #    so an implausible command can hide behind intermittent voltage limiting.
  sed 's|    g_drive.torq_held_ms += dt;|    g_drive.torq_dev_ms = 0U;|' \
      "$ROOT/Core/Src/drive.c" > "$OUT/mut_torqsat.c"
  # 8. Trip on the first millisecond of deviation rather than at the bound.
  sed 's|g_drive.torq_dev_ms >= DRIVE_TORQ_DEV_MS|g_drive.torq_dev_ms >= 1U|' \
      "$ROOT/Core/Src/drive.c" > "$OUT/mut_torqfast.c"
  # 9. Run the monitor outside RUN, where the bridge is down and the delivered
  #    current is zero by construction - every disarmed drive then looks
  #    implausible.
  sed 's|  if (g_drive.state != (uint32_t)DRIVE_RUN)|  if (0)|' \
      "$ROOT/Core/Src/drive.c" > "$OUT/mut_torqrun.c"

  # 8. Drop the winding over-temperature trip in Drive_Step, keeping the
  #    reading and the warning flag. This is the one that matters most of the
  #    set: LIM_IQ_MAX_MA sits deliberately ABOVE the machine's continuous
  #    rating so short bursts are available, so nothing else in the drive
  #    stops a sustained overload. Without this trip the firmware will hold
  #    113 Arms into a 100 Arms motor until something melts, and every other
  #    test still passes.
  sed 's|      if (g_therm.motor_c_x10 > LIM_TEMP_MOTOR_MAX_CX10)|      if (0)|' \
      "$ROOT/Core/Src/drive.c" > "$OUT/mut_hotrun.c"
  # 9. Let a LOST temperature sensor arm the drive - the "no reading means
  #    probably fine" mistake. A KTY that has come adrift reads as a fixed,
  #    plausible, entirely fictional temperature, so this is indistinguishable
  #    from a cold motor right up until it is not.
  sed 's|      bad = DRIVE_FAULT_OVERTEMP;|      ;|' \
      "$ROOT/Core/Src/drive.c" > "$OUT/mut_hotsensor.c"

  fails=0
  for m in clear nosafe miso uvlatch torqsat torqfast torqrun hotrun hotsensor; do
    "$CC" -std=gnu11 -O2 "${INC[@]}" "${DSRC[@]}" "$OUT/mut_$m.c"           -o "$OUT/mut_$m.exe" -lm 2>/dev/null
    if "$OUT/mut_$m.exe" >"$OUT/mut_$m.log" 2>&1; then
      echo "  NOT CAUGHT  drive mutant '$m' passed - the tests are too weak"
      fails=$((fails + 1))
    else
      echo "  caught      drive mutant '$m': $(grep -c '  FAIL' "$OUT/mut_$m.log") test(s) failed"
    fi
  done

  # 10. An 'MTPA solution' on a surface-magnet machine: id driven off zero,
  #     which buys no torque and costs copper loss. This is the mistake the
  #     note in foc.h exists to prevent.
  sed 's|  f->id_ref = 0.0f;|  f->id_ref = 0.5f;|' \
      "$ROOT/Core/Src/foc.c" > "$OUT/mut_torqid.c"
  # 11. Echo the REQUEST back instead of what was accepted, so a clamped
  #     over-ask looks to the plausibility monitor like an implausible
  #     command and faults a drive that behaved correctly.
  sed 's|  return (int32_t)(FOC_IqToTorque((float)iq_ma \* 0.001f) \* 1000.0f);|  return t_mnm;|' \
      "$ROOT/Core/Src/foc.c" > "$OUT/mut_torqecho.c"
  # 12. Wrong torque constant - the kt the VCU's whole torque request rests on.
  sed 's|  return iq_a \* FOC_KT_NM_PER_A;|  return iq_a * FOC_KT_NM_PER_A * 2.0f;|' \
      "$ROOT/Core/Src/foc.c" > "$OUT/mut_torqkt.c"

  # 13. Field weakening fed the POST-limiter magnitude. The limiter scales
  #     vd/vq down to vmax, so the headroom reads zero rather than negative
  #     and the loop never engages at all - it compiles, it runs, it does
  #     nothing, and only a saturated operating point shows it.
  sed 's|    float headroom = f->vmax - vmag;|    float headroom = f->vmax - fm_sqrtf(f->vd * f->vd + f->vq * f->vq);|' \
      "$ROOT/Core/Src/foc.c" > "$OUT/mut_fwpost.c"
  # 14. Drop the upper clamps, so weakening can drive id POSITIVE - spending
  #     current to strengthen the field and make the saturation worse.
  sed -e 's|    if (f->fw_integ > 0.0f)          { f->fw_integ = 0.0f; }||' \
      -e 's|    if (f->id_ref > 0.0f)          { f->id_ref = 0.0f; }||' \
      "$ROOT/Core/Src/foc.c" > "$OUT/mut_fwsign.c"
  # 15. Drop the magnitude bound, so the weakening loop can claim the whole
  #     current budget that iq needs to share with it.
  sed -e 's|    if (f->fw_integ < -f->fw_id_max) { f->fw_integ = -f->fw_id_max; }||' \
      -e 's|    if (f->id_ref < -f->fw_id_max) { f->id_ref = -f->fw_id_max; }||' \
      "$ROOT/Core/Src/foc.c" > "$OUT/mut_fwbound.c"

  # 16. Measure the inductance ripple without letting the resistance phase's
  #     current decay first, so the peak-to-peak reads the decay instead.
  sed 's|    if (s->tick < IDENT_L_SETTLE_TICKS) { return; }||' \
      "$ROOT/Core/Src/ident.c" > "$OUT/mut_identdec.c"
  # 17. Drop the overcurrent abort. There is no hardware overcurrent path on
  #     this board, so this routine is its own protection.
  sed 's|    ident_stop(s, (uint32_t)IDENT_FAIL, IDENT_FAIL_OVERCUR);||' \
      "$ROOT/Core/Src/ident.c" > "$OUT/mut_identabort.c"

  for m in pi aw torqid torqecho torqkt fwpost fwsign fwbound identdec identabort; do
    # Mutants are built without -Werror: they leave variables unused by
    # construction, and that is not what is being checked.
    # ident mutants stand in for ident.c; the rest stand in for foc.c.
    # Swap whichever file the mutant replaces out of the source list.
    case "$m" in
      ident*) MSRC=( "$ROOT/test/host/test_foc.c" "$ROOT/test/host/pmsm.c"
                     "$ROOT/test/host/cordic_model.c" "$ROOT/Core/Src/foc.c" ) ;;
      *)      MSRC=( "${SRC[@]}" ) ;;
    esac
    "$CC" -std=gnu11 -O2 "${INC[@]}" "${MSRC[@]}" "$OUT/mut_$m.c" \
          -o "$OUT/mut_$m.exe" -lm 2>/dev/null
    if "$OUT/mut_$m.exe" >"$OUT/mut_$m.log" 2>&1; then
      echo "  NOT CAUGHT  mutant '$m' passed the suite - the tests are too weak"
      fails=$((fails + 1))
    else
      echo "  caught      mutant '$m': $(grep -c '  FAIL' "$OUT/mut_$m.log") test(s) failed"
    fi
  done
  [ "$fails" -eq 0 ] || exit 1
fi

exit $rc
