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
      "$ROOT/test/host/cordic_model.c" )
INC=( -I"$ROOT/test/host/shim" -I"$ROOT/Core/Inc" -I"$ROOT/test/host" )

# -Werror here too: the tests are code, and they were already caught once
# indenting two statements under one if.
build() { "$CC" -std=gnu11 -O2 -g -Wall -Wextra -Werror "${INC[@]}" "${SRC[@]}" "$1" -o "$2" -lm; }

build "$ROOT/Core/Src/foc.c" "$OUT/test_foc.exe"
"$OUT/test_foc.exe"
rc=$?

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

  fails=0
  for m in pi aw; do
    # Mutants are built without -Werror: they leave variables unused by
    # construction, and that is not what is being checked.
    "$CC" -std=gnu11 -O2 "${INC[@]}" "${SRC[@]}" "$OUT/mut_$m.c" \
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
