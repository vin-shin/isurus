/**
  ******************************************************************************
  * @file    test/host/shim/fastmath.h
  * @brief   Host stand-in for Core/Inc/fastmath.h.
  *
  *          The real one is a single VSQRT.F32 in inline ARM assembly, which
  *          an x86 host obviously cannot assemble. Semantics are identical for
  *          every argument this code produces: sqrtf of a non-negative value.
  *
  *          The real header is explicit that VSQRT returns NaN rather than
  *          setting errno for a negative argument, and that every caller must
  *          pass something non-negative by construction. sqrtf here behaves the
  *          same way given -ffast-math is not in play, so a caller that ever
  *          violated that would produce NaN on both host and target rather
  *          than diverging - which is what makes this a fair substitution
  *          rather than a papered-over difference.
  ******************************************************************************
  */
#ifndef HOST_FASTMATH_H
#define HOST_FASTMATH_H

#include <math.h>

static inline float fm_sqrtf(float x) { return sqrtf(x); }

#endif /* HOST_FASTMATH_H */
