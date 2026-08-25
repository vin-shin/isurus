/**
  ******************************************************************************
  * @file    fastmath.h
  * @brief   Single-instruction float helpers for the control ISR.
  *
  *          The Cortex-M4F has a hardware square root, VSQRT.F32, that retires
  *          in about 14 cycles. Calling sqrtf() does NOT get you it here:
  *          the C standard requires sqrtf to set errno for a negative
  *          argument, so GCC emits a call into libm. -fno-math-errno lifts
  *          that requirement, but the builtin expansion that turns it into the
  *          instruction only runs when optimisation is on, and this project
  *          builds at -O0 for debuggability.
  *
  *          The cost is not academic. Two sqrtf calls in the position
  *          profile's braking maths measured ~1400 cycles between them and
  *          pushed the worst-case control ISR from 39.2 us to 50.1 us - past
  *          the 50 us PWM period, which means dropped control steps.
  *
  *          So the instruction is requested explicitly. This stays correct at
  *          any optimisation level and does not depend on a compiler flag
  *          surviving a CubeMX regeneration.
  ******************************************************************************
  */
#ifndef FASTMATH_H
#define FASTMATH_H

#ifdef __cplusplus
extern "C" {
#endif

/* Hardware square root.
 *
 * VSQRT returns NaN for a negative argument rather than setting errno, so
 * every caller must pass a value that is non-negative by construction. All
 * current callers pass either a sum of squares or a magnitude scaled by a
 * positive constant. */
__attribute__((always_inline))
static inline float fm_sqrtf(float x)
{
  float r;
  __asm__ volatile ("vsqrt.f32 %0, %1" : "=t" (r) : "t" (x));
  return r;
}

#ifdef __cplusplus
}
#endif

#endif /* FASTMATH_H */
