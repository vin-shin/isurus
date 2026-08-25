/**
  ******************************************************************************
  * @file    haptic.c
  * @brief   Force-feedback dial. See haptic.h for what the terms mean.
  ******************************************************************************
  */

#include "haptic.h"

#define DEG_X10_PER_COUNT   (3600.0f / (float)HAPTIC_COUNTS_PER_REV)

void Haptic_Init(HapticState_t *h)
{
  h->detent_count             = HAPTIC_DETENTS_DEFAULT;
  h->detent_ma                = HAPTIC_DETENT_MA_DEFAULT;
  h->detent_shape             = HAPTIC_SHAPE_DEFAULT;
  h->spring_ma_per_deg_x100   = 0;
  h->spring_center_deg_x10    = 0;
  h->damping_ma_per_dps_x100  = HAPTIC_DAMPING_DEFAULT;
  h->friction_ma              = 0;
  h->endstop_lo_deg_x10       = 0;
  h->endstop_hi_deg_x10       = 0;
  h->endstop_ma_per_deg_x100  = 0;
  h->torque_max_ma            = HAPTIC_TORQUE_MAX_DEFAULT;

  h->detent_index = 0;
  h->torque_ma    = 0;
  h->in_endstop   = 0;
  h->updates      = 0U;
}

/* sin(pi * x) for x in [-1, 1], to about 0.1%.
 *
 * A detent well is a sine, and this runs 20000 times a second, so it does not
 * call sinf(). The usual parabola 4x(1-|x|) is only good to ~5% and its error
 * peaks exactly where the detent wall is steepest, which is the part you feel;
 * folding in the |y|*y term fixes that for two more multiplies. No branches,
 * no table, and constant time - which is what an interrupt wants. */
static float sin_pi(float x)
{
  float ax = (x < 0.0f) ? -x : x;
  float y  = 4.0f * x * (1.0f - ax);
  float ay = (y < 0.0f) ? -y : y;
  return (0.775f * y) + (0.225f * y * ay);
}

float Haptic_Torque(HapticState_t *h, int32_t pos_counts, float vel_dps)
{
  float pos_deg_x10 = (float)pos_counts * DEG_X10_PER_COUNT;
  float ma = 0.0f;

  /* ---- detents --------------------------------------------------------- */
  if ((h->detent_count > 0) && (h->detent_ma != 0))
  {
    int32_t period = HAPTIC_COUNTS_PER_REV / h->detent_count;
    if (period < 2) { period = 2; }

    /* Offset from the NEAREST well centre, in [-period/2, +period/2).
     *
     * The rounding has to be symmetric about zero or the dial feels different
     * clockwise and anticlockwise: C division truncates toward zero, so a
     * plain pos/period puts the seam in the wrong place for negative angles.
     * Adding half a period before flooring keeps every well the same width on
     * both sides of the origin. */
    int32_t half = period / 2;
    int32_t idx  = (pos_counts >= 0)
                     ? ((pos_counts + half) / period)
                     : (-((-pos_counts + half) / period));
    int32_t off  = pos_counts - (idx * period);

    h->detent_index = idx;

    /* x = 0 at the centre, +/-1 at the boundary with the next well. The
     * boundary is where sin() crosses zero going the wrong way - an unstable
     * equilibrium - which is exactly the snap from one click to the next. */
    float x = (float)off / (float)half;
    if (h->detent_shape == HAPTIC_SHAPE_RAMP)
    {
      /* Linear in the offset, so |torque| is greatest at the boundary and the
       * sign flip across it is the click. */
      ma -= (float)h->detent_ma * x;
    }
    else
    {
      ma -= (float)h->detent_ma * sin_pi(x);
    }
  }
  else
  {
    h->detent_index = 0;
  }

  /* ---- spring ---------------------------------------------------------- */
  if (h->spring_ma_per_deg_x100 != 0)
  {
    float err_deg = (pos_deg_x10 - (float)h->spring_center_deg_x10) * 0.1f;
    ma -= ((float)h->spring_ma_per_deg_x100 * 0.01f) * err_deg;
  }

  /* ---- endstops -------------------------------------------------------- */
  h->in_endstop = 0;
  if (h->endstop_ma_per_deg_x100 != 0)
  {
    float k = (float)h->endstop_ma_per_deg_x100 * 0.01f;
    if (pos_deg_x10 > (float)h->endstop_hi_deg_x10)
    {
      ma -= k * ((pos_deg_x10 - (float)h->endstop_hi_deg_x10) * 0.1f);
      h->in_endstop = 1;
    }
    else if (pos_deg_x10 < (float)h->endstop_lo_deg_x10)
    {
      ma += k * (((float)h->endstop_lo_deg_x10 - pos_deg_x10) * 0.1f);
      h->in_endstop = -1;
    }
  }

  /* ---- damping: viscous, proportional to speed ------------------------- */
  if (h->damping_ma_per_dps_x100 != 0)
  {
    ma -= ((float)h->damping_ma_per_dps_x100 * 0.01f) * vel_dps;
  }

  /* ---- friction: dry, constant magnitude opposing travel ---------------- *
   *
   * Deadbanded around standstill on purpose. A constant torque that flips
   * sign with the sign of a noisy velocity estimate is an oscillator, and at
   * zero speed the estimate is nothing but noise - it would buzz rather than
   * feel like a brake. Below the threshold, friction simply stops existing. */
  if (h->friction_ma != 0)
  {
    if (vel_dps >  2.0f) { ma -= (float)h->friction_ma; }
    if (vel_dps < -2.0f) { ma += (float)h->friction_ma; }
  }

  /* ---- clamp ----------------------------------------------------------- */
  float lim = (float)h->torque_max_ma;
  if (lim < 0.0f) { lim = 0.0f; }
  if (ma >  lim) { ma =  lim; }
  if (ma < -lim) { ma = -lim; }

  h->torque_ma = (int32_t)ma;
  h->updates++;

  return ma * 0.001f;
}
