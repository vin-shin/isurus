/**
  ******************************************************************************
  * @file    ident.c
  * @brief   Stationary R and L measurement. See ident.h for the method.
  ******************************************************************************
  */

#include "ident.h"

/* Control period, derived rather than written down - a switching-frequency
 * change has to move the inductance result with it, because L is computed
 * from a slope measured over exactly one of these. */
#define IDENT_TS   (1.0f / (float)PWM_FREQ_HZ)

static void ident_stop(IdentState_t *s, uint32_t phase, uint32_t fail)
{
  s->vd_cmd    = 0.0f;
  s->vq_cmd    = 0.0f;
  s->phase     = phase;
  s->fail_code = fail;
}

void Ident_Init(IdentState_t *s)
{
  s->phase     = (uint32_t)IDENT_IDLE;
  s->tick      = 0U;
  s->vd_cmd    = 0.0f;
  s->vq_cmd    = 0.0f;
  s->r_acc     = 0.0f;
  s->r_n       = 0U;
  s->i_min     = 0.0f;
  s->i_max     = 0.0f;
  s->r_mohm    = 0;
  s->l_uh      = 0;
  s->fail_code = IDENT_FAIL_NONE;
}

void Ident_Start(IdentState_t *s, uint32_t what)
{
  Ident_Init(s);
  s->phase = what;
}

void Ident_Step(IdentState_t *s, float id_a, float iq_a, float vbus_v)
{
  if ((s->phase == (uint32_t)IDENT_IDLE) ||
      (s->phase == (uint32_t)IDENT_DONE) ||
      (s->phase == (uint32_t)IDENT_FAIL))
  {
    s->vd_cmd = 0.0f;
    s->vq_cmd = 0.0f;
    return;
  }

  /* Abort before anything else, on either axis. iq is checked too even though
   * nothing here commands q: if the rotor is turning, or the electrical angle
   * is not what the encoder says, injected d-axis current appears partly on q
   * and the assumption this whole routine rests on - that the shaft cannot
   * move - is already false. */
  float mag_d = (id_a < 0.0f) ? -id_a : id_a;
  float mag_q = (iq_a < 0.0f) ? -iq_a : iq_a;
  if ((mag_d > IDENT_I_ABORT_A) || (mag_q > IDENT_I_ABORT_A))
  {
    ident_stop(s, (uint32_t)IDENT_FAIL, IDENT_FAIL_OVERCUR);
    return;
  }

  s->tick++;
  s->vq_cmd = 0.0f;

  if (s->phase == (uint32_t)IDENT_R)
  {
    /* Regulate the current to the target rather than ramping the voltage at
     * it - see IDENT_R_KI. Runs for the whole phase, including while
     * averaging, so R is always measured at a known current. */
    s->vd_cmd += IDENT_R_KI * (IDENT_R_TARGET_A - mag_d);
    if (s->vd_cmd < 0.0f) { s->vd_cmd = 0.0f; }

    if (s->r_n == 0U)
    {
      if (mag_d < (IDENT_R_TARGET_A * 0.98f))
      {
        if (s->tick > IDENT_R_MAX_TICKS)
        {
          /* Never got there. An open phase, a dead gate driver, or a winding
           * whose resistance is far above what this was sized for. */
          ident_stop(s, (uint32_t)IDENT_FAIL, IDENT_FAIL_TIMEOUT);
        }
        return;
      }
      s->tick = 0U;     /* target reached: restart the clock for settle+avg */
    }

    /* R = V / I, with V in volts rather than per-unit. */
    if (mag_d > 0.001f)
    {
      s->r_acc += (s->vd_cmd * vbus_v) / id_a;
      s->r_n++;
    }

    if (s->tick >= IDENT_R_AVG_TICKS)
    {
      float r = (s->r_n > 0U) ? (s->r_acc / (float)s->r_n) : 0.0f;
      if (r < 0.0f) { r = -r; }
      s->r_mohm = (int32_t)(r * 1000.0f);

      /* Straight on to inductance, from zero volts and zero current. */
      s->phase  = (uint32_t)IDENT_L;
      s->tick   = 0U;
      s->vd_cmd = 0.0f;
      s->i_min  = 0.0f;    /* previous sample */
      s->i_max  = 0.0f;    /* sum of |di| */
      s->r_acc  = 0.0f;    /* count */
      s->r_n    = 0U;      /* no previous sample yet */
    }
    return;
  }

  if (s->phase == (uint32_t)IDENT_L)
  {
    /* Square wave: reverse on every tick. The mean stays at zero, so the R*i
     * term in di/dt = (V - R*i)/L stays negligible and the slope is V/L. */
    s->vd_cmd = ((s->tick & 1U) != 0U) ? IDENT_L_V_PU : -IDENT_L_V_PU;

    if (s->tick < IDENT_L_SETTLE_TICKS) { return; }

    /* MEAN per-tick slope, not min/max over the window.
     *
     * With the voltage reversing every tick the current is a triangle whose
     * consecutive samples differ by the full swing, so |di| per tick IS the
     * ripple - and averaging it rejects the sense noise that min/max
     * deliberately seeks out. On hardware the extremal version read L 28% low
     * against the datasheet-derived constant, because over 600 ticks the
     * extremes it found were the noisiest samples rather than the ripple. */
    if (s->r_n != 0U)          /* r_n reused here as "have a previous sample" */
    {
      float di = id_a - s->i_min;      /* i_min holds the previous sample */
      if (di < 0.0f) { di = -di; }
      s->i_max += di;                  /* i_max accumulates the sum */
      s->r_acc += 1.0f;                /* r_acc counts them */
    }
    s->i_min = id_a;
    s->r_n   = 1U;

    if (s->tick >= (IDENT_L_SETTLE_TICKS + IDENT_L_MEAS_TICKS))
    {
      float ipp = (s->r_acc > 0.0f) ? (s->i_max / s->r_acc) : 0.0f;
      if (ipp < 0.01f)
      {
        /* No ripple to divide by. Either the bridge is not actually driving,
         * or L is so large that a tick of this voltage barely moves the
         * current - both worth stopping for rather than reporting a number
         * derived from sense noise. */
        ident_stop(s, (uint32_t)IDENT_FAIL, IDENT_FAIL_NO_RIPPLE);
        return;
      }

      /* L = V * Ts / di_pp. The voltage swings +V to -V, so the current
       * ramps for one tick in each direction and di_pp is the excursion from
       * one tick's worth of slope. */
      float l_h = (IDENT_L_V_PU * vbus_v * IDENT_TS) / ipp;
      s->l_uh = (int32_t)(l_h * 1e6f);

      ident_stop(s, (uint32_t)IDENT_DONE, IDENT_FAIL_NONE);
    }
    return;
  }

  /* Unknown phase: stop rather than drive an undefined voltage. */
  ident_stop(s, (uint32_t)IDENT_FAIL, IDENT_FAIL_TIMEOUT);
}
