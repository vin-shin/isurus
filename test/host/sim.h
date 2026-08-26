/**
  ******************************************************************************
  * @file    test/host/sim.h
  * @brief   The simulated drive: foc.c closed around the PMSM model.
  *
  *          Header-only and shared by test_foc.c (assertions) and sim_dump.c
  *          (CSV for the plotting tools), so both drive the loop through
  *          exactly the same path. If they diverged, a plot could show
  *          something the tests never exercise, which is the opposite of what
  *          a simulator is for.
  *
  *          The transport delay is modelled: duties computed on step k are
  *          applied on step k+1. That one-step buffer plus the inherent
  *          half-period of zero-order hold is 1.5 periods, so foc.c's delay
  *          compensation is tested against a plant that actually has a delay
  *          rather than against an instantaneous one.
  *
  *          KNOWN FIDELITY LIMIT, and it matters for one thing only. The
  *          firmware compensates FOC_DELAY_PERIODS = 1.65 periods, because on
  *          the target the ADC is triggered PWM_ADC_LEAD_NS *before* the
  *          period boundary and that lead adds 0.15 of a period. This model
  *          samples exactly at the boundary, so its true delay is 1.5 and the
  *          firmware over-compensates it by 0.15 periods - about half a degree
  *          electrical at 1670 rad/s.
  *
  *          Consequence: numbers from this model may be used to judge the
  *          DECOUPLING, which does not depend on the delay, and must NOT be
  *          used to judge the delay compensation itself, which is measured
  *          here against a plant whose delay is deliberately not the one it
  *          was designed for. Enabling delay compensation in the model makes
  *          the d-axis transient slightly worse for exactly that reason, and
  *          that is an artefact of this file rather than a finding about
  *          foc.c. Modelling the sub-period sampling instant would fix it and
  *          has not been done.
  ******************************************************************************
  */
#ifndef SIM_H
#define SIM_H

#include "foc.h"
#include "main.h"
#include "motor_pwm.h"
#include "pmsm.h"
#include "position.h"

#include <math.h>
#include <string.h>

/* The simulator must step at the rate the firmware thinks it is running at.
 *
 * This was `1.0 / 30000.0`, which agreed with the firmware only for as long
 * as the firmware stayed at 30 kHz. Retargeting to a 20 kHz board made the
 * model advance 1.5x further per control tick than the code under test
 * believed, and the two inductance tests went +45% - which is very nearly the
 * 50% the ratio predicts, and looks exactly like a real identification error
 * rather than a harness one.
 *
 * That is the failure CLAUDE.md warns about, arriving through the back door:
 * the firmware derives everything from PWM_FREQ_HZ and the harness did not. */
#define SIM_TS      (1.0 / (double)PWM_FREQ_HZ)
#define SIM_TWO_PI  6.283185307179586

/* Velocity as the CONTROL sees it, not as the model knows it.
 *
 * This used to hand FOC_Update the model's exact instantaneous omega_m, which
 * made the feedforward perfect by construction and hid the one failure that
 * keeps f->decouple switched off on the bench: the omega_e the firmware
 * actually has is a 1 kHz encoder difference through an EMA, and during a
 * reversal it lags far enough to turn the feedforward into a disturbance.
 * A model that cannot reproduce that cannot be used to fix it.
 *
 * So the path here is position.c's, step for step - difference the QUANTISED
 * encoder count at POS_DECIM, low-pass with the same EMA - which brings the
 * lag and the quantisation noise with it. Both matter: the whole design
 * question is how far the corner can be raised before the noise it lets
 * through costs more than the lag it removes. */
typedef struct {
  Pmsm_t     m;
  FocState_t f;
  double     d_u, d_v, d_w;   /* duties in flight, applied next step */
  double     t;               /* seconds since sim_init */

  /* The position loop's velocity path, modelled. */
  double     vel_filt;        /* EMA state, mechanical rad/s              */
  double     vel_alpha;       /* EMA alpha at POS_RATE_HZ; POS_VEL_ALPHA  */
  long       enc_accum;       /* unwrapped encoder counts                 */
  long       enc_at_last;     /* accumulator at the previous decimation   */
  int        enc_prev;        /* last raw 15-bit count, for unwrapping    */
  unsigned   vel_decim;       /* counts up to POS_DECIM                   */
  int        vel_ideal;       /* 1 = bypass all of it and use omega_m     */
} Sim_t;

/* position.c keeps this private; same expression, same source constant. */
#define SIM_RAD_PER_COUNT   (SIM_TWO_PI / (double)POS_COUNTS_PER_REV)

static inline void sim_init(Sim_t *s)
{
  memset(s, 0, sizeof(*s));
  Pmsm_Init(&s->m);
  FOC_Init(&s->f);
  Cordic_Host_Reset();               /* FOC_Init used its own access order */
  FOC_SetGainsForVbus(&s->f, (int32_t)(s->m.vbus * 1000.0));
  s->d_u = s->d_v = s->d_w = 0.5;    /* zero volts */
  s->t = 0.0;

  s->vel_filt    = 0.0;
  s->vel_alpha   = (double)POS_VEL_ALPHA;   /* the shipped 8 Hz corner */
  s->enc_accum   = 0;
  s->enc_at_last = 0;
  s->enc_prev    = Pmsm_EncoderCount(&s->m);
  s->vel_decim   = 0U;

  /* DEFAULTS TO IDEAL, and that is a compromise worth reading.
   *
   * The modelled path is the physical one and ought to be the default. It is
   * not, because switching it on globally made the suite WEAKER: run.sh
   * --mutants stopped catching the anti-windup mutant, which it had been
   * catching by a single test. Shipping a simulator change that quietly
   * removes mutant coverage is precisely the failure --mutants exists to
   * report, so the modelled path is opt-in until the anti-windup tests are
   * strong enough not to depend on which omega_e they are handed.
   *
   * That the coverage was that marginal is itself the finding. See
   * docs/BENCH-2026-08-25.md. */
  s->vel_ideal   = 1;
}

/* One step of position.c's velocity path. Called every control tick; does the
 * work only on the POS_DECIM boundary, exactly as Position_Step does. */
static inline void sim_velocity_step(Sim_t *s)
{
  int cnt = Pmsm_EncoderCount(&s->m);
  int d   = cnt - s->enc_prev;
  if (d >  16384) { d -= 32768; }          /* unwrap the 15-bit count */
  if (d < -16384) { d += 32768; }
  s->enc_prev   = cnt;
  s->enc_accum += d;

  if (++s->vel_decim < POS_DECIM) { return; }
  s->vel_decim = 0U;

  double raw_vel = (double)(s->enc_accum - s->enc_at_last)
                 * SIM_RAD_PER_COUNT * (double)POS_RATE_HZ;
  s->enc_at_last = s->enc_accum;

  double a = s->vel_alpha;
  if (a <= 0.0) { a = 0.001; }
  if (a >  1.0) { a = 1.0;   }
  s->vel_filt += a * (raw_vel - s->vel_filt);
}

static inline void sim_step(Sim_t *s, double t_load)
{
  double iu, iv, iw, vd, vq;

  /* Apply what the PREVIOUS control step commanded - the transport delay. */
  Pmsm_DutiesToDq(&s->m, s->d_u, s->d_v, s->d_w, &vd, &vq);
  Pmsm_Step(&s->m, vd, vq, t_load, SIM_TS);

  Pmsm_Phases(&s->m, &iu, &iv, &iw);
  sim_velocity_step(s);
  FOC_Update(&s->f,
             (int32_t)lround(iu * 1000.0),
             (int32_t)lround(iw * 1000.0),
             (uint16_t)Pmsm_EncoderCount(&s->m),
             (float)(s->vel_ideal ? s->m.omega_m : s->vel_filt));

  s->d_u = s->f.duty_u;
  s->d_v = s->f.duty_v;
  s->d_w = s->f.duty_w;
  s->t  += SIM_TS;
}

static inline void sim_run(Sim_t *s, double seconds, double t_load)
{
  long n = lround(seconds / SIM_TS);
  for (long i = 0; i < n; i++) { sim_step(s, t_load); }
}

#endif /* SIM_H */
