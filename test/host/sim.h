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
#include "pmsm.h"

#include <math.h>
#include <string.h>

#define SIM_TS      (1.0 / 30000.0)
#define SIM_TWO_PI  6.283185307179586

typedef struct {
  Pmsm_t     m;
  FocState_t f;
  double     d_u, d_v, d_w;   /* duties in flight, applied next step */
  double     t;               /* seconds since sim_init */
} Sim_t;

static inline void sim_init(Sim_t *s)
{
  memset(s, 0, sizeof(*s));
  Pmsm_Init(&s->m);
  FOC_Init(&s->f);
  Cordic_Host_Reset();               /* FOC_Init used its own access order */
  FOC_SetGainsForVbus(&s->f, (int32_t)(s->m.vbus * 1000.0));
  s->d_u = s->d_v = s->d_w = 0.5;    /* zero volts */
  s->t = 0.0;
}

static inline void sim_step(Sim_t *s, double t_load)
{
  double iu, iv, iw, vd, vq;

  /* Apply what the PREVIOUS control step commanded - the transport delay. */
  Pmsm_DutiesToDq(&s->m, s->d_u, s->d_v, s->d_w, &vd, &vq);
  Pmsm_Step(&s->m, vd, vq, t_load, SIM_TS);

  Pmsm_Phases(&s->m, &iu, &iv, &iw);
  FOC_Update(&s->f,
             (int32_t)lround(iu * 1000.0),
             (int32_t)lround(iw * 1000.0),
             (uint16_t)Pmsm_EncoderCount(&s->m),
             (float)s->m.omega_m);

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
