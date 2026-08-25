/**
  ******************************************************************************
  * @file    test/host/pmsm.h
  * @brief   Numerical PMSM model for the host harness.
  *
  *          Forward Euler on the rotor-frame state equations:
  *
  *              d(id)/dt = (vd - R*id + w_e*Lq*iq) / Ld
  *              d(iq)/dt = (vq - R*iq - w_e*Ld*id - w_e*lambda_m) / Lq
  *              Te       = 1.5 * p * lambda_m * iq            (Ld = Lq here)
  *              d(w_m)/dt= (Te - B*w_m - T_load) / J
  *
  *          Euler rather than anything better on purpose: the step is the
  *          control period, 33.3 us, and the fastest electrical time constant
  *          is L/R = 0.64 ms - about nineteen steps - so the integration error
  *          is far below the effects being tested. Using a fancier integrator
  *          would make the model less like the thing it stands in for, which
  *          is a real motor sampled once per PWM period.
  *
  *          The conversions to and from phase quantities deliberately mirror
  *          foc.c's own Clarke convention (ialpha = iu, amplitude-invariant),
  *          because a model using the OTHER common convention would rescale
  *          every current by 2/3 and quietly make the loop look mistuned.
  ******************************************************************************
  */
#ifndef PMSM_H
#define PMSM_H

typedef struct {
  /* Parameters - defaults are the bench 8309, see foc.h */
  double R, Ld, Lq, lambda_m;
  int    p;                 /* pole pairs */
  double J, B;              /* rotor inertia, viscous damping */
  double vbus;

  /* State */
  double id, iq;            /* A, rotor frame */
  double theta_m;           /* rad, mechanical, wrapped to [0, 2pi) */
  double omega_m;           /* rad/s, mechanical */

  /* Fault injection, applied when reading the model out */
  int    enc_frozen;        /* 1 = encoder returns a fixed count */
  int    enc_frozen_count;
  int    enc_jump;          /* counts added to the reported angle, once */
  int    sensor_rail_u;     /* 1 = phase U current reads full scale */
  int    phase_open_u;      /* 1 = phase U carries no current */
} Pmsm_t;

void   Pmsm_Init(Pmsm_t *m);

/* One control period. vd/vq are in VOLTS, in the model's own true rotor
 * frame - the harness derives them from the duties foc.c produced. */
void   Pmsm_Step(Pmsm_t *m, double vd, double vq, double t_load, double dt);

/* Read-out, with fault injection applied. */
void   Pmsm_Phases(const Pmsm_t *m, double *iu, double *iv, double *iw);
int    Pmsm_EncoderCount(const Pmsm_t *m);          /* 15-bit, 0..32767 */
double Pmsm_ThetaE(const Pmsm_t *m);

/* Turn the duties foc.c wrote back into rotor-frame volts. */
void   Pmsm_DutiesToDq(const Pmsm_t *m, double du, double dv, double dw,
                       double *vd, double *vq);

#endif /* PMSM_H */
