#include "pmsm.h"
#include "foc.h"
#include <math.h>

#define TWO_PI  6.283185307179586

void Pmsm_Init(Pmsm_t *m)
{
  /* The machine: EMRAX 228 HV on a 140s2p pack, values from foc.h.
   *
   * These are deliberately taken FROM the firmware's constants rather than
   * written out again, so the two cannot drift apart - a mismatch here does
   * not look like a harness bug, it looks like a control bug, which is
   * exactly how a 20 kHz firmware against a hardcoded 30 kHz simulator once
   * presented as a 45% inductance error.
   *
   * The caveat that applied to the bench machine applies here too, and more
   * so. lambda_m in foc.h is PROVISIONAL - derived, not measured, and the
   * three available derivations span 35%. A model carrying the same
   * unverified constant as the firmware will agree with it perfectly and
   * prove nothing about the motor. What these tests still check is that the
   * CONTROL is right given a machine; they cannot check that the machine is
   * the one on the dyno. Only a bench measurement does that. */
  m->R        = FOC_R_OHM;
  m->Ld       = FOC_LD_H;
  m->Lq       = FOC_LQ_H;
  m->lambda_m = FOC_LAMBDA_M_WB;
  m->p        = FOC_POLE_PAIRS;

  /* Inertia is an estimate, not a measurement. The EMRAX 228's rotor is
   * quoted around 0.0421 kg.m^2 - two and a half orders of magnitude above
   * the outrunner this harness used to model, which matters for the velocity
   * path and not at all for current-loop tests three orders of magnitude
   * faster. */
  m->J = 4.21e-2;
  m->B = 1.0e-3;

  m->vbus = (double)FOC_VBUS_NOM_MV / 1000.0;

  m->id = m->iq = 0.0;
  m->theta_m = 0.0;
  m->omega_m = 0.0;

  m->enc_frozen = 0; m->enc_frozen_count = 0; m->enc_jump = 0;
  m->sensor_rail_u = 0; m->phase_open_u = 0;
}

double Pmsm_ThetaE(const Pmsm_t *m)
{
  double te = fmod(m->theta_m * (double)m->p, TWO_PI);
  if (te < 0.0) { te += TWO_PI; }
  return te;
}

void Pmsm_Step(Pmsm_t *m, double vd, double vq, double t_load, double dt)
{
  double we = m->omega_m * (double)m->p;

  double did = (vd - m->R * m->id + we * m->Lq * m->iq) / m->Ld;
  double diq = (vq - m->R * m->iq - we * m->Ld * m->id - we * m->lambda_m) / m->Lq;

  m->id += did * dt;
  m->iq += diq * dt;

  double Te  = 1.5 * (double)m->p * m->lambda_m * m->iq;
  double dwm = (Te - m->B * m->omega_m - t_load) / m->J;

  m->omega_m += dwm * dt;
  m->theta_m += m->omega_m * dt;

  m->theta_m = fmod(m->theta_m, TWO_PI);
  if (m->theta_m < 0.0) { m->theta_m += TWO_PI; }
}

void Pmsm_Phases(const Pmsm_t *m, double *iu, double *iv, double *iw)
{
  double th = Pmsm_ThetaE(m);
  double c = cos(th), s = sin(th);

  /* Inverse Park then inverse Clarke, in foc.c's convention: ialpha = iu. */
  double ia = m->id * c - m->iq * s;
  double ib = m->id * s + m->iq * c;

  double u = ia;
  double v = (-ia + sqrt(3.0) * ib) * 0.5;
  double w = -(u + v);

  if (m->phase_open_u)   { u = 0.0; }
  if (m->sensor_rail_u)  { u = 40.0; }   /* CT4022 full scale */

  *iu = u; *iv = v; *iw = w;
}

int Pmsm_EncoderCount(const Pmsm_t *m)
{
  if (m->enc_frozen) { return m->enc_frozen_count & 0x7FFF; }

  int c = (int)llround(m->theta_m / TWO_PI * 32768.0);
  c += m->enc_jump;
  return c & 0x7FFF;
}

void Pmsm_DutiesToDq(const Pmsm_t *m, double du, double dv, double dw,
                     double *vd, double *vq)
{
  /* Duty 0.5 is zero volts relative to the DC-link midpoint. */
  double vu = (du - 0.5) * m->vbus;
  double vv = (dv - 0.5) * m->vbus;
  double vw = (dw - 0.5) * m->vbus;

  /* A three-wire machine sees only the differential part, so the common mode
   * - including whatever the min/max injection added - drops out here. That
   * is exactly why the injection is free, and modelling it wrongly would make
   * the injection look like a disturbance. */
  double vn = (vu + vv + vw) / 3.0;
  vu -= vn; vv -= vn; vw -= vn;

  double va = vu;
  double vb = (vu + 2.0 * vv) / sqrt(3.0);

  double th = Pmsm_ThetaE(m);
  double c = cos(th), s = sin(th);

  *vd =  va * c + vb * s;
  *vq = -va * s + vb * c;
}
