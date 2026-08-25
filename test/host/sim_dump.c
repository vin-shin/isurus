/**
  ******************************************************************************
  * @file    test/host/sim_dump.c
  * @brief   Run a scenario through the simulated drive and write CSV.
  *
  *          Same harness the assertions use (sim.h), so a plot cannot show
  *          behaviour the tests never exercised.
  *
  *          Usage:
  *            sim_dump step  <iq_lo_mA> <iq_hi_mA> <w_e> [ms]   > out.csv
  *            sim_dump sweep <iq_mA>    <w_e>                   > out.csv
  *
  *          `step` mirrors what tools/step_trace.sh does on hardware: hold at
  *          iq_lo, step to iq_hi, capture across the transient. Feed both to
  *          tools/viz.py compare and the model and the machine can be laid
  *          over each other - which is the only way to find out whether a
  *          disagreement is the firmware or the model.
  *
  *          w_e is held fixed rather than let the rotor accelerate, matching
  *          the bench procedure: the current loop settles in about a
  *          millisecond and the machine takes hundreds, so speed is constant
  *          across the window either way, and pinning it makes runs
  *          comparable.
  ******************************************************************************
  */

#include "sim.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static void hold_speed(Sim_t *s, double we)
{
  s->m.omega_m = we / (double)s->m.p;
}

static void emit_header(void)
{
  printf("t_ms,id_a,iq_a,vd,vq,duty_u,duty_v,duty_w,theta_e,we,iq_ref\n");
}

static void emit_row(const Sim_t *s, double t0)
{
  printf("%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.3f,%.6f\n",
         (s->t - t0) * 1000.0,
         s->m.id, s->m.iq,
         (double)s->f.vd, (double)s->f.vq,
         (double)s->f.duty_u, (double)s->f.duty_v, (double)s->f.duty_w,
         Pmsm_ThetaE(&s->m),
         s->m.omega_m * (double)s->m.p,
         (double)s->f.iq_ref);
}

int main(int argc, char **argv)
{
  if (argc < 2) { fprintf(stderr, "see the comment at the top of this file\n"); return 2; }

  Sim_t s; sim_init(&s);
  s.f.enabled = 1;

  if (strcmp(argv[1], "step") == 0 && argc >= 5)
  {
    double lo = atof(argv[2]) / 1000.0;
    double hi = atof(argv[3]) / 1000.0;
    double we = atof(argv[4]);
    double ms = (argc > 5) ? atof(argv[5]) : 6.0;

    /* Settle at the low current first, exactly as the bench does, so the
     * integrator state at the step is the same in both. */
    s.f.iq_ref = (float)lo;
    for (long i = 0; i < lround(0.05 / SIM_TS); i++) { hold_speed(&s, we); sim_step(&s, 0.0); }

    emit_header();
    double t0 = s.t;

    /* 2 ms of pre-roll, matching STEP_PRE_TICKS on the target. */
    for (long i = 0; i < lround(0.002 / SIM_TS); i++)
    { hold_speed(&s, we); sim_step(&s, 0.0); emit_row(&s, t0 + 0.002); }

    s.f.iq_ref = (float)hi;
    for (long i = 0; i < lround(ms / 1000.0 / SIM_TS); i++)
    { hold_speed(&s, we); sim_step(&s, 0.0); emit_row(&s, t0 + 0.002); }
    return 0;
  }

  if (strcmp(argv[1], "sweep") == 0 && argc >= 4)
  {
    /* Steady current at a fixed speed - for looking at ripple and the dq
     * locus rather than a transient. */
    double iq = atof(argv[2]) / 1000.0;
    double we = atof(argv[3]);

    s.f.iq_ref = (float)iq;
    for (long i = 0; i < lround(0.05 / SIM_TS); i++) { hold_speed(&s, we); sim_step(&s, 0.0); }

    emit_header();
    double t0 = s.t;
    for (long i = 0; i < lround(0.02 / SIM_TS); i++)
    { hold_speed(&s, we); sim_step(&s, 0.0); emit_row(&s, t0); }
    return 0;
  }

  fprintf(stderr, "unknown scenario\n");
  return 2;
}
