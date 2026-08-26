/**
  ******************************************************************************
  * @file    test/host/sim_dump.c
  * @brief   Run a scenario through the simulated drive and write CSV.
  *
  *          step     <lo_ma> <hi_ma> <we> [ms]   speed HELD, current loop only
  *          sweep    <iq_ma> <we>                steady state, ripple and dq
  *          reversal <iq_ma> [spin_ms] [rev_ms]  rotor FREE, the decouple case
  *
  *          opts: decouple= delay= vmax= lambda_err= L_err= vel_alpha=
  *                vel_ideal= fw= fw_ki= fw_kp= fw_id_max=
  *
  *          Same harness the assertions use (sim.h), so a plot cannot show
  *          behaviour the tests never exercised.
  *
  *          Usage:
  *            sim_dump step  <iq_lo_mA> <iq_hi_mA> <w_e> [ms] [opts] > out.csv
  *            sim_dump sweep <iq_mA>    <w_e>              [opts] > out.csv
  *
  *          Options are key=value and may follow in any order:
  *
  *            decouple=0|1     f->decouple          (default: firmware default)
  *            delay=0|1        f->delay_comp        (default: firmware default)
  *            lambda_err=1.15  scale the FIRMWARE's lambda_m against the
  *                             model's, to study a parameter error - 1.15 is
  *                             the 15% over-estimate the bench shipped with
  *            vmax=0.25        modulation ceiling
  *
  *          lambda_err is the interesting one. The model keeps the measured
  *          2.68 mWb and the firmware constant is left alone, so the error is
  *          introduced by scaling the MODEL's magnet down - which is
  *          indistinguishable, from the firmware's point of view, from a motor
  *          whose lambda_m it has wrong.
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

/* key=value options, applied after sim_init so they override the firmware
 * defaults rather than being overwritten by them. */
static void apply_opts(Sim_t *s, int argc, char **argv, int from)
{
  for (int i = from; i < argc; i++)
  {
    const char *a = argv[i];
    const char *eq = strchr(a, '=');
    if (!eq) { continue; }
    double v = atof(eq + 1);

    if      (!strncmp(a, "decouple=", 9))   { s->f.decouple   = (uint32_t)v; }
    else if (!strncmp(a, "delay=", 6))      { s->f.delay_comp = (uint32_t)v; }
    else if (!strncmp(a, "vmax=", 5))       { s->f.vmax       = (float)v; }
    else if (!strncmp(a, "lambda_err=", 11) && v > 0.0)
    {
      /* Scale the MODEL's magnet, leaving the firmware constant alone: from
       * the loop's point of view that is exactly a motor whose lambda_m it
       * has wrong by this factor. */
      s->m.lambda_m = (double)FOC_LAMBDA_M_WB / v;
    }
    else if (!strncmp(a, "L_err=", 6) && v > 0.0)
    {
      /* Same trick for inductance: scale the MODEL's Ld/Lq and leave
       * FOC_LD_H / FOC_LQ_H alone, so the firmware keeps designing its gains
       * and its w_e*Lq*iq feedforward for the value it was given while the
       * plant behaves like a motor that is wrong by this factor. L_err > 1
       * means the firmware OVER-estimates - the case worth testing, because
       * a bench step measured a current loop about 3x faster than the model,
       * which is what a real inductance below the assumed one looks like.
       *
       * Ld and Lq move together: this machine is surface-magnet and the
       * firmware defines both as FOC_L_H, so splitting them here would model
       * a saliency neither the motor nor the controller has. */
      s->m.Ld = (double)FOC_LD_H / v;
      s->m.Lq = (double)FOC_LQ_H / v;
    }
    else if (!strncmp(a, "fw=", 3))          { s->f.fw_enable = (uint32_t)v; }
    else if (!strncmp(a, "fw_ki=", 6))       { s->f.fw_ki     = (float)v;    }
    else if (!strncmp(a, "fw_kp=", 6))       { s->f.fw_kp     = (float)v;    }
    else if (!strncmp(a, "fw_id_max=", 10))  { s->f.fw_id_max = (float)v;    }
    else if (!strncmp(a, "vel_alpha=", 10) && v > 0.0)
    {
      /* The EMA the position loop applies to the 1 kHz encoder difference.
       * POS_VEL_ALPHA (0.05) is the shipped 8 Hz corner; how far that can be
       * raised is the whole question the reversal scenario exists to ask. */
      s->vel_alpha = v;
    }
    else if (!strncmp(a, "vel_ideal=", 10))
    {
      /* Bypass the modelled velocity path and hand the loop the model's exact
       * omega_m. Not physical - it is the control this failure is measured
       * against, showing what the feedforward is worth when its omega_e has
       * no lag and no quantisation. */
      s->vel_ideal = (int)v;
    }
    else { fprintf(stderr, "ignoring unknown option '%s'\n", a); }
  }
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
    double ms = (argc > 5 && !strchr(argv[5], '=')) ? atof(argv[5]) : 6.0;
    apply_opts(&s, argc, argv, 5);

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
    apply_opts(&s, argc, argv, 4);

    s.f.iq_ref = (float)iq;
    for (long i = 0; i < lround(0.05 / SIM_TS); i++) { hold_speed(&s, we); sim_step(&s, 0.0); }

    emit_header();
    double t0 = s.t;
    for (long i = 0; i < lround(0.02 / SIM_TS); i++)
    { hold_speed(&s, we); sim_step(&s, 0.0); emit_row(&s, t0); }
    return 0;
  }

  if (strcmp(argv[1], "reversal") == 0 && argc >= 3)
  {
    /* The case that keeps f->decouple switched off.
     *
     * Spin up FREE - no hold_speed - until back-EMF meets the modulation
     * ceiling, then command the opposite current and let the rotor actually
     * decelerate through zero and reverse. That is where omega_e's lag turns
     * the feedforward into a disturbance: braking at this current decelerates
     * the rotor at roughly 16000 electrical rad/s^2, so an omega_e running
     * 20 ms behind is wrong by ~320 rad/s, and 320 rad/s of lambda_m across
     * an 85 mOhm winding is about 10 A.
     *
     * A SHORT reversal does not show this and reads as a pass - see the work
     * plan. The rotor has to have reached the speed where it bites. */
    double iq      = atof(argv[2]) / 1000.0;
    double spin_ms = (argc > 3 && !strchr(argv[3], '=')) ? atof(argv[3]) : 500.0;
    double rev_ms  = (argc > 4 && !strchr(argv[4], '=')) ? atof(argv[4]) : 400.0;

    /* This scenario exists to study omega_e's lag, so it needs the modelled
     * velocity path rather than the model's exact speed. Set before
     * apply_opts so vel_ideal= can still override it. */
    s.vel_ideal = 0;
    apply_opts(&s, argc, argv, 3);

    s.f.iq_ref = (float)iq;
    for (long i = 0; i < lround(spin_ms / 1000.0 / SIM_TS); i++) { sim_step(&s, 0.0); }

    emit_header();
    double t0 = s.t;
    for (long i = 0; i < lround(0.002 / SIM_TS); i++)
    { sim_step(&s, 0.0); emit_row(&s, t0 + 0.002); }

    s.f.iq_ref = -(float)iq;
    for (long i = 0; i < lround(rev_ms / 1000.0 / SIM_TS); i++)
    { sim_step(&s, 0.0); emit_row(&s, t0 + 0.002); }
    return 0;
  }

  fprintf(stderr, "unknown scenario\n");
  return 2;
}
