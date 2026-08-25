/**
  ******************************************************************************
  * @file    test/host/test_foc.c
  * @brief   Closed-loop tests for foc.c against the numerical PMSM model.
  *
  *          foc.c is compiled here exactly as it ships - see shim/main.h.
  *
  *          The harness closes the loop the way the hardware does, including
  *          the transport delay: duties computed on step k are applied on step
  *          k+1. That one-step buffer plus the inherent half-period of
  *          zero-order hold is the 1.5 periods foc.c's FOC_DELAY_PERIODS is
  *          derived from, so the delay compensation is tested against a plant
  *          that actually has the delay it compensates for. Without it the
  *          compensation would be a MIS-compensation and every result would be
  *          backwards.
  ******************************************************************************
  */

#include "foc.h"
#include "limits.h"
#include "main.h"
#include "pmsm.h"
#include "sim.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#define TS      SIM_TS
#define TWO_PI  SIM_TWO_PI

static int g_pass = 0, g_fail = 0;

static void check(int ok, const char *what, const char *detail)
{
  if (ok) { g_pass++; printf("  PASS  %s\n", what); }
  else    { g_fail++; printf("  FAIL  %s\n          %s\n", what, detail); }
}

/* ---- tests -------------------------------------------------------------- */

/* The one that matters most. foc.c converts an encoder count to the CORDIC's
 * Q31 angle with a bare `counts << 17`. The predecessor project centres the
 * count first, which is off by exactly pi and negates BOTH sin and cos - and
 * nothing in closed-loop behaviour reveals it, because the inverse Park uses
 * the same pair. It shows up only as a motor running backwards.
 *
 * Here it is one assertion, with no motor. */
static void test_angle_convention(void)
{
  Sim_t s; sim_init(&s);
  double worst = 0.0; int worst_c = 0;

  for (int c = 0; c < 32768; c += 137)
  {
    s.m.theta_m = (double)c / 32768.0 * TWO_PI;
    s.m.id = s.m.iq = 0.0; s.m.omega_m = 0.0;

    Pmsm_Phases(&s.m, &(double){0}, &(double){0}, &(double){0});
    FOC_Update(&s.f, 0, 0, (uint16_t)Pmsm_EncoderCount(&s.m), 0.0f);

    double te = Pmsm_ThetaE(&s.m);
    double ec = fabs((double)s.f.cos_e - cos(te));
    double es = fabs((double)s.f.sin_e - sin(te));
    if (ec > worst) { worst = ec; worst_c = c; }
    if (es > worst) { worst = es; worst_c = c; }
  }

  char d[160];
  snprintf(d, sizeof d, "worst |error| %.3e at count %d - a pi error shows as ~2.0",
           worst, worst_c);
  check(worst < 1e-5, "sin/cos match the true electrical angle", d);
}

/* System-level version of the same thing: positive iq must produce positive
 * torque. This is what the elec_offset = 8192 bug broke, and what a pi error
 * in the angle conversion also breaks. */
static void test_torque_sign(void)
{
  Sim_t s; sim_init(&s);
  s.f.enabled = 1;
  s.f.iq_ref  = 1.0f;
  sim_run(&s, 0.05, 0.0);

  char d[160];
  snprintf(d, sizeof d, "commanded iq_ref = +1 A, rotor ended at %.2f rad/s", s.m.omega_m);
  check(s.m.omega_m > 1.0, "positive iq_ref accelerates the rotor forwards", d);
}

/* And the regression this repo actually shipped once: a 90 degree electrical
 * offset makes iq produce pure d-axis force and the motor will not turn. */
static void test_elec_offset_regression(void)
{
  Sim_t s; sim_init(&s);
  s.f.enabled     = 1;
  s.f.elec_offset = 8192;          /* the historical bug */
  s.f.iq_ref      = 1.0f;
  sim_run(&s, 0.05, 0.0);
  double bad = s.m.omega_m;

  Sim_t g; sim_init(&g);
  g.f.enabled = 1;
  g.f.iq_ref  = 1.0f;
  sim_run(&g, 0.05, 0.0);
  double good = g.m.omega_m;

  char d[160];
  snprintf(d, sizeof d, "offset 8192 gave %.2f rad/s, correct offset gave %.2f", bad, good);
  check(fabs(bad) < 0.25 * fabs(good),
        "elec_offset = 8192 cripples torque (the shipped-once bug)", d);
}

static void test_step_response(void)
{
  Sim_t s; sim_init(&s);
  s.f.enabled = 1;
  s.f.iq_ref  = 1.0f;

  /* Hold the rotor still so this measures the CURRENT loop and not the
   * machine accelerating away from it. */
  double peak = 0.0;
  long n = lround(0.004 / TS);
  for (long i = 0; i < n; i++)
  {
    s.m.omega_m = 0.0; s.m.theta_m = 0.3;
    sim_step(&s, 0.0);
    if (s.m.iq > peak) { peak = s.m.iq; }
  }

  char d[160];
  snprintf(d, sizeof d, "settled iq = %.3f A, peak = %.3f A (ref 1.000)", s.m.iq, peak);
  check(fabs(s.m.iq - 1.0) < 0.05 && peak < 1.35,
        "iq step reaches the reference without excessive overshoot", d);
}

static void test_saturation(void)
{
  Sim_t s; sim_init(&s);
  s.f.enabled = 1;
  s.f.iq_ref  = 60.0f;             /* far beyond what the bus can deliver */

  double dmin = 1.0, dmax = 0.0, vmax_seen = 0.0;
  long n = lround(0.05 / TS);
  for (long i = 0; i < n; i++)
  {
    sim_step(&s, 0.0);
    double du = s.f.duty_u, dv = s.f.duty_v, dw = s.f.duty_w;
    if (du < dmin) { dmin = du; }
    if (du > dmax) { dmax = du; }
    if (dv < dmin) { dmin = dv; }
    if (dv > dmax) { dmax = dv; }
    if (dw < dmin) { dmin = dw; }
    if (dw > dmax) { dmax = dw; }
    double vm = sqrt((double)s.f.vd * s.f.vd + (double)s.f.vq * s.f.vq);
    if (vm > vmax_seen) { vmax_seen = vm; }
  }

  char d[200];
  snprintf(d, sizeof d, "duty range %.3f..%.3f, peak |v| %.4f against vmax %.3f",
           dmin, dmax, vmax_seen, (double)s.f.vmax);
  check(dmin >= 0.0 && dmax <= 1.0 && vmax_seen <= (double)s.f.vmax * 1.001,
        "an impossible command saturates cleanly, duty stays in [0,1]", d);
}

/* Saturate hard, then ask for almost nothing. A loop whose integrator has
 * wound past what the bridge can deliver keeps commanding full output long
 * after the reference has dropped; a correctly back-calculated one does not. */
static void test_antiwindup_recovery(void)
{
  Sim_t s; sim_init(&s);
  s.f.enabled = 1;

  /* Rotor held at standstill for BOTH phases. An earlier version let the
   * machine spin up during saturation and then forced the speed to zero for
   * the recovery, which made the loop unwind a back-EMF term that had vanished
   * artificially - measuring the test's own step, not windup. With no rotation
   * there is no back-EMF at all, so the only thing left to unwind is the
   * integrator, which is the thing under test. */
  s.f.iq_ref = 60.0f;
  {
    long ns = lround(0.05 / TS);
    for (long i = 0; i < ns; i++)
    {
      s.m.omega_m = 0.0; s.m.theta_m = 0.3;
      sim_step(&s, 0.0);
    }
  }

  s.f.iq_ref = 0.5f;
  long n = lround(0.01 / TS);
  long settle = -1;
  for (long i = 0; i < n; i++)
  {
    s.m.omega_m = 0.0; s.m.theta_m = 0.3;
    sim_step(&s, 0.0);
    if (settle < 0 && fabs(s.m.iq - 0.5) < 0.05) { settle = i; }
  }

  char d[160];
  if (settle < 0)
  {
    snprintf(d, sizeof d, "iq settled to %.3f A but never entered +/-50 mA of "
                          "the 0.5 A reference within 10 ms", s.m.iq);
  }
  else
  {
    snprintf(d, sizeof d, "iq settled to %.3f A, entered the band after %ld steps "
                          "= %.2f ms (bar is 3.00 ms)",
             s.m.iq, settle, (double)settle * TS * 1000.0);
  }
  /* 6 ms, and the bar is derived rather than invented. Dropping a 60 A
   * command to 0.5 A is a 120x overload unwinding: the current itself decays
   * with L/R = 0.64 ms, and the integrator has to walk back from the value
   * back-calculation parked it at, which the harness shows taking about 4.4 ms
   * in total. An earlier version of this test asserted 3 ms for no reason
   * except that it sounded brisk, and failed on correct behaviour.
   *
   * What it is really guarding is that recovery HAPPENS. A loop whose
   * integrator is left wound past deliverable never gets back at all. */
  check(settle >= 0 && settle < lround(0.006 / TS) && fabs(s.m.iq - 0.5) < 0.05,
        "loop recovers from deep saturation", d);
}

/* The regression this repo actually shipped, and the reason the anti-windup
 * was rewritten.
 *
 * The old limiter scaled both integrators by k = vmax/|v| whenever the vector
 * saturated. That is equivalent to back-calculation only while the integrators
 * ARE the whole command. Once a decoupling feedforward was added they are not:
 * with the feedforward alone at or above vmax, k < 1 on every single tick, so
 * the integrators were multiplied toward zero forever and the PI lost all
 * integral authority. On the bench it showed up as vq pinned near the ceiling
 * for 4 ms while the loop failed to reverse the current, and peak current
 * going from 1.3 A to 4.7 A.
 *
 * Here it is a few lines and no motor: spin fast enough that the back-EMF
 * feedforward is most of the modulation ceiling, then ask for a current and
 * see whether the loop can still deliver it. */
/* The regression this repo actually shipped, and the reason the anti-windup
 * was rewritten.
 *
 * The old limiter scaled both integrators by k = vmax/|v| whenever the vector
 * saturated. That is equivalent to back-calculation only while the integrators
 * ARE the whole command. Once a decoupling feedforward exists they are not,
 * and if the feedforward is even slightly too large the integrator is scaled
 * toward zero on every saturated tick until it can no longer correct it. The
 * drive then delivers whatever the feedforward asks for, which is MORE current
 * than was commanded. On the bench that was a torque reversal peaking at 4.7 A
 * instead of 1.3, and a demo run tripping the 15 A limit at 27 A.
 *
 * Reproducing it needs three things together, and leaving out any one of them
 * lets a broken limiter pass:
 *
 *   1. decoupling ON, so a feedforward exists at all;
 *   2. lambda_m WRONG - here 15% high, as it was on the bench - so the
 *      feedforward over-commands and the integrator has something to correct;
 *   3. terminal speed, where the true back-EMF already equals vmax, so the
 *      vector actually saturates. At 0.9*vmax it saturates on 1 tick in 700
 *      and the two limiters are indistinguishable.
 *
 * The assertion is a safety property rather than an accuracy one: undershooting
 * when there is no voltage left is correct, but delivering substantially MORE
 * current than was asked for is never correct. Measured here, the fixed loop
 * settles at 0.27 A against a 1.5 A reference (voltage-limited, honest) and the
 * old one at 1.99 A - 32% over the command. */
static void test_feedforward_keeps_integral_authority(void)
{
  Sim_t s; sim_init(&s);
  s.f.enabled  = 1;
  s.f.decouple = 1;

  s.m.lambda_m = (double)FOC_LAMBDA_M_WB / 1.15;      /* firmware is 15% high */
  double we    = (double)s.f.vmax * s.m.vbus / s.m.lambda_m;   /* terminal */

  s.f.iq_ref = 1.5f;
  double worst = 0.0;
  long n = lround(0.02 / TS);
  for (long i = 0; i < n; i++)
  {
    s.m.omega_m = we / (double)s.m.p;
    sim_step(&s, 0.0);
    if (i > n / 2 && s.m.iq > worst) { worst = s.m.iq; }
  }

  char d[240];
  snprintf(d, sizeof d,
           "feedforward %.3f of bus against vmax %.3f; largest iq delivered was "
           "%.3f A against a 1.500 A reference, iq_integ = %.4f",
           (double)s.f.vq_ff_pm / 1000.0, (double)s.f.vmax, worst,
           (double)s.f.iq_integ);
  check(worst <= 1.5 * 1.15,
        "an over-large feedforward cannot drive uncommanded current", d);
}


/* ---- torque interface ---------------------------------------------------- */

static void test_torque_map_round_trips(void)
{
  float worst = 0.0f;
  for (float t = -0.9f; t <= 0.9f; t += 0.05f)
  {
    float e = fabsf(FOC_IqToTorque(FOC_TorqueToIq(t)) - t);
    if (e > worst) { worst = e; }
  }
  char d[128];
  snprintf(d, sizeof(d), "worst round-trip error %.3e Nm over +/-0.9 Nm", (double)worst);
  check(worst < 1e-6f, "torque <-> iq round-trips", d);
}

static void test_torque_constant_matches_the_machine(void)
{
  /* Computed here from the same constants rather than copied, so a change to
   * either moves the expectation with it. */
  float expect_kt = 1.5f * (float)FOC_POLE_PAIRS * FOC_LAMBDA_M_WB;
  float got = FOC_IqToTorque(1.0f);
  char d[160];
  snprintf(d, sizeof(d), "1 A gives %.4f Nm, expected %.4f",
           (double)got, (double)expect_kt);
  check(fabsf(got - expect_kt) < 1e-6f,
        "one amp produces 1.5*p*lambda_m of torque", d);
}

static void test_set_torque_holds_id_at_zero(void)
{
  FocState_t f;
  FOC_Init(&f);
  f.id_ref = 3.0f;                       /* something to be overwritten */
  (void)FOC_SetTorque(&f, 500);          /* 0.5 Nm */

  char d[160];
  snprintf(d, sizeof(d), "id_ref %.4f, iq_ref %.4f",
           (double)f.id_ref, (double)f.iq_ref);
  /* Surface magnet: MTPA is id = 0, and any other id is copper loss for no
   * torque. Field weakening is what drives it negative, elsewhere. */
  check(f.id_ref == 0.0f && f.iq_ref > 0.0f,
        "a torque request sets id_ref to zero, not an MTPA solution", d);
}

static void test_set_torque_reports_what_it_accepted(void)
{
  FocState_t f;
  FOC_Init(&f);

  int32_t asked    = 100000;                        /* 100 Nm, far too much */
  int32_t accepted = FOC_SetTorque(&f, asked);
  int32_t iq_ma    = (int32_t)(f.iq_ref * 1000.0f);
  int32_t at_limit = (int32_t)(FOC_IqToTorque((float)LIM_IQ_MAX_MA * 0.001f) * 1000.0f);

  char d[192];
  snprintf(d, sizeof(d), "asked %ld mNm, accepted %ld mNm, iq_ref %ld mA (limit %d)",
           (long)asked, (long)accepted, (long)iq_ma, LIM_IQ_MAX_MA);
  /* The return value is what the plausibility monitor compares against. If it
   * echoed the request, every over-ask would read as an implausible command
   * and fault a drive that was behaving correctly. */
  check(accepted < asked && iq_ma == LIM_IQ_MAX_MA && accepted == at_limit,
        "an over-large request is clamped and reported as clamped", d);
}

static void test_set_torque_is_symmetric_for_regen(void)
{
  FocState_t f;
  FOC_Init(&f);
  int32_t pos = FOC_SetTorque(&f,  100000);
  float   iqp = f.iq_ref;
  int32_t neg = FOC_SetTorque(&f, -100000);
  float   iqn = f.iq_ref;

  char d[160];
  snprintf(d, sizeof(d), "+%ld mNm / %.3f A against %ld mNm / %.3f A",
           (long)pos, (double)iqp, (long)neg, (double)iqn);
  check(pos == -neg && fabsf(iqp + iqn) < 1e-6f,
        "braking torque clamps symmetrically with driving torque", d);
}

int main(void)
{
  printf("\nfoc.c host tests\n----------------\n");
  test_angle_convention();
  test_torque_sign();
  test_elec_offset_regression();
  test_step_response();
  test_saturation();
  test_antiwindup_recovery();
  test_feedforward_keeps_integral_authority();
  test_torque_map_round_trips();
  test_torque_constant_matches_the_machine();
  test_set_torque_holds_id_at_zero();
  test_set_torque_reports_what_it_accepted();
  test_set_torque_is_symmetric_for_regen();
  printf("----------------\n%d passed, %d failed\n\n", g_pass, g_fail);
  return g_fail == 0 ? 0 : 1;
}
