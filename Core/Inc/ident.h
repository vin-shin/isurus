/**
  ******************************************************************************
  * @file    ident.h
  * @brief   Stationary measurement of phase resistance and inductance.
  *
  *          FOC_R_OHM, FOC_L_H and FOC_LAMBDA_M_WB are three compile-time
  *          constants that everything else keys off: the current-loop gains,
  *          the decoupling feedforward, the torque map and field weakening all
  *          derive from them. Nothing in this firmware has ever measured them.
  *
  *          That is survivable at 48 V and is not at 600 V. The w_e*Lq*iq
  *          coupling term is 441 V of a 600 V bus at 917 Hz and 300 A, so a
  *          20% error in Lq leaves 88 V uncancelled - see the derivation in
  *          foc.h. This is the routine that turns those constants into
  *          measurements.
  *
  *  ---- why both phases keep the rotor still ------------------------------
  *
  *          Everything here injects on the D axis only, at whatever electrical
  *          angle the rotor happens to sit at. d-axis current produces no
  *          torque on a surface-magnet machine - that is the same fact that
  *          makes MTPA collapse to id = 0 - so the shaft does not turn and no
  *          load, brake or dyno is needed. It also means these run before the
  *          machine has ever been spun, which is when you most want them.
  *
  *          lambda_m is NOT measurable this way: it only appears in the
  *          back-EMF, which requires motion. That one needs a coast-down and
  *          is deliberately not attempted here.
  *
  *  ---- resistance --------------------------------------------------------
  *
  *          Ramp vd until |id| reaches IDENT_R_TARGET_A, hold, and average
  *          R = vd*Vbus / id over IDENT_R_AVG_TICKS.
  *
  *          Ramped rather than applied as a step because R is the unknown: a
  *          fixed voltage chosen for the R we expect produces V/R amps if that
  *          expectation is wrong, and with no hardware overcurrent path
  *          (HARDWARE_NOTES section 10) an optimistic guess is the whole fault
  *          budget. Ramping means the current arrives at the target from
  *          below, whatever R turns out to be.
  *
  *  ---- inductance --------------------------------------------------------
  *
  *          Alternate vd between +IDENT_L_V and -IDENT_L_V on every control
  *          tick and measure the peak-to-peak current ripple. Over one tick
  *          di/dt is (V - R*i)/L, and with the mean current held near zero by
  *          the alternation the R*i term is negligible, so
  *
  *              L = V * Ts / di_pp
  *
  *          Reversing every tick is also what makes this safe: the current
  *          cannot run away because the voltage that drives it changes sign
  *          before it can, which is a stronger guarantee than a current limit
  *          polled at 750 Hz.
  *
  *  ---- what the bench actually gave, 2026-08-25 --------------------------
  *
  *          Three consecutive runs on the 20-pole-pair bench motor at 28.4 V:
  *
  *              R   120, 120, 120 mOhm   against FOC_R_OHM  85 mOhm
  *              L    43,  42,  43 uH     against FOC_L_H  54.3 uH
  *
  *          Repeatable to the last digit on R and to +/-1 uH on L, so this is
  *          a real instrument. Whether it is an ACCURATE one is a separate
  *          question, and for R the answer is already no:
  *
  *          **R IS A TERMINAL MEASUREMENT, NOT A WINDING ONE.** V/I at the
  *          bridge includes the FET Rds(on) in the path and, dominantly, the
  *          deadtime volt-second error - 185 ns at 30 kHz is 0.55% of the bus,
  *          which at 2 A appears as tens of milliohms of resistance that is
  *          not in the winding. That is most of the gap to 85 mOhm, and it is
  *          why nothing here overwrites FOC_R_OHM.
  *
  *          The separation is straightforward and not yet done: the deadtime
  *          error is a fixed VOLTAGE while the winding drop is proportional to
  *          current, so measuring at two currents solves both at once -
  *
  *              V(I) = V_deadtime + R*I
  *              R = (V2 - V1) / (I2 - I1)
  *
  *          which also yields the deadtime term that f->dtc_pm exists to
  *          compensate and that has never been measured directly.
  *
  *          L's 43 uH against 54.3 is not explained by deadtime, which would
  *          bias it the other way, and is left open. Note the 54.3 figure it
  *          disagrees with has itself never been measured on this machine.
  ******************************************************************************
  */
#ifndef IDENT_H
#define IDENT_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "motor_pwm.h"   /* PWM_FREQ_HZ */
#include "foc.h"        /* FOC_R_OHM, FOC_L_H, FOC_VBUS_NOM_MV */

typedef enum {
  IDENT_IDLE = 0U,
  IDENT_R    = 1U,       /* ramping, then averaging, resistance   */
  IDENT_L    = 2U,       /* square-wave injection, inductance     */
  IDENT_DONE = 3U,
  IDENT_FAIL = 4U
} IdentPhase_t;

/* Target current for the resistance measurement, A. Large enough that the
 * sense noise floor (about 20 mA rms on this board) is a small fraction of it,
 * small enough to sit far inside LIM_IQ_MAX_MA and to dissipate little in a
 * stationary winding.
 *
 * Raised from 2 A for the EMRAX, because what this phase actually has to
 * resolve is the VOLTAGE across the winding, and 2 A across 23.22 mOhm is
 * 46 mV where the bench machine's 85 mOhm gave 170 mV. The sense is coarser
 * here as well - roughly +/-82 A against +/-40 A - so the same target would
 * have been about seven times worse off for signal-to-noise.
 *
 * 10 A restores 232 mV, better than the bench ever had, dissipates 2.3 W in a
 * stationary EMRAX winding, and sits at 15% of LIM_IQ_MAX_MA. */
#define IDENT_R_TARGET_A     10.0f

/* Integral gain for the current regulator that holds id at the target while R
 * is measured, per-unit volts per amp of error per tick.
 *
 * A FIXED RAMP DOES NOT WORK HERE, which took a measurement to establish. The
 * ramp climbs at a rate set in volts while the current follows it with the
 * winding's own L/R lag, and that lag varies by 17x across the machines this
 * has to serve - 0.6 ms on the bench motor, 11 ms on the EMRAX 228. Sized for
 * the bench it overshoots the slow machine badly: the EMRAX ended the ramp at
 * 5.97 A against a 2 A target and tripped the abort. R still read correctly,
 * because V/I is V/I whatever I is, but the current was three times intended
 * on a board with no hardware overcurrent path.
 *
 * Regulating instead is self-limiting: the voltage stops climbing when the
 * current arrives, and backs off if it overshoots. The gain is deliberately
 * slow enough that the loop is far below the L/R pole of the slowest machine,
 * so it cannot ring against a winding it was not tuned for.
 *
 * !! The gain is in PER-UNIT volts, so it does not survive a change of bus on
 * its own. !! 3.0e-7 was tuned against a 15.55 V bench supply; the same
 * number on a 518 V pack is 33x the actual volts per amp of error, into a
 * winding whose L/R pole is only 4.7x slower. That is a regulator handed 33x
 * its designed loop gain, which is how a loop that was deliberately
 * over-damped starts ringing.
 *
 * So it is derived from the physical gain that WAS validated - 0.14 volts per
 * amp of error per second - and converted through the actual bus and tick
 * rate. Any future board gets the same dynamics without anyone re-tuning. */
#define IDENT_R_KI_V_PER_A_S 0.14f
#define IDENT_R_KI           (IDENT_R_KI_V_PER_A_S / (float)PWM_FREQ_HZ                               / ((float)FOC_VBUS_NOM_MV * 1.0e-3f))

/* There is deliberately NO settle between reaching the target current and
 * starting to average. One was added when this used a fixed voltage ramp,
 * where it was necessary - the ramp outran the winding and the average began
 * mid-transient. Replacing the ramp with a regulator made it redundant: the
 * current is held at the target for the whole phase, so there is no transient
 * to wait out. Removed after measuring, not assumed - with and without it the
 * recovered resistance was identical on all three test machines, and no test
 * could be written that failed without it. */

/* Derived from the rate rather than written in ticks, so the averaging window
 * and the timeout stay 100 ms and 5 s whatever the switching frequency is.
 * As literals these were 100 ms and 5 s at 30 kHz and became 150 ms and 7.5 s
 * at 20 kHz - harmless here, but only by luck. */
#define IDENT_R_AVG_TICKS    (PWM_FREQ_HZ / 10U)    /* 100 ms */
/* 10 s, up from 5.
 *
 * The regulator's settling time is R / (volts per amp per second), so it
 * scales with the winding being measured: 0.17 s for the EMRAX's 23 mOhm and
 * 1.3 s for a 180 mOhm machine, which needs about 5.1 s to reach 98% of
 * target and so just missed the old budget. Raising the timeout rather than
 * the gain, because the gain is the one that was validated against real
 * hardware and the one that can ring. Nothing is dissipated by waiting: the
 * timeout only runs long when the current has NOT arrived. */
#define IDENT_R_MAX_TICKS    (PWM_FREQ_HZ * 10U)    /* 10 s   */

/* Injection amplitude for the inductance measurement, per-unit of bus.
 *
 * !! Per-unit again, and this one was actively dangerous on the new board. !!
 * 0.05 pu of a 15.55 V bench bus across 54.3 uH for one 33 us tick is 0.48 A
 * of ripple. The same 0.05 pu of a 518 V pack across 255 uH for one 50 us
 * tick is 5.08 A - not a proportional increase, a tenfold one, because the
 * bus rose 33x while the winding only got 4.7x more inductive. It lands just
 * under IDENT_I_ABORT_A and the identification aborts, which is what the host
 * tests showed: L came back as 0 uH and the phase as FAIL.
 *
 * Aborting is the good outcome. Had the abort band been a little wider this
 * would have injected several amps of square wave into a traction machine
 * because a constant expressed as a fraction went unexamined.
 *
 * Expressed as a target RIPPLE now, and converted to per-unit through the bus
 * and the winding. FOC_L_H is the starting estimate for the very quantity
 * being measured, which sounds circular and is not: it only has to be right
 * enough to choose a safe excitation, and the measurement then corrects it. */
#define IDENT_L_RIPPLE_A     1.0f
#define IDENT_L_V_PU         (IDENT_L_RIPPLE_A * FOC_L_H * (float)PWM_FREQ_HZ                               / ((float)FOC_VBUS_NOM_MV * 1.0e-3f))
/* The R phase leaves the winding carrying IDENT_R_TARGET_A, and that current
 * decays with L/R once the voltage goes to zero. Measure the ripple before it
 * has gone and the peak-to-peak reads the DECAY rather than the injection.
 * Sized at several time constants of the slowest expected machine. */
#define IDENT_L_SETTLE_TICKS 3000U
#define IDENT_L_MEAS_TICKS   600U

/* Abort if the current ever leaves this band, A. Neither phase should come
 * close; if one does, something is not what this routine assumed.
 *
 * 20 A is 2x the R target and 20x the L ripple, and still under a third of
 * LIM_IQ_MAX_MA. It was 6 A against a 2 A target - the same 3x ratio. */
#define IDENT_I_ABORT_A      20.0f

typedef struct {
  uint32_t phase;         /*  0  IdentPhase_t                              */
  uint32_t tick;          /*  4  ticks in the current phase                */

  float    vd_cmd;        /*  8  what the ISR should apply on d, per-unit  */
  float    vq_cmd;        /* 12  always zero - q would make torque         */

  float    r_acc;         /* 16  running sum for the resistance average    */
  uint32_t r_n;           /* 20  samples in that sum                       */
  float    i_min;         /* 24  ripple tracking for the inductance        */
  float    i_max;         /* 28                                            */

  int32_t  r_mohm;        /* 32  result: phase resistance, milliohms       */
  int32_t  l_uh;          /* 36  result: phase inductance, microhenries    */
  uint32_t fail_code;     /* 40  0 = none, else see Ident_Step             */
} IdentState_t;

#define IDENT_FAIL_NONE      0U
#define IDENT_FAIL_OVERCUR   1U   /* current left the abort band           */
#define IDENT_FAIL_TIMEOUT   2U   /* ramp never reached the target         */
#define IDENT_FAIL_NO_RIPPLE 3U   /* ripple too small to divide by         */

void Ident_Init(IdentState_t *s);

/* Begin a measurement. `what` is IDENT_R to measure resistance and then go on
 * to inductance, which is the normal use. */
void Ident_Start(IdentState_t *s, uint32_t what);

/* One control tick. Give it the measured currents and the bus, and it updates
 * s->vd_cmd / s->vq_cmd for the caller to apply. Pure with respect to
 * everything else, which is what lets the host tests close it around the PMSM
 * model and check it recovers parameters the model is known to have. */
void Ident_Step(IdentState_t *s, float id_a, float iq_a, float vbus_v);

#ifdef __cplusplus
}
#endif

#endif /* IDENT_H */
