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
 * stationary winding. */
#define IDENT_R_TARGET_A     2.0f

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
 * so it cannot ring against a winding it was not tuned for. */
#define IDENT_R_KI           3.0e-7f

/* There is deliberately NO settle between reaching the target current and
 * starting to average. One was added when this used a fixed voltage ramp,
 * where it was necessary - the ramp outran the winding and the average began
 * mid-transient. Replacing the ramp with a regulator made it redundant: the
 * current is held at the target for the whole phase, so there is no transient
 * to wait out. Removed after measuring, not assumed - with and without it the
 * recovered resistance was identical on all three test machines, and no test
 * could be written that failed without it. */

#define IDENT_R_AVG_TICKS    3000U    /* 100 ms at 30 kHz */
#define IDENT_R_MAX_TICKS    150000U  /* 5 s - the regulator is slow on purpose */

/* Injection amplitude for the inductance measurement, per-unit of bus.
 * 0.05 of a 22 V bus is 1.1 V, which across 54 uH for one 33 us tick is about
 * 0.7 A of ripple - well clear of the noise floor and well inside every
 * limit. */
#define IDENT_L_V_PU         0.05f
/* The R phase leaves the winding carrying IDENT_R_TARGET_A, and that current
 * decays with L/R once the voltage goes to zero. Measure the ripple before it
 * has gone and the peak-to-peak reads the DECAY rather than the injection.
 * Sized at several time constants of the slowest expected machine. */
#define IDENT_L_SETTLE_TICKS 3000U
#define IDENT_L_MEAS_TICKS   600U

/* Abort if the current ever leaves this band, A. Neither phase should come
 * close; if one does, something is not what this routine assumed. */
#define IDENT_I_ABORT_A      6.0f

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
