/**
  ******************************************************************************
  * @file    foc.h
  * @brief   Field-oriented current control.
  *
  *          Runs from the HRTIM period ISR at the PWM rate, so every step -
  *          current sample, transform, PI, duty update - happens at the same
  *          point in every switching period.
  *
  *          Only U and W are instrumented; V is reconstructed as -(iu+iw),
  *          which holds because there is no neutral connection.
  *
  *          Electrical angle comes straight from the encoder with no software
  *          offset: the A1333's ZERO_OFFSET is programmed so encoder zero IS
  *          electrical zero. See HARDWARE_NOTES section 8.
  ******************************************************************************
  */
#ifndef FOC_H
#define FOC_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#define FOC_POLE_PAIRS      20U
#define FOC_ENC_COUNTS      32768U

/* Current control gains, sized from the plant rather than guessed.
 *
 * Duty is normalised, so the plant from duty deviation u to phase current is
 *     I/u = Vbus / (R + sL)
 * with Vbus = 15.55 V (measured at the supply), R = 85 mohm, L = 54.3 uH.
 *
 * Placing the closed-loop bandwidth at 1 kHz with pole-zero cancellation:
 *     kp = w * L / Vbus = 6283 * 54.3e-6 / 15.55 = 0.022
 *     ki = w * R / Vbus = 6283 * 0.085  / 15.55 = 34
 *
 * The first attempt used kp = 0.08, which meant a 1 A error commanded 1.49 V
 * across an 85 mohm winding - a demand for 17 A. Loop gain of ~17 oscillates
 * no matter how clean the current feedback is.
 *
 * Note where Vbus sits: in the DENOMINATOR of both gains. It is not a
 * calibration detail, it is the plant gain. Raise the supply to 31 V without
 * touching these numbers and every duty the loop commands delivers twice the
 * current it was sized for - the loop gain doubles, the phase margin goes with
 * it, and a controller tuned to be critically damped at 15.5 V rings or
 * oscillates. This is exactly the mistake the kp = 0.08 attempt above made,
 * arrived at from the other direction.
 *
 * So these two are only the values for FOC_VBUS_NOM_MV. The live gains are
 * recomputed from the measured bus by FOC_SetGainsForVbus - see below. */
#define FOC_KP_DEFAULT      0.022f
#define FOC_KI_DEFAULT      34.0f

/* Plant parameters the gains are derived from, kept explicit so the bus-
 * voltage rescale can recompute rather than merely scale. */
#define FOC_BW_RADS         6283.0f    /* 1 kHz target bandwidth       */
#define FOC_R_OHM           0.085f     /* phase resistance             */
#define FOC_L_H             54.3e-6f   /* phase inductance             */
#define FOC_VBUS_NOM_MV     15550      /* bus the defaults were sized at */

/* d- and q-axis inductances, for the decoupling terms.
 *
 * Both are FOC_L_H. The bench motor is an EaglePower 8309, a surface-magnet
 * outrunner, so the magnets sit in the airgap and the rotor presents the same
 * reluctance whichever way it is pointing - Ld = Lq, and there is no
 * reluctance torque to chase. Named separately anyway because the decoupling
 * equations are written in terms of both, and because the EMRAX 228 is also
 * surface-PM (axial flux) so the same equality carries over rather than being
 * a bench-only simplification that has to be revisited.
 *
 * If a motor with saliency is ever fitted, these are the two numbers to
 * measure, and MTPA stops being id = 0 - see the note in phase 5. */
#define FOC_LD_H            FOC_L_H
#define FOC_LQ_H            FOC_L_H

/* Rotor flux linkage, Wb (peak, per phase). Derived from the nameplate Kv.
 *
 * The motor is an EaglePower 8309 KV90, and FOC_POLE_PAIRS = 20.
 *
 * Kv is no-load rpm per volt of DC bus. At no load the drive has to produce a
 * phase-voltage amplitude equal to the back-EMF amplitude w_e * lambda_m, and
 * the largest amplitude a bus Vdc can deliver with the min/max injection this
 * firmware already does is Vdc/sqrt(3). (The same relation falls out of the
 * six-step view: line-to-line peak reaches Vdc, and for a sinusoidal machine
 * E_ll_peak = sqrt(3) * w_e * lambda_m.) So
 *
 *     w_e * lambda_m = Vdc / sqrt(3),   w_e = p * N * 2*pi/60,   N = Kv * Vdc
 *     => lambda_m = 60 / (2*pi*sqrt(3)*p*Kv)
 *                 = 60 / (2*pi*1.7320508*20*90)
 *                 = 3.063 mWb
 *
 * That gives Kt = 1.5*p*lambda_m = 0.092 N.m per amp of iq.
 *
 * Kv figures are a well-known source of factor-of-sqrt(3) errors, so this was
 * checked against something the bench already does rather than trusted. At
 * LIM_VEL_MAX_DPS (3600 deg/s = 600 rpm = 200 Hz electrical) the predicted
 * back-EMF amplitude is w_e*lambda_m = 3.849 V, and the modulation ceiling
 * allows FOC_VMAX_DEFAULT * 15.55 V = 3.888 V. The ratio is 0.99: this motor
 * runs out of voltage within 1% of exactly where it is observed to. A
 * sqrt(3) error either way would have put that ratio at 0.57 or 1.71 and the
 * bench would top out somewhere else entirely.
 *
 * Worth seeing what that means for phase 1b: at the top of the bench's range
 * the back-EMF term alone is w_e*lambda_m/Vbus = 0.248 of normalised duty,
 * against a vmax of 0.25. Essentially the whole voltage budget is spent
 * opposing back-EMF, and without feedforward the integrator is what has to
 * find it - from zero, every time. */
#define FOC_LAMBDA_M_WB     3.063e-3f

/* Sanity window for the measured bus. Outside it the reading is not trusted
 * and the gains are left alone: a bus of 0 would otherwise divide to infinity
 * and put NaN into the duty registers, which is the single worst thing that
 * can reach a motor bridge. */
#define FOC_VBUS_MIN_MV     6000
#define FOC_VBUS_MAX_MV     60000

/* Modulation ceiling, as a fraction of the available bus voltage.
 *
 * This caps how much voltage the loop may apply, which in turn caps speed:
 * back-EMF rises with rpm and the loop saturates once it can no longer push
 * current against it. It does NOT set the current - the PI does that - so
 * raising it is safe as long as the current loop is behaving.
 * 0.10 limited the motor to ~68 rpm. */
#define FOC_VMAX_DEFAULT    0.25f

/* Electrical offset in encoder counts (32768 = 360 deg electrical).
 *
 * Should be ZERO. openloop.c now builds phases as cos(theta), matching foc.c's
 * inverse Clarke (vu = valpha), so "electrical zero" means the same thing in
 * both modules - and the A1333 ZERO_OFFSET was recalibrated against a
 * FOC-convention vector, putting the alignment in the sensor itself.
 *
 * It was 8192 (90 deg) before that: openloop used sin(theta) and therefore
 * held the rotor 90 degrees away during the original calibration, so FOC's iq
 * produced pure d-axis force and the motor would not turn at any current.
 * Kept as a tunable in case the convention ever drifts again. */
#define FOC_ELEC_OFFSET_DEFAULT   0

typedef struct {
  /* Commands */
  float    id_ref;        /* A, normally 0 for a surface-magnet motor */
  float    iq_ref;        /* A, torque-producing                      */

  /* Measured */
  float    iu, iv, iw;    /* A                                        */
  float    ialpha, ibeta;
  float    id, iq;

  /* Output */
  float    vd, vq;
  float    valpha, vbeta;
  float    duty_u, duty_v, duty_w;   /* 0..1 */

  /* PI state */
  float    id_integ, iq_integ;
  float    kp, ki, vmax;

  /* Angle */
  /* Electrical offset in encoder counts (32768 = 360 deg electrical).
   * The A1333 ZERO_OFFSET aligns the encoder to whatever vector was applied
   * during calibration, but openloop.c builds phases as sin(theta) while FOC's
   * inverse Clarke uses vu = valpha (a cosine convention). Those differ by 90
   * degrees, so a correction belongs here until the calibration is redone
   * against a FOC-convention vector. 8192 counts = 90 deg electrical. */
  int32_t  elec_offset;

  uint16_t enc_raw;
  uint16_t elec_counts;   /* 0..32767 electrical                      */
  float    sin_e, cos_e;

  /* Diagnostics */
  uint32_t updates;
  uint32_t isr_cycles;    /* DWT cycles for the last ISR              */
  uint32_t isr_max;       /* worst case seen                          */
  uint32_t enabled;

  /* Integer mirrors - the SWD reader has no float support. */
  int32_t  id_ma, iq_ma;
  int32_t  iu_ma, iw_ma;
  int32_t  vd_mv, vq_mv;         /* as per-mille of bus, x1000 */
  int32_t  duty_u_pm, duty_v_pm, duty_w_pm;
  int32_t  iq_ref_ma, id_ref_ma;
  int32_t  elec_deg_x10;      /* electrical angle, tenths of a degree */
  int32_t  vmax_pm;           /* modulation ceiling, per-mille        */
  int32_t  vmag_pm;           /* applied vector magnitude, per-mille  */

  /* Bus-voltage gain tracking. Appended at the END of the struct on purpose:
   * tools/foc_dash.sh addresses the mirrors above by fixed byte offset. */
  uint32_t vbus_track;        /* 1 = rescale kp/ki from the measured bus */
  int32_t  vbus_used_mv;      /* bus the live gains were computed for    */
  int32_t  kp_x10000;         /* live kp, for SWD readout                */
  int32_t  ki_x100;           /* live ki, for SWD readout                */

  /* Decimation counter for the integer mirrors below the control maths.
   * Appended at the END so the fixed byte offsets the tools use do not move. */
  uint32_t mirror_div;

  /* Transport-delay compensation. Appended at the END for the same reason. */
  float    omega_e;           /* electrical rad/s, from the position loop  */
  uint32_t delay_comp;        /* 1 = advance the inverse Park (default on) */
  int32_t  omega_e_rads_x10;  /* omega_e for SWD readout, tenths of rad/s  */
  int32_t  theta_adv_deg_x10; /* applied advance, tenths of a degree       */

  /* Cross-coupling decoupling and back-EMF feedforward. At the END as ever. */
  uint32_t decouple;          /* 1 = apply the feedforward (default on)    */
  float    inv_vbus;          /* 1/Vbus, volts^-1. See FOC_SetGainsForVbus */
  int32_t  vd_ff_pm;          /* d-axis feedforward, per-mille of bus      */
  int32_t  vq_ff_pm;          /* q-axis feedforward, per-mille of bus      */
} FocState_t;

void FOC_Init(FocState_t *f);

/* Recompute kp/ki for the measured bus voltage, holding the closed-loop
 * bandwidth at FOC_BW_RADS regardless of what the supply is set to.
 *
 * Cheap enough to call from the main loop every time the bus is sampled; it
 * does nothing unless f->vbus_track is set and the reading is inside the
 * sanity window. Safe to call while the loop is running - kp and ki are read
 * afresh each ISR, and a single-word float store cannot tear on this core. */
void FOC_SetGainsForVbus(FocState_t *f, int32_t vbus_mv);

/* One control step. iu_ma / iw_ma are signed milliamps, enc_raw is the 15-bit
 * encoder reading. Writes the resulting duties into the state; the caller
 * applies them.
 *
 * vel_mech_rads is the rotor's MECHANICAL velocity in rad/s - hand it
 * g_pos.vel_rads, the estimate the position loop already maintains. It is a
 * parameter rather than a second estimator on purpose: two velocity estimates
 * that can disagree are worse than one that is imperfect, and this one is
 * differentiated from the same encoder counts the angle comes from. foc.c
 * converts to electrical; pole pairs are its business, not the position
 * loop's. */
void FOC_Update(FocState_t *f, int32_t iu_ma, int32_t iw_ma, uint16_t enc_raw,
                float vel_mech_rads);

/* Zero the integrators - call whenever the loop is (re)enabled so stale
 * windup cannot kick the first output. */
void FOC_Reset(FocState_t *f);

#ifdef __cplusplus
}
#endif

#endif /* FOC_H */
