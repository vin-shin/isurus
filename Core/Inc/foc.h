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
 * no matter how clean the current feedback is. */
#define FOC_KP_DEFAULT      0.022f
#define FOC_KI_DEFAULT      34.0f

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
} FocState_t;

void FOC_Init(FocState_t *f);

/* One control step. iu_ma / iw_ma are signed milliamps, enc_raw is the 15-bit
 * encoder reading. Writes the resulting duties into the state; the caller
 * applies them. */
void FOC_Update(FocState_t *f, int32_t iu_ma, int32_t iw_ma, uint16_t enc_raw);

/* Zero the integrators - call whenever the loop is (re)enabled so stale
 * windup cannot kick the first output. */
void FOC_Reset(FocState_t *f);

#ifdef __cplusplus
}
#endif

#endif /* FOC_H */
