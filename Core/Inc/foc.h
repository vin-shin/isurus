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

/* Current control gains. Plant is a phase winding: R = 85 mohm, L = 54.3 uH,
 * so the electrical time constant is L/R = 639 us - about 13 PWM periods at
 * 20 kHz. Start conservative; these are tuned on hardware. */
#define FOC_KP_DEFAULT      0.08f
#define FOC_KI_DEFAULT      60.0f

/* Modulation ceiling, as a fraction of the available bus voltage. */
#define FOC_VMAX_DEFAULT    0.10f

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
