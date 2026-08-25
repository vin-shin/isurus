/**
  ******************************************************************************
  * @file    foc.c
  * @brief   Field-oriented current control.
  ******************************************************************************
  */

#include "foc.h"
#include "main.h"
#include "motor_pwm.h"   /* PWM_FREQ_HZ: the rate this loop actually runs at */
#include "fastmath.h"

#define FOC_DT          (1.0f / (float)PWM_FREQ_HZ)

#define ONE_BY_SQRT3    0.57735026919f
#define TWO_BY_SQRT3    1.15470053838f

/* CORDIC configuration, written once in FOC_Init and never touched again.
 *
 * FUNC = 0 is cosine; PRECISION = 5 requests 20 iterations, which the hardware
 * retires four per cycle. NRES = 2 makes one operation yield cos AND sin, so
 * the ISR pays for a single CORDIC pass rather than two.
 *
 * 20 iterations was checked against sinf/cosf across the whole circle before
 * being trusted: worst error 1.9e-6, or 0.0001 degrees of dq-frame
 * misalignment. That is four orders of magnitude below the encoder's own
 * 32768-count resolution, so it is not the limiting error anywhere in this
 * loop. Dropping to PRECISION = 4 costs one cycle less and gives 0.0018
 * degrees, which would also be fine - the extra iterations are kept because
 * they are nearly free and Phase 1a will lean on this angle harder. */
#define FOC_CORDIC_CFG  ((0U << CORDIC_CSR_FUNC_Pos)      /* cosine        */ \
                       | (5U << CORDIC_CSR_PRECISION_Pos) /* 20 iterations */ \
                       | CORDIC_CSR_NRES)                 /* cos and sin   */

/* sin/cos of an electrical angle expressed in encoder counts.
 *
 * The CORDIC's angle argument is Q31 spanning [-pi, pi): the value v means
 * v/2^31 * pi radians. elec_counts is already an integer angle - 0..32767 for
 * 0..2*pi - so the whole conversion is one shift:
 *
 *     q31 = counts << 17
 *
 * and it is exact. The 15-bit count walks into the Q31 sign bit at count
 * 16384, which is precisely where the angle passes pi and the CORDIC's range
 * wraps to -pi, so counts above a half turn come out negative and mean the
 * same angle. No float, no fmodf, no wrap test.
 *
 * Do NOT centre the count first, i.e. (counts - 16384) << 17. That form is
 * off by exactly pi and negates BOTH sin and cos. Nothing here would catch it:
 * the inverse Park uses the same pair, so the loop stays self-consistent and
 * perfectly stable - it just runs in a dq frame rotated 180 degrees, where a
 * positive iq_ref produces negative torque. It is the same failure as the
 * elec_offset = 8192 convention bug in foc.h, and just as invisible until a
 * motor turns the wrong way. */
__attribute__((always_inline))
static inline void FOC_SinCos(uint16_t counts, float *sin_out, float *cos_out)
{
  CORDIC->WDATA = (uint32_t)counts << 17;

  /* RRDY is set when the operation completes - with NRES = 2 that means both
   * results are latched - and is cleared only once RDATA has been read twice.
   * So one poll covers the pair; a second poll before the sine would spin
   * zero times and buy nothing. */
  while ((CORDIC->CSR & CORDIC_CSR_RRDY) == 0U) { }

  int32_t cos_q31 = (int32_t)CORDIC->RDATA;   /* results come out cos first */
  int32_t sin_q31 = (int32_t)CORDIC->RDATA;

  *cos_out = (float)cos_q31 * (1.0f / 2147483648.0f);
  *sin_out = (float)sin_q31 * (1.0f / 2147483648.0f);
}

void FOC_Reset(FocState_t *f)
{
  f->id_integ = 0.0f;
  f->iq_integ = 0.0f;
}

void FOC_SetGainsForVbus(FocState_t *f, int32_t vbus_mv)
{
  if (f->vbus_track == 0U) { return; }
  if ((vbus_mv < FOC_VBUS_MIN_MV) || (vbus_mv > FOC_VBUS_MAX_MV)) { return; }

  float vbus = (float)vbus_mv * 0.001f;

  /* Same derivation as the defaults, just against the bus that is actually
   * present: pole-zero cancellation places the closed loop at FOC_BW_RADS,
   * and dividing by vbus is what keeps it there as the supply moves. */
  f->kp = (FOC_BW_RADS * FOC_L_H)   / vbus;
  f->ki = (FOC_BW_RADS * FOC_R_OHM) / vbus;

  f->vbus_used_mv = vbus_mv;
  f->kp_x10000    = (int32_t)(f->kp * 10000.0f);
  f->ki_x100      = (int32_t)(f->ki * 100.0f);
}

void FOC_Init(FocState_t *f)
{
  /* Bring up the CORDIC here, not in the ISR. Everything that never varies -
   * function, precision, result count, and the magnitude argument - is set
   * once, so the control step costs one register write and two reads.
   *
   * Called from main() before MotorPwm_EnableControlIsr(), which is what makes
   * that safe: the ISR's first FOC_SinCos cannot run before this returns. */
  RCC->AHB1ENR |= RCC_AHB1ENR_CORDICEN;
  (void)RCC->AHB1ENR;   /* read back so the CSR write below cannot overtake
                         * the clock actually coming up - a peripheral write
                         * that lands before its clock is simply discarded */

  /* Seed the magnitude argument explicitly rather than inheriting whatever
   * the CORDIC happens to hold. For sin/cos it SCALES both results, so a
   * value other than 1.0 would quietly scale every current the Park transform
   * reports and every voltage the inverse Park applies - a gain error in the
   * middle of the control loop with nothing to attribute it to.
   *
   * Written once with NARGS = 2 (angle then magnitude); the CORDIC retains it,
   * so the steady-state config below drops to NARGS = 1 and the ISR supplies
   * the angle alone. */
  CORDIC->CSR   = FOC_CORDIC_CFG | CORDIC_CSR_NARGS;
  CORDIC->WDATA = 0U;             /* angle 0                     */
  CORDIC->WDATA = 0x7FFFFFFFU;    /* magnitude 1.0 in Q31        */
  while ((CORDIC->CSR & CORDIC_CSR_RRDY) == 0U) { }
  (void)CORDIC->RDATA;            /* drain both, or RRDY stays set and the */
  (void)CORDIC->RDATA;            /* first real call reads these instead   */

  CORDIC->CSR = FOC_CORDIC_CFG;

  f->id_ref  = 0.0f;
  f->iq_ref  = 0.0f;
  f->kp      = FOC_KP_DEFAULT;
  f->ki      = FOC_KI_DEFAULT;
  f->vmax    = FOC_VMAX_DEFAULT;
  f->enabled     = 0U;
  f->elec_offset = FOC_ELEC_OFFSET_DEFAULT;
  f->vbus_track   = 1U;
  f->vbus_used_mv = FOC_VBUS_NOM_MV;
  f->kp_x10000    = (int32_t)(f->kp * 10000.0f);
  f->ki_x100      = (int32_t)(f->ki * 100.0f);
  f->updates = 0U;
  f->isr_max = 0U;
  f->mirror_div = 0U;
  FOC_Reset(f);
}

void FOC_Update(FocState_t *f, int32_t iu_ma, int32_t iw_ma, uint16_t enc_raw)
{
  /* ---- currents ---------------------------------------------------- */
  f->iu = (float)iu_ma * 0.001f;
  f->iw = (float)iw_ma * 0.001f;
  /* No neutral, so the three must sum to zero. */
  f->iv = -(f->iu + f->iw);

  /* ---- Clarke: 3-phase -> stationary alpha/beta --------------------- */
  f->ialpha = f->iu;
  f->ibeta  = (f->iu + 2.0f * f->iv) * ONE_BY_SQRT3;

  /* ---- electrical angle -------------------------------------------- */
  /* Encoder zero IS electrical zero (programmed into the A1333), so this
   * needs no offset term - just the pole-pair multiply, wrapped. */
  f->enc_raw = enc_raw;
  {
    int32_t e = (int32_t)(((uint32_t)(enc_raw & 0x7FFFU) * FOC_POLE_PAIRS)
                          % FOC_ENC_COUNTS);
    e += f->elec_offset;
    e %= (int32_t)FOC_ENC_COUNTS;
    if (e < 0) { e += (int32_t)FOC_ENC_COUNTS; }
    f->elec_counts = (uint16_t)e;
  }

  /* Straight from the integer angle - see FOC_SinCos. The float radian value
   * that used to sit here existed only to be handed to sinf/cosf, and building
   * it was the cheap half of the cost. */
  FOC_SinCos(f->elec_counts, &f->sin_e, &f->cos_e);

  /* ---- Park: stationary -> rotor frame ------------------------------ */
  f->id =  f->ialpha * f->cos_e + f->ibeta * f->sin_e;
  f->iq = -f->ialpha * f->sin_e + f->ibeta * f->cos_e;

  /* ---- PI on both axes ---------------------------------------------- */
  float ed = f->id_ref - f->id;
  float eq = f->iq_ref - f->iq;

  /* dt is the PWM period, because this runs once per period. Derived from
   * PWM_FREQ_HZ rather than written out, so changing the switching frequency
   * cannot silently rescale the integrator gain. */
  f->id_integ += ed * f->ki * FOC_DT;
  f->iq_integ += eq * f->ki * FOC_DT;

  f->vd = ed * f->kp + f->id_integ;
  f->vq = eq * f->kp + f->iq_integ;

  /* Limit the vector magnitude, not each axis separately - clipping d and q
   * independently rotates the applied vector away from where it was asked
   * for. */
  float vmag = fm_sqrtf(f->vd * f->vd + f->vq * f->vq);
  if (vmag > f->vmax)
  {
    float k = f->vmax / vmag;

    /* Back-calculation anti-windup. Clamping each integrator to +/-vmax
     * separately is not enough: two axes each at the limit reach 1.41*vmax
     * combined, so they keep winding past anything the output can deliver and
     * the loop stops responding. Scaling them by the same factor that limits
     * the vector holds the integrators at exactly what is achievable. */
    f->id_integ *= k;
    f->iq_integ *= k;

    f->vd *= k;
    f->vq *= k;
  }

  /* ---- inverse Park -------------------------------------------------- */
  f->valpha = f->vd * f->cos_e - f->vq * f->sin_e;
  f->vbeta  = f->vd * f->sin_e + f->vq * f->cos_e;

  /* ---- inverse Clarke ------------------------------------------------ */
  float vu = f->valpha;
  float vv = -0.5f * f->valpha + 0.86602540378f * f->vbeta;
  float vw = -0.5f * f->valpha - 0.86602540378f * f->vbeta;

  /* Min/max (third-harmonic) injection: shifting all three by the midpoint
   * of their extremes costs nothing differentially but buys ~15% more usable
   * bus voltage before clipping. */
  float vmaxp = vu, vminp = vu;
  if (vv > vmaxp) { vmaxp = vv; }
  if (vw > vmaxp) { vmaxp = vw; }
  if (vv < vminp) { vminp = vv; }
  if (vw < vminp) { vminp = vw; }

  float voff = 0.5f * (vmaxp + vminp);
  vu -= voff;
  vv -= voff;
  vw -= voff;

  /* ---- to duty ------------------------------------------------------- */
  f->duty_u = vu + 0.5f;
  f->duty_v = vv + 0.5f;
  f->duty_w = vw + 0.5f;

  if (f->duty_u < 0.0f) { f->duty_u = 0.0f; }
  if (f->duty_v < 0.0f) { f->duty_v = 0.0f; }
  if (f->duty_w < 0.0f) { f->duty_w = 0.0f; }
  if (f->duty_u > 1.0f) { f->duty_u = 1.0f; }
  if (f->duty_v > 1.0f) { f->duty_v = 1.0f; }
  if (f->duty_w > 1.0f) { f->duty_w = 1.0f; }

  /* Integer mirrors for SWD readout - decimated to 1 kHz.
   *
   * About twenty float-to-int conversions that exist only so a debugger can
   * read this state. Nothing in the control path uses them, and no debugger
   * samples faster than a few hundred hertz, so running them every switching
   * period was spending real ISR budget on nobody's behalf. At 30 kHz that
   * budget is no longer spare. */
  if (++f->mirror_div >= (PWM_FREQ_HZ / 1000U))
  {
    f->mirror_div = 0U;
    f->id_ma     = (int32_t)(f->id * 1000.0f);
    f->iq_ma     = (int32_t)(f->iq * 1000.0f);
    f->iu_ma     = iu_ma;
    f->iw_ma     = iw_ma;
    f->vd_mv     = (int32_t)(f->vd * 1000.0f);
    f->vq_mv     = (int32_t)(f->vq * 1000.0f);
    f->duty_u_pm = (int32_t)(f->duty_u * 1000.0f);
    f->duty_v_pm = (int32_t)(f->duty_v * 1000.0f);
    f->duty_w_pm = (int32_t)(f->duty_w * 1000.0f);
    f->iq_ref_ma = (int32_t)(f->iq_ref * 1000.0f);
    f->id_ref_ma = (int32_t)(f->id_ref * 1000.0f);
    f->elec_deg_x10 = (int32_t)(((uint32_t)f->elec_counts * 3600U) / FOC_ENC_COUNTS);
    f->vmax_pm   = (int32_t)(f->vmax * 1000.0f);
    f->vmag_pm   = (int32_t)(fm_sqrtf(f->vd * f->vd + f->vq * f->vq) * 1000.0f);
  }

  f->updates++;
}
