/**
  ******************************************************************************
  * @file    foc.c
  * @brief   Field-oriented current control.
  ******************************************************************************
  */

#include "foc.h"
#include "main.h"
#include <math.h>

#define ONE_BY_SQRT3    0.57735026919f
#define TWO_BY_SQRT3    1.15470053838f

void FOC_Reset(FocState_t *f)
{
  f->id_integ = 0.0f;
  f->iq_integ = 0.0f;
}

void FOC_Init(FocState_t *f)
{
  f->id_ref  = 0.0f;
  f->iq_ref  = 0.0f;
  f->kp      = FOC_KP_DEFAULT;
  f->ki      = FOC_KI_DEFAULT;
  f->vmax    = FOC_VMAX_DEFAULT;
  f->enabled     = 0U;
  f->elec_offset = FOC_ELEC_OFFSET_DEFAULT;
  f->updates = 0U;
  f->isr_max = 0U;
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

  float theta = (float)f->elec_counts * (6.28318530718f / (float)FOC_ENC_COUNTS);
  f->sin_e = sinf(theta);
  f->cos_e = cosf(theta);

  /* ---- Park: stationary -> rotor frame ------------------------------ */
  f->id =  f->ialpha * f->cos_e + f->ibeta * f->sin_e;
  f->iq = -f->ialpha * f->sin_e + f->ibeta * f->cos_e;

  /* ---- PI on both axes ---------------------------------------------- */
  float ed = f->id_ref - f->id;
  float eq = f->iq_ref - f->iq;

  f->id_integ += ed * f->ki * (1.0f / 20000.0f);
  f->iq_integ += eq * f->ki * (1.0f / 20000.0f);

  f->vd = ed * f->kp + f->id_integ;
  f->vq = eq * f->kp + f->iq_integ;

  /* Limit the vector magnitude, not each axis separately - clipping d and q
   * independently rotates the applied vector away from where it was asked
   * for. */
  float vmag = sqrtf(f->vd * f->vd + f->vq * f->vq);
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

  /* Integer mirrors for SWD readout. */
  f->id_ma     = (int32_t)(f->id * 1000.0f);
  f->iq_ma     = (int32_t)(f->iq * 1000.0f);
  f->iu_ma     = iu_ma;
  f->iw_ma     = iw_ma;
  f->vd_mv     = (int32_t)(f->vd * 1000.0f);
  f->vq_mv     = (int32_t)(f->vq * 1000.0f);
  f->duty_u_pm = (int32_t)(f->duty_u * 1000.0f);
  f->duty_v_pm = (int32_t)(f->duty_v * 1000.0f);
  f->duty_w_pm = (int32_t)(f->duty_w * 1000.0f);

  f->updates++;
}
