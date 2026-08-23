/**
  ******************************************************************************
  * @file    openloop.c
  * @brief   Open-loop (V/f) three-phase drive - a rotating voltage vector.
  ******************************************************************************
  */

#include "openloop.h"
#include "motor_pwm.h"
#include "main.h"

/* sin(2*pi*i/256) in Q15. Entry 256 duplicates entry 0 so interpolation can
 * read idx+1 without wrapping. Integer throughout - no FPU, so this stays
 * safe to call from an ISR when the control loop moves there. */
static const int16_t s_sin_q15[257] = {
         0,    804,   1608,   2410,   3212,   4011,   4808,   5602,
      6393,   7179,   7962,   8739,   9512,  10278,  11039,  11793,
     12539,  13279,  14010,  14732,  15446,  16151,  16846,  17530,
     18204,  18868,  19519,  20159,  20787,  21403,  22005,  22594,
     23170,  23731,  24279,  24811,  25329,  25832,  26319,  26790,
     27245,  27683,  28105,  28510,  28898,  29268,  29621,  29956,
     30273,  30571,  30852,  31113,  31356,  31580,  31785,  31971,
     32137,  32285,  32412,  32521,  32609,  32678,  32728,  32757,
     32767,  32757,  32728,  32678,  32609,  32521,  32412,  32285,
     32137,  31971,  31785,  31580,  31356,  31113,  30852,  30571,
     30273,  29956,  29621,  29268,  28898,  28510,  28105,  27683,
     27245,  26790,  26319,  25832,  25329,  24811,  24279,  23731,
     23170,  22594,  22005,  21403,  20787,  20159,  19519,  18868,
     18204,  17530,  16846,  16151,  15446,  14732,  14010,  13279,
     12539,  11793,  11039,  10278,   9512,   8739,   7962,   7179,
      6393,   5602,   4808,   4011,   3212,   2410,   1608,    804,
         0,   -804,  -1608,  -2410,  -3212,  -4011,  -4808,  -5602,
     -6393,  -7179,  -7962,  -8739,  -9512, -10278, -11039, -11793,
    -12539, -13279, -14010, -14732, -15446, -16151, -16846, -17530,
    -18204, -18868, -19519, -20159, -20787, -21403, -22005, -22594,
    -23170, -23731, -24279, -24811, -25329, -25832, -26319, -26790,
    -27245, -27683, -28105, -28510, -28898, -29268, -29621, -29956,
    -30273, -30571, -30852, -31113, -31356, -31580, -31785, -31971,
    -32137, -32285, -32412, -32521, -32609, -32678, -32728, -32757,
    -32767, -32757, -32728, -32678, -32609, -32521, -32412, -32285,
    -32137, -31971, -31785, -31580, -31356, -31113, -30852, -30571,
    -30273, -29956, -29621, -29268, -28898, -28510, -28105, -27683,
    -27245, -26790, -26319, -25832, -25329, -24811, -24279, -23731,
    -23170, -22594, -22005, -21403, -20787, -20159, -19519, -18868,
    -18204, -17530, -16846, -16151, -15446, -14732, -14010, -13279,
    -12539, -11793, -11039, -10278,  -9512,  -8739,  -7962,  -7179,
     -6393,  -5602,  -4808,  -4011,  -3212,  -2410,  -1608,   -804,
         0,
};

static uint32_t s_period = 0;
static uint32_t s_half   = 0;

/* Phase offsets for a balanced three-phase set: 0, -120, +120 degrees. */
#define OL_PHASE_120  ((uint32_t)(4294967296.0 / 3.0))

/* Interpolated sine. phase is the full uint32 electrical angle. */
static int32_t OpenLoop_Sin(uint32_t phase)
{
  uint32_t idx  = phase >> 24;                 /* 0..255            */
  uint32_t frac = (phase >> 16) & 0xFFU;       /* 0..255 between    */
  int32_t  a    = s_sin_q15[idx];
  int32_t  b    = s_sin_q15[idx + 1U];

  return a + (((b - a) * (int32_t)frac) >> 8);
}

void OpenLoop_Init(OpenLoopState_t *s, uint32_t pwm_period)
{
  s_period = pwm_period;
  s_half   = pwm_period / 2U;

  s->phase        = 0;
  s->inc          = 0;
  s->freq_x100    = 0;
  s->mod_permille = 0;
  s->duty_u       = 0;
  s->duty_v       = 0;
  s->duty_w       = 0;
  s->updates      = 0;
  s->mode         = OL_IDLE;
  s->align_left   = 0;
  s->freq_now_x100 = 0;
  s->freq_tgt_x100 = 0;
  s->ramp_x100    = 0;
}

static uint32_t OpenLoop_IncFor(uint32_t freq_x100)
{
  return (uint32_t)(((uint64_t)freq_x100 << 32) /
                    ((uint64_t)OL_UPDATE_HZ * 100ULL));
}

void OpenLoop_Start(OpenLoopState_t *s, uint32_t freq_x100,
                    uint32_t mod_permille, uint32_t align_ms,
                    uint32_t ramp_ms)
{
  if (mod_permille > OL_MOD_MAX_PERMILLE)
  {
    mod_permille = OL_MOD_MAX_PERMILLE;
  }

  s->mod_permille  = mod_permille;
  s->phase         = 0;              /* align to electrical zero */
  s->freq_now_x100 = 0;
  s->freq_tgt_x100 = freq_x100;
  s->inc           = 0;
  s->align_left    = (align_ms * OL_UPDATE_HZ) / 1000U;

  /* Frequency step per update so the ramp lands on target after ramp_ms. */
  uint32_t steps = (ramp_ms * OL_UPDATE_HZ) / 1000U;
  s->ramp_x100 = (steps > 0U) ? ((freq_x100 + steps - 1U) / steps) : freq_x100;

  s->mode = OL_ALIGN;
}

void OpenLoop_SetCommand(OpenLoopState_t *s, uint32_t freq_x100,
                         uint32_t mod_permille)
{
  if (mod_permille > OL_MOD_MAX_PERMILLE)
  {
    mod_permille = OL_MOD_MAX_PERMILLE;
  }

  s->freq_x100    = freq_x100;
  s->mod_permille = mod_permille;

  /* inc = freq * 2^32 / update_rate, in Hz*100 units. 64-bit here so the
   * shift cannot overflow; it runs once per command, not per update. */
  s->inc = (uint32_t)(((uint64_t)freq_x100 << 32) /
                      ((uint64_t)OL_UPDATE_HZ * 100ULL));
}

void OpenLoop_Update(OpenLoopState_t *s)
{
  if (s->mode == OL_ALIGN)
  {
    /* Hold the vector still at electrical zero; the rotor is being dragged to
     * a known angle. Do not advance the phase. */
    if (s->align_left > 0U) { s->align_left--; }
    if (s->align_left == 0U) { s->mode = OL_RUN; }
  }
  else if (s->mode == OL_RUN)
  {
    if (s->freq_now_x100 < s->freq_tgt_x100)
    {
      s->freq_now_x100 += s->ramp_x100;
      if (s->freq_now_x100 > s->freq_tgt_x100)
      {
        s->freq_now_x100 = s->freq_tgt_x100;
      }
    }
    s->inc = OpenLoop_IncFor(s->freq_now_x100);
    s->phase += s->inc;
  }
  else
  {
    s->phase += s->inc;   /* plain SetCommand behaviour */
  }

  /* Amplitude in counts. half * 500/1000 max = quarter period of swing each
   * way about the 50% midpoint. */
  int32_t amp = (int32_t)((s_half * s->mod_permille) / 1000U);

  int32_t su = OpenLoop_Sin(s->phase);
  int32_t sv = OpenLoop_Sin(s->phase - OL_PHASE_120);
  int32_t sw = OpenLoop_Sin(s->phase + OL_PHASE_120);

  /* amp * sin_q15 peaks around 25600 * 32767 = 8.4e8, inside an int32_t. */
  int32_t du = (int32_t)s_half + ((amp * su) >> 15);
  int32_t dv = (int32_t)s_half + ((amp * sv) >> 15);
  int32_t dw = (int32_t)s_half + ((amp * sw) >> 15);

  if (du < 0) { du = 0; }
  if (dv < 0) { dv = 0; }
  if (dw < 0) { dw = 0; }

  s->duty_u = (uint32_t)du;
  s->duty_v = (uint32_t)dv;
  s->duty_w = (uint32_t)dw;
  s->updates++;

  MotorPwm_SetDuty(s->duty_u, s->duty_v, s->duty_w);
}

void OpenLoop_Stop(OpenLoopState_t *s)
{
  s->phase        = 0;
  s->inc          = 0;
  s->freq_x100    = 0;
  s->mod_permille = 0;
  s->duty_u       = 0;
  s->duty_v       = 0;
  s->duty_w       = 0;
  s->mode         = OL_IDLE;
  s->align_left   = 0;
  s->freq_now_x100 = 0;
  s->freq_tgt_x100 = 0;

  MotorPwm_SetDuty(0, 0, 0);
}
