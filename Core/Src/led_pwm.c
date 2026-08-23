/**
  ******************************************************************************
  * @file    led_pwm.c
  * @brief   Brightness control for the debug LED on PB1, via TIM3_CH4 (AF2).
  *
  *          Driven with direct register writes rather than HAL_TIM on purpose.
  *          CubeMX never enabled TIM for this project, so the HAL TIM driver
  *          was never copied into Drivers/, and no local copy matches this
  *          project's HAL release (V1.2.6) closely enough to mix in safely.
  *          A PWM channel is ~10 register writes, so pulling in a mismatched
  *          vendor driver would be the riskier option.
  ******************************************************************************
  */

#include "led_pwm.h"
#include "main.h"

/* CIE 1931 lightness curve, sampled at 65 evenly spaced perceived levels.
 *
 * PWM duty is proportional to *luminance*, but the eye responds to *lightness*,
 * so a linear duty ramp looks like it saturates almost immediately. This table
 * inverts that: index i is a perceived brightness of i/64, and the value is the
 * duty that produces it.
 *
 *     Y = ((L* + 16) / 116)^3   for L* > 8
 *     Y = L* / 903.3            otherwise      where L* = 100 * i/64
 *
 * Note how uneven the steps are - 7 counts at the bottom versus 163 at the top.
 * That ratio is exactly the problem being corrected. */
static const uint16_t s_cie_lut[65] = {
       0,    7,   14,   21,   28,   35,   43,   51,
      61,   71,   83,   96,  110,  126,  143,  161,
     181,  202,  225,  250,  277,  305,  335,  368,
     402,  438,  476,  517,  560,  605,  652,  702,
     754,  809,  867,  927,  990, 1055, 1124, 1195,
    1269, 1347, 1427, 1511, 1597, 1687, 1781, 1877,
    1977, 2081, 2188, 2299, 2414, 2532, 2654, 2780,
    2909, 3043, 3181, 3323, 3469, 3619, 3774, 3933,
    4096,
};

void LedPwm_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_TIM3_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /* TIM3 sits on APB1. With APB1CLKDivider = DIV1 the timer clock equals
   * HCLK, so 128 MHz for the current SystemClock_Config():
   *   128 MHz / (30 + 1) / 4096 = 1008 Hz, far above visible flicker. */
  TIM3->CR1  = 0;
  TIM3->PSC  = LED_PWM_PSC;
  TIM3->ARR  = LED_PWM_ARR;
  TIM3->CCR4 = 0;

  /* Channel 4: PWM mode 1 (OC4M = 110) with preload on the compare register,
   * so duty updates take effect at the next update event instead of glitching
   * mid-period. */
  TIM3->CCMR2 &= ~(TIM_CCMR2_OC4M | TIM_CCMR2_CC4S);
  TIM3->CCMR2 |= TIM_CCMR2_OC4M_2 | TIM_CCMR2_OC4M_1 | TIM_CCMR2_OC4PE;

  TIM3->CCER |= TIM_CCER_CC4E;

  /* Buffer ARR too, then force an update so PSC/ARR load before the counter
   * starts. */
  TIM3->CR1 |= TIM_CR1_ARPE;
  TIM3->EGR  = TIM_EGR_UG;
  TIM3->CR1 |= TIM_CR1_CEN;

  /* Takes PB1 over from the plain push-pull output MX_GPIO_Init configured. */
  GPIO_InitStruct.Pin       = GPIO_PIN_1;
  GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull      = GPIO_NOPULL;
  GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

void LedPwm_SetDuty(uint32_t duty)
{
  if (duty > (LED_PWM_ARR + 1U))
  {
    duty = LED_PWM_ARR + 1U;
  }
  TIM3->CCR4 = duty;
}

uint32_t LedPwm_PerceivedToDuty(uint32_t perceived, uint32_t scale)
{
  if (scale == 0U)
  {
    return 0U;
  }
  if (perceived >= scale)
  {
    return LED_PWM_ARR + 1U;
  }

  /* Table position, keeping the remainder so we can interpolate between
   * entries instead of stepping in 64 visible jumps. All intermediates stay
   * inside a uint32_t: perceived < 36000, so perceived * 64 < 2.31e6, and the
   * largest (d1 - d0) * rem is about 163 * 36000 = 5.9e6. */
  uint32_t idx = (perceived * 64U) / scale;
  uint32_t rem = (perceived * 64U) % scale;

  uint32_t d0 = s_cie_lut[idx];
  uint32_t d1 = s_cie_lut[idx + 1U];

  return d0 + (((d1 - d0) * rem) / scale);
}

uint32_t LedPwm_SetFromAngle(uint32_t deg_x100)
{
  if (deg_x100 > 35999U)
  {
    deg_x100 = 35999U;
  }

  /* Angle is the *perceived* brightness we want; the LUT converts it to the
   * duty that actually looks that bright. 0 deg = off, 360 deg = full. */
  uint32_t duty = LedPwm_PerceivedToDuty(deg_x100, 36000U);

  LedPwm_SetDuty(duty);
  return duty;
}
