/**
  ******************************************************************************
  * @file    led_pwm.h
  * @brief   Brightness control for the debug LED on PB1, via TIM3_CH4 (AF2).
  *
  *          PB1 is the same pin MX_GPIO_Init sets up as a plain push-pull
  *          output, so LedPwm_Init() must run *after* MX_GPIO_Init() — it
  *          reconfigures the pin to alternate function.
  ******************************************************************************
  */
#ifndef LED_PWM_H
#define LED_PWM_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* 12-bit duty resolution at ~1 kHz: 128 MHz / (30+1) / 4096 = 1008 Hz, well
 * above anything the eye resolves as flicker. */
#define LED_PWM_ARR     4095U
#define LED_PWM_PSC     30U

void LedPwm_Init(void);

/* Raw duty, 0..LED_PWM_ARR+1 */
void LedPwm_SetDuty(uint32_t duty);

/* Convert a *perceived* brightness (0..scale) into the PWM duty that actually
 * looks that bright, via a CIE 1931 lightness curve. Duty is proportional to
 * luminance, but the eye responds to lightness, so a linear duty ramp appears
 * to saturate almost immediately. */
uint32_t LedPwm_PerceivedToDuty(uint32_t perceived, uint32_t scale);

/* Map encoder angle (hundredths of a degree, 0..35999) onto brightness:
 * 0 deg = off, 360 deg = full, perceptually linear.
 * Returns the duty it applied. */
uint32_t LedPwm_SetFromAngle(uint32_t deg_x100);

#ifdef __cplusplus
}
#endif

#endif /* LED_PWM_H */
