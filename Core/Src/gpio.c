/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    gpio.c
  * @brief   This file provides code for the configuration
  *          of all used GPIO pins.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "gpio.h"
#include "board.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/*----------------------------------------------------------------------------*/
/* Configure GPIO                                                             */
/*----------------------------------------------------------------------------*/
/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/** Configure pins as
        * Analog
        * Input
        * Output
        * EVENT_OUT
        * EXTI
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /* Retargeted to the Mako Desori.
   *
   * The generated version drove PC13/14/15, PC0/1/2, PA2, PA4, PB1/2/9 and
   * PD2 as push-pull OUTPUTS, because that is what they were on Mako Longfin.
   * On this board that list is actively wrong and one entry is destructive:
   *
   *   PA2      DC LINK CURRENT INPUT. Driving it push-pull puts the MCU in
   *            contention with the current sensor's output stage.
   *   PC0 PC1  LPUART1. Reclaimed later by MX_LPUART1_UART_Init, so this only
   *            happened to be harmless because of init ordering.
   *   PB1      ADC3 input.
   *   the rest  unused, or unexplained inputs.
   *
   * What this board actually needs from here is one output - the encoder chip
   * select - plus leaving the analogue pins alone so the ADC MSPs can claim
   * them. The gate driver enable on PC8 is deliberately NOT here: it is set
   * up by MotorPwm_GateInit, which drives it to the disabled state before
   * making it an output, and it must stay the single owner of that pin.
   *
   * The twelve pins the .ioc configures as plain inputs and does not explain
   * (PA8 PA9 PA12, PB0 PB2 PB4 PB10 PB11 PB14 PB15, PC9, PD2) are left in
   * their reset state rather than configured here. Several are probably
   * gate-driver fault outputs; giving them pulls or modes before knowing
   * which would be guessing at hardware. See board.h section 9.
   */

  /*Configure GPIO pin Output Level : encoder CS idle high */
  HAL_GPIO_WritePin(BOARD_ENC_XDIR_PORT, BOARD_ENC_XDIR_PIN, GPIO_PIN_SET);

  /*Configure GPIO pin : PA15 - encoder chip select */
  GPIO_InitStruct.Pin = BOARD_ENC_XDIR_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(BOARD_ENC_XDIR_PORT, &GPIO_InitStruct);


}

/* USER CODE BEGIN 2 */

/* USER CODE END 2 */
