/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "main.h"
#include "adc.h"
#include "fdcan.h"
#include "hrtim.h"
#include "opamp.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "encoder.h"
#include "led_pwm.h"
#include "csense.h"
#include "motor_pwm.h"
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* Encoder telemetry. Laid out as one contiguous block of 32-bit words so the
 * SWD reader can pull the whole set in a single transaction instead of six —
 * over a 1.8 MHz SWD link that is the difference between a ~6 ms and a ~1 ms
 * sample. See HARDWARE_NOTES.md section 4. */
typedef struct {
  uint32_t raw;       /* raw 15-bit count [0, 32767]           */
  uint32_t deg_x100;  /* hundredths of a degree [0, 35999]     */
  uint32_t reads;     /* successful reads                      */
  uint32_t errors;    /* SPI timeouts / failures               */
  uint32_t rate_hz;   /* measured encoder reads per second     */
  uint32_t frames;    /* rx1 in low half, rx2 in high half     */
  uint32_t duty;      /* LED PWM compare, 0..LED_PWM_ARR+1     */
} EncTelem_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* Print one UART line every N encoder reads. Printing every sample at 115200
 * would throttle the loop to ~150 Hz, since Telem_Printf blocks until the last
 * character is out. The SWD reader is unaffected either way. */
#define TELEM_PRINT_EVERY   200U

/* Heartbeat LED on PB2. PB1 is the brightness LED driven by TIM3_CH4, so a
 * separate pin is needed for a plain "still alive" blink. */
#define HEARTBEAT_PIN       GPIO_PIN_2
#define HEARTBEAT_MS        1000U

/* Sample the current sensors every N encoder reads. */
#define CS_READ_EVERY       500U

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Readable live over SWD with OpenOCD, which is the only printout path that
 * works with a bare ST-Link/V2 (it has no VCP). */
volatile EncTelem_t g_enc = {0};

/* Phase current sense telemetry, also read live over SWD. */
volatile CSenseTelem_t g_cs = {0};
volatile int32_t g_cs_init_rc = 0;

/* HRTIM three-phase PWM telemetry. Outputs stay DISABLED at boot. */
volatile MotorPwmTelem_t g_pwm = {0};
volatile int32_t g_pwm_init_rc = 0;
volatile int32_t g_cs_trig_rc = 0;

static uint32_t s_rate_t0     = 0;  /* window start for the rate counter */
static uint32_t s_rate_reads0 = 0;
static uint32_t s_print_div   = 0;
static uint32_t s_hb_t0       = 0;
static uint32_t s_cs_div      = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* Blocking printf over USART1 (PB6 TX, 115200 8N1).
 * Deliberately integer-only: newlib-nano drops float formatting unless the
 * link is given -u _printf_float, so %f would silently print nothing. */
static void Telem_Printf(const char *fmt, ...)
{
  char buf[96];
  va_list args;

  va_start(args, fmt);
  int len = vsnprintf(buf, sizeof(buf), fmt, args);
  va_end(args);

  if (len <= 0) return;
  if (len > (int)sizeof(buf) - 1) len = (int)sizeof(buf) - 1;

  HAL_UART_Transmit(&huart1, (uint8_t *)buf, (uint16_t)len, HAL_MAX_DELAY);
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  // MX_FDCAN1_Init();
  // MX_HRTIM1_Init();
  // MX_OPAMP1_Init();
  // MX_OPAMP2_Init();
  // MX_OPAMP3_Init();
  // MX_OPAMP4_Init();
  // MX_OPAMP5_Init();
  // MX_OPAMP6_Init();
  MX_SPI1_Init();
  // MX_SPI2_Init();
  // MX_SPI3_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  // __enable_irq();
  // HAL_SYSTICK_Config(SystemCoreClock / 1000U);
  Encoder_Init();
  LedPwm_Init();

  /* Current sense. Zero-offset capture happens here, so the motor must be
   * de-energised and at rest at this point - which it is: nothing drives the
   * gate outputs yet. */
  g_cs_init_rc = CSense_Init((CSenseTelem_t *)&g_cs);

  /* Three-phase PWM. Counters run so the timebase can be verified, but the
   * outputs are NOT enabled - nothing is commanded to the gate drivers. */
  g_pwm_init_rc = MotorPwm_Init();
  MotorPwm_SetDutyPermille(0, 0, 0);

  /* Now that the HRTIM ADC trigger exists, move current sensing onto it so
   * both phases are sampled at the same point in every PWM period. */
  g_cs_trig_rc = CSense_UseHrtimTrigger();

  /* Re-zero on the triggered path. The offset captured during software-start
   * sampling is a fraction of an LSB off once conversions are locked to a
   * fixed point in the PWM period. Still safe to do: outputs are disabled and
   * the gate drivers are held off, so no current can be flowing. */
  if (g_cs_trig_rc == 0)
  {
    (void)CSense_CalibrateZero((CSenseTelem_t *)&g_cs);
  }
  Telem_Printf("\r\n--- makolongfin2 encoder (A1333 on SPI1) ---\r\n");
  Telem_Printf("SystemCoreClock = %lu Hz\r\n", (unsigned long)SystemCoreClock);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    /* Sample the encoder flat out — no delay. Everything else in this loop is
     * rate-limited off HAL_GetTick so it cannot throttle the read. */
    uint16_t raw = 0;
    Encoder_Status_t st = Encoder_ReadAngle(&raw);

    if (st == ENC_OK)
    {
      g_enc.reads++;
    }
    else
    {
      g_enc.errors++;
    }

    uint16_t rx1 = 0, rx2 = 0;
    Encoder_GetDebugData(&rx1, &rx2);

    g_enc.raw      = raw;
    g_enc.deg_x100 = Encoder_RawToDegX100(raw);
    g_enc.frames   = ((uint32_t)rx2 << 16) | (uint32_t)rx1;

    /* LED brightness tracks the angle: 0 deg dark, 360 deg full. */
    g_enc.duty = LedPwm_SetFromAngle(g_enc.deg_x100);

    uint32_t now = HAL_GetTick();

    /* Latch the measured sample rate once per second. */
    if ((now - s_rate_t0) >= 1000U)
    {
      g_enc.rate_hz = g_enc.reads - s_rate_reads0;
      s_rate_reads0 = g_enc.reads;
      s_rate_t0     = now;
    }

    if (++s_print_div >= TELEM_PRINT_EVERY)
    {
      s_print_div = 0;
      Telem_Printf("angle %3lu.%02lu deg  raw %5lu  rx1 %04lX rx2 %04lX  "
                   "%lu Hz  duty %4lu  err %lu\r\n",
                   (unsigned long)(g_enc.deg_x100 / 100U),
                   (unsigned long)(g_enc.deg_x100 % 100U),
                   (unsigned long)g_enc.raw,
                   (unsigned long)rx1,
                   (unsigned long)rx2,
                   (unsigned long)g_enc.rate_hz,
                   (unsigned long)g_enc.duty,
                   (unsigned long)g_enc.errors);
    }

    /* Current sense, decimated - the ADC polling is far slower than the
     * encoder read and would otherwise dominate the loop. */
    if (++s_cs_div >= CS_READ_EVERY)
    {
      s_cs_div = 0;
      (void)CSense_Read((CSenseTelem_t *)&g_cs);
      MotorPwm_GetTelem((MotorPwmTelem_t *)&g_pwm);
    }

    /* Heartbeat so a stalled loop is visible without a debugger attached.
     * Rate-limited off HAL_GetTick - at ~15 kHz, toggling per iteration would
     * be a blur, not a blink. */
    if ((now - s_hb_t0) >= HEARTBEAT_MS)
    {
      s_hb_t0 = now;
      HAL_GPIO_TogglePin(GPIOB, HEARTBEAT_PIN);
    }

  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 16;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
