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
#include "csense.h"
#include "motor_pwm.h"
#include "openloop.h"
#include "foc.h"
#include "position.h"
#include "can.h"
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
} EncTelem_t;

/* Bench command block, written over SWD so the power stage can be driven while
 * someone holds a scope probe - no reflash between steps.
 *
 * Everything defaults to off. Write the duty/enable fields first, then write
 * `apply` = 1; the main loop latches them and clears `apply`. Ordering is
 * enforced in the loop: on the way up duty -> outputs -> gates, on the way
 * down gates -> outputs, because only the gate line actually turns FETs off. */
typedef struct {
  uint32_t apply;       /* write 1 to latch the fields below */
  uint32_t duty_u;      /* per-mille, 0..1000                */
  uint32_t duty_v;
  uint32_t duty_w;
  uint32_t outputs_en;  /* 1 = enable the HRTIM outputs      */
  uint32_t gate_en;     /* 1 = enable the gate drivers (PC5) */
  uint32_t ol_enable;   /* 1 = run the open-loop rotating vector */
  uint32_t ol_freq_x100;/* electrical frequency, Hz * 100        */
  uint32_t ol_mod;      /* modulation index, per-mille           */
  uint32_t clear_fault; /* write 1 to clear an overcurrent latch */
  uint32_t ol_start;    /* write 1 to run align-then-ramp start   */
  uint32_t ol_align_ms; /* DC alignment hold, ms                  */
  uint32_t ol_ramp_ms;  /* 0 -> target frequency ramp time, ms    */
  uint32_t enc_cmd;     /* 1=read ANG regs, 2=zero->shadow,
                           3=zero->EEPROM (uses a write cycle!),
                           4=write enc_arg as ZERO_OFFSET->shadow */
  uint32_t enc_arg;     /* raw 12-bit offset for enc_cmd 4 / 5    */
} BenchCmd_t;
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
/* Sample the current sensors every N encoder reads. Fast, because this is the
 * only overcurrent protection there is - nothing is wired to HRTIM's hardware
 * fault inputs on this board. 20 reads is roughly 750 Hz. */
#define CS_READ_EVERY       20U

/* Trip the power stage above this on either phase.
 *
 * Steady-state peak at the usual 3% modulation is ~2200 mA, so 3000 left only
 * 27% headroom and nuisance-tripped on any small load disturbance. 4000 keeps
 * real protection (only ~700 mA is needed to actually turn the motor) while
 * leaving room for normal transients. */
#define OC_TRIP_MA          15000

/* After a trip, wait this long then re-arm and try again. */
#define OC_RETRY_MS         3000U

/* Consecutive retries before giving up and staying latched. Retrying forever
 * into a genuine short is how hardware dies, so this has to be bounded. The
 * counter resets once the drive has run clean for OC_CLEAN_MS. */
#define OC_MAX_RETRIES      5U
#define OC_CLEAN_MS         5000U

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

/* All-off at boot. Nothing here changes until something writes apply = 1. */
volatile BenchCmd_t g_cmd = {0};

volatile OpenLoopState_t g_ol = {0};
volatile FocState_t      g_foc = {0};
volatile PosState_t      g_pos = {0};

/* ---- CAN ---------------------------------------------------------------- *
 *
 * g_can_wants_bridge is how can.c asks for the power stage without reaching
 * into the peripheral itself: bringing the bridge up has a required order
 * (duty, then outputs, then gates) and exactly one place in this file
 * enforces it. CAN raises the request; the main loop performs it.
 *
 * The tx_* fields are a self-test hook. With g_can_loopback set, the node
 * receives its own transmissions, so writing a command here sends a real
 * protocol frame to ourselves and exercises filters, framing and decode with
 * no second node on the bus. */
volatile CanTelem_t g_can = {0};
volatile uint32_t g_can_wants_bridge = 0;
volatile uint32_t g_can_loopback     = 0;   /* set, then g_can_reinit, to self-test */
volatile uint32_t g_can_reinit       = 0;   /* write 1 to re-init FDCAN1            */
volatile uint32_t g_can_tx_cmd       = 0;   /* CAN_CMD_* to transmit                */
volatile uint32_t g_can_tx_go        = 0;   /* write 1 to send it                   */
volatile uint32_t g_can_tx_len       = 0;   /* payload bytes: 0, 1, 4 or 8          */
volatile int32_t  g_can_tx_arg       = 0;   /* first  int32 of the payload          */
volatile int32_t  g_can_tx_arg2      = 0;   /* second int32, for the 8-byte forms   */
static   uint32_t s_bridge_up        = 0;
volatile uint32_t g_oc_trips  = 0;   /* overcurrent trip count */
volatile int32_t  g_oc_peak   = 0;   /* worst |I| seen, mA     */
volatile uint32_t g_faulted   = 0;   /* latched: needs clear_fault or retry */
volatile uint32_t g_oc_retries = 0;  /* consecutive auto-retries            */
volatile uint32_t g_oc_gaveup  = 0;  /* 1 = retry budget exhausted          */

/* A1333 zero-calibration results, read over SWD. */
volatile uint32_t g_enc_shadow_ang = 0;
volatile uint32_t g_enc_ee_ang     = 0;
volatile uint32_t g_enc_zero_off   = 0;
volatile int32_t  g_enc_cmd_rc     = -1;
volatile int32_t  g_vbus_start_rc   = -99;
volatile int32_t  g_vbus_read_rc    = -99;
volatile uint32_t g_adc1_state      = 0;
volatile uint32_t g_adc1_err        = 0;

static uint32_t s_fault_tick = 0;
static uint32_t s_clean_tick = 0;

static uint32_t s_ol_tick = 0;

static uint32_t s_rate_t0     = 0;  /* window start for the rate counter */
static uint32_t s_rate_reads0 = 0;
static uint32_t s_print_div   = 0;
static uint32_t s_hb_t0       = 0;
static uint32_t s_cs_div      = 0;

/* 1 while the position loop owns g_foc.iq_ref. ISR-only. */
static uint32_t s_pos_drove   = 0;
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

static inline int32_t g_enc_abs(int32_t v) { return (v < 0) ? -v : v; }

/* Control ISR - HRTIM Timer A repetition, 20 kHz, phase-locked to the PWM.
 *
 * Everything time-critical lives here: fresh current from the HRTIM-triggered
 * ADCs, a fast encoder read, the FOC step, and the duty update. It runs only
 * when g_foc.enabled is set, so the open-loop path is untouched otherwise. */
void HRTIM1_TIMA_IRQHandler(void)
{
  uint32_t t0 = DWT->CYCCNT;

  HRTIM1_TIMA->TIMxICR = HRTIM_TIMICR_REPC;

  if (g_foc.enabled != 0U)
  {
    /* Conversions were triggered earlier this period, so DR is fresh. */
    uint32_t u_raw = hadc5.Instance->DR;
    uint32_t w_raw = hadc2.Instance->DR;

    int32_t iu = CSense_RawToMa(u_raw, g_cs.u_zero);
    int32_t iw = CSense_RawToMa(w_raw, g_cs.w_zero);

    uint16_t enc = Encoder_ReadAngleFast();

    /* Outer position loop. It self-decimates to POS_RATE_HZ, and is stepped
     * even while disengaged (g_pos.enabled clear) so its multi-turn count and
     * velocity estimate stay live - engaging then picks up from a correct
     * state instead of from a cold start with a stale angle. Only its OUTPUT
     * is conditional. */
    float iq_cmd = Position_Step((PosState_t *)&g_pos, enc);

    if (g_pos.enabled != 0U)
    {
      g_foc.id_ref = 0.0f;
      g_foc.iq_ref = iq_cmd;
      s_pos_drove  = 1U;
    }
    else if (s_pos_drove != 0U)
    {
      /* Disengaging must also drop the torque it was commanding. Simply
       * ceasing to write iq_ref would leave the current loop holding the last
       * servo output for as long as FOC stays enabled - the motor would keep
       * pushing at whatever it happened to need at the instant the operator
       * switched the position loop off. */
      s_pos_drove  = 0U;
      g_foc.id_ref = 0.0f;
      g_foc.iq_ref = 0.0f;
    }

    FOC_Update((FocState_t *)&g_foc, iu, iw, enc);

    MotorPwm_SetDutyNorm(g_foc.duty_u, g_foc.duty_v, g_foc.duty_w);
  }

  uint32_t dt = DWT->CYCCNT - t0;
  g_foc.isr_cycles = dt;
  if (dt > g_foc.isr_max) { g_foc.isr_max = dt; }
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

  /* Current sense. Zero-offset capture happens here, so the motor must be
   * de-energised and at rest at this point - which it is: nothing drives the
   * gate outputs yet. */
  g_cs_init_rc = CSense_Init((CSenseTelem_t *)&g_cs);

  /* Three-phase PWM. Counters run so the timebase can be verified, but the
   * outputs are NOT enabled - nothing is commanded to the gate drivers. */
  g_pwm_init_rc = MotorPwm_Init();
  MotorPwm_SetDutyPermille(0, 0, 0);
  OpenLoop_Init((OpenLoopState_t *)&g_ol, MotorPwm_GetPeriod());
  FOC_Init((FocState_t *)&g_foc);
  Position_Init((PosState_t *)&g_pos);
  (void)Can_Init(0U);
  Can_GetTelem((CanTelem_t *)&g_can);

  /* DWT cycle counter, used to time the control ISR. */
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0;
  DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;

  MotorPwm_EnableControlIsr();

  /* Now that the HRTIM ADC trigger exists, move current sensing onto it so
   * both phases are sampled at the same point in every PWM period. */
  g_cs_trig_rc = CSense_UseHrtimTrigger();

  /* VDDA has been measured, so ADC1 is free for the bus divider. */
  g_vbus_start_rc = CSense_StartVbus();

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
    /* SPI1 has exactly one owner at a time. Once the control ISR is running it
     * reads the encoder every PWM period, so the main loop must not touch the
     * bus: concurrent transfers corrupt each other and, worse, corrupt the
     * angle the Park transform depends on. Take the ISR's value instead. */
    uint16_t raw = 0;
    uint16_t rx1 = 0, rx2 = 0;

    if (g_foc.enabled != 0U)
    {
      raw = g_foc.enc_raw;
      g_enc.reads++;
    }
    else
    {
      Encoder_Status_t st = Encoder_ReadAngle(&raw);

      if (st == ENC_OK)
      {
        g_enc.reads++;
      }
      else
      {
        g_enc.errors++;
      }

      Encoder_GetDebugData(&rx1, &rx2);
    }

    g_enc.raw      = raw;
    g_enc.deg_x100 = Encoder_RawToDegX100(raw);
    g_enc.frames   = ((uint32_t)rx2 << 16) | (uint32_t)rx1;

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
                   "%lu Hz  err %lu\r\n",
                   (unsigned long)(g_enc.deg_x100 / 100U),
                   (unsigned long)(g_enc.deg_x100 % 100U),
                   (unsigned long)g_enc.raw,
                   (unsigned long)rx1,
                   (unsigned long)rx2,
                   (unsigned long)g_enc.rate_hz,
                   (unsigned long)g_enc.errors);
    }

    /* Current sense, decimated - the ADC polling is far slower than the
     * encoder read and would otherwise dominate the loop. */
    if (++s_cs_div >= CS_READ_EVERY)
    {
      s_cs_div = 0;
      (void)CSense_Read((CSenseTelem_t *)&g_cs);
      g_vbus_read_rc = CSense_ReadVbus((CSenseTelem_t *)&g_cs);

      /* Keep the current-loop gains matched to the bus that is actually
       * present. Vbus is the plant gain, so a supply change silently retunes
       * the loop unless the gains move with it - see foc.h. Costs two divides
       * at ~750 Hz and is a no-op if g_foc.vbus_track is cleared. */
      if (g_vbus_read_rc == 0)
      {
        FOC_SetGainsForVbus((FocState_t *)&g_foc, (int32_t)g_cs.vbus_mv);
      }
      g_adc1_state   = HAL_ADC_GetState(&hadc1);
      g_adc1_err     = HAL_ADC_GetError(&hadc1);
      MotorPwm_GetTelem((MotorPwmTelem_t *)&g_pwm);
    }

    /* Overcurrent trip. Software-only and therefore slow, but it is the only
     * protection present: nothing is wired to HRTIM's hardware fault inputs.
     * Latches until clear_fault is written. */
    if (g_faulted == 0U)
    {
      int32_t iu = g_enc_abs(g_cs.u_ma);
      int32_t iw = g_enc_abs(g_cs.w_ma);
      int32_t ip = (iu > iw) ? iu : iw;

      if (ip > g_oc_peak) { g_oc_peak = ip; }

      if (ip > OC_TRIP_MA)
      {
        MotorPwm_EmergencyStop();
        OpenLoop_Stop((OpenLoopState_t *)&g_ol);
        g_faulted    = 1U;
        g_oc_trips++;
        s_fault_tick = HAL_GetTick();
        /* g_cmd.ol_enable / gate_en are deliberately left alone - they record
         * what the operator asked for, and the retry below needs to know. */
      }
      else if ((HAL_GetTick() - s_clean_tick) >= OC_CLEAN_MS)
      {
        /* Run clean for long enough and the retry budget is restored. */
        g_oc_retries = 0U;
        g_oc_gaveup  = 0U;
        s_clean_tick = HAL_GetTick();
      }
    }
    else
    {
      /* Latched. Re-arm after OC_RETRY_MS, but only while the operator still
       * has the drive commanded on, and only within the retry budget. */
      if (((HAL_GetTick() - s_fault_tick) >= OC_RETRY_MS) &&
          (g_cmd.ol_enable != 0U) && (g_cmd.gate_en != 0U))
      {
        if (g_oc_retries < OC_MAX_RETRIES)
        {
          g_oc_retries++;
          g_oc_peak    = 0;
          g_faulted    = 0U;
          s_clean_tick = HAL_GetTick();

          MotorPwm_SetDuty(0, 0, 0);
          MotorPwm_EnableOutputs();
          MotorPwm_GateEnable();
          OpenLoop_Start((OpenLoopState_t *)&g_ol, g_cmd.ol_freq_x100,
                         g_cmd.ol_mod, g_cmd.ol_align_ms, g_cmd.ol_ramp_ms);
        }
        else
        {
          g_oc_gaveup = 1U;
        }
      }
    }

    /* Open-loop rotating vector, stepped at exactly OL_UPDATE_HZ off SysTick
     * so the commanded frequency does not depend on loop timing. */
    if ((g_faulted == 0U) && (g_cmd.ol_enable != 0U))
    {
      uint32_t t = HAL_GetTick();
      if (t != s_ol_tick)
      {
        s_ol_tick = t;
        OpenLoop_Update((OpenLoopState_t *)&g_ol);
      }
    }

    /* A1333 register access. Runs from the main loop so it shares the SPI
     * with the angle reads; the loop simply pauses while it runs. */
    if (g_cmd.enc_cmd != 0U)
    {
      uint32_t c = g_cmd.enc_cmd;
      g_cmd.enc_cmd = 0U;

      uint32_t sh = 0, ee = 0;
      uint16_t off = 0;

      if (c == 1U)
      {
        /* Read-only: proves the extended-access framing without spending an
         * EEPROM write cycle. */
        Encoder_Status_t a = Encoder_ExtRead(A1333_SHADOW_ANG, &sh);
        Encoder_Status_t b = Encoder_ExtRead(A1333_EE_ANG,     &ee);
        g_enc_shadow_ang = sh;
        g_enc_ee_ang     = ee;
        g_enc_cmd_rc     = ((a == ENC_OK) && (b == ENC_OK)) ? 0 : -1;
      }
      else if (c == 4U)
      {
        /* Write an explicit offset to SHADOW only. Used to measure how far a
         * known ZERO_OFFSET actually moves the reported angle, which settles
         * the 12-bit-offset vs 15-bit-angle scaling. Shadow is volatile and
         * unlimited, so this costs nothing. */
        Encoder_Status_t a = Encoder_SetZeroOffset((uint16_t)g_cmd.enc_arg, 0U);
        g_enc_cmd_rc = (a == ENC_OK) ? 0 : -1;
        (void)Encoder_ExtRead(A1333_SHADOW_ANG, &sh);
        g_enc_shadow_ang = sh;
      }
      else if (c == 5U)
      {
        /* Commit an explicit, already-validated offset to EEPROM. Deliberately
         * separate from ZeroHere: ZeroHere reads the CURRENT angle, which is
         * already offset-corrected, so re-running it would double-apply. This
         * writes a known-good number instead. Spends one of ~100 cycles. */
        Encoder_Status_t a = Encoder_SetZeroOffset((uint16_t)g_cmd.enc_arg, 1U);
        g_enc_cmd_rc = (a == ENC_OK) ? 0 : -1;
        (void)Encoder_ExtRead(A1333_SHADOW_ANG, &sh);
        (void)Encoder_ExtRead(A1333_EE_ANG,     &ee);
        g_enc_shadow_ang = sh;
        g_enc_ee_ang     = ee;
      }
      else if ((c == 2U) || (c == 3U))
      {
        /* c == 3 burns one of ~100 EEPROM write cycles. */
        Encoder_Status_t a = Encoder_ZeroHere(&off, (c == 3U) ? 1U : 0U);
        g_enc_zero_off = off;
        g_enc_cmd_rc   = (a == ENC_OK) ? 0 : -1;
        (void)Encoder_ExtRead(A1333_SHADOW_ANG, &sh);
        (void)Encoder_ExtRead(A1333_EE_ANG,     &ee);
        g_enc_shadow_ang = sh;
        g_enc_ee_ang     = ee;
      }
    }

    /* Bench commands from the debugger. */
    if (g_cmd.apply != 0U)
    {
      g_cmd.apply = 0U;

      if (g_cmd.clear_fault != 0U)
      {
        g_cmd.clear_fault = 0U;
        g_faulted    = 0U;
        g_oc_peak    = 0;
        g_oc_retries = 0U;
        g_oc_gaveup  = 0U;
        s_clean_tick = HAL_GetTick();
      }

      if (g_cmd.ol_start != 0U)
      {
        g_cmd.ol_start = 0U;
        OpenLoop_Start((OpenLoopState_t *)&g_ol, g_cmd.ol_freq_x100,
                       g_cmd.ol_mod, g_cmd.ol_align_ms, g_cmd.ol_ramp_ms);
      }
      else
      {
        OpenLoop_SetCommand((OpenLoopState_t *)&g_ol,
                            g_cmd.ol_freq_x100, g_cmd.ol_mod);
      }
      if (g_cmd.ol_enable == 0U)
      {
        OpenLoop_Stop((OpenLoopState_t *)&g_ol);
      }

      if (g_cmd.gate_en == 0U)
      {
        /* Gates first on the way down - the only true all-off. */
        MotorPwm_GateDisable();
      }

      if (g_cmd.ol_enable == 0U)
      {
        MotorPwm_SetDutyPermille(g_cmd.duty_u, g_cmd.duty_v, g_cmd.duty_w);
      }

      if (g_cmd.outputs_en != 0U)
      {
        MotorPwm_EnableOutputs();
      }
      else
      {
        MotorPwm_DisableOutputs();
      }

      if (g_cmd.gate_en != 0U)
      {
        /* Gates last on the way up, so the PWM is already correct before the
         * drivers are allowed to act on it. */
        MotorPwm_GateEnable();
      }
    }

    /* ---- CAN ------------------------------------------------------------ */
    if (g_can_reinit != 0U)
    {
      g_can_reinit = 0U;
      (void)Can_Init((uint8_t)g_can_loopback);
    }

    Can_Poll();
    Can_CheckTimeout();
    Can_PublishTelem();
    Can_GetTelem((CanTelem_t *)&g_can);

    /* Self-test transmit hook; see the g_can_tx_* declarations.
     *
     * Triggered by its own g_can_tx_go flag rather than by a non-zero command
     * id. ESTOP is command 0x00, so keying the trigger off the command made
     * the one frame most worth testing the one frame impossible to send. */
    if (g_can_tx_go != 0U)
    {
      g_can_tx_go = 0U;
      uint8_t  payload[8] = {0};
      uint8_t  len = (uint8_t)g_can_tx_len;
      uint32_t a   = (uint32_t)g_can_tx_arg;
      uint32_t b   = (uint32_t)g_can_tx_arg2;

      payload[0] = (uint8_t)a;         payload[1] = (uint8_t)(a >> 8);
      payload[2] = (uint8_t)(a >> 16); payload[3] = (uint8_t)(a >> 24);
      payload[4] = (uint8_t)b;         payload[5] = (uint8_t)(b >> 8);
      payload[6] = (uint8_t)(b >> 16); payload[7] = (uint8_t)(b >> 24);

      (void)Can_Send(CAN_ID(CAN_NODE_ID, g_can_tx_cmd & 0x1FU), payload, len);
    }

    /* CAN asked for the power stage. Same ordering as the bench block below -
     * outputs before gates on the way up, gates first on the way down. */
    if (g_can_wants_bridge != s_bridge_up)
    {
      s_bridge_up = g_can_wants_bridge;
      if (s_bridge_up != 0U)
      {
        g_faulted = 0U;
        MotorPwm_SetDutyPermille(0, 0, 0);
        MotorPwm_EnableOutputs();
        MotorPwm_GateEnable();
        g_foc.id_ref = 0.0f;
        g_foc.iq_ref = 0.0f;
        g_foc.enabled = 1U;
      }
      else
      {
        g_foc.enabled = 0U;
        MotorPwm_GateDisable();
        MotorPwm_DisableOutputs();
      }
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
