/**
  ******************************************************************************
  * @file    motor_pwm.c
  * @brief   Three-phase PWM on HRTIM1. Outputs start DISABLED.
  ******************************************************************************
  */

#include "motor_pwm.h"
#include "hrtim.h"
#include "main.h"

#define PWM_OUTPUTS   (HRTIM_OUTPUT_TA2 | HRTIM_OUTPUT_TB1 | HRTIM_OUTPUT_TB2)
#define PWM_TIMERS    (HRTIM_TIMERID_TIMER_A | HRTIM_TIMERID_TIMER_B)

static int MotorPwm_ConfigAdcTrigger(void);

static uint32_t s_period       = 0;
static uint32_t s_outputs_en   = 0;
static uint32_t s_gate_en      = 0;
static uint32_t s_adc_trig_pos = 0;

/* ---------------- Gate driver enable (PC5) ---------------- */

void MotorPwm_GateInit(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOC_CLK_ENABLE();

  /* Drive the line to DISABLED *before* switching the pin to an output, so
   * enabling the pin cannot produce even a momentary enable pulse. */
  HAL_GPIO_WritePin(GATE_EN_PORT, GATE_EN_PIN,
                    GATE_EN_ACTIVE_HIGH ? GPIO_PIN_RESET : GPIO_PIN_SET);

  GPIO_InitStruct.Pin   = GATE_EN_PIN;
  GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull  = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GATE_EN_PORT, &GPIO_InitStruct);

  HAL_GPIO_WritePin(GATE_EN_PORT, GATE_EN_PIN,
                    GATE_EN_ACTIVE_HIGH ? GPIO_PIN_RESET : GPIO_PIN_SET);
  s_gate_en = 0;
}

void MotorPwm_GateEnable(void)
{
  HAL_GPIO_WritePin(GATE_EN_PORT, GATE_EN_PIN,
                    GATE_EN_ACTIVE_HIGH ? GPIO_PIN_SET : GPIO_PIN_RESET);
  s_gate_en = 1;
}

void MotorPwm_GateDisable(void)
{
  HAL_GPIO_WritePin(GATE_EN_PORT, GATE_EN_PIN,
                    GATE_EN_ACTIVE_HIGH ? GPIO_PIN_RESET : GPIO_PIN_SET);
  s_gate_en = 0;
}

uint32_t MotorPwm_GateIsEnabled(void)
{
  return s_gate_en;
}

void MotorPwm_SafeShutdown(void)
{
  /* Gate drivers first: that is the only action which actually turns every
   * FET off. Dropping the HRTIM outputs alone just parks the pins low, which
   * with the external inverter means the low-side devices conduct. */
  MotorPwm_GateDisable();
  MotorPwm_DisableOutputs();
  MotorPwm_SetDuty(0, 0, 0);
}

static int MotorPwm_ConfigTimer(uint32_t timer_idx)
{
  HRTIM_TimeBaseCfgTypeDef pTimeBaseCfg = {0};
  HRTIM_TimerCfgTypeDef    pTimerCfg    = {0};
  HRTIM_TimerCtlTypeDef    pTimerCtl    = {0};

  pTimeBaseCfg.Period            = s_period;
  pTimeBaseCfg.RepetitionCounter = 0x00;
  pTimeBaseCfg.PrescalerRatio    = HRTIM_PRESCALERRATIO_MUL8;
  pTimeBaseCfg.Mode              = HRTIM_MODE_CONTINUOUS;
  if (HAL_HRTIM_TimeBaseConfig(&hhrtim1, timer_idx, &pTimeBaseCfg) != HAL_OK)
  {
    return -1;
  }

  pTimerCtl.UpDownMode          = HRTIM_TIMERUPDOWNMODE_UP;
  pTimerCtl.DualChannelDacEnable = HRTIM_TIMER_DCDE_DISABLED;
  if (HAL_HRTIM_WaveformTimerControl(&hhrtim1, timer_idx, &pTimerCtl) != HAL_OK)
  {
    return -1;
  }

  pTimerCfg.InterruptRequests      = HRTIM_TIM_IT_NONE;
  pTimerCfg.DMARequests            = HRTIM_TIM_DMA_NONE;
  pTimerCfg.DMASrcAddress          = 0x0000;
  pTimerCfg.DMADstAddress          = 0x0000;
  pTimerCfg.DMASize                = 0x1;
  pTimerCfg.HalfModeEnable         = HRTIM_HALFMODE_DISABLED;
  pTimerCfg.InterleavedMode        = HRTIM_INTERLEAVED_MODE_DISABLED;
  pTimerCfg.StartOnSync            = HRTIM_SYNCSTART_DISABLED;
  pTimerCfg.ResetOnSync            = HRTIM_SYNCRESET_DISABLED;
  pTimerCfg.DACSynchro             = HRTIM_DACSYNC_NONE;
  /* Preload + update-on-reset: a duty written mid-period is latched at the
   * next period boundary, so all three phases change together and no output
   * ever sees a half-written compare value. */
  pTimerCfg.PreloadEnable          = HRTIM_PRELOAD_ENABLED;
  pTimerCfg.UpdateGating           = HRTIM_UPDATEGATING_INDEPENDENT;
  pTimerCfg.BurstMode              = HRTIM_TIMERBURSTMODE_MAINTAINCLOCK;
  pTimerCfg.RepetitionUpdate       = HRTIM_UPDATEONREPETITION_DISABLED;
  pTimerCfg.PushPull               = HRTIM_TIMPUSHPULLMODE_DISABLED;
  pTimerCfg.FaultEnable            = HRTIM_TIMFAULTENABLE_NONE;
  pTimerCfg.FaultLock              = HRTIM_TIMFAULTLOCK_READWRITE;
  /* No HRTIM dead-time insertion: the UCC21330's own RDT does that, and the
   * external inverter means the MCU only ever emits one edge per phase. */
  pTimerCfg.DeadTimeInsertion      = HRTIM_TIMDEADTIMEINSERTION_DISABLED;
  pTimerCfg.DelayedProtectionMode  = HRTIM_TIMER_A_B_C_DELAYEDPROTECTION_DISABLED;
  pTimerCfg.UpdateTrigger          = HRTIM_TIMUPDATETRIGGER_NONE;
  pTimerCfg.ResetTrigger           = HRTIM_TIMRESETTRIGGER_NONE;
  pTimerCfg.ResetUpdate            = HRTIM_TIMUPDATEONRESET_ENABLED;
  pTimerCfg.ReSyncUpdate           = HRTIM_TIMERESYNC_UPDATE_UNCONDITIONAL;
  if (HAL_HRTIM_WaveformTimerConfig(&hhrtim1, timer_idx, &pTimerCfg) != HAL_OK)
  {
    return -1;
  }

  return 0;
}

/* Edge-aligned PWM: the output is set at the period rollover and cleared at
 * the compare match, so duty = compare / period. */
static int MotorPwm_ConfigOutput(uint32_t timer_idx, uint32_t output,
                                 uint32_t reset_src)
{
  HRTIM_OutputCfgTypeDef pOutputCfg = {0};

  pOutputCfg.Polarity              = HRTIM_OUTPUTPOLARITY_HIGH;
  pOutputCfg.SetSource             = HRTIM_OUTPUTSET_TIMPER;
  pOutputCfg.ResetSource           = reset_src;
  pOutputCfg.IdleMode              = HRTIM_OUTPUTIDLEMODE_NONE;
  pOutputCfg.IdleLevel             = HRTIM_OUTPUTIDLELEVEL_INACTIVE;
  pOutputCfg.FaultLevel            = HRTIM_OUTPUTFAULTLEVEL_NONE;
  pOutputCfg.ChopperModeEnable     = HRTIM_OUTPUTCHOPPERMODE_DISABLED;
  pOutputCfg.BurstModeEntryDelayed = HRTIM_OUTPUTBURSTMODEENTRY_REGULAR;

  if (HAL_HRTIM_WaveformOutputConfig(&hhrtim1, timer_idx, output, &pOutputCfg) != HAL_OK)
  {
    return -1;
  }

  return 0;
}

static int MotorPwm_ConfigCompare(uint32_t timer_idx, uint32_t unit, uint32_t value)
{
  HRTIM_CompareCfgTypeDef pCompareCfg = {0};

  pCompareCfg.CompareValue = value;
  if (HAL_HRTIM_WaveformCompareConfig(&hhrtim1, timer_idx, unit, &pCompareCfg) != HAL_OK)
  {
    return -1;
  }

  return 0;
}

int MotorPwm_Init(void)
{
  /* Gate drivers held off before anything else. */
  MotorPwm_GateInit();

  /* MX_HRTIM1_Init runs the DLL calibration and sets up the output GPIOs. */
  MX_HRTIM1_Init();

  /* HRTIM kernel clock = APB2 timer clock. SystemClock_Config leaves the APB2
   * prescaler at 1, so that equals HCLK. */
  s_period = (SystemCoreClock * PWM_HRTIM_MUL) / PWM_FREQ_HZ;

  if (MotorPwm_ConfigTimer(HRTIM_TIMERINDEX_TIMER_A) != 0) { return -1; }
  if (MotorPwm_ConfigTimer(HRTIM_TIMERINDEX_TIMER_B) != 0) { return -1; }

  /* Start every phase at 0% before any output can be enabled. */
  if (MotorPwm_ConfigCompare(HRTIM_TIMERINDEX_TIMER_A, HRTIM_COMPAREUNIT_1, PWM_CMP_MIN) != 0) { return -1; }
  if (MotorPwm_ConfigCompare(HRTIM_TIMERINDEX_TIMER_B, HRTIM_COMPAREUNIT_1, PWM_CMP_MIN) != 0) { return -1; }
  if (MotorPwm_ConfigCompare(HRTIM_TIMERINDEX_TIMER_B, HRTIM_COMPAREUNIT_2, PWM_CMP_MIN) != 0) { return -1; }

  /* W on Timer A output 2, driven from compare unit 1. */
  if (MotorPwm_ConfigOutput(HRTIM_TIMERINDEX_TIMER_A, HRTIM_OUTPUT_TA2,
                            HRTIM_OUTPUTRESET_TIMCMP1) != 0) { return -1; }
  /* V and U share Timer B, on compare units 1 and 2 respectively. */
  if (MotorPwm_ConfigOutput(HRTIM_TIMERINDEX_TIMER_B, HRTIM_OUTPUT_TB1,
                            HRTIM_OUTPUTRESET_TIMCMP1) != 0) { return -1; }
  if (MotorPwm_ConfigOutput(HRTIM_TIMERINDEX_TIMER_B, HRTIM_OUTPUT_TB2,
                            HRTIM_OUTPUTRESET_TIMCMP2) != 0) { return -1; }

  if (MotorPwm_ConfigAdcTrigger() != 0) { return -1; }

  /* Both counters started in one register write, so they stay locked. */
  if (HAL_HRTIM_WaveformCounterStart(&hhrtim1, PWM_TIMERS) != HAL_OK)
  {
    return -1;
  }

  /* Outputs deliberately NOT started. Nothing is commanded to the drivers
   * until MotorPwm_EnableOutputs() is called. */
  s_outputs_en = 0;

  return 0;
}

void MotorPwm_SetDuty(uint32_t u, uint32_t v, uint32_t w)
{
  uint32_t cu = u, cv = v, cw = w;

  if (cu < PWM_CMP_MIN) { cu = PWM_CMP_MIN; }
  if (cv < PWM_CMP_MIN) { cv = PWM_CMP_MIN; }
  if (cw < PWM_CMP_MIN) { cw = PWM_CMP_MIN; }
  if (cu > s_period - 1U) { cu = s_period - 1U; }
  if (cv > s_period - 1U) { cv = s_period - 1U; }
  if (cw > s_period - 1U) { cw = s_period - 1U; }

  /* Preload registers - latched together at the next period boundary. */
  __HAL_HRTIM_SETCOMPARE(&hhrtim1, HRTIM_TIMERINDEX_TIMER_B, HRTIM_COMPAREUNIT_2, cu);
  __HAL_HRTIM_SETCOMPARE(&hhrtim1, HRTIM_TIMERINDEX_TIMER_B, HRTIM_COMPAREUNIT_1, cv);
  __HAL_HRTIM_SETCOMPARE(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, HRTIM_COMPAREUNIT_1, cw);
}

void MotorPwm_SetDutyPermille(uint32_t u, uint32_t v, uint32_t w)
{
  if (u > 1000U) { u = 1000U; }
  if (v > 1000U) { v = 1000U; }
  if (w > 1000U) { w = 1000U; }

  MotorPwm_SetDuty((s_period * u) / 1000U,
                   (s_period * v) / 1000U,
                   (s_period * w) / 1000U);
}

void MotorPwm_EnableOutputs(void)
{
  (void)HAL_HRTIM_WaveformOutputStart(&hhrtim1, PWM_OUTPUTS);
  s_outputs_en = 1;
}

void MotorPwm_DisableOutputs(void)
{
  (void)HAL_HRTIM_WaveformOutputStop(&hhrtim1, PWM_OUTPUTS);
  s_outputs_en = 0;
}

void MotorPwm_SetAdcTriggerPoint(uint32_t counts)
{
  HRTIM_CompareCfgTypeDef pCompareCfg = {0};

  if (counts < PWM_CMP_MIN)      { counts = PWM_CMP_MIN; }
  if (counts > s_period - 1U)    { counts = s_period - 1U; }

  s_adc_trig_pos = counts;
  pCompareCfg.CompareValue = counts;
  (void)HAL_HRTIM_WaveformCompareConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A,
                                        HRTIM_COMPAREUNIT_3, &pCompareCfg);
}

/* Timer A compare unit 3 drives HRTIM ADC trigger 1. Both ADC2 (W) and ADC5
 * (U) select that same trigger, so the two phases are sampled at the same
 * instant every PWM period. The counters run whether or not the outputs are
 * enabled, so the trigger is verifiable with the power stage inert. */
static int MotorPwm_ConfigAdcTrigger(void)
{
  HRTIM_ADCTriggerCfgTypeDef pADCTriggerCfg = {0};

  MotorPwm_SetAdcTriggerPoint((s_period * PWM_ADC_TRIG_PERMILLE) / 1000U);

  pADCTriggerCfg.UpdateSource = HRTIM_ADCTRIGGERUPDATE_TIMER_A;
  pADCTriggerCfg.Trigger      = HRTIM_ADCTRIGGEREVENT13_TIMERA_CMP3;
  if (HAL_HRTIM_ADCTriggerConfig(&hhrtim1, HRTIM_ADCTRIGGER_1,
                                 &pADCTriggerCfg) != HAL_OK)
  {
    return -1;
  }

  return 0;
}

void MotorPwm_GetTelem(MotorPwmTelem_t *t)
{
  t->period     = s_period;
  t->pwm_hz     = (s_period != 0U)
                    ? ((SystemCoreClock * PWM_HRTIM_MUL) / s_period)
                    : 0U;
  t->cmp_u      = hhrtim1.Instance->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_B].CMP2xR;
  t->cmp_v      = hhrtim1.Instance->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_B].CMP1xR;
  t->cmp_w      = hhrtim1.Instance->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_A].CMP1xR;
  t->cnt_a      = hhrtim1.Instance->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_A].CNTxR;
  t->cnt_b      = hhrtim1.Instance->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_B].CNTxR;
  t->outputs_en   = s_outputs_en;
  t->oenr         = hhrtim1.Instance->sCommonRegs.OENR;
  t->gate_en      = s_gate_en;
  t->adc_trig_pos = s_adc_trig_pos;
}
