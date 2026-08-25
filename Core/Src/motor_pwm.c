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

void MotorPwm_EmergencyStop(void)
{
  /* DIS high = all three drivers off. BSRR is a single atomic write and needs
   * no read-modify-write, so this works even with a corrupted stack. */
  GATE_EN_PORT->BSRR = GATE_EN_PIN;

  /* Disconnect every HRTIM output. Writing ODISR is also a single store.
   * HRTIM is hardware and keeps switching through a halted CPU, so a fault
   * handler that only spins would leave the bridge live. */
  HRTIM1->sCommonRegs.ODISR = 0x3FFFU;

  s_gate_en    = 0;
  s_outputs_en = 0;
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
                                 uint32_t set_src, uint32_t reset_src)
{
  HRTIM_OutputCfgTypeDef pOutputCfg = {0};

  pOutputCfg.Polarity              = HRTIM_OUTPUTPOLARITY_HIGH;
  pOutputCfg.SetSource             = set_src;
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

  /* Compares must be valid before the outputs are configured. SetDuty(0,0,0)
   * below then clears the set-sources so all three are genuinely dead. */
  if (MotorPwm_ConfigCompare(HRTIM_TIMERINDEX_TIMER_A, HRTIM_COMPAREUNIT_1, PWM_CMP_MIN) != 0) { return -1; }
  if (MotorPwm_ConfigCompare(HRTIM_TIMERINDEX_TIMER_B, HRTIM_COMPAREUNIT_1, PWM_CMP_MIN) != 0) { return -1; }
  if (MotorPwm_ConfigCompare(HRTIM_TIMERINDEX_TIMER_B, HRTIM_COMPAREUNIT_2, PWM_CMP_MIN) != 0) { return -1; }

  /* W on Timer A output 2, driven from compare unit 1. */
  if (MotorPwm_ConfigOutput(HRTIM_TIMERINDEX_TIMER_A, HRTIM_OUTPUT_TA2,
                            HRTIM_OUTPUTSET_TIMCMP1, HRTIM_OUTPUTRESET_TIMCMP3) != 0) { return -1; }
  /* V and U share Timer B, on compare units 1 and 2 respectively. */
  if (MotorPwm_ConfigOutput(HRTIM_TIMERINDEX_TIMER_B, HRTIM_OUTPUT_TB1,
                            HRTIM_OUTPUTSET_TIMCMP1, HRTIM_OUTPUTRESET_TIMCMP3) != 0) { return -1; }
  if (MotorPwm_ConfigOutput(HRTIM_TIMERINDEX_TIMER_B, HRTIM_OUTPUT_TB2,
                            HRTIM_OUTPUTSET_TIMCMP2, HRTIM_OUTPUTRESET_TIMCMP4) != 0) { return -1; }

  /* All three outputs dead before the counters run. */
  MotorPwm_SetDuty(0, 0, 0);

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

/* Apply one phase.
 *
 * A commanded 0 must produce a genuinely dead output, and a small compare
 * cannot do that: below PWM_CMP_MIN the compare is ignored and the output
 * latches high for the whole period. So 0 clears the set-source instead - the
 * output is never driven high, and the (still valid) compare guarantees any
 * currently-high output gets reset once and stays low.
 *
 * setxr points at SETx1R or SETx2R for the phase's output. */
static void MotorPwm_ApplyPhase(volatile uint32_t *setxr, uint32_t timer_idx,
                                uint32_t set_unit, uint32_t reset_unit,
                                uint32_t set_src, uint32_t counts)
{
  /* Center-aligned in plain UP counting: place the pulse symmetrically about
   * the period midpoint using two compares.
   *
   *     SET on CMPa = PER/2 - counts/2
   *     RESET on CMPb = PER/2 + counts/2
   *
   * The HIGH pulse is centred on PER/2 and the zero vector straddles the
   * period boundary, which is where the ADC samples - the point at which the
   * ripple current equals its average. Up-down mode is not needed for this.
   * (Technique taken from the minifocer implementation.) */
  uint32_t centre = s_period / 2U;

  if (counts == 0U)
  {
    *setxr = 0U;                       /* never set -> output stays low */
    __HAL_HRTIM_SETCOMPARE(&hhrtim1, timer_idx, set_unit,   centre);
    __HAL_HRTIM_SETCOMPARE(&hhrtim1, timer_idx, reset_unit, centre + 1U);
    return;
  }

  if (counts > s_period - 2U) { counts = s_period - 2U; }

  uint32_t half = counts / 2U;
  uint32_t ca   = centre - half;
  uint32_t cb   = centre + half;

  if (ca < PWM_CMP_MIN)     { ca = PWM_CMP_MIN; }
  if (cb > s_period - 1U)   { cb = s_period - 1U; }
  if (cb <= ca)             { cb = ca + 1U; }

  *setxr = set_src;
  __HAL_HRTIM_SETCOMPARE(&hhrtim1, timer_idx, set_unit,   ca);
  __HAL_HRTIM_SETCOMPARE(&hhrtim1, timer_idx, reset_unit, cb);
}

void MotorPwm_SetDuty(uint32_t u, uint32_t v, uint32_t w)
{
  volatile HRTIM_Timerx_TypeDef *ta = &hhrtim1.Instance->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_A];
  volatile HRTIM_Timerx_TypeDef *tb = &hhrtim1.Instance->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_B];

  /* U: Timer B out2, CMP2 set / CMP4 reset.
   * V: Timer B out1, CMP1 set / CMP3 reset.
   * W: Timer A out2, CMP1 set / CMP3 reset. */
  MotorPwm_ApplyPhase(&tb->SETx2R, HRTIM_TIMERINDEX_TIMER_B,
                      HRTIM_COMPAREUNIT_2, HRTIM_COMPAREUNIT_4,
                      HRTIM_OUTPUTSET_TIMCMP2, u);
  MotorPwm_ApplyPhase(&tb->SETx1R, HRTIM_TIMERINDEX_TIMER_B,
                      HRTIM_COMPAREUNIT_1, HRTIM_COMPAREUNIT_3,
                      HRTIM_OUTPUTSET_TIMCMP1, v);
  MotorPwm_ApplyPhase(&ta->SETx2R, HRTIM_TIMERINDEX_TIMER_A,
                      HRTIM_COMPAREUNIT_1, HRTIM_COMPAREUNIT_3,
                      HRTIM_OUTPUTSET_TIMCMP1, w);
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

  /* COMPARE UNIT 4. Not 3 - CMP1 and CMP3 on Timer A are W's centred pulse
   * edges, and writing either from here would corrupt the W duty. */
  s_adc_trig_pos = counts;
  pCompareCfg.CompareValue = counts;
  (void)HAL_HRTIM_WaveformCompareConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A,
                                        HRTIM_COMPAREUNIT_4, &pCompareCfg);
}

/* Timer A compare unit 3 drives HRTIM ADC trigger 1. Both ADC2 (W) and ADC5
 * (U) select that same trigger, so the two phases are sampled at the same
 * instant every PWM period. The counters run whether or not the outputs are
 * enabled, so the trigger is verifiable with the power stage inert. */
static int MotorPwm_ConfigAdcTrigger(void)
{
  HRTIM_ADCTriggerCfgTypeDef pADCTriggerCfg = {0};

  /* Trigger EARLY enough that the conversion has finished before the control
   * ISR reads DR at the period event - see PWM_ADC_TRIG_PERMILLE. CMP4 is used
   * because Timer A's CMP1/CMP3 now carry W's centred pulse edges. */
  /* Trigger a fixed time before the period event, not a fixed fraction of it -
   * see PWM_ADC_LEAD_NS. Guard against a period shorter than the lead itself. */
  {
    uint32_t lead = PWM_ADC_LEAD_COUNTS;
    if (lead > (s_period / 2U)) { lead = s_period / 2U; }
    MotorPwm_SetAdcTriggerPoint(s_period - lead);
  }

  pADCTriggerCfg.UpdateSource = HRTIM_ADCTRIGGERUPDATE_TIMER_A;
  pADCTriggerCfg.Trigger      = HRTIM_ADCTRIGGEREVENT13_TIMERA_CMP4;
  if (HAL_HRTIM_ADCTriggerConfig(&hhrtim1, HRTIM_ADCTRIGGER_1,
                                 &pADCTriggerCfg) != HAL_OK)
  {
    return -1;
  }

  return 0;
}

void MotorPwm_SetDutyNorm(float u, float v, float w)
{
  MotorPwm_SetDuty((uint32_t)(u * (float)s_period),
                   (uint32_t)(v * (float)s_period),
                   (uint32_t)(w * (float)s_period));
}

/* Timer A repetition event fires once per PWM period. With RepetitionCounter
 * at 0 that is PWM_FREQ_HZ, and it is phase-locked to the same timebase driving the
 * ADC trigger - so the control loop always runs at the same point in the
 * switching period. */
void MotorPwm_EnableControlIsr(void)
{
  HRTIM1_TIMA->TIMxICR  = HRTIM_TIMICR_REPC;   /* clear stale flag */
  HRTIM1_TIMA->TIMxDIER = HRTIM_TIMDIER_REPIE;
  HAL_NVIC_SetPriority(HRTIM1_TIMA_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(HRTIM1_TIMA_IRQn);
}

void MotorPwm_DisableControlIsr(void)
{
  HRTIM1_TIMA->TIMxDIER = 0U;
  HAL_NVIC_DisableIRQ(HRTIM1_TIMA_IRQn);
}

uint32_t MotorPwm_GetPeriod(void)
{
  return s_period;
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
