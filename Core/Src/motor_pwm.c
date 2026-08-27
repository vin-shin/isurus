/**
  ******************************************************************************
  * @file    motor_pwm.c
  * @brief   Three-phase complementary PWM on HRTIM1. Outputs start DISABLED.
  ******************************************************************************
  */

#include "motor_pwm.h"
#include "hrtim.h"
#include "main.h"

/* One timer per phase, both outputs of each. U=B, V=F, W=C. */
#define PWM_TIMER_U   HRTIM_TIMERINDEX_TIMER_B
#define PWM_TIMER_V   HRTIM_TIMERINDEX_TIMER_F
#define PWM_TIMER_W   HRTIM_TIMERINDEX_TIMER_C

#define PWM_OUTPUTS   (HRTIM_OUTPUT_TB1 | HRTIM_OUTPUT_TB2 | \
                       HRTIM_OUTPUT_TF1 | HRTIM_OUTPUT_TF2 | \
                       HRTIM_OUTPUT_TC1 | HRTIM_OUTPUT_TC2)
#define PWM_TIMERS    (HRTIM_TIMERID_TIMER_B | \
                       HRTIM_TIMERID_TIMER_F | \
                       HRTIM_TIMERID_TIMER_C)

static int MotorPwm_ConfigAdcTrigger(void);

static uint32_t s_period       = 0;
static uint32_t s_outputs_en   = 0;
static uint32_t s_gate_en      = 0;
static uint32_t s_adc_trig_pos = 0;

/* ---------------- Gate driver enable (PC8, ACTIVE HIGH) ---------------- */

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
  /* Gate drivers off. BSRR is a single atomic store and needs no
   * read-modify-write, so this works even with a corrupted stack. Which half
   * of BSRR that means depends on the enable polarity, which differs between
   * this board and Mako Longfin - GATE_EN_BSRR_DISABLE resolves it at compile
   * time so there is no branch here to get wrong. */
  GATE_EN_PORT->BSRR = GATE_EN_BSRR_DISABLE;

  /* Disconnect every HRTIM output. Writing ODISR is also a single store.
   * HRTIM is hardware and keeps switching through a halted CPU, so a fault
   * handler that only spins would leave the bridge live. */
  HRTIM1->sCommonRegs.ODISR = 0x3FFFU;

  s_gate_en    = 0;
  s_outputs_en = 0;
}

void MotorPwm_SafeShutdown(void)
{
  /* On this board either action alone opens the bridge: the outputs drive both
   * gates of each leg directly, and the gate enable cuts the drivers. Both are
   * done anyway, because the second one is free and this is the path a fault
   * takes. Mako Longfin needed a specific order here - the gate line was the
   * only true all-off there - and that constraint no longer applies. */
  MotorPwm_GateDisable();
  MotorPwm_DisableOutputs();
  MotorPwm_SetDuty(0, 0, 0);
}

static int MotorPwm_ConfigTimer(uint32_t timer_idx)
{
  HRTIM_TimeBaseCfgTypeDef pTimeBaseCfg = {0};
  HRTIM_TimerCfgTypeDef    pTimerCfg    = {0};
  HRTIM_TimerCtlTypeDef    pTimerCtl    = {0};
  HRTIM_DeadTimeCfgTypeDef pDeadTimeCfg = {0};

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
  /* HRTIM dead-time insertion, which is what makes output 2 the complement of
   * output 1 rather than a second waveform that has to be programmed to agree
   * with the first. Mako Longfin had this DISABLED - its dead time came from
   * the gate driver's own RDT resistor and the MCU emitted one edge per
   * phase. Here the MCU drives both devices, so the dead band is ours. */
  pTimerCfg.DeadTimeInsertion      = HRTIM_TIMDEADTIMEINSERTION_ENABLED;
  pTimerCfg.DelayedProtectionMode  = HRTIM_TIMER_A_B_C_DELAYEDPROTECTION_DISABLED;
  pTimerCfg.UpdateTrigger          = HRTIM_TIMUPDATETRIGGER_NONE;
  pTimerCfg.ResetTrigger           = HRTIM_TIMRESETTRIGGER_NONE;
  pTimerCfg.ResetUpdate            = HRTIM_TIMUPDATEONRESET_ENABLED;
  pTimerCfg.ReSyncUpdate           = HRTIM_TIMERESYNC_UPDATE_UNCONDITIONAL;
  if (HAL_HRTIM_WaveformTimerConfig(&hhrtim1, timer_idx, &pTimerCfg) != HAL_OK)
  {
    return -1;
  }

  /* Same values on both edges, both signs positive, prescaler DIV1 - straight
   * from the board's CubeMX project. The locks are left writeable: locking
   * them is a one-way door until reset and there is nothing here yet that has
   * earned that much confidence in the value. */
  pDeadTimeCfg.Prescaler       = HRTIM_TIMDEADTIME_PRESCALERRATIO_DIV1;
  pDeadTimeCfg.RisingValue     = PWM_DT_RISING;
  pDeadTimeCfg.RisingSign      = HRTIM_TIMDEADTIME_RISINGSIGN_POSITIVE;
  pDeadTimeCfg.RisingLock      = HRTIM_TIMDEADTIME_RISINGLOCK_WRITE;
  pDeadTimeCfg.RisingSignLock  = HRTIM_TIMDEADTIME_RISINGSIGNLOCK_WRITE;
  pDeadTimeCfg.FallingValue    = PWM_DT_FALLING;
  pDeadTimeCfg.FallingSign     = HRTIM_TIMDEADTIME_FALLINGSIGN_POSITIVE;
  pDeadTimeCfg.FallingLock     = HRTIM_TIMDEADTIME_FALLINGLOCK_WRITE;
  pDeadTimeCfg.FallingSignLock = HRTIM_TIMDEADTIME_FALLINGSIGNLOCK_WRITE;
  if (HAL_HRTIM_DeadTimeConfig(&hhrtim1, timer_idx, &pDeadTimeCfg) != HAL_OK)
  {
    return -1;
  }

  return 0;
}

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

/* Each phase owns a whole timer, so each gets compare units 1 and 2 for its
 * own centred pulse. On Mako Longfin two phases shared Timer B and the four
 * compare units had to be rationed between them, which is why the ADC trigger
 * ended up on a specific unit "because 1 and 3 were taken". Here CMP3 and CMP4
 * are free on every timer. */
static int MotorPwm_ConfigPhase(uint32_t timer_idx, uint32_t out_low)
{
  if (MotorPwm_ConfigCompare(timer_idx, HRTIM_COMPAREUNIT_1, PWM_CMP_MIN) != 0) { return -1; }
  if (MotorPwm_ConfigCompare(timer_idx, HRTIM_COMPAREUNIT_2, PWM_CMP_MIN) != 0) { return -1; }

  /* Output 1 only, which on this board is the LOW gate. The high gate is
   * produced from it by the dead-time unit and must NOT be given set/reset
   * sources of its own - doing so overrides the complement and removes the
   * dead band.
   *
   * SET on CMP2 and RESET on CMP1, which is the reverse of the obvious
   * ordering and is the whole correction: it makes output 1 low between CMP1
   * and CMP2, i.e. across the middle of the period, so its complement - the
   * high gate - is a centred pulse. See MotorPwm_ApplyPhase. */
  if (MotorPwm_ConfigOutput(timer_idx, out_low,
                            HRTIM_OUTPUTSET_TIMCMP2,
                            HRTIM_OUTPUTRESET_TIMCMP1) != 0) { return -1; }

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

  if (MotorPwm_ConfigTimer(PWM_TIMER_U) != 0) { return -1; }
  if (MotorPwm_ConfigTimer(PWM_TIMER_V) != 0) { return -1; }
  if (MotorPwm_ConfigTimer(PWM_TIMER_W) != 0) { return -1; }

  /* TB1 / TF1 / TC1 are PA10, PC6 and PB12 - the LOW gates. See motor_pwm.h. */
  if (MotorPwm_ConfigPhase(PWM_TIMER_U, HRTIM_OUTPUT_TB1) != 0) { return -1; }
  if (MotorPwm_ConfigPhase(PWM_TIMER_V, HRTIM_OUTPUT_TF1) != 0) { return -1; }
  if (MotorPwm_ConfigPhase(PWM_TIMER_W, HRTIM_OUTPUT_TC1) != 0) { return -1; }

  /* All three high sides dead before the counters run. */
  MotorPwm_SetDuty(0, 0, 0);

  if (MotorPwm_ConfigAdcTrigger() != 0) { return -1; }

  /* All three counters started in one register write, so they stay locked. */
  if (HAL_HRTIM_WaveformCounterStart(&hhrtim1, PWM_TIMERS) != HAL_OK)
  {
    return -1;
  }

  /* Outputs deliberately NOT started. Nothing is commanded to the drivers
   * until MotorPwm_EnableOutputs() is called. */
  s_outputs_en = 0;

  return 0;
}

/* Apply one phase. `counts` is the HIGH-side on-time.
 *
 * Output 1 is the LOW gate on this board, so the waveform programmed here is
 * the low gate's and the high gate is its dead-time complement.
 *
 * Centre-aligned in plain UP counting, with the compare values placed exactly
 * as they would be for a centred high-side pulse:
 *
 *     ca = PER/2 - counts/2      cb = PER/2 + counts/2
 *
 * and then output 1 is RESET at ca and SET at cb. That leaves the low gate
 * LOW between ca and cb and HIGH across the period boundary - so the high
 * gate, being the complement, is a pulse of width `counts` centred on PER/2.
 *
 * The zero vector therefore still straddles the period boundary, which is
 * where the ADC samples and where the ripple current equals its average. None
 * of that changed when the high/low mix-up was corrected; only which register
 * gets the set source and which gets the reset.
 *
 * setxr and rstxr point at SETx1R and RSTx1R for the phase. */
static void MotorPwm_ApplyPhase(volatile uint32_t *setxr, volatile uint32_t *rstxr,
                                uint32_t timer_idx, uint32_t counts)
{
  uint32_t centre = s_period / 2U;

  if (counts == 0U)
  {
    /* High side off for the whole period, so the LOW gate must be on for the
     * whole period - set at the period rollover and never reset.
     *
     * This is the case the high/low mix-up made dangerous. Clearing the set
     * source, which is what a "dead output" meant when output 1 was believed
     * to be the high gate, leaves output 1 permanently LOW here - and its
     * complement, the HIGH gate, permanently ON. A commanded zero would have
     * clamped every phase to the positive rail. */
    *setxr = HRTIM_OUTPUTSET_TIMPER;
    *rstxr = 0U;
    __HAL_HRTIM_SETCOMPARE(&hhrtim1, timer_idx, HRTIM_COMPAREUNIT_1, centre);
    __HAL_HRTIM_SETCOMPARE(&hhrtim1, timer_idx, HRTIM_COMPAREUNIT_2, centre + 1U);
    return;
  }

  if (counts > s_period - 2U) { counts = s_period - 2U; }

  uint32_t half = counts / 2U;
  uint32_t ca   = centre - half;
  uint32_t cb   = centre + half;

  if (ca < PWM_CMP_MIN)     { ca = PWM_CMP_MIN; }
  if (cb > s_period - 1U)   { cb = s_period - 1U; }
  if (cb <= ca)             { cb = ca + 1U; }

  *setxr = HRTIM_OUTPUTSET_TIMCMP2;
  *rstxr = HRTIM_OUTPUTRESET_TIMCMP1;
  __HAL_HRTIM_SETCOMPARE(&hhrtim1, timer_idx, HRTIM_COMPAREUNIT_1, ca);
  __HAL_HRTIM_SETCOMPARE(&hhrtim1, timer_idx, HRTIM_COMPAREUNIT_2, cb);
}

void MotorPwm_SetDuty(uint32_t u, uint32_t v, uint32_t w)
{
  volatile HRTIM_Timerx_TypeDef *tu = &hhrtim1.Instance->sTimerxRegs[PWM_TIMER_U];
  volatile HRTIM_Timerx_TypeDef *tv = &hhrtim1.Instance->sTimerxRegs[PWM_TIMER_V];
  volatile HRTIM_Timerx_TypeDef *tw = &hhrtim1.Instance->sTimerxRegs[PWM_TIMER_W];

  MotorPwm_ApplyPhase(&tu->SETx1R, &tu->RSTx1R, PWM_TIMER_U, u);
  MotorPwm_ApplyPhase(&tv->SETx1R, &tv->RSTx1R, PWM_TIMER_V, v);
  MotorPwm_ApplyPhase(&tw->SETx1R, &tw->RSTx1R, PWM_TIMER_W, w);
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

  /* COMPARE UNIT 3 on the U timer. Units 1 and 2 carry U's centred pulse
   * edges; writing either from here would corrupt the U duty. */
  s_adc_trig_pos = counts;
  pCompareCfg.CompareValue = counts;
  (void)HAL_HRTIM_WaveformCompareConfig(&hhrtim1, PWM_TIMER_U,
                                        HRTIM_COMPAREUNIT_3, &pCompareCfg);
}

/* Timer B compare unit 3 drives HRTIM ADC trigger 1, which ADC1 selects, so
 * all five ADC1 channels are converted from the same instant every PWM period.
 * The counters run whether or not the outputs are enabled, so the trigger is
 * verifiable with the power stage inert. */
static int MotorPwm_ConfigAdcTrigger(void)
{
  HRTIM_ADCTriggerCfgTypeDef pADCTriggerCfg = {0};

  /* Trigger a fixed time before the period event, not a fixed fraction of it -
   * see PWM_ADC_LEAD_NS. Guard against a period shorter than the lead. */
  {
    uint32_t lead = PWM_ADC_LEAD_COUNTS;
    if (lead > (s_period / 2U)) { lead = s_period / 2U; }
    MotorPwm_SetAdcTriggerPoint(s_period - lead);
  }

  pADCTriggerCfg.UpdateSource = HRTIM_ADCTRIGGERUPDATE_TIMER_B;
  pADCTriggerCfg.Trigger      = HRTIM_ADCTRIGGEREVENT13_TIMERB_CMP3;
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

/* Timer B repetition event fires once per PWM period. With RepetitionCounter
 * at 0 that is PWM_FREQ_HZ, and it is phase-locked to the same timebase
 * driving the ADC trigger - so the control loop always runs at the same point
 * in the switching period. */
void MotorPwm_EnableControlIsr(void)
{
  HRTIM1_TIMB->TIMxICR  = HRTIM_TIMICR_REPC;   /* clear stale flag */
  HRTIM1_TIMB->TIMxDIER = HRTIM_TIMDIER_REPIE;
  HAL_NVIC_SetPriority(HRTIM1_TIMB_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(HRTIM1_TIMB_IRQn);
}

void MotorPwm_DisableControlIsr(void)
{
  HRTIM1_TIMB->TIMxDIER = 0U;
  HAL_NVIC_DisableIRQ(HRTIM1_TIMB_IRQn);
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
  t->cmp_u      = hhrtim1.Instance->sTimerxRegs[PWM_TIMER_U].CMP1xR;
  t->cmp_v      = hhrtim1.Instance->sTimerxRegs[PWM_TIMER_V].CMP1xR;
  t->cmp_w      = hhrtim1.Instance->sTimerxRegs[PWM_TIMER_W].CMP1xR;
  t->cnt_u      = hhrtim1.Instance->sTimerxRegs[PWM_TIMER_U].CNTxR;
  t->cnt_v      = hhrtim1.Instance->sTimerxRegs[PWM_TIMER_V].CNTxR;
  t->outputs_en   = s_outputs_en;
  t->oenr         = hhrtim1.Instance->sCommonRegs.OENR;
  t->gate_en      = s_gate_en;
  t->adc_trig_pos = s_adc_trig_pos;
}
