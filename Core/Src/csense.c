/**
  ******************************************************************************
  * @file    csense.c
  * @brief   Phase current sensing - CT4022-A40BSN8 TMR sensors on U and W.
  ******************************************************************************
  */

#include "csense.h"
#include "adc.h"
#include "opamp.h"
#include "main.h"

#define CS_CONV_TIMEOUT_MS  10U

/* ADC5 is not in the .ioc, so its handle and bring-up live here rather than in
 * the CubeMX-generated adc.c (which a regeneration would overwrite). */
ADC_HandleTypeDef hadc5;

/* Measured supply, used for the mV conversion. Starts at nominal so a read
 * before CSense_MeasureVdda() still returns something sane. */
static uint32_t s_vdda_mv = CS_VREF_MV;

/* 0 = software start (bring-up / calibration), 1 = HRTIM triggered. */
static uint8_t s_triggered = 0;

static int CSense_Adc5Init(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};
  ADC_ChannelConfTypeDef   sConfig       = {0};

  /* ADC345 runs off its own kernel clock select, separate from ADC12. */
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC345;
  PeriphClkInit.Adc345ClockSelection = RCC_ADC345CLKSOURCE_SYSCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    return -1;
  }
  __HAL_RCC_ADC345_CLK_ENABLE();

  hadc5.Instance                   = ADC5;
  hadc5.Init.ClockPrescaler        = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc5.Init.Resolution            = ADC_RESOLUTION_12B;
  hadc5.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
  hadc5.Init.GainCompensation      = 0;
  hadc5.Init.ScanConvMode          = ADC_SCAN_DISABLE;
  hadc5.Init.EOCSelection          = ADC_EOC_SINGLE_CONV;
  hadc5.Init.LowPowerAutoWait      = DISABLE;
  hadc5.Init.ContinuousConvMode    = DISABLE;
  hadc5.Init.NbrOfConversion       = 1;
  hadc5.Init.DiscontinuousConvMode = DISABLE;
  hadc5.Init.ExternalTrigConv      = ADC_SOFTWARE_START;
  hadc5.Init.ExternalTrigConvEdge  = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc5.Init.DMAContinuousRequests = DISABLE;
  /* MUST be OVERWRITTEN, not PRESERVED. The HRTIM triggers conversions at
   * 20 kHz while this is polled far slower, so overrun is continuous. With
   * DATA_PRESERVED the ADC discards every new conversion and DR stays frozen
   * on the first sample forever - which reads as a perfectly dead sensor. */
  hadc5.Init.Overrun               = ADC_OVR_DATA_OVERWRITTEN;
  hadc5.Init.OversamplingMode      = DISABLE;
  if (HAL_ADC_Init(&hadc5) != HAL_OK)
  {
    return -1;
  }

  sConfig.Channel      = ADC_CHANNEL_VOPAMP5;
  sConfig.Rank         = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_24CYCLES_5;
  sConfig.SingleDiff   = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset       = 0;
  if (HAL_ADC_ConfigChannel(&hadc5, &sConfig) != HAL_OK)
  {
    return -1;
  }

  return 0;
}

/* Point ADC2 at the OPAMP3 output instead of the placeholder channel CubeMX
 * generated (ADC_CHANNEL_3). */
static int CSense_Adc2Retarget(void)
{
  ADC_ChannelConfTypeDef sConfig = {0};

  sConfig.Channel      = ADC_CHANNEL_VOPAMP3_ADC2;
  sConfig.Rank         = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_24CYCLES_5;
  sConfig.SingleDiff   = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset       = 0;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    return -1;
  }

  return 0;
}

/* Point ADC1 at the internal reference so we can back out the true VDDA.
 * The sensors are ratiometric on the same rail, so this does NOT change the
 * current scaling - only the reported voltage. */
static int CSense_Adc1Retarget(void)
{
  ADC_ChannelConfTypeDef sConfig = {0};

  sConfig.Channel      = ADC_CHANNEL_VREFINT;
  sConfig.Rank         = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_247CYCLES_5;  /* VREFINT is slow */
  sConfig.SingleDiff   = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset       = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    return -1;
  }

  return 0;
}

static int CSense_SampleOnce(ADC_HandleTypeDef *hadc, uint32_t *out)
{
  if (s_triggered)
  {
    /* Already armed and free-running off the HRTIM trigger; just wait for the
     * next end-of-conversion. Reading DR clears EOC. At 20 kHz a fresh sample
     * is never more than 50 us away, so this loop is short. */
    uint32_t guard = 2000000U;

    /* Overrun is expected and harmless in OVERWRITTEN mode - DR always holds
     * the most recent conversion. Clear the flag so it does not accumulate. */
    __HAL_ADC_CLEAR_FLAG(hadc, ADC_FLAG_OVR);

    while (__HAL_ADC_GET_FLAG(hadc, ADC_FLAG_EOC) == 0U)
    {
      if (--guard == 0U) { return -1; }
    }
    *out = hadc->Instance->DR;
    return 0;
  }

  if (HAL_ADC_Start(hadc) != HAL_OK)
  {
    return -1;
  }
  if (HAL_ADC_PollForConversion(hadc, CS_CONV_TIMEOUT_MS) != HAL_OK)
  {
    (void)HAL_ADC_Stop(hadc);
    return -1;
  }

  *out = HAL_ADC_GetValue(hadc);
  (void)HAL_ADC_Stop(hadc);

  return 0;
}

int CSense_Init(CSenseTelem_t *t)
{
  /* Followers with InternalOutput enabled - configured by CubeMX, but never
   * started by it. Without HAL_OPAMP_Start the ADC sees nothing useful. */
  MX_OPAMP3_Init();
  MX_OPAMP5_Init();
  if (HAL_OPAMP_Start(&hopamp3) != HAL_OK) { return -1; }
  if (HAL_OPAMP_Start(&hopamp5) != HAL_OK) { return -1; }

  if (CSense_Adc1Retarget() != 0) { return -1; }
  if (CSense_Adc2Retarget() != 0) { return -1; }
  if (CSense_Adc5Init()     != 0) { return -1; }

  /* Self-calibration must happen while the ADC is disabled, i.e. before the
   * first conversion. Skipping it costs several LSB of offset. */
  if (HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED) != HAL_OK) { return -1; }
  if (HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED) != HAL_OK) { return -1; }
  if (HAL_ADCEx_Calibration_Start(&hadc5, ADC_SINGLE_ENDED) != HAL_OK) { return -1; }

  if (CSense_MeasureVdda(t) != 0) { return -1; }

  return CSense_CalibrateZero(t);
}

int CSense_CalibrateZero(CSenseTelem_t *t)
{
  uint32_t u_acc = 0, w_acc = 0;
  uint32_t n     = 0;

  for (uint32_t i = 0; i < CS_ZERO_SAMPLES; i++)
  {
    uint32_t u = 0, w = 0;

    if (CSense_SampleOnce(&hadc5, &u) != 0) { t->errors++; continue; }
    if (CSense_SampleOnce(&hadc2, &w) != 0) { t->errors++; continue; }

    u_acc += u;
    w_acc += w;
    n++;
  }

  if (n == 0U)
  {
    return -1;
  }

  t->u_zero = u_acc / n;
  t->w_zero = w_acc / n;

  return 0;
}

int CSense_Read(CSenseTelem_t *t)
{
  uint32_t u = 0, w = 0;

  if (CSense_SampleOnce(&hadc5, &u) != 0) { t->errors++; return -1; }
  if (CSense_SampleOnce(&hadc2, &w) != 0) { t->errors++; return -1; }

  t->u_raw = u;
  t->w_raw = w;
  t->u_mv  = CSense_RawToMv(u);
  t->w_mv  = CSense_RawToMv(w);
  t->u_ma  = CSense_RawToMa(u, t->u_zero);
  t->w_ma  = CSense_RawToMa(w, t->w_zero);
  t->samples++;

  return 0;
}

int CSense_MeasureVdda(CSenseTelem_t *t)
{
  uint32_t acc = 0;
  uint32_t n   = 0;

  for (uint32_t i = 0; i < 64U; i++)
  {
    uint32_t v = 0;
    if (CSense_SampleOnce(&hadc1, &v) != 0) { t->errors++; continue; }
    acc += v;
    n++;
  }

  if (n == 0U)
  {
    return -1;
  }

  t->vrefint_raw = acc / n;
  t->vdda_mv = __HAL_ADC_CALC_VREFANALOG_VOLTAGE(t->vrefint_raw, ADC_RESOLUTION_12B);
  s_vdda_mv  = t->vdda_mv;

  return 0;
}

/* Re-init one ADC onto the HRTIM trigger and leave it armed. */
static int CSense_ArmTriggered(ADC_HandleTypeDef *hadc, uint32_t channel)
{
  ADC_ChannelConfTypeDef sConfig = {0};

  if (HAL_ADC_Stop(hadc) != HAL_OK) { return -1; }

  hadc->Init.ExternalTrigConv     = ADC_EXTERNALTRIG_HRTIM_TRG1;
  hadc->Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  /* See the note in CSense_Adc5Init - PRESERVED freezes DR under continuous
   * overrun, which is exactly what a 20 kHz trigger produces here. */
  hadc->Init.Overrun              = ADC_OVR_DATA_OVERWRITTEN;
  if (HAL_ADC_Init(hadc) != HAL_OK) { return -1; }

  sConfig.Channel      = channel;
  sConfig.Rank         = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_24CYCLES_5;
  sConfig.SingleDiff   = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset       = 0;
  if (HAL_ADC_ConfigChannel(hadc, &sConfig) != HAL_OK) { return -1; }

  /* Arms the ADC; from here every HRTIM trigger produces one conversion. */
  if (HAL_ADC_Start(hadc) != HAL_OK) { return -1; }

  return 0;
}

int CSense_UseHrtimTrigger(void)
{
  if (CSense_ArmTriggered(&hadc5, ADC_CHANNEL_VOPAMP5)      != 0) { return -1; }
  if (CSense_ArmTriggered(&hadc2, ADC_CHANNEL_VOPAMP3_ADC2) != 0) { return -1; }

  s_triggered = 1;
  return 0;
}

int32_t CSense_RawToMv(uint32_t raw)
{
  return (int32_t)((raw * s_vdda_mv) / (uint32_t)CS_ADC_FULL_SCALE);
}

int32_t CSense_RawToMa(uint32_t raw, uint32_t zero)
{
  /* delta is at most +/-4096, so delta * 24414 stays around 1e8 - well inside
   * an int32_t. */
  int32_t delta = (int32_t)raw - (int32_t)zero;

  return (delta * CS_UA_PER_LSB) / 1000;
}
