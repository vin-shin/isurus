/**
  ******************************************************************************
  * @file    thermal.c
  * @brief   Motor and power-stage temperature, from ADC2 over DMA.
  ******************************************************************************
  */

#include "thermal.h"
#include "csense.h"
#include "adc.h"
#include "main.h"

static DMA_HandleTypeDef s_hdma_adc2;
static volatile uint32_t s_seq[TH_SEQ_LEN];

static int Thermal_DmaInit(void)
{
  __HAL_RCC_DMA2_CLK_ENABLE();
  __HAL_RCC_DMAMUX1_CLK_ENABLE();

  s_hdma_adc2.Instance                 = DMA2_Channel1;
  s_hdma_adc2.Init.Request             = DMA_REQUEST_ADC2;
  s_hdma_adc2.Init.Direction           = DMA_PERIPH_TO_MEMORY;
  s_hdma_adc2.Init.PeriphInc           = DMA_PINC_DISABLE;
  s_hdma_adc2.Init.MemInc              = DMA_MINC_ENABLE;
  s_hdma_adc2.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
  s_hdma_adc2.Init.MemDataAlignment    = DMA_MDATAALIGN_WORD;
  s_hdma_adc2.Init.Mode                = DMA_CIRCULAR;
  /* LOW priority, deliberately. This shares the DMA controller with the
   * current sense, which has a deadline; temperature does not care whether it
   * is serviced this period or the next. */
  s_hdma_adc2.Init.Priority            = DMA_PRIORITY_LOW;
  if (HAL_DMA_Init(&s_hdma_adc2) != HAL_OK) { return -1; }

  __HAL_LINKDMA(&hadc2, DMA_Handle, s_hdma_adc2);

  return 0;
}

int Thermal_Init(ThermalTelem_t *t)
{
  ADC_ChannelConfTypeDef sConfig = {0};
  uint32_t i;

  /* The KTY only. PA5/PA6/PA7 are APWM, not analogue - see thermal.h. */
  static const uint32_t chan[TH_SEQ_LEN] = {
    ADC_CHANNEL_5       /* PC4  motor KTY */
  };
  static const uint32_t rank[TH_SEQ_LEN] = {
    ADC_REGULAR_RANK_1
  };

  MX_ADC2_Init();

  hadc2.Init.ScanConvMode          = ADC_SCAN_ENABLE;
  hadc2.Init.NbrOfConversion       = TH_SEQ_LEN;
  hadc2.Init.EOCSelection          = ADC_EOC_SEQ_CONV;
  hadc2.Init.ContinuousConvMode    = ENABLE;
  hadc2.Init.DMAContinuousRequests = ENABLE;
  hadc2.Init.ExternalTrigConv      = ADC_SOFTWARE_START;
  hadc2.Init.ExternalTrigConvEdge  = ADC_EXTERNALTRIGCONVEDGE_NONE;
  /* Same reasoning as ADC1: PRESERVED freezes the data register on the first
   * overrun, and a frozen register reads as a working sensor reporting a
   * constant. Which, for a temperature, is the most dangerous constant there
   * is - it reads cold forever. */
  hadc2.Init.Overrun               = ADC_OVR_DATA_OVERWRITTEN;
  if (HAL_ADC_Init(&hadc2) != HAL_OK) { return -1; }

  if (HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED) != HAL_OK)
  {
    return -1;
  }

  for (i = 0; i < TH_SEQ_LEN; i++)
  {
    sConfig.Channel      = chan[i];
    sConfig.Rank         = rank[i];
    /* 92.5 cycles, as the board configured it. These are slow, high-impedance
     * sources with no deadline: a short window would read the sampling
     * capacitor rather than the sensor. */
    sConfig.SamplingTime = ADC_SAMPLETIME_92CYCLES_5;
    sConfig.SingleDiff   = ADC_SINGLE_ENDED;
    sConfig.OffsetNumber = ADC_OFFSET_NONE;
    sConfig.Offset       = 0;
    if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK) { return -1; }
  }

  if (Thermal_DmaInit() != 0) { return -1; }

  if (HAL_ADC_Start_DMA(&hadc2, (uint32_t *)s_seq, TH_SEQ_LEN) != HAL_OK)
  {
    return -1;
  }

  t->valid        = 0U;
  t->samples      = 0U;
  t->range_errors = 0U;

  return 0;
}

uint32_t Thermal_RawToOhm(uint32_t raw)
{
  /* !! THIS MODEL IS KNOWN TO BE THE WRONG SHAPE FOR THIS BOARD. !!
   *
   * Schematic sheet 9 is a two-stage TLV9302 circuit driving the KTY node,
   * not a passive pull-up, so the relationship is linear rather than the
   * ratiometric divider below. Replace this function - do not retune
   * THERM_PULLUP_OHM. See the header comment in thermal.h.
   *
   * Left in place so the module compiles and reports something monotonic in
   * temperature while the front end is worked out. The band check in
   * Thermal_Read is what stops it being trusted meanwhile.
   *
   * Assumed conditioning: KTY from the pin to ground, pull-up to the
   * reference. Then
   *
   *     raw / FULL = R / (R + Rpull)   ->   R = Rpull * raw / (FULL - raw)
   *
   * The reference cancels out of that entirely, which is the one good thing
   * about this topology: it does not matter what VREF+ turns out to be. What
   * DOES matter is Rpull, and that is the placeholder. */
  if (raw >= CS_ADC_FULL_SCALE - 1U) { return 0xFFFFFFFFU; }

  return ((uint32_t)THERM_PULLUP_OHM * raw) / (CS_ADC_FULL_SCALE - raw);
}

int32_t Thermal_OhmToCx10(uint32_t ohm)
{
  /* Invert R(T) = R25 * (1 + a*dT + b*dT^2) for dT.
   *
   * Solved by bisection rather than by the quadratic formula. The formula
   * needs a square root and careful handling of the branch, all in fixed
   * point, to save an iteration count that is bounded at 16 and runs in the
   * main loop where nothing is waiting for it. Bisection is obviously correct
   * by inspection, which the alternative is not.
   *
   * Range is clamped to -55..250 C, past the part's usable span in both
   * directions, so a reading outside it saturates rather than wrapping - and
   * the caller sees the clamp as an out-of-range fault via TH_RAW_MIN/MAX
   * rather than as a plausible number. */
  int32_t lo = -550;    /* -55.0 C, in tenths */
  int32_t hi = 2500;    /* 250.0 C            */
  int32_t i;

  for (i = 0; i < 16; i++)
  {
    int32_t mid = (lo + hi) / 2;
    int32_t dt  = mid - 250;            /* tenths of a degree from 25 C */

    /* R25 * (1 + a*dT + b*dT^2), all in tenths of a degree.
     *
     * a is 7.874e-3 per K, so per tenth-K it is 7.874e-4; held as
     * TH_KTY_A_E6 / 1e6 per K and scaled here. dt is at most 2250 tenths, so
     * dt*dt is 5.06e6 and the b term stays well inside an int32 once scaled. */
    int32_t lin  = ((int32_t)TH_KTY_A_E6 * dt) / 10000;        /* x1e3 */
    int32_t quad = ((int32_t)TH_KTY_B_E6 * ((dt * dt) / 100)) / 1000; /* x1e3 */
    int32_t r    = (int32_t)TH_KTY_R25_OHM
                   + (((int32_t)TH_KTY_R25_OHM * (lin + quad)) / 1000);

    if ((uint32_t)((r < 0) ? 0 : r) < ohm) { lo = mid; }
    else                                   { hi = mid; }
  }

  return (lo + hi) / 2;
}

int Thermal_Read(ThermalTelem_t *t)
{
  uint32_t i;
  uint32_t raw;

  for (i = 0; i < TH_SEQ_LEN; i++) { t->raw[i] = s_seq[i]; }

  t->samples++;

  raw = t->raw[TH_IDX_MOTOR];
  if ((raw < TH_RAW_MIN) || (raw > TH_RAW_MAX))
  {
    /* Open sensor, shorted sensor, or conditioning that is not what
     * THERM_PULLUP_OHM assumes. Reported as invalid rather than converted -
     * the point of the band is that a broken sensor should NOT produce a
     * plausible temperature. */
    t->valid = 0U;
    t->range_errors++;
    return -1;
  }

  t->motor_ohm   = Thermal_RawToOhm(raw);
  t->motor_c_x10 = Thermal_OhmToCx10(t->motor_ohm);
  t->valid       = 1U;

  return 0;
}
