/**
  ******************************************************************************
  * @file    csense.c
  * @brief   Three phase currents, DC link current and bus voltage from one
  *          HRTIM-triggered ADC1 sequence over DMA.
  ******************************************************************************
  */

#include "csense.h"
#include "board.h"
#include "adc.h"
#include "main.h"

#define CS_CONV_TIMEOUT_MS  10U

/* DMA is not in this project's .ioc, so the handle and its bring-up live here
 * rather than in a generated dma.c that a regeneration would overwrite - the
 * same reasoning that kept ADC5 in this file on the previous board. */
static DMA_HandleTypeDef s_hdma_adc1;

/* The conversion buffer DMA writes into. Volatile because DMA writes it
 * behind the CPU's back, and 32-bit wide because the ADC data register is. */
static volatile uint32_t s_seq[CS_SEQ_LEN];

/* Measured reference, used for every mV conversion. Starts at nominal so a
 * read before CSense_MeasureVdda() still returns something sane. */
static uint32_t s_vref_mv = CS_VREF_MV;

/* Microamps of phase current per ADC count, computed once from the MEASURED
 * reference and the sensor's published sensitivity:
 *
 *     uA/count = (VREF_uV / 4096) / (sensitivity in uV per A)
 *
 * Derived rather than written down because the two inputs are of different
 * kinds. The sensitivity is a property of the sensor and comes from its
 * datasheet; the reference is a property of THIS board and is measured at
 * startup. Writing a single amps-per-count literal would bake an assumed
 * reference into a sensor constant, which is how the bus scale came to be 6x
 * wrong before anyone read the divider.
 *
 * It also does the right thing whether or not the sensor is ratiometric. If
 * the output tracks a rail that is also VREF+, both move together and the
 * quotient is unchanged - the cancellation happens numerically. If the output
 * is absolute, or tracks a different rail, the measured reference correctly
 * divides in. The only case this gets wrong is a sensor ratiometric to a rail
 * that is NOT VREF+, which no scaling can fix without measuring that rail. */
static uint32_t s_ua_per_lsb = 0;

static void CSense_ComputeCurrentScale(void)
{
  /* VREF in microvolts is at most a few million, so this stays inside 32
   * bits with room to spare. */
  uint32_t uv_per_lsb = (s_vref_mv * 1000U) / CS_ADC_FULL_SCALE;

  s_ua_per_lsb = (uv_per_lsb * 1000000U) / (uint32_t)BOARD_I_SENS_UV_PER_A
                 / 1000U;
}

/* 0 = free-running (bring-up / calibration), 1 = HRTIM triggered. */
static uint8_t s_triggered = 0;

/* ---------------- ADC1 configuration ---------------- */

/* One channel, software started, no DMA. Used only to read VREFINT before the
 * scan sequence is set up. */
static int CSense_Adc1Single(uint32_t channel, uint32_t *out)
{
  ADC_ChannelConfTypeDef sConfig = {0};

  hadc1.Init.ScanConvMode          = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode    = DISABLE;
  hadc1.Init.NbrOfConversion       = 1;
  hadc1.Init.EOCSelection          = ADC_EOC_SINGLE_CONV;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.ExternalTrigConv      = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge  = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.Overrun               = ADC_OVR_DATA_OVERWRITTEN;
  if (HAL_ADC_Init(&hadc1) != HAL_OK) { return -1; }

  sConfig.Channel      = channel;
  sConfig.Rank         = ADC_REGULAR_RANK_1;
  /* VREFINT sits behind a high source impedance, so a short window reads the
   * sampling capacitor rather than the bandgap. */
  sConfig.SamplingTime = ADC_SAMPLETIME_247CYCLES_5;
  sConfig.SingleDiff   = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset       = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) { return -1; }

  if (HAL_ADC_Start(&hadc1) != HAL_OK) { return -1; }
  if (HAL_ADC_PollForConversion(&hadc1, CS_CONV_TIMEOUT_MS) != HAL_OK)
  {
    (void)HAL_ADC_Stop(&hadc1);
    return -1;
  }
  *out = HAL_ADC_GetValue(&hadc1);
  (void)HAL_ADC_Stop(&hadc1);

  return 0;
}

static int CSense_DmaInit(void)
{
  __HAL_RCC_DMA1_CLK_ENABLE();
  __HAL_RCC_DMAMUX1_CLK_ENABLE();

  s_hdma_adc1.Instance                 = DMA1_Channel1;
  s_hdma_adc1.Init.Request             = DMA_REQUEST_ADC1;
  s_hdma_adc1.Init.Direction           = DMA_PERIPH_TO_MEMORY;
  s_hdma_adc1.Init.PeriphInc           = DMA_PINC_DISABLE;
  s_hdma_adc1.Init.MemInc              = DMA_MINC_ENABLE;
  s_hdma_adc1.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
  s_hdma_adc1.Init.MemDataAlignment    = DMA_MDATAALIGN_WORD;
  /* CIRCULAR, so the sequence refills itself on every trigger and nothing has
   * to re-arm it from the control ISR. */
  s_hdma_adc1.Init.Mode                = DMA_CIRCULAR;
  s_hdma_adc1.Init.Priority            = DMA_PRIORITY_VERY_HIGH;
  if (HAL_DMA_Init(&s_hdma_adc1) != HAL_OK) { return -1; }

  __HAL_LINKDMA(&hadc1, DMA_Handle, s_hdma_adc1);

  return 0;
}

/* The five-channel scan. `triggered` selects free-running (bring-up) or
 * HRTIM-triggered (running). */
static int CSense_Adc1Sequence(uint8_t triggered)
{
  ADC_ChannelConfTypeDef sConfig = {0};
  uint32_t i;

  static const uint32_t chan[CS_SEQ_LEN] = {
    ADC_CHANNEL_9,      /* PC3  phase W */
    ADC_CHANNEL_1,      /* PA0  phase V */
    ADC_CHANNEL_2,      /* PA1  phase U */
    ADC_CHANNEL_3,      /* PA2  DC link */
    ADC_CHANNEL_4       /* PA3  DC bus  */
  };
  static const uint32_t rank[CS_SEQ_LEN] = {
    ADC_REGULAR_RANK_1, ADC_REGULAR_RANK_2, ADC_REGULAR_RANK_3,
    ADC_REGULAR_RANK_4, ADC_REGULAR_RANK_5
  };

  hadc1.Init.ScanConvMode          = ADC_SCAN_ENABLE;
  hadc1.Init.NbrOfConversion       = CS_SEQ_LEN;
  hadc1.Init.EOCSelection          = ADC_EOC_SEQ_CONV;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.ContinuousConvMode    = triggered ? DISABLE : ENABLE;
  hadc1.Init.ExternalTrigConv      = triggered ? ADC_EXTERNALTRIG_HRTIM_TRG1
                                               : ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge  = triggered ? ADC_EXTERNALTRIGCONVEDGE_RISING
                                               : ADC_EXTERNALTRIGCONVEDGE_NONE;
  /* MUST be OVERWRITTEN, not PRESERVED. With DATA_PRESERVED the ADC discards
   * every conversion after the first overrun and the register stays frozen -
   * which reads as a perfectly dead sensor rather than as an error. That cost
   * a debugging session on the previous board; the mechanism is in the ADC,
   * not in either board, so it transfers unchanged. */
  hadc1.Init.Overrun               = ADC_OVR_DATA_OVERWRITTEN;
  if (HAL_ADC_Init(&hadc1) != HAL_OK) { return -1; }

  for (i = 0; i < CS_SEQ_LEN; i++)
  {
    sConfig.Channel      = chan[i];
    sConfig.Rank         = rank[i];
    /* 12.5 cycles, matching the board's own configuration. Five conversions
     * at 12.5 + 12.5 cycles on a 40 MHz ADC clock is 3.125 us, which is what
     * PWM_ADC_LEAD_NS was sized against. Lengthening this without revisiting
     * that lead pushes the conversion past the control ISR that reads it. */
    sConfig.SamplingTime = ADC_SAMPLETIME_12CYCLES_5;
    sConfig.SingleDiff   = ADC_SINGLE_ENDED;
    sConfig.OffsetNumber = ADC_OFFSET_NONE;
    sConfig.Offset       = 0;
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) { return -1; }
  }

  return 0;
}

static int CSense_StartSequence(void)
{
  if (HAL_ADC_Start_DMA(&hadc1, (uint32_t *)s_seq, CS_SEQ_LEN) != HAL_OK)
  {
    return -1;
  }
  return 0;
}

/* ---------------- reference ---------------- */

int CSense_MeasureVdda(CSenseTelem_t *t)
{
  uint32_t raw = 0;

  if (CSense_Adc1Single(ADC_CHANNEL_VREFINT, &raw) != 0) { return -1; }
  if (raw == 0U) { return -1; }

  t->vrefint_raw = raw;

  /* VREF+ = 3.0 V * VREFINT_CAL / VREFINT_measured.
   *
   * The 3000 belongs to the factory datum - the calibration constant in
   * system memory was taken at 3.0 V and 30 C - and is not an assumption
   * about this board's rail. That is the whole point: the rail is what is
   * being measured. */
  t->vdda_mv = (3000U * (uint32_t)(*VREFINT_CAL_ADDR)) / raw;
  s_vref_mv  = t->vdda_mv;

  return 0;
}

/* ---------------- conversions ---------------- */

int32_t CSense_RawToMv(uint32_t raw)
{
  return (int32_t)((raw * s_vref_mv) / CS_ADC_FULL_SCALE);
}

int32_t CSense_RawToMa(uint32_t raw, uint32_t zero)
{
  int32_t d = (int32_t)raw - (int32_t)zero;

  /* Bipolar about the MEASURED zero, so sensor offset, rail tolerance and ADC
   * offset are calibrated out rather than assumed away. Only the SLOPE comes
   * from a constant, and that constant is the sensor's sensitivity - see
   * s_ua_per_lsb above and the datasheet checklist in board.h. */
  return (d * (int32_t)s_ua_per_lsb) / 1000;
}

static uint32_t CSense_RawToVbusMv(uint32_t raw)
{
  /* (raw / 4096) * VREF+ * 400, ordered so the intermediate stays inside 32
   * bits: raw is at most 4095 and the reference a few thousand mV, so the
   * product is about 1.3e7 microvolts and the divide happens before the 400x. */
  uint32_t at_pin_uv = (raw * s_vref_mv * 1000U) / CS_ADC_FULL_SCALE;
  return (at_pin_uv * CS_VBUS_DIV_NUM) / (CS_VBUS_DIV_DEN * 1000U);
}

/* ---------------- public ---------------- */

int CSense_Init(CSenseTelem_t *t)
{
  MX_ADC1_Init();

  /* Self-calibration must happen while the ADC is disabled, i.e. before the
   * first conversion. Skipping it costs several LSB of offset, which on a
   * bipolar current reading is a standing current error rather than noise. */
  if (HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED) != HAL_OK)
  {
    return -1;
  }

  /* Reference first, while the ADC is still a simple single-channel device.
   * Everything below scales against it. */
  if (CSense_MeasureVdda(t) != 0) { return -1; }
  CSense_ComputeCurrentScale();

  if (CSense_DmaInit() != 0)        { return -1; }
  if (CSense_Adc1Sequence(0U) != 0) { return -1; }
  if (CSense_StartSequence() != 0)  { return -1; }
  s_triggered = 0;

  /* Let the free-running sequence fill the buffer before averaging it. */
  HAL_Delay(2);

  return CSense_CalibrateZero(t);
}

int CSense_CalibrateZero(CSenseTelem_t *t)
{
  uint32_t u_acc = 0, v_acc = 0, w_acc = 0, dc_acc = 0;
  uint32_t n = 0;
  uint32_t i;

  for (i = 0; i < CS_ZERO_SAMPLES; i++)
  {
    u_acc  += s_seq[CS_IDX_U];
    v_acc  += s_seq[CS_IDX_V];
    w_acc  += s_seq[CS_IDX_W];
    dc_acc += s_seq[CS_IDX_DC];
    n++;

    /* Space the samples out, so this averages over noise rather than over one
     * conversion read many times. The sequence refreshes every PWM period. */
    {
      volatile uint32_t d;
      for (d = 0; d < 400U; d++) { }
    }
  }

  if (n == 0U) { return -1; }

  t->u_zero  = u_acc  / n;
  t->v_zero  = v_acc  / n;
  t->w_zero  = w_acc  / n;
  t->dc_zero = dc_acc / n;

  return 0;
}

int CSense_UseHrtimTrigger(void)
{
  (void)HAL_ADC_Stop_DMA(&hadc1);

  if (CSense_Adc1Sequence(1U) != 0) { return -1; }
  if (CSense_StartSequence() != 0)  { return -1; }

  s_triggered = 1;
  return 0;
}

int CSense_ReadPhases(CSenseTelem_t *t)
{
  uint32_t u = s_seq[CS_IDX_U];
  /* Reported rather than merely tracked. Free-running conversions still
   * produce plausible-looking currents, so "is the sequence actually locked
   * to the PWM period" is not visible from the values themselves - and it is
   * exactly the thing that reads fine in a static test and destabilises the
   * loop the moment anything moves. See PWM_ADC_LEAD_NS. */
  uint32_t v = s_seq[CS_IDX_V];
  uint32_t w = s_seq[CS_IDX_W];
  int32_t iu, iv, iw, resid, corr;

  t->u_raw = u;
  t->v_raw = v;
  t->w_raw = w;

  iu = CSense_RawToMa(u, t->u_zero);
  iv = CSense_RawToMa(v, t->v_zero);
  iw = CSense_RawToMa(w, t->w_zero);

  /* The three must sum to zero in a three-wire machine. Whatever they do sum
   * to is common-mode error, so it is reported and then divided out across
   * the three. See the note in csense.h: a residual that GROWS WITH CURRENT
   * is a gain mismatch, and this does not fix that one. */
  resid = iu + iv + iw;
  corr  = resid / 3;

  t->resid_ma = resid;
  t->u_ma = iu - corr;
  t->v_ma = iv - corr;
  t->w_ma = iw - corr;

  t->u_mv = CSense_RawToMv(u);
  t->w_mv = CSense_RawToMv(w);

  t->samples++;
  t->triggered = s_triggered;

  return 0;
}

int CSense_Read(CSenseTelem_t *t)
{
  (void)CSense_ReadPhases(t);

  t->dc_ma = CSense_RawToMa(s_seq[CS_IDX_DC], t->dc_zero);

  return CSense_ReadVbus(t);
}

int CSense_StartVbus(void)
{
  /* The bus is rank 5 of the same sequence, so there is nothing to start.
   * Kept because the bring-up path and drive.c both call it, and a board
   * where it IS a separate conversion should not force the call sites to
   * change. */
  return 0;
}

int CSense_ReadVbus(CSenseTelem_t *t)
{
  t->vbus_raw = s_seq[CS_IDX_VBUS];
  t->vbus_mv  = CSense_RawToVbusMv(t->vbus_raw);
  return 0;
}
