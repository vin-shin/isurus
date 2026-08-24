/**
  ******************************************************************************
  * @file    csense.h
  * @brief   Phase current sensing - CT4022-A40BSN8 TMR sensors on U and W.
  *
  *          Both sensors reach the ADC through internal OPAMP followers, not
  *          directly from a pin:
  *
  *            U : PC3 -> OPAMP5_VINP -> (internal) -> ADC5 ADC_CHANNEL_VOPAMP5
  *            W : PA1 -> OPAMP3_VINP -> (internal) -> ADC2 ADC_CHANNEL_VOPAMP3_ADC2
  *
  *          OPAMP5's output is reachable *only* from ADC5, which CubeMX never
  *          configured, so this module brings ADC5 up itself (clock included).
  ******************************************************************************
  */
#ifndef CSENSE_H
#define CSENSE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "main.h"

extern ADC_HandleTypeDef hadc5;

/* CT4022-A40BSN8: 40 A bidirectional, ratiometric on the 3V3 rail.
 * Sensitivity 33 mV/A, so zero current sits at VDDA/2 ~ 1650 mV and full
 * scale is 1650 +/- 1320 mV. */
#define CS_VREF_MV          3300
#define CS_ADC_FULL_SCALE   4096
#define CS_MV_PER_A         33

/* (3300 / 4096) mV per LSB / 33 mV per A = 24.414 mA per LSB.
 * Held in microamps so the conversion stays integer. */
#define CS_UA_PER_LSB       24414

/* Number of samples averaged when capturing the zero-current offset. */
#define CS_ZERO_SAMPLES     256U

typedef struct {
  uint32_t u_raw;      /* last raw ADC code, U phase            */
  uint32_t w_raw;      /* last raw ADC code, W phase            */
  int32_t  u_mv;       /* sensor output in mV, U                */
  int32_t  w_mv;       /* sensor output in mV, W                */
  int32_t  u_ma;       /* current in mA, signed, U              */
  int32_t  w_ma;       /* current in mA, signed, W              */
  uint32_t u_zero;     /* calibrated zero code, U               */
  uint32_t w_zero;     /* calibrated zero code, W               */
  uint32_t samples;    /* successful sample pairs               */
  uint32_t errors;     /* conversion timeouts                   */
  uint32_t vrefint_raw;/* raw VREFINT code, ADC1                */
  uint32_t vdda_mv;    /* measured VDDA / VREF+ in mV           */
  uint32_t vbus_raw;   /* raw ADC1 code, PF0                    */
  uint32_t vbus_mv;    /* DC bus voltage in mV                  */
} CSenseTelem_t;

/* Brings up OPAMP3/OPAMP5, ADC2/ADC5, runs ADC self-calibration, then
 * captures the zero-current offset. The motor must be de-energised and at
 * rest when this runs. Returns 0 on success. */
int CSense_Init(CSenseTelem_t *t);

/* Re-capture the zero-current offset. Motor must be de-energised. */
int CSense_CalibrateZero(CSenseTelem_t *t);

/* Switch ADC2/ADC5 from software start to the HRTIM ADC trigger, so both
 * phases are sampled at the same instant every PWM period. Call *after*
 * MotorPwm_Init(), since the trigger has to exist first. Returns 0 on success. */
int CSense_UseHrtimTrigger(void);

/* Sample both phases and fill in the derived fields. Returns 0 on success. */
int CSense_Read(CSenseTelem_t *t);

/* Raw code -> sensor output voltage in mV, using the measured VDDA rather
 * than the nominal 3300 mV. Falls back to nominal before VDDA is known. */
int32_t CSense_RawToMv(uint32_t raw);

/* DC bus sense: PF0 = ADC1_IN10, behind a 190k/10k divider (so Vbus/20) with
 * a 1k/100pF RC filter. ADC1 is retargeted to it after VDDA is measured at
 * startup, since VREFINT is only needed once. */
#define CS_VBUS_DIV_NUM     200U    /* (190k + 10k) */
#define CS_VBUS_DIV_DEN     10U

int CSense_StartVbus(void);
int CSense_ReadVbus(CSenseTelem_t *t);

/* Measure VDDA via the internal reference and its factory calibration.
 * Result also lands in t->vdda_mv / t->vrefint_raw. */
int CSense_MeasureVdda(CSenseTelem_t *t);

/* Raw code and zero reference -> signed current in mA. */
int32_t CSense_RawToMa(uint32_t raw, uint32_t zero);

#ifdef __cplusplus
}
#endif

#endif /* CSENSE_H */
