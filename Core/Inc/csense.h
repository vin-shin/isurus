/**
  ******************************************************************************
  * @file    csense.h
  * @brief   Analogue front end - three phase currents, DC link current and bus
  *          voltage, all from one HRTIM-triggered ADC1 sequence over DMA.
  *
  *          A different shape from Mako Longfin, which polled two phases
  *          through internal OPAMP followers on ADC2 and ADC5. There are no
  *          OPAMPs in this path: the sensors drive the pins directly.
  *
  *            rank 1  PC3  ADC1_IN9   phase W current
  *            rank 2  PA0  ADC1_IN1   phase V current
  *            rank 3  PA1  ADC1_IN2   phase U current
  *            rank 4  PA2  ADC1_IN3   DC link current
  *            rank 5  PA3  ADC1_IN4   DC bus voltage
  *
  *          One trigger converts all five, so every quantity the control loop
  *          uses comes from the same instant in the switching period. The
  *          board's own bring-up code left ADC1 free-running and read whatever
  *          DMA had last written, which is fine for a print loop and not for a
  *          current loop - see PWM_ADC_LEAD_NS in motor_pwm.h for the long
  *          version of why sampling position is a correctness requirement.
  *
  * ---------------------------------------------------------------------------
  * Three phases measured, two handed on
  * ---------------------------------------------------------------------------
  *          Mako Longfin measured two and inferred the third. This board
  *          measures all three, which makes the set overdetermined: iu+iv+iw
  *          must be zero in a three-wire machine, so whatever they actually
  *          sum to is common-mode error - sensor offset drift, a shifted
  *          reference, thermal drift since the last zero capture.
  *
  *          That sum is divided out across the three before anything else
  *          sees them. It costs two adds and a multiply and it removes the
  *          error term that a two-phase measurement has no way to observe:
  *          with two sensors, an offset on either one is indistinguishable
  *          from real current, and it lands in the Park transform as a
  *          once-per-revolution torque ripple.
  *
  *          FOC still receives U and W, so foc.c is untouched and its tests
  *          stay valid. The third sensor buys accuracy, not a new interface.
  ******************************************************************************
  */
#ifndef CSENSE_H
#define CSENSE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "main.h"
#include "board.h"

/* Number of samples averaged when capturing the zero-current offset. */
#define CS_ZERO_SAMPLES     BOARD_ADC_ZERO_SAMPLES

#define CS_ADC_FULL_SCALE   4096U
#define CS_ADC_MID          (CS_ADC_FULL_SCALE / 2U)

/* Fallback reference until VREFINT has been read. See CSense_MeasureVdda. */
#define CS_VREF_MV          BOARD_VREF_NOMINAL_MV

/* Ranks in the ADC1 sequence, and so indices into the DMA buffer. */
#define CS_IDX_W            0U
#define CS_IDX_V            1U
#define CS_IDX_U            2U
#define CS_IDX_DC           3U
#define CS_IDX_VBUS         4U
#define CS_SEQ_LEN          5U

typedef struct {
  uint32_t u_raw;      /* last raw ADC code, U phase            */
  uint32_t v_raw;      /* last raw ADC code, V phase            */
  uint32_t w_raw;      /* last raw ADC code, W phase            */
  int32_t  u_mv;       /* sensor output in mV, U                */
  int32_t  w_mv;       /* sensor output in mV, W                */
  int32_t  u_ma;       /* current in mA, signed, U              */
  int32_t  v_ma;       /* current in mA, signed, V              */
  int32_t  w_ma;       /* current in mA, signed, W              */
  int32_t  dc_ma;      /* DC link current in mA, signed         */
  int32_t  resid_ma;   /* iu+iv+iw before correction - see below*/
  uint32_t u_zero;     /* calibrated zero code, U               */
  uint32_t v_zero;     /* calibrated zero code, V               */
  uint32_t w_zero;     /* calibrated zero code, W               */
  uint32_t dc_zero;    /* calibrated zero code, DC link         */
  uint32_t samples;    /* successful sample sets                */
  uint32_t errors;     /* conversion timeouts / DMA stalls      */
  uint32_t vrefint_raw;/* raw VREFINT code                      */
  uint32_t vdda_mv;    /* measured VREF+ in mV                  */
  uint32_t vbus_raw;   /* raw ADC1 code, PA3                    */
  uint32_t vbus_mv;    /* DC bus voltage in mV                  */
  uint32_t triggered;  /* 1 once the sequence is HRTIM-triggered */
} CSenseTelem_t;

/* resid_ma is the diagnostic that the third sensor makes possible: the sum of
 * the three phase currents, which is zero in a healthy three-wire machine.
 *
 * It is worth watching rather than merely correcting. A steady residual is
 * offset drift and is exactly what the correction removes. A residual that
 * grows with current is a GAIN mismatch between sensors, which the correction
 * cannot fix and which no amount of zero calibration will touch. A residual
 * that appears suddenly is a sensor or a phase connection that has failed.
 * None of those are observable at all with only two sensors. */

/* Brings up ADC1 with its five-channel sequence and DMA, runs ADC
 * self-calibration, measures VREF+ against VREFINT, then captures the
 * zero-current offsets. The motor must be de-energised and at rest when this
 * runs. Returns 0 on success. */
int CSense_Init(CSenseTelem_t *t);

/* Re-capture the zero-current offsets. Motor must be de-energised. */
int CSense_CalibrateZero(CSenseTelem_t *t);

/* Switch ADC1 from free-running to the HRTIM ADC trigger, so the whole
 * sequence is converted from one instant every PWM period. Call *after*
 * MotorPwm_Init(), since the trigger has to exist first. Returns 0. */
int CSense_UseHrtimTrigger(void);

/* THE ISR PATH. Three phase currents only: buffer read, three conversions,
 * and the common-mode correction. Returns 0.
 *
 * Split out from CSense_Read because the control loop needs the phases every
 * period and needs nothing else. Bus voltage feeds the gain rescale and the
 * over/under-voltage trips, which run in the main loop at a few hundred hertz;
 * DC link current is telemetry. Converting those at 20 kHz would be two
 * multiplies and two divides per period spent on values nobody reads that
 * often, inside a loop with a hard 50 us deadline. */
int CSense_ReadPhases(CSenseTelem_t *t);

/* The main-loop path: everything CSense_ReadPhases does, plus DC link current
 * and bus voltage. Returns 0. */
int CSense_Read(CSenseTelem_t *t);

/* Raw code -> sensor output voltage in mV, using the measured VREF+ rather
 * than the nominal. Falls back to nominal before VREF+ is known. */
int32_t CSense_RawToMv(uint32_t raw);

/* ---- DC bus ------------------------------------------------------------- *
 *
 * PA3 = ADC1_IN4, behind a 400:1 divider: 4M7 + 4M7 + 560k + 15k over 25k,
 * which is 9.975 MOhm over 25 kOhm and totals exactly 10 MOhm. A full 588 V
 * pack presents 1.47 V at the pin.
 *
 * The four-part top leg is HV practice rather than redundancy - each resistor
 * drops under 300 V at a full pack and the creepage is spread over four
 * footprints.
 *
 * !! The constant this replaces was 0.05 V per count, which implies a 62:1
 * divider and would have reported that 588 V pack as 91 V. FOC_SetGainsForVbus
 * DIVIDES by this number, so the whole current loop would have run 6.4x over
 * gain. Do not reintroduce a volts-per-count literal here; it is derived from
 * the divider and the MEASURED reference for exactly that reason. */
#define CS_VBUS_DIV_NUM     BOARD_VBUS_DIV_NUM
#define CS_VBUS_DIV_DEN     BOARD_VBUS_DIV_DEN

/* Both retained for the existing call sites. The bus is part of the same
 * sequence now, so starting it is a no-op and reading it is a buffer copy. */
int CSense_StartVbus(void);
int CSense_ReadVbus(CSenseTelem_t *t);

/* Measure VREF+ via the internal reference and its factory calibration.
 * Result lands in t->vdda_mv / t->vrefint_raw.
 *
 * Not optional on this board. VREFBUF is disabled and VREF+ comes from an
 * external part this project has never seen; the plausible references span a
 * 60% difference in volts per count, which is the same class of error as the
 * 0.05 above. VREFINT is an on-die bandgap with a factory calibration
 * constant in system memory, so the answer is available for the asking. */
int CSense_MeasureVdda(CSenseTelem_t *t);

/* Raw code and zero reference -> signed current in mA. */
int32_t CSense_RawToMa(uint32_t raw, uint32_t zero);

#ifdef __cplusplus
}
#endif

#endif /* CSENSE_H */
