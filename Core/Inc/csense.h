/**
  ******************************************************************************
  * @file    csense.h
  * @brief   Analogue front end - three phase currents, DC link current and bus
  *          voltage, all from one HRTIM-triggered ADC1 sequence over DMA.
  *
  *          Sensor outputs are conditioned by op-amps on the board (schematic
  *          sheet 6) and drive the pins directly - unlike Mako Longfin, which
  *          used the MCU's internal OPAMP followers on ADC2 and ADC5.
  *
  *          From the schematic, sheets 2 and 6:
  *
  *            rank 1  PC3  ADC1_IN9   UC_ISNS_U   phase U current
  *            rank 2  PA0  ADC1_IN1   UC_ISNS_W   phase W current
  *            rank 3  PA1  ADC1_IN2   UC_ISNS_DC  DC link current
  *            rank 4  PA3  ADC1_IN4   TS_VSENSE   DC bus voltage
  *
  *          One trigger converts all four, so every quantity the control loop
  *          uses comes from the same instant in the switching period. The
  *          board's own bring-up code left ADC1 free-running and read whatever
  *          DMA had last written, which is fine for a print loop and not for a
  *          current loop - see PWM_ADC_LEAD_NS in motor_pwm.h for the long
  *          version of why sampling position is a correctness requirement.
  *
  * ---------------------------------------------------------------------------
  * There are TWO phase current sensors, not three
  * ---------------------------------------------------------------------------
  *          U and W, plus the DC link. Same shape as Mako Longfin: the third
  *          phase is inferred as -(iu+iw), which is exact in a three-wire
  *          machine.
  *
  *          This module briefly measured three and did a common-mode
  *          correction on iu+iv+iw. That was wrong and worth recording, since
  *          the mistake is easy to repeat: gr_motherfocer's own bring-up code
  *          reads a "V_current" from PA0 and a DC current from PA2, and this
  *          module copied that mapping before the schematic was available.
  *
  *          The schematic says PA0 is UC_ISNS_W and PA2 is NOT CONNECTED. So
  *          the phantom V channel was actually the W sensor read a second
  *          time, U and W were transposed, and the DC link was being read from
  *          an unconnected pin. The common-mode correction then took that
  *          fictional residual and subtracted a third of it from the two real
  *          measurements - actively corrupting good data with noise from a
  *          floating input.
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

/* Ranks in the ADC1 sequence, and so indices into the DMA buffer.
 *
 * PA2 is deliberately absent: the schematic shows it unconnected, and
 * converting it would spend 625 ns per period digitising a floating pin. */
#define CS_IDX_U            0U
#define CS_IDX_W            1U
#define CS_IDX_DC           2U
#define CS_IDX_VBUS         3U
#define CS_SEQ_LEN          4U

typedef struct {
  uint32_t u_raw;      /* last raw ADC code, U phase            */
  uint32_t w_raw;      /* last raw ADC code, W phase            */
  int32_t  u_mv;       /* sensor output in mV, U                */
  int32_t  w_mv;       /* sensor output in mV, W                */
  int32_t  u_ma;       /* current in mA, signed, U              */
  int32_t  w_ma;       /* current in mA, signed, W              */
  int32_t  v_ma;       /* INFERRED, -(iu+iw). Not measured.     */
  int32_t  dc_ma;      /* DC link current in mA, signed         */
  uint32_t u_zero;     /* calibrated zero code, U               */
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

/* v_ma is computed, not measured: -(iu + iw), exact in a three-wire machine
 * because the currents must sum to zero. It is published because a host
 * plotting three phase currents is a normal thing to want, and because
 * leaving it out invites someone to re-add a third sensor that does not
 * exist. With two sensors there is no residual to check it against - an
 * offset on either sensor is indistinguishable from real current, and lands
 * in the Park transform as a once-per-revolution torque ripple. That is a
 * limitation of the hardware, not of this code. */

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

/* Zero-bus ADC code. The isolated sense chain may not put zero volts at code
 * zero - see board.h section 4 - so the conversion carries an offset even
 * though it is 0 until somebody calibrates it. Present and named so that the
 * calibration has an obvious home, rather than being discovered as a missing
 * term later. */
#define CS_VBUS_ZERO_CODE   0

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
