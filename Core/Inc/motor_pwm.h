/**
  ******************************************************************************
  * @file    motor_pwm.h
  * @brief   Three-phase PWM on HRTIM1. Outputs start DISABLED.
  *
  *          COMPLEMENTARY pairs with hardware dead-time insertion, one HRTIM
  *          timer per phase. From the schematic, sheet 2:
  *
  *            U : PA11 HRTIM1_CHB2 = HG_U   PA10 HRTIM1_CHB1 = LG_U   Timer B
  *            V : PC7  HRTIM1_CHF2 = HG_V   PC6  HRTIM1_CHF1 = LG_V   Timer F
  *            W : PB13 HRTIM1_CHC2 = HG_W   PB12 HRTIM1_CHC1 = LG_W   Timer C
  *
  *          !! THE LOW SIDE IS ON OUTPUT 1, NOT THE HIGH SIDE. !!
  *
  *          This was the other way round here until the schematic was read,
  *          and it is worth being explicit about why it matters. Only output 1
  *          of each pair is given set and reset sources; output 2 is generated
  *          from it by the dead-time unit, which is what makes the pair
  *          genuinely complementary - two independently programmed waveforms
  *          that happen to be inverses would have no guaranteed dead band.
  *
  *          So output 1 IS the reference waveform, and on this board output 1
  *          is the LOW gate. A duty computed for the high side and written
  *          straight to output 1 lands on the low side instead, and the
  *          inverter output is inverted: commanded duty D produces (1-D) at
  *          the phase. All three phases inverted negates the applied voltage
  *          vector, which turns the current loop into positive feedback.
  *
  *          The fix is in MotorPwm_ApplyPhase, not here: the set and reset
  *          sources are swapped so output 1 is LOW across the middle of the
  *          period, which makes its complement - the high gate - a centred
  *          pulse of the commanded width. Nothing about the compare values,
  *          the centring or the ADC trigger position changes.
  *
  *          Polarity inversion would NOT have been a valid fix. Dead time
  *          works by making both outputs INACTIVE during the dead band; invert
  *          the polarity and "inactive" becomes high, so both gates would be
  *          driven at once. That is the shoot-through the dead time exists to
  *          prevent.
  *
  *          The three timers are independent but share a prescaler and period,
  *          and are started in one register write so their counters stay
  *          locked. All three phases must share one PWM period.
  *
  *          !! SAFETY, and this is a CHANGE FROM MAKO LONGFIN !!
  *
  *          On Mako Longfin one MCU pin per phase fed an external inverter, so
  *          a low pin turned the low-side device ON and no MCU state turned
  *          every FET off - the gate drivers' DIS line was the only true
  *          all-off. That is NOT the situation here. This board's HRTIM drives
  *          both devices of each leg directly, so disabling the HRTIM outputs
  *          drops both gates and genuinely opens the bridge.
  *
  *          What has NOT changed is what zero duty means. Zero duty holds the
  *          high side off and, through the dead-time unit, the low side on -
  *          all three phases tied to the negative rail, which is a BRAKE, not
  *          a coast. Coasting means disabling the outputs. See
  *          MotorPwm_SafeShutdown.
  ******************************************************************************
  */
#ifndef MOTOR_PWM_H
#define MOTOR_PWM_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "board.h"

/* 20 kHz switching. HRTIM kernel clock is the APB2 timer clock (160 MHz here,
 * APB2 prescaler is 1), and PRESCALERRATIO_MUL8 gives a 1.28 GHz counter, so
 * one period is 64000 counts and the duty resolution is about 0.78 ns.
 *
 * Mako Longfin ran at 30 kHz, having deliberately moved up from 20 kHz to get
 * out of the audible band. This board is at 20 kHz because that is what its
 * hardware was configured around - the dead time, the ADC sampling times and
 * whatever filtering sits on the current sense. None of those have been
 * measured here, so 20 kHz is where the port starts, not a finding that
 * 20 kHz is right. Moving it is a bench exercise.
 *
 * ANYTHING that assumes a control-loop rate must derive it from this symbol.
 * Three places used to hardcode 20000 - both FOC integrators and the position
 * loop's output filter - and none of them would have failed loudly; they would
 * have silently run with the wrong gain. */
#define PWM_FREQ_HZ         BOARD_PWM_FREQ_HZ
#define PWM_HRTIM_MUL       BOARD_HRTIM_MUL

/* Minimum usable compare value.
 *
 * HRTIM will not act on a compare below one full fHRTIM clock period, which at
 * PRESCALERRATIO_MUL8 is 8 counter LSBs. Measured on Mako Longfin: CMP <= 7 is
 * silently ignored, so the output is set at the period rollover and NEVER
 * reset - a commanded 0% comes out as a stuck 100%. That is the worst possible
 * failure direction for a motor bridge, so this carries 2x margin.
 *
 * The mechanism is in the HRTIM itself, not in either board, so it transfers
 * unchanged. True 0% does not use a small compare at all - MotorPwm_SetDuty
 * clears the output's set-source instead, so it can never go high. */
#define PWM_CMP_MIN         16U

/* Dead time. Both edges, in counts of the 1.28 GHz HRTIM clock divided by the
 * dead-time prescaler. See board.h for why this is not written in nanoseconds
 * anywhere: the divider chain has not been confirmed against RM0440 and the
 * result has not been seen on a scope, and a dead time that is wrong in the
 * short direction is a shoot-through. */
#define PWM_DT_RISING       BOARD_DT_RISING
#define PWM_DT_FALLING      BOARD_DT_FALLING

/* Gate driver enable, PC8.
 *
 * !! ACTIVE HIGH on this board, which is the OPPOSITE of Mako Longfin, where
 * PC5 drove a UCC21330 DIS pin directly and low meant enabled. Three separate
 * places in this board's own bring-up code agree on the polarity: the reset
 * state is low, the disable routine drives low, and the reset routine ends by
 * driving high. See board.h section 3.
 *
 * Assuming the old polarity here energises the bridge at reset. */
#define GATE_EN_ACTIVE_HIGH BOARD_GATE_EN_ACTIVE_HIGH
#define GATE_EN_PORT        BOARD_GATE_EN_PORT
#define GATE_EN_PIN         BOARD_GATE_EN_PIN

/* Single-store "drive the gate line to its disabled state", for use in an
 * emergency stop where nothing about the system can be trusted. BSRR sets on
 * the low half-word and resets on the high half-word, so the polarity has to
 * be resolved at compile time rather than branched on. */
#if GATE_EN_ACTIVE_HIGH
#define GATE_EN_BSRR_DISABLE  ((uint32_t)GATE_EN_PIN << 16U)
#else
#define GATE_EN_BSRR_DISABLE  ((uint32_t)GATE_EN_PIN)
#endif

/* How long BEFORE the period event the ADC is triggered.
 *
 * This is a DEADLINE, not a preference. The control ISR fires at the period
 * event and reads the sampled currents on its first instruction, so the
 * conversion must already be complete by then.
 *
 * On this board ADC1 converts five channels - three phase currents, DC link
 * current, bus voltage - at 12.5 cycle sampling on a PCLK/4 (40 MHz) ADC
 * clock. One 12-bit conversion is 12.5 + 12.5 = 25 cycles, so the sequence
 * costs 125 cycles, or 3.125 us. The 5000 ns lead below covers that with
 * about 60% margin and is 10% of the 20 kHz period.
 *
 * Expressed in nanoseconds rather than as a fraction of the period, because
 * the conversion takes a fixed amount of time and does not care how long the
 * period is. It was once 900 per-mille, which happened to be 5 us at 20 kHz -
 * but the same 900 per-mille at 30 kHz is only 3.33 us, and the failure would
 * have been the silent one described below.
 *
 * Do NOT move this to the period event to "sample in the zero vector". That
 * makes the trigger simultaneous with the ISR, so the data is stale by a
 * period or racing the new conversion. It reads perfectly in a static test - a
 * delayed copy of a constant is the same constant - and destabilises the
 * current loop the moment anything moves. That exact mistake cost a debugging
 * session on Mako Longfin; the symptom was iq tracking ~12% of command with
 * ripple growing as vmax rose.
 *
 * NOTE that the sampling argument itself does not transfer. Mako Longfin used
 * in-line TMR phase sensors, which read phase current in every switching
 * state, so where in the period it sampled only affected how well it landed on
 * the ripple average. This board's current sense topology is not documented in
 * the material available - if it turns out to be low-side shunts, sampling
 * position stops being a refinement and becomes a correctness requirement. */
#define PWM_ADC_LEAD_NS        5000U

/* Counts of lead, at whatever the HRTIM counter is actually running at.
 *
 * This used to be written as (ns * 1024) / 1000, with the 1024 being counts
 * per microsecond at Mako Longfin's 128 MHz. That constant is 1280 here, and
 * nothing would have complained about the old one - the trigger would simply
 * have sat 22% closer to the period event than intended. Derived from
 * BOARD_HRTIM_TICK_HZ so it follows the clock instead. */
#define PWM_COUNTS_PER_US      (BOARD_HRTIM_TICK_HZ / 1000000U)
#define PWM_ADC_LEAD_COUNTS    ((PWM_ADC_LEAD_NS * PWM_COUNTS_PER_US) / 1000U)

typedef struct {
  uint32_t period;        /* HRTIM period in counts                 */
  uint32_t pwm_hz;        /* computed switching frequency           */
  uint32_t cmp_u;         /* compare value, U (Timer B)             */
  uint32_t cmp_v;         /* compare value, V (Timer F)             */
  uint32_t cmp_w;         /* compare value, W (Timer C)             */
  uint32_t cnt_u;         /* Timer B counter snapshot               */
  uint32_t cnt_v;         /* Timer F counter snapshot               */
  uint32_t outputs_en;    /* 1 once outputs have been enabled       */
  uint32_t oenr;          /* raw HRTIM output enable register       */
  uint32_t gate_en;       /* 1 if the gate drivers are enabled      */
  uint32_t adc_trig_pos;  /* ADC trigger position in counts         */
} MotorPwmTelem_t;

/* Configures the timebase, compare units, dead time and output waveforms, then
 * starts the three counters. Outputs are left DISABLED - the pins stay
 * inactive and nothing is commanded to the gate drivers. Returns 0 on
 * success. */
int MotorPwm_Init(void);

/* Duty per phase in counts, 0..period, referred to the HIGH side - so 0 means
 * the phase is clamped to the negative rail and `period` means the positive
 * one, whichever output the timer happens to drive. Applied through the
 * preload registers so all three take effect on the same period boundary. */
void MotorPwm_SetDuty(uint32_t u, uint32_t v, uint32_t w);

/* Duty per phase in per-mille (0..1000). Convenience wrapper. */
void MotorPwm_SetDutyPermille(uint32_t u, uint32_t v, uint32_t w);

/* Enable / disable all six HRTIM outputs. Nothing reaches the gate drivers
 * until Enable is called. Unlike on Mako Longfin, Disable here really does
 * open the whole bridge. */
void MotorPwm_EnableOutputs(void);
void MotorPwm_DisableOutputs(void);

/* Gate driver enable (PC8, active high). GateInit drives the line to DISABLED
 * and must run before anything else touches the power stage. */
void MotorPwm_GateInit(void);
void MotorPwm_GateEnable(void);
void MotorPwm_GateDisable(void);
uint32_t MotorPwm_GateIsEnabled(void);

/* Kill the power stage using direct register writes only - no HAL, no
 * interrupts, no assumptions about system state. Safe to call from a fault
 * handler or any other broken context. Drives PC8 to disabled and disconnects
 * every HRTIM output. */
void MotorPwm_EmergencyStop(void);

/* Drop the HRTIM outputs AND the gate drivers. Either one alone opens the
 * bridge on this board; both are done because the ordering costs nothing and
 * the redundancy is free. */
void MotorPwm_SafeShutdown(void);

/* Move the HRTIM->ADC trigger within the PWM period, in counts. */
void MotorPwm_SetAdcTriggerPoint(uint32_t counts);

/* Enable/disable the HRTIM period interrupt that drives the control loop. */
void MotorPwm_EnableControlIsr(void);
void MotorPwm_DisableControlIsr(void);

/* Set duties from normalised 0..1 values (what FOC produces). */
void MotorPwm_SetDutyNorm(float u, float v, float w);

/* PWM period in counts. Valid immediately after MotorPwm_Init, unlike the
 * telemetry struct which is only populated once the main loop runs. */
uint32_t MotorPwm_GetPeriod(void);

/* Fill in a telemetry snapshot for SWD readout. */
void MotorPwm_GetTelem(MotorPwmTelem_t *t);

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_PWM_H */
