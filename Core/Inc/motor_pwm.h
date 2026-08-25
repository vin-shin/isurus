/**
  ******************************************************************************
  * @file    motor_pwm.h
  * @brief   Three-phase PWM on HRTIM1. Outputs start DISABLED.
  *
  *          One MCU signal per phase; an external inverter on the board derives
  *          the complementary INB for each UCC21330 gate driver, and the
  *          driver's own 20 kohm RDT (~185 ns) provides the dead time. So the
  *          MCU emits three plain single-ended PWMs, no complementary pairs and
  *          no HRTIM dead-time insertion.
  *
  *            U : PA11 -> HRTIM1_CHB2  (Timer B, compare unit 2)
  *            V : PA10 -> HRTIM1_CHB1  (Timer B, compare unit 1)
  *            W : PA9  -> HRTIM1_CHA2  (Timer A, compare unit 1)
  *
  *          U and V share Timer B, which is fine and in fact desirable: all
  *          three phases must share one PWM period anyway. Timer A carries W
  *          and is started in the same register write as Timer B so the two
  *          counters stay locked.
  *
  *          !! SAFETY !! With the external inverter there is NO MCU state that
  *          turns every FET off - a low pin turns the low-side device ON. The
  *          gate drivers' DIS pin is the only true all-off. See HARDWARE_NOTES.
  ******************************************************************************
  */
#ifndef MOTOR_PWM_H
#define MOTOR_PWM_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* 30 kHz switching. HRTIM kernel clock is the APB2 timer clock (128 MHz here,
 * APB2 prescaler is 1), and PRESCALERRATIO_MUL8 gives a 1.024 GHz counter, so
 * one period is 34133 counts and the duty resolution is about 0.98 ns.
 *
 * Raised from 20 kHz, which sits right at the top of the audible range and was
 * part of what could be heard from the motor. 30 kHz is inaudible outright,
 * cuts current ripple by a third (ripple goes as 1/fsw), and costs 1.5x the
 * switching loss - where 40 kHz would have cost 2x and, more to the point, did
 * not fit: the control ISR measures ~28 us and a 40 kHz period is 25 us.
 *
 * ANYTHING that assumes a control-loop rate must derive it from this symbol.
 * Three places used to hardcode 20000 - both FOC integrators and the position
 * loop's output filter - and none of them would have failed loudly; they would
 * have silently run with the wrong gain. */
#define PWM_FREQ_HZ         30000U
#define PWM_HRTIM_MUL       8U

/* Minimum usable compare value.
 *
 * HRTIM will not act on a compare below one full fHRTIM clock period, which at
 * PRESCALERRATIO_MUL8 is 8 counter LSBs. Measured on hardware: CMP <= 7 is
 * silently ignored, so the output is set at the period rollover and NEVER
 * reset - a commanded 0% comes out as a stuck 100%. That is the worst possible
 * failure direction for a motor bridge, so this carries 2x margin.
 *
 * True 0% does not use a small compare at all - MotorPwm_SetDuty clears the
 * output's set-source instead, so it can never go high. See that function. */
#define PWM_CMP_MIN         16U

/* Gate driver enable, PC5, common to all three UCC21330s.
 *
 * PC5 is tied DIRECTLY to the drivers' DIS pins - there is no inverter in this
 * path (unlike the PWM path, which does have one). The UCC21330's DIS pin is
 * active-HIGH-*disable*, so:
 *
 *     PC5 HIGH -> DIS high -> outputs disabled   <- safe state
 *     PC5 LOW  -> DIS low  -> outputs ENABLED
 *
 * Hence this is an active-LOW enable. Do not "tidy" it to 1 because the
 * schematic net is called an enable; the net name is the misleading part. */
#define GATE_EN_ACTIVE_HIGH 0
#define GATE_EN_PORT        GPIOC
#define GATE_EN_PIN         GPIO_PIN_5

/* How long BEFORE the period event the ADC is triggered.
 *
 * This is a DEADLINE, not a preference. The control ISR fires at the period
 * event and reads the ADC data register on its first instruction, so the
 * conversion must already be complete by then. With 8x oversampling at 6.5
 * cycle sampling on a PCLK/4 ADC clock, one reading costs 8 x 19 = 152 ADC
 * cycles, about 4.75 us.
 *
 * Expressed in nanoseconds rather than as a fraction of the period, because
 * the conversion takes a fixed amount of time and does not care how long the
 * period is. This was 900 per-mille, which happened to be 5 us at 20 kHz - but
 * the same 900 per-mille at 30 kHz is only 3.33 us, i.e. less than the
 * conversion needs, and the failure would have been the exact silent one
 * described below. 5000 ns reproduces the validated 20 kHz behaviour bit for
 * bit and stays correct as the switching frequency moves.
 *
 * Do NOT move this to the period event to "sample in the zero vector". That
 * makes the trigger simultaneous with the ISR, so DR is stale by a period or
 * racing the new conversion. It reads perfectly in a static test - a delayed
 * copy of a constant is the same constant - and destabilises the current loop
 * the moment anything moves. That exact mistake cost a debugging session; the
 * symptom was iq tracking ~12% of command with ripple growing as vmax rose.
 *
 * Sampling in the zero vector is not required here anyway: the CT4022s are
 * in-line TMR phase sensors, not shunts, so they read phase current in every
 * switching state. The zero vector only matters for landing on the ripple
 * average, which 900 per-mille approximates well for duty up to ~0.8. */
#define PWM_ADC_LEAD_NS        5000U

/* Counts of lead at the 1.024 GHz HRTIM counter. */
#define PWM_ADC_LEAD_COUNTS    ((PWM_ADC_LEAD_NS * 1024U) / 1000U)

typedef struct {
  uint32_t period;        /* HRTIM period in counts                 */
  uint32_t pwm_hz;        /* computed switching frequency           */
  uint32_t cmp_u;         /* compare value, U                       */
  uint32_t cmp_v;         /* compare value, V                       */
  uint32_t cmp_w;         /* compare value, W                       */
  uint32_t cnt_a;         /* Timer A counter snapshot               */
  uint32_t cnt_b;         /* Timer B counter snapshot               */
  uint32_t outputs_en;    /* 1 once outputs have been enabled       */
  uint32_t oenr;          /* raw HRTIM output enable register       */
  uint32_t gate_en;       /* 1 if the gate drivers are enabled      */
  uint32_t adc_trig_pos;  /* ADC trigger position in counts         */
} MotorPwmTelem_t;

/* Configures the timebase, compare units and output waveforms, then starts the
 * Timer A and Timer B counters. Outputs are left DISABLED - the pins stay in
 * their inactive state and nothing is commanded to the gate drivers.
 * Returns 0 on success. */
int MotorPwm_Init(void);

/* Duty per phase in counts, 0..period. Applied through the preload registers
 * so all three take effect on the same period boundary. */
void MotorPwm_SetDuty(uint32_t u, uint32_t v, uint32_t w);

/* Duty per phase in per-mille (0..1000). Convenience wrapper. */
void MotorPwm_SetDutyPermille(uint32_t u, uint32_t v, uint32_t w);

/* Enable / disable the three HRTIM outputs. Nothing reaches the gate drivers
 * until Enable is called. */
void MotorPwm_EnableOutputs(void);
void MotorPwm_DisableOutputs(void);

/* Gate driver enable (PC5). GateInit drives the line to DISABLED and must run
 * before anything else touches the power stage. */
void MotorPwm_GateInit(void);
void MotorPwm_GateEnable(void);
void MotorPwm_GateDisable(void);
uint32_t MotorPwm_GateIsEnabled(void);

/* Kill the power stage using direct register writes only - no HAL, no
 * interrupts, no assumptions about system state. Safe to call from a fault
 * handler or any other broken context. Asserts DIS on PC5 and disables every
 * HRTIM output. */
void MotorPwm_EmergencyStop(void);

/* Drop the gate drivers AND the HRTIM outputs, in that order. The gate line is
 * the only true all-off on this board - see HARDWARE_NOTES section 7. */
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
