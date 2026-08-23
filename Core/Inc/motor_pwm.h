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

/* 20 kHz switching. HRTIM kernel clock is the APB2 timer clock (128 MHz here,
 * APB2 prescaler is 1), and PRESCALERRATIO_MUL8 gives a 1.024 GHz counter, so
 * one period is 51200 counts and the duty resolution is about 0.98 ns. */
#define PWM_FREQ_HZ         20000U
#define PWM_HRTIM_MUL       8U

/* HRTIM compare registers must be >= 3. A count of 3 is ~2.9 ns, which is
 * below the UCC21330's 5 ns input deglitch filter, so it reads as a true 0%. */
#define PWM_CMP_MIN         3U

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

/* Where in the PWM period the ADC is triggered, as a fraction of the period.
 * Default sits late in the period, after the duty edges have settled, away
 * from switching noise. Tune on a scope. */
#define PWM_ADC_TRIG_PERMILLE  900U

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

/* Drop the gate drivers AND the HRTIM outputs, in that order. The gate line is
 * the only true all-off on this board - see HARDWARE_NOTES section 7. */
void MotorPwm_SafeShutdown(void);

/* Move the HRTIM->ADC trigger within the PWM period, in counts. */
void MotorPwm_SetAdcTriggerPoint(uint32_t counts);

/* Fill in a telemetry snapshot for SWD readout. */
void MotorPwm_GetTelem(MotorPwmTelem_t *t);

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_PWM_H */
