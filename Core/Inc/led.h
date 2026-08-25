/**
  ******************************************************************************
  * @file    led.h
  * @brief   The two-LED front panel: PB1 and PB2.
  *
  *          Once this board leaves the bench there is no debugger, no UART and
  *          no CAN host - two LEDs are the entire user interface. They have to
  *          answer the only three questions anyone actually asks of a drive:
  *
  *              is it alive?
  *              is the power stage live? (the one that can hurt you)
  *              if it is not working, why?
  *
  *          Both are plain push-pull outputs on `main`. PB1's TIM3_CH4 PWM
  *          exists only on the hw-verification branch, so this scheme uses no
  *          brightness at all - two binary lamps, which is also the version
  *          that still works if a timer is ever needed elsewhere.
  *
  *  ---- PB1, the power stage, driven from the CONTROL ISR -----------------
  *
  *          Gates OFF:  a 40 ms flash once a second.
  *          Gates ON:   INVERTED - lit continuously, with a 40 ms notch once
  *                      a second.
  *
  *          Two things at once. The duty says whether the bridge is energised,
  *          and mostly-lit for "live" is deliberate: it is the reading you get
  *          from the corner of your eye, and the dangerous state should be the
  *          bright one. The blink says the 30 kHz control ISR is still running,
  *          because this is stepped FROM that ISR - a lamp driven by the main
  *          loop would keep winking cheerfully while the control loop was dead
  *          and the bridge sat on stale duties, which is exactly the failure
  *          the windowed watchdog exists for.
  *
  *          It reads MotorPwm_GateIsEnabled() rather than any state variable,
  *          so it reports the hardware and cannot be desynchronised from it by
  *          a tool poking flags over SWD.
  *
  *  ---- PB2, status, driven from the MAIN LOOP ----------------------------
  *
  *          Blink codes, all on a repeating cycle:
  *
  *              INIT       solid
  *              SELFTEST   fast flicker, ~10 Hz
  *              READY      1 blip, then a pause
  *              RUN        2 blips, then a pause
  *              FAULT      N blips, then a pause, where N is the fault cause
  *                         from DriveFault_t - 1 overcurrent, 2 overvoltage,
  *                         3 undervoltage, 4 encoder, 5 current sense,
  *                         6 self-test, 7 watchdog, 8 command
  *
  *          Counting flashes is a poor interface and the right one here: it
  *          needs no second device, survives being read across a workshop, and
  *          the cause is the thing a bench note can be written against.
  *
  *  ---- reading the pair together -----------------------------------------
  *
  *          Splitting them across the two contexts makes the pair diagnostic
  *          in a way neither is alone:
  *
  *              PB1 blinking, PB2 blinking    both loops running
  *              PB1 blinking, PB2 frozen      main loop hung, ISR fine
  *              PB1 frozen,   PB2 blinking    ISR hung - though the watchdog
  *                                            should have reset the board
  *                                            first and PB2 would then be
  *                                            showing fault code 7
  *              both dark                     no firmware at all: bad boot,
  *                                            held in reset, or no 3V3
  ******************************************************************************
  */
#ifndef LED_H
#define LED_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* PB1 timing, in control-ISR ticks. Derived from PWM_FREQ_HZ so the pattern
 * stays one second long whatever the switching frequency becomes. */
#define LED_STAGE_PERIOD_MS   1000U
#define LED_STAGE_PULSE_MS      40U

/* PB2 blink-code timing, ms. A blip has to be long enough to count reliably
 * by eye and short enough that eight of them fit in a cycle someone will wait
 * through: 8 blips is 8*(140+220) + 1100 = 4.0 s worst case. */
#define LED_BLIP_ON_MS         140U
#define LED_BLIP_OFF_MS        220U
#define LED_BLIP_GAP_MS       1100U
#define LED_FLICKER_MS          50U   /* SELFTEST half-period, ~10 Hz */

void Led_Init(void);

/* Step PB1. Call every control ISR tick; it is a counter, a compare and one
 * BSRR store. */
void Led_StepIsr(void);

/* Step PB2. Call from the main loop with HAL_GetTick(). */
void Led_StepMain(uint32_t now_ms);

#ifdef __cplusplus
}
#endif

#endif /* LED_H */
