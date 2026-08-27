/**
  ******************************************************************************
  * @file    led.c
  * @brief   The two-LED front panel. See led.h for the scheme.
  ******************************************************************************
  */

#include "led.h"
#include "board.h"
#include "main.h"
#include "motor_pwm.h"
#include "drive.h"

#define LED_PORT       GPIOB
#define LED_STAGE_PIN  GPIO_PIN_1    /* PB1 - power stage + ISR liveness */
#define LED_STATUS_PIN GPIO_PIN_2    /* PB2 - drive state / fault code   */

/* BSRR is a single atomic store and needs no read-modify-write, so the ISR
 * side can set a pin without touching ODR and without a critical section -
 * which matters because the two lamps are driven from different contexts. */
static inline void led_write(uint16_t pin, uint32_t on)
{
#if !BOARD_HAS_LEDS
  /* No LEDs on this board - see BOARD_HAS_LEDS in board.h. Swallowed here
   * rather than at each call site, so the pattern logic below stays compiled
   * and reviewable instead of rotting behind an #if. */
  (void)pin; (void)on;
  return;
#else
  LED_PORT->BSRR = on ? (uint32_t)pin : ((uint32_t)pin << 16);
#endif
}

/* Derived, not written down: the pattern stays one second long whatever the
 * switching frequency is set to. */
#define STAGE_PERIOD_TICKS  ((PWM_FREQ_HZ * LED_STAGE_PERIOD_MS) / 1000U)
#define STAGE_PULSE_TICKS   ((PWM_FREQ_HZ * LED_STAGE_PULSE_MS)  / 1000U)

void Led_Init(void)
{
#if BOARD_HAS_LEDS
  /* gpio.c configures both as push-pull outputs; this only settles them into
   * a known state. Dark means "no firmware", so leaving them lit here would
   * make a hung boot look healthy. */
  led_write(LED_STAGE_PIN, 0U);
  led_write(LED_STATUS_PIN, 0U);
#endif
}

void Led_StepIsr(void)
{
  static uint32_t tick  = 0U;
  static uint32_t gates = 0U;

  if (++tick >= STAGE_PERIOD_TICKS) { tick = 0U; }

  /* Ask the HARDWARE, not the state machine. A bench tool that brings the
   * gates up through g_cmd without going through Drive_Arm would leave any
   * state-derived indicator lying about a live bridge, and this is the lamp
   * that must never lie.
   *
   * Cached and refreshed every 64 ticks rather than called every tick. That
   * call is across a translation unit so it does not inline, and measured at
   * every tick it cost 108 cycles of the ISR budget - disproportionate for a
   * lamp. 64 ticks is 2.1 ms of lag on the indicator, which no eye resolves
   * and which is three orders of magnitude faster than the 1 s blink it is
   * modulating. */
  if ((tick & 63U) == 0U) { gates = MotorPwm_GateIsEnabled(); }

  uint32_t on = (tick < STAGE_PULSE_TICKS) ? 1U : 0U;
  if (gates != 0U) { on = !on; }

  led_write(LED_STAGE_PIN, on);
}

void Led_StepMain(uint32_t now_ms)
{
  uint32_t st = g_drive.state;

  if (st == (uint32_t)DRIVE_INIT)
  {
    led_write(LED_STATUS_PIN, 1U);
    return;
  }

  if (st == (uint32_t)DRIVE_SELFTEST)
  {
    /* DARK, not a flicker.
     *
     * This was a 10 Hz flicker on the reasoning that a self-test lasts
     * milliseconds and the pattern would never really be seen. That was wrong
     * twice over: an undervoltage bus keeps SELFTEST retrying indefinitely, so
     * with the supply off this is the drive's RESTING state, and a permanent
     * 10 Hz strobe on a bench is intolerable to sit next to.
     *
     * Dark is the right answer anyway. Waiting for a bus is benign, PB1 is
     * still flashing once a second to say the firmware is alive, and anything
     * that actually failed latches to FAULT and blinks its cause. So dark
     * means "not ready yet", lit means something worth reading. */
    led_write(LED_STATUS_PIN, 0U);
    return;
  }

  /* Everything else is a blip count. FAULT reports its cause so the lamp is
   * a diagnosis rather than just an alarm; READY and RUN get 1 and 2, which
   * keeps "how many flashes" the only thing anyone has to remember. */
  uint32_t blips;
  if      (st == (uint32_t)DRIVE_READY) { blips = 1U; }
  else if (st == (uint32_t)DRIVE_RUN)   { blips = 2U; }
  else
  {
    blips = g_drive.fault;
    /* FAULT with no cause recorded should be impossible, but a lamp that goes
     * dark forever is the worst way to find out. */
    if (blips == 0U) { blips = 1U; }
  }

  uint32_t slot  = LED_BLIP_ON_MS + LED_BLIP_OFF_MS;
  uint32_t cycle = (blips * slot) + LED_BLIP_GAP_MS;
  uint32_t pos   = now_ms % cycle;

  uint32_t on = 0U;
  if (pos < (blips * slot))
  {
    on = ((pos % slot) < LED_BLIP_ON_MS) ? 1U : 0U;
  }

  led_write(LED_STATUS_PIN, on);
}
