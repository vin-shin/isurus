# Debug LEDs — Isurus

> **This board has no LEDs.** The GR MotherFOCer pinout has nothing spare for
> them - PB1 is an ADC3 input and PB2 is one of the unexplained inputs in
> `board.h` section 9 - so `BOARD_HAS_LEDS` is 0 and `led.c` compiles to
> nothing. Everything below describes Mako Longfin and is kept because the
> patterns are the design, not the wiring: if a pin is ever found, this is
> what should go on it. See `docs/PORT-POWER-UNIT.md`.

Two LEDs on PB1 and PB2 are the whole front panel. Once the board leaves the
bench there is no debugger, no UART and no CAN host, so the pair has to answer
the only three questions anyone actually asks of a drive:

- is it alive?
- is the power stage live — the one that can hurt you?
- if it is not working, why?

Both are plain push-pull outputs, active high: lit means the pin is driven
high. Neither uses brightness. `led_pwm.c`, which drove PB1's brightness from
the encoder angle via TIM3_CH4, was bring-up instrumentation and is not on the
trunk — see HARDWARE_NOTES section 5.

The source of truth is `Core/Inc/led.h` and `Core/Src/led.c`. This file is the
bench-side reading of it.

## The two lamps are stepped from different contexts

That split is the point of the scheme, not an implementation detail.

| Lamp | Pin | Stepped by | Called from |
|---|---|---|---|
| Power stage | PB1 | `Led_StepIsr()` | the 30 kHz control ISR, `main.c` |
| Status | PB2 | `Led_StepMain(HAL_GetTick())` | the main loop, `main.c` |

A lamp driven from the main loop keeps winking cheerfully while the control
loop is dead and the bridge sits on stale duties. Driving PB1 from the ISR
means its blink is itself the evidence that the ISR is still running.

## PB1 — power stage, and ISR liveness

| Bridge | Pattern | Reads as |
|---|---|---|
| Gates **off** | 40 ms flash once a second | mostly dark |
| Gates **on** | inverted — lit continuously with a 40 ms notch once a second | mostly lit |

Two facts on one lamp. The **duty** says whether the bridge is energised, and
mostly-lit for "live" is deliberate: it is the reading you get from the corner
of your eye, and the dangerous state should be the bright one. The **blink**
says the control ISR is still running.

The period is derived from `PWM_FREQ_HZ`, not written down, so the pattern
stays one second long whatever the switching frequency becomes:

```c
#define STAGE_PERIOD_TICKS  ((PWM_FREQ_HZ * LED_STAGE_PERIOD_MS) / 1000U)
#define STAGE_PULSE_TICKS   ((PWM_FREQ_HZ * LED_STAGE_PULSE_MS)  / 1000U)
```

**PB1 cannot lie about the bridge.** It calls `MotorPwm_GateIsEnabled()` and
reads the hardware, not `g_drive.state` or any flag. A bench tool that brings
the gates up through `g_cmd` without going through `Drive_Arm` would leave a
state-derived indicator claiming the bridge was down while it was live. This
is the lamp that must never do that.

The hardware read is cached and refreshed every 64 ticks rather than every
tick — the cross-translation-unit call does not inline and measured at 108
cycles of ISR budget, which is disproportionate for a lamp. That is 2.1 ms of
lag on an indicator modulating a 1 s blink, which no eye resolves.

## PB2 — drive state, and the fault cause

| State | Pattern |
|---|---|
| `INIT` | solid |
| `SELFTEST` | **dark** |
| `READY` | 1 blip, then a pause |
| `RUN` | 2 blips, then a pause |
| `FAULT` | N blips, then a pause, where N is the fault code below |

Counting flashes is a poor interface and the right one here: it needs no
second device, it survives being read across a workshop, and the cause is the
thing a bench note can be written against.

### Fault codes

N is `DriveFault_t` from `Core/Inc/drive.h`, latched at the **first** trip —
it is one value, not a bitmask, because what matters after the fact is what
tripped first. Later faults are counted in `g_drive.fault_count`, not merged.

| N | Cause | Means |
|---|---|---|
| 1 | `DRIVE_FAULT_OVERCURRENT` | phase current past the trip |
| 2 | `DRIVE_FAULT_OVERVOLTAGE` | bus above limit |
| 3 | `DRIVE_FAULT_UNDERVOLT` | bus below limit |
| 4 | `DRIVE_FAULT_ENCODER` | no response, or an implausible angle |
| 5 | `DRIVE_FAULT_CSENSE` | current-sense zero out of tolerance |
| 6 | `DRIVE_FAULT_SELFTEST` | a check failed that has no finer cause |
| 7 | `DRIVE_FAULT_WATCHDOG` | the control ISR stopped feeding the WWDG |
| 8 | `DRIVE_FAULT_COMMAND` | a command was refused as implausible |

**FAULT latches.** It is left only by an explicit `Drive_ClearFault()`, and
that returns to `SELFTEST`, not `READY` — nothing gets back to `RUN` without
re-passing the checks.

### Blip timing

```
blip on   140 ms
blip off  220 ms      -> one slot = 360 ms
gap      1100 ms
cycle    = N * 360 + 1100 ms
```

| N | Cycle |
|---|---|
| 1 | 1.46 s |
| 2 | 1.82 s |
| 4 | 2.54 s |
| 8 | 3.98 s |

Worst case is under four seconds, which is a cycle someone will actually wait
through. Count from the long gap — it is the only unambiguous landmark, and at
1100 ms against a 220 ms inter-blip gap it is a 5:1 ratio you can read without
a stopwatch.

### Why SELFTEST is dark

An undervoltage bus keeps `SELFTEST` retrying indefinitely, so **with the
motor supply off this is the drive's resting state**, not a passing phase. It
was briefly a 10 Hz flicker on the reasoning that a self-test lasts
milliseconds; that was wrong twice over, and a permanent strobe on a bench is
intolerable to sit next to.

Nothing is lost by going dark. PB1 still flashes once a second to say the
firmware is alive, and anything that genuinely failed latches to `FAULT` and
blinks its cause. So on PB2, **dark means "not ready yet" and lit means
something worth reading.**

`SELFTEST` gives up after `DRIVE_SELFTEST_TIMEOUT_MS` (2000 ms), which is
generous on purpose — the checks are fast, and the only reason to wait is a
Vbus reading that has not arrived because the supply is still ramping.

## Reading the pair together

This is what the split across two contexts buys, and it is diagnostic in a way
neither lamp is alone:

| PB1 | PB2 | Diagnosis |
|---|---|---|
| blinking | blinking | both loops running |
| blinking | frozen | main loop hung, ISR fine |
| frozen | blinking | ISR hung — though the watchdog should have reset the board first, in which case PB2 would be showing code 7 |
| dark | dark | no firmware at all: bad boot, held in reset, or no 3V3 |

**Both dark is the one to read carefully.** On this package PB8 is both BOOT0
and FDCAN1_RX, so a factory-fresh chip boots the ST bootloader instead of
flash and every symptom points at the application. Read `HARDWARE_NOTES.md`
before treating a dark board as a firmware bug.

### A halted core looks like a dead board

OpenOCD leaves the core **stopped** when it exits, and cortex-debug halts at
`main` on launch. Ending a session mid-halt freezes both lamps, which reads as
a regression and is not one. End with `reset run`:

```bash
openocd -f openocd.cfg -c "init" -c "reset halt" \
  -c "program build/Debug/makolongfin2.elf verify" \
  -c "reset run" -c "exit"
```

## Known ambiguity: 1 blip is both READY and overcurrent

`Led_StepMain` assigns 1 blip to `DRIVE_READY` and N blips to a fault, and
`DRIVE_FAULT_OVERCURRENT` is 1. The two therefore produce an **identical**
pattern on PB2 — and because the bridge is off in both, PB1 is identical too.
A drive sitting in `READY` and a drive latched on an overcurrent cannot be
told apart from the front panel.

The `RUN` / code-2 collision does **not** have this problem: `RUN` has the
bridge live, so PB1 is mostly-lit there and flashing in `FAULT`.

Until the scheme changes, resolve it over SWD — `g_drive.state` is 2 for
`READY` and 4 for `FAULT`:

```bash
openocd -f openocd.cfg -c "init" \
  -c "mdw [expr {[dict get $syms g_drive]}]" -c "exit"
```

Resolve `g_drive`'s address from the ELF at run time with `arm-none-eabi-nm`
the way every script in `tools/` does. Never bake it into a script: any
code-size change moves the BSS layout, and a stale address does not error, it
reads a different variable.

## Where this is implemented

| | |
|---|---|
| `Core/Inc/led.h` | the scheme, and the reasoning behind each choice |
| `Core/Src/led.c` | `Led_Init`, `Led_StepIsr`, `Led_StepMain` |
| `Core/Inc/drive.h` | `DriveState_t`, `DriveFault_t`, the latching rule |
| `Core/Src/gpio.c` | PB1/PB2 as push-pull outputs, via CubeMX |
| `Core/Src/main.c` | the two call sites |
