# Isurus

Isurus is the motor control stack: the FOC current loop, the motion and haptic
loops, the CAN interface, the SWD tooling and the host harness. It is meant to
move between boards, so each hardware target gets its own branch named for the
board rather than its own repository.

> **This branch is a port in progress.** `power-unit` retargets the stack from
> Mako Longfin to the **GR MotherFOCer** inverter board. The hardware map for
> the new board is [`Core/Inc/board.h`](Core/Inc/board.h) and the plan, the
> board-to-board deltas and the open questions are in
> [`docs/PORT-POWER-UNIT.md`](docs/PORT-POWER-UNIT.md). **Everything below this
> note still describes Mako Longfin** and will be rewritten as each module is
> ported - read it as the source's current state, not as this board.
>
> Two differences are dangerous enough to repeat here: the gate enable is
> active *high* on the new board and active *low* on the old one, and the new
> bridge is complementary, so unlike Mako Longfin, disabling the HRTIM outputs
> really does turn every FET off.

The board here is **Mako Longfin** - an STM32G474RET6 driving a three-phase
BLDC through a UCC21330 gate-driver stage, with an Allegro A1333 magnetic
encoder for rotor feedback and a CAN control interface.

Torque, velocity, position and haptic control all run off one 30 kHz current
loop.

## Hardware

| | |
|---|---|
| MCU | STM32G474RET6, LQFP64, 128 MHz, 512 KB flash / 128 KB RAM |
| Gate drivers | 3x UCC21330BQDRQ1, isolated, 8 V UVLO, 185 ns dead time set by RDT |
| Current sense | 2x CT4022-A40BSN8 TMR, +/-40 A, on phases U and W |
| Encoder | Allegro A1333, 15-bit absolute, SPI1 |
| Motor | 20 pole pairs, 22 A continuous, 12S (44.4 V nominal / 50.4 V full) |
| Bench supply | ~15.5 V, about a third of the motor's design voltage |

Motor phases are driven by HRTIM1 at 30 kHz. One MCU pin per phase; an external
inverter on the board derives each driver's complementary input.

**Before assuming a board that will not run is a firmware problem, read
[HARDWARE_NOTES.md](HARDWARE_NOTES.md).** PB8 is both BOOT0 and FDCAN1_RX on
this package, so a factory-fresh chip boots the ST bootloader instead of flash
and every symptom points at the application.

## Control

The 30 kHz HRTIM ISR runs current sense, the encoder read, Clarke/Park, two PI
current loops with back-calculation anti-windup, and inverse Park into min/max
(third-harmonic) injected duty. The outer loops are decimated from it: the
position and velocity PID at 1 kHz, output smoothing and haptic torque at the
full 30 kHz.

| Mode | | |
|---|---|---|
| 0 | idle | no torque |
| 1 | torque | direct iq command |
| 2 | velocity | deg/s |
| 3 | position | multi-turn, trapezoid or S-curve profile |
| 4 | haptic | renders a force field - detents, endstops, springs, damping |

Every setpoint is saturated to the bounds in `Core/Inc/limits.h`, which are
traceable to the motor, the sensors or a measurement rather than to preference.
The clamping happens in the motion loop, so it applies equally to CAN, the SWD
tools, and anything written by hand with a debugger.

## CAN interface

Classic CAN 2.0A, 11-bit IDs, 1 Mbit/s on FDCAN1 (PA12 TX / PB8 RX). The
identifier carries a 6-bit node address and a 5-bit command; commands are
`0x00`-`0x0F` and telemetry is `0x10`-`0x1F`, so a broadcast e-stop is
identifier `0x000` and wins arbitration against everything else on the bus.

Full wire format, framing and host examples: **[docs/CAN_PROTOCOL.md](docs/CAN_PROTOCOL.md)**.
The machine-readable definition is `Core/Inc/can_proto.h`, which has no HAL
dependency so a host can include or transcribe it directly.

## Building

Requires CMake >= 3.22, Ninja, and `arm-none-eabi-gcc`.

```bash
cmake --preset Debug
cmake --build --preset Debug
```

Output is `build/Debug/makolongfin2.elf`. The CMake target, the ELF and
`makolongfin2.ioc` still carry the old name on purpose: renaming them moves the
artifact every tool in `tools/` resolves symbols from, for no functional gain.

Files on the ISR path (`foc.c`, `position.c`, `haptic.c`, `csense.c`,
`encoder.c`, `motor_pwm.c`, `main.c`) are built at `-O2` while the rest of the
project stays at `-O0`. That is a real-time requirement, not a preference - see
the comment in `CMakeLists.txt` before changing it.

## Flashing

```bash
openocd -f openocd.cfg -c "init" -c "reset halt" \
  -c "program build/Debug/makolongfin2.elf verify" \
  -c "reset run" -c "exit"
```

NRST is not wired to the probe, so all resets are software resets. OpenOCD
leaves the core halted when it exits; end with `reset run` if you want the board
to keep running standalone.

## Tools

Everything in `tools/` talks to the running target over SWD and resolves symbol
addresses from the ELF at run time, so a rebuild never leaves a script pointing
at a stale address. Only one process can hold the ST-Link at a time.

| | |
|---|---|
| `motor_ctl.py` | interactive console - type control requests at the board |
| `dial.py` | haptic force-feedback dial, with presets and a demo mode |
| `foc_dash.sh` | live FOC dashboard: electrical angle, dq vector, id/iq traces |
| `pos_dash.sh` | position loop: dial, multi-turn ruler, error trace, current |
| `pos_listen.sh` | passive position telemetry |
| `encoder_dial.sh` | encoder angle as an ASCII dial |
| `watch_encoder.sh` | poll encoder telemetry while the target runs free |

The shell scripts render inside OpenOCD's own Tcl interpreter, so do not pipe
them through anything - it block-buffers and destroys the redraw.

## Layout

```
Core/Inc, Core/Src   application and CubeMX-generated peripheral setup
  can.c/.h           FDCAN transport, polled from the main loop
  can_proto.h        CAN wire protocol - the specification, HAL-free
  csense.c           phase current sense, HRTIM-triggered ADC
  encoder.c          A1333 over SPI1
  foc.c              Clarke/Park, PI current loops, SVPWM
  haptic.c           force-field rendering
  limits.h           what the machine can physically do, in one place
  motor_pwm.c        HRTIM1 setup, duty, gate-driver enable
  openloop.c         open-loop vector drive, for bring-up
  position.c         position and velocity loops, motion profiling
Drivers              STM32G4 HAL
tools                SWD dashboards and host-side control
cmake                toolchain files
```

## Documentation

- [HARDWARE_NOTES.md](HARDWARE_NOTES.md) - board-level quirks that are not
  derivable from the source tree, and the measurements behind the constants
- [docs/CAN_PROTOCOL.md](docs/CAN_PROTOCOL.md) - CAN wire format and framing
- [docs/LED_CODES.md](docs/LED_CODES.md) - the two-LED front panel: what each
  pattern means and how to tell a hung ISR from a hung main loop
- [docs/BENCH-2026-08-25.md](docs/BENCH-2026-08-25.md) - simulator vs bench
  validation of the current loop, and an inductance hypothesis that the
  measurement refuted
- [docs/OPEN-ITEMS.md](docs/OPEN-ITEMS.md) - what is still open, and the
  limits of the bench and the simulator that shape it
- [CLAUDE.md](CLAUDE.md) - working conventions for this repo
