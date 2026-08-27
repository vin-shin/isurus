# Isurus

Isurus is the motor control stack: the FOC current loop, the motion and haptic
loops, the CAN interface, the SWD tooling and the host harness. It is meant to
move between boards, so each hardware target gets its own branch named for the
board rather than its own repository.

The board on this branch is **Mako Desori** — an STM32G474RET6 driving an
**EMRAX 228 HV** axial-flux traction motor through six isolated SiC
half-bridge drivers, from a 140s2p pack. Its designers know it as the GR
MotherFOCer; the codename follows this repo's convention of naming targets
after mako sharks, after the genus the stack itself is named for.

> ### This firmware has never been run on hardware.
>
> The port is complete and the host tests pass, but every number in it came
> from a schematic and four datasheets. A dozen things are inferences that have
> never met a board.
>
> **Start with `tools/bringup_check.sh`.** Ten minutes, ST-Link only, nothing
> else connected — it validates or destroys those inferences one at a time and
> names the assumption behind each failure.
>
> **Do not arm anything above open-loop** until `FOC_LAMBDA_M_WB` has been
> measured. Three derivations of it span 35%, and the too-high direction puts
> the feedforward alone above the modulation ceiling. See
> [`docs/PORT-MAKO-DESORI.md`](docs/PORT-MAKO-DESORI.md) §5.

Torque, velocity, position and haptic control all run off one 20 kHz current
loop.

## Hardware

| | |
|---|---|
| MCU | STM32G474RET6, LQFP64, 160 MHz, 512 KB flash / 128 KB RAM |
| Bridge | 6x Infineon IMCQ120R004M2H, CoolSiC 1200 V / 3.7 mΩ |
| Gate drivers | 6x TI UCC21756-Q1, isolated, DESAT, per-switch fault and ready |
| Current sense | 2x Mornsun TL200-A2PV on U and W, ±500 A, plus DC link |
| Bus sense | 400:1 divider into an AMC0311 isolated amplifier |
| Temperature | KTY81 in the motor winding; three gate-driver APWM channels |
| Encoder | differential SSI-class, SPI3 through RS422 transceivers |
| Motor | EMRAX 228 HV, 10 pole pairs, 100 Arms continuous / 240 Arms peak |
| Pack | 140s2p — 350 V empty, 518 V nominal, 588 V full |

Three complementary HRTIM output pairs at 20 kHz, with hardware dead-time
insertion. **The gate enable on PC8 is active HIGH**, which is the opposite of
the previous board on this repo.

The full hardware map, with the provenance of every constant and what is still
unverified, is **[`Core/Inc/board.h`](Core/Inc/board.h)**.

> The schematic and datasheets are deliberately **not** in this repository —
> `.gitignore` excludes `docs/*.pdf`, because this remote is public and the
> schematic is Gaucho Racing's own design. Everything taken from them is
> written into the source with its provenance. Drop the files into `docs/`
> locally if you want to check the working.

## Control

The 20 kHz HRTIM ISR runs current sense, the gate driver status poll, a fast
encoder read, Clarke/Park, two PI current loops with back-calculation
anti-windup, and inverse Park into min/max (third-harmonic) injected duty. The
outer loops are decimated from it: position and velocity PID at 1 kHz, output
smoothing and haptic torque at the full rate.

| Mode | | |
|---|---|---|
| 0 | idle | no torque |
| 1 | torque | direct iq command |
| 2 | velocity | deg/s |
| 3 | position | multi-turn, trapezoid or S-curve profile |
| 4 | haptic | renders a force field — detents, endstops, springs, damping |

Every setpoint is saturated to the bounds in `Core/Inc/limits.h`, which are
traceable to the motor, the sensors or a measurement rather than to preference.
The clamping happens in the motion loop, so it applies equally to CAN, the SWD
tools, and anything written by hand with a debugger.

## Protection

What stops this drive hurting itself, fastest first:

| | responds in | status |
|---|---|---|
| Gate driver DESAT | 200 ns, in the driver | **active** — the driver acts alone; `gatedrv.c` reads `FLT` and latches `DRIVE_FAULT_GATEDRV` |
| Sensor OCD, ±400 A | 0.3 µs, in the sensor | **not connected** — see [`docs/HARDWARE-CHANGES.md`](docs/HARDWARE-CHANGES.md) §1a |
| COMP2 bus overvoltage | hardware comparator | **not armed** — needs a DAC threshold |
| Current limit | 50 µs | active, bounded by the motor at 339 A peak |
| Winding over-temperature | 200 ms | active — and load-bearing, because the current limit is deliberately above the machine's *continuous* rating |
| Power-stage temperature | — | **none** — the channels are APWM, see [`docs/LATER.md`](docs/LATER.md) §2 |

A lost temperature sensor faults exactly as hard as a hot one, and a gate
driver that is merely not-ready is treated as a precondition rather than a
failure. The reasoning for both is in `drive.c`.

## CAN interface

Classic CAN 2.0A, 11-bit IDs, 1 Mbit/s on FDCAN2 (PB6 TX / PB5 RX) through a
TCAN1044A. The identifier carries a 6-bit node address and a 5-bit command;
commands are `0x00`-`0x0F` and telemetry is `0x10`-`0x1F`, so a broadcast
e-stop is identifier `0x000` and wins arbitration against everything else.

**Commands and telemetry are in different units** — commands in mA and dps,
telemetry in deciamps, centivolts and RPM. That is not an oversight: four
telemetry fields overflowed on HV hardware and were rescaled, while the 32-bit
command fields never did.

Full wire format and framing: **[docs/CAN_PROTOCOL.md](docs/CAN_PROTOCOL.md)**.
The machine-readable definition is `Core/Inc/can_proto.h`, which has no HAL
dependency so a host can include or transcribe it directly.

## Building

Requires CMake >= 3.22, Ninja, and `arm-none-eabi-gcc`.

```bash
cmake --preset Debug
cmake --build --preset Debug
```

Output is `build/Debug/makolongfin2.elf`. The CMake target, the ELF and the
`.ioc` still carry a previous board's name on purpose: renaming them moves the
artifact every tool in `tools/` resolves symbols from, for no functional gain.

Files on the ISR path (`foc.c`, `drive.c`, `led.c`, `position.c`, `haptic.c`,
`csense.c`, `encoder.c`, `motor_pwm.c`, `gatedrv.c`, `main.c`) are built at
`-O2` while the rest stays at `-O0`. That is a real-time requirement, not a preference — see
the comment in `CMakeLists.txt` before changing it.

## Testing

```bash
test/host/run.sh              # 53 tests against a numerical PMSM model
test/host/run.sh --mutants    # verify the tests can actually fail
```

`foc.c`, `drive.c` and `ident.c` compile unmodified for the host; `shim/`
supplies the CORDIC and `fastmath.h`. The `--mutants` pass plants twenty known
bugs and checks the suite goes red for each — a green suite that has never been
seen to fail proves nothing, and this one has caught a real hole twice.

## Flashing

```bash
openocd -f openocd.cfg -c "init" -c "reset halt" \
  -c "program build/Debug/makolongfin2.elf verify" \
  -c "reset run" -c "exit"
```

NRST is not wired to the probe, so all resets are software resets. OpenOCD
leaves the core halted when it exits; end with `reset run` if you want the
board to keep running standalone.

## Tools

Everything in `tools/` talks to the running target over SWD and resolves symbol
addresses from the ELF at run time, so a rebuild never leaves a script pointing
at a stale address. Only one process can hold the ST-Link at a time.

| | |
|---|---|
| `bringup_check.sh` | **start here** — first-power-on check, logic power only |
| `motor_ctl.py` | interactive console — type control requests at the board |
| `dial.py` | haptic force-feedback dial, with presets and a demo mode |
| `foc_dash.sh` | live FOC dashboard: electrical angle, dq vector, id/iq traces |
| `pos_dash.sh` | position loop: dial, multi-turn ruler, error trace, current |
| `pos_listen.sh` | passive position telemetry |
| `encoder_dial.sh` | encoder angle as an ASCII dial |
| `watch_encoder.sh` | poll encoder telemetry while the target runs free |
| `isr_budget.sh` | per-stage ISR timing — **stale, still names a 33.3 µs deadline** |

The shell scripts render inside OpenOCD's own Tcl interpreter, so do not pipe
them through anything — it block-buffers and destroys the redraw.

## Layout

```
Core/Inc, Core/Src   application and CubeMX-generated peripheral setup
  board.h            the hardware map, and what is not known about it
  can.c/.h           FDCAN2 transport, polled from the main loop
  can_proto.h        CAN wire protocol - the specification, HAL-free
  csense.c           2 phase currents + DC link + bus, HRTIM-triggered ADC1
  drive.c            state machine, self-test, fault handling
  encoder.c          the position encoder over SPI3
  foc.c              Clarke/Park, PI current loops, SVPWM
  gatedrv.c          the twelve gate-driver fault and ready lines
  haptic.c           force-field rendering
  ident.c            phase R and L identification, on the machine itself
  limits.h           what the machine can physically do, in one place
  motor_pwm.c        HRTIM1 setup, duty, gate-driver enable
  openloop.c         open-loop vector drive, for bring-up
  position.c         position and velocity loops, motion profiling
  thermal.c          motor winding temperature, ADC2
Drivers              STM32G4 HAL
tools                SWD dashboards and host-side control
test/host            the simulator harness and its mutants
```

## Documentation

Read in this order if the board is new to you:

- [docs/PORT-MAKO-DESORI.md](docs/PORT-MAKO-DESORI.md) — **what this branch did
  and what is still unverified.** Includes §4, where this port was wrong and
  why, and §6, the bring-up order
- [docs/HARDWARE-CHANGES.md](docs/HARDWARE-CHANGES.md) — hardware findings: the
  unconnected fast protection, the dead time, and why CAN FD needs no new part
- [docs/LATER.md](docs/LATER.md) — deferred firmware, and why each is safe to
  defer
- [docs/CAN_PROTOCOL.md](docs/CAN_PROTOCOL.md) — CAN wire format and framing
- [CLAUDE.md](CLAUDE.md) — working conventions for this repo

Carried over from the previous board, and still worth reading for the
reasoning rather than the pin numbers:

- [HARDWARE_NOTES.md](HARDWARE_NOTES.md) — board quirks not derivable from the
  source tree. **Its BOOT0 / FDCAN1_RX section does not apply here** — this
  board does not use PB8 — but the failure it describes was indistinguishable
  from broken firmware and is worth recognising
- [docs/BENCH-2026-08-25.md](docs/BENCH-2026-08-25.md) — simulator vs bench
  validation of the current loop, and an inductance hypothesis the measurement
  refuted
- [docs/LED_CODES.md](docs/LED_CODES.md) — the two-LED front panel. This board
  has no LEDs; kept because the patterns are the design, not the wiring
- [docs/OPEN-ITEMS.md](docs/OPEN-ITEMS.md) — open items from the bench board
