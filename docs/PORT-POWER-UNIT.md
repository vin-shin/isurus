# Porting Isurus to the GR MotherFOCer

This branch (`power-unit`) retargets the Isurus stack from **Mako Longfin** to
the **GR MotherFOCer** inverter board. It follows the repo's convention that a
hardware target is a branch, not a repository.

The source of truth for the new hardware is the CubeMX project that shipped
with the board — `Inverter.ioc` and the LL code generated from it, in the
`gr_motherfocer` tree. `Core/Inc/board.h` is the transcription of that into
this project, and is the only file a later hardware revision should need to
touch.

**There is no schematic in the source material.** Everything below is derived
from a pin-assignment file and a bring-up program. That is enough to say which
peripheral sits on which pin; it is not enough to say what the pin is wired to.
Section 4 lists what that leaves open, and none of it should be closed by
guessing.

## 1. What the two boards do not share

| | Mako Longfin | GR MotherFOCer |
|---|---|---|
| Core clock | 128 MHz | 160 MHz |
| Bridge drive | 1 pin/phase, external inverter | 3 complementary pairs |
| Dead time | gate driver RDT, ~185 ns | HRTIM, 160 counts |
| Switching | 30 kHz | 20 kHz |
| HRTIM timers | A, B | B, C, F |
| Gate enable | PC5, **active low** | PC8, **active high** |
| Phase current | 2 phases, via OPAMP followers | 3 phases, direct to ADC1 |
| Extra analogue | — | DC link current, 4x temperature, "audio" |
| Bus sense | PF0, 190k/10k divider | PA3, shared with COMP2 |
| Overcurrent | firmware threshold | COMP2 + DAC1_CH2 hardware trip |
| Encoder | A1333, 15-bit, SPI1 | RM44SI, 13-bit, SPI3 |
| CAN | FDCAN1 (PA12/PB8) | FDCAN2 (PB6/PB5) |
| Debug output | SWD only | SWD + LPUART1 |
| Motor | 20 pole pairs, 12S | 10 poles(?), 5600 rpm |

Two of those rows are the ones that bite.

**The gate enable polarity is inverted between the boards.** Mako Longfin's
PC5 drove a UCC21330 `DIS` pin directly, so low meant enabled and the safe
state was high. This board's PC8 is the other way round in all three places
its own code touches it: reset state low, `disableGateDriver()` drives low,
`resetGateDriver()` ends by driving high. Carrying the old polarity across
energises the bridge at reset.

**The complementary bridge changes what "off" means.** `motor_pwm.h` on Mako
Longfin carries a prominent warning that no MCU state turns every FET off,
because a low pin there turned the *low-side* device on through the external
inverter — the gate drivers' `DIS` line was the only true all-off. On this
board the HRTIM drives both devices of each leg directly with hardware dead
time, so disabling the HRTIM outputs genuinely does open the whole bridge.
That warning must be rewritten rather than copied; leaving it in place would
be describing a hazard that no longer exists, which is its own kind of wrong.

## 2. What carries over untouched

The point of the branch-per-board layout is that most of the stack is not
hardware. These need no change beyond constants:

- `foc.c` — Clarke/Park, the PI current loops, third-harmonic injection
- `position.c` — position and velocity loops, motion profiling
- `haptic.c` — force-field rendering
- `drive.c` — the state machine and fault handling
- `can_proto.h` / `can.c` — the wire protocol (the transport binds to a
  different FDCAN instance, the protocol does not change)
- `ident.c` — phase R and L identification, which is if anything *more*
  useful here, because the new motor's parameters are inherited from someone
  else's initialiser and have never been measured on this machine
- `test/host/` — the whole simulator harness

## 3. What has to be rewritten, in dependency order

Ordered so that each step can be checked before the next one can hurt
anything. Nothing below step 3 should run with the bus energised.

1. **`board.h`** — done. The hardware map, and the record of what is not
   known about it.

2. **Clock and peripheral init** — `main.c`'s `SystemClock_Config` to 160 MHz
   (PLLN 20), and the CubeMX layer (`gpio.c`, `adc.c`, `spi.c`, `fdcan.c`,
   `hrtim.c`, plus a new `usart.c` target) to the new pinout. The board's
   generated code is LL; Isurus is HAL throughout and stays HAL, because the
   entire stack above is written against HAL handles and converting it would
   be rewriting the parts of the project that are *not* supposed to change
   between boards.

3. **`motor_pwm.c` / `.h`** — timers B/F/C instead of A/B, complementary
   outputs with dead-time insertion instead of single-ended, active-high gate
   enable, 20 kHz, and the ADC trigger lead recomputed for a 1.28 GHz counter
   (`PWM_ADC_LEAD_COUNTS` currently hardcodes 1024 counts per microsecond,
   which is the 128 MHz figure). The safety comment gets rewritten per §1.

4. **`csense.c` / `.h`** — the largest single rewrite. Mako Longfin reads two
   phases through internal OPAMP followers on ADC2 and ADC5, polled on an
   HRTIM trigger. This board has three phases plus DC link current and bus
   voltage in one free-running ADC1 sequence with DMA. Two things change at
   once: the plumbing, and the fact that a third phase measurement makes the
   Clarke transform overdetermined, which is a genuine improvement worth
   using rather than dropping.

   **ADC1 must be moved onto the HRTIM trigger as part of this step.** As the
   board ships it free-runs, and the main loop reads whatever DMA last wrote.
   For a print loop that is fine. For a current loop it is not: the samples
   are not aligned to a known point in the switching period, so what the loop
   sees is a phase current sampled at an arbitrary place in the ripple. Mako
   Longfin's `PWM_ADC_LEAD_NS` comment describes at length how a mistake in
   exactly this area reads perfectly in a static test and destabilises the
   loop the moment anything moves — that lesson transfers directly.

5. **`encoder.c` / `.h`** — RM44SI on SPI3, 13 bits, manual CS on PA15. The
   A1333's two-frame command/NOP transaction does not apply. Blocked on the
   frame format (§4).

6. **`can.c`** — bind to FDCAN2 and recompute the bit timing for 160 MHz. The
   existing prescaler 8 / seg1 12 / seg2 3 gives 1 Mbit at 128 MHz; at 160 MHz
   the same divisors give 1.25 Mbit, which will not communicate.

7. **`limits.h`** — every bound is traceable to Mako Longfin's motor, sensors
   or bench, and none of those are the hardware here. This file should be
   emptied back to first principles against the new motor rather than scaled.

8. **`led.c`, `docs/LED_CODES.md`** — no LED appears in the new pinout, and
   `led.c` drives GPIOB pins that this board configures as inputs. Either the
   LEDs move to whichever of the unexplained pins turn out to be LEDs, or the
   two-LED front panel and its documentation go away on this branch.

9. **`tools/`** — the SWD dashboards resolve symbols from the ELF at run time,
   so they survive the port. They do assume `PWM_FREQ_HZ` and encoder counts;
   both come from headers, so they should follow automatically. `isr_budget.sh`
   needs re-running from scratch: the deadline moves from 33.3 us to 50 us at
   20 kHz, and the core is 25% faster, so the existing 21.1 us figure means
   nothing here.

## 4. What is not known, and must not be guessed

These are blocking questions for anything that energises the bridge. Each one
is unanswerable from the material available.

1. **Twelve configured, unlabelled input pins**: PA8, PA9, PA12, PB0, PB2,
   PB4, PB10, PB11, PB14, PB15, PC9, PD2. Some of these are near-certainly
   gate-driver fault outputs. A fault line left as a floating input is a fault
   nobody ever hears about, which is the failure mode that fault lines exist
   to prevent.

2. **What COMP2 protects against.** Its input is PA3, which the .ioc also
   assigns to ADC1_IN4 — the DC bus voltage. So the hardware comparator
   appears to watch bus voltage, while the board's error masks
   (`ERR_OCP`/`ERR_OVP`/`ERR_UVP`/`ERR_OTP`) suggest an overcurrent trip was
   intended. One of those readings is wrong. Arming the comparator against
   the wrong signal gives a trip that either never fires or fires constantly.

3. **`N_POLES = 10` — poles or pole pairs?** The board's code says "poles".
   Mako Longfin's equivalent number is pole pairs. Read the wrong way this is
   a factor of two on the electrical angle, which does not fail loudly; it
   produces a motor that turns weakly and heats up.

4. **The current sense scale.** `0.04 A/LSB` and `0.05 V/LSB` are inherited
   from the board's bring-up loop with no sensor part number and no divider
   ratio behind them. The bus scale feeds the undervoltage trip, so it needs a
   meter against it before anything depends on it.

5. **The RM44SI frame format** — where the angle sits in the 16-bit response,
   and whether there are status bits worth checking.

6. **The dead-time value in real units.** 160 counts at DT prescaler DIV1 is
   what is configured; the nanoseconds depend on the divider chain in RM0440
   and nobody here has confirmed it or seen it on a scope.

7. **Which motor is actually attached**, and whether the inherited kv, R and
   L describe it. `ident.c` answers R and L directly and is the cheapest thing
   to run once the board powers up at all.

## 5. Suggested bring-up order

Roughly the order Mako Longfin was brought up in, which worked:

1. Build and flash. Confirm the core runs at 160 MHz and LPUART1 prints.
2. Encoder only — no bus voltage. Turn the shaft by hand and watch the angle
   wrap cleanly across 8192 counts.
3. CAN loopback, then CAN against a host at 1 Mbit.
4. ADC with the bridge disabled: confirm the zero-current offsets settle and
   that bus voltage reads something plausible against a meter. This is where
   question 4 gets answered.
5. HRTIM outputs on a scope, gate drivers still disabled. Confirm the
   complementary pairs and measure the actual dead time. Question 6.
6. Open-loop vector drive at low bus voltage (`openloop.c`) — the first step
   with current in the motor, and the one that answers question 3, since a
   pole-count error shows up immediately as the wrong number of electrical
   revolutions per mechanical one.
7. `ident.c` for R and L. Question 7.
8. Closed-loop current, then the outer loops.

Steps 1-5 need no bus voltage beyond logic power and are worth completing
before the DC link is ever connected.
