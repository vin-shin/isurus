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

1. **`board.h`** — **done.** The hardware map, and the record of what is not
   known about it.

2. **Clock and peripheral init** — **mostly done.** `SystemClock_Config` runs
   at 160 MHz from `BOARD_PLLN`. `hrtim.c`, `fdcan.c`, `spi.c` and `usart.c`
   are on the new pinout; `gpio.c` and `adc.c` are not yet.

   The board's generated code is LL; Isurus is HAL throughout and stays HAL,
   because the entire stack above is written against HAL handles and
   converting it would be rewriting the parts of the project that are *not*
   supposed to change between boards.

   The debug console moved from USART1 on PB6/PB7 to **LPUART1 on PC0/PC1**,
   which is what the board assigns. That was not optional: PB6 is FDCAN2_TX
   here, so the two MspInits were fighting over the same pin, and the loser
   was whichever ran first. Mako Longfin had no serial output at all — every
   byte went over SWD — so this is new capability rather than a port.

   **`gpio.c` and `adc.c` are still Mako Longfin's** and are the next thing to
   do. `gpio.c` configures PC13/14/15, PC0, PA2/PA4, PB1/PB2/PB9 and PD2 with
   no basis on this board, and `adc.c` sets up PF0 and PA6 for the old bus
   sense. Neither currently collides with a bridge output, a CAN line or the
   encoder, which is the only reason they are not in the paragraph above.

3. **`motor_pwm.c` / `.h`** — **done.** Timers B/F/C instead of A/B,
   complementary outputs with dead-time insertion instead of single-ended,
   active-high gate enable, 20 kHz, and the ADC trigger lead derived from
   `BOARD_HRTIM_TICK_HZ` instead of a hardcoded 1024 counts per microsecond.
   `hrtim.c`'s MSP no longer claims PC8, which is the gate enable. The
   control ISR moved from the Timer A repetition event to Timer B's.

   Two things fell out of this that were not on the list. `EmergencyStop`
   wrote a raw BSRR bit sized for the old board's active-low enable, so from
   a fault handler it would have *enabled* this board's gate drivers; it goes
   through a compile-time `GATE_EN_BSRR_DISABLE` now. And the host simulator
   hardcoded 30 kHz, so moving the firmware to 20 made both inductance tests
   read +45% - an apparent identification error that was really a harness
   that had not been told the rate changed. `SIM_TS` derives from
   `PWM_FREQ_HZ` now.

4. **`csense.c` / `.h`** — **refused at run time, not yet rewritten.** This
   is the largest single rewrite.

   `CSense_Init` now returns an error immediately and the Mako Longfin
   implementation is fenced behind `BOARD_HAS_OPAMP_CSENSE`, which is 0.
   That is deliberate on two counts. The OPAMP MspInits it used to call claim
   PA1, PA3, PB0, PB11, PB12 and PC3 — and on this board PA1 and PC3 are
   phase current inputs, PA3 is the bus voltage shared with the overcurrent
   comparator, and **PB12 is the W high-side gate**. And the refusal doubles
   as the interlock: leaving the zero-current offsets at 0 puts them 2048
   codes from mid-scale, far outside `DRIVE_CS_ZERO_TOL_CODES`, so
   `Drive_SelfTest` raises `DRIVE_FAULT_CSENSE` and the bridge cannot be
   armed. A board whose current sense has never been read should not arm.

   The code is fenced rather than deleted because it is the reference for
   what replaces it, and fenced rather than left after an early `return`
   because unreachable code is a cppcheck `style` finding and that job fails
   the build.

   What the rewrite involves: Mako Longfin reads two
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

5. **`encoder.c` / `.h`** — **done.** RM44SI on SPI3, manual CS on PA15, one
   14-bit frame per read instead of the A1333's command/NOP pipeline. The
   frame format turned out to be answered after all, by the board's own SPI3
   interrupt handler: `angle = received & 0x1fff`, so the angle is the low 13
   bits of the 14 clocked back and the 14th bit is masked off unexamined.

   The reported count is widened to the project's 15-bit convention rather
   than the angle path being rescaled to 8192. `foc.c`'s `counts << 17`
   CORDIC conversion is exact only because a half turn lands on the Q31 sign
   bit, and the comment there describes how the off-by-pi version of the same
   expression stays self-consistent and silently inverts torque. That is not
   code to renegotiate to save a shift. The real cost is stated in
   `encoder.h`: the bottom two bits of every count are always zero, and
   mechanical resolution is genuinely 4x coarser than on Mako Longfin.

   The A1333's whole register surface - unlock, direct and extended register
   access, `SetZeroOffset`, `ZeroHere` - is gone, along with the SWD commands
   in `main.c` that drove it. Electrical zero moves into `g_foc.elec_offset`,
   which `foc.c` already applies and the SWD tools already reach. **The
   consequence is that the offset no longer survives a power cycle**, where
   the A1333 held it in EEPROM. Re-applying it at boot is an open item.

   Also worth recording: the dead-link detector tested frames against
   `0xFFFF`. On a 14-bit transfer the top two bits never arrive, so that
   constant could never have matched - the detector would have compiled,
   existed, and been permanently blind. It tests `0x3FFF` now.

6. **`can.c`** — **done.** Bound to FDCAN2 on PB6/PB5, prescaler 8 -> 10 so
   1 Mbit survives the move from a 128 MHz kernel clock to 160. The segment
   split is untouched, so the 81.25% sample point is bit-for-bit the one
   validated on the old board. `can_proto.h` did not change at all, which is
   what it was written for.

7. **The motor constants, and `limits.h`** - **done, with one number still
   unmeasured.** The machine is an EMRAX 228 HV on a 140s2p pack: 350 V empty,
   518 V nominal, 588 V full.

   `foc.h` now describes it - 10 pole pairs, 23.22 mOhm, 255 uH, Ld = Lq
   (surface-PM axial flux). The current-loop gains fell about 100x, almost all
   of it the bus: 518 V against 15.55 V is a factor of 33 on its own. Keeping
   1 kHz of bandwidth costs phase margin at the lower switching frequency -
   61 degrees here against 70 on the bench - which is stated in the file
   rather than inherited silently.

   **`FOC_LAMBDA_M_WB` is provisional and must be measured before the
   feedforward or the decoupling is enabled.** Three derivations exist and
   they span 35%: 60.1 mWb from `foc.h`'s own HV analysis, 54.4 mWb from
   kv = 10.14 read as rpm per DC volt, 44.4 mWb from kv read the way EMRAX
   publish it. The disagreement is entirely about which quantity "10.14
   rpm/V" is per, and no algebra settles that. The lowest is chosen because
   the error directions are not symmetric - too low and the integrator walks
   out the remainder, too high and the feedforward alone exceeds the
   modulation ceiling, which is the failure this file already recorded once at
   14% on a machine where back-EMF was a much smaller share of the bus.

   `limits.h` is rebuilt: 588 V / 350 V pack bounds, 33600 deg/s (5600 rpm,
   where 3600 deg/s would have capped this drive at 600 rpm), and
   `LIM_ID_FW_MAX_MA` expressed as a third of `LIM_IQ_MAX_MA` - the ratio it
   always was - rather than a literal that would have become 6% of the budget.

   `ident.h`'s excitation had to be re-sized, and that is the finding worth
   keeping: **its levels were per-unit of bus, and the bus went up 33x.**
   `IDENT_L_V_PU = 0.05` was 0.48 A of ripple on the bench and is 5.08 A here
   - not proportional, because the bus rose 33x while the winding only got
   4.7x more inductive. It lands just under the abort band, so identification
   simply failed. Both that and the R regulator's gain are now derived from
   physical quantities (a target ripple current, and volts per amp per second)
   and converted through the actual bus and tick rate.

   Still open: characterising Lq against current and lambda_m against
   temperature. `foc.h` names that as the deliverable that decides whether the
   HV current loop works, and it is a motor-test-rig task rather than a
   firmware one.

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

3. ~~**`N_POLES = 10` - poles or pole pairs?**~~ **Probably answered, worth
   confirming.** Read as ten *pole pairs*. The evidence is the sibling
   project: MiniFOCer, same author and same `defines.h` layout, sets
   `N_POLES` to 7 - and a motor cannot have an odd number of poles, since
   they come in north/south pairs. So `N_POLES` means pole pairs in these
   projects. That is an inference from a naming convention rather than a
   datasheet, and bring-up step 6 settles it in seconds: with the pole count
   wrong, an open-loop spin gives visibly the wrong number of electrical
   revolutions per mechanical one.

4. **The sense scales, and they are now known to be blocking.** `0.04 A/LSB`
   and `0.05 V/LSB` are inherited from the board's bring-up loop with no
   sensor part number and no divider ratio behind them. Against a 12-bit ADC
   they imply **+/-81.9 A and 0..204.8 V**. The EMRAX wants 141 A peak
   continuous and 339 A peak, on a pack that runs 350-588 V.

   So as scaled, the bus sense cannot read even the empty pack, and the
   current sense tops out below half the machine's continuous rating. Both
   fail in the direction that hurts: a saturated current reading makes the
   loop believe it has arrived while the real current climbs, and a saturated
   bus reading is divided by in `FOC_SetGainsForVbus`, so clamped at 204.8 V
   on a 518 V bus every gain comes out 2.5x too large.

   Schematic pending. Until it lands, `LIM_IQ_MAX_MA` is set from the
   *measurable* range rather than the motor - 65 A, 80% of the implied sensor
   ceiling, which is 46 Arms and comfortably inside the machine either way.

5. ~~**The RM44SI frame format.**~~ **Answered**, from the board's own SPI3
   interrupt handler: 14-bit frames, angle in the low 13 bits. What the 14th
   bit carries is still unknown - it is masked off, so a status or error flag
   living there is currently being ignored.

6. **The dead-time value in real units.** 160 counts at DT prescaler DIV1 is
   what is configured; the nanoseconds depend on the divider chain in RM0440
   and nobody here has confirmed it or seen it on a scope.

7. ~~**Which motor is actually attached.**~~ **Answered: EMRAX 228 HV on a
   140s2p pack.** The inherited constants turned out to be EMRAX constants
   rather than placeholders - `foc.h`'s HV analysis quotes 917 Hz electrical
   (10 pole pairs at 5500 rpm) and back-solves to exactly the 255 uH in the
   board's own defines. R and L should still be confirmed with `ident.c` on
   the machine, and lambda_m must be measured; see step 7 above.

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
