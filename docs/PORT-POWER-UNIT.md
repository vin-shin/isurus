# Porting Isurus to the GR MotherFOCer

This branch (`power-unit`) retargets the Isurus stack from **Mako Longfin** to
the **GR MotherFOCer** inverter board. It follows the repo's convention that a
hardware target is a branch, not a repository.

The source of truth for the new hardware is the CubeMX project that shipped
with the board — `Inverter.ioc` and the LL code generated from it, in the
`gr_motherfocer` tree. `Core/Inc/board.h` is the transcription of that into
this project, and is the only file a later hardware revision should need to
touch.

**The `gr_motherfocer` hardware is the same board, except for the FETs.** That
raises how much of that project can be trusted, and it is worth being precise
about which parts:

- **Trust the pinout, peripheral mapping and interrupt layout.** These are
  CubeMX output describing real silicon on the real board. Its
  `HRTIM1_TIMB_IRQHandler` independently confirms Timer B as the intended
  control ISR, which is what this port had already chosen; its Clarke uses all
  three phase currents, which is what `csense.c` now measures.
- **Do NOT trust its analogue scaling.** `0.04` A/count is a rounded copy of
  MiniFOCer's precisely-derived `0.040584415584415584`, and its `0.05` V/count
  matches neither MiniFOCer's divider nor this board's 400:1. Neither was ever
  derived for this hardware. Four commits ending at "encoder works" is
  consistent with that.
- **Re-derive the DEAD TIME.** It is the one constant that "same except the
  FETs" directly falsifies — see below.

There is still no schematic, so what a pin is *wired to* remains outside the
source. Section 4 lists what that leaves open, and none of it should be closed
by guessing.

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

4. **`csense.c` / `.h`** — **done.** Five channels off one HRTIM-triggered
   ADC1 sequence into a circular DMA buffer: three phase currents, DC link
   current, bus voltage. Three phases are measured and the residual
   `iu+iv+iw` is reported and divided out as common-mode error; FOC still
   receives U and W, so `foc.c` is untouched. Scaling is derived from the
   measured VREF+ and the sensor sensitivity rather than from any literal.

   Wiring it up turned up two pin conflicts: `MX_GPIO_Init` drove PA2 - the DC
   link current input - as a push-pull output, and ADC1's MSP claimed PF0
   while claiming none of the five channels in use.

   What this replaced, for the record:

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

8. **`led.c`, `docs/LED_CODES.md`** — **done, by removal.**
   `BOARD_HAS_LEDS` is 0 and `led.c` compiles to nothing. It had been writing
   BSRR at pins that stopped being outputs when `gpio.c` was retargeted, so it
   was already doing nothing - just not saying so. The diagnostic that is lost
   is worth reading in `board.h`: the stage lamp asked the hardware whether
   the gates were live rather than asking the state machine, which on a 588 V
   bridge is the one indicator worth having without a debugger.

   ~~Superseded:~~ — no LED appears in the new pinout, and
   `led.c` drives GPIOB pins that this board configures as inputs. Either the
   LEDs move to whichever of the unexplained pins turn out to be LEDs, or the
   two-LED front panel and its documentation go away on this branch.

9. **`tools/`** — the SWD dashboards resolve symbols from the ELF at run time,
   so they survive the port. They do assume `PWM_FREQ_HZ` and encoder counts;
   both come from headers, so they should follow automatically. `isr_budget.sh`
   needs re-running from scratch: the deadline moves from 33.3 us to 50 us at
   20 kHz, and the core is 25% faster, so the existing 21.1 us figure means
   nothing here.

## 4. What the schematic answered, and what is still open

`docs/powerunit.pdf` arrived after most of this port was written. It resolved
five of the open questions, corrected two firmware bugs, and left three
things open that no schematic can settle.

### Answered — and two of them were bugs

**The twelve "unexplained" pins are all gate-driver status lines.** Six
`DRV_RDY_*` and six `DRV_FLT_*`, one pair per switch:

| | ready | fault | | ready | fault |
|---|---|---|---|---|---|
| U high | PB4 | PD2 | V high | PC9 | PA8 |
| U low | PA9 | PA12 | V low | PB14 | PB15 |
| W high | PB0 | PB11 | W low | PB10 | PB2 |

**None of them are read.** That is now the largest protection gap on the
board — see `board.h` section 9 for why they are also the *fastest*
protection available, and what implementing them needs.

**The high and low gates were swapped in `motor_pwm.c`.** The schematic has
`HG_U` on PA11 and `LG_U` on PA10, and similarly for V and W — the opposite
of what this port assumed. HRTIM output 1 carries the programmed waveform and
output 2 is its dead-time complement, so the commanded duty was landing on the
*low* gate: every phase inverted, which negates the applied voltage vector and
turns the current loop into positive feedback. Fixed by swapping the set and
reset sources so output 1 is low across the middle of the period; compare
values, centring and ADC trigger position are all unchanged.

**There are two phase current sensors, not three.** PC3 is `UC_ISNS_U`, PA0 is
`UC_ISNS_W`, PA1 is `UC_ISNS_DC` — and **PA2 is not connected**.
`gr_motherfocer`'s bring-up code reads a "V current" from PA0 and a DC current
from PA2, and this port copied that mapping. So U and W were transposed, the
"V" channel was the W sensor read twice, and the DC link was being read from a
floating pin — which the three-phase common-mode correction then averaged into
the two real measurements. Reverted to measuring U and W and inferring
`iv = -(iu+iw)`, which is exact in a three-wire machine.

**COMP2 is an OVERVOLTAGE trip.** Its input PA3 is `TS_VSENSE`, the tractive
system bus. That is the right thing for this machine to have — regen into a
full or disconnected pack climbs faster than a 20 kHz loop will catch — but
the DAC threshold is still unset and `COMP1_2_3_IRQHandler` is an empty stub.

**PA15 is `XDIR`, a transceiver direction pin, not a chip select.** The
encoder link is differential, through SN65176B RS485 transceivers and an
NXU0304BQ level shifter: clock pair, bidirectional data pair, direction
control. That is an SSI/BiSS/EnDat-class interface. The current code drives
PA15 low for the frame and calls it a chip select, which happens to hold the
transceiver in receive and so may work — for the wrong reason, and it can
never transmit.

### Still open

1. ~~**The current sense chain.**~~ **Closed, from the datasheet.**

   Conditioning, from sheet 6: a difference amplifier per channel across the
   sensor's own `VOUT`/`VREF` pair, 10k in and 8k2 feedback, referenced to
   `VREFHALF`:

   ```
   V_pin = 0.82 * (ISNS - IREF) + VREFHALF
   ```

   The sensor's zero cancels in *hardware*, `VREFHALF` is `VREF/2` so zero
   current lands on ADC code 2048 by construction, and the stage does not
   invert.

   Sensor, from `docs/TLxxx-A2(T)PV.pdf`: **G = 3.125 mV/A**, `Vref` 2.5 V,
   `Vout = Vref + G*Ip`. So **2.5625 mV/A at the pin, 314 mA per ADC count**,
   and the converter clips at ±644 A.

   **Two earlier conclusions on this branch were wrong and are withdrawn.**
   The "200" in TL200-A2PV is the *effective* range IPN, not a ceiling — every
   part in the family gives ±0.625 V at its own IPN — and the **measurement
   range is ±500 A**. So the sensor is not undersized; the ±350 A part
   recommended earlier is unnecessary. And inferring 10 mV/A from the 0.82
   gain, on the assumption the design fills the ADC at full scale, was wrong
   by 3.2x: the gain exists to fit ±500 A into the converter, not IPN.

   Both mistakes came from reasoning about a part number instead of reading
   the part.

   Left over: the **OCD pin trips at ±400 A** and is not connected — above the
   motor's 339 A peak so it cannot nuisance trip, below the sensor's range so
   the reading is still good when it fires, and a comparator inside the sensor
   so it responds in 0.3 µs. Three of them, unconnected.

2. **The bus sense gain AND offset.** Sheet 5 shows the divider is only the
   first third: 400:1, then an AMC0311 reinforced isolated amplifier on an
   isolated supply, then an MCP6496 difference amplifier biased from 3V3A. The
   firmware models the divider alone with zero offset. Two known bus voltages
   and two ADC codes give both terms directly, and measure what the board does
   rather than what three datasheets say it should.

3. **The dead time, from the new FETs.** See §3 item 6.

4. **Gate driver part number**, for the polarity and latching behaviour of the
   twelve lines above.

5. **The encoder part and its protocol**, and the idle sense of XDIR.

6. **Phase sense polarity**, still. The conditioning is now known not to
   invert, so any sign flip is the sensor's own convention plus which way the
   conductor passes through it — which a schematic cannot show.

7. **`lambda_m`**, which is a measurement on the machine and not a document.

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
