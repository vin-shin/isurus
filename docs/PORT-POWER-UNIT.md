# Porting Isurus to the GR MotherFOCer

This branch (`power-unit`) retargets the Isurus stack from **Mako Longfin** —
a 15 V bench servo — to the **GR MotherFOCer**, an EMRAX 228 HV traction
inverter on a 140s2p pack. It follows the repo's convention that a hardware
target is a branch, not a repository.

**Status: the software port is complete and nothing has been run on hardware.**
It builds clean and 53 host tests pass. Several constants are derived rather
than measured, and those are listed in §5.

## Sources, and how far each can be trusted

| | |
|---|---|
| `docs/powerunit.pdf` | The schematic. Authoritative, and the only source for what a pin is *wired to*. |
| `docs/TLxxx-A2(T)PV.pdf` | Current sensor datasheet. Authoritative. |
| `gr_motherfocer` tree | The board's CubeMX project and bring-up code. **Trust the pinout, peripheral mapping and interrupt layout; do not trust its analogue scaling.** |

That last split cost real time and is worth keeping. The CubeMX output
describes real silicon — its `HRTIM1_TIMB_IRQHandler` independently confirms
Timer B as the intended control ISR, which this port had chosen on other
grounds. But its `0.04` A/count is a rounded copy of MiniFOCer's
`0.040584415584415584`, and its `0.05` V/count matches neither MiniFOCer's
divider nor this board's. Neither was derived for this hardware. Four commits
ending at "encoder works" is consistent with a project that never got as far
as calibrating current.

**Three of this port's own wrong turns came from trusting that code, or from
reasoning about part numbers, where the schematic or a datasheet was
available.** They are recorded in §4 rather than quietly corrected.

## 1. What the two boards do not share

| | Mako Longfin | GR MotherFOCer |
|---|---|---|
| Core clock | 128 MHz | 160 MHz |
| Switching | 30 kHz | 20 kHz |
| Bridge | 1 pin/phase, external inverter | 3 complementary pairs |
| HRTIM timers | A, B | B (U), F (V), C (W) |
| Dead time | gate driver RDT, ~185 ns | HRTIM, 160 counts ≈ 1.0 µs |
| Gate enable | PC5, **active low** | PC8 `DRV_RST`, **active high** |
| Gate driver status | none | 12 fault/ready lines, polled by `gatedrv.c` |
| Phase current | 2 phases via internal OPAMPs | 2 phases (U, W) + DC link, external op-amps |
| Current sensor | CT4022, ±40 A | Mornsun TL200-A2PV, ±500 A |
| Bus sense | PF0, 190k/10k divider | PA3, 400:1 into an **isolated** amp chain |
| Temperature | none | motor KTY + 3 power-stage channels |
| Comparator | — | COMP2 on the bus = **overvoltage** trip, unarmed |
| Encoder | A1333, 15-bit, SPI1, chip select | differential SSI-class, SPI3 + RS485, **XDIR** |
| CAN | FDCAN1 (PA12/PB8) | FDCAN2 (PB6/PB5), TCAN1044A |
| Debug output | SWD only | SWD + LPUART1 (PC0/PC1) |
| LEDs | 2 | none |
| Motor | 20 pole pairs, 12S, 22 A | EMRAX 228 HV, 10 pole pairs, 140s2p, 339 A peak |

Three of those rows bite hardest.

**The gate enable polarity is inverted.** Mako Longfin's PC5 drove a UCC21330
`DIS` pin directly, so low meant enabled. This board's PC8 is the other way
round in all three places its own code touches it. Carrying the old polarity
across energises the bridge at reset — and `MotorPwm_EmergencyStop` wrote the
raw BSRR bit, so from a fault handler it would have *enabled* the drivers.

**The complementary bridge changes what "off" means.** Mako Longfin's
`motor_pwm.h` warned that no MCU state turns every FET off, because a low pin
turned the *low-side* device on through the external inverter. Here the HRTIM
drives both devices directly with hardware dead time, so disabling the outputs
genuinely opens the bridge. That warning was rewritten, not copied — leaving
it would describe a hazard that no longer exists.

**Everything is a factor of ten larger.** 518 V against 15.55 V, 339 A against
12 A, 5600 rpm against 600. Most of the port's real bugs were constants that
were correct at bench scale and silently wrong at this one.

## 2. What carried over untouched

- `foc.c` — Clarke/Park, the PI current loops, third-harmonic injection, the
  CORDIC angle conversion
- `position.c` — position and velocity loops, motion profiling
- `haptic.c` — force-field rendering
- `openloop.c` — open-loop vector drive
- `can_proto.h`'s **framing** — identifiers, node addressing, command set

Note what is *not* on this list and was expected to be. `drive.c` gained an
over-temperature fault. `can.c` kept its protocol but had four telemetry
fields rescaled. `ident.c`'s excitation had to be re-sized. The host harness
needed `SIM_TS`, the motor model and four tests changed. "Only the hardware
layer moves" was the intent, and it did not survive contact with a 33x bus.

## 3. Module by module

Ordered so each step can be checked before the next can hurt anything.

1. **`board.h`** — **done.** The hardware map, and the record of what is not
   known about it. The only file a later board revision should need to touch.

2. **Clock and peripheral init** — **done.** 160 MHz from `BOARD_PLLN`.
   `gpio.c`, `hrtim.c`, `adc.c`, `spi.c`, `fdcan.c` and `usart.c` are on the
   new pinout.

   The debug console moved from USART1 (PB6/PB7) to **LPUART1 on PC0/PC1**.
   Not optional: PB6 is FDCAN2_TX, so the two MspInits were fighting over one
   pin and the winner was whichever ran first. Mako Longfin had no serial
   output at all, so this is new capability rather than a port.

   `MX_GPIO_Init` was driving **PA2 as a push-pull output** — PA2 is an
   analogue input — and PC0/PC1, which LPUART1 then took back. Both fixed.

3. **`motor_pwm.c` / `.h`** — **done.** Timers B/F/C, complementary outputs
   with HRTIM dead-time insertion, active-high gate enable, 20 kHz, ADC
   trigger lead derived from `BOARD_HRTIM_TICK_HZ`. The control ISR moved to
   the Timer B repetition event. `hrtim.c`'s MSP no longer claims PC8, which
   is the gate enable — `MotorPwm_GateInit` must stay that pin's only owner.

   **The high and low gates were swapped** on all three phases until the
   schematic was read. See §4.

4. **`csense.c` / `.h`** — **done.** Four channels off one HRTIM-triggered
   ADC1 sequence into a circular DMA buffer: phase U (PC3), phase W (PA0),
   DC link (PA1), bus voltage (PA3). The third phase is inferred as
   `-(iu+iw)`, exact in a three-wire machine.

   The board's own code left ADC1 free-running and read whatever DMA last
   wrote. Fine for a print loop; not for a current loop, which needs all
   channels from a known point in the switching period — `PWM_ADC_LEAD_NS`
   has the long version of why that reads perfectly in a static test and
   destabilises the loop the moment anything moves.

   Scaling derives from the measured VREF+ and the sensor sensitivity, never a
   literal. `CSense_Read` is split so the ISR path (`CSense_ReadPhases`) does
   phases only — bus voltage and DC link current feed the gain rescale and the
   voltage trips at a few hundred hertz, and converting them at 20 kHz would
   spend multiplies and divides inside a hard 50 µs deadline on values nobody
   reads that often.

5. **`thermal.c` / `.h`** — **done, and new.** ADC2 over DMA, free-running:
   the motor KTY on PC4 plus three power-stage channels on PA5/PA6/PA7.

   Load-bearing rather than a nicety. `LIM_IQ_MAX_MA` is the motor's *peak*
   rating, deliberately above its continuous one so bursts are available — so
   the current limit does not protect the winding from sustained overload and
   was never meant to. This does. The KTY81-2xx curve is inverted by bisection
   rather than the quadratic formula: sixteen iterations in the main loop
   where nothing waits, against a square root and a branch choice in fixed
   point.

   `DRIVE_FAULT_OVERTEMP` is checked both in `Drive_SelfTest` — arming into a
   motor left hot by the last run is what a periodic check cannot catch in
   time — and every 200 ms in `Drive_Step`. **A lost sensor faults as hard as
   a hot one**: a KTY adrift reads as a fixed, plausible, fictional
   temperature.

6. **`gatedrv.c` / `.h`** — **done, and new.** The twelve gate-driver status
   lines, polled from the control ISR in four port reads.

   Not EXTI, and the reason is worth keeping. It does not fit — PA9/PC9 and
   PD2/PB2 collide, and EXTI lines are numbered by pin rather than port — but
   more to the point it is not needed. The UCC21756 protects itself: DESAT
   turns the switch off in 200 ns without the MCU, and `FLT` is the driver
   *telling* us afterwards. What the MCU owes is to stop commanding, latch the
   reason and say which switch, none of which is measured in nanoseconds.

   `FLT` is active low and latches in the driver; `RDY` is active high, a
   power-good on that driver's isolated supply. Both are open drain, so
   `GateDrv_Init` gives all twelve pull-ups — until it ran they floated.

   `DRIVE_FAULT_GATEDRV` latches on `FLT`. **Not-ready is treated as a
   precondition**, like an undervoltage bus: the isolated supplies take
   milliseconds to come up, and latching there would need a manual clear after
   every quick power-on. Not-ready alone also does not trip a *running* drive
   — a driver that genuinely cannot drive asserts `FLT` too, and faulting on a
   marginal rail buys nothing.

   There is no per-switch clear: `FLT` clears by pulsing `RST/EN`, which is
   PC8 and shared by all six drivers, so recovery is a whole-bridge operation.

7. **`encoder.c` / `.h`** — **done, with the protocol unconfirmed.** SPI3,
   manual control of PA15, one 14-bit frame per read. The angle is the low 13
   bits, from `gr_motherfocer`'s own SPI3 interrupt handler.

   Counts are widened to the project's 15-bit convention rather than the angle
   path being rescaled: `foc.c`'s `counts << 17` is exact only because a half
   turn lands on the Q31 sign bit. The cost is stated where it belongs — the
   bottom two bits of every count are always zero.

   The A1333's whole register surface is gone, so electrical zero moves to
   `g_foc.elec_offset` and **no longer survives a power cycle**.

   **PA15 is `XDIR`, not a chip select.** See §4.

8. **`can.c`** — **done.** FDCAN2 on PB6/PB5, prescaler 8 → 10 so 1 Mbit
   survives 128 → 160 MHz. The segment split is untouched, so the 81.25%
   sample point is the one validated on the old board.

   **Four telemetry fields overflowed** and were rescaled — bus to centivolts,
   currents to deciamps, velocity to RPM. Commands were 32-bit and untouched,
   which leaves the two directions in different units; `docs/CAN_PROTOCOL.md`
   now carries a per-direction table.

9. **Motor constants and `limits.h`** — **done, one number unmeasured.**
   EMRAX 228 HV, 140s2p: 350 V empty, 518 V nominal, 588 V full.

   `foc.h` describes it — 10 pole pairs, 23.22 mΩ, 255 µH, Ld = Lq. The
   current-loop gains fell ~100x, almost all of it the bus. Holding 1 kHz of
   bandwidth costs phase margin at the lower switching frequency, 61° against
   70°, stated in the file rather than inherited silently.

   **`FOC_LAMBDA_M_WB` is provisional. Do not enable the feedforward or the
   decoupling until it is measured.** Three derivations span 35%: 60.1 mWb
   from `foc.h`'s own HV analysis, 54.4 mWb from kv as rpm per DC volt,
   44.4 mWb from kv as EMRAX publish it. The lowest is chosen because the
   error directions are not symmetric — too low and the integrator walks out
   the remainder; too high and the feedforward alone exceeds the modulation
   ceiling, a failure `foc.h` already records at 14% on a machine where
   back-EMF was a far smaller share of the bus. Here it is 58%.

   `limits.h`: 588/350 V pack bounds, 33600 deg/s (5600 rpm — 3600 would have
   capped this drive at 600 rpm), `LIM_IQ_MAX_MA` at the motor's 339 A peak,
   `LIM_ID_FW_MAX_MA` as the third of it that it always was.

   **`ident.h`'s excitation levels were per-unit of bus, and the bus went up
   33x.** `IDENT_L_V_PU = 0.05` was 0.48 A of ripple on the bench and 5.08 A
   here — tenfold, not proportional, because the bus rose 33x while the
   winding got only 4.7x more inductive. It landed under the abort band and
   identification simply failed. Both that and the R regulator gain are now
   specified as physical quantities and converted through the actual bus and
   tick rate.

10. **`led.c`** — **done, by removal.** `BOARD_HAS_LEDS` is 0. It had been
   writing BSRR at pins that stopped being outputs when `gpio.c` was
   retargeted. What is lost is worth knowing: the stage lamp asked the
   *hardware* whether the gates were live rather than the state machine, so a
   bench tool arming outside `Drive_Arm` could not produce a lamp that lied.

11. **`tools/`** — **not revisited.** They resolve symbols from the ELF at run
    time, so they should survive, but `isr_budget.sh` still names a 33.3 µs
    deadline and it is 50 µs here.

## 4. Where this port was wrong

Recorded because each has the same shape — trusting inherited code, or
reasoning about a part number, where a primary source was available.

**The high and low gates were swapped.** The schematic has `HG_U` on PA11 and
`LG_U` on PA10, and likewise for V and W. HRTIM output 1 carries the
programmed waveform and output 2 is its dead-time complement, so output 1 *is*
the reference — and here it is the **low** gate. Every commanded duty landed
on the low side, inverting the phase. All three inverted negates the applied
voltage vector, which turns the current loop into positive feedback. Worse,
the zero-duty case cleared the set source, leaving output 1 permanently low
and therefore the **high** gate permanently on: a commanded zero would have
clamped every phase to the positive rail.

Fixed by swapping the set and reset sources. Polarity inversion would *not*
have worked — dead time drives both outputs inactive during the dead band, so
inverting makes "inactive" high and drives both gates at once.

**A phase current sensor that does not exist.** PC3 is `UC_ISNS_U`, PA0 is
`UC_ISNS_W`, PA1 is `UC_ISNS_DC`, and **PA2 is not connected**.
`gr_motherfocer` reads a "V current" from PA0 and a DC current from PA2, and
this port copied it. So U and W were transposed, the V channel was the W
sensor read twice, and the DC link came from a floating pin — which a
three-phase common-mode correction then averaged into the two real
measurements. An "improvement" that was actively corrupting good data.

**The current sensor is not undersized.** The "200" in TL200-A2PV is IPN, the
effective range, not a ceiling; the measurement range is **±500 A** against a
339 A motor peak. A recommendation to fit a ±350 A part is withdrawn.

**And its sensitivity was wrong by 3.2x.** 10 mV/A was inferred from the 0.82
conditioning gain, on the assumption that a design fills its converter at full
scale. It does not — the gain exists to fit ±500 A into the ADC. The datasheet
says 3.125 mV/A, so 2.5625 mV/A at the pin and 314 mA per ADC count.

## 5. Still open

Firmware cannot close any of these.

1. **Two known bus voltages and their ADC codes.** The bus chain is a 400:1
   divider into an AMC0311 isolated amplifier into an MCP6496 difference
   amplifier. Firmware models the divider alone and assumes zero offset. Two
   points give gain and offset directly.

2. **Phase sense polarity, per phase.** The conditioning is known not to
   invert, so any flip is the sensor's convention plus conductor orientation.
   A reversed sensor corrupts the Clarke transform into a *rotating* error.

3. **`FOC_LAMBDA_M_WB`**, by spinning the motor with the feedforward disabled
   and reading back the voltage the loop demands. See §3.9.

4. **Dead time**, from the new FETs. See `docs/HARDWARE-CHANGES.md` §4.

5. **The gate driver part number**, for the polarity and latching behaviour of
   the twelve status lines. See `docs/HARDWARE-CHANGES.md` §1b.

6. **The encoder part and its protocol**, and the idle sense of XDIR.

7. **The ISR budget.** `CLAUDE.md` requires before/after numbers from
   `tools/isr_budget.sh` whenever the control ISR changes, and this port
   rewrote most of what it calls. The deadline moved from 33.3 µs to 50 µs and
   the core is 25% faster, so the old 21.1 µs figure means nothing here.
   **Owed, and needs the target.**

Deliberately deferred work — the KTY conversion's shape, whether `TEMP_U/V/W`
are analogue or PWM, power-stage over-temperature, `tools/` — is in
**[`LATER.md`](LATER.md)**. None of it blocks bring-up steps 1–5.

Hardware recommendations are in **[`HARDWARE-CHANGES.md`](HARDWARE-CHANGES.md)**.
The twelve gate-driver fault and ready lines listed there are now **read** —
see `gatedrv.c` — leaving the three TL200 `OCD` outputs as the remaining
unconnected fast protection.

## 6. Bring-up order

Steps 1–5 need no bus voltage beyond logic power and are worth completing
before the DC link is ever connected.

1. **Build and flash.** Confirm 160 MHz and that LPUART1 prints.
2. **Encoder only.** Turn the shaft by hand; the angle should sweep cleanly
   through a full turn. Settles §5.6.
3. **CAN**, loopback then against a host at 1 Mbit.
4. **ADC with the bridge disabled.** Confirm the zero-current offsets settle
   near mid-scale — the hardware intends *exactly* mid-scale, so a captured
   zero away from it means the conditioning is not what `board.h` says. Then
   two bus points against a meter: §5.1.
5. **HRTIM on a scope, gate drivers still disabled.** Confirm the
   complementary pairs, that the *high* gate carries the commanded duty, and
   measure the actual dead time: §5.4.
6. **Open-loop vector drive at low bus voltage** (`openloop.c`). First current
   in the motor. Confirms the pole count — a factor-of-two error shows as the
   wrong number of electrical revolutions per mechanical one — and phase
   polarity, §5.2.
7. **`ident.c`** for R and L, then `lambda_m` by measurement: §5.3.
8. **Closed-loop current**, then the outer loops. Not before §5.1, §5.2 and
   §5.3 are answered.
