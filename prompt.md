# Isurus — HV-readiness work plan

Read `CLAUDE.md` and `HARDWARE_NOTES.md` before touching anything. Both contain
constraints that are not obvious from the code.

## Context

This repo is a 12S FOC drive on an STM32G474, 30 kHz HRTIM control loop, A1333
encoder, CT4022 TMR current sensors on U and W. It works. The bench supply has
been running at ~22–24 V rather than the nominal 44.4 V.

It is also the prototype for a **600 V / 300 A SiC traction inverter** (EMRAX
228 HV, 10 pole pairs, R = 23.22 mΩ, L ≈ 255 µH, ~5500 rpm max → **917 Hz
electrical**) for a Formula SAE Electric car. The point of the work below is to
develop and validate, at 48 V where mistakes are cheap, the firmware
architecture that the HV inverter needs and this codebase currently lacks.

Every task below is judged by that standard: does it transfer, and is it
correct at 917 Hz electrical rather than the ~280 Hz this bench reaches today.

## Hard constraints

1. **ISR budget is the binding constraint.** The 30 kHz control ISR has a
   33.3 µs deadline and currently runs **24.3 µs — 73% loaded**, leaving
   ~9 µs. Measure before and after with `tools/isr_budget.sh` and report the
   delta in the commit message. **If a change pushes worst case past 30 µs,
   stop and say so rather than shipping it.**
2. **Derive the loop rate from `PWM_FREQ_HZ`** (`Core/Inc/motor_pwm.h`). Never
   hard-code 30000, 33.3 µs, or a period in seconds.
3. **No dynamic allocation.** No `malloc`, no VLAs, no recursion in the ISR
   path.
4. **`FocState_t` and `PosState_t` field order matters.** The SWD tools address
   fields by fixed byte offset. New fields go at the **end**.
5. **No AI attribution** in commits, PR bodies, or comments — see `CLAUDE.md`.
6. Match the existing comment style: explain *why* a number is what it is and
   what breaks if it changes, not what the line does.
7. One phase per PR. Do not batch.
8. **Nothing goes on by default that has not been measured on the motor.** Two
   corrections were shipped enabled on reasoning alone and both made the bench
   worse. `f->decouple` and `f->dtc_pm` are the standing examples.
9. **A test that has never been seen to fail is not evidence.** `run.sh
   --mutants` exists because of this. Applies to bench measurements too: check
   the sample count resolves the effect before believing a number.

---

# What is left

Everything not listed below is done and merged. The dependent stack that used
to live in branches — 1a through 3, plus phase 4 and the phase 6 build work —
landed on `mako-longfin` through PRs #1–#5, so there are no phase branches left
to go looking for. See the comments in `foc.h`, `drive.h`, `led.h` and
`HARDWARE_NOTES.md` sections 10–12 for what was found and why the numbers are
what they are.

## A. 1b — PARKED. The premise did not survive measurement

`f->decouple` defaults to 0 and should stay there for now, but not for the
reason this section used to give.

**What still holds.** The decoupling works. Measured 2026-08-25 on a step at
matched operating points, 12 reps per arm:

| step | decouple off | decouple on | benefit |
|---|---|---|---|
| 400 → 1200 mA | 245 mA | 105 mA | −57% peak, −55% rms |
| 400 → 2400 mA | 336 mA | 106 mA | −68% peak, −59% rms |

The benefit grows with current, as `w_e·Lq·iq` predicts, and the simulator
reproduces both the effect and the trend (−64% / −70%). See
`docs/BENCH-2026-08-25.md`.

**What does not hold: the 6× reversal penalty.** It was re-measured at
terminal speed and does not reproduce.

| capture | decouple off | decouple on | ratio |
|---|---|---|---|
| decim 24, 83% of terminal | 1224 mA | 1421 mA | 1.16× |
| decim 24, at terminal | 1227 mA | 1761 mA | 1.43× |
| **full ISR rate, at terminal** | **2333 ± 253 mA** | **2340 ± 372 mA** | **1.00×** |

Three answers from the same bench in one sitting. The only variables were the
sampling rate and how close to terminal the rotor was, and neither is visible
in the number. The last row is the trustworthy one: the damaging transient
sits at t = 4 ms — the step itself — and lasts a few ISR ticks, so a decimated
capture records its true peak only by luck. That aliasing is what produced the
apparent penalty, and the apparent divergence across reps with it.

The stated mechanism does not survive either:

- **Integrators do not accumulate.** Captured at both rep boundaries,
  `id_integ` entering each rep is bounded with no trend and headroom is
  0.006–0.009 on every rep, outliers included.
- **The arithmetic assumes volts that a saturated drive cannot apply.**
  "0.86 V across an 85 mΩ winding is 10.1 A" requires those volts to reach the
  winding. This section says two sentences earlier that there is no headroom —
  `w_e·λ_m/Vbus` 0.250 against a `vmax` of 0.250 — and the vector limiter
  clips the command. Both cannot be true.
- The 1489 mA baseline does not reproduce either. A direct terminal-speed
  reversal peaks at ~2.3 A with everything off.

**Before anyone reopens this**, the one thing not replicated is the demo's
actual profile — 0 → ±0.5 → ±1.0 with dwells, rather than a direct ±1 A flip.
That is the only place the 9326 mA can still be hiding. Capture it at **full
ISR rate**, and quote the sampling rate and the pre-step `w_e·λ_m/Vbus` with
any number that comes out. Without both, a figure for this test means nothing.

Do not write the faster `omega_e` on the strength of the old text. It targets a
static lag error, and there is currently no measured effect for it to fix.

## B. Phase 5 — torque interface and field weakening (not started)

The interface is currently `iq_ref` in milliamps. The HV inverter's interface is
a torque request from the VCU.

- ~~Torque → (i_d, i_q) map.~~ Done 2026-08-25. `FOC_TorqueToIq` /
  `FOC_IqToTorque` / `FOC_SetTorque` in `foc.c`, with `FOC_KT_NM_PER_A` derived
  from `FOC_POLE_PAIRS` and `FOC_LAMBDA_M_WB` rather than written down. The
  "do not implement an IPM MTPA solver" note is in `foc.h` and a mutant
  (`torqid`) fails the suite if anyone drives `id` off zero.

  Reachable as **CAN `0x0A SET_TORQUE_MNM`**, milli-newton-metres.
  `0x02 SET_TORQUE` keeps its existing meaning — milliamps of iq, whatever its
  name suggests. Redefining 0x02 would have been tidier and genuinely
  dangerous: same identifier, same four-byte length, silently different
  meaning, and a host still sending `1000` goes from asking for 1 A to asking
  for ~12 A with nothing reporting an error.

  `FOC_SetTorque` returns **the torque it accepted**, not the one requested,
  and that return value is load-bearing: the plausibility monitor compares
  delivered against *commanded*, so a VCU over-asking would otherwise look
  implausible and fault a drive that was behaving correctly. Mutant
  `torqecho` covers it.
- ~~**Field weakening.**~~ Implemented 2026-08-25 and **shipping disabled**.
  A PI on (vmax - |v|) driving `i_d` negative, clamped to negative values and
  to `LIM_ID_FW_MAX_MA` (4 A of the 12 A budget). ISR 24.77 -> 24.96 us
  disabled, 26.01 us / 78.0% enabled - inside the 30 us bar either way.

  Works in the model: inert while the bus has headroom, then +56% q-axis
  current at w_e 2200 and more above that. Five tests, three mutants.

  **The bench cannot test it, structurally.** Twelve reps per arm at terminal
  speed: iq +60 +/- 126 mA (0.5 sigma), terminal speed -0.3 +/- 2.1 rad/s
  (0.1 sigma). Two things hold the error at zero and both behave correctly -
  the back-calculation anti-windup exists to keep the demand at `vmax`, which
  is the very quantity this loop differences against it; and a free-spinning
  rotor self-limits to the boundary where headroom is zero by definition.
  Weakening is what would let it past that, so the test is circular.

  The plan's suggestion of lowering `FOC_VMAX_DEFAULT` does not work here
  either: at bench speeds back-EMF alone is ~0.138 pu against a demand of
  ~0.149, so the window between "cannot hold speed" and "not saturated" is
  about 8%. **Testing this properly needs a dyno**, or any load that can hold
  the rotor above base speed.

  The 20 ms wind time is calibrated for a headroom of a tenth of the ceiling.
  The headroom actually seen at the boundary is nearer a thousandth, so the
  loop winds ~25x slower than designed. Revisit against a real over-speed
  condition, not against this bench.
- ~~**Torque plausibility monitor.**~~ Done 2026-08-25. `Drive_TorqueMonitor`
  in `drive.c`, 100 ms bound, faults as `DRIVE_FAULT_COMMAND`. Six host tests
  and three mutants. Runs from the MAIN LOOP, not the control ISR: measured
  before 24.77 us / 74.3%, after 24.81 us / 74.4% — five cycles, noise.

  Two design points worth knowing before touching it:

  - **A voltage-limited drive is not an implausible command.** At terminal
    speed this bench saturates (measured `w_e·λ_m/Vbus` 0.242–0.244 against a
    `vmax` of 0.250), and a saturated drive cannot deliver the current it was
    asked for. A naive requested-vs-delivered monitor would trip every time
    the car reaches top speed. The accumulator is HELD while the vector limit
    is active — held, not cleared, so an implausible command cannot hide
    behind intermittent saturation — and the held time is counted separately.
  - **A scheduling gap is not evidence either.** If the monitor is not called
    for a while, the elapsed time is not charged; it re-baselines. Otherwise
    one 100 ms hiccup in the main loop is a false trip, and a safety monitor
    that fires spuriously is one that gets disabled.

  **The rule number is deliberately not cited.** The requirement is stable but
  the numbering moves between rulebook years, and a stale citation in a safety
  comment reads as though someone checked. Fill it in from the edition the car
  is entered under, and check the 2000 mA band and the 100 ms bound against
  that edition — they are engineering defaults, not quotations.

Note that field weakening will interact with the anti-windup and the vector
limit, which is exactly where the last two control changes went wrong. Put it
through the host harness before the motor.

## C. Finish Phase 6 — the harness only covers `foc.c`

Done: warnings-as-errors, a real Release build, CI, and a host harness for
`foc.c` with seven tests and a mutant check.

Left:

- ~~**Fault injection against `drive.c`.**~~ Done 2026-08-25.
  `test/host/test_drive.c`, 15 tests, `drive.c` compiled as it ships with
  `drive_stubs.c` supplying the gate driver, both sensors and the clock.
  Covers silent encoder, all-ones frame, current-sense zero in and out of
  tolerance, bus collapse, overvoltage, failed bus read, and the machine's own
  rules: FAULT latches, a clear re-runs the checks, arming is refused while
  latched, the first cause wins, the gate never precedes the outputs.

  Four `drive.c` mutants guard it and **two of them escaped the first version
  of these tests** — worth recording, because both looked covered:

  - the clear-goes-straight-to-READY mutant survived because the assertion was
    tautological. The discriminating case is clearing while the cause is still
    true, which must land back in `FAULT`; asserting only the healthy clear
    cannot tell the two apart, since both end in `READY`.
  - the no-EmergencyStop mutant survived because `Drive_Enter` *also* takes the
    bridge down on the way into any non-RUN state. "The bridge ended up down"
    is therefore not evidence the fault path ran. The property with teeth is
    that `MotorPwm_EmergencyStop` was called at all — freewheel first,
    unconditionally, before any bookkeeping.

  Closed-loop injection through the PMSM model is still open: `pmsm.h`'s flags
  (`enc_frozen`, `enc_jump`, `sensor_rail_u`, `phase_open_u`) remain unused.
  Those exercise the RUN-time detectors, which live in `main.c` and
  `position.c` rather than `drive.c` and so need a different harness.
- ~~**Verify cppcheck actually catches `errors & !MASK`.**~~ Done 2026-08-25.
  Planted in `encoder.c`'s A1333 ready-bit test on a scratch branch: the
  firmware job compiled it clean and cppcheck went red with `clarifyCondition`
  and `knownConditionTrueFalse`. Both are **style**-class, so `--enable` must
  keep `style` — dropping it to quieten unrelated noise disables the check
  silently. Recorded in the workflow.
- ~~**First CI run will need iterating.**~~ Done 2026-08-25, and it needed it.
  CI had in fact been running and failing on *every* commit since the first.
  Two independent causes: a literal `
` in the apt line, so the toolchain
  never installed and the firmware job never compiled anything; and cppcheck
  running over all of `Core/Src`, where 14 of 18 findings were
  `constParameterPointer` on HAL callbacks nobody here can change. All three
  jobs green as of `81c4c94`.
- **Model fidelity: the sub-period ADC sampling instant.** `sim.h` applies
  duties with a 1.5-period delay while the firmware compensates 1.65, because
  the real ADC triggers before the period boundary. Until that is modelled the
  simulator cannot be used to judge phase 1a at all.
- **`tools/viz.py live` has never run against a board.** Read-only by
  construction; the plumbing is untested.

## D. Hardware requirements for the next board spin

Neither is firmware's to fix. Both are written up in full.

- **No hardware overcurrent path** — `HARDWARE_NOTES` §10. There is no
  comparator on the sense path and no `HRTIM_FLT` pin assigned; the trip lives
  in the main loop at ~750 Hz, so current reaches ~575 A against a 15 A trip
  before anything looks. Route a current-limit comparison to an HRTIM fault
  input. Note PC3 is not a comparator input on this part, and whether PA1 is a
  `COMP1_INP` was **not** verified — check the datasheet, not memory.
- **No phase-V current sensor** — `HARDWARE_NOTES` §12, confirmed 2026-08-25.
  So the three-phase sum check is impossible: `i_v` is reconstructed and the
  sum is zero by construction. A third sensor buys a real sum check and better
  common-mode rejection from the full 3-phase Clarke.

Also outstanding and not firmware: **the A1333 frame CRC needs the datasheet.**
The encoder is checked for gross failure only — dead link, impossible motion —
and should not be described as verified.

## E. Motor characterisation for the HV build

The conclusion of the bandwidth derivation in `foc.h`, and the largest open
risk. At 30 kHz the EMRAX current loop is delay-limited to 1.8× f_e; the
conventional 5–10× rule needs 83–165 kHz, which is not a switching frequency at
600 V / 300 A. So bandwidth cannot reject the cross-coupling and the
feedforward has to — which moves the risk off the controller and onto the motor
parameters:

    w_e·Lq·iq at 917 Hz and 300 A = 441 V, 73% of a 600 V bus
    20% error in Lq              =  88 V uncancelled

**Characterise Lq against current and λ_m against temperature, and budget for
tables rather than the two constants that suffice at 48 V.** This is a
motor-test-rig task. It decides whether the HV current loop works.

Do not spend thermal budget buying switching frequency for control margin: it
buys 1.2× versus 1.8× of f_e against a requirement of 5×.

## F. Loose ends worth closing

- **`g_cmd` can arm the bridge behind the state machine's back.** That is how
  `foc_dash` works, and it means `g_drive.state` can disagree with the
  hardware. The ISR now safes the bridge and calls `Drive_LoopStopped` when the
  loop is disabled, so it cannot be left live — but the arming path should go
  through `Drive_Arm` too.
- **The demo's 1489 mA baseline** is 1.5× the commanded current with every
  correction off. The current loop itself is clean — a step from rest peaks at
  988 mA against a 1000 mA command with no overshoot — so this is ripple at
  speed plus reversal transients. The 6th-harmonic component is *not* mostly
  deadtime; that was measured and compensating it does not help. Most of it is
  likely the machine's own back-EMF harmonics and cogging, which would need
  harmonic injection to cancel and is beyond this plan's scope.

---

## Corrections to the original plan, kept because they cost time to find

- **`-Wall` does not catch `errors & !MASK`.** Nor does `-Wextra`,
  `-Wlogical-op`, `-Wint-in-bool-context`, `-Wbool-operation` or
  `-Wconversion` — all tested. GCC compiles it to `movs r0, #0`: the interlock
  becomes unconditional dead code with no diagnostic at any level. That class
  needs a static analyser.
- **Deadtime is not invisible at 48 V** — but it is also not the problem. The
  reasoning in the original plan ran from bus voltage; what matters is winding
  impedance. 185 ns × 30 kHz × 22.4 V is 0.124 V, and across 85 mΩ that is
  1.5 A against a 1.0 A command. Compensating it measurably does not help
  though, so the term is implemented and disabled.
- **Phase 1a's acceptance criterion cannot be met at 48 V.** Steady-state `id`
  at `id_ref = 0` is nulled by the d-axis integrator whether or not the inverse
  Park angle is right, so that test is blind to the thing it was meant to
  check. The step test does not resolve it either — the effect is ~16 mA under
  a ~20 mA noise floor. 1a stands on derivation, and the mechanism is verified
  (3.90° applied against 3.96° predicted).
- **The transport delay is 1.65 periods here, not 1.5.** The ADC triggers
  `PWM_ADC_LEAD_NS` before the period boundary. Derived in `foc.c` from
  `PWM_ADC_LEAD_NS` and `PWM_FREQ_HZ` so it tracks both.
