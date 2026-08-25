# makolongfin — HV-readiness work plan

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

Everything not listed here is done — see the closed branches and the comments
in `foc.h`, `drive.h`, `led.h` and `HARDWARE_NOTES.md` sections 10–12 for what
was found and why the numbers are what they are.

## A. Merge the branch stack

Phase 0 is on `main`. The rest is a dependent stack, none of it reviewed:

    phase1a-delay-comp → phase1b-decoupling → phase1c-bandwidth
      → phase2-fault-path → foc-decouple-fix
      → phase3-sensor-plausibility → phase6-build

`phase6-build` is the tip and carries everything after phase 2 — the decouple
regression fix, phase 3, phase 4, the build/CI work, the host harness, the LED
front panel and the plotting tools. Split it if the PRs need to be reviewable
one phase at a time.

## B. Finish 1b — the decoupling is implemented and disabled

`f->decouple` defaults to 0. It works when the parameters are right: in
simulation, peak |id| through an iq step is 121 mA off against 8 mA on, and on
the bench it measured −41% peak and −35% rms d-axis disturbance.

It is off because it costs 6× peak current on a reversal at terminal speed —
9326 mA against 1489 mA — and the cause is understood:

- `omega_e` comes from the position loop, filtered at ~8 Hz, a 20 ms lag.
- Under 1 A of braking this rotor decelerates at ~16 000 electrical rad/s², so
  during a reversal `omega_e` is wrong by ~320 rad/s.
- That is 0.86 V of feedforward error, and 0.86 V across an 85 mΩ winding is
  10.1 A. Measured 9.3.
- There is no headroom to correct it: at terminal speed `w_e·λ_m/Vbus` is
  0.250 against a `vmax` of 0.250.

**What it needs:** an `omega_e` for the feedforward that tracks faster than
8 Hz. Not a second estimator that can disagree with the position loop's — the
same encoder differences, filtered for a different purpose. The simulator can
now explore this directly: the filter constant is settable and there is no
noise floor to hide behind.

**Re-test with the demo, not a short reversal.** A 1.5 s reversal showed 1359
vs 1375 mA and looked like a pass, because the motor never reached the speed
where this bites. The bar is: d-axis disturbance down **and** peak current on a
full `foc_dash --demo` cycle no worse than with the flag off.

## C. Phase 5 — torque interface and field weakening (not started)

The interface is currently `iq_ref` in milliamps. The HV inverter's interface is
a torque request from the VCU.

- Torque → (i_d, i_q) map. The EMRAX 228 is axial-flux surface-PM, so
  L_d ≈ L_q and MTPA collapses to i_d = 0 — say this explicitly in a comment so
  the next person does not implement an IPM MTPA solver for nothing.
- **Field weakening:** a PI on (vmax − |v|) whose output drives i_d negative,
  clamped to negative values and to a magnitude bound in `limits.h`. Develop it
  here by lowering `FOC_VMAX_DEFAULT` artificially to force early saturation at
  bench speeds.
- **Torque plausibility monitor.** FSAE EV rules require detecting an
  implausible torque command and reaching a safe state within a bounded time.
  Implement the monitor and its timing, and put the rule reference in the
  comment. `DRIVE_FAULT_COMMAND` is already reserved for it.

Note that field weakening will interact with the anti-windup and the vector
limit, which is exactly where the last two control changes went wrong. Put it
through the host harness before the motor.

## D. Finish Phase 6 — the harness only covers `foc.c`

Done: warnings-as-errors, a real Release build, CI, and a host harness for
`foc.c` with seven tests and a mutant check.

Left:

- **Fault injection against `drive.c`.** The plan's list — encoder freeze,
  encoder jump, sensor rail, phase open, Vbus collapse — asserting the state
  machine lands in `FAULT` with the right cause each time. `pmsm.h` already
  carries the injection flags; nothing consumes them yet. This needs `drive.c`
  and its `motor_pwm` calls stubbed the way `foc.c`'s CORDIC was.
- **Verify cppcheck actually catches `errors & !MASK`.** CI runs it for exactly
  this, and it is unverified — no GCC warning flag catches that bug, tested.
  Plant a deliberate case and confirm the job goes red. Do not trust the green
  tick until you have seen it fail.
- **First CI run will need iterating.** It has never executed; the runner
  environment is the untested part.
- **Model fidelity: the sub-period ADC sampling instant.** `sim.h` applies
  duties with a 1.5-period delay while the firmware compensates 1.65, because
  the real ADC triggers before the period boundary. Until that is modelled the
  simulator cannot be used to judge phase 1a at all.
- **`tools/viz.py live` has never run against a board.** Read-only by
  construction; the plumbing is untested.

## E. Hardware requirements for the next board spin

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

## F. Motor characterisation for the HV build

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

## G. Loose ends worth closing

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
