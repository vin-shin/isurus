# Open items and known limitations — Isurus

Replaces the old `prompt.md` work plan, whose phases are done. What follows is
only what is still open, plus the limits that showed up while closing them.

The reasoning that used to live in the plan is in the code where it belongs:
the HV bandwidth derivation and the 441 V coupling term are in `foc.h`, the
missing hardware overcurrent path and phase-V sensor are `HARDWARE_NOTES`
sections 10 and 12, and the measurements from 2026-08-25 are in
`docs/BENCH-2026-08-25.md`.

---

## 1. The bench cannot answer several of the remaining questions

This is the single biggest constraint, and it is not a firmware problem.

**A free-spinning rotor self-limits to base speed.** With no load it
accelerates until the drive runs out of volts, then sits exactly at the
boundary where voltage headroom is zero. Field weakening is the mechanism that
would let it past that point, so testing weakening on a free shaft is circular:
measured 2560.9 ± 4.7 rad/s with it off against 2560.6 ± 5.6 with it on, a
0.1σ difference.

**The machine's own ripple sets a floor no amount of patience clears.** Phase
1a's transport-delay compensation is a −9 mA effect on peak |id| against a
29.5–46.6 mA noise floor: ~500–1300 reps for a 2σ read. That is not an
underpowered measurement, it is an infeasible one by that method.

**What would fix both: a load.** A dyno, a brake, anything that can hold the
rotor above base speed. It unblocks field weakening, the phase 5 torque
interface end to end, and the Lq characterisation below. Highest-value
purchase on this list by some distance.

## 2. Firmware issues found and deliberately left

Roughly in the order I would fix them.

**`CSense_MeasureVdda` still measures only at boot.** *Partly fixed
2026-08-28.* It ran once at boot and latched whatever came back. One boot in
seven read VREFINT 1728 against a stable 1522, putting VDDA at 2890 mV instead
of 3280 and every derived voltage 13% low for the whole session. With
`vbus_track = 1` that mis-scales the current-loop gains too, and at 12S it
moves the overvoltage trip to an actual 58 V — see
`docs/HV-12S-BRINGUP.md` section 4.3.

The sanity check is now in: the burst is retried up to `CS_VDDA_ATTEMPTS`
times and a result outside ±8% of nominal is rejected rather than latched,
falling back to the nominal rail (0.6% out) instead of a known-bad reading
(13% out). Rejections are counted in `vdda_rejects`.

**Still open:** it is not re-measured periodically, so drift with temperature
or load is not tracked. Doing so means retargeting ADC1 from `PF0` back to
VREFINT and returning it, while the drive may be armed — more invasive than
the startup guard, and not yet done. The retry's 1 ms settling delay is also a
hypothesis about the root cause, not a confirmed one; nobody has caught the
bad burst on a scope.

**Field weakening is starved by the anti-windup, structurally.** The loop
differences `vmax` against the demanded `|v|`; the back-calculation anti-windup
exists to stop that demand exceeding `vmax`. They work against each other by
construction. A weakening loop triggered on the *current* error
(`iq_ref - iq`) or on modulation index would not have this problem. The
present one is correct in simulation and will stay disabled until there is a
load to test it against.

**Its gain is calibrated for a headroom that does not occur.** 20 ms to wind
assumes over-demand of a tenth of the ceiling; the real figure at the
saturation boundary is nearer a thousandth, so it winds ~25× slower than
designed.

**The torque plausibility monitor's numbers are engineering defaults.** The
2000 mA band and 100 ms bound are mine, not the rulebook's, and the rule
citation is deliberately absent because numbering moves between editions. All
three want checking before scrutineering. This one is *enabled*.

**`g_cmd` can arm the bridge behind the state machine.** Pre-existing. The ISR
now safes the bridge and calls `Drive_LoopStopped` when the loop is disabled,
so it cannot be left live, but the arming path should still go through
`Drive_Arm`.

**Smaller, all real:** `viz.py live` plots the integer mirrors, which only
update while the loop runs — against a drive in `READY` it draws a flat line
from whenever the loop last ran and it looks like data. `step_trace.sh` safes
the power stage on exit but does not restore the correction flags it toggled.
`foc.h`'s `decouple` field is still commented "(default on)" while `foc.c` sets
it to 0. `HARDWARE_NOTES` section 5 points at the `hw-verification` branch,
which was pruned — the commits are still reachable from the trunk but the
sentence is wrong.

## 3. Simulator limitations

**The sub-period ADC sampling instant is still not modelled.** `sim.h` applies
duties with a 1.5-period delay while the firmware compensates 1.65, because the
real ADC triggers `PWM_ADC_LEAD_NS` before the period boundary. Until that is
modelled the simulator cannot judge phase 1a at all — which matters more now
that the bench provably cannot either.

**No harmonics and no cogging.** The model cannot produce the ~±0.4 A ripple
the real machine carries, so it is optimistic about d-axis disturbance and
cannot be used to reason about the residual that ripple causes.

**Saturation behaviour diverges from the bench.** In a reversal the model
spends 40–86% of the window at `vmax` where the bench spends 0.2–0.8%. The
model reaches a different terminal speed, so any conclusion that depends on how
long the drive is clipped needs checking against hardware rather than assumed.

**Anti-windup test coverage is one test deep.** Enabling the modelled velocity
path globally stopped `run.sh --mutants` catching the anti-windup mutant, which
is why that path is opt-in. Strengthen those tests before anything touches the
anti-windup again — field weakening already wants to.

## 4. Measurement discipline, learned the hard way

**Quote the sampling rate and the operating point with every number, or the
number means nothing.** The decoupling reversal test gave 1.16×, 1.43× and
1.00× on the same bench in one sitting. The only things that changed were the
capture decimation and how close to terminal the rotor was, and neither is
visible in the result. The damaging transient is ~4 ms wide; a decimated
capture records its true peak only by luck.

**A test that has never been seen to fail is not evidence.** The mutant checks
caught four separate weak tests written on 2026-08-25 — two in the drive fault
paths, and both would have shipped looking thorough. One assertion was outright
tautological. Anything added to the suites should come with a mutant.

**Docstring and code disagree more often than expected.** Three instances in
one day, two in `viz.py`: `select()` matched the rep number as well as the flag
against a docstring saying that blend was worse than not comparing;
`_find_step` searched for the edge when its docstring said the firmware's index
"is the answer". Both compiled, ran, and produced plausible wrong numbers.
Worth grepping for deliberately.

**A green tick nobody has seen go red is worth nothing.** CI had been failing
on every commit since it was written — a literal `\n` in the apt line, and
cppcheck run over vendor code it could never pass. Nobody read it because it
was always red, which is worse than never having run.

## 5. Hardware, for the next board spin

Both written up in full in `HARDWARE_NOTES`, both unchanged and still the
limiting items:

- **No hardware overcurrent path** (section 10). The trip lives in the main
  loop at ~750 Hz, so current can reach ~575 A against a 15 A trip before
  anything looks. It also bounds how hard any divergent behaviour can be
  investigated on the bench — testing stops early because the protection is
  slow, not because the answer was found.
- **No phase-V current sensor** (section 12). `i_v` is reconstructed, so the
  three-phase sum check is zero by construction and impossible.
- **The A1333 frame CRC needs the datasheet.** The encoder is checked for gross
  failure only — dead link, impossible motion — and should not be described as
  verified.

## 6. The largest open risk for the HV build

Unchanged: **characterise Lq against current and λ_m against temperature, and
budget for tables rather than two constants.** The derivation is in `foc.h`.
At 917 Hz and 300 A the `w_e·Lq·iq` coupling term is 441 V of a 600 V bus, and
a 20% error in Lq leaves 88 V uncancelled. The current loop is delay-limited to
1.8× f_e at 30 kHz, so bandwidth cannot reject that coupling and the
feedforward has to — which puts the risk on the motor parameters rather than on
the controller. A motor-test-rig task, and the same rig that would unblock
everything in section 1.
