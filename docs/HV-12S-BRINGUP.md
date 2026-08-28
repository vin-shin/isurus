# 12S bring-up prep — Mako Longfin at rated bus

Written 2026-08-28, before the first run above bench voltage. Nothing in this
document has been executed yet.

The bench has always run at **15.55 V** against a motor specified for **12S,
44.4 V nominal / 50.4 V full**. Going to 12S is a **2.9x** step at nominal and
**3.2x** at full charge. This is the prep for that: what scales, what already
handles it, what does not, and what has to be answered before the supply goes
past bench voltage.

Sources: `HARDWARE_NOTES.md` sections 6, 7, 8, 10 and 11, `Core/Inc/limits.h`,
`Core/Inc/foc.h`, `docs/OPEN-ITEMS.md`.

---

## 1. Blocking — the board's own voltage ceiling is not recorded

`limits.h` says this in as many words, and it is still true:

> `LIM_VBUS_MAX_MV` bounds the MOTOR only. The board's own ceiling - FET Vds,
> bulk capacitor rating, gate driver supply - is not recorded in
> HARDWARE_NOTES and has not been verified here.

The 50.4 V limit in firmware is the *motor's* rating. Nothing in this repo
records what the *inverter* is rated for. Three numbers are needed from the
schematic or the BOM before the supply goes past bench voltage:

1. **FET Vds.** Only the 220 A current rating is recorded (`limits.h`), never a
   voltage. A part chosen for a 24 V bench could be 40 V or 60 V; at 50.4 V bus
   plus switching overshoot, 60 V is not comfortable margin.
2. **Bulk capacitor voltage rating.** An electrolytic rated 25 V or 35 V is the
   likely failure at 44 V, and it fails loudly.
3. **Gate driver bias.** The UCC21330 is isolated with an 8 V UVLO, so its own
   supply is probably independent of the bus — but confirm the high-side bias
   source is not derived from the bus in a way that scales with it.

Until those three are written into `HARDWARE_NOTES` with a source, 12S is a
hardware question and not a firmware one. Everything below assumes they come
back clear.

## 2. What already handles the step correctly

Verified by reading the code, not by running it:

- **Current-loop gains track the bus.** `kp = w*L/Vbus`, `ki = w*R/Vbus`, both
  recomputed by `FOC_SetGainsForVbus` from the measured bus at ~750 Hz, with
  `vbus_track = 1` by default. Vbus is the plant gain, so this is the thing
  that would otherwise triple the loop gain at 12S. It is handled.
  `FOC_VBUS_NOM_MV = 15550` is only the point the *defaults* were sized at; it
  does not need changing.
- **The rescale window is wide enough.** `FOC_VBUS_MIN_MV = 6000`,
  `FOC_VBUS_MAX_MV = 60000`. 12S sits inside it.
- **The ADC divider has headroom.** 190k/10k gives Vbus/20, so 50.4 V presents
  2.52 V to a ~3.3 V ADC. Saturation is at ~66 V. No hardware change.
- **Current-sense scaling is unaffected.** `HARDWARE_NOTES` section 6: the rail
  voltage cancels out of the current scaling. Bus voltage never entered it.
- **The ISR budget is unaffected.** Nothing on the ISR path branches on bus
  voltage; the 21.1 us / 33.3 us figure carries over. Re-measure with
  `tools/isr_budget.sh` only if the ISR itself is changed (see 4.1).
- **Freewheel is still the correct safe state** — but for a reason that is not
  the one recorded. See section 3.

## 3. Freewheel vs ASC depends on vmax, not on bus voltage

`HARDWARE_NOTES` section 11 concludes freewheel is unconditionally correct "on
this bench" and attributes it to the 24 V supply. That framing does not survive
the algebra, and the correction matters at 12S.

Reachable electrical speed at the voltage ceiling, and the speed above which
freewheel back-charges the pack, are **both linear in Vbus**:

```
    w_reachable = vmax * Vbus / lambda_m
    w_crossover = Vbus / (sqrt(3) * lambda_m)

    w_reachable / w_crossover = vmax * sqrt(3)          <- Vbus cancels
```

So the safety of freewheel is set by `vmax` alone. Raising the bus does not
erode it; raising `vmax` does.

| vmax | `vmax * sqrt(3)` | |
|---|---|---|
| 0.25 (`FOC_VMAX_DEFAULT`) | 0.43 | freewheel safe, 2.3x margin, at any bus |
| 0.577 | 1.00 | the crossover |
| 0.60 (used in the section 7 speed test) | 1.04 | back-charge region, at any bus |

Two consequences:

- **Keep `vmax` at 0.25 for 12S bring-up.** At 0.25 freewheel remains correct
  at 44.4 V and at 50.4 V alike, with the same margin it had at 15.55 V.
- **The section 7 speed table was already marginally in the back-charge
  region.** At 15.3 V and `vmax = 0.60` the crossover is 3296 rad/s and the
  measured 1600 rpm is 3351 rad/s electrical — 1.7% over, which is inside the
  uncertainty on `lambda_m` and so is "at the crossover", not "proven past
  it". It did not matter at 15.3 V into a bench supply. At 44 V into a charged
  pack the same test is a different event, because what freewheel back-charges
  into is then a pack that can already be near its ceiling.

`Drive_ChooseSafeState()` computes the crossover from the live bus and does not
need changing — it will pick correctly. The point is that its answer changes
with `vmax`, so `vmax` is now a safety parameter and not just a performance one.

## 4. What gets worse by 3.2x

### 4.1 The overcurrent trip is main-loop software, and 12S is where that bites

`HARDWARE_NOTES` section 10 in full: no comparator, no HRTIM fault input
assigned, no `HRTIM_FLT*` pin in the `.ioc`. The trip is a poll in the
`CS_READ_EVERY` block of the **main loop**, ~750 Hz, ~1300 us worst case.

Current into a shorted bridge builds at `di/dt = Vbus / L`, L = 54.3 uH:

| bus | di/dt | reached in 1300 us (main loop) | reached in 33.3 us (one ISR) |
|---|---|---|---|
| 15.55 V bench | 286 A/ms | 372 A | 9.5 A |
| 44.4 V (12S nom) | 818 A/ms | 1063 A | 27.2 A |
| 50.4 V (12S full) | 928 A/ms | 1206 A | 30.9 A |

Against a 15 A trip, a 40 A sensor range and a 220 A FET rating. At 12S the
main-loop path lets a short pass the FET rating in ~270 us — five times over
before it looks. Moving the trip into the control ISR keeps it at 27-31 A,
inside both the sensor range and the FET rating.

That move is named in section 10 as "the fastest honest improvement available
in firmware... a 40x improvement and still 33x slower than the hardware would
be, so it is a mitigation, not the fix." At bench voltage it was optional. At
12S it is the difference between a trip and a destroyed bridge.

It touches the ISR, so it needs before/after `tools/isr_budget.sh` numbers on
target per the repo convention. Budget is 21.1 us of 33.3; two abs, two
compares and a branch is small, but it has to be measured, not estimated.

**The supply itself is the strongest protection available right now.** A
current-limited lab supply cannot deliver 1000 A no matter what the firmware
does; a 12S LiPo can. That alone argues for doing the entire bring-up on the
HV supply and not connecting a pack until the OC path is in the ISR.

### 4.2 The overvoltage trip is on the same slow path

`main.c` checks `g_cs.vbus_mv > LIM_VBUS_MAX_MV` in the same ~750 Hz block. At
15.55 V there was nothing that could push the bus up. At 12S there is: a lab
supply **cannot sink current**, so any deceleration or reversal pumps the
recovered energy into the bulk capacitance and the bus rises until something
absorbs it. The bulk cap is also the part whose voltage rating is unknown
(section 1).

This is the case for doing early motion tests against a **pack**, which absorbs
regen, rather than against the supply — and it is in direct tension with 4.1,
which argues for the current-limited supply. The resolution is ordering: static
and low-speed tests on the current-limited supply, and no test that decelerates
a loaded rotor until the OC path has moved into the ISR.

### 4.3 The VDDA latch bug now undermines the overvoltage trip

Top item in `docs/OPEN-ITEMS.md` section 2: `CSense_MeasureVdda` runs **once**,
from `CSense_Init`, and latches `s_vdda_mv` for the session. One boot in seven
read VDDA as 2890 mV instead of 3280 — 13% low.

`CSense_ReadVbus` derives the bus straight from that static:

```c
t->vbus_mv = ((v * s_vdda_mv) / CS_ADC_FULL_SCALE) * CS_VBUS_DIV_NUM / CS_VBUS_DIV_DEN;
```

So on a bad-VDDA boot every bus reading is 13% low. At 15.55 V that was a wrong
diagnosis (and cost one, on 2026-08-25). At 12S it means:

- **The OV trip does not fire until the real bus is ~58 V**, past the motor's
  50.4 V rating and past any plausible bulk cap rating.
- The self-test's OV check passes for the same reason.
- The gain rescale runs 13% hot.

At bench voltage this was a measurement annoyance. At 12S it is a silent,
1-in-7 defeat of the only overvoltage protection on the board. **Fix this
before 12S**, not after. It is a main-loop change with no ISR cost: re-measure
periodically, and sanity-check the result against the nominal rail so an
implausible VREFINT sample is rejected rather than latched.

### 4.4 Thresholds that need a decision

- **`LIM_VBUS_MAX_MV = 50400` is exactly a full 12S pack.** A freshly charged
  pack, or a supply set to its rated 50.4 V, sits *on* the trip. With ripple it
  trips on arrival. Either run the supply at 44-46 V, or give the trip headroom
  above 50.4 and accept that the trip no longer means "at the motor's rating".
  This is a decision, not a bug — but it has to be made deliberately before the
  first 12S power-on, or it will present as a mystery `DRIVE_FAULT_OVERVOLTAGE`.
- **`LIM_VBUS_WARN_MV = 45000` is defined and never used.** Nothing reads it —
  `grep` finds only the definition. There is no warning band before the hard
  trip. At 12S nominal (44.4 V) the bus sits just under it, so if it were wired
  up it would need re-siting too.
- **`OC_TRIP_MA = 15000` carries a stale comment** describing 3000 and 4000 mA
  and "the usual 3% modulation". At 12S the same modulation is a different
  current, so the comment's operating point no longer holds either. Cosmetic,
  but it is the comment someone will read while deciding whether to raise the
  trip.

## 5. What the drive will actually do differently at 12S

- **Speed no longer self-limits where it used to.** `docs/OPEN-ITEMS.md`
  section 1 leans on "a free-spinning rotor self-limits to base speed" — that
  ceiling scales with the bus, so at 12S it moves up by ~2.9x. In **torque
  mode** there is no speed clamp at all; `LIM_VEL_MAX_DPS = 3600` (600 rpm)
  binds only the velocity and position profiles. An unloaded shaft commanded in
  torque mode at 12S will go substantially faster than anything this bench has
  seen. Confirm whatever is on the shaft is rated for it before the first
  torque command.
- **Electrical frequency roughly triples**, so encoder sampling and the "speed
  deltas alias" warning in section 7 apply at a lower rpm than before. Sample
  continuously and accumulate.
- **Field weakening should stay disabled.** It is disabled, is known to be
  structurally starved by the anti-windup, and its `LIM_ID_FW_MAX_MA = 4000`
  bound is explicitly unverified against temperature rise. 12S is not the run
  to enable it on.

## 6. Proposed order

Nothing here is executable until section 1 is answered.

1. **Get the three board numbers** (FET Vds, bulk cap rating, gate bias) and
   record them in `HARDWARE_NOTES`. If any is below ~60 V, stop.
2. **Fix the VDDA latch** (4.3). Main loop only, no ISR budget impact.
3. **Move the OC trip into the control ISR** (4.1), with before/after
   `isr_budget.sh` numbers.
4. **Decide the OV threshold** (4.4) and set the supply accordingly.
5. **Confirm `vmax` is 0.25** and leave it there (section 3).
6. **Staged power-on, on the current-limited HV supply**, pack disconnected:
   15.5 V (reproduce a known-good result), then 24 V, 33 V, 44 V. At each step,
   before energising: check the reported `vbus_mv` against a meter — that is
   also the check on 4.3.
7. **Test with `id`, not `iq`** (section 7 of `HARDWARE_NOTES`): d-axis current
   holds the rotor still, so the current loop is characterised without the
   mechanics running away. The gains are rescaled per-bus, so the d-axis
   tracking result at 44 V should match the 1% figure recorded at 15.5 V. If it
   does not, the rescale is where to look first.
8. **Only then** anything that spins, and only after the OC trip is in the ISR.
