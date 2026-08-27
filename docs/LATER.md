# Deferred work — GR MotherFOCer

Things that are understood well enough to write down and deliberately not done
yet. Distinct from `PORT-POWER-UNIT.md` §5, which is what nobody can answer
from a desk, and from `HARDWARE-CHANGES.md`, which is what a board revision
should change.

Nothing here blocks bring-up steps 1–5.

---

## 1. Temperature: the KTY conversion has the wrong shape

`thermal.c` reads the motor KTY on PC4 and converts it with

```
R = Rpull * raw / (FULL - raw)
```

which is the ratiometric form for a sensor in a passive divider. **Schematic
sheet 9 is not that.** It is a two-stage TLV9302 circuit driving the sensor
node through a 100 Ω series resistor, with a 100k/100k network setting the
first stage's reference, a 3v3 clamp and filtering at the node, and a second
stage buffering to the ADC.

A driven node gives a **linear** relationship between resistance and pin
voltage, not a ratiometric one. So `Thermal_RawToOhm` needs replacing, not
retuning — `THERM_PULLUP_OHM` should disappear rather than acquire a better
value.

**Why it is safe to defer.** The over-temperature trip is live and the band
check in `Thermal_Read` is what stops the reading being trusted: anything
outside `TH_RAW_MIN..TH_RAW_MAX` faults as `DRIVE_FAULT_OVERTEMP` rather than
producing a plausible number. The failure direction that matters — reading
the winding *cooler* than it is, so the trip never fires — is the one the band
catches, because a wrong-shaped conversion of a real sensor lands outside the
band long before it lands at a plausible-but-cold value.

**What closing it needs.** The resistor values on sheet 9 did not render
legibly enough to commit to, and the topology decides which linear form
applies. The honest route is not to derive it at all: substitute two known
resistors for the KTY, read the ADC, and fit the line. That calibrates the
whole chain including the second stage's gain and offset, and it is the same
bench session as the current and bus calibrations in `PORT-POWER-UNIT.md` §5.

## 2. Temperature: are `TEMP_U/V/W` analogue or PWM?

`thermal.c` converts PA5/PA6/PA7 as ADC channels. They may not be voltages.

The UCC21756-Q1 carries an isolated analogue sense channel whose output,
`APWM`, encodes its measurement as a **duty cycle** — the datasheet offers it
for exactly this job, "temperature sensing with NTC, PTC, or thermal diode".
Each driver on schematic sheet 1 brings `APWM` off-sheet alongside `RDY` and
`FLT`, and there are three `TEMP_` nets for six drivers.

If those are the same signal, an ADC samples a square wave at an arbitrary
phase and averaging recovers the duty only crudely. The right reader is a
timer input capture.

**Why it is safe to defer.** Nothing reads the three stage channels. They are
converted into the DMA buffer and never looked at, so nothing is currently
wrong — the motor KTY on PC4 is a genuine analogue front end (sheet 9) and is
unaffected. **They must not be plumbed in as voltages until this is settled.**

**What closing it needs.** Follow one `TEMP_` net back to a driver pin on the
schematic, or put a scope on it. Five minutes either way.

## 3. Power-stage over-temperature has no trip

Once §2 is settled and the three channels are readable, they want the same
treatment the motor KTY already has: a limit in `limits.h`, a check in
`Drive_SelfTest`, a periodic check in `Drive_Step`, and a fault cause. The
machinery is all there — `DRIVE_FAULT_OVERTEMP` and the 200 ms monitor — so
this is mostly plumbing once the reader is right.

Worth stating what is protected today: **the motor winding, and nothing
else.** The SiC devices are 1200 V / 3.7 mΩ parts running a machine that peaks
at 339 A, so they are not close to their limits, but a failing cooling loop or
a blocked duct would be invisible to this firmware.

## 4. `tools/` has not been revisited

The SWD dashboards resolve symbols from the ELF at run time, so they should
survive the port, but nothing has been checked against hardware.

`isr_budget.sh` is the one that is definitely wrong: it names a 33.3 µs
deadline and it is 50 µs here. It also needs re-running from scratch — the
core is 25% faster and the ISR was substantially rewritten, so the 21.1 µs
figure in `CLAUDE.md` means nothing on this board. That measurement is
**owed**, per `CLAUDE.md`, and needs the target.

## 5. CAN FD

Not needed, and not a hardware change — the transceiver is already a
TCAN1044A and FDCAN2 is an FD controller. It would buy payload space for the
channels this port added, not precision. Reasoning in
`HARDWARE-CHANGES.md` §3.

## 6. Electrical zero does not survive a power cycle

The A1333 held `ZERO_OFFSET` in EEPROM, so encoder zero *was* electrical zero
and `foc.c` needed no offset term. This board's encoder has no equivalent
register, so alignment lives in `g_foc.elec_offset` — which is RAM.

It therefore has to be re-applied at every boot. Options are a flash-backed
constant, a CAN command from the VCU at startup, or an alignment routine that
runs as part of bring-up. None is written.
