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

## 2. Temperature: `TEMP_U/V/W` are APWM — confirmed

They are the isolated analogue sense outputs of three UCC21756-Q1 gate
drivers. The driver reports its measurement as a **duty cycle on a 400 kHz
carrier**, not as a voltage.

From the datasheet:

| | |
|---|---|
| `I_AIN` | 203 µA constant current source into the sensing node |
| `V_AIN` | 0.6 – 4.5 V usable, so a **3.0 – 22.2 kΩ** sensor window |
| `f_APWM` | 400 kHz (380–420) |
| `D_APWM` | 88% at 0.6 V, 50% at 2.5 V, 10% at 4.5 V |

Those three points are exactly linear at −20%/V, so

```
V_AIN    = 2.5 + (50 - D) / 20        volts, D in percent
R_sensor = V_AIN / 203 µA
```

The driver biases the thermistor itself, which is why there is no external
divider to find on the schematic.

**The three ranks have been removed from the ADC2 sequence.** Sampling a
400 kHz carrier at a 20 kHz conversion trigger is not merely crude, it is
degenerate: exactly twenty carrier periods per sample, so the samples are not
even randomised across the duty cycle. They land wherever the fixed phase
relationship puts them and average to a number that is stable, plausible and
unrelated to temperature — worse than noise. Leaving that in a buffer named
for temperature is how it gets read as temperature.

### Two ways to get them back

**An RC per channel, and they become ordinary ADC inputs again.** 10 kΩ and
100 nF puts the corner at 159 Hz, which is 68 dB below the carrier, and
temperature needs no bandwidth at all. Two passives per channel, and the ADC
path that already exists starts working. It is a board change, so it is in
`HARDWARE-CHANGES.md` §5.

**Or timer input capture with DMA.** PA5, PA6 and PA7 all have capture
alternate functions on this package — TIM2_CH1, TIM3_CH1 and TIM3_CH2, which
wants confirming against the AF table. At 160 MHz a 2.5 µs period is 400 timer
counts, so the 10–88% duty span is 312 counts of range: plenty of resolution.
Firmware only, but 800k edges per second across three channels means DMA
rather than interrupts, and it is meaningfully more work than two passives.

Either way, §3 below is what follows.

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
