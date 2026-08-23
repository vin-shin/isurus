# Hardware Notes — makolongfin2

Board-level quirks for this motor controller that are **not** derivable from the
source tree. Read this before debugging a board that "doesn't run".

Target: **STM32G474RET6**, LQFP64, 512 KB flash / 128 KB RAM.

---

## 1. BOOT0 / FDCAN1_RX pin conflict (critical)

### TL;DR

Pin 44 on LQFP64 is **PB8-BOOT0**, and this board routes **FDCAN1_RX** to it. The CAN
transceiver drives that line high when idle, so the MCU samples `BOOT0 = 1` at every
reset and boots the **ST system bootloader** instead of your application.

**Fixed in the option bytes:** `FLASH_OPTR = 0xF9EFF8AA` (`nSWBOOT0 = 0`, `nBOOT0 = 0`).

> The fix lives in the **chip**, not in this repo. A new assembled board, or a full
> chip erase that restores default option bytes, will reproduce the bug exactly.
> See [Re-applying the fix on a fresh board](#re-applying-the-fix-on-a-fresh-board).

### Symptom

- The application never runs. LED dead, no UART, no CAN.
- A debugger attaches fine but drops you on `Reset_Handler` in
  `startup_stm32g474xx.s`, making it look like the code is resetting in a loop.
- Halting shows `pc` somewhere in `0x1FFF0000`–`0x1FFF7FFF` (e.g. `0x1fff5048`).

This is extremely misleading: every visible symptom points at application code, but
the CPU is not executing a single instruction from `0x08000000`.

### Root cause

From `makolongfin2.ioc`:

```
Mcu.Package = LQFP64
Mcu.Pin44   = PB8-BOOT0
PB8-BOOT0.Signal = FDCAN1_RX
```

A CAN transceiver's RXD output idles **recessive = high**. Measured on the pad with
PB8 forced to input:

| Configuration | `GPIOB_IDR` bit 8 |
|---|---|
| Floating input | `1` |
| Input + internal pull-down (~40 kΩ) | `1` |

Still high against the internal pull-down, so it is being **actively driven**, not
floating. With factory option bytes (`nSWBOOT0 = 1`) BOOT0 comes from the pin, and
`BOOT0 = 1` + `nBOOT1 = 1` selects System memory.

### Diagnostic evidence

| Register | Value | Meaning |
|---|---|---|
| `pc` after reset | `0x1fff5048` | System bootloader, not the app |
| flash `@0x08000000` | `20020000 08001691` | App **is** correctly programmed (matches ELF) |
| `RCC_CSR` @ `0x40021094` | `0x14000000` | PINRSTF + SFTRSTF only — **no** IWDG / WWDG / BOR reset |
| `FLASH_OPTR` @ `0x40022020` | `0xFFEFF8AA` | `nSWBOOT0=1` (BOOT0 from pin), `nBOOT1=1`, RDP level 0 |
| `DBGMCU_IDCODE` @ `0xE0042000` | `0x20036469` | DEV_ID `0x469` = STM32G47x/G48x |

Note `RCC_CSR` ruling out watchdog and brownout resets early — that is what moves the
search off software and onto boot configuration.

### The fix

```
FLASH_OPTR:  0xFFEFF8AA  ->  0xF9EFF8AA
             bit 25  nSWBOOT0  1 -> 0    BOOT0 from option bit, ignore PB8 pin
             bit 26  nBOOT0    1 -> 0    that bit value = boot main flash
```

PB8 remains fully usable as FDCAN1_RX afterwards — only boot-mode selection ignores it.
No board rework required.

### Polarity gotcha

On STM32G4 the effective BOOT0 equals the `nBOOT0` bit **directly — it is NOT inverted**,
despite the `n` prefix.

The widely-copied STM32G0 recipe (`nBOOT_SEL=1, nBOOT0=1` -> main flash) **does not carry
over**. Setting `nSWBOOT0 = 0` while leaving `nBOOT0 = 1` still boots the bootloader.
Both bits must be cleared.

### Re-applying the fix on a fresh board

With OpenOCD (the `stm32l4x` driver covers G4):

```bash
openocd -f openocd.cfg \
  -c "init" -c "reset halt" \
  -c "stm32l4x option_write 0 0x20 0xF9EFF8AA 0x06000000" \
  -c "stm32l4x option_load 0" -c "exit"
```

`0x20` is the `FLASH_OPTR` offset; mask `0x06000000` covers bits 25 and 26.

Or with STM32CubeProgrammer:

```bash
STM32_Programmer_CLI -c port=SWD mode=UR -ob nSWBOOT0=0 nBOOT0=0
```

**Verify** by reconnecting and reading back — do not trust the write's own output:

```bash
openocd -f openocd.cfg -c "init" -c "reset halt" \
  -c "mdw 0x40022020 1" -c "reg pc" -c "reg sp" -c "exit"
```

Expected: `FLASH_OPTR = 0xf9eff8aa`, `pc = 0x08001690` (`Reset_Handler`),
`sp = 0x20020000` (`_estack`).

> `stm32l4x option_load` **always** reports `option load failed`. This is expected and
> not an error: `OBL_LAUNCH` resets the chip, which drops the SWD session. The write
> has already succeeded — reconnect and read `FLASH_OPTR` to confirm.

### Consequences

- **DFU / bootloader entry is now disabled.** BOOT0 comes from the option bit, so
  holding the pin does nothing. To use the ST bootloader, set `nSWBOOT0` back to `1`
  over SWD, then set it back to `0` when finished.
- **For a board respin:** move FDCAN1_RX to **PA11** (AF9) and leave PB8 unconnected.
  That removes the conflict at the schematic level and restores DFU.

---

## 2. Debug workflow notes

### OpenOCD

msys2 OpenOCD 0.12.0, scripts at `C:/msys64/mingw64/share/openocd/scripts`.
The repo's `openocd.cfg` sets `reset_config none` — **NRST is not wired to the probe**,
so all resets are `SYSRESETREQ` software resets.

Flash and run free:

```bash
ninja -C build/Debug
openocd -f openocd.cfg -c "init" -c "reset halt" \
  -c "program build/Debug/makolongfin2.elf verify" \
  -c "reset run" -c "exit"
```

### A halted core looks like a dead board

OpenOCD leaves the core **stopped** when it exits, and cortex-debug halts at `main` on
launch. Ending a session mid-halt freezes the LED, which reads as a regression but
isn't. End with `reset run` if you want the board to keep running standalone.

### ST-Link/V2 wedges periodically

Probe: VID:PID `0483:3748`, FW `V2J47S0`, WinUSB driver.

Failure mode — it still enumerates and `STM32_Programmer_CLI -l` reports the serial
number, but the firmware string comes back empty and every open fails:

```
openocd:  libusb_open() failed with LIBUSB_ERROR_ACCESS
CubeProg: ST-LINK error (DEV_CONNECT_ERR)
```

**Check for a stray OpenOCD first** — an orphaned one holding the probe produces
an identical error and is far more likely:

```powershell
Get-Process openocd -ErrorAction SilentlyContinue
```

If that is empty and the device is genuinely wedged, **a physical unplug/replug
clears it.** `Disable-PnpDevice` / `Enable-PnpDevice` requires an elevated shell
and otherwise fails with "Generic failure".

---

## 3. Verifying the blink without halting the core

Memory can be read over SWD while the target runs, so the application can be observed
without perturbing it. Useful symbol addresses in the current build:

| Symbol | Address |
|---|---|
| `SystemCoreClock` | `0x20000000` |
| `uwTick` | `0x20000028` |
| `GPIOB_ODR` (PB1 = LED) | `0x48000414` |

```tcl
# live.cfg — run with: openocd -f live.cfg
source [find interface/stlink.cfg]
transport select hla_swd
source [find target/stm32g4x.cfg]
reset_config none
init
for {set i 0} {$i < 12} {incr i} {
    set odr  [read_memory 0x48000414 32 1]
    set tick [read_memory 0x20000028 32 1]
    echo [format "t=%4d ms   PB1=%d   uwTick=%d" [expr {$i*250}] [expr {($odr>>1)&1}] $tick]
    sleep 250
}
exit
```

Healthy output — 500 ms on / 500 ms off, SysTick at 1 kHz:

```
t=   0 ms   PB1=1   uwTick=27177
t= 500 ms   PB1=0   uwTick=27696
t=1000 ms   PB1=1   uwTick=28224
t=1500 ms   PB1=0   uwTick=28756
```

`SystemCoreClock` should read `0x07A12000` = 128 MHz, matching `SystemClock_Config()`
in `Core/Src/main.c` (HSI16 -> PLLM=1, PLLN=16, PLLR=2).

---

## 4. Encoder — Allegro A1333 on SPI1

Ported from `makoshortfin`, which drives the same part on **SPI3** with the LL
drivers. This board uses **SPI1** and the project is HAL-based, so the transfer
goes through `HAL_SPI_TransmitReceive` instead. Driver: `Core/Src/encoder.c`.

### Wiring

| Signal | Pin | Mode |
|---|---|---|
| SPI1_SCK | PA5 | AF5 |
| SPI1_MISO | PB4 | AF5 |
| SPI1_MOSI | PB5 | AF5 |
| Encoder CS | **PA4** | GPIO output, manual (software NSS) |

> **Don't confuse this with SPI3.** This board *also* has the `makoshortfin` SPI3
> pinout wired up — PC10 (SCK) / PC11 (MISO) / PC12 (MOSI) with **CS on PA15**.
> That is a different bus. The encoder this driver talks to is on **SPI1, CS PA4**.
> `makoshortfin`'s `config.h` pin block describes the SPI3 bus, so copying it
> verbatim points you at the wrong chip select.

### Protocol

The A1333 is pipelined — the answer to a command arrives in the *next* frame:

1. CS low, transfer `0x3200` (ANG15 read), CS high
2. Wait >350 ns with CS idle
3. CS low, transfer `0x0000` (NOP), CS high
4. Angle = `rx2 & 0x7FFF`, a 15-bit count over one mechanical revolution

Frame 1's response is discarded (reads `0x0000` here).

### SPI1 settings that had to change

The CubeMX defaults were wrong for this part in four ways. Fixed in both
`Core/Src/spi.c` **and** `makolongfin2.ioc`, so regeneration keeps them:

| Field | CubeMX default | Required |
|---|---|---|
| `DataSize` | `SPI_DATASIZE_4BIT` | `SPI_DATASIZE_16BIT` |
| `CLKPolarity` | `SPI_POLARITY_LOW` | `SPI_POLARITY_HIGH` |
| `CLKPhase` | `SPI_PHASE_1EDGE` | `SPI_PHASE_2EDGE` |
| `NSSPMode` | `SPI_NSS_PULSE_ENABLE` | `SPI_NSS_PULSE_DISABLE` |
| `BaudRatePrescaler` | `DIV64` (2 MHz) | `DIV32` (4 MHz) |

CPOL=1 + CPHA=1 is **SPI mode 3**. NSS pulsing must be off because CS is driven
manually — leaving it on makes the peripheral toggle a hardware NSS that isn't
wired to anything, and it can glitch the frame timing.

### Reading the angle out

**Over SWD (works with a bare ST-Link/V2 — no extra hardware).**
`tools/watch_encoder.sh [samples] [interval_ms]` resolves the telemetry symbols
from the ELF at run time and polls them while the target runs free.

From **Git Bash** (or the Claude Code prompt, prefixing with `!`):

```bash
./tools/watch_encoder.sh            # 100 samples, 20 ms apart
./tools/watch_encoder.sh 300 10     # longer and faster
```

From **PowerShell** or the default VS Code terminal — it is a bash script, so
invoke it through Git Bash:

```powershell
& 'C:\Program Files\Git\bin\bash.exe' -lc "cd /c/Users/vinsh/Documents/makolongfin2 && ./tools/watch_encoder.sh 100 20"
```

> **Only one process can hold the ST-Link at a time.** If you get
> `LIBUSB_ERROR_ACCESS`, something else already has it — a running OpenOCD, or an
> active cortex-debug session in VS Code. Stop that first. See section 2 for the
> separate case where the probe itself has wedged.

Rebuild first if you have changed the firmware, or you will be reading stale
symbol addresses:

```bash
ninja -C build/Debug
```

**Live ASCII dial.** `tools/encoder_dial.sh [interval_ms]` draws the angle as a
needle on a circle, with live I_U / I_W and a scrolling current waveform under
it, and runs until Ctrl-C:

```
                        90
                   .O...........
              .....  =          .....
            ...      ==             ...
   ...
180  .                   +                   . 0
   ...
                        270

   angle  103.15 deg    raw  9389    rx2 0x24AD
   rate   15600 Hz    reads    833200    errors 0
   LED   ########......................  1173/4096
```

All rendering happens inside OpenOCD's Tcl — piping it through anything would
block-buffer and destroy the redraw. Note that this build of jimtcl has **no
math functions**, so the trig is an integer sine table with quadrant symmetry.

**Rotation convention:** 0° right, and increasing angle runs **clockwise** —
90° at the bottom, 180° left, 270° at the top. The maths convention (0 at east,
positive counter-clockwise) is the opposite, so the dial mirrors the angle
(`ddeg = 360 - adeg`) before drawing. Without that, turning the shaft clockwise
draws counter-clockwise. If it ever looks backwards again, that one line and the
90/270 label positions are what to flip.

> **Ctrl-C leaves nothing behind, but a `kill` of the wrapper used to.** The
> script now tracks its OpenOCD pid and kills it from the exit trap. Before that,
> anything which signalled only the wrapper (a `timeout`, for instance) orphaned
> OpenOCD, which kept holding the ST-Link — and that presents *exactly* like the
> wedged probe in section 2. If you hit `LIBUSB_ERROR_ACCESS`, check
> `Get-Process openocd` before reaching for the USB cable.

Healthy output — note the tiny jitter at rest, which is what distinguishes real
data from framing garbage:

```
   deg      raw     rx1     rx2      reads  err   Hz
  93.53     8514  0x0000  0x2142      45400    0  17084
  93.69     8528  0x0000  0x2150      45925    0  17084
  93.72     8531  0x0000  0x2153      47000    0  17084
  91.24     8305  0x0000  0x2071      50200    0  17084
  86.69     7890  0x0000  0x1ED5      50738    0  17084
```

### Sample rate

The `Hz` column is measured on-target: `main.c` latches
`reads` per one-second window into `g_enc.rate_hz`. Currently **~17 kHz**, which
is the SPI transaction plus HAL overhead and nothing else.

Getting there needed the main loop to stop blocking:

| | Before | After |
|---|---|---|
| Loop pacing | `HAL_Delay(100)` | none — samples flat out |
| UART | printed every sample | decimated, `TELEM_PRINT_EVERY` (200) |
| LED | toggled per iteration | rate-limited off `HAL_GetTick` |
| Sample rate | 10 Hz | ~17,000 Hz |

At 10 Hz a hand-turned shaft **aliases** — successive samples land more than
half a revolution apart and the angle appears to jump at random. If the readout
ever looks like noise while the shaft is moving smoothly, check the `Hz` column
before suspecting the SPI.

Two things keep the loop fast, and both matter:

- `Telem_Printf` **blocks** until the last character leaves the UART. At 115200
  a ~70-character line is ~6 ms, so printing every sample would cap the loop at
  roughly 150 Hz — a 100× penalty. Hence the decimation.
- `g_enc` is one contiguous block of `uint32_t`, so the reader fetches a whole
  sample in **one** SWD transaction instead of six.

**Over UART:** `main.c` also prints each sample on **USART1 / PB6 TX**, 115200 8N1.
Needs a USB-serial adapter on PB6 — the ST-Link/V2 (PID `3748`) has no virtual COM
port, which is why the SWD path above exists.

`Telem_Printf` is deliberately **integer-only**. newlib-nano omits float
formatting unless the link gets `-u _printf_float`, so `%f` would silently print
nothing; the angle is carried as hundredths of a degree in `g_enc_deg_x100`.

---

## 5. Debug LED brightness — PB1 via TIM3_CH4

> **This lives on the `hw-verification` branch, not `main`.** `led_pwm.c` and
> `tools/encoder_dial.sh` are bring-up instrumentation; `main` keeps PB1 free
> for the control firmware. The section is kept here so the findings (the AF
> mapping proof, the CIE curve) are not lost.


`Core/Src/led_pwm.c` drives the debug LED's brightness from the encoder angle:
**0° = off, 360° = full**, linear. PB1 is taken over from the plain push-pull
output that `MX_GPIO_Init` sets up, so `LedPwm_Init()` must run **after**
`MX_GPIO_Init()`.

| | |
|---|---|
| Pin | PB1, `GPIO_AF2_TIM3` |
| Timer | TIM3 channel 4, PWM mode 1, compare preload on |
| Clock | 128 MHz / (PSC 30 + 1) / (ARR 4095 + 1) = **1008 Hz** |
| Duty | `deg_x100 * 4096 / 36000`, 12-bit |

### Why direct register writes and not HAL_TIM

CubeMX never enabled TIM for this project, so **`stm32g4xx_hal_tim.c` was never
copied into `Drivers/`**. Every local copy on this machine came from a different
HAL release — `makoshortfin` is V1.2.5 against this project's V1.2.6, and the one
tree that *claims* V1.2.6 differs from this one by 1933 lines in `hal_spi.c`
alone, so its version string cannot be trusted.

Mixing HAL releases to get one PWM channel is the riskier trade. A PWM channel is
about ten register writes, so `led_pwm.c` does it directly and the project keeps
a single coherent HAL.

### Verifying the pin mapping empirically

There is no CubeMX pin database installed, so PB1's timer mapping was confirmed
on hardware rather than from a datasheet table. With the core halted (TIM3 keeps
running — `CNT` advances), force `CCR4` and sample `GPIOB_IDR` bit 1:

```
  CCR4=0     0%    ->  PB1 high in  0 / 40 samples
  CCR4=1024  25%   ->  PB1 high in  9 / 40 samples
  CCR4=2048  50%   ->  PB1 high in 20 / 40 samples
  CCR4=3072  75%   ->  PB1 high in 33 / 40 samples
  CCR4=4096  100%  ->  PB1 high in 40 / 40 samples
```

Duty maps straight onto measured pin state, which proves both the AF mapping and
that the output reaches the pad. This trick generalises: **any** pin's alternate
function can be confirmed this way without a scope.

### Perceptual (gamma) correction

PWM duty is proportional to **luminance**, but the eye responds to **lightness**,
so a linear duty ramp looks like it saturates almost immediately — most of the
visible change is crammed into the first sliver of travel.

`led_pwm.c` corrects this with a 65-entry CIE 1931 lightness table and linear
interpolation between entries. The angle is treated as the *desired perceived
brightness*, and the table returns the duty that actually looks that bright:

```
Y = ((L* + 16) / 116)^3     for L* > 8
Y = L* / 903.3              otherwise        where L* = 100 * angle/360
```

How uneven the table is *is* the correction — 7 duty counts per step at the
bottom versus 163 at the top:

```
   0,    7,   14,   21,   28,   35,   43,   51,     <- first eight entries
 ...
3469, 3619, 3774, 3933, 4096,                       <- last five
```

Measured against the formula on hardware:

| angle | duty (corrected) | duty (old linear) |
|---|---|---|
| 82.66° | **155** | 940 |
| 341.96° | **3587** | 3889 |

At 82.66° — 23% of a revolution — the LED now sits at 3.8% duty instead of 23%.
Integer maths throughout, no FPU dependency, so this stays safe to call from an
ISR later.

`LedPwm_PerceivedToDuty(perceived, scale)` is exposed separately if you want the
same curve for another indicator.

### Heartbeat LED — PB2

PB1 belongs to TIM3 now, so the plain "still alive" blink lives on **PB2**
(already a GPIO output from `MX_GPIO_Init`), toggling every 1 s off
`HAL_GetTick`. At ~15 kHz loop rate, toggling per iteration would be a blur
rather than a blink.

Verified by sampling `GPIOB_IDR` bit 2 — one flip per ~1 s as intended, giving a
2 s on/off cycle. Change `HEARTBEAT_MS` in `main.c` to retune it.

> Section 3's blink check no longer applies to PB1: that pin is PWM-driven, so
> `GPIOB_ODR` bit 1 stays 0. Use bit 2 for the heartbeat, or watch PB1 brightness
> track the shaft.

---

## 6. Phase current sensing — CT4022-A40BSN8

Sensors: CT4022-A40BSN8 TMR, **40 A bidirectional**, ratiometric on 3V3,
**33 mV/A**. Zero current sits at VDDA/2 (~1650 mV); full scale is
1650 ± 1320 mV, so the sensor deliberately does not use the whole ADC range.

### The signal path is not what it looks like

Neither sensor reaches an ADC pin directly. Both go through an **internal OPAMP
follower**, per the `.ioc`:

| Phase | Pin | OPAMP | ADC channel | ADC |
|---|---|---|---|---|
| U | PC3 | OPAMP5 (`Follower_Internally_Connected`) | `ADC_CHANNEL_VOPAMP5` | **ADC5** |
| W | PA1 | OPAMP3 (`Follower_Internally_Connected`) | `ADC_CHANNEL_VOPAMP3_ADC2` | **ADC2** |

Two consequences that cost time if you miss them:

- **OPAMP5's output is reachable only from ADC5.** `IS_ADC_CHANNEL` in
  `stm32g4xx_hal_adc_ex.h` gates `ADC_CHANNEL_VOPAMP5` to `ADC5` alone. CubeMX
  configured only ADC1 and ADC2, so `csense.c` brings ADC5 up itself, including
  `RCC_PERIPHCLK_ADC345` and `__HAL_RCC_ADC345_CLK_ENABLE()`.
- **CubeMX generates the OPAMP init but never starts them.** There is no
  `HAL_OPAMP_Start` anywhere in the generated `opamp.c`. Without it the ADC
  reads garbage. `CSense_Init()` starts OPAMP3 and OPAMP5 explicitly.
- **OPAMP3 shipped as `NORMALSPEED` while every other OPAMP was `HIGHSPEED`.**
  Mismatched bandwidth between the U and W measurements would appear as a fake
  current imbalance once PWM is switching. Fixed in both `Core/Src/opamp.c` and
  the `.ioc` (`OPAMP3.PowerMode=OPAMP_POWERMODE_HIGHSPEED`), so a CubeMX
  regeneration keeps it. Both use `OPAMP_TRIMMING_FACTORY`, which holds separate
  trim values per power mode, so no recalibration was needed — and the measured
  zero codes were unchanged (2037 / 2039) before and after.

ADC2's generated channel was a placeholder (`ADC_CHANNEL_3`); `csense.c`
retargets rank 1 to `ADC_CHANNEL_VOPAMP3_ADC2`.

### Conversion

```
mV  = raw * 3300 / 4096
mA  = (raw - zero) * 24414 / 1000      (24.414 mA per LSB)
```

24.414 mA/LSB at 12 bits gives ±50 A of theoretical span for a ±40 A sensor.
Integer maths throughout. `CSense_Init()` self-calibrates both ADCs
(`HAL_ADCEx_Calibration_Start`) and then averages 256 samples to capture the
zero-current offset — **the motor must be de-energised and at rest when it runs**,
which it is, since nothing drives the gate outputs yet.

### Verified at zero

```
VREFINT raw = 1520   -> measured VDDA = 3286 mV
ideal mid-scale code = 2048  ->  1643 mV

   U_raw  U_mV  U_mA  |  W_raw  W_mV  W_mA  | Uzero Wzero  err
   2037  1631     0  |  2040  1634    24  |  2037  2039    0
   2038  1632    24  |  2039  1633     0  |  2037  2039    0
   2039  1633    48  |  2040  1634    24  |  2037  2039    0
```

(Measured with both OPAMPs at `HIGHSPEED`. VDDA reads 3281–3286 mV across runs;
that 5 mV spread is VREFINT quantisation, roughly 2 mV per LSB, not rail drift.)

**The rail is not 3.300 V.** Measured via VREFINT against its factory
calibration, VDDA is **3286 mV**, so the true half-rail is **1643 mV**, not 1650.
Quoting a 1650 mV target is measuring against the wrong number.

- Both sensors sit at **~1634 mV**, about **9 mV (11 codes, 0.55%) below
  half-rail**. That is genuine sensor + ADC offset, and it is absorbed by the
  startup zero calibration (`u_zero` / `w_zero`), so it does not reach the
  current reading.
- Noise is **±1 LSB (±24 mA)**, i.e. quantisation-limited. On a 40 A sensor that
  is 0.06%.
- U and W differ by 2 LSB — ordinary device-to-device offset.
- `errors = 0`, `init_rc = 0`.

### Rail voltage cancels out of the current scaling

Worth knowing before anyone "corrects" the 3300 in the maths: the sensor is
**ratiometric on the same rail that feeds VREF+**, so VDDA cancels exactly.

```
sensitivity   = 33 mV/A * (VDDA / 3300)
ADC LSB       = VDDA / 4096
A per LSB     = (VDDA/4096) / (33 * VDDA/3300) = 3300 / (33 * 4096) = 24.414 mA
```

So **24.414 mA/LSB is exact regardless of the actual rail voltage**. The
measured VDDA only affects the reported *voltage* (`u_mv` / `w_mv`), never the
current. `CS_UA_PER_LSB` should stay at 24414 even though the rail is 3286 mV.

### Before closing the loop

- ~~OPAMP power-mode mismatch~~ — fixed, see above. Both are `HIGHSPEED`.
- ADC conversion is **software-triggered polling** here. For FOC it has to be
  triggered from the PWM timer so sampling lands in the correct part of the
  switching period.
- Consider ADC oversampling: 24 mA/LSB is coarse for low-current control.

---

## 7. Three-phase PWM — HRTIM1, and the UCC21330 gate drivers

Gate drivers: **UCC21330BQDRQ1**, one per phase. Dual-channel isolated, 4 A
source / 6 A sink, 3 kVRMS.

| Phase | MCU pin | HRTIM output | Timer | Compare unit |
|---|---|---|---|---|
| U | PA11 | `HRTIM1_CHB2` | B | CMP2 |
| V | PA10 | `HRTIM1_CHB1` | B | CMP1 |
| W | PA9 | `HRTIM1_CHA2` | A | CMP1 |

One MCU signal per phase. An **external inverter on the board** derives the
complementary INB for each driver, and the driver's **20 kΩ RDT** sets the dead
time, so the MCU emits three plain single-ended PWMs — no complementary pairs,
no HRTIM dead-time insertion.

```
t_DT = 8.6 * R_DT(kohm) + 13 ns = 8.6*20 + 13 = 185 ns
```

U and V share Timer B (CMP1/CMP2 give two independent duties off one counter),
W is on Timer A. Both counters are started in a single register write so they
stay locked. Measured: the A-to-B offset is a constant 112 counts, which is the
CPU time between the two register reads, not drift.

### ⚠️ There is no all-off state from the MCU

This is the single most important consequence of the inverter topology. The
driver's logic table (with a DT resistor fitted) is:

| INA | INB | OUTA | OUTB |
|---|---|---|---|
| L | L | L | L |
| L | H | L | H |
| H | L | H | L |
| **H** | **H** | **L** | **L** |

Because an inverter guarantees INA and INB are always opposite, the `L L` row —
the only both-off state — **is unreachable**. A low MCU pin turns the low-side
device on; a high pin turns the high-side device on. One FET per phase is
always conducting whenever the drivers are powered.

Consequences:

- At 0% duty all three low-side FETs conduct. That is the legitimate SVPWM zero
  vector, not a fault — but it **shorts the motor windings together**, so
  spinning the rotor by hand produces braking torque.
- **The DIS pin is the only true all-off.** Asserting DIS high shuts both
  outputs down in ~48 ns. Treat it as the emergency stop; it is not optional on
  this board the way it would be with a dual-input topology.
- Disabling the HRTIM outputs does **not** de-energise anything. The pins simply
  sit low, which is the zero vector, not off.

### Verified with outputs disabled

```
pwm_init_rc = 0
period = 51200 counts   -> 20000 Hz
cmp  U=3  V=3  W=3
outputs_en = 0   HRTIM OENR = 0x0000
   cnt_a=  4288  cnt_b=  4400        <- advancing, locked
   PA9(W)=0  PA10(V)=0  PA11(U)=0
```

HRTIM kernel clock is the APB2 timer clock (128 MHz, APB2 prescaler 1);
`PRESCALERRATIO_MUL8` gives a 1.024 GHz counter, so 51200 counts is exactly
20 kHz with ~0.98 ns duty resolution. Compare registers have a hardware minimum
of 3, which at 2.9 ns is below the driver's 5 ns input deglitch filter and so
reads as a true 0%.

`MotorPwm_EnableOutputs()` is the only thing that connects HRTIM to the pins,
and nothing calls it at boot.

### ⚠️ HRTIM ignores compares below one fHRTIM period — 0% became 100%

HRTIM will not act on a compare value smaller than one full `fHRTIM` clock
period, which at `PRESCALERRATIO_MUL8` is **8 counter LSBs**. A compare below
that is silently ignored: the output is still set at the period rollover and
then **never reset**, so it latches high for the entire period.

An early version used `PWM_CMP_MIN = 3` for "0%". The result was that a
commanded **0% duty came out as a stuck 100%** — full high-side on, the worst
possible failure direction for a bridge. It was invisible on the U phase
(running at 25%, well above the threshold) and only appeared once V and W were
commanded to zero.

Measured on this part, sweeping Timer B CMP1R and sampling PA10:

| CMP | PA10 high | |
|---|---|---|
| 3 | 100% | ignored -> stuck on |
| 6 | 100% | ignored -> stuck on |
| **7** | **100%** | last failing value |
| **8** | **0%** | first working value |
| 12…512 | tracks duty | correct |

Two fixes, both in `motor_pwm.c`:

- `PWM_CMP_MIN` is **16** (2x the measured minimum, for margin).
- **True 0% never uses a small compare.** `MotorPwm_SetDuty` clears the output's
  set-source (`SETx1R`/`SETx2R` = 0) so the output can never be driven high, and
  leaves a valid compare in place so any currently-high output is reset once and
  stays low. A non-zero duty puts `HRTIM_OUTPUTSET_TIMPER` back.

Verified after the fix — 0% is genuinely 0%, and duty is linear:

```
   cmd%   PA11(U)   PA10(V)   PA9(W)
      0       0%        0%        0%
    250      24%        0%        0%
    750      75%        0%        0%
   1000     100%        0%        0%
```

The general lesson: **on a motor bridge, always confirm that a commanded zero
actually produces zero at the pin.** A duty that looks right in the middle of
the range says nothing about the endpoints.
### Gate driver enable — PC5

`PC5` drives the DIS/enable net common to all three drivers. It is configured as
a push-pull output and **driven to the disabled state before the pin is switched
to an output**, so bringing the pin up cannot emit even a momentary enable pulse.
`MotorPwm_GateInit()` is the first thing `MotorPwm_Init()` does.

> **⚠️ PC5 is an active-LOW enable — `GATE_EN_ACTIVE_HIGH` is 0.**
> PC5 is tied **directly to the drivers' DIS pins**; there is no inverter in
> this path, unlike the PWM path which does have one. DIS is active-HIGH
> *disable*, so:
>
> ```
> PC5 HIGH -> DIS high -> outputs disabled   <- safe state, and the boot state
> PC5 LOW  -> DIS low  -> outputs ENABLED
> ```
>
> This bit an earlier version of this firmware. The schematic net is named like
> an enable, so `GATE_EN_ACTIVE_HIGH` was initially 1 and the board booted with
> all three drivers **live** while the code believed they were off. The net name
> is the misleading part — do not "tidy" this back to 1. Verified after the fix:
> `PC5 = 1` at boot with `OENR = 0x0000`.

`MotorPwm_SafeShutdown()` drops the gate line first, then the HRTIM outputs —
that order matters, because only the gate line actually turns FETs off.

### HRTIM-triggered ADC sampling

Timer A compare unit 3 drives `HRTIM_ADCTRIGGER_1`; ADC2 (W) and ADC5 (U) both
select `ADC_EXTERNALTRIG_HRTIM_TRG1`, so the two phases are sampled at the same
instant every PWM period. Default position is 90% of the period
(`PWM_ADC_TRIG_PERMILLE`), late enough that the duty edges have settled — tune
it on a scope. `MotorPwm_SetAdcTriggerPoint()` moves it at run time.

The counters run whether or not the outputs are enabled, so the whole trigger
path is verifiable with the power stage inert:

```
cs_trig_rc = 0   gate_en = 0   OENR = 0x0000   adc_trig_pos = 46080

   U_raw  U_mV  U_mA  |  W_raw  W_mV  W_mA  | samples  err
    2039  1634    24  |   2041  1636    24  |     117    0
```

`errors` staying at 0 while `samples` climbs is the proof the trigger fires — a
dead trigger shows up as EOC timeouts, not as bad data.

Note the readings are now **perfectly steady** rather than dithering by an LSB:
sampling is locked to a fixed point in the period instead of landing wherever
the polling loop happened to be.

**The residual ±1 LSB (±24 mA) is quantisation, not miscalibration.** The true
zero sits at a fractional code (~2038.5) while `u_zero` / `w_zero` are integers,
so re-running the zero capture just moves the residual between −24 and +24 mA.
Storing the zero as sub-LSB fixed point would remove it if that 24 mA ever
matters for torque ripple.

### Driver bring-up checklist

- **VDDA−VSSA and VDDB−VSSB ≥ 9.2 V.** The **B** suffix is the **8 V UVLO**
  option (A = 5 V, C = 12 V). Below that the outputs never turn on and
  everything else looks healthy.
- **VCCI 3.0–5.5 V** — the 3V3 rail is fine.
- **DIS is pulled HIGH internally = outputs disabled.** It must be pulled or
  driven low to operate. Route it to an MCU pin if it is not already.
- **DT resistor must go to GND.** Floating or tied to VCCI disables the
  interlock and permits overlap.
- INA/INB have 90 kΩ internal pull-downs, so a Hi-Z MCU pin reads low.

---

## 8. Motor parameters and encoder zero calibration

Parameters carried over from `makoshortfin/Core/Inc/config.h`:

```
MOTOR_POLE_PAIRS            20
MOTOR_PHASE_RESISTANCE_OHM  0.085      (0.17 ohm phase-to-phase / 2)
MOTOR_PHASE_INDUCTANCE_H    54.3 uH
MOTOR_RATED_CURRENT_A       100
```

**Both key numbers were independently confirmed on this board**, which is worth
recording because each was measured before the config file was consulted:

- **Phase resistance.** Current scales linearly at ~73 mA per 0.1% modulation,
  implying ~0.13 ohm — the same order as the documented 85 mohm, measured purely
  from the current-vs-modulation sweep.
- **Pole pairs.** Stepping a DC vector through 720 degrees electrical moved the
  rotor 35.76 degrees mechanical, giving **20.13 pole pairs**. Expected 36.00
  for exactly 20, so 0.67% error — inside cogging hysteresis.

Do **not** measure pole pairs from a spinning open-loop rotor. That gave ~32,
because with no current control the rotor slips rather than locking to the
field. Stepped DC alignments are static, so slip cannot corrupt them.

### Encoder zero, programmed into the sensor

The A1333 applies `Angle_out = Angle_RAW - ZERO_OFFSET` internally, so the
electrical-angle offset lives in the sensor rather than in firmware:

```
electrical_angle = (encoder_angle * 20) mod 360
```

No software offset constant, no per-boot calibration, and it survives power
cycles.

| | |
|---|---|
| Register | `ANG`, bits [11:0] |
| EEPROM | `0x1C` — permanent, **~100 write cycles**, ~24 ms per write |
| Shadow | `0x5C` — volatile, immediate, unlimited |
| Unlock | KeyCode `0x0027811F77`, five separate byte writes to `0x3C` |
| Programmed value | **3032** |

**Always trial in shadow first.** The write budget is small and does not renew.

Scaling was measured, not assumed: writing 1024 shifted the reported angle
90.01 degrees and 2048 shifted it 180.05, so `ZERO_OFFSET` is 12 bits across
360 degrees and `offset = raw15 >> 3`. Granularity is 0.088 degrees.

Two independent checks that the calibration is good:

- After programming, the rotor held at electrical zero read **0.00 +/- 0.07
  degrees**, i.e. within one offset LSB.
- Electrical zeros must land on multiples of 18 degrees mechanical for 20 pole
  pairs. A later alignment settled at **305.87** against a predicted
  **306.00** — 0.13 degrees out.

> **`Encoder_ZeroHere` must not be run twice.** It reads the *current* angle,
> which is already offset-corrected, so a second call double-applies. To change
> a programmed offset, either zero the offset first and re-align, or write a
> known value directly (`enc_cmd 5`).

---

## 9. Debugging heuristic worth remembering

**If a Cortex-M appears to restart rather than hang, suspect boot configuration before
application code.**

Every fault handler in `Core/Src/stm32g4xx_it.c` is a `while(1)` trap. A HardFault,
BusFault or UsageFault therefore *parks* there — it cannot bounce you to the reset
vector. So landing on `Reset_Handler` means a genuine reset (or a boot into other
memory), never a crash. Check in this order:

1. `RCC_CSR` @ `0x40021094` — reset cause flags (IWDG / WWDG / BOR / PIN / SFT)
2. `FLASH_OPTR` @ `0x40022020` — option bytes and boot selection
3. `pc` after `reset halt` — is it even inside `0x08000000`?

Only then start reading C.
