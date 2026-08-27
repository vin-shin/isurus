/**
  ******************************************************************************
  * @file    board.h
  * @brief   GR MotherFOCer - the hardware map, in one place.
  *
  *          Isurus moves between boards on a branch per board. This file is
  *          the seam: everything that is true of THIS board and not of the
  *          stack lives here, so a port is a diff against one header rather
  *          than an archaeology dig through twenty files.
  *
  *          Every value here was derived from the CubeMX project that came
  *          with the board (Inverter.ioc and the LL code it generated in the
  *          gr_motherfocer tree), not from a schematic. That distinction
  *          matters: the .ioc records which peripheral is on which pin, and
  *          it does NOT record what the net on the other side of the pin is
  *          connected to. Where those are different questions, the text below
  *          says which one it is answering.
  *
  *          Anything marked BOARD_UNKNOWN is something the .ioc configured but
  *          did not explain. Those are listed rather than omitted, because a
  *          pin that is configured and unexplained is exactly the thing that
  *          turns into a bring-up mystery.
  *
  * ---------------------------------------------------------------------------
  * What is different from Mako Longfin - the short list that matters
  * ---------------------------------------------------------------------------
  *   1. The bridge is COMPLEMENTARY. Three HRTIM output pairs with hardware
  *      dead-time insertion, where Mako Longfin had one pin per phase and an
  *      external inverter. The safety note in motor_pwm.h that said "there is
  *      NO MCU state that turns every FET off" does NOT apply to this board -
  *      disabling the HRTIM outputs really does turn everything off here.
  *   2. The gate enable is ACTIVE HIGH, where Mako Longfin's was active low.
  *      Getting this backwards energises the bridge at reset.
  *   3. Three phase currents are measured, not two, and they arrive by DMA
  *      from one ADC sequence rather than through internal OPAMP followers.
  *   4. The core runs at 160 MHz, not 128 MHz, so every count-based timing
  *      constant derived from the HRTIM clock moves with it.
  *   5. The encoder is a 13-bit RM44SI on SPI3, not a 15-bit A1333 on SPI1.
  *   6. CAN is on FDCAN2, not FDCAN1 - so the PB8 BOOT0 collision documented
  *      in HARDWARE_NOTES for Mako Longfin does not exist on this board.
  ******************************************************************************
  */
#ifndef BOARD_H
#define BOARD_H

#ifdef __cplusplus
extern "C" {
#endif

#define BOARD_NAME              "GR MotherFOCer"

/* ==========================================================================
 * 1. Core and clock
 * ==========================================================================
 * STM32G474RET6, LQFP64, 512 KB flash / 128 KB RAM.
 *
 * HSI16 -> PLLM /1 -> PLLN x20 -> PLLR /2 = 160 MHz, AHB /1, APB1 /1, APB2 /1,
 * flash latency 4, Range 1 boost. Mako Longfin ran the same part at 128 MHz
 * (PLLN 16); this board's CubeMX project asks for 160, and the port keeps it,
 * because the HRTIM and ADC timings below were all computed from it.
 *
 * The part is rated to 170 MHz, so 160 is not near an edge.
 */
#define BOARD_SYSCLK_HZ         160000000UL
#define BOARD_PLLN              20U

/* ==========================================================================
 * 2. Power stage - HRTIM1
 * ==========================================================================
 * Three complementary pairs, one timer per phase:
 *
 *   U : PA10 HRTIM1_CHB1 (high) / PA11 HRTIM1_CHB2 (low)   Timer B
 *   V : PC6  HRTIM1_CHF1 (high) / PC7  HRTIM1_CHF2 (low)   Timer F
 *   W : PB12 HRTIM1_CHC1 (high) / PB13 HRTIM1_CHC2 (low)   Timer C
 *
 * Each timer is independent but all three share the prescaler and period, and
 * the master timer runs alongside them at the same period. Only output 1 of
 * each pair has a set/reset source; output 2 is produced entirely by the
 * dead-time unit, which is what makes the pair genuinely complementary rather
 * than two waveforms that happen to be inverses of each other.
 *
 * PRESCALERRATIO_MUL8 at 160 MHz gives a 1.28 GHz counter. A period of 64000
 * counts is therefore 20 kHz, and one duty LSB is about 0.78 ns.
 *
 * ON THE SWITCHING FREQUENCY. Mako Longfin deliberately moved from 20 kHz to
 * 30 kHz to get out of the audible band. This board's CubeMX project is at
 * 20 kHz, and the port keeps 20 kHz, because the dead time, the ADC sampling
 * times and whatever filtering is on the current sense were all chosen around
 * it on this hardware and none of them have been measured here. It is a
 * starting point that matches the board as configured, not a finding that
 * 20 kHz is right. Raising it is a bench change, not an edit.
 */
#define BOARD_HRTIM_MUL         8U
#define BOARD_PWM_FREQ_HZ       20000U

/* HRTIM counter rate in Hz: 160 MHz x 8 = 1.28 GHz. */
#define BOARD_HRTIM_TICK_HZ     (BOARD_SYSCLK_HZ * BOARD_HRTIM_MUL)

/* Period in counts: 1.28 GHz / 20 kHz = 64000. The .ioc writes 63999, since
 * the register holds period - 1. */
#define BOARD_PWM_PERIOD        (BOARD_HRTIM_TICK_HZ / BOARD_PWM_FREQ_HZ)

/* Dead time, from the .ioc: rising 160, falling 160, DT prescaler DIV1, both
 * signs positive.
 *
 * At DIV1 the LL header states fDTG = fHRTIM, which is the 160 MHz kernel
 * clock and NOT the x8 multiplied counter clock, so one dead-time count is
 * 6.25 ns and 160 of them is 1.0 us.
 *
 * !! THIS IS THE ONE INHERITED CONSTANT THAT A FET CHANGE FALSIFIES. !!
 *
 * The rest of this board is reported to match the gr_motherfocer hardware
 * exactly EXCEPT for the FETs - which makes almost everything in that project
 * authoritative here, and makes this number the exception rather than the
 * rule. Dead time exists to cover the turn-off delay of the device plus the
 * propagation mismatch between the two gate driver channels. Turn-off delay
 * is a property of the FET: its gate charge, the gate resistor, the driver's
 * sink current. Change the FET and the required dead time changes with it.
 *
 * 1.0 us is also long for a MOSFET bridge - typical silicon parts want
 * 100-500 ns - which cuts both ways. Too long is not dangerous, it is lost
 * modulation range and distortion near the zero crossing. Too SHORT is a
 * shoot-through, and it is not a fault the firmware can detect or fault on:
 * both devices in a leg conduct for a few tens of nanoseconds and the damage
 * is thermal and cumulative.
 *
 * So this stays at the inherited 160 counts, because erring long is the safe
 * direction, and it must be re-derived from the new FETs' datasheet and then
 * confirmed on a scope at a switch node before the bridge runs at any real
 * bus voltage. Do not shorten it to recover modulation range on the strength
 * of a datasheet alone.
 */
#define BOARD_DT_RISING         160U
#define BOARD_DT_FALLING        160U

/* ==========================================================================
 * 3. Gate driver enable - PC8
 * ==========================================================================
 * !! SAFETY !! ACTIVE HIGH. This is the opposite of Mako Longfin.
 *
 * The evidence, all from the board's own generated and hand-written code:
 *   - MX_GPIO_Init drives PC8 LOW before configuring it as an output, i.e.
 *     the chosen reset state is LOW.
 *   - disableGateDriver() resets PC8.
 *   - resetGateDriver() pulses PC8 low, waits ~1 ms, then SETS it.
 *
 * All three agree that LOW is off and HIGH is on. Mako Longfin's PC5 drove a
 * UCC21330 DIS pin through no inverter and was therefore active low. If this
 * gets "tidied" to match the old board, the bridge comes up live.
 */
#define BOARD_GATE_EN_PORT      GPIOC
#define BOARD_GATE_EN_PIN       GPIO_PIN_8
#define BOARD_GATE_EN_ACTIVE_HIGH 1

/* The board's bring-up code holds the driver in reset for ~1 ms before
 * releasing it. Named rather than left as a literal so the requirement is
 * visible - but the gate driver part number is not in the .ioc, so the true
 * minimum has not been checked against any datasheet. */
#define BOARD_GATE_RESET_US     1000U

/* ==========================================================================
 * 4. Analogue
 * ==========================================================================
 * Three ADCs, each free-running into its own DMA buffer. A different shape
 * from Mako Longfin, which polled two OPAMP-fed channels on an HRTIM trigger.
 * Here the phase currents come straight off pins.
 *
 * ADC1, DMA1 channel 1, 5 conversions, 12.5 cycle sampling:
 *   rank 1  PC3  ADC1_IN9   phase W current
 *   rank 2  PA0  ADC1_IN1   phase V current
 *   rank 3  PA1  ADC1_IN2   phase U current
 *   rank 4  PA2  ADC1_IN3   DC link current
 *   rank 5  PA3  ADC1_IN4   DC bus voltage
 *
 * ADC2, DMA2 channel 1, 4 conversions, 92.5 cycle sampling:
 *   rank 1  PC4  ADC2_IN5    temperature
 *   rank 2  PA5  ADC2_IN13   temperature
 *   rank 3  PA6  ADC2_IN3    temperature
 *   rank 4  PA7  ADC2_IN4    temperature
 *
 *   The board's code labels these as a group - "motor & MOSFET temperature
 *   sense" - but does not say which pin is which, and there are four of them.
 *   Which is which is BOARD_UNKNOWN.
 *
 * ADC3, DMA1 channel 2, 1 conversion, 47.5 cycle sampling:
 *   rank 1  PB1  ADC3_IN1   labelled "audio" in the board's code
 *
 * !! The phase-current conversions are NOT synchronised to the PWM period on
 * this board as configured. ADC1 is started once and free-runs; the main loop
 * reads whatever DMA last wrote. That is fine for the bring-up print loop it
 * was written for, and it is not good enough for a current loop, which needs
 * all three phases sampled at the same known point in the switching period.
 * Retargeting ADC1 to the HRTIM trigger is part of the port, not a later
 * refinement - see docs/PORT-POWER-UNIT.md.
 */
#define BOARD_ADC1_RANKS        5U
#define BOARD_ADC2_RANKS        4U
#define BOARD_ADC3_RANKS        1U

/* Indices into the concatenated DMA buffer the board's code fills. */
#define BOARD_ADC_IDX_IW        0U
#define BOARD_ADC_IDX_IV        1U
#define BOARD_ADC_IDX_IU        2U
#define BOARD_ADC_IDX_IDC       3U
#define BOARD_ADC_IDX_VBUS      4U
#define BOARD_ADC_IDX_TEMP0     5U
#define BOARD_ADC_IDX_AUDIO     9U

/* ---- DC bus divider: 400:1, from the schematic ----------------------------
 *
 *     4M7 + 4M7 + 560k + 15k  =  9.975 MOhm   over   25 kOhm
 *     9.975M + 25k = 10.000 MOhm exactly, so the ratio is exactly 400:1
 *
 * The four-resistor top leg is not redundancy, it is HV practice: at 588 V
 * each part drops under 300 V and the creepage is spread over four
 * footprints. The exact 10 MOhm total says the ratio was chosen rather than
 * fallen into.
 *
 * !! THE INHERITED SCALE FACTOR WAS WRONG BY ABOUT 6x, IN THE DANGEROUS
 * DIRECTION. !! The board's bring-up loop used `vbus = raw * 0.05`, which is
 * 204.8 V of full scale and implies a 62:1 divider. The divider is 400:1. At
 * VREF 3.3 V a full 588 V pack presents 1.47 V to the pin, converts to 1825
 * counts, and that formula reports it as 91 V.
 *
 * Reading 91 V for a 588 V bus is not a cosmetic error. FOC_SetGainsForVbus
 * DIVIDES by the bus, so every current-loop gain would have come out 6.4x too
 * large, on a machine whose coupling terms are already three quarters of the
 * supply. The undervoltage trip would also have fired permanently, which is
 * the one piece of luck in it: the drive would have refused to arm rather
 * than armed badly.
 *
 * So the earlier note in this file that the bus sense "cannot read the pack"
 * was wrong, and wrong precisely because it trusted that 0.05. The HARDWARE
 * is fine and has better than 2x headroom - only the firmware constant was
 * broken.
 */
/* WHERE THE WRONG CONSTANT CAME FROM, since it explains how much of the rest
 * of that project to trust.
 *
 * The sibling project MiniFOCer, same author and same file layout, scales its
 * phase currents by 0.040584415584415584 - a precisely derived number - and
 * its bus by 0.008, which is 32.8 V of full scale for a 4S bench board.
 * gr_motherfocer has 0.04 and 0.05: a ROUNDED COPY of MiniFOCer's current
 * constant, and a bus constant that matches neither MiniFOCer's divider nor
 * this board's.
 *
 * So neither figure was ever derived for this hardware. That is consistent
 * with the state of the project - four commits ending at "encoder works" - and
 * it means the analogue scaling is the part of gr_motherfocer NOT to inherit,
 * while its pinout, peripheral mapping and interrupt layout are exactly the
 * parts that are trustworthy.
 *
 * One more thing worth carrying across from MiniFOCer: it negates the W phase
 * current (`* -0.040584...`) and not U. Phase sense POLARITY is per-board
 * wiring, it is invisible in a pinout, and getting it wrong does not fail
 * loudly - it corrupts the Clarke transform into a rotating error. Check the
 * sign of each phase against a known current before closing the loop.
 * gr_motherfocer applies no inversion, which is evidence but not proof. */
#define BOARD_VBUS_DIV_NUM      400U      /* (9.975M + 25k) / 25k */
#define BOARD_VBUS_DIV_DEN      1U

/* ---- VREF+ : measured, not assumed ----------------------------------------
 *
 * The board disables the internal VREFBUF and puts VREF+ in external mode
 * (HAL_SYSCFG_DisableVREFBUF, in its stm32g4xx_hal_msp.c), so the reference
 * comes from a part this project has never seen. Every candidate gives a
 * workable full scale and a different volts-per-count:
 *
 *     VREF     V/LSB     full scale     588 V uses
 *     3.300    0.3223      1320 V          45%
 *     3.000    0.2930      1200 V          49%
 *     2.500    0.2441      1000 V          59%
 *     2.048    0.2000       819 V          72%
 *
 * Guessing among those is a 60% error on the bus reading, the same class of
 * mistake as the 0.05 above. It does not have to be guessed: VREFINT is an
 * on-die bandgap with a factory calibration constant, so the firmware can
 * measure VREF+ at startup and derive volts-per-count from it. csense.c
 * already does exactly that on the previous board - CSense_MeasureVdda.
 *
 * The value below is a fallback for the window before that measurement runs.
 * Nothing that matters should be using it. */
#define BOARD_VREF_NOMINAL_MV   3300U

/* ---- current sensor: Mornsun TL200-A2PV -----------------------------------
 *
 * A Hall-effect current transducer with an analogue voltage output.
 *
 * !! THE NUMBERS BELOW ARE NOT FROM THE DATASHEET AND MUST BE. !! They are
 * placeholders with the right SHAPE so that the scaling code is correct and
 * only the constants move. Do not energise the bridge against them.
 *
 * THE SENSOR DOES NOT DRIVE THE PIN DIRECTLY. There is a signal conditioning
 * stage between the two which maps the sensor onto a 0..3V3 range and is
 * referenced to a VREF of its own. That changes what the constant below
 * means, and mostly for the better:
 *
 *   - BOARD_I_SENS_UV_PER_A is the sensitivity AT THE ADC PIN, after
 *     conditioning gain. It is NOT the sensor's own mV/A, and reading one off
 *     the TL200 datasheet and dropping it in here would be wrong by whatever
 *     that gain is.
 *   - "conditioned to a 3V3 range" is what makes the earlier rail-to-rail
 *     guess reasonable rather than optimistic. The conditioning exists
 *     precisely to fill the converter's span, so the swing should be close to
 *     full scale.
 *   - a zero at mid-scale is now a design intent rather than an accident,
 *     since the stage sets its own zero. The self-test's mid-scale check
 *     therefore becomes a real check on the CONDITIONING, not just on the
 *     sensor.
 *   - if that stage's reference is the same one feeding VREF+, gain and
 *     reference track together and the reading is ratiometric end to end. If
 *     it is a separate reference, they do not, and the measured VREF+ has to
 *     divide in - which is what csense.c does.
 *
 * So the number to obtain is the amps-per-volt (or volts-per-amp) of the
 * WHOLE chain, sensor plus conditioning, measured at the pin. One known
 * current through the sensor and a voltmeter on the ADC input settles it in
 * a minute and is worth more than either datasheet.
 *
 * What the datasheet has to settle, in order of how badly each one bites:
 *
 *   1. WHERE THE CONDITIONED ZERO SITS. Expected at mid-scale, since the
 *      stage is built to fill 0..3V3, and Drive_SelfTest enforces that: a
 *      captured zero further than DRIVE_CS_ZERO_TOL_CODES from 2048 raises
 *      DRIVE_FAULT_CSENSE. That check is now doing useful work rather than
 *      merely being satisfied - if the conditioning is not what is assumed
 *      here, it says so at boot instead of at full current.
 *
 *   2. SENSITIVITY OF THE WHOLE CHAIN, in mV per amp at the pin - sensor
 *      times conditioning gain, not the sensor alone. This is what actually
 *      converts a count to a current, and it is what BOARD_I_SENS_UV_PER_A
 *      holds. Expressing the
 *      scale as a sensitivity rather than as full-scale-over-half-the-codes
 *      is deliberate: sensitivity is what a datasheet publishes, it stays
 *      correct if the output never reaches the rails, and it does not quietly
 *      assume a rail-to-rail swing that Hall parts generally do not have.
 *
 *   3. MEASURING RANGE, which on Hall transducers is frequently a MULTIPLE of
 *      the nominal current in the part number - 2x or 3x is common. If the
 *      TL200's measuring range is meaningfully above 200 A, then the earlier
 *      conclusion in this file that the sensor is undersized for the EMRAX
 *      may simply be wrong, and it should be corrected rather than left
 *      standing. The motor needs 339 A peak to reach its 240 Nm.
 *
 *   4. RATIOMETRIC OR NOT. If the output is ratiometric to a supply that is
 *      also VREF+, supply movement cancels and the scaling needs no reference
 *      term. If it is ratiometric to a rail that is NOT VREF+, or is
 *      absolute, it does not cancel and the measured VREF+ has to divide in.
 *      csense.c computes the scale from the measured reference either way,
 *      which is correct for the absolute case and harmless for the
 *      ratiometric one as long as the two rails are the same.
 *
 * The ZERO is the one thing not being assumed: CSense_CalibrateZero measures
 * it with the bridge down, so sensor offset, rail tolerance and ADC offset
 * are calibrated out. Only its DISTANCE FROM MID-SCALE is checked, by the
 * drive self-test, which is what makes point 1 above show up loudly rather
 * than silently.
 */

/* Sensitivity, microvolts per amp. PLACEHOLDER - see above.
 *
 * 8250 uV/A would put +/-200 A at +/-1.65 V, i.e. exactly rail to rail on a
 * 3V3 reference. That is a plausible-looking number and it is a guess; it is
 * here so the arithmetic below has something dimensionally correct to work
 * with, not because anything says it is right. */
#define BOARD_I_SENS_UV_PER_A   8250

/* The sensor measuring range, amps, used by limits.h to bound what may be
 * commanded. PLACEHOLDER at the part number's nominal current - point 3. */
#define BOARD_I_FS_A            200

/* Zero-current offset: conversions averaged at startup with the bridge down.
 * This is the one part of the current chain that is measured rather than
 * assumed, which is why sensor offset and rail tolerance do not need to be
 * known in advance - only the slope does. */
#define BOARD_ADC_ZERO_SAMPLES  128U

/* No front-panel LEDs on this board.
 *
 * Mako Longfin had two, on PB1 and PB2. Here PB1 is an ADC3 input and PB2 is
 * one of the unexplained inputs in section 9, so neither is available and
 * nothing in the pinout replaces them.
 *
 * led.c compiles to nothing under this flag rather than being left writing
 * BSRR at pins that are not outputs - which is what it was doing, silently,
 * once gpio.c stopped configuring them.
 *
 * What is LOST is worth stating, because it is not cosmetic. The stage lamp
 * asked the HARDWARE whether the gates were live rather than asking the state
 * machine, specifically so that a bench tool bringing the bridge up outside
 * Drive_Arm could not produce a lamp that lied. On a 588 V traction inverter
 * "is the bridge live" is exactly the question a person standing next to it
 * wants answered without a debugger. If a spare pin can be found, this is the
 * thing to spend it on. */
#define BOARD_HAS_LEDS          0

/* No front-panel LEDs on this board.
 *
 * Mako Longfin had two, on PB1 and PB2. Here PB1 is an ADC3 input and PB2 is
 * one of the unexplained inputs in section 9, so neither is available and
 * nothing in the pinout replaces them.
 *
 * led.c compiles to nothing under this flag rather than being left writing
 * BSRR at pins that are not outputs - which is what it was doing, silently,
 * once gpio.c stopped configuring them.
 *
 * What is LOST is worth stating, because it is not cosmetic. The stage lamp
 * asked the HARDWARE whether the gates were live rather than asking the state
 * machine, specifically so that a bench tool bringing the bridge up outside
 * Drive_Arm could not produce a lamp that lied. On a 588 V traction inverter
 * "is the bridge live" is exactly the question a person standing next to it
 * wants answered without a debugger. If a spare pin can be found, this is the
 * thing to spend it on. */
#define BOARD_HAS_LEDS          0

/* ==========================================================================
 * 5. Overcurrent comparator - COMP2 + DAC1_CH2
 * ==========================================================================
 * COMP2 compares its non-inverting input against DAC1 channel 2, with 10 mV
 * hysteresis, non-inverted output and an interrupt on both edges. The DAC
 * channel is internally connected only - no output buffer, no pin.
 *
 * !! The comparator input is PA3, which is ALSO ADC1_IN4: the .ioc records
 * PA3 as SharedAnalog carrying both COMP2_INP and ADC1_IN4. ADC1_IN4 is the
 * DC bus voltage in the sequence above. Taken at face value, this comparator
 * watches the BUS VOLTAGE, which would make it an overvoltage trip rather
 * than the overcurrent trip the error masks imply.
 *
 * That contradiction is not resolvable from the .ioc. The board's error masks
 * define ERR_OCP, ERR_OVP, ERR_UVP and ERR_OTP, and nothing says which one
 * COMP2 raises. It has to be settled from the schematic before the comparator
 * is armed, because arming it against the wrong signal buys a hardware trip
 * that either never fires or fires constantly.
 *
 * BOARD_UNKNOWN: what COMP2 actually protects against.
 */
#define BOARD_COMP_INSTANCE     COMP2
#define BOARD_COMP_DAC_CHANNEL  2U

/* ==========================================================================
 * 6. Encoder - SPI3
 * ==========================================================================
 * The board's code names the part RM44SI and reads it as 13-bit:
 * N_STEP_ENCODER is 8192, against Mako Longfin's 32768 for the A1333.
 *
 *   SCK   PC10  SPI3_SCK
 *   MISO  PC11  SPI3_MISO
 *   MOSI  PC12  SPI3_MOSI
 *   CS    PA15  GPIO output, manual
 *
 * The read is interrupt-driven there - a 16-bit transmit of 0 with RXNE
 * enabled - where Isurus does a blocking HAL_SPI_TransmitReceive from inside
 * the control ISR. The transaction shape differs from the A1333's as well:
 * the A1333 needs a command frame and then a NOP frame to clock the answer
 * out, and this part appears to answer within one frame.
 *
 * BOARD_UNKNOWN: the exact frame format, where the angle field sits inside
 * the 16-bit response, and whether there are status or error bits to check.
 * encoder.c cannot be ported faithfully without that.
 */
#define BOARD_ENC_COUNTS        8192U
#define BOARD_ENC_CS_PORT       GPIOA
#define BOARD_ENC_CS_PIN        GPIO_PIN_15

/* ==========================================================================
 * 7. Motor - EMRAX 228 HV
 * ==========================================================================
 * Axial-flux surface-PM synchronous machine. Ten pole pairs, and that is now
 * a datasheet fact rather than the inference recorded here earlier - the
 * sibling-project argument (no motor has an odd pole count, and MiniFOCer
 * sets N_POLES to 7) turned out to reach the right answer.
 *
 * Two independent confirmations that this is the machine the numbers below
 * describe, both from foc.h's own HV analysis, which was written against this
 * motor before this branch existed:
 *
 *   - it quotes 917 Hz electrical, and 10 pole pairs at 5500 rpm is 917 Hz
 *   - it quotes w_e * Lq * iq = 441 V at 917 Hz and 300 A, which back-solves
 *     to Lq = 255.1 uH - the 255 uH inherited from the board's defines.h
 *
 * So the inherited constants are EMRAX constants, not placeholders.
 *
 *   pole pairs   10
 *   R            23.22 mOhm   (datasheet is ~18 mOhm cold; this is plausible
 *                              as a warm figure, and ident.c measures it)
 *   L            255 uH       (Ld = Lq, surface-PM, no saliency)
 *   max          5600 rpm     recovery threshold 5400; the motor is rated to
 *                             6500, so this is a system limit, not the motor's
 *
 * ---------------------------------------------------------------------------
 * lambda_m is NOT settled, and the two available derivations disagree by 35%
 * ---------------------------------------------------------------------------
 *   from foc.h's HV analysis   346 V / 5762 rad/s        = 60.1 mWb
 *   from kv = 10.14 rpm/V      Kt = 60/(2*pi*kv) = 0.942 = 44.4 mWb
 *                              Nm/Arms, /(1.5*p*sqrt(2))
 *
 * Neither is trustworthy and the disagreement is the point. foc.h already
 * carries the scar from exactly this: its bench lambda_m was once DERIVED
 * from a nameplate Kv and came out 14% high, which put the feedforward alone
 * above the entire modulation ceiling. The note there ends "For the EMRAX
 * this is the same warning: measure lambda_m, do not take it from a
 * nameplate."
 *
 * It matters more here than it did there. The back-EMF term is 58% of the bus
 * at 917 Hz, so a 35% error in lambda_m is 20% of the whole supply left
 * uncancelled in the feedforward.
 *
 * The measurement is the one foc.h describes: spin the motor with the
 * feedforward disabled and read back what voltage the loop actually needed,
 * at several speeds below the modulation ceiling. Until that exists there is
 * no defensible number to put in FOC_LAMBDA_M_WB.
 *
 * ---------------------------------------------------------------------------
 * The sense chain, now that the schematic has been read
 * ---------------------------------------------------------------------------
 * Both questions are answered in section 4, and they came out differently.
 *
 * The BUS sense is fine. The divider is exactly 400:1 with better than 2x
 * headroom over a full pack. It was the inherited 0.05 V/LSB constant that
 * was wrong, by about 6x, and it would have reported a 588 V bus as 91 V.
 *
 * The CURRENT sensor is a Mornsun TL200-A2PV, and whether it is big enough is
 * OPEN. The nominal 200 A in the part number is below the 339 A peak this
 * motor needs for its 240 Nm - but Hall transducers commonly measure to a
 * multiple of their nominal current, so the measuring range may well cover
 * it. Section 4 lists what the datasheet has to settle. Until then the
 * scaling constants there are placeholders and nothing should be energised
 * against them.
 */
#define BOARD_MOTOR_NAME        "EMRAX 228 HV"
#define BOARD_MOTOR_POLE_PAIRS  10U
#define BOARD_MOTOR_KV          10.14f
#define BOARD_MOTOR_R_OHM       0.02322f
#define BOARD_MOTOR_L_H         255e-6f
#define BOARD_MOTOR_MAX_RPM     5600U
#define BOARD_UVLO_MV           30000U
#define BOARD_UVLO_HYST_MV      5000U

/* ==========================================================================
 * 8. Communications
 * ==========================================================================
 *   FDCAN2   PB5 RX / PB6 TX
 *   LPUART1  PC0 RX / PC1 TX  - the board's code uses this for printf-style
 *                               bring-up output, which Mako Longfin did not
 *                               have; everything there went over SWD.
 *
 * FDCAN2 rather than FDCAN1 is the one piece of unambiguously good news in
 * this port: the PB8 BOOT0 / FDCAN1_RX collision that made a factory-fresh
 * Mako Longfin boot the ST bootloader cannot happen here, because PB8 is not
 * used at all. The bit timing still has to be recomputed - can.c's prescaler
 * was worked out against a 128 MHz clock and this board runs at 160.
 */
#define BOARD_FDCAN_INSTANCE    FDCAN2
#define BOARD_HAS_DEBUG_UART    1

/* ==========================================================================
 * 9. Pins CubeMX configured and did not explain
 * ==========================================================================
 * All of these are plain inputs, no pull, no label:
 *
 *   PA8  PA9  PA12
 *   PB0  PB2  PB4  PB10  PB11  PB14  PB15
 *   PC9
 *   PD2
 *
 * BOARD_UNKNOWN, every one of them. Some are almost certainly gate-driver
 * fault outputs, and a fault line read as a floating input is a fault that is
 * never noticed - so this list is worth closing out early rather than late.
 *
 * There is no LED among the labelled pins either, so docs/LED_CODES.md
 * describes a front panel this board may not have: led.c drives GPIOB pins
 * that are configured as inputs here.
 */

#ifdef __cplusplus
}
#endif

#endif /* BOARD_H */
