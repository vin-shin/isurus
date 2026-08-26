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
 * Recorded in raw counts because that is what was configured and what was
 * verified against the generated code. The nanosecond equivalent depends on
 * the dead-time clock divider chain in RM0440 and has NOT been confirmed
 * against the reference manual or seen on a scope. Do not write a nanosecond
 * figure into a comment here until someone has put a probe on a switch node:
 * a dead time that is wrong in the short direction is a shoot-through.
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

/* ---- current sensor: +/-200 A, and undersized for this motor --------------
 *
 * Bidirectional, +/-200 A peak, which is 141 Arms.
 *
 *     motor continuous   100 Arms = 141 A peak    fits, about 30% margin
 *     motor peak         240 Arms = 339 A peak    1.7x OVER the sensor
 *
 * Continuous operation is fully measurable; the machine's peak rating is not.
 * Torque follows current, so taking the EMRAX 240 Nm at 240 Arms:
 *
 *     200 A peak (sensor ceiling)   141 Arms   ~141 Nm    59% of peak torque
 *     160 A peak (80% margin)       113 Arms   ~113 Nm    47% of peak torque
 *     350 A peak (a bigger part)    248 Arms   ~248 Nm   103% of peak torque
 *
 * A +/-350 A part covers the motor 339 A peak with 3% to spare, which is why
 * 350 A is the right number rather than merely a round one. As built, about
 * half the machine peak torque is unreachable - not because the inverter
 * cannot deliver it, but because current above the sensor range cannot be
 * MEASURED, and a current loop reading a saturated sensor believes it has
 * arrived and stops pushing while the real current climbs.
 *
 * That is a hardware change, not a firmware one. Nothing here should be
 * relaxed to work around it.
 *
 * BOARD_I_FS_A is the part rating and is what limits.h bounds against, which
 * holds whatever the output swing turns out to be. The per-LSB figure below
 * additionally assumes the sensor full range spans the whole ADC - that needs
 * the part number to confirm, and it is used for scaling a reading, never for
 * deciding a limit. */
#define BOARD_I_FS_A            200
#define BOARD_I_UA_PER_LSB      97656     /* 200 A / 2048 codes, provisional */

/* Zero-current offset: the board's code averages 128 samples of every ADC
 * channel at startup with the bridge disabled. Same idea as Mako Longfin's
 * CS_ZERO_SAMPLES, different count. */
#define BOARD_ADC_ZERO_SAMPLES  128U

/* 0 on this board: there are no OPAMPs in the current-sense path.
 *
 * Mako Longfin fed two phases to ADC2 and ADC5 through internal OPAMP
 * followers, and csense.c is still written around that. This flag fences that
 * code off and makes CSense_Init refuse, which - because the refusal leaves
 * the zero-current offsets at 0, far outside DRIVE_CS_ZERO_TOL_CODES - also
 * stops Drive_SelfTest arming the bridge. See csense.c. */
#define BOARD_HAS_OPAMP_CSENSE  0

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
 * The CURRENT sensor is genuinely undersized: +/-200 A against a motor whose
 * peak rating is 339 A peak. Continuous operation fits with margin; about
 * half the machine peak torque does not, and that is a hardware limit rather
 * than something firmware can work around. A +/-350 A part would cover it.
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
