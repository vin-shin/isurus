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

/* Scale factors, taken from the board's bring-up loop:
 *     current = (raw - offset) * 0.04    -> 40 mA per LSB
 *     vbus    =  raw          * 0.05     -> 50 mV per LSB
 *
 * These are the numbers the board's author used, not numbers derived here
 * from a shunt value and a gain. Neither has been checked against a divider
 * ratio, and the current sensor's part number is not recorded anywhere in the
 * project. Treat both as provisional until they are calibrated against a
 * meter - the Vbus one especially, because it feeds the undervoltage trip.
 */
#define BOARD_I_UA_PER_LSB      40000
#define BOARD_VBUS_UV_PER_LSB   50000

/* Zero-current offset: the board's code averages 128 samples of every ADC
 * channel at startup with the bridge disabled. Same idea as Mako Longfin's
 * CS_ZERO_SAMPLES, different count. */
#define BOARD_ADC_ZERO_SAMPLES  128U

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
 * 7. Motor
 * ==========================================================================
 * From the board's defines.h and its motor_t initialiser. This is a DIFFERENT
 * motor from Mako Longfin's 20-pole-pair 12S machine, so none of the tuned
 * gains or limits carry over.
 *
 *   N_POLES  10          the board's code calls this "poles"; whether it means
 *                        poles or pole pairs is BOARD_UNKNOWN, and taking it
 *                        the wrong way is a factor-of-two error in the
 *                        electrical angle
 *   kv       10.14
 *   R        23.22 mOhm
 *   L        255 uH
 *   max      5600 rpm    recovery threshold 5400
 *   UVLO     30 V, 5 V hysteresis
 *
 * Phase 7's ident.c measures R and L on the machine itself. Running it here is
 * the cheapest way to find out whether these numbers describe the motor that
 * is actually attached.
 */
#define BOARD_MOTOR_POLES       10U
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
