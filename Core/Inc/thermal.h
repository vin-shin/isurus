/**
  ******************************************************************************
  * @file    thermal.h
  * @brief   Motor and power-stage temperature, from ADC2 over DMA.
  *
  *          Mako Longfin had none of this: a 22 A bench motor on a 15 V supply
  *          could not overheat itself faster than someone would notice. This
  *          board can. The EMRAX is rated 100 Arms continuous and the command
  *          ceiling deliberately sits above that (see LIM_IQ_MAX_MA), so
  *          SUSTAINED overload is not caught by the current limit by design -
  *          it is caught here. Without this module the drive has no thermal
  *          protection at all.
  *
  *          ADC2, DMA2 channel 1, ONE channel at 92.5 cycle sampling:
  *
  *            rank 1  PC4  ADC2_IN5    motor winding, KTY
  *
  *          PA5, PA6 and PA7 - TEMP_U/V/W - used to be ranks 2 to 4 and are
  *          NOT ANALOGUE. They are APWM outputs from the gate drivers: 400 kHz
  *          square waves whose DUTY carries the measurement. Converting them
  *          produced numbers, which is the problem - see the note further
  *          down. They are out of the sequence until something can read them
  *          properly.
  *
  *          The long sampling time is right and should stay: these are slow,
  *          high-impedance sources and there is no deadline anywhere near
  *          them. This sequence free-runs and is read from the main loop -
  *          unlike the current sense, nothing here needs to be aligned to the
  *          switching period.
  *
  * ---------------------------------------------------------------------------
  * The motor channel: KTY on PC4
  * ---------------------------------------------------------------------------
  *          The EMRAX 228 ships with a KTY81-210 embedded in the stator
  *          winding, which is what PC4 reads. It is a silicon PTC: resistance
  *          RISES with temperature, about 2000 ohm at 25 C and roughly
  *          0.8%/K near there. The curve is a known property of the part:
  *
  *              R(T) = R25 * (1 + a*(T-25) + b*(T-25)^2)
  *
  *          with a = 7.874e-3 and b = 1.874e-5 for the KTY81-2xx family. That
  *          is inverted numerically below rather than linearised, because the
  *          quadratic term is worth 8 K by the time the winding is at 120 C -
  *          which is exactly where being wrong matters.
  *
  *          !! THE CONDITIONING IS NOT A PULL-UP, SO THE MODEL BELOW HAS THE
  *          WRONG SHAPE - not merely the wrong constant. !!
  *
  *          Schematic sheet 9, "Emrax KTY", is a two-stage circuit around a
  *          TLV9302 dual op-amp: a first stage driving the KTY node through a
  *          100 Ohm series resistor with a 100k/100k network setting its
  *          reference, a 3v3 clamp and filtering at the sensor node, and a
  *          second stage buffering it to PC4.
  *
  *          Thermal_RawToOhm still models "KTY to ground, pull-up to the
  *          reference", which is the ratiometric divider form. A driven node
  *          - whether that first stage is a voltage source behind 100 Ohm or
  *          a current source - gives a LINEAR relationship between resistance
  *          and pin voltage instead. The right fix is to replace the function,
  *          not to retune THERM_PULLUP_OHM.
  *
  *          The resistor values on that sheet did not render legibly enough to
  *          commit to, and the topology decides which linear form applies, so
  *          both are still open.
  *
  *          Getting it wrong is not symmetric. Any error that makes the
  *          computed resistance read LOW reports the winding COOLER than it
  *          is, and the trip never fires - the direction that burns a motor.
  *          That is why an out-of-range reading is treated as a fault rather
  *          than as "probably fine", and why the whole module should be
  *          calibrated by substituting two known resistors for the KTY and
  *          reading the ADC, rather than by deriving anything.
  ******************************************************************************
  */
#ifndef THERMAL_H
#define THERMAL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "main.h"
#include "board.h"

#define TH_SEQ_LEN          1U
#define TH_IDX_MOTOR        0U      /* PC4  KTY, motor winding */

/* ---------------------------------------------------------------------------
 * TEMP_U/V/W are APWM, and an ADC cannot read them. CONFIRMED.
 * ---------------------------------------------------------------------------
 * They are the isolated analogue sense outputs of three UCC21756-Q1 gate
 * drivers, and the driver reports its measurement as a DUTY CYCLE on a
 * 400 kHz carrier. From the datasheet:
 *
 *      I_AIN     203 uA constant current source into the sensing node
 *      V_AIN     0.6 .. 4.5 V usable, so a 3.0 .. 22.2 kOhm sensor window
 *      f_APWM    400 kHz  (380..420)
 *      D_APWM    88% at 0.6 V, 50% at 2.5 V, 10% at 4.5 V
 *
 * Those three points are exactly linear at -20% per volt, so
 *
 *      V_AIN = 2.5 + (50 - D) / 20        volts, D in percent
 *      R_sensor = V_AIN / 203 uA
 *
 * and the driver biases the thermistor itself, which is why there is no
 * external divider to find on the schematic.
 *
 * Sampling that with the ADC is not merely crude, it is degenerate: a 400 kHz
 * carrier against a 20 kHz conversion trigger is exactly twenty carrier
 * periods per sample, so the samples are not even randomised across the duty
 * cycle - they land wherever the fixed phase relationship puts them and
 * average to a number that is stable, plausible, and unrelated to
 * temperature. That is worse than noise.
 *
 * So the three ranks are OUT of the sequence rather than converted and
 * ignored. Leaving meaningless values in a buffer named for temperature is
 * how they end up being read as temperature.
 *
 * TWO WAYS TO GET THEM BACK, in docs/LATER.md section 2:
 *   - one RC per channel and they become ordinary ADC inputs again. 10k and
 *     100 nF puts the corner at 159 Hz, 68 dB below the carrier, and
 *     temperature does not need bandwidth. Cheapest by far, but it is a board
 *     change.
 *   - timer input capture with DMA. PA5, PA6 and PA7 all have capture
 *     alternate functions on this package (TIM2_CH1, TIM3_CH1, TIM3_CH2 -
 *     confirm against the AF table). At 160 MHz a 2.5 us period is 400 timer
 *     counts, so the 10..88% duty span is 312 counts of range: plenty.
 *     Firmware only, but 800k edges per second across three channels means
 *     DMA, not interrupts. */

/* KTY81-210, from the part's own characteristic. */
#define TH_KTY_R25_OHM      2000
#define TH_KTY_A_E6         7874    /* a * 1e6 */
#define TH_KTY_B_E6         19      /* b * 1e6, 1.874e-5 rounded */

/* !! PLACEHOLDER, AND FOR A TOPOLOGY THIS BOARD DOES NOT HAVE. !!
 *
 * Kept only so Thermal_RawToOhm compiles and reports something monotonic in
 * temperature while the real front end is worked out - see the header. The
 * fix is to replace that function with the linear form the TLV9302 stage
 * actually implements, at which point this constant disappears rather than
 * being retuned. */
#define THERM_PULLUP_OHM    2200

/* Codes outside this band mean the KTY is open, shorted, or the conditioning
 * is not what this module models. That last case is currently TRUE - see the
 * header - so this band is doing more work than it should until the front end
 * is corrected. Treated as a fault rather than ignored:
 * a disconnected temperature sensor reads as a fixed, plausible, entirely
 * fictional temperature, which is the same failure mode the encoder
 * substitution counter exists for. */
#define TH_RAW_MIN          100U
#define TH_RAW_MAX          3995U

typedef struct {
  uint32_t raw[TH_SEQ_LEN];  /* last conversion, the KTY                 */
  int32_t  motor_c_x10;      /* motor winding, tenths of a degree C      */
  uint32_t motor_ohm;        /* KTY resistance, ohms                     */
  uint32_t valid;            /* 1 if the motor channel is in range       */
  uint32_t samples;
  uint32_t range_errors;     /* conversions outside TH_RAW_MIN..MAX      */
} ThermalTelem_t;

/* Brings up ADC2 with its four-channel sequence and DMA, and starts it
 * free-running. Returns 0 on success. */
int Thermal_Init(ThermalTelem_t *t);

/* Copy the latest conversions and convert the motor channel. Main loop only -
 * this is not on the control path. Returns 0 on success, -1 if the motor
 * channel is out of range. */
int Thermal_Read(ThermalTelem_t *t);

/* Raw code -> KTY resistance in ohms, using the measured reference. */
uint32_t Thermal_RawToOhm(uint32_t raw);

/* KTY resistance -> tenths of a degree C. Inverts the quadratic numerically;
 * saturates rather than wrapping outside the part's usable range. */
int32_t Thermal_OhmToCx10(uint32_t ohm);

#ifdef __cplusplus
}
#endif

#endif /* THERMAL_H */
