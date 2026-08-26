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
  *          ADC2, DMA2 channel 1, four channels at 92.5 cycle sampling:
  *
  *            rank 1  PC4  ADC2_IN5    motor winding, KTY
  *            rank 2  PA5  ADC2_IN13   power stage
  *            rank 3  PA6  ADC2_IN3    power stage
  *            rank 4  PA7  ADC2_IN4    power stage
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
  *          !! WHAT IS NOT KNOWN is the conditioning around it. A KTY needs
  *          either a bias current or a pull-up to turn resistance into a
  *          voltage, and this branch has never seen that part of the
  *          schematic. THERM_PULLUP_OHM below assumes a simple pull-up to the
  *          ADC reference and is a PLACEHOLDER. Everything downstream of it -
  *          the reported temperature, the trip - is only as right as that
  *          number.
  *
  *          Getting it wrong is not symmetric. A pull-up assumed too LARGE
  *          makes the computed resistance read low, so the winding reports
  *          COOLER than it is and the trip never fires. That is the direction
  *          that burns a motor, so the fault is written to treat an
  *          out-of-range reading as a fault rather than as "probably fine".
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

#define TH_SEQ_LEN          4U

#define TH_IDX_MOTOR        0U      /* PC4, the KTY   */
#define TH_IDX_STAGE0       1U      /* PA5            */
#define TH_IDX_STAGE1       2U      /* PA6            */
#define TH_IDX_STAGE2       3U      /* PA7            */

/* KTY81-210, from the part's own characteristic. */
#define TH_KTY_R25_OHM      2000
#define TH_KTY_A_E6         7874    /* a * 1e6 */
#define TH_KTY_B_E6         19      /* b * 1e6, 1.874e-5 rounded */

/* !! PLACEHOLDER. The conditioning network is not known - see the header
 * comment. A KTY81-210 in a simple pull-up to the reference gives its widest
 * swing across 0..150 C when the pull-up is near the sensor's mid-range
 * resistance, and 2200 ohm is that number for this part. It is a plausible
 * choice, not a measured one. */
#define THERM_PULLUP_OHM    2200

/* Codes outside this band mean the KTY is open, shorted, or the conditioning
 * is not what THERM_PULLUP_OHM says. Treated as a fault rather than ignored:
 * a disconnected temperature sensor reads as a fixed, plausible, entirely
 * fictional temperature, which is the same failure mode the encoder
 * substitution counter exists for. */
#define TH_RAW_MIN          100U
#define TH_RAW_MAX          3995U

typedef struct {
  uint32_t raw[TH_SEQ_LEN];  /* last conversions, all four channels      */
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
