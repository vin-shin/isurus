/**
  ******************************************************************************
  * @file    limits.h
  * @brief   What this machine can physically do, in one place.
  *
  *          Every number here is traceable to a datasheet, a measurement, or
  *          HARDWARE_NOTES.md - none of them are preferences. Control gains
  *          and setpoints live with their loops; this file is only for bounds
  *          that exist because of the hardware.
  *
  *          Commands are SATURATED to these bounds rather than rejected. A
  *          request slightly over the limit is usually a host that scaled
  *          something wrong, and doing the largest safe thing is better than
  *          doing nothing - but saturation is silent, so every clamp also
  *          increments a counter that telemetry exposes. Silence and
  *          invisibility are different things.
  *
  *          The clamping is applied in the motion loop, not in the transport,
  *          so it protects every path equally: CAN, the SWD tools, and
  *          anything written by hand with a debugger.
  *
  * ---------------------------------------------------------------------------
  * The hardware, cheapest limit first
  * ---------------------------------------------------------------------------
  *   motor            EMRAX 228 HV, 100 Arms continuous / 240 Arms peak
  *   pack             140s2p, 350 V empty to 588 V full, 518 V nominal
  *   current sensors  unknown part; 0.04 A/LSB implies +/-82 A
  *   bus sense        unknown divider; 0.05 V/LSB implies 0..204.8 V
  *
  *   The SENSORS are the binding constraint here, which is the opposite of
  *   the previous board, where they had been sized around the motor on
  *   purpose. The current sensor bounds commandable current to less than half
  *   the machine's continuous rating, and the bus sense cannot read the pack
  *   at all. Both scale factors are inherited from a bring-up program with no
  *   schematic behind them - board.h section 7 has the three possibilities
  *   and why none of them should be guessed.
  ******************************************************************************
  */
#ifndef LIMITS_H
#define LIMITS_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* ---- current ------------------------------------------------------------ *
 *
 * iq_ref is a PEAK amplitude, not an RMS phase current. The EMRAX's ratings
 * are published in Arms, so they are converted before being compared here.
 *
 * Four numbers bound this and the smallest wins:
 *
 *   339 A  motor, peak       240 Arms - a short rating, not a design point
 *   141 A  motor, continuous 100 Arms - the real thermal limit of the machine
 *    82 A  current sensors            - 0.04 A/LSB about the calibrated zero
 *                                       over a 12-bit ADC. Current above this
 *                                       cannot be MEASURED, so it cannot be
 *                                       controlled either: the loop reads a
 *                                       CEILING, concludes it has arrived, and
 *                                       stops pushing while the real current
 *                                       keeps climbing
 *    65 A  this bound                 - 80% of the sensor
 *
 * So the sensor binds, at less than half the motor's continuous rating, and
 * this ceiling is set from the sensor rather than the machine.
 *
 * !! THE SENSOR FIGURE IS UNVERIFIED. !! 0.04 A/LSB comes from the board's
 * bring-up code with no sensor part number and no schematic behind it - see
 * board.h section 7. It is used anyway because it is the only number
 * available and it is the CONSERVATIVE choice in both directions: if the real
 * sensor is wider, 65 A is merely cautious; if it is genuinely this narrow,
 * 65 A is correct. Raising this needs the schematic, not a decision.
 *
 * 65 A peak is 46 Arms, comfortably inside the machine's 100 Arms continuous,
 * so nothing here is thermally interesting to the motor. That is the right
 * place to be for a bridge that has never been energised. */
#define LIM_IQ_MAX_MA           65000

/* ---- bus voltage -------------------------------------------------------- *
 *
 * 140s2p. 588 V is 4.2 V/cell and a full pack; 350 V is 2.5 V/cell and an
 * empty one. Above the maximum the drive latches the same fault path as an
 * overcurrent.
 *
 * !! The bus SENSE cannot currently read this range. !! At the 0.05 V/LSB
 * recorded in board.h a 12-bit conversion tops out at 204.8 V, which is a
 * third of the pack. That is not something this file can fix - it is a
 * scaling or hardware question - but it is why these numbers are a
 * SPECIFICATION rather than something the drive can presently enforce. A
 * saturated bus reading is the dangerous direction, because
 * FOC_SetGainsForVbus divides by it: clamped at 204.8 V on a 518 V bus, every
 * gain comes out 2.5x too large.
 *
 * NOTE this bounds the MOTOR and the PACK. The board's own ceiling - FET Vds,
 * bulk capacitor rating, gate driver supply - is not in the source material
 * and has not been verified. Check the schematic before running near this.
 *
 * The undervoltage bound is not a rating, it is a sanity floor: below it the
 * bus reading is not trustworthy and the gain rescale would divide by a bad
 * number. It sits far below the pack's 350 V on purpose, so that bring-up on
 * a lab supply still works; protecting cells from over-discharge is the BMS's
 * job, not this loop's. See FOC_VBUS_MIN_MV, which must agree with it - a bus
 * the self-test accepts but FOC_SetGainsForVbus rejects would leave the loop
 * running on gains sized for a different supply. */
#define LIM_VBUS_MAX_MV         588000
#define LIM_VBUS_MIN_MV         20000

/* 4.05 V/cell. Close enough to full that regen has nowhere to put its energy,
 * which is what this warning is for on a traction drive. */
#define LIM_VBUS_WARN_MV        567000

/* ---- speed -------------------------------------------------------------- *
 *
 * This is a control sanity bound, not a hardware one. The drive is back-EMF
 * limited long before anything mechanical complains: at vmax = 0.25 of a
 * 518 V bus the applied vector is ~130 V, and the motor simply stops
 * accelerating when back-EMF matches it.
 *
 * What the bound is really for is the profile generator. Asking for 10^6
 * deg/s does not break the motor, it breaks the maths - the S-curve computes
 * a braking distance from v^2/2a, and an absurd v makes that distance longer
 * than any move, so the profile decelerates from the first tick and never
 * gets anywhere.
 *
 * 33600 deg/s is 5600 rpm, which is BOARD_MOTOR_MAX_RPM - the system limit
 * the board's own defines carry, below the EMRAX's own 6500 rpm rating. This
 * was 3600 deg/s, i.e. 600 rpm, for the bench machine, and would silently
 * have capped this drive at a ninth of its speed range. */
#define LIM_VEL_MAX_DPS         33600

/* ---- profile ------------------------------------------------------------ *
 *
 * Bounds rather than targets. Zero acceleration would stall the profile
 * permanently; absurd acceleration just saturates the current loop, which is
 * survivable but makes the S-curve meaningless. Jerk of 0 is legal and means
 * "plain trapezoid". */
#define LIM_ACCEL_MIN_DPS2      10
#define LIM_ACCEL_MAX_DPS2      100000
#define LIM_JERK_MAX_DPS3       2000000

/* ---- position ----------------------------------------------------------- *
 *
 * +/-100 turns. The unwrapped count is a signed 32-bit number of encoder
 * counts, so it does not overflow until ~65000 turns, but a command far
 * outside the machine's travel is a host error rather than an intent - and it
 * would take minutes to execute at any sane speed. */
#define LIM_POS_MAX_DEG_X10     360000

/* ---- gains -------------------------------------------------------------- *
 *
 * Loose bounds whose only job is to stop a corrupted or mis-scaled write from
 * turning the loop into an oscillator. Real tuning lives far inside these. */
#define LIM_KP_X1000_MAX        200000
#define LIM_KI_X1000_MAX        500000
#define LIM_KD_X1000_MAX        20000
#define LIM_VKP_X1000_MAX       20000
#define LIM_VKI_X1000_MAX       200000

/* ---- field weakening ---------------------------------------------------- */

/* How negative id may be driven, mA. Field weakening spends d-axis current to
 * buy SPEED, not torque: it produces no torque on a surface-magnet machine
 * and costs copper loss and thermal budget the whole time it is applied.
 *
 * Bounded well below LIM_IQ_MAX_MA on purpose. id and iq share one current
 * limit through the vector magnitude, so every amp spent weakening is an amp
 * not available for torque, and an unbounded weakening loop that has decided
 * it needs more flux reduction will happily take all of it.
 *
 * A third of LIM_IQ_MAX_MA, which is the ratio the bench carried (4 A of 12)
 * and is now expressed as that ratio rather than as a literal. Left at 4000
 * it would have been 6% of this board's budget - not a deliberate tightening,
 * just a number that did not follow its own justification across the port,
 * and one that would have quietly capped the top of the speed range.
 *
 * NOT verified on the motor. A derived bound, and the number wants checking
 * against measured temperature rise before anything relies on it - more so
 * here, because weakening an EMRAX is tens of amps of pure copper loss. */
#define LIM_ID_FW_MAX_MA        (LIM_IQ_MAX_MA / 3)

/* ---- haptics ------------------------------------------------------------ */
#define LIM_DETENT_COUNT_MAX    512
#define LIM_ENDSTOP_K_MAX       100000

/* Saturate v into [lo, hi]; increments *hits if it had to. */
static inline int32_t Lim_Clamp(int32_t v, int32_t lo, int32_t hi, uint32_t *hits)
{
  if (v < lo) { if (hits) { (*hits)++; } return lo; }
  if (v > hi) { if (hits) { (*hits)++; } return hi; }
  return v;
}

#ifdef __cplusplus
}
#endif

#endif /* LIMITS_H */
