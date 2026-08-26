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
  *   motor            12S LiPo, 44.4 V nominal / 50.4 V full, 22 A continuous
  *   current sensors  CT4022-A40BSN8, +/-40 A bidirectional
  *   FETs             220 A
  *   bench OC trip    OC_TRIP_MA, 15 A - a bench margin, not a hardware limit
  *
  *   The motor is the binding constraint on current, not the FETs, and the
  *   sensors are sized around the motor for exactly that reason - see
  *   HARDWARE_NOTES section 6 before "upgrading" anything.
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
 * Four numbers bound this and the smallest wins:
 *
 *   220 A   FETs                    - irrelevant, nothing else gets close
 *    40 A   current sensors         - current above this cannot be MEASURED,
 *                                     so it cannot be controlled either; the
 *                                     loop would be running open-loop into a
 *                                     saturated sensor
 *    22 A   motor, continuous       - the real thermal limit of the machine
 *    15 A   OC_TRIP_MA              - the bench trip; commanding above this
 *                                     guarantees a trip rather than motion
 *
 * So the command ceiling has to sit below the trip, not below the motor
 * rating. 12 A leaves 20% margin to the trip for transients and ripple. If
 * the trip is ever raised, raise this with it - and check the motor's 22 A
 * before going past it. */
#define LIM_IQ_MAX_MA           12000

/* ---- bus voltage -------------------------------------------------------- *
 *
 * 50.4 V is a full 12S pack and the most the MOTOR is specified for. Above
 * that the drive latches the same fault path as an overcurrent.
 *
 * NOTE: this bounds the MOTOR only. The board's own ceiling - FET Vds, bulk
 * capacitor rating, gate driver supply - is not recorded in HARDWARE_NOTES and
 * has not been verified here. Check the schematic before running near this.
 *
 * The undervoltage bound is not a rating, it is a sanity floor: below it the
 * bus reading is not trustworthy and the gain rescale would divide by a bad
 * number. See FOC_VBUS_MIN_MV. */
#define LIM_VBUS_MAX_MV         50400

/* The undervoltage bound the comment above already described but that nothing
 * defined until the drive self-test needed to test against it. Same value as
 * FOC_VBUS_MIN_MV and for the same reason - the two must agree, because a bus
 * the self-test accepts but FOC_SetGainsForVbus rejects would leave the loop
 * running on gains sized for a different supply. */
#define LIM_VBUS_MIN_MV         6000
#define LIM_VBUS_WARN_MV        45000

/* ---- speed -------------------------------------------------------------- *
 *
 * This is a control sanity bound, not a hardware one. The drive is back-EMF
 * limited long before anything mechanical complains: at vmax = 0.25 of a
 * 26 V bus the applied vector is ~6.5 V, and the motor simply stops
 * accelerating when back-EMF matches it.
 *
 * What the bound is really for is the profile generator. Asking for 10^6
 * deg/s does not break the motor, it breaks the maths - the S-curve computes
 * a braking distance from v^2/2a, and an absurd v makes that distance longer
 * than any move, so the profile decelerates from the first tick and never
 * gets anywhere. 3600 deg/s is 600 rpm, comfortably past anything this bench
 * has reached. */
#define LIM_VEL_MAX_DPS         3600

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
 * it needs more flux reduction will happily take all of it. 4 A of the 12 A
 * budget is the most this is allowed to claim.
 *
 * NOT verified on the motor. This is a derived bound, and the number wants
 * checking against measured temperature rise before anything relies on it. */
#define LIM_ID_FW_MAX_MA        4000

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
