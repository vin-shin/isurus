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
  *   current sensor   Mornsun TL200-A2PV, +/-500 A measurement range
  *   bus sense        400:1 divider into an isolated amplifier chain
  *
  *   The MOTOR is the binding constraint on current, at 339 A peak against a
  *   sensor good for 500 A and a converter that clips at 644 A. That is the
  *   right way round. Sustained overload is the winding temperature's job,
  *   not the current limit's.
  *
  *   Neither analogue chain's absolute scaling is verified on hardware yet.
  *   The current one is now datasheet-derived end to end; the bus one models
  *   only the first of its three stages. See board.h section 4.
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
 *   644 A  ADC clip                    - 2048 counts at 314 mA per count
 *   500 A  sensor measurement range    - BOARD_I_FS_A, TL200-A2PV IPM
 *   400 A  sensor OCD trip             - fires in hardware, unconnected here
 *   339 A  motor, peak 240 Arms        - THIS ONE, and it is the right one
 *
 * So the MOTOR binds, which is the correct arrangement and the same one Mako
 * Longfin had. An earlier version of this file set the ceiling from the
 * sensor because the part number was misread as a +/-200 A range; the
 * datasheet says +/-500 A and that claim is withdrawn.
 *
 * 339 A peak is the machine's 10-second rating, not a place to live. Nothing
 * here stops the drive sitting at it - deliberately, because a current limit
 * that enforced the CONTINUOUS rating would also forbid the bursts this
 * machine exists to deliver. Sustained overload is caught by winding
 * temperature instead, which is why LIM_TEMP_MOTOR_MAX_CX10 below is not
 * optional and why Drive_SelfTest refuses to arm without a working KTY.
 *
 * The OCD line at 400 A would be the right backstop for this: above the
 * motor's peak so it cannot nuisance trip, below the sensor's range so the
 * reading is still good when it fires, and a comparator inside the sensor so
 * it responds in 0.3 us rather than a control period. It is not connected.
 * See board.h. */
#define LIM_IQ_MAX_MA           339000

/* ---- bus voltage -------------------------------------------------------- *
 *
 * 140s2p. 588 V is 4.2 V/cell and a full pack; 350 V is 2.5 V/cell and an
 * empty one. Above the maximum the drive latches the same fault path as an
 * overcurrent.
 *
 * The sense chain measures this comfortably: an exactly 400:1 divider, so a
 * full pack presents 1.47 V to the pin and the conversion has better than 2x
 * headroom. What was broken was the firmware SCALE FACTOR inherited from the
 * board bring-up loop, which implied a 62:1 divider and would have reported a
 * full pack as 91 V - see board.h section 4. Everything in this block assumes
 * that has been fixed and that VREF+ is measured rather than assumed.
 *
 * NOTE this bounds the MOTOR and the PACK. The board own ceiling - FET Vds,
 * bulk capacitor rating, gate driver supply - is not in the source material
 * and has not been verified. Check the schematic before running near this.
 *
 * The undervoltage bound is not a rating, it is a sanity floor: below it the
 * bus reading is not trustworthy and the gain rescale would divide by a bad
 * number. It sits far below the pack 350 V on purpose, so that bring-up on a
 * lab supply still works; protecting cells from over-discharge is the BMS
 * job, not this loop. See FOC_VBUS_MIN_MV, which must agree with it - a bus
 * the self-test accepts but FOC_SetGainsForVbus rejects would leave the loop
 * running on gains sized for a different supply. */
#define LIM_VBUS_MAX_MV         588000
#define LIM_VBUS_MIN_MV         20000

/* 4.05 V/cell. Close enough to full that regen has nowhere to put its energy,
 * which is what this warning is for on a traction drive. */
#define LIM_VBUS_WARN_MV        567000

/* ---- temperature -------------------------------------------------------- *
 *
 * The EMRAX 228 datasheet gives 120 C as the maximum winding temperature.
 * That is a LIMIT, not an operating point, so the trip sits below it.
 *
 * This is load-bearing rather than belt-and-braces, and the reason is
 * LIM_IQ_MAX_MA above. That ceiling is 113 Arms against a machine rated 100
 * Arms continuous - deliberately, so short bursts past continuous are
 * available - which means the CURRENT limit does not protect the motor from
 * sustained overload and was never meant to. This does. Remove it and the
 * drive will happily hold 113 Arms until something melts.
 *
 * 110 C to fault, 95 C to warn. The gap exists because thermal time constants
 * on a machine this size are tens of seconds: a warning that arrives 15 K
 * before the trip is minutes of notice, which is enough for a host to back
 * the torque off rather than be cut off mid-corner.
 *
 * Tenths of a degree, matching what thermal.c reports, so that no call site
 * has to remember a scale factor. */
#define LIM_TEMP_MOTOR_MAX_CX10   1100
#define LIM_TEMP_MOTOR_WARN_CX10   950

/* ---- temperature -------------------------------------------------------- *
 *
 * The EMRAX 228 datasheet gives 120 C as the maximum winding temperature.
 * That is a LIMIT, not an operating point, so the trip sits below it.
 *
 * This is load-bearing rather than belt-and-braces, and the reason is
 * LIM_IQ_MAX_MA above. That ceiling is 113 Arms against a machine rated 100
 * Arms continuous - deliberately, so short bursts past continuous are
 * available - which means the CURRENT limit does not protect the motor from
 * sustained overload and was never meant to. This does. Remove it and the
 * drive will happily hold 113 Arms until something melts.
 *
 * 110 C to fault, 95 C to warn. The gap exists because thermal time constants
 * on a machine this size are tens of seconds: a warning that arrives 15 K
 * before the trip is minutes of notice, which is enough for a host to back
 * the torque off rather than be cut off mid-corner.
 *
 * Tenths of a degree, matching what thermal.c reports, so that no call site
 * has to remember a scale factor. */
#define LIM_TEMP_MOTOR_MAX_CX10   1100
#define LIM_TEMP_MOTOR_WARN_CX10   950

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
