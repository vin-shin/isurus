/**
  ******************************************************************************
  * @file    haptic.h
  * @brief   Force-feedback dial: render a physical feel by commanding torque
  *          as a function of position and velocity.
  *
  *          There is no loop here and nothing to tune for stability. Every
  *          other mode in this project measures an error and corrects it; this
  *          one just answers "given where the shaft is and how fast it is
  *          moving, what should it feel like pushing against?" and hands that
  *          to the current loop. The rotor is free - your hand closes the loop.
  *
  *          Because of that it runs at the FULL 30 kHz ISR rate rather than
  *          the 1 kHz outer loop. Detents are the one thing here that a slow
  *          update makes obviously worse: the wall of a detent is a torque
  *          edge, and rendering that edge 1 ms late is the difference between
  *          a click and a mush. The law is a handful of multiplies, so the
  *          cost of running it 30x more often is small.
  *
  * ---------------------------------------------------------------------------
  * Composition
  * ---------------------------------------------------------------------------
  *          The terms below are SUMMED, not selected. A dial is rarely one
  *          effect - a good detented knob with limits is detents plus endstops
  *          plus a little damping, and a spring-return throttle is spring plus
  *          damping plus friction. Setting a gain to zero removes that term,
  *          so one law covers every preset without a mode switch inside a
  *          mode switch.
  *
  *            detents    periodic wells, `detent_count` per revolution
  *            spring     pull toward a fixed angle
  *            endstops   free inside a range, stiff wall outside
  *            damping    opposes velocity; viscous, the feel of stirring oil
  *            friction   opposes direction of travel; dry, the feel of a brake
  *
  * ---------------------------------------------------------------------------
  * What it will feel like on THIS motor
  * ---------------------------------------------------------------------------
  *          20 pole pairs means real cogging - the motor has its own detents
  *          at 20 per revolution whether you render any or not, and they are
  *          not compensated here. Expect a background roughness that light
  *          settings will not mask. Rendering detents at a multiple of 20 will
  *          partly hide it by lining up; anything else will beat against it.
  ******************************************************************************
  */
#ifndef HAPTIC_H
#define HAPTIC_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* Encoder counts per mechanical revolution; must match POS_COUNTS_PER_REV. */
#define HAPTIC_COUNTS_PER_REV   32768

/* Well shape.
 *
 * SINE is the textbook detent and is smoother in the middle of a well, but
 * its torque falls to ZERO at the boundary between wells - which is precisely
 * where the shaft needs the most help. Measured on this motor: parked 0.3 deg
 * inside a boundary, a 900 mA sine detent produced 113 mA, lost to ~250 mA of
 * stiction, and the dial simply sat there. It left a dead band roughly 9% of
 * each detent wide where nothing happened.
 *
 * RAMP is a sawtooth: torque grows linearly from zero at the centre to full
 * strength at the boundary, then reverses. Maximum force exactly where the
 * dead band was, no dead band at all, and the discontinuity at the boundary
 * IS the click. It is also closer to what a real detent does mechanically -
 * a ball riding up a ramp and dropping over the crest. */
#define HAPTIC_SHAPE_SINE   0
#define HAPTIC_SHAPE_RAMP   1

/* Defaults describe a plain detented knob: 24 clicks per turn, ramp wells
 * strong enough to beat this motor's stiction, a little damping so it settles
 * instead of ringing, and no limits. 24 rather than 20 is deliberate - see the
 * cogging note at the top; try both and feel the difference. */
#define HAPTIC_DETENTS_DEFAULT      24
#define HAPTIC_SHAPE_DEFAULT        HAPTIC_SHAPE_RAMP
#define HAPTIC_DETENT_MA_DEFAULT    500
#define HAPTIC_DAMPING_DEFAULT      40     /* mA per deg/s, x100 -> 0.40    */
#define HAPTIC_TORQUE_MAX_DEFAULT   900

/* All 32-bit and contiguous so a debugger can read or write the whole block
 * in one transaction. Commands first, then telemetry. */
typedef struct {
  /* ---- parameters: write these ----------------------------- offset ---- */
  int32_t  detent_count;      /*  0  wells per revolution, 0 = off         */
  int32_t  detent_ma;         /*  4  peak detent torque, mA                */
  int32_t  detent_shape;      /*  8  HAPTIC_SHAPE_*                        */
  int32_t  spring_ma_per_deg_x100; /* 12  pull toward spring_center, 0=off */
  int32_t  spring_center_deg_x10;  /* 16                                   */
  int32_t  damping_ma_per_dps_x100;/* 20  viscous, opposes velocity        */
  int32_t  friction_ma;       /* 24  dry, opposes direction of travel      */
  int32_t  endstop_lo_deg_x10;/* 28  lower limit                           */
  int32_t  endstop_hi_deg_x10;/* 32  upper limit                           */
  int32_t  endstop_ma_per_deg_x100; /* 36  wall stiffness, 0 = no limits   */
  int32_t  torque_max_ma;     /* 40  clamp on the summed result            */

  /* ---- telemetry: read only -------------------------------------------- */
  int32_t  detent_index;      /* 44  which click the dial is on, signed    */
  int32_t  torque_ma;         /* 48  what was commanded                    */
  int32_t  in_endstop;        /* 52  -1 below range, +1 above, 0 inside    */
  uint32_t updates;           /* 56                                        */
} HapticState_t;

void Haptic_Init(HapticState_t *h);

/* Torque for the present shaft state, in amps, ready for the current loop.
 * pos_counts is the multi-turn encoder count and vel_dps the filtered
 * velocity in mechanical degrees per second. */
float Haptic_Torque(HapticState_t *h, int32_t pos_counts, float vel_dps);

#ifdef __cplusplus
}
#endif

#endif /* HAPTIC_H */
