/**
  ******************************************************************************
  * @file    position.h
  * @brief   Outer position loop, cascaded on top of the FOC current loop.
  *
  *          Structure is the usual three-deep cascade collapsed into two:
  *
  *              cmd -> [ramp] -> target -> [PID] -> iq_ref -> [FOC] -> duty
  *
  *          There is no separate velocity loop; velocity appears only as the
  *          derivative term, taken on the MEASUREMENT rather than the error so
  *          a step in the target cannot produce a derivative kick.
  *
  *          Rate separation is what makes the cascade stable: the current loop
  *          is closed at 20 kHz with ~1 kHz of bandwidth, so the position loop
  *          runs at POS_RATE_HZ (1 kHz) and is tuned for well under that. It
  *          is stepped from the same HRTIM ISR, decimated by POS_DECIM, so its
  *          sample interval is exact and not subject to main-loop jitter.
  *
  *          Angle unwrapping happens at the FULL ISR rate, not the decimated
  *          one: the 15-bit encoder only stays unambiguous while the rotor
  *          moves less than half a turn between samples, and doing it at
  *          20 kHz buys 20x the headroom for free (it is integer-only).
  *
  *          SWD is the only console this board has and OpenOCD cannot format
  *          floats, so every command and every readout below is an integer in
  *          a fixed scale. The floats are internal.
  ******************************************************************************
  */
#ifndef POSITION_H
#define POSITION_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* Encoder counts per mechanical revolution (A1333, 15-bit). */
#define POS_COUNTS_PER_REV   32768

/* Outer-loop rate. POS_DECIM must be 20000 / POS_RATE_HZ. */
#define POS_RATE_HZ          1000.0f
#define POS_DECIM            20U

/* Velocity is a first difference of a quantised position, so it is noisy:
 * one count over one 1 ms tick is already 0.19 rad/s. This EMA puts a ~35 Hz
 * corner on it, well above the loop bandwidth being aimed for and far enough
 * below the sample rate to actually remove the quantisation hash that kd
 * would otherwise amplify straight into the current command.
 *
 * Runtime-tunable via vel_filt_x1000, because how much is enough depends on
 * the encoder and the load. Lowering it costs phase margin in the damping
 * term, so it is not a free knob - measure overshoot after changing it.
 *
 * 0.05 (~6 Hz) rather than the 0.20 this started at, because kd multiplying
 * velocity noise is the single largest source of audible current dither at
 * standstill. Measured holding position, with the output filter off so this
 * term is isolated:
 *
 *     alpha    velocity noise    commanded current
 *     0.20        36 deg/s          106 mA pp
 *     0.10        18 deg/s           80 mA pp
 *     0.05         8 deg/s           52 mA pp
 *     0.03         4 deg/s           47 mA pp   <- diminishing
 *
 * 0.05 is where the returns stop. Going further is not free: at 0.03 the
 * added phase lag showed up as a 360 degree move taking 1.70 s to settle
 * instead of 0.95, and arriving with undershoot. At 0.05 the move times are
 * unchanged (0.35 s for 90 deg, 0.95 s for 360) and overshoot stays under
 * 0.8 deg, so the damping term is still doing its job. */
#define POS_VEL_ALPHA        0.05f

/* Corner frequency of the low-pass on the OUTPUT current command, in Hz.
 * 0 disables it.
 *
 * The PID runs at POS_RATE_HZ but the current loop consumes its output at
 * 20 kHz, so without this the FOC sees a 1 kHz staircase - a zero-order hold,
 * with all the harmonics that implies, landing right in the audible band.
 *
 * At standstill the steps are not small. Measured holding position, the
 * commanded iq dithered 127 mA peak-to-peak; with kd forced to zero it fell
 * to 38 mA, so ~89 mA of it was the derivative term differentiating encoder
 * quantisation. 40 deg/s of velocity noise x kd 0.12 = 84 mA, which is that
 * number almost exactly. The motor turns the result into an audible whistle.
 *
 * The corner has to be chosen against the 1 kHz staircase, not against the
 * position loop's bandwidth. 300 Hz was tried first on the reasoning that six
 * times the loop bandwidth was safely out of the way - and it did almost
 * nothing, 96 mA pp down to 91, because one pole at 300 Hz is only ~10 dB at
 * 1 kHz. Measured with the velocity filter held at 0.05:
 *
 *     corner      commanded current
 *     off             64 mA pp
 *     150 Hz          31 mA pp
 *      80 Hz          28 mA pp   <- diminishing, and 80 Hz starts costing
 *                                   settle time on long moves
 *
 * 150 Hz is still three times the loop bandwidth, which the measured move
 * times bear out. This does NOT remove the ~148 mA pp the current loop
 * already has on its own from ADC noise and PWM ripple; that floor is not the
 * outer loop's to fix. */
#define POS_OUT_LPF_HZ       150

/* How far the ramped target may run ahead of the rotor before the ramp holds.
 *
 * Without this, commanding a distant target into a stalled or heavily loaded
 * motor lets the target keep sliding away at vel_max while the rotor stays
 * put. The error - and with it the stored "catch up" energy - grows without
 * bound, and the instant the obstruction clears the motor slams to the target
 * at the current limit. Holding the ramp whenever the rotor is more than this
 * far behind turns the profile into something the machine can actually track.
 * 0.6 rad is about 34 mechanical degrees. */
#define POS_TRACK_WINDOW_RAD 0.6f

/* Defaults. These are starting points sized from what the motor needs, not
 * measured optima - expect to tune kp/kd on the bench.
 *
 * These were measured on the bench, not derived. Eight consecutive 90 degree
 * steps around a full revolution settle in 0.25 s - which IS the slew time, so
 * the loop is arriving with nothing left to correct - to a final error of 0.0
 * to 0.1 deg, drawing 200-270 mA of peak current.
 *
 *   kp 12.0 A/rad     -> the dominant term, and what sets stiffness. Lower kp
 *                        is not gentler here, it is worse: at 4.0 the loop
 *                        approached so weakly that it parked in the stiction
 *                        deadband and left the integrator to do the last
 *                        degree. 12.0 both settles faster AND draws LESS peak
 *                        current (270 mA vs 454 mA) because it stops fighting
 *                        its own residual.
 *   ki 30.0 A/(rad*s) -> walks out the static error left by cogging and
 *                        friction, which on a 20-pole-pair motor dominates.
 *                        This looks aggressive against the 2.0 that an
 *                        untuned loop wanted, and it is only safe because the
 *                        integrator is frozen during the slew (see
 *                        position.c) - without that freeze it charges all the
 *                        way along the move and flings the rotor past the
 *                        target. With the freeze, 60.0 still showed no
 *                        hunting; 30.0 is that result with margin.
 *                        Undersizing it is expensive: at 6.0 the integrator
 *                        climbs at only ~50 mA/s, so a rotor parked half a
 *                        degree short on a stiff spot took 2.8 s to break
 *                        free - on a 10 degree move whose slew was 14 ms.
 *   kd 0.12 A/(rad/s) -> damping, scaled with kp to hold the same ratio. */
#define POS_KP_DEFAULT       12.0f
#define POS_KI_DEFAULT       30.0f
#define POS_KD_DEFAULT       0.12f
/* Ramp rate and current clamp.
 *
 * 360 deg/s is a tracking limit, not a torque limit - a 90 degree step peaks
 * at ~270 mA against a 1.0 A clamp, so there is plenty of current spare. What
 * bounds it is the FOC modulation ceiling: FOC_VMAX_DEFAULT of 0.25 saturates
 * somewhere around 170 rpm (~1000 deg/s), and the position loop must stay
 * clear of the speed where the current loop can no longer push against
 * back-EMF. 720 deg/s does run, but it arrives with enough momentum that
 * settling costs more than the faster slew saves.
 *
 * An earlier note here claimed 180 deg/s caused a breakaway overshoot. That
 * was mis-attributed: the cause was kp 4.0 approaching too weakly and the
 * integrator winding up to ~750 mA against stiction. It is speed-independent
 * and it is fixed by the gains above. */
#define POS_VEL_MAX_DPS      360     /* cruise speed, mech deg/s     */
#define POS_IQ_MAX_MA        1000    /* output clamp                 */

/* Acceleration and jerk limits for the motion profile.
 *
 * A constant-velocity ramp is a STEP in velocity at both ends of a move. The
 * start demands infinite acceleration, so the rotor simply lags until kp*err
 * has built enough torque to drag it along; the end is worse, because the
 * profile stops dead while the rotor is still travelling and the loop has to
 * catch and reverse it. That is what "moves linearly, stops suddenly, then
 * damps in" looks like from outside - and none of it is the tuning, it is the
 * profile asking for motion no physical shaft can produce.
 *
 * Limiting acceleration turns the velocity ramp into a trapezoid, so the
 * profile arrives with velocity already at zero and there is nothing left to
 * catch. Limiting jerk rounds the corners of that trapezoid into an S-curve,
 * which is what removes the last of the visible jolt at the transitions.
 *
 * 3600 deg/s^2 reaches the 360 deg/s cruise in 0.1 s, and 36000 deg/s^3
 * reaches full acceleration in 0.1 s. A 90 degree move then spends 18 deg
 * accelerating, 54 cruising and 18 braking. Set jerk to 0 for a plain
 * trapezoid. */
#define POS_ACCEL_MAX_DPS2   3600
#define POS_JERK_MAX_DPS3    36000

/* Below these the profile is considered arrived and is snapped exactly onto
 * the command, so it cannot creep on floating-point residue. */
#define POS_ARRIVE_RAD       0.0005f   /* ~0.03 deg  */
#define POS_ARRIVE_VEL       0.02f     /* ~1 deg/s   */

/* All 32-bit fields are laid out first and contiguously, commands then
 * telemetry, so a debugger can read the whole block in one transaction and
 * address any field as a fixed byte offset from the struct. Do not reorder
 * without updating tools/pos_dash.sh. */
typedef struct {
  /* ---- commands: write these over SWD ---------------------- offset ---- */
  uint32_t enabled;        /*  0  1 = position loop drives iq_ref          */
  int32_t  cmd_deg_x10;    /*  4  target, tenths of a mech degree,
                                  multi-turn (3600 = one full turn)        */
  int32_t  kp_x1000;       /*  8  A/rad         x1000                      */
  int32_t  ki_x1000;       /* 12  A/(rad*s)     x1000                      */
  int32_t  kd_x1000;       /* 16  A/(rad/s)     x1000                      */
  int32_t  vel_max_dps;    /* 20  profile cruise speed, mech deg/s         */
  int32_t  accel_max_dps2; /* 24  profile acceleration limit               */
  int32_t  jerk_max_dps3;  /* 28  profile jerk limit, 0 = plain trapezoid  */
  int32_t  iq_max_ma;      /* 32  output clamp, mA                         */
  uint32_t zero_here;      /* 36  write 1: call the present rotor position
                                  zero and hold it                         */

  /* ---- telemetry: read only ---------------------------------------------- */
  int32_t  pos_deg_x10;    /* 40  measured, multi-turn                     */
  int32_t  target_deg_x10; /* 44  profile target actually being chased     */
  int32_t  err_deg_x10;    /* 48  target - measured                        */
  int32_t  vel_dps;        /* 52  measured, filtered, mech deg/s           */
  int32_t  target_vel_dps; /* 56  profile velocity - the trapezoid itself  */
  int32_t  target_acc_dps2;/* 60  profile acceleration                     */
  int32_t  iq_out_ma;      /* 64  what was handed to the current loop      */
  int32_t  integ_ma;       /* 68  integrator contribution alone            */
  int32_t  turns;          /* 72  completed revolutions, signed            */
  uint32_t updates;        /* 76  outer-loop steps                         */

  /* ---- appended later; kept at the end so every offset above stays put --- */
  int32_t  out_lpf_hz;     /* 80  output smoothing corner, Hz. 0 = off     */
  int32_t  vel_filt_x1000; /* 84  velocity EMA alpha x1000                 */
  int32_t  iq_raw_ma;      /* 88  PID output BEFORE smoothing              */

  /* ---- internal ---------------------------------------------------------- */
  int32_t  pos_counts;     /* unwrapped multi-turn count                   */
  uint16_t last_raw;       /* previous 15-bit reading                      */
  uint8_t  seeded;         /* 0 until last_raw is valid                    */
  uint8_t  was_enabled;    /* edge detect for engage                       */
  uint32_t decim;

  float    pos_rad;
  float    target_rad;
  float    target_vel;     /* rad/s,   profile state */
  float    target_acc;     /* rad/s^2, profile state */
  float    vel_rads;
  float    integ;          /* A */
  float    iq_raw;         /* A, PID output           */
  float    iq_out;         /* A, smoothed, what ships */
} PosState_t;

void Position_Init(PosState_t *p);

/* One ISR tick. Unwraps the encoder every call and runs the PID every
 * POS_DECIM calls; returns the iq command in amps. The return value is only
 * meaningful while p->enabled is set - when it is clear this still tracks the
 * angle (so engaging is bumpless) but returns 0 and holds the PID reset. */
float Position_Step(PosState_t *p, uint16_t enc_raw);

/* Define the present rotor position as zero and hold there. */
void Position_ZeroHere(PosState_t *p);

#ifdef __cplusplus
}
#endif

#endif /* POSITION_H */
