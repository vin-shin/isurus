/**
  ******************************************************************************
  * @file    position.c
  * @brief   Outer position loop, cascaded on top of the FOC current loop.
  ******************************************************************************
  */

#include "position.h"
#include "haptic.h"
#include "drive.h"
#include "foc.h"

extern volatile uint32_t g_enc_glitch;
extern volatile int32_t  g_enc_glitch_max;

/* Read-only here. The plausibility check needs to know whether the current
 * loop is actually consuming the angle, because that is what decides whether
 * a corrupted frame can do any harm. */
extern volatile FocState_t g_foc;
#include <math.h>
#include "fastmath.h"

#define TWO_PI            6.28318530718f
#define RAD_PER_COUNT     (TWO_PI / (float)POS_COUNTS_PER_REV)
#define DEG_X10_PER_RAD   (1800.0f / 3.14159265359f)
#define RAD_PER_DEG_X10   (3.14159265359f / 1800.0f)
#define DEG_TO_RAD        (3.14159265359f / 180.0f)
#define POS_DT            (1.0f / POS_RATE_HZ)

/* Set by main.c. The haptic parameters live in their own object so this
 * module's struct layout - which the tools address by byte offset - does not
 * have to grow every time a new feel is added. */
void *g_haptic_ptr = 0;

/* Rebuild the output filter's coefficient from out_lpf_hz.
 *
 * Split out from the filter itself so it runs when the corner CHANGES rather
 * than once per switching period. Callers are Position_Init and the decimated
 * block, immediately after out_lpf_hz is clamped - keeping the two together is
 * what guarantees the cached alpha can never reflect an out-of-range corner. */
static void Position_SetOutLpf(PosState_t *p)
{
  float a = 1.0f;
  if (p->out_lpf_hz > 0)
  {
    a = (TWO_PI * (float)p->out_lpf_hz) * (1.0f / (float)PWM_FREQ_HZ);
    if (a > 1.0f) { a = 1.0f; }
  }
  p->out_lpf_alpha = a;
}

static float clampf(float v, float lim)
{
  if (v >  lim) { return  lim; }
  if (v < -lim) { return -lim; }
  return v;
}

void Position_Init(PosState_t *p)
{
  p->enabled      = 0U;
  p->cmd_deg_x10  = 0;
  p->kp_x1000     = (int32_t)(POS_KP_DEFAULT * 1000.0f);
  p->ki_x1000     = (int32_t)(POS_KI_DEFAULT * 1000.0f);
  p->kd_x1000     = (int32_t)(POS_KD_DEFAULT * 1000.0f);
  p->vel_max_dps  = POS_VEL_MAX_DPS;
  p->accel_max_dps2 = POS_ACCEL_MAX_DPS2;
  p->jerk_max_dps3  = POS_JERK_MAX_DPS3;
  p->iq_max_ma    = POS_IQ_MAX_MA;
  p->mode           = MOTION_MODE_POSITION;
  p->torque_cmd_ma  = 0;
  p->vel_cmd_dps    = 0;
  p->vkp_x1000      = (int32_t)(POS_VKP_DEFAULT * 1000.0f);
  p->vki_x1000      = (int32_t)(POS_VKI_DEFAULT * 1000.0f);
  p->vel_ref_dps    = 0;
  p->vel_integ_ma   = 0;
  p->vel_ref        = 0.0f;
  p->vel_integ      = 0.0f;
  p->last_mode      = MOTION_MODE_POSITION;
  p->out_lpf_hz     = POS_OUT_LPF_HZ;
  Position_SetOutLpf(p);   /* seed the cached alpha before the loop can run:
                            * the decimated block that refreshes it is a whole
                            * POS_DECIM away, and an alpha of 0 until then
                            * would hold iq_out at zero */
  p->vel_filt_x1000 = (int32_t)(POS_VEL_ALPHA * 1000.0f);
  p->zero_here    = 0U;

  p->pos_counts = 0;
  p->last_raw   = 0U;
  p->seeded     = 0U;
  p->was_enabled = 0U;
  p->decim      = 0U;

  p->pos_rad    = 0.0f;
  p->target_rad = 0.0f;
  p->target_vel = 0.0f;
  p->target_acc = 0.0f;
  p->vel_rads   = 0.0f;
  p->integ      = 0.0f;
  p->iq_raw     = 0.0f;
  p->iq_out     = 0.0f;
  p->updates    = 0U;

  p->pos_deg_x10 = 0;
  p->target_deg_x10 = 0;
  p->err_deg_x10 = 0;
  p->vel_dps     = 0;
  p->target_vel_dps = 0;
  p->target_acc_dps2 = 0;
  p->iq_out_ma   = 0;
  p->iq_raw_ma   = 0;
  p->integ_ma    = 0;
  p->turns       = 0;
  p->clamped     = 0U;
}

void Position_ZeroHere(PosState_t *p)
{
  p->pos_counts  = 0;
  p->pos_rad     = 0.0f;
  p->target_rad  = 0.0f;
  p->target_vel  = 0.0f;
  p->target_acc  = 0.0f;
  p->cmd_deg_x10 = 0;
  p->integ       = 0.0f;
  p->vel_rads    = 0.0f;
}

float Position_Step(PosState_t *p, uint16_t enc_raw)
{
  uint16_t raw = (uint16_t)(enc_raw & 0x7FFFU);

  /* ---- unwrap, every ISR tick ---------------------------------------- */
  if (p->seeded == 0U)
  {
    p->last_raw = raw;
    p->seeded   = 1U;
  }

  /* Shortest-path difference. Sign-extending a 15-bit delta into
   * [-16384, +16383] is what makes this a *relative* measurement, so it keeps
   * counting through the wrap at 32767 -> 0 instead of reporting a full turn
   * backwards. Valid as long as the rotor moves less than half a turn between
   * samples: at 30 kHz that ceiling is 0.5 * 30000 = 15000 rev/s, or 900000
   * rpm, so it is not a constraint any real shaft can reach. */
  int32_t d = (int32_t)raw - (int32_t)p->last_raw;
  if (d >  (POS_COUNTS_PER_REV / 2)) { d -= POS_COUNTS_PER_REV; }
  if (d < -(POS_COUNTS_PER_REV / 2)) { d += POS_COUNTS_PER_REV; }
  p->last_raw    = raw;
  p->pos_counts += d;

  /* Kinematic plausibility. A jump larger than POS_ENC_DMAX_COUNTS is not
   * something the rotor can do in one tick, so it is a corrupted frame.
   *
   * This LATCHES a fault rather than filtering, and only while the current
   * loop is actually using the angle. Silently rejecting the sample would be
   * worse than useless - the loop would keep running on the previous angle
   * with no indication, which is precisely the failure the encoder
   * substitution counter exists to expose. And faulting while the bridge is
   * down would punish someone turning the shaft by hand for no benefit: a bad
   * angle only does damage when it can steer current. */
  {
    int32_t ad = (d < 0) ? -d : d;
    if (ad > POS_ENC_DMAX_COUNTS)
    {
      g_enc_glitch++;
      if (ad > g_enc_glitch_max) { g_enc_glitch_max = ad; }

      if (g_foc.enabled != 0U)
      {
        Drive_Fault(DRIVE_FAULT_ENCODER);
      }
    }
  }

  if (p->zero_here != 0U)
  {
    p->zero_here = 0U;
    Position_ZeroHere(p);
  }

  /* ---- haptics, EVERY tick --------------------------------------------- *
   *
   * Evaluated here rather than in the decimated block below because it is the
   * one mode whose quality depends on update rate: a detent is a torque edge
   * and the hand feels its timing directly. It is cheap enough to afford -
   * a few multiplies and a polynomial sine, no loop state. */
  if ((p->enabled != 0U) && (p->mode == MOTION_MODE_HAPTIC))
  {
    p->iq_raw = Haptic_Torque((HapticState_t *)g_haptic_ptr, p->pos_counts,
                              (float)p->vel_dps);
  }

  /* ---- output smoothing, EVERY tick ------------------------------------ *
   *
   * The PID only refreshes at POS_RATE_HZ, but the current loop consumes this
   * at the full PWM_FREQ_HZ, so handing it p->iq_raw directly would feed it a
   * 1 kHz staircase. Running the filter out here - on every tick, not the
   * decimated one - is what turns those steps into a continuous command.
   *
   * The alpha is the small-angle form of 1 - exp(-2*pi*f/fs), which stays good
   * while the corner is well below the sample rate - it is 0.031 for 150 Hz at
   * 30 kHz. fs is the PWM rate because this runs every ISR tick.
   *
   * The FILTER runs every tick; its coefficient does not need to. Alpha is a
   * function of out_lpf_hz alone, so rebuilding it here spent an int-to-float
   * conversion and two multiplies per period recomputing a constant. It is
   * cached by Position_SetOutLpf() instead, called from Init and from the
   * decimated block right after out_lpf_hz is clamped - a corner change lands
   * within one POS_DECIM period, which is 1 ms and inaudible in a 150 Hz
   * filter. */
  p->iq_out += p->out_lpf_alpha * (p->iq_raw - p->iq_out);

  /* ---- PID, decimated ------------------------------------------------- */
  if (++p->decim < POS_DECIM)
  {
    return p->iq_out;
  }
  p->decim = 0U;

  float prev_rad = p->pos_rad;
  p->pos_rad = (float)p->pos_counts * RAD_PER_COUNT;

  /* Derivative of the measurement, low-passed. Differentiating position
   * rather than carrying a separate velocity estimator keeps the two in exact
   * agreement, which matters because kd is what damps this loop. */
  float raw_vel = (p->pos_rad - prev_rad) * POS_RATE_HZ;
  float valpha  = (float)p->vel_filt_x1000 * 0.001f;
  if (valpha <= 0.0f) { valpha = 0.001f; }
  if (valpha >  1.0f) { valpha = 1.0f;   }
  p->vel_rads += valpha * (raw_vel - p->vel_rads);

  /* Saturate every command to what the hardware can actually do, before any
   * of it is used. Done here rather than in the transports so that CAN, the
   * SWD tools and a debugger poking memory by hand are all bounded by the
   * same code - a limit that only one path enforces is not a limit. */
  {
    uint32_t *hits = (uint32_t *)&p->clamped;
    p->iq_max_ma      = Lim_Clamp(p->iq_max_ma,      0, LIM_IQ_MAX_MA,     hits);
    p->torque_cmd_ma  = Lim_Clamp(p->torque_cmd_ma,  -p->iq_max_ma, p->iq_max_ma, hits);
    p->vel_cmd_dps    = Lim_Clamp(p->vel_cmd_dps,    -LIM_VEL_MAX_DPS, LIM_VEL_MAX_DPS, hits);
    p->vel_max_dps    = Lim_Clamp(p->vel_max_dps,    1, LIM_VEL_MAX_DPS,   hits);
    p->cmd_deg_x10    = Lim_Clamp(p->cmd_deg_x10,    -LIM_POS_MAX_DEG_X10, LIM_POS_MAX_DEG_X10, hits);
    p->accel_max_dps2 = Lim_Clamp(p->accel_max_dps2, LIM_ACCEL_MIN_DPS2, LIM_ACCEL_MAX_DPS2, hits);
    p->jerk_max_dps3  = Lim_Clamp(p->jerk_max_dps3,  0, LIM_JERK_MAX_DPS3, hits);
    p->kp_x1000       = Lim_Clamp(p->kp_x1000,       0, LIM_KP_X1000_MAX,  hits);
    p->ki_x1000       = Lim_Clamp(p->ki_x1000,       0, LIM_KI_X1000_MAX,  hits);
    p->kd_x1000       = Lim_Clamp(p->kd_x1000,       0, LIM_KD_X1000_MAX,  hits);
    p->vkp_x1000      = Lim_Clamp(p->vkp_x1000,      0, LIM_VKP_X1000_MAX, hits);
    p->vki_x1000      = Lim_Clamp(p->vki_x1000,      0, LIM_VKI_X1000_MAX, hits);
    p->out_lpf_hz     = Lim_Clamp(p->out_lpf_hz,     0, (int32_t)(PWM_FREQ_HZ / 4U), hits);
    p->vel_filt_x1000 = Lim_Clamp(p->vel_filt_x1000, 1, 1000,              hits);
  }

  /* out_lpf_hz is writable over SWD and CAN at any moment, so the cached
   * coefficient is refreshed here - after the clamp above, never before it. */
  Position_SetOutLpf(p);

  float iq_max_a = (float)p->iq_max_ma * 0.001f;

  /* A mode change must not kick the motor. Clear both integrators and re-seed
   * the position target from where the rotor actually is, so whichever loop
   * takes over starts from the present state rather than from whatever the
   * previous mode left behind. */
  if (p->mode != p->last_mode)
  {
    p->last_mode  = p->mode;
    p->integ      = 0.0f;
    p->vel_integ  = 0.0f;

    /* Hand over at the state the rotor is ACTUALLY in, so a mode change while
     * moving is continuous rather than a step. Seeding these to zero would ask
     * a spinning rotor to be at rest, and the new loop would answer that with
     * a violent correction. */
    p->target_rad = p->pos_rad;
    p->target_vel = p->vel_rads;
    p->target_acc = 0.0f;
    p->vel_ref    = p->vel_rads;

    /* cmd_deg_x10 is deliberately NOT touched here.
     *
     * It is on the engage path below, where a stale target from boot could
     * otherwise fling the rotor to mechanical zero. But a mode change is the
     * operator explicitly asking for position control, and they will have
     * written the setpoint first - overwriting it here silently discarded the
     * command, so "p 0" after a velocity move held the current position
     * instead of returning to zero. */
  }

  /* Servo-on, tracked on the `enabled` edge and NOTHING else.
   *
   * Seeding the position command from the present rotor angle is a safety
   * measure for exactly one situation: cmd_deg_x10 is 0 at boot, so enabling
   * a servo whose rotor is elsewhere would fling it to mechanical zero. It
   * must therefore fire once per enable - not once per mode change.
   *
   * Keying it off the position branch instead was wrong twice over. The
   * torque and velocity branches had to clear the flag to get seeding on the
   * way back, and the idle branch cleared it too - so the first entry into
   * position mode after either one silently overwrote whatever setpoint had
   * just been written, and "p 0" held station instead of returning to zero. */
  if (p->enabled == 0U)
  {
    p->was_enabled = 0U;
  }
  else if (p->was_enabled == 0U)
  {
    p->was_enabled = 1U;
    p->target_rad  = p->pos_rad;
    p->target_vel  = 0.0f;
    p->target_acc  = 0.0f;
    p->vel_ref     = 0.0f;
    p->cmd_deg_x10 = (int32_t)(p->pos_rad * DEG_X10_PER_RAD);
    p->integ       = 0.0f;
    p->vel_integ   = 0.0f;
  }

  if ((p->enabled == 0U) || (p->mode == MOTION_MODE_IDLE))
  {
    /* Off, or idling: keep tracking the angle so an engage is bumpless, but
     * hold every integrator in reset and command nothing. Note this does NOT
     * clear was_enabled - idle is a mode, not a disable. */
    p->integ      = 0.0f;
    p->vel_integ  = 0.0f;
    p->vel_ref    = 0.0f;
    p->iq_raw     = 0.0f;
    p->target_rad = p->pos_rad;
    p->target_vel = 0.0f;
    p->target_acc = 0.0f;
  }
  else if (p->mode == MOTION_MODE_HAPTIC)
  {
    /* Already computed at full rate above; nothing to do at this rate except
     * hold the position loop's state ready for a switch back. */
    p->integ      = 0.0f;
    p->vel_integ  = 0.0f;
    p->target_rad = p->pos_rad;
    p->target_vel = p->vel_rads;
    p->target_acc = 0.0f;
  }
  else if (p->mode == MOTION_MODE_TORQUE)
  {
    /* Straight through. Nothing here limits speed - only the back-EMF the
     * current loop eventually cannot push against does. */
    p->iq_raw = clampf((float)p->torque_cmd_ma * 0.001f, iq_max_a);
  }
  else if (p->mode == MOTION_MODE_VELOCITY)
  {
    float vkp  = (float)p->vkp_x1000 * 0.001f;
    float vki  = (float)p->vki_x1000 * 0.001f;
    float vcmd = (float)p->vel_cmd_dps * DEG_TO_RAD;
    float amax = (float)p->accel_max_dps2 * DEG_TO_RAD;
    if (amax <= 0.0f) { amax = 1.0f; }

    /* Ramp the reference rather than stepping it, for the same reason the
     * position mode has a profile: a step demands infinite acceleration and
     * the loop can only answer it by saturating. */
    float dv    = vcmd - p->vel_ref;
    float dvmax = amax * POS_DT;
    if (dv >  dvmax) { dv =  dvmax; }
    if (dv < -dvmax) { dv = -dvmax; }
    p->vel_ref += dv;

    float verr  = p->vel_ref - p->vel_rads;
    float integ = p->vel_integ + vki * verr * POS_DT;
    float prop  = vkp * verr;
    float iq    = prop + integ;

    /* Same back-calculation as the position loop: hold the integrator at
     * exactly what the output can deliver instead of letting it wind past. */
    if (iq >  iq_max_a) { iq =  iq_max_a; integ =  iq_max_a - prop; }
    if (iq < -iq_max_a) { iq = -iq_max_a; integ = -iq_max_a - prop; }

    p->vel_integ = clampf(integ, iq_max_a);
    p->iq_raw    = iq;
  }
  else
  {
    float kp     = (float)p->kp_x1000 * 0.001f;
    float ki     = (float)p->ki_x1000 * 0.001f;
    float kd     = (float)p->kd_x1000 * 0.001f;
    float iq_max = iq_max_a;

    /* ---- motion profile ------------------------------------------------ *
     *
     * Generates target position, velocity and acceleration as an S-curve:
     * jerk-limited ramps into and out of a constant-acceleration phase, which
     * in turn ramps into and out of a constant-velocity cruise. The rotor is
     * then being asked for motion it can actually produce, at both ends.
     *
     * Deceleration is not scheduled ahead of time; it falls out of asking, on
     * every tick, "what is the fastest I could be going and still stop in the
     * distance left?" That is v = sqrt(2*a*d), and clamping the demanded
     * velocity to it produces the braking ramp automatically - no phase state
     * machine, and a mid-move command change is handled by construction. */
    float cmd_rad = (float)p->cmd_deg_x10 * RAD_PER_DEG_X10;
    float vmax    = (float)p->vel_max_dps    * DEG_TO_RAD;
    float amax    = (float)p->accel_max_dps2 * DEG_TO_RAD;
    float jmax    = (float)p->jerk_max_dps3  * DEG_TO_RAD;
    if (amax <= 0.0f) { amax = 1.0f; }

    float lag   = p->target_rad - p->pos_rad;
    float to_go = cmd_rad - p->target_rad;

    /* Hold the profile once the rotor has fallen POS_TRACK_WINDOW_RAD behind,
     * but only in the direction that would make it worse - the target must
     * always stay free to come back toward the rotor. */
    uint8_t blocked =
        ((lag >  POS_TRACK_WINDOW_RAD) && (to_go > 0.0f)) ||
        ((lag < -POS_TRACK_WINDOW_RAD) && (to_go < 0.0f));

    uint8_t slewing = 0U;

    if (blocked)
    {
      /* Something is holding the shaft. Abandon the profile rather than let
       * it keep integrating distance the rotor is not covering; it restarts
       * cleanly from rest once the obstruction clears. */
      p->target_vel = 0.0f;
      p->target_acc = 0.0f;
    }
    else
    {
      float dist  = to_go;
      float adist = fabsf(dist);

      /* Fastest velocity from which the profile can still stop inside `dist`.
       *
       * The obvious answer, sqrt(2*a*d), is the one for a body that can apply
       * full braking instantly. This one cannot: the jerk limit means the
       * deceleration has to be ramped up, and the distance covered during
       * that ramp is not free. Measured with the naive formula, a 90 degree
       * move sailed 15 degrees past the target, reversed at full speed, and
       * undershot to 31 - the profile generator itself oscillating, with the
       * current loop faithfully following it.
       *
       * Solving v^2/(2a) + v*a/(2j) = d for v adds the ramp distance to the
       * budget and gives the corrected limit below. It moves the start of
       * braking from 18 degrees out to 36 for these settings - exactly the
       * factor the jerk ramp costs. With jmax = 0 the correction term
       * vanishes and this collapses back to sqrt(2*a*d), which is the right
       * answer for a plain trapezoid.
       *
       * Below v = a^2/j the true bound is the cube-root branch, v =
       * (d^2*j)^(1/3); this quadratic is conservative there, so the profile
       * brakes a fraction early rather than late. That is the safe direction
       * and it avoids a cube root in the ISR. */
      float c = (jmax > 0.0f) ? ((amax * amax) / (2.0f * jmax)) : 0.0f;
      float v_stop = fm_sqrtf((c * c) + (2.0f * amax * adist * 0.98f)) - c;
      if (v_stop < 0.0f) { v_stop = 0.0f; }
      float v_lim  = (v_stop < vmax) ? v_stop : vmax;
      float v_des  = (dist >= 0.0f) ? v_lim : -v_lim;

      /* Acceleration to chase v_des - and the same "can I still stop?" test
       * as above, applied one derivative up.
       *
       * Clamping this to amax alone is not enough. At the end of a move the
       * profile is braking at full amax when the velocity reaches zero, and
       * jerk-limiting means that acceleration needs a further 0.1 s to unwind.
       * The velocity carries straight through zero while it does, and the
       * profile drives back the way it came: measured as a 90 degree move
       * stopping at 89.5, reversing to 68, then running forward again past
       * 112 - a limit cycle in the generator, which the current loop then
       * follows perfectly into an oscillation.
       *
       * sqrt(2*j*|dv|) is the acceleration from which jerk can still bring
       * the acceleration back to zero within the velocity that is left. It
       * rounds the corners of the trapezoid from the inside, so acceleration,
       * velocity and position all arrive at zero together. */
      float dv     = v_des - p->target_vel;
      float a_need = fabsf(dv) * POS_RATE_HZ;   /* land on v_des this tick */
      float a_mag  = amax;
      if (jmax > 0.0f)
      {
        float a_lim = fm_sqrtf(2.0f * jmax * fabsf(dv));
        if (a_lim < a_mag) { a_mag = a_lim; }
      }
      if (a_need < a_mag) { a_mag = a_need; }
      float a_des = (dv >= 0.0f) ? a_mag : -a_mag;

      if (jmax > 0.0f)
      {
        /* Jerk limit: slew the acceleration itself. This is what rounds the
         * trapezoid's corners into an S-curve. */
        float da   = a_des - p->target_acc;
        float dmax = jmax * POS_DT;
        if (da >  dmax) { da =  dmax; }
        if (da < -dmax) { da = -dmax; }
        p->target_acc += da;
      }
      else
      {
        p->target_acc = a_des;
      }

      p->target_vel += p->target_acc * POS_DT;
      if (p->target_vel >  vmax) { p->target_vel =  vmax; }
      if (p->target_vel < -vmax) { p->target_vel = -vmax; }

      p->target_rad += p->target_vel * POS_DT;

      /* Arrived: snap on exactly, so the profile cannot creep on float
       * residue or leave a tail of micro-velocity behind. */
      if ((fabsf(cmd_rad - p->target_rad) < POS_ARRIVE_RAD) &&
          (fabsf(p->target_vel) < POS_ARRIVE_VEL))
      {
        p->target_rad = cmd_rad;
        p->target_vel = 0.0f;
        p->target_acc = 0.0f;
      }
      else
      {
        slewing = 1U;
      }
    }

    /* ---- PID ------------------------------------------------------------ */
    float err   = p->target_rad - p->pos_rad;
    float prop  = kp * err;

    /* Hold the integrator while the ramp is still advancing.
     *
     * A moving target leaves a standing error even when the loop is tracking
     * perfectly - it is velocity lag, the kp*err the motor needs to keep up,
     * not a disturbance to be corrected. Integrating it charges the
     * integrator all the way along the move, and that stored current is still
     * there at the end: measured 211 mA of it arriving at a 90 degree target,
     * which carried the rotor 1.9 degrees past and then took three seconds to
     * unwind. Freezing during the slew leaves the integrator doing only the
     * job it is there for, which is walking out the static error once the
     * rotor has arrived.
     *
     * A `blocked` ramp is deliberately NOT frozen: there the rotor has fallen
     * behind something real, and the integrator building against it is what
     * breaks stiction. */
    float integ = slewing ? p->integ : (p->integ + ki * err * POS_DT);
    float deriv = -kd * p->vel_rads;

    float iq = prop + integ + deriv;

    if (iq > iq_max)
    {
      /* Back-calculate the integrator to exactly what the output can deliver.
       * Plain clamping of iq alone would let integ keep growing while
       * saturated, and the loop would then refuse to come off the limit until
       * that surplus had been unwound - overshoot on every large move. */
      iq    = iq_max;
      integ = iq_max - prop - deriv;
    }
    else if (iq < -iq_max)
    {
      iq    = -iq_max;
      integ = -iq_max - prop - deriv;
    }

    p->integ  = clampf(integ, iq_max);
    p->iq_raw = iq;
  }

  /* ---- integer mirrors ------------------------------------------------- */
  p->pos_deg_x10    = (int32_t)(p->pos_rad * DEG_X10_PER_RAD);
  p->target_deg_x10 = (int32_t)(p->target_rad * DEG_X10_PER_RAD);
  p->err_deg_x10    = p->target_deg_x10 - p->pos_deg_x10;
  p->vel_dps        = (int32_t)(p->vel_rads * DEG_X10_PER_RAD * 0.1f);
  p->target_vel_dps = (int32_t)(p->target_vel / DEG_TO_RAD);
  p->target_acc_dps2= (int32_t)(p->target_acc / DEG_TO_RAD);
  p->iq_raw_ma      = (int32_t)(p->iq_raw * 1000.0f);
  p->vel_ref_dps    = (int32_t)(p->vel_ref / DEG_TO_RAD);
  p->vel_integ_ma   = (int32_t)(p->vel_integ * 1000.0f);
  p->iq_out_ma      = (int32_t)(p->iq_out * 1000.0f);
  p->integ_ma       = (int32_t)(p->integ * 1000.0f);
  p->turns          = p->pos_counts / POS_COUNTS_PER_REV;
  p->updates++;

  return p->iq_out;
}
