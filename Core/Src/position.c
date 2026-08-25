/**
  ******************************************************************************
  * @file    position.c
  * @brief   Outer position loop, cascaded on top of the FOC current loop.
  ******************************************************************************
  */

#include "position.h"
#include <math.h>
#include "fastmath.h"

#define TWO_PI            6.28318530718f
#define RAD_PER_COUNT     (TWO_PI / (float)POS_COUNTS_PER_REV)
#define DEG_X10_PER_RAD   (1800.0f / 3.14159265359f)
#define RAD_PER_DEG_X10   (3.14159265359f / 1800.0f)
#define DEG_TO_RAD        (3.14159265359f / 180.0f)
#define POS_DT            (1.0f / POS_RATE_HZ)

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
  p->out_lpf_hz     = POS_OUT_LPF_HZ;
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
   * samples, which at 20 kHz is 300000 rpm. */
  int32_t d = (int32_t)raw - (int32_t)p->last_raw;
  if (d >  (POS_COUNTS_PER_REV / 2)) { d -= POS_COUNTS_PER_REV; }
  if (d < -(POS_COUNTS_PER_REV / 2)) { d += POS_COUNTS_PER_REV; }
  p->last_raw    = raw;
  p->pos_counts += d;

  if (p->zero_here != 0U)
  {
    p->zero_here = 0U;
    Position_ZeroHere(p);
  }

  /* ---- output smoothing, EVERY tick ------------------------------------ *
   *
   * The PID only refreshes at POS_RATE_HZ, but the current loop consumes this
   * at the full 20 kHz, so handing it p->iq_raw directly would feed it a
   * 1 kHz staircase. Running the filter out here - on every tick, not the
   * decimated one - is what turns those steps into a continuous command.
   *
   * The alpha is the small-angle form of 1 - exp(-2*pi*f/fs); at 300 Hz of
   * 20 kHz that is 0.094, where the approximation is still good. */
  {
    float a = 1.0f;
    if (p->out_lpf_hz > 0)
    {
      a = (6.28318530718f * (float)p->out_lpf_hz) * (1.0f / 20000.0f);
      if (a > 1.0f) { a = 1.0f; }
    }
    p->iq_out += a * (p->iq_raw - p->iq_out);
  }

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

  if (p->enabled == 0U)
  {
    /* Disabled: keep tracking the angle so an engage is bumpless, but hold
     * the PID in reset and command nothing. */
    p->integ      = 0.0f;
    p->iq_raw     = 0.0f;
    p->target_rad = p->pos_rad;
    p->target_vel = 0.0f;
    p->target_acc = 0.0f;
    p->was_enabled = 0U;
  }
  else
  {
    if (p->was_enabled == 0U)
    {
      /* Servo-on. Take the target from where the rotor IS, not from whatever
       * cmd_deg_x10 happens to hold - at boot that is 0, and a fresh enable
       * would otherwise fling the rotor to mechanical zero at vel_max. The
       * operator writes the target AFTER enabling. */
      p->was_enabled = 1U;
      p->target_rad  = p->pos_rad;
      p->target_vel  = 0.0f;
      p->target_acc  = 0.0f;
      p->cmd_deg_x10 = (int32_t)(p->pos_rad * DEG_X10_PER_RAD);
      p->integ       = 0.0f;
    }

    float kp     = (float)p->kp_x1000 * 0.001f;
    float ki     = (float)p->ki_x1000 * 0.001f;
    float kd     = (float)p->kd_x1000 * 0.001f;
    float iq_max = (float)p->iq_max_ma * 0.001f;

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
  p->iq_out_ma      = (int32_t)(p->iq_out * 1000.0f);
  p->integ_ma       = (int32_t)(p->integ * 1000.0f);
  p->turns          = p->pos_counts / POS_COUNTS_PER_REV;
  p->updates++;

  return p->iq_out;
}
