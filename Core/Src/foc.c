/**
  ******************************************************************************
  * @file    foc.c
  * @brief   Field-oriented current control.
  ******************************************************************************
  */

#include "foc.h"
#include "limits.h"   /* LIM_IQ_MAX_MA, for the torque map's clamp */
#include "main.h"
#include "motor_pwm.h"   /* PWM_FREQ_HZ: the rate this loop actually runs at */
#include "fastmath.h"

#define FOC_DT          (1.0f / (float)PWM_FREQ_HZ)

#define ONE_BY_SQRT3    0.57735026919f
#define TWO_BY_SQRT3    1.15470053838f

/* Transport delay, in switching periods, between measuring a current and the
 * voltage computed from it actually reaching the motor.
 *
 * DERIVE this, never write down a number. The usual textbook figure is 1.5
 * periods, and it is wrong here by the ADC's trigger lead. Walk the chain for
 * a current sampled during period n:
 *
 *   1. The ADC is triggered PWM_ADC_LEAD_NS BEFORE the period boundary, not
 *      at it - see motor_pwm.h, where the lead is a fixed 5 us because the
 *      conversion takes a fixed time. So the sample instant is
 *      (n+1)*Ts - PWM_ADC_LEAD_NS, EARLIER than the boundary, and every delay
 *      below is measured from there.
 *   2. The repetition event at (n+1)*Ts runs the control ISR, which reads that
 *      conversion and computes a duty.
 *   3. The compare registers are preloaded and latch on counter reset
 *      (motor_pwm.c). The ISR's write happens after the reset at (n+1)*Ts, so
 *      it latches at (n+2)*Ts - one full period of latency.
 *   4. The duty is therefore applied across period n+2, and the pulse is
 *      centred on PER/2, so the voltage's centroid is at (n+2.5)*Ts. This is
 *      the usual half-period of zero-order hold.
 *
 *   delay = (n+2.5)*Ts - ((n+1)*Ts - lead) = 1.5*Ts + lead
 *
 * At 30 kHz the 5 us lead is 0.15 of a period, so the true figure is 1.65, not
 * 1.5. That 0.15 is worth 1.7 degrees electrical on the EMRAX at 917 Hz -
 * small, but free to get right, and it moves if either the switching frequency
 * or the conversion time changes. Which is exactly why it is computed from
 * both rather than typed in.
 *
 * If the ADC trigger is ever moved to the period event, this becomes 1.5 on
 * its own - but read the warning in motor_pwm.h before doing that, because
 * that move breaks the current loop for an unrelated reason. */
#define FOC_DELAY_PERIODS  (1.5f + ((float)PWM_ADC_LEAD_NS * 1.0e-9f \
                                    * (float)PWM_FREQ_HZ))

/* Electrical angle advance per unit of electrical velocity, in encoder counts
 * per (rad/s). Folds the delay, the loop rate and the counts-per-radian scale
 * into one constant so the ISR pays a single multiply. */
#define FOC_ADV_COUNTS_PER_RADS  ((FOC_DELAY_PERIODS / (float)PWM_FREQ_HZ) \
                                  * ((float)FOC_ENC_COUNTS / 6.28318530718f))

/* CORDIC configuration, written once in FOC_Init and never touched again.
 *
 * FUNC = 0 is cosine; PRECISION = 5 requests 20 iterations, which the hardware
 * retires four per cycle. NRES = 2 makes one operation yield cos AND sin, so
 * the ISR pays for a single CORDIC pass rather than two.
 *
 * 20 iterations was checked against sinf/cosf across the whole circle before
 * being trusted: worst error 1.9e-6, or 0.0001 degrees of dq-frame
 * misalignment. That is four orders of magnitude below the encoder's own
 * 32768-count resolution, so it is not the limiting error anywhere in this
 * loop. Dropping to PRECISION = 4 costs one cycle less and gives 0.0018
 * degrees, which would also be fine - the extra iterations are kept because
 * they are nearly free and Phase 1a will lean on this angle harder. */
#define FOC_CORDIC_CFG  ((0U << CORDIC_CSR_FUNC_Pos)      /* cosine        */ \
                       | (5U << CORDIC_CSR_PRECISION_Pos) /* 20 iterations */ \
                       | CORDIC_CSR_NRES)                 /* cos and sin   */

/* sin/cos of an electrical angle expressed in encoder counts.
 *
 * The CORDIC's angle argument is Q31 spanning [-pi, pi): the value v means
 * v/2^31 * pi radians. elec_counts is already an integer angle - 0..32767 for
 * 0..2*pi - so the whole conversion is one shift:
 *
 *     q31 = counts << 17
 *
 * and it is exact. The 15-bit count walks into the Q31 sign bit at count
 * 16384, which is precisely where the angle passes pi and the CORDIC's range
 * wraps to -pi, so counts above a half turn come out negative and mean the
 * same angle. No float, no fmodf, no wrap test.
 *
 * Do NOT centre the count first, i.e. (counts - 16384) << 17. That form is
 * off by exactly pi and negates BOTH sin and cos. Nothing here would catch it:
 * the inverse Park uses the same pair, so the loop stays self-consistent and
 * perfectly stable - it just runs in a dq frame rotated 180 degrees, where a
 * positive iq_ref produces negative torque. It is the same failure as the
 * elec_offset = 8192 convention bug in foc.h, and just as invisible until a
 * motor turns the wrong way. */
__attribute__((always_inline))
static inline void FOC_SinCos(uint16_t counts, float *sin_out, float *cos_out)
{
  CORDIC->WDATA = (uint32_t)counts << 17;

  /* RRDY is set when the operation completes - with NRES = 2 that means both
   * results are latched - and is cleared only once RDATA has been read twice.
   * So one poll covers the pair; a second poll before the sine would spin
   * zero times and buy nothing. */
  while ((CORDIC->CSR & CORDIC_CSR_RRDY) == 0U) { }

  int32_t cos_q31 = (int32_t)CORDIC->RDATA;   /* results come out cos first */
  int32_t sin_q31 = (int32_t)CORDIC->RDATA;

  *cos_out = (float)cos_q31 * (1.0f / 2147483648.0f);
  *sin_out = (float)sin_q31 * (1.0f / 2147483648.0f);
}

void FOC_Reset(FocState_t *f)
{
  f->id_integ = 0.0f;
  f->iq_integ = 0.0f;
}

void FOC_SetGainsForVbus(FocState_t *f, int32_t vbus_mv)
{
  if (f->vbus_track == 0U) { return; }
  if ((vbus_mv < FOC_VBUS_MIN_MV) || (vbus_mv > FOC_VBUS_MAX_MV)) { return; }

  float vbus = (float)vbus_mv * 0.001f;

  /* Same derivation as the defaults, just against the bus that is actually
   * present: pole-zero cancellation places the closed loop at FOC_BW_RADS,
   * and dividing by vbus is what keeps it there as the supply moves. */
  f->kp = (FOC_BW_RADS * FOC_L_H)   / vbus;
  f->ki = (FOC_BW_RADS * FOC_R_OHM) / vbus;

  /* The decoupling feedforward is computed in volts and applied as normalised
   * duty, so it needs 1/Vbus. It is derived HERE, from the same reading that
   * has already passed the sanity window above, rather than anywhere else -
   * a second place that divides by a bus voltage is a second place that can
   * divide by zero and put NaN into the duty registers. If the reading is bad
   * this function has already returned and inv_vbus keeps its last good
   * value, which is the same protection kp and ki get. */
  f->inv_vbus = 1.0f / vbus;

  f->vbus_used_mv = vbus_mv;
  f->kp_x10000    = (int32_t)(f->kp * 10000.0f);
  f->ki_x100      = (int32_t)(f->ki * 100.0f);
}

void FOC_Init(FocState_t *f)
{
  /* Bring up the CORDIC here, not in the ISR. Everything that never varies -
   * function, precision, result count, and the magnitude argument - is set
   * once, so the control step costs one register write and two reads.
   *
   * Called from main() before MotorPwm_EnableControlIsr(), which is what makes
   * that safe: the ISR's first FOC_SinCos cannot run before this returns. */
  RCC->AHB1ENR |= RCC_AHB1ENR_CORDICEN;
  (void)RCC->AHB1ENR;   /* read back so the CSR write below cannot overtake
                         * the clock actually coming up - a peripheral write
                         * that lands before its clock is simply discarded */

  /* Seed the magnitude argument explicitly rather than inheriting whatever
   * the CORDIC happens to hold. For sin/cos it SCALES both results, so a
   * value other than 1.0 would quietly scale every current the Park transform
   * reports and every voltage the inverse Park applies - a gain error in the
   * middle of the control loop with nothing to attribute it to.
   *
   * Written once with NARGS = 2 (angle then magnitude); the CORDIC retains it,
   * so the steady-state config below drops to NARGS = 1 and the ISR supplies
   * the angle alone. */
  CORDIC->CSR   = FOC_CORDIC_CFG | CORDIC_CSR_NARGS;
  CORDIC->WDATA = 0U;             /* angle 0                     */
  CORDIC->WDATA = 0x7FFFFFFFU;    /* magnitude 1.0 in Q31        */
  while ((CORDIC->CSR & CORDIC_CSR_RRDY) == 0U) { }
  (void)CORDIC->RDATA;            /* drain both, or RRDY stays set and the */
  (void)CORDIC->RDATA;            /* first real call reads these instead   */

  CORDIC->CSR = FOC_CORDIC_CFG;

  f->id_ref  = 0.0f;
  f->iq_ref  = 0.0f;
  f->kp      = FOC_KP_DEFAULT;
  f->ki      = FOC_KI_DEFAULT;
  f->vmax    = FOC_VMAX_DEFAULT;
  f->enabled     = 0U;
  f->elec_offset = FOC_ELEC_OFFSET_DEFAULT;
  f->vbus_track   = 1U;
  f->vbus_used_mv = FOC_VBUS_NOM_MV;
  f->kp_x10000    = (int32_t)(f->kp * 10000.0f);
  f->ki_x100      = (int32_t)(f->ki * 100.0f);
  f->updates = 0U;
  f->isr_max = 0U;
  f->mirror_div = 0U;

  /* On by default. The uncompensated loop is the wrong one - it is only
   * survivable at this bench's electrical frequency - so the corrected
   * behaviour is what runs unless someone deliberately turns it off to
   * compare. */
  f->delay_comp = 1U;
  f->omega_e    = 0.0f;
  f->omega_e_rads_x10  = 0;
  f->theta_adv_deg_x10 = 0;

  /* OFF by default. It does what it claims and it is still not ready.
   *
   * What works: at ~266 Hz electrical, 12 ensemble-averaged iq steps, it cuts
   * the d-axis disturbance it exists to cut -
   *
   *       peak |id|   123 mA on   209 mA off    -41%
   *       rms id     51.1 mA on  78.6 mA off    -35%
   *
   * What does not: run the actual foc_dash demo, which spins to terminal
   * speed and then reverses, and peak phase current is 9326 mA with this on
   * against 1489 mA with it off. Six times worse. It no longer TRIPS the 15 A
   * limit - that took fixing a lambda_m 14% high (foc.h) and an anti-windup
   * that crushed the integrators (below) - but six times the peak current is
   * not a correction, it is a different bug.
   *
   * The cause is the velocity estimate, and it is a gap in the original
   * analysis rather than a bug in the code. omega_e comes from the position
   * loop, filtered at ~8 Hz because kd there must not amplify encoder
   * quantisation. That is a 20 ms lag. Under 1 A of braking this rotor
   * decelerates at ~16000 electrical rad/s^2, so during a hard reversal
   * omega_e is wrong by ~320 rad/s, the feedforward is wrong by 0.86 V, and
   * 0.86 V across an 85 mohm winding is 10.1 A. Measured 9.3 A.
   *
   * It is worse than that arithmetic suggests, because at terminal speed
   * w_e*lambda_m/Vbus is 0.250 against a vmax of 0.250 - the feedforward
   * alone fills the entire modulation ceiling, so there is no headroom left
   * for the PI to correct the error quickly.
   *
   * The delay compensation was checked against this same lag and is fine: it
   * uses omega_e for an ANGLE advance, where 320 rad/s is 0.3 degrees. The
   * feedforward uses it for a VOLTAGE magnitude, where the same error is
   * volts. Checking one and assuming the other was the mistake.
   *
   * What this needs before it goes on by default: an omega_e for the
   * feedforward that tracks faster than 8 Hz. Not a second estimator that can
   * disagree with the position loop's - the same encoder differences, filtered
   * for a different purpose. And re-test with the DEMO, not a short reversal;
   * a 1.5 s reversal showed 1359 vs 1375 mA and looked like a pass, because
   * the motor never reached the speed where this bites. */
  f->decouple = 0U;

  /* Deadtime compensation starts DISABLED, with the theoretical magnitude
   * loaded but not applied. The sign depends on the current-sense and gate
   * polarities together and a wrong sign doubles the distortion, so it is
   * swept and measured on hardware before being trusted - dtc_pm is set to
   * the winning value once that has been done. */
  f->dtc_pm      = 0;

  /* Field weakening OFF. Nothing about it has been measured on the motor, and
   * f->decouple and f->dtc_pm are the standing examples of what shipping a
   * correction enabled on reasoning alone costs. */
  f->fw_enable  = 0U;
  f->fw_kp      = 0.0f;                 /* pure integrator - see foc.h */
  f->fw_ki      = FOC_FW_KI_DEFAULT;
  f->fw_integ   = 0.0f;
  f->fw_id_max  = (float)LIM_ID_FW_MAX_MA * 0.001f;
  f->fw_id_ma      = 0;
  f->fw_headroom_pm = 0;
  f->fw_last_demand = 0.0f;
  f->ident_active   = 0U;
  f->ident_vd       = 0.0f;
  f->dtc_hyst_ma = FOC_DTC_HYST_MA;
  f->vd_ff_pm = 0;
  f->vq_ff_pm = 0;

  /* Seeded for FOC_VBUS_NOM_MV so the feedforward is scaled correctly from
   * the first ISR tick, before the main loop has measured the bus even once.
   * Zero here would make the whole feedforward vanish until the first
   * successful Vbus read, which is a silent wrong-behaviour window. */
  f->inv_vbus = 1.0f / ((float)FOC_VBUS_NOM_MV * 0.001f);

  FOC_Reset(f);
}

/* ---- torque interface ---------------------------------------------------
 *
 * Command path, not ISR path: these run when a request arrives, at whatever
 * rate the transport delivers, and none of them is called from FOC_Update.
 * The 30 kHz loop still works in amps.
 */

float FOC_TorqueToIq(float t_nm)
{
  return t_nm / FOC_KT_NM_PER_A;
}

float FOC_IqToTorque(float iq_a)
{
  return iq_a * FOC_KT_NM_PER_A;
}

int32_t FOC_SetTorque(FocState_t *f, int32_t t_mnm)
{
  /* No null check, matching every other entry point in this file: these are
   * internal APIs always called with a real state, and a defensive test that
   * cannot fire is noise on the ISR path's own translation unit. It also
   * pulled in NULL, which foc.c had no other reason to know about - and got
   * away with it locally only because msys2's headers happen to reach
   * stddef.h transitively where the CI runner's do not. */
  float iq_a = FOC_TorqueToIq((float)t_mnm * 0.001f);

  /* Saturate in CURRENT, not in torque, because the limit is a current limit:
   * LIM_IQ_MAX_MA is what the sensors, the FETs and the machine's thermal
   * budget allow. Converting the clamp back into torque afterwards keeps the
   * two consistent by construction - clamping a torque figure against a
   * torque bound derived from the same kt would be the same arithmetic done
   * twice, with two places to get it wrong. */
  int32_t iq_ma = (int32_t)(iq_a * 1000.0f);
  if (iq_ma >  LIM_IQ_MAX_MA) { iq_ma =  LIM_IQ_MAX_MA; }
  if (iq_ma < -LIM_IQ_MAX_MA) { iq_ma = -LIM_IQ_MAX_MA; }

  f->iq_ref = (float)iq_ma * 0.001f;

  /* Surface magnet: MTPA is id = 0. See the note in foc.h before changing
   * this - the reluctance term it would exploit is identically zero here.
   * Field weakening drives id negative through a different path, and would
   * overwrite this deliberately. */
  f->id_ref = 0.0f;

  return (int32_t)(FOC_IqToTorque((float)iq_ma * 0.001f) * 1000.0f);
}

void FOC_Update(FocState_t *f, int32_t iu_ma, int32_t iw_ma, uint16_t enc_raw,
                float vel_mech_rads)
{
  /* ---- currents ---------------------------------------------------- */
  f->iu = (float)iu_ma * 0.001f;
  f->iw = (float)iw_ma * 0.001f;
  /* No neutral, so the three must sum to zero. */
  f->iv = -(f->iu + f->iw);

  /* ---- Clarke: 3-phase -> stationary alpha/beta --------------------- */
  f->ialpha = f->iu;
  f->ibeta  = (f->iu + 2.0f * f->iv) * ONE_BY_SQRT3;

  /* ---- electrical angle -------------------------------------------- */
  /* Encoder zero IS electrical zero (programmed into the A1333), so this
   * needs no offset term - just the pole-pair multiply, wrapped. */
  f->enc_raw = enc_raw;
  {
    int32_t e = (int32_t)(((uint32_t)(enc_raw & 0x7FFFU) * FOC_POLE_PAIRS)
                          % FOC_ENC_COUNTS);
    e += f->elec_offset;
    e %= (int32_t)FOC_ENC_COUNTS;
    if (e < 0) { e += (int32_t)FOC_ENC_COUNTS; }
    f->elec_counts = (uint16_t)e;
  }

  /* Straight from the integer angle - see FOC_SinCos. The float radian value
   * that used to sit here existed only to be handed to sinf/cosf, and building
   * it was the cheap half of the cost. */
  FOC_SinCos(f->elec_counts, &f->sin_e, &f->cos_e);

  /* ---- transport-delay compensation --------------------------------- *
   *
   * The two Park transforms do NOT describe the same instant, and using one
   * angle for both is only harmless while the rotor barely moves between
   * periods.
   *
   *   - The FORWARD Park converts a current that was MEASURED, so it belongs
   *     at the angle the rotor was at when the ADC sampled: theta. Unchanged.
   *   - The INVERSE Park converts a voltage that has not been APPLIED yet. By
   *     the time it reaches the motor the rotor has turned by
   *     omega_e * FOC_DELAY_PERIODS / PWM_FREQ_HZ, so it belongs at the angle
   *     the rotor WILL be at: theta + that.
   *
   * Uncompensated, the applied vector lands behind the rotor by that angle,
   * which cross-couples q-axis command into the d axis - the loop commands
   * torque and gets some flux with it. On this bench at ~200 Hz electrical it
   * is 4.0 degrees and easy to miss. On the EMRAX at 917 Hz it is 18.2
   * degrees, where cos(18.2) = 0.95 of the intended torque appears and
   * sin(18.2) = 0.31 of the current vector becomes an unrequested d-axis
   * disturbance the PI then has to fight.
   *
   * omega_e is the position loop's existing velocity estimate, not a new one.
   * It is filtered (~8 Hz corner), so it lags during hard acceleration - worth
   * knowing, but checked and negligible: an EMRAX going 0 to 5500 rpm in 2 s
   * moves 58 electrical rad/s during that lag, which is 0.18 degrees of
   * advance error. The filter is not the limiting term here.
   *
   * DO NOT expect this to show up in steady-state id, and do not "fix" it when
   * it does not. Measured on the bench at 200 Hz electrical, 1800 samples per
   * condition (tools/delay_comp_ab.sh): rms id 150.7 mA with compensation on,
   * 151.3 mA off - a difference of -0.4% +/- 2.4%, which is nothing.
   *
   * That is the correct result, not a failure, and the reason is structural.
   * The FORWARD Park is right, so measured id IS the true d-axis current; the
   * d-axis PI has integral action, so it drives that to zero at steady state
   * whether or not the inverse Park angle is right. The angle error changes
   * which vd/vq are needed to get there and it costs phase margin, but the
   * equilibrium is nulled either way. All that leaks into steady-state id is
   * sin(3.9 deg) = 6.8% of the iq RIPPLE, which is single-digit mA against a
   * ~150 mA noise floor.
   *
   * The cost is paid in the TRANSIENT, where the integrator has not caught up
   * yet, and in phase margin as f_e rises - 3.9 degrees here, 18.2 on the
   * EMRAX at 917 Hz.
   *
   * The step test does not resolve it either, and that is also expected. With
   * tools/step_trace.sh at ~289 Hz, decoupling left on, 56 ensemble-averaged
   * steps per condition: peak |id| 46 mA compensated against 57 mA not, with
   * a post-averaging noise floor of 19-24 mA. The difference is smaller than
   * the floor. Rep counts of 12, 26 and 56 gave -30%, -23% and -19%, shrinking
   * as averaging removed noise - the signature of an effect that was never
   * there rather than one being uncovered.
   *
   * Arithmetic says the same thing. After the transient, vq only has to rise
   * by R*diq = 68 mV; the back-EMF term is unchanged by an iq step. A 5.73
   * degree misalignment leaks sin(5.73) of that onto d, which is 6.8 mV, or
   * 16 mA against R + kp*Vbus. Sixteen milliamps under a twenty milliamp
   * floor is not a measurement this bench can make.
   *
   * So this is kept on derivation, not on evidence, and that is the honest
   * status. What IS verified is the mechanism: the advance is applied
   * accurately on a spinning rotor (3.90 degrees measured against 3.96
   * predicted), with the correct sign, and it costs 130 cycles. What scales
   * to the EMRAX is not the number above but the cross-coupling fraction
   * sin(theta_err), which goes from 0.10 here to 0.31 at 917 Hz - a factor of
   * 3.1 on a machine whose axes are far more strongly coupled to begin with.
   * Do not conclude from the bench nulls that this can be dropped for the HV
   * build; conclude that 48 V cannot see it. */
  f->omega_e = vel_mech_rads * (float)FOC_POLE_PAIRS;

  float   sin_o = f->sin_e;
  float   cos_o = f->cos_e;
  int32_t adv   = 0;

  if (f->delay_comp != 0U)
  {
    adv = (int32_t)(f->omega_e * FOC_ADV_COUNTS_PER_RADS);

    /* Bound it. The largest advance any real machine here can ask for is
     * about 18 degrees; a quarter turn is far outside that, so anything
     * beyond it is a broken velocity estimate rather than a fast rotor. Left
     * unbounded, one bad estimate rotates the applied vector to an arbitrary
     * angle - at 48 V a twitch, at 600 V full current into the wrong place.
     * Phase 3 is where the estimate itself gets policed; this is just a
     * ceiling on the damage in the meantime. */
    if (adv >  (int32_t)(FOC_ENC_COUNTS / 4U)) { adv =  (int32_t)(FOC_ENC_COUNTS / 4U); }
    if (adv < -(int32_t)(FOC_ENC_COUNTS / 4U)) { adv = -(int32_t)(FOC_ENC_COUNTS / 4U); }

    /* FOC_ENC_COUNTS is a power of two, so masking wraps the sum correctly
     * for a negative advance too: two's complement makes (-1 & 32767) = 32767,
     * which is the angle one count below zero. No branch, no modulo. */
    uint16_t theta_out =
      (uint16_t)(((int32_t)f->elec_counts + adv) & (int32_t)(FOC_ENC_COUNTS - 1U));

    FOC_SinCos(theta_out, &sin_o, &cos_o);
  }

  /* ---- Park: stationary -> rotor frame ------------------------------ */
  f->id =  f->ialpha * f->cos_e + f->ibeta * f->sin_e;
  f->iq = -f->ialpha * f->sin_e + f->ibeta * f->cos_e;

  /* ---- PI on both axes ---------------------------------------------- */
  float ed = f->id_ref - f->id;
  float eq = f->iq_ref - f->iq;

  /* dt is the PWM period, because this runs once per period. Derived from
   * PWM_FREQ_HZ rather than written out, so changing the switching frequency
   * cannot silently rescale the integrator gain. */
  f->id_integ += ed * f->ki * FOC_DT;
  f->iq_integ += eq * f->ki * FOC_DT;

  f->vd = ed * f->kp + f->id_integ;
  f->vq = eq * f->kp + f->iq_integ;

  /* Parameter identification takes the d axis outright. The integrators are
   * held at zero rather than left alone: id_ref is 0 throughout while the
   * measurement pushes 2 A through d, so an integrator that kept running
   * would wind against that whole error and dump it when the override ended.
   * One load and one branch when inactive, which is what this costs the
   * 30 kHz loop the rest of the time. */
  if (f->ident_active != 0U)
  {
    f->id_integ = 0.0f;
    f->iq_integ = 0.0f;
    f->vd       = f->ident_vd;
    f->vq       = 0.0f;
  }

  /* ---- cross-coupling decoupling and back-EMF feedforward ------------- *
   *
   * The d and q axes are not independent plants. Rotating the frame couples
   * them, and the magnet adds a speed-proportional voltage on q:
   *
   *     vd = R*id + Ld*d(id)/dt - w_e*Lq*iq
   *     vq = R*iq + Lq*d(iq)/dt + w_e*Ld*id + w_e*lambda_m
   *
   * The PI only ever saw the R and L terms. Everything with a w_e in it
   * arrived as an unexplained disturbance it had to integrate its way out of,
   * which works while that disturbance is small compared to what the PI can
   * generate - and stops working when it is not.
   *
   * Scale matters here more than structure. On this bench w_e*L is 68 mOhm at
   * 200 Hz against R = 85 mOhm, so the coupling is merely comparable to the
   * plant. On the EMRAX at 917 Hz it is 1.47 Ohm against 23.2 mOhm - 63 times
   * the resistance. No PI rejects a disturbance 63x its own plant gain; it is
   * not a tuning problem.
   *
   * The back-EMF term is the larger one by far. At the top of this bench's
   * range w_e*lambda_m is 0.248 of normalised duty against a vmax of 0.25, so
   * without this the integrator has to discover essentially the entire
   * voltage budget by itself, from zero, on every enable - and it can only do
   * that as fast as ki allows. Feeding it forward means the PI starts from
   * roughly the right answer and is left doing what it is good at: correcting
   * the small remainder.
   *
   * These terms are volts; everything else in this function is normalised
   * duty. inv_vbus does that conversion and comes from the one guarded place
   * a bus voltage is ever divided - see FOC_SetGainsForVbus.
   *
   * Applied BEFORE the vector limit below, so the feedforward is bounded by
   * the same ceiling as everything else and cannot command a duty the bridge
   * has no way to produce. One interaction that follows from that and is
   * worth knowing: at the very top of the speed range the feedforward alone
   * nearly fills vmax, so the vector limit engages and the back-calculation
   * scales the integrators even though the PI is not what saturated. That is
   * correct - there is genuinely no voltage left - and it is not a change,
   * because without feedforward the integrator reached the same ceiling by
   * itself. It does mean the drive is out of voltage at 600 rpm either way. */
  float vd_ff = 0.0f;
  float vq_ff = 0.0f;

  if (f->decouple != 0U)
  {
    vd_ff = -f->omega_e * (FOC_LQ_H * f->iq)                   * f->inv_vbus;
    vq_ff =  f->omega_e * (FOC_LD_H * f->id + FOC_LAMBDA_M_WB) * f->inv_vbus;

    f->vd += vd_ff;
    f->vq += vq_ff;
  }
  /* The integer mirrors of these two live in the decimated block at the end,
   * not here. Same reason as everything else there: a float-to-int conversion
   * that exists only so a debugger can read the value has no business running
   * 30000 times a second. */

  /* Limit the vector magnitude, not each axis separately - clipping d and q
   * independently rotates the applied vector away from where it was asked
   * for. */
  float vmag = fm_sqrtf(f->vd * f->vd + f->vq * f->vq);
  f->fw_last_demand = vmag;   /* pre-limit, for the weakening loop and its mirror */
  if (vmag > f->vmax)
  {
    float k      = f->vmax / vmag;
    float vd_cmd = f->vd;
    float vq_cmd = f->vq;

    f->vd *= k;
    f->vq *= k;

    /* Back-calculation anti-windup: subtract exactly the voltage that could
     * NOT be delivered. Clamping each integrator to +/-vmax separately is not
     * enough - two axes each at the limit reach 1.41*vmax combined, so they
     * wind past anything the output can produce and the loop stops
     * responding.
     *
     * This used to scale the integrators by k instead, which is only
     * equivalent while the integrators are the whole of the command. Once the
     * decoupling feedforward was added they are not, and scaling became
     * actively wrong: with the feedforward alone at or above vmax, k < 1 on
     * every tick, so the integrators were multiplied down toward zero
     * forever and the PI lost all integral authority. Measured on the bench -
     * vq sat pinned at 0.246 of a 0.25 ceiling for 4 ms after a torque
     * reversal while the loop failed to reverse the current, and peak current
     * went from 1.3 A to 4.7 A.
     *
     * Subtracting the undelivered part instead lets the integrator absorb an
     * over-large feedforward: it simply winds negative until PI + ff lands on
     * what the bridge can actually produce, which is the correct answer and
     * the behaviour scaling could never reach. */
    f->id_integ -= (vd_cmd - f->vd);
    f->iq_integ -= (vq_cmd - f->vq);
  }

  /* ---- field weakening ------------------------------------------------
   *
   * Deliberately AFTER the vector limit, and fed from `vmag` - the magnitude
   * demanded BEFORE the limiter scaled it. Measured after, |v| is vmax by
   * construction whenever it matters, the headroom reads zero rather than
   * negative, and the loop never engages. That is the whole trap.
   *
   * The id it produces is applied on the NEXT tick. At 30 kHz that is 33 us
   * against a weakening loop deliberately orders of magnitude slower, so the
   * delay is not part of its dynamics.
   */
  if (f->fw_enable != 0U)
  {
    float headroom = f->vmax - vmag;      /* negative once the demand is over */

    f->fw_integ += f->fw_ki * headroom;

    /* Clamping IS the anti-windup here, and it is sufficient because this is
     * an integrator with no other state: there is nothing else to unwind. The
     * upper clamp at zero is what keeps it a WEAKENING loop - a positive id
     * on a surface-magnet machine strengthens the field, costing current to
     * make the saturation worse. */
    if (f->fw_integ > 0.0f)          { f->fw_integ = 0.0f; }
    if (f->fw_integ < -f->fw_id_max) { f->fw_integ = -f->fw_id_max; }

    f->id_ref = f->fw_integ + f->fw_kp * headroom;
    if (f->id_ref > 0.0f)          { f->id_ref = 0.0f; }
    if (f->id_ref < -f->fw_id_max) { f->id_ref = -f->fw_id_max; }
  }
  else if (f->fw_integ != 0.0f)
  {
    /* Switched off while it was weakening. Give back the d-axis demand it
     * owned rather than leaving the last value it happened to reach standing
     * as a permanent uncommanded current. */
    f->fw_integ = 0.0f;
    f->id_ref   = 0.0f;
  }

  /* ---- inverse Park -------------------------------------------------- */
  /* sin_o/cos_o, not sin_e/cos_e: this vector is applied a period and a half
   * from now, at an angle the rotor has not reached yet. See the delay block
   * above. With delay_comp off they are the same pair and this is the old
   * behaviour exactly, which is what makes the A/B comparison honest. */
  f->valpha = f->vd * cos_o - f->vq * sin_o;
  f->vbeta  = f->vd * sin_o + f->vq * cos_o;

  /* ---- inverse Clarke ------------------------------------------------ */
  float vu = f->valpha;
  float vv = -0.5f * f->valpha + 0.86602540378f * f->vbeta;
  float vw = -0.5f * f->valpha - 0.86602540378f * f->vbeta;

  /* Min/max (third-harmonic) injection: shifting all three by the midpoint
   * of their extremes costs nothing differentially but buys ~15% more usable
   * bus voltage before clipping. */
  float vmaxp = vu, vminp = vu;
  if (vv > vmaxp) { vmaxp = vv; }
  if (vw > vmaxp) { vmaxp = vw; }
  if (vv < vminp) { vminp = vv; }
  if (vw < vminp) { vminp = vw; }

  float voff = 0.5f * (vmaxp + vminp);
  vu -= voff;
  vv -= voff;
  vw -= voff;

  /* ---- to duty ------------------------------------------------------- */
  f->duty_u = vu + 0.5f;
  f->duty_v = vv + 0.5f;
  f->duty_w = vw + 0.5f;

  /* ---- deadtime compensation ---------------------------------------- *
   *
   * Applied per PHASE, not per axis, because that is where the error lives:
   * each leg's applied volt-seconds are short by t_d*f_sw*Vbus in the
   * direction its own current is flowing. See the derivation in foc.h.
   *
   * After the +0.5 offset and before the clamp, so the compensation is part
   * of what gets bounded to a producible duty rather than something added on
   * top of an already-saturated one.
   *
   * The hysteresis band matters more than it looks. Inside it no compensation
   * is applied at all, which leaves a small uncorrected wedge around each
   * zero crossing - accepted deliberately, because the alternative is taking
   * the sign from a measurement whose noise floor is comparable to the band
   * and chattering the correction at the noise frequency. */
  if (f->dtc_pm != 0)
  {
    float dtc = (float)f->dtc_pm      * 0.001f;
    float h   = (float)f->dtc_hyst_ma * 0.001f;

    if      (f->iu >  h) { f->duty_u += dtc; }
    else if (f->iu < -h) { f->duty_u -= dtc; }

    if      (f->iv >  h) { f->duty_v += dtc; }
    else if (f->iv < -h) { f->duty_v -= dtc; }

    if      (f->iw >  h) { f->duty_w += dtc; }
    else if (f->iw < -h) { f->duty_w -= dtc; }
  }

  if (f->duty_u < 0.0f) { f->duty_u = 0.0f; }
  if (f->duty_v < 0.0f) { f->duty_v = 0.0f; }
  if (f->duty_w < 0.0f) { f->duty_w = 0.0f; }
  if (f->duty_u > 1.0f) { f->duty_u = 1.0f; }
  if (f->duty_v > 1.0f) { f->duty_v = 1.0f; }
  if (f->duty_w > 1.0f) { f->duty_w = 1.0f; }

  /* Integer mirrors for SWD readout - decimated to 1 kHz.
   *
   * About twenty float-to-int conversions that exist only so a debugger can
   * read this state. Nothing in the control path uses them, and no debugger
   * samples faster than a few hundred hertz, so running them every switching
   * period was spending real ISR budget on nobody's behalf. At 30 kHz that
   * budget is no longer spare. */
  if (++f->mirror_div >= (PWM_FREQ_HZ / 1000U))
  {
    f->mirror_div = 0U;
    f->id_ma     = (int32_t)(f->id * 1000.0f);
    f->iq_ma     = (int32_t)(f->iq * 1000.0f);
    f->iu_ma     = iu_ma;
    f->iw_ma     = iw_ma;
    f->vd_mv     = (int32_t)(f->vd * 1000.0f);
    f->vq_mv     = (int32_t)(f->vq * 1000.0f);
    f->duty_u_pm = (int32_t)(f->duty_u * 1000.0f);
    f->duty_v_pm = (int32_t)(f->duty_v * 1000.0f);
    f->duty_w_pm = (int32_t)(f->duty_w * 1000.0f);
    f->iq_ref_ma = (int32_t)(f->iq_ref * 1000.0f);
    f->id_ref_ma = (int32_t)(f->id_ref * 1000.0f);
    f->elec_deg_x10 = (int32_t)(((uint32_t)f->elec_counts * 3600U) / FOC_ENC_COUNTS);
    f->vmax_pm   = (int32_t)(f->vmax * 1000.0f);
    f->vmag_pm   = (int32_t)(fm_sqrtf(f->vd * f->vd + f->vq * f->vq) * 1000.0f);

    /* Field weakening, for the SWD tools. The headroom mirrored here is the
     * PRE-limiter one the loop actually acts on - mirroring the post-limiter
     * figure would show a permanent zero and hide the very thing anyone
     * reading this would be looking for. */
    f->fw_id_ma       = (int32_t)(f->id_ref * 1000.0f);
    f->fw_headroom_pm = (int32_t)((f->vmax - f->fw_last_demand) * 1000.0f);
    f->omega_e_rads_x10  = (int32_t)(f->omega_e * 10.0f);
    f->theta_adv_deg_x10 = (adv * 3600) / (int32_t)FOC_ENC_COUNTS;
    f->vd_ff_pm  = (int32_t)(vd_ff * 1000.0f);
    f->vq_ff_pm  = (int32_t)(vq_ff * 1000.0f);
  }

  f->updates++;
}
