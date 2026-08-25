/**
  ******************************************************************************
  * @file    foc.h
  * @brief   Field-oriented current control.
  *
  *          Runs from the HRTIM period ISR at the PWM rate, so every step -
  *          current sample, transform, PI, duty update - happens at the same
  *          point in every switching period.
  *
  *          Only U and W are instrumented; V is reconstructed as -(iu+iw),
  *          which holds because there is no neutral connection.
  *
  *          Electrical angle comes straight from the encoder with no software
  *          offset: the A1333's ZERO_OFFSET is programmed so encoder zero IS
  *          electrical zero. See HARDWARE_NOTES section 8.
  ******************************************************************************
  */
#ifndef FOC_H
#define FOC_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#define FOC_POLE_PAIRS      20U
#define FOC_ENC_COUNTS      32768U

/* Current control gains, sized from the plant rather than guessed.
 *
 * Duty is normalised, so the plant from duty deviation u to phase current is
 *     I/u = Vbus / (R + sL)
 * with Vbus = 15.55 V (measured at the supply), R = 85 mohm, L = 54.3 uH.
 *
 * Placing the closed-loop bandwidth at 1 kHz with pole-zero cancellation:
 *     kp = w * L / Vbus = 6283 * 54.3e-6 / 15.55 = 0.022
 *     ki = w * R / Vbus = 6283 * 0.085  / 15.55 = 34
 *
 * The first attempt used kp = 0.08, which meant a 1 A error commanded 1.49 V
 * across an 85 mohm winding - a demand for 17 A. Loop gain of ~17 oscillates
 * no matter how clean the current feedback is.
 *
 * Note where Vbus sits: in the DENOMINATOR of both gains. It is not a
 * calibration detail, it is the plant gain. Raise the supply to 31 V without
 * touching these numbers and every duty the loop commands delivers twice the
 * current it was sized for - the loop gain doubles, the phase margin goes with
 * it, and a controller tuned to be critically damped at 15.5 V rings or
 * oscillates. This is exactly the mistake the kp = 0.08 attempt above made,
 * arrived at from the other direction.
 *
 * So these two are only the values for FOC_VBUS_NOM_MV. The live gains are
 * recomputed from the measured bus by FOC_SetGainsForVbus - see below. */
#define FOC_KP_DEFAULT      0.022f
#define FOC_KI_DEFAULT      34.0f

/* Plant parameters the gains are derived from, kept explicit so the bus-
 * voltage rescale can recompute rather than merely scale. */
/* Closed-loop current bandwidth. 1 kHz here, and NOT a number to copy to the
 * HV inverter without reading the rest of this comment.
 *
 * ---- what sets the ceiling -------------------------------------------
 *
 * After pole-zero cancellation the open loop is just an integrator behind the
 * transport delay of FOC_DELAY_PERIODS (foc.c):
 *
 *     L(s) = (w_bw / s) * exp(-s*Td)
 *
 * The integrator contributes -90 degrees at every frequency and the delay
 * contributes -w*Td, so at crossover
 *
 *     PM = 90 deg - w_bw * Td
 *
 * That is the whole story: bandwidth and phase margin trade against each
 * other through Td alone, and Td is set by the switching frequency. On this
 * bench Td is 55 us (1.65 periods at 30 kHz, including the 5 us ADC lead), so
 * FOC_BW_RADS = 6283 rad/s gives PM = 70 degrees. Healthy, and the reason the
 * bench value is being left alone.
 *
 * ---- what that means for the EMRAX at 917 Hz electrical --------------
 *
 * Taking Td = 1.5/f_sw (the HV board's ADC lead is not known yet and only
 * makes this worse):
 *
 *     f_sw     Td       bw @ PM 60      x f_e     ripple into 255 uH
 *     10 kHz   150 us     556 Hz        0.61x     58.8 A pp  (19.6%)
 *     20 kHz    75 us    1111 Hz        1.21x     29.4 A pp  ( 9.8%)
 *     30 kHz    50 us    1667 Hz        1.82x     19.6 A pp  ( 6.5%)
 *
 * The usual "current loop wants 5 to 10 times f_e" would need 83 kHz for 5x
 * and 165 kHz for 10x. At 600 V and 300 A those are not switching
 * frequencies, they are a thermal design that does not close - 30 kHz is
 * already at the aggressive end for a SiC traction bridge at this power.
 *
 * So the honest answer to "is 30 kHz enough" is: it is enough for everything
 * except the thing the rule of thumb is actually about, and no reachable
 * frequency fixes that one. Bandwidth is not the lever here.
 *
 * ---- which changes what the design has to get right -------------------
 *
 * Torque RESPONSE is not the binding requirement and never was. 1667 Hz is a
 * 0.21 ms rise time against a VCU that issues torque commands at 100-1000 Hz.
 * The requirement the 5-10x rule encodes is DISTURBANCE REJECTION - having
 * enough loop gain at f_e to reject the cross-coupling as an unknown. That is
 * what cannot be bought at any practical f_sw, and the scale of it is why:
 *
 *     w_e * Lq * iq at 917 Hz and 300 A = 441 V, or 73% of a 600 V bus
 *     w_e * lambda_m at 917 Hz          = 346 V, or 58% of the bus
 *
 * Three quarters of the bus is not a disturbance a controller rejects. It has
 * to be computed and fed forward, which is what FOC_LD_H / FOC_LQ_H /
 * FOC_LAMBDA_M_WB and f->decouple are for. That makes the decoupling
 * load-bearing rather than an optimisation, and moves the risk from the
 * controller onto the MOTOR PARAMETERS:
 *
 *     Lq error 10%  ->  44 V uncancelled  ( 7.3% of bus)
 *     Lq error 20%  ->  88 V uncancelled  (14.7% of bus)
 *     lambda_m 11%  ->  38 V uncancelled  ( 6.4% of bus)
 *
 * 20% is not a pessimistic figure for Lq on an axial-flux machine at 300 A -
 * that is what saturation does - and 11% is roughly what 100 K does to NdFeB
 * remanence. Against kp = w_bw*Lq = 2.67 V/A, a 34 V feedforward error is a
 * 13 A standing error for the integrator to walk out.
 *
 * ---- what to take to the hardware review ------------------------------
 *
 *   1. 30 kHz is defensible and 20 kHz is workable; the choice between them
 *      is a loss-and-cooling question, not a control-bandwidth one, because
 *      neither reaches the coupling-rejection bandwidth and both are far
 *      beyond what torque response needs.
 *   2. Do not spend thermal budget buying switching frequency in the hope of
 *      controller margin. It buys 1.2x versus 1.8x of f_e; the requirement is
 *      5x. The money goes to feedforward accuracy instead.
 *   3. Characterise Lq against current and lambda_m against temperature, and
 *      budget for a table rather than the two constants that suffice at 48 V.
 *      This is the deliverable that decides whether the HV current loop
 *      works, and it is a motor-test-rig task, not a firmware one.
 *   4. Transport-delay compensation gets MORE important, not less: at 917 Hz
 *      the uncompensated frame error is 18.2 degrees, and sin(18.2) = 0.31 of
 *      a 441 V coupling term is not a rounding error. See foc.c.
 */
#define FOC_BW_RADS         6283.0f    /* 1 kHz target bandwidth       */
#define FOC_R_OHM           0.085f     /* phase resistance             */
#define FOC_L_H             54.3e-6f   /* phase inductance             */
#define FOC_VBUS_NOM_MV     15550      /* bus the defaults were sized at */

/* d- and q-axis inductances, for the decoupling terms.
 *
 * Both are FOC_L_H. The bench motor is an EaglePower 8309, a surface-magnet
 * outrunner, so the magnets sit in the airgap and the rotor presents the same
 * reluctance whichever way it is pointing - Ld = Lq, and there is no
 * reluctance torque to chase. Named separately anyway because the decoupling
 * equations are written in terms of both, and because the EMRAX 228 is also
 * surface-PM (axial flux) so the same equality carries over rather than being
 * a bench-only simplification that has to be revisited.
 *
 * If a motor with saliency is ever fitted, these are the two numbers to
 * measure, and MTPA stops being id = 0 - see the note in phase 5. */
#define FOC_LD_H            FOC_L_H
#define FOC_LQ_H            FOC_L_H

/* Rotor flux linkage, Wb (peak, per phase). MEASURED, not derived.
 *
 * 2.68 mWb, measured on the bench 2026-08-25 by spinning the motor with the
 * feedforward disabled and reading back what voltage the loop actually needed:
 *
 *     vq * Vbus = w_e * lambda_m + R * iq    (steady state, id ~ 0)
 *
 * Five operating points, of which the three below the modulation ceiling are
 * the trustworthy ones:
 *
 *     iq_cmd   vq(norm)   iq(mA)   w_e     lambda_m
 *      0.30     0.0704      299     623     2.639 mWb
 *      0.45     0.1551      587    1344     2.697 mWb
 *      0.60     0.2344      575    2021     2.727 mWb
 *
 * Spread about 2%. Kt = 1.5 * p * lambda_m = 0.080 N.m per amp of iq.
 *
 * This replaces 3.063 mWb, which was DERIVED from the nameplate KV90 as
 * 60/(2*pi*sqrt(3)*p*Kv) and was 14% too high. The measurement corresponds to
 * Kv = 103 rather than 90, which is an ordinary amount for a hobby motor
 * nameplate to be out by.
 *
 * How that error survived is worth recording, because the check that should
 * have caught it did not. Reading g_foc.vq_ff_pm back off the target and
 * finding it matched w_e*lambda_m/Vbus to 1% proved nothing at all: the
 * firmware COMPUTES vq_ff from this constant, so that comparison can only
 * ever confirm the arithmetic, never the number. A parameter that describes
 * the motor has to be measured against the MOTOR - here, against the voltage
 * the loop demands when it is left to find that voltage by itself.
 *
 * What the error cost: at 2133 rad/s the feedforward came out at 0.276 of the
 * bus against a vmax of 0.25, so the feedforward ALONE exceeded the entire
 * modulation ceiling. The 0.82 V of excess drives 9.6 A through an 85 mohm
 * winding on its own. See the anti-windup comment in foc.c for the second
 * half of that failure.
 *
 * For the EMRAX this is the same warning as the bandwidth derivation below:
 * measure lambda_m, do not take it from a nameplate. The back-EMF term there
 * is 58% of the bus, so 14% of it is 8% of the entire supply. */
#define FOC_LAMBDA_M_WB     2.68e-3f

/* ---- deadtime compensation ---------------------------------------------
 *
 * During the gate-driver deadtime both devices in a leg are off and the phase
 * is clamped by whichever body diode the current is already flowing through.
 * The applied volt-seconds therefore differ from the commanded ones by a
 * fixed amount whose SIGN follows the phase current:
 *
 *     v_error = -sign(i_phase) * t_d * f_sw * Vbus
 *
 * so adding +sign(i_phase) * t_d * f_sw to each phase duty cancels it.
 *
 * On this board the deadtime is not the MCU's - HRTIM inserts none, the
 * UCC21330's own 20 kohm RDT sets it (HARDWARE_NOTES section 7):
 *
 *     t_d = 8.6 * 20 + 13 = 185 ns
 *     t_d * f_sw = 185e-9 * 30000 = 0.0056 = 5.6 per-mille of bus
 *
 * The work plan expects this to be invisible at 48 V and only matter at
 * 600 V. That is wrong for THIS machine, and the reason is the winding, not
 * the voltage: 5.6 per-mille of a 22.4 V bus is 0.124 V, and across an 85
 * mohm phase resistance that is 1.5 A - against a 1.0 A command. Low
 * impedance is what makes a small voltage error a large current error.
 *
 * Measured before compensating, during a reversal at ~320 Hz electrical, the
 * iq ripple has its largest component at 1917 Hz = 6.0 x f_e with 105 mA
 * amplitude. Sixth harmonic is the deadtime signature - the error is a square
 * wave in phase with sign(i) on each of three phases - and 1917 Hz is nearly
 * twice the 1 kHz loop bandwidth, so the current loop has almost no gain left
 * to reject it. Open-loop the same 0.124 V into |R + jwL| = 0.66 ohm at that
 * frequency would give 188 mA, so the loop is removing roughly half.
 *
 * dtc_pm is SIGNED and runtime-settable rather than compiled in, because the
 * sign depends on the polarity of the current sense and the gate drive
 * together, and getting it backwards DOUBLES the distortion instead of
 * cancelling it.
 *
 * ---- measured, and it does not pay here ----
 *
 * Swept on the bench at ~247 Hz electrical, 0.5 A, two independent runs,
 * reading the 6th-harmonic amplitude of iq and the total ripple rms:
 *
 *     dtc_pm     6th harmonic (mA)      ripple rms (mA)
 *       -6         87  /  105             76  /  82
 *        0        104  /  101             82  /  86
 *       +3        118  /  121            105  / 101
 *       +9        111  /  137            143  / 197
 *
 * The WRONG sign reproduces perfectly: positive dtc_pm makes both the 6th
 * harmonic and the total ripple monotonically worse, which is the expected
 * signature of adding the deadtime error to itself instead of subtracting it.
 *
 * The RIGHT sign buys nothing measurable. -6 came out 87 mA in one run and
 * 105 mA in the other against 104 and 101 with compensation off - a spread as
 * large as the effect, so the first run's apparent 17% improvement was noise.
 *
 * So the ~100 mA of 6th harmonic on this machine is NOT mostly deadtime.
 * Theory says 185 ns should contribute 188 mA open-loop, or roughly 105 mA
 * after loop rejection; the measured contribution is indistinguishable from
 * zero. Something in that chain over-predicts - most likely the effective
 * diode-conduction window is much shorter than the driver's nominal RDT time
 * at these currents. What is left is almost certainly the motor's own
 * back-EMF harmonic content and cogging, which no amount of deadtime
 * compensation touches.
 *
 * Kept, defaulted OFF, because none of the above transfers to the HV
 * inverter: 600 V with a ~500 ns deadtime is 9 V per phase, against an EMRAX
 * winding of 23 mohm. There the term is not negotiable. The code, the
 * hysteresis reasoning and the sign-determination method are what this phase
 * produced; the bench simply cannot show the benefit. */
#define FOC_DTC_PM_THEORY   6      /* t_d * f_sw = 5.6 per-mille, rounded */

/* Hysteresis band around zero phase current, mA.
 *
 * The compensation's sign comes from the sign of the phase current, and near
 * a zero crossing that sign is whatever the noise says. position.h records
 * the current loop's own noise floor as ~148 mA pp from ADC noise and PWM
 * ripple, so a band narrower than that would have the compensation chattering
 * between +dtc and -dtc at the noise frequency - injecting exactly the kind of
 * broadband disturbance it exists to remove. 150 mA covers that floor and is
 * 15% of a 1 A command, so the uncompensated wedge around each zero crossing
 * stays small. */
#define FOC_DTC_HYST_MA     150

/* Sanity window for the measured bus. Outside it the reading is not trusted
 * and the gains are left alone: a bus of 0 would otherwise divide to infinity
 * and put NaN into the duty registers, which is the single worst thing that
 * can reach a motor bridge. */
#define FOC_VBUS_MIN_MV     6000
#define FOC_VBUS_MAX_MV     60000

/* Modulation ceiling, as a fraction of the available bus voltage.
 *
 * This caps how much voltage the loop may apply, which in turn caps speed:
 * back-EMF rises with rpm and the loop saturates once it can no longer push
 * current against it. It does NOT set the current - the PI does that - so
 * raising it is safe as long as the current loop is behaving.
 * 0.10 limited the motor to ~68 rpm. */
#define FOC_VMAX_DEFAULT    0.25f

/* Electrical offset in encoder counts (32768 = 360 deg electrical).
 *
 * Should be ZERO. openloop.c now builds phases as cos(theta), matching foc.c's
 * inverse Clarke (vu = valpha), so "electrical zero" means the same thing in
 * both modules - and the A1333 ZERO_OFFSET was recalibrated against a
 * FOC-convention vector, putting the alignment in the sensor itself.
 *
 * It was 8192 (90 deg) before that: openloop used sin(theta) and therefore
 * held the rotor 90 degrees away during the original calibration, so FOC's iq
 * produced pure d-axis force and the motor would not turn at any current.
 * Kept as a tunable in case the convention ever drifts again. */
#define FOC_ELEC_OFFSET_DEFAULT   0

typedef struct {
  /* Commands */
  float    id_ref;        /* A, normally 0 for a surface-magnet motor */
  float    iq_ref;        /* A, torque-producing                      */

  /* Measured */
  float    iu, iv, iw;    /* A                                        */
  float    ialpha, ibeta;
  float    id, iq;

  /* Output */
  float    vd, vq;
  float    valpha, vbeta;
  float    duty_u, duty_v, duty_w;   /* 0..1 */

  /* PI state */
  float    id_integ, iq_integ;
  float    kp, ki, vmax;

  /* Angle */
  /* Electrical offset in encoder counts (32768 = 360 deg electrical).
   * The A1333 ZERO_OFFSET aligns the encoder to whatever vector was applied
   * during calibration, but openloop.c builds phases as sin(theta) while FOC's
   * inverse Clarke uses vu = valpha (a cosine convention). Those differ by 90
   * degrees, so a correction belongs here until the calibration is redone
   * against a FOC-convention vector. 8192 counts = 90 deg electrical. */
  int32_t  elec_offset;

  uint16_t enc_raw;
  uint16_t elec_counts;   /* 0..32767 electrical                      */
  float    sin_e, cos_e;

  /* Diagnostics */
  uint32_t updates;
  uint32_t isr_cycles;    /* DWT cycles for the last ISR              */
  uint32_t isr_max;       /* worst case seen                          */
  uint32_t enabled;

  /* Integer mirrors - the SWD reader has no float support. */
  int32_t  id_ma, iq_ma;
  int32_t  iu_ma, iw_ma;
  int32_t  vd_mv, vq_mv;         /* as per-mille of bus, x1000 */
  int32_t  duty_u_pm, duty_v_pm, duty_w_pm;
  int32_t  iq_ref_ma, id_ref_ma;
  int32_t  elec_deg_x10;      /* electrical angle, tenths of a degree */
  int32_t  vmax_pm;           /* modulation ceiling, per-mille        */
  int32_t  vmag_pm;           /* applied vector magnitude, per-mille  */

  /* Bus-voltage gain tracking. Appended at the END of the struct on purpose:
   * tools/foc_dash.sh addresses the mirrors above by fixed byte offset. */
  uint32_t vbus_track;        /* 1 = rescale kp/ki from the measured bus */
  int32_t  vbus_used_mv;      /* bus the live gains were computed for    */
  int32_t  kp_x10000;         /* live kp, for SWD readout                */
  int32_t  ki_x100;           /* live ki, for SWD readout                */

  /* Decimation counter for the integer mirrors below the control maths.
   * Appended at the END so the fixed byte offsets the tools use do not move. */
  uint32_t mirror_div;

  /* Transport-delay compensation. Appended at the END for the same reason. */
  float    omega_e;           /* electrical rad/s, from the position loop  */
  uint32_t delay_comp;        /* 1 = advance the inverse Park (default on) */
  int32_t  omega_e_rads_x10;  /* omega_e for SWD readout, tenths of rad/s  */
  int32_t  theta_adv_deg_x10; /* applied advance, tenths of a degree       */

  /* Cross-coupling decoupling and back-EMF feedforward. At the END as ever. */
  uint32_t decouple;          /* 1 = apply the feedforward (default on)    */
  float    inv_vbus;          /* 1/Vbus, volts^-1. See FOC_SetGainsForVbus */
  int32_t  vd_ff_pm;          /* d-axis feedforward, per-mille of bus      */
  int32_t  vq_ff_pm;          /* q-axis feedforward, per-mille of bus      */

  /* Deadtime compensation. At the END like everything else. */
  int32_t  dtc_pm;            /* per-mille of bus added per phase, SIGNED;
                               * 0 disables. See FOC_DTC_PM_THEORY.          */
  int32_t  dtc_hyst_ma;       /* +/- band around zero current where no
                               * compensation is applied                     */
} FocState_t;

void FOC_Init(FocState_t *f);

/* Recompute kp/ki for the measured bus voltage, holding the closed-loop
 * bandwidth at FOC_BW_RADS regardless of what the supply is set to.
 *
 * Cheap enough to call from the main loop every time the bus is sampled; it
 * does nothing unless f->vbus_track is set and the reading is inside the
 * sanity window. Safe to call while the loop is running - kp and ki are read
 * afresh each ISR, and a single-word float store cannot tear on this core. */
void FOC_SetGainsForVbus(FocState_t *f, int32_t vbus_mv);

/* One control step. iu_ma / iw_ma are signed milliamps, enc_raw is the 15-bit
 * encoder reading. Writes the resulting duties into the state; the caller
 * applies them.
 *
 * vel_mech_rads is the rotor's MECHANICAL velocity in rad/s - hand it
 * g_pos.vel_rads, the estimate the position loop already maintains. It is a
 * parameter rather than a second estimator on purpose: two velocity estimates
 * that can disagree are worse than one that is imperfect, and this one is
 * differentiated from the same encoder counts the angle comes from. foc.c
 * converts to electrical; pole pairs are its business, not the position
 * loop's. */
void FOC_Update(FocState_t *f, int32_t iu_ma, int32_t iw_ma, uint16_t enc_raw,
                float vel_mech_rads);

/* Zero the integrators - call whenever the loop is (re)enabled so stale
 * windup cannot kick the first output. */
void FOC_Reset(FocState_t *f);

#ifdef __cplusplus
}
#endif

#endif /* FOC_H */
