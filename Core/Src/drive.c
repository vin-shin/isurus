/**
  ******************************************************************************
  * @file    drive.c
  * @brief   Drive state machine and the one safe-state path.
  ******************************************************************************
  */

#include "drive.h"
#include "main.h"
#include "motor_pwm.h"
#include "foc.h"
#include "position.h"
#include "csense.h"
#include "encoder.h"
#include "limits.h"

volatile DriveTelem_t g_drive = {0};

extern volatile FocState_t    g_foc;
extern volatile PosState_t    g_pos;
extern volatile CSenseTelem_t g_cs;

/* Mirror of DRIVE_FAULT for the SWD tools and the CAN status flag, which both
 * predate this file and address g_faulted directly. It is derived here and
 * written nowhere else, so the state machine stays the single authority even
 * though the old name survives. */
extern volatile uint32_t g_faulted;

static uint32_t s_state_t0 = 0U;
static uint32_t s_selftest_tick = 0U;

/* How often SELFTEST re-checks while waiting for the bus to come up. */
#define DRIVE_SELFTEST_RETRY_MS   500U

const char *Drive_StateName(uint32_t s)
{
  switch (s)
  {
    case DRIVE_INIT:     return "INIT";
    case DRIVE_SELFTEST: return "SELFTEST";
    case DRIVE_READY:    return "READY";
    case DRIVE_RUN:      return "RUN";
    case DRIVE_FAULT:    return "FAULT";
    default:             return "?";
  }
}

/* The only place g_drive.state is written, and the only place the bridge and
 * the loop-enable flags are touched. Everything else asks for a transition. */
static void Drive_Enter(DriveState_t s)
{
  if (g_drive.state == (uint32_t)s) { return; }

  g_drive.state = (uint32_t)s;
  g_drive.state_entries++;
  s_state_t0 = HAL_GetTick();

  if (s == DRIVE_RUN)
  {
    /* Order matters and is the same one main.c enforced before: duty to zero
     * first so nothing is latched from a previous run, then the HRTIM
     * outputs, then the gate drivers - because the gate line is the only one
     * that actually turns FETs on. */
    MotorPwm_SetDutyPermille(0, 0, 0);
    MotorPwm_EnableOutputs();
    MotorPwm_GateEnable();

    g_foc.id_ref = 0.0f;
    g_foc.iq_ref = 0.0f;

    /* Clear integrator windup on every arm. foc.h has always said to do this
     * whenever the loop is re-enabled; the old bridge-up path set the
     * references to zero but left the integrators holding whatever they had
     * when the bridge last went down, so the first duty out of a re-arm was
     * built on stale state. */
    FOC_Reset((FocState_t *)&g_foc);
    g_foc.enabled = 1U;
  }
  else
  {
    /* Every other state has the bridge down. Stand the control loop down
     * before the hardware so the last thing the ISR does is not to write a
     * duty into a bridge that is being disabled underneath it. */
    g_foc.enabled = 0U;
    g_foc.iq_ref  = 0.0f;
    g_foc.id_ref  = 0.0f;
    g_pos.enabled = 0U;
    MotorPwm_GateDisable();
    MotorPwm_DisableOutputs();
  }

  g_faulted = (s == DRIVE_FAULT) ? 1U : 0U;
}

void Drive_SafeState(void)
{
  /* Hardware first, unconditionally, before any bookkeeping. If this is being
   * called from a fault handler then the bookkeeping may not survive, and the
   * bridge going down is the part that matters. */
  MotorPwm_EmergencyStop();

  g_foc.enabled = 0U;
  g_foc.iq_ref  = 0.0f;
  g_foc.id_ref  = 0.0f;
  g_pos.enabled = 0U;
}

DriveSafeMode_t Drive_ChooseSafeState(DriveFault_t cause, float omega_e,
                                      int32_t vbus_mv)
{
  /* Anything that implicates the BRIDGE gets freewheel, at any speed.
   *
   * Active short circuit needs the gate drivers live and all three low-side
   * devices deliberately on - which is precisely what must not be commanded
   * when the reason for the fault is that a device or a gate driver may
   * already be misbehaving. An ASC into a bridge with one leg shorted is a
   * shoot-through, so the state that requires the least of the hardware wins
   * whenever the hardware is what is in doubt. */
  if ((cause == DRIVE_FAULT_OVERCURRENT) ||
      (cause == DRIVE_FAULT_WATCHDOG)    ||
      (cause == DRIVE_FAULT_CSENSE))
  {
    return DRIVE_SAFE_FREEWHEEL;
  }

  /* Otherwise it is a speed question. Freewheel is fine while the back-EMF
   * cannot push current into the pack, i.e. while
   *
   *     sqrt(3) * w_e * lambda_m  <  Vbus
   *
   * Below that crossover freewheel costs nothing and dissipates nothing;
   * above it the body diodes become an uncontrolled rectifier feeding a pack
   * that may already be at its ceiling. */
  if (vbus_mv < LIM_VBUS_MIN_MV) { return DRIVE_SAFE_FREEWHEEL; }

  {
    float vbus  = (float)vbus_mv * 0.001f;
    float w_thr = vbus / (1.73205081f * FOC_LAMBDA_M_WB);
    float w     = (omega_e < 0.0f) ? -omega_e : omega_e;

    g_drive.asc_threshold = (int32_t)w_thr;

    if (w > w_thr) { return DRIVE_SAFE_ASC; }
  }

  return DRIVE_SAFE_FREEWHEEL;
}

void Drive_Fault(DriveFault_t cause)
{
  /* Freewheel FIRST, unconditionally, then reconsider.
   *
   * Whatever the analysis concludes, the bridge must be off within
   * microseconds of getting here, and freewheel is the state that needs
   * nothing to still be working. Choosing before acting would put a float
   * divide and a branch between the fault and the FETs. */
  Drive_SafeState();

  g_drive.fault_count++;

  /* Latch the FIRST cause only. A fault cascade - overcurrent tripping, the
   * bus sagging, the encoder then reading nonsense - would otherwise leave
   * the last symptom in the register instead of the thing that started it. */
  if (g_drive.state != (uint32_t)DRIVE_FAULT)
  {
    g_drive.fault = (uint32_t)cause;
    Drive_Enter(DRIVE_FAULT);
  }

  /* Safe-state choice goes AFTER the state entry, not before it.
   *
   * Drive_Enter drops the gate drivers on the way into any non-RUN state,
   * which is right for every other transition and would silently undo an
   * active short circuit if this ran first. That ordering bug is invisible on
   * this bench - the ASC branch below cannot be reached at 24 V - so it is
   * called out here rather than left for the HV board to discover. */
  {
    DriveSafeMode_t m = Drive_ChooseSafeState(cause, g_foc.omega_e,
                                              (int32_t)g_cs.vbus_mv);
    g_drive.safe_mode = (uint32_t)m;

    if (m == DRIVE_SAFE_ASC)
    {
      /* All three low-side devices on. On this board that is what dropping
       * the HRTIM outputs while LEAVING the gate drivers enabled does - the
       * pins park low and, through the external inverter, the low-side
       * devices conduct. See motor_pwm.c.
       *
       * NOT REACHABLE ON THIS BENCH, and deliberately so. The crossover here
       * is Vbus/(sqrt(3)*lambda_m) = 5170 rad/s at 24 V, and the motor is
       * voltage-limited to about 1820 rad/s electrical, so this branch cannot
       * be entered at 48 V. That is the correct outcome rather than a gap in
       * coverage - an ASC at bench top speed would draw 37.4 A through +/-40 A
       * sensors and a 22 A motor. The decision logic is what is being
       * developed here; the action itself waits for HV hardware. */
      MotorPwm_GateEnable();
      MotorPwm_DisableOutputs();
    }
  }
}

void Drive_Init(void)
{
  g_drive.state         = (uint32_t)DRIVE_INIT;
  g_drive.fault         = (uint32_t)DRIVE_FAULT_NONE;
  g_drive.fault_count   = 0U;
  g_drive.clears        = 0U;
  g_drive.selftest_fail = (uint32_t)DRIVE_FAULT_NONE;
  g_drive.state_entries = 0U;
  g_drive.ms_in_state   = 0U;
  s_state_t0            = HAL_GetTick();

  Drive_SafeState();
}

void Drive_SelfTest(void)
{
  DriveFault_t bad = DRIVE_FAULT_NONE;

  Drive_Enter(DRIVE_SELFTEST);

  /* 1. Current-sense zero. CSense_Init has already averaged 256 samples at
   *    rest; this checks the result is credible rather than re-measuring it.
   *    A zero far from mid-scale means an open sensor, a dead OPAMP, or a
   *    calibration taken while the shaft was turning - all of which inject a
   *    large constant error straight into the Park transform, where it looks
   *    like a real current the loop then tries to correct. */
  g_drive.cs_u_zero = (int32_t)g_cs.u_zero;
  g_drive.cs_w_zero = (int32_t)g_cs.w_zero;
  {
    int32_t du = g_drive.cs_u_zero - 2048;
    int32_t dw = g_drive.cs_w_zero - 2048;
    if (du < 0) { du = -du; }
    if (dw < 0) { dw = -dw; }
    if ((du > DRIVE_CS_ZERO_TOL_CODES) || (dw > DRIVE_CS_ZERO_TOL_CODES))
    {
      bad = DRIVE_FAULT_CSENSE;
    }
  }

  /* 2. Encoder answers, with a plausible angle.
   *
   *    NOTE: this does not yet check the A1333's frame CRC, because nothing
   *    in this firmware does - that is phase 3. Until then a corrupted frame
   *    that is not 0xFFFF passes here exactly as it passes everywhere else,
   *    so treat this as "the SPI link is alive", not "the angle is right". */
  if (bad == DRIVE_FAULT_NONE)
  {
    uint16_t raw = 0U;
    if (Encoder_ReadAngle(&raw) != ENC_OK) { bad = DRIVE_FAULT_ENCODER; }
    else if (raw == 0xFFFFU)               { bad = DRIVE_FAULT_ENCODER; }
  }

  /* 3. Bus inside the window the drive was designed for. Under-voltage
   *    matters as much as over: the current-loop gains are computed as
   *    something/Vbus, so a collapsing bus raises loop gain until the loop
   *    oscillates - see the comment on FOC_KP_DEFAULT. */
  if (bad == DRIVE_FAULT_NONE)
  {
    if (CSense_ReadVbus((CSenseTelem_t *)&g_cs) != 0)
    {
      bad = DRIVE_FAULT_SELFTEST;
    }
    else
    {
      g_drive.vbus_mv = (int32_t)g_cs.vbus_mv;
      if (g_drive.vbus_mv > LIM_VBUS_MAX_MV)      { bad = DRIVE_FAULT_OVERVOLTAGE; }
      else if (g_drive.vbus_mv < LIM_VBUS_MIN_MV) { bad = DRIVE_FAULT_UNDERVOLT;   }
    }
  }

  g_drive.selftest_fail = (uint32_t)bad;
  s_selftest_tick = HAL_GetTick();

  /* An undervoltage bus is a PRECONDITION that is not met yet, not a fault to
   * latch. The commonest way to see it is the bench supply simply not being
   * switched on, and latching there means the drive still refuses to arm
   * after power arrives until someone sends a clear - which is friction with
   * no safety value, since a dead bus cannot hurt anything.
   *
   * So this one case stays in SELFTEST and retries. Everything else latches:
   * a bad current-sense zero or a silent encoder will not fix itself, and
   * quietly retrying those would turn a hard fault into an intermittent one.
   *
   * Note this is only the STARTUP check. An undervoltage that appears while
   * running is a different event on a different path and is not softened
   * here. */
  if (bad == DRIVE_FAULT_UNDERVOLT)
  {
    return;   /* stay in SELFTEST; Drive_Step retries */
  }

  if (bad != DRIVE_FAULT_NONE) { Drive_Fault(bad); }
  else                         { Drive_Enter(DRIVE_READY); }
}

int32_t Drive_Arm(void)
{
  /* Only from READY. This is the check that makes a CAN enable arriving
   * during SELFTEST, or after a fault, do nothing instead of energising. */
  if (g_drive.state != (uint32_t)DRIVE_READY) { return -1; }
  Drive_Enter(DRIVE_RUN);
  return 0;
}

int32_t Drive_Disarm(void)
{
  if (g_drive.state != (uint32_t)DRIVE_RUN) { return -1; }
  Drive_Enter(DRIVE_READY);
  return 0;
}

void Drive_ClearFault(void)
{
  if (g_drive.state != (uint32_t)DRIVE_FAULT) { return; }

  g_drive.clears++;
  g_drive.fault = (uint32_t)DRIVE_FAULT_NONE;

  /* Back to SELFTEST, never straight to READY or RUN. Whatever tripped may
   * still be true - a shorted phase does not repair itself because someone
   * pressed reset - and the checks are what establish that it is not. */
  Drive_SelfTest();
}

void Drive_Step(uint32_t now_ms)
{
  g_drive.ms_in_state = now_ms - s_state_t0;

  /* INIT is left automatically once the caller has finished bringing
   * peripherals up and started calling this. Every other transition is
   * requested explicitly - notably FAULT, which has no timer out of it. */
  if (g_drive.state == (uint32_t)DRIVE_INIT)
  {
    Drive_SelfTest();
  }
  else if (g_drive.state == (uint32_t)DRIVE_SELFTEST)
  {
    /* Only reachable when the last attempt stopped on an undervoltage bus -
     * every other outcome leaves SELFTEST immediately. Retry on a timer so
     * the drive becomes READY on its own once the supply arrives. */
    if ((now_ms - s_selftest_tick) >= DRIVE_SELFTEST_RETRY_MS)
    {
      Drive_SelfTest();
    }
  }
}

/* ---- windowed watchdog ------------------------------------------------- */

void Drive_WatchdogStart(void)
{
  RCC->APB1ENR1 |= RCC_APB1ENR1_WWDGEN;
  (void)RCC->APB1ENR1;   /* read back so CFR below cannot beat the clock up */

  /* Freeze the counter whenever the core is halted by the debugger.
   *
   * Without this the watchdog is unusable here: every script in tools/ halts
   * the core to read a coherent sample, and a halted core stops feeding, so
   * the board would reset in the middle of every measurement. The freeze is a
   * DEBUG feature only - it does nothing when no debugger is attached, so it
   * does not weaken the watchdog in the field. */
  DBGMCU->APB1FZR1 |= DBGMCU_APB1FZR1_DBG_WWDG_STOP;

  /* Window first, then activate. Writing CFR after WDGA is set is legal but
   * pointless-ordering it this way means the very first counted interval is
   * already governed by the window. WDGTB = 0: 32 us per count. */
  WWDG->CFR = (0U << WWDG_CFR_WDGTB_Pos) | DRIVE_WWDG_WINDOW;

  /* WDGA is set here and can only be cleared by a reset - that is the point
   * of it. From this instant the first feed must arrive within 2.05 ms. */
  WWDG->CR  = WWDG_CR_WDGA | DRIVE_WWDG_RELOAD;
}

void Drive_WatchdogFeed(void)
{
  static uint32_t s_div = 0U;

  if (++s_div < DRIVE_WWDG_FEED_TICKS) { return; }
  s_div = 0U;

  /* Single store, no read-modify-write: WDGA cannot be cleared by software
   * anyway, so writing it back is free and keeps this to one instruction's
   * worth of work in the control ISR. */
  WWDG->CR = WWDG_CR_WDGA | DRIVE_WWDG_RELOAD;
}

uint32_t Drive_WatchdogTripped(void)
{
  uint32_t tripped = (RCC->CSR & RCC_CSR_WWDGRSTF) ? 1U : 0U;

  /* Clear the whole reset-cause set. If this is not done, the flag persists
   * across every subsequent reset and every boot afterwards looks like a
   * watchdog reset - which is exactly the sort of stale evidence that sends a
   * debugging session in the wrong direction. HARDWARE_NOTES section 9 leans
   * on RCC_CSR being trustworthy. */
  RCC->CSR |= RCC_CSR_RMVF;

  return tripped;
}
