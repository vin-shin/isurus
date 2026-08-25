/**
  ******************************************************************************
  * @file    drive.h
  * @brief   Drive state machine and the one safe-state path.
  *
  *          Before this, "is the drive allowed to move" was spread across
  *          g_foc.enabled, g_pos.enabled, s_bridge_up, g_cmd.outputs_en,
  *          g_cmd.gate_en and g_faulted - six places, no single answer, and
  *          nothing that made an illegal combination impossible. That is
  *          survivable on a bench where the worst case is a twitch. It is not
  *          a structure to put 600 V behind.
  *
  *              INIT -> SELFTEST -> READY -> RUN
  *                          |         |       |
  *                          +---------+-------+---> FAULT (latching)
  *
  *          The state is the authority. The old flags still exist because the
  *          control ISR and the SWD tools read them, but they are now OUTPUTS
  *          of this machine rather than independent switches - only
  *          Drive_Enter() writes them.
  *
  *          FAULT latches. It is left only by an explicit Drive_ClearFault(),
  *          and that returns to SELFTEST rather than READY, so nothing gets
  *          back to RUN without re-passing the checks. The previous behaviour
  *          re-armed the bridge by itself a few seconds after an overcurrent;
  *          see Drive_ClearFault for why that is gone.
  ******************************************************************************
  */
#ifndef DRIVE_H
#define DRIVE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

typedef enum {
  DRIVE_INIT     = 0U,   /* peripherals coming up; bridge cannot be armed   */
  DRIVE_SELFTEST = 1U,   /* checking sensors and bus before allowing arm    */
  DRIVE_READY    = 2U,   /* checks passed, bridge OFF, waiting to be told   */
  DRIVE_RUN      = 3U,   /* bridge live, control loop driving               */
  DRIVE_FAULT    = 4U    /* latched, bridge safe, needs an explicit clear   */
} DriveState_t;

/* Latched cause. One value, not a bitmask: what matters after the fact is
 * what tripped FIRST, and a bitmask of everything that went wrong afterwards
 * buries it. Subsequent faults are counted, not merged. */
typedef enum {
  DRIVE_FAULT_NONE        = 0U,
  DRIVE_FAULT_OVERCURRENT = 1U,
  DRIVE_FAULT_OVERVOLTAGE = 2U,
  DRIVE_FAULT_UNDERVOLT   = 3U,
  DRIVE_FAULT_ENCODER     = 4U,   /* no response, or an implausible angle   */
  DRIVE_FAULT_CSENSE      = 5U,   /* current-sense zero out of tolerance    */
  DRIVE_FAULT_SELFTEST    = 6U,   /* a check failed that has no finer cause */
  DRIVE_FAULT_WATCHDOG    = 7U,   /* control ISR stopped feeding the WWDG   */
  DRIVE_FAULT_COMMAND     = 8U    /* a command was refused as implausible   */
} DriveFault_t;

/* Self-test tolerance on the current-sense zero.
 *
 * CSense_Init averages 256 samples at rest to find each sensor's zero, and
 * section 6 of HARDWARE_NOTES records the measured codes as 2037 and 2039
 * against an ideal half-rail of 2048. So a healthy sensor sits within about
 * 15 codes of mid-scale. 200 codes is ~4.9 A of apparent standing current and
 * is far outside anything a working sensor does at rest - it means an open
 * sensor, a dead OPAMP, or a calibration taken while the motor was moving,
 * all of which put a large constant error into the Park transform. */
#define DRIVE_CS_ZERO_TOL_CODES   200

/* How long SELFTEST may take before it gives up, ms. Generous: the checks
 * themselves are fast, and the only reason to wait is a Vbus reading that has
 * not arrived yet because the supply is still ramping. */
#define DRIVE_SELFTEST_TIMEOUT_MS 2000U

typedef struct {
  /* ---- read these over SWD / CAN --------------------------- offset ---- */
  uint32_t state;         /*  0  DriveState_t                              */
  uint32_t fault;         /*  4  DriveFault_t, latched at the FIRST trip   */
  uint32_t fault_count;   /*  8  faults since boot, including re-trips     */
  uint32_t clears;        /* 12  explicit fault clears since boot          */
  uint32_t selftest_fail; /* 16  DriveFault_t of the last self-test failure */
  uint32_t state_entries; /* 20  state transitions, for spotting churn     */
  int32_t  cs_u_zero;     /* 24  self-test snapshot, ADC codes             */
  int32_t  cs_w_zero;     /* 28                                            */
  int32_t  vbus_mv;       /* 32  bus at the time of the last self-test     */
  uint32_t ms_in_state;   /* 36  updated by Drive_Step                     */
  uint32_t safe_mode;     /* 40  DriveSafeMode_t chosen at the last fault  */
  int32_t  asc_threshold; /* 44  w_e above which freewheel back-charges,
                           *     rad/s, at the bus seen at that fault      */
} DriveTelem_t;

extern volatile DriveTelem_t g_drive;

/* Which safe state to fall into. See HARDWARE_NOTES section 11 for the
 * analysis; the short version is that the correct answer is speed-dependent
 * and neither choice is free.
 *
 *   FREEWHEEL - every FET off. Nothing is dissipated in the machine, but a
 *               spinning PMSM's back-EMF then rectifies through the body
 *               diodes into the DC link, and once the line-to-line peak
 *               exceeds the pack that is an uncontrolled back-charge.
 *   ASC       - all three low-side devices on, a symmetrical short. The
 *               back-EMF is clamped into the winding instead of the pack, at
 *               the cost of sinking roughly lambda_m/Ld continuously.
 */
typedef enum {
  DRIVE_SAFE_FREEWHEEL = 0U,
  DRIVE_SAFE_ASC       = 1U
} DriveSafeMode_t;

/* Pick the safe state for a given fault, speed and bus. Pure decision, no
 * side effects, so it can be reasoned about and tested on its own.
 *
 * The threshold is computed rather than tabulated: freewheel back-charges
 * once sqrt(3) * w_e * lambda_m exceeds Vbus, so the crossover is
 * Vbus / (sqrt(3) * lambda_m) and it moves with the bus as the pack sags. */
DriveSafeMode_t Drive_ChooseSafeState(DriveFault_t cause, float omega_e,
                                      int32_t vbus_mv);

/* Put the bridge in its safe state and latch a fault. ISR-SAFE and callable
 * from any state, including from a fault handler with a corrupted stack: the
 * hardware part is MotorPwm_EmergencyStop, which is two single-store register
 * writes and takes no locks.
 *
 * Calling this when already in FAULT does not overwrite the original cause -
 * it counts. The first thing that went wrong is the useful one. */
void Drive_Fault(DriveFault_t cause);

/* Bridge to its safe state WITHOUT latching a fault - the disarm path. Also
 * ISR-safe. Use Drive_Fault for anything abnormal. */
void Drive_SafeState(void);

void Drive_Init(void);

/* Run the self-test and enter READY or FAULT. Blocking, main-loop only: it
 * reads the encoder over SPI and samples the bus, neither of which belongs in
 * an ISR. */
void Drive_SelfTest(void);

/* Ask for the bridge. Returns 0 if the request was accepted.
 *
 * Refuses from anywhere except READY (to arm) or RUN (to disarm), which is
 * what stops a CAN frame arriving mid-self-test or after a fault from
 * energising anything. */
int32_t Drive_Arm(void);
int32_t Drive_Disarm(void);

/* The only exit from FAULT. Goes to SELFTEST, not READY: whatever tripped may
 * still be true, and the checks are what establish it is not.
 *
 * There is deliberately no timer that does this by itself. The previous code
 * re-armed the bridge OC_RETRY_MS after an overcurrent, up to OC_MAX_RETRIES
 * times, which is a reasonable bench convenience and an unreasonable thing to
 * ship on a car - a drive that re-energises itself into a genuine short is
 * how hardware dies, and FSAE rules require a latched shutdown to stay
 * latched until a driver acts. */
void Drive_ClearFault(void);

/* Main-loop housekeeping: state timing, and the automatic INIT -> SELFTEST
 * transition. Cheap; call it every pass. */
void Drive_Step(uint32_t now_ms);

/* ---- windowed watchdog ------------------------------------------------
 *
 * Fed from the CONTROL ISR, not the main loop, and that is the whole point.
 * A watchdog kicked by a main loop proves the main loop is alive, which is
 * not the thing that matters here: the main loop can be running perfectly
 * while the 30 kHz ISR is starved, and a starved ISR leaves whatever duty it
 * last wrote sitting on the bridge indefinitely. The fed-from-ISR version
 * catches exactly that.
 *
 * Windowed rather than IWDG because the window catches the opposite failure
 * too. Feeding EARLY is also a reset, so an ISR firing faster than it should
 * - a mis-programmed HRTIM period, a repetition counter set wrong, a clock
 * tree that came up at the wrong frequency - trips it as well. A free-running
 * watchdog would happily accept a control loop running at twice its intended
 * rate, which is a real way to get double the commanded current.
 *
 * Numbers, at PCLK1 = 128 MHz with APB1 undivided:
 *
 *   count clock = PCLK1 / 4096 / 2^WDGTB = 31250 Hz at WDGTB = 0
 *               = 32 us per count
 *   fed every DRIVE_WWDG_FEED_TICKS = 8 control periods = 266.7 us = 8.3 counts
 *   reload T = 0x7F, window W = 0x79
 *
 *   too slow: counter runs 0x7F -> 0x3F, 64 counts = 2.05 ms, about 61
 *             control periods of starvation before the reset.
 *   too fast: a refresh is only legal once the counter has fallen to W, so
 *             feeding sooner than 6 counts (192 us) resets. That is a control
 *             loop running more than 1.39x too fast.
 *
 * The 28% of early margin between nominal (8.3) and the window edge (6) is
 * what absorbs ISR jitter without nuisance-resetting. */
#define DRIVE_WWDG_FEED_TICKS   8U
#define DRIVE_WWDG_RELOAD       0x7FU
#define DRIVE_WWDG_WINDOW       0x79U

/* Start the watchdog. Call AFTER the control ISR is running - the first feed
 * has to arrive within 2 ms of this, so arming it earlier resets the board
 * during the rest of init. */
void Drive_WatchdogStart(void);

/* Feed. Call every control ISR tick; it decimates internally. */
void Drive_WatchdogFeed(void);

/* Did the last reset come from the watchdog? Reads and clears the RCC flags,
 * so call it once, early. */
uint32_t Drive_WatchdogTripped(void);

const char *Drive_StateName(uint32_t s);

#ifdef __cplusplus
}
#endif

#endif /* DRIVE_H */
