/**
  ******************************************************************************
  * @file    test/host/test_drive.c
  * @brief   Fault injection against drive.c's state machine.
  *
  *          drive.c compiled exactly as it ships, with everything it calls
  *          into supplied by drive_stubs.c. Each test breaks ONE input and
  *          asserts two things: that the machine latches FAULT with the right
  *          cause, and that the bridge actually came down. A cause that is
  *          right while the bridge stays live is not a pass.
  *
  *          Why the causes are asserted individually rather than just "it
  *          faulted": the whole point of DriveFault_t being one value and not
  *          a bitmask is that what tripped FIRST is recoverable afterwards,
  *          and the LED blinks that number at someone standing in front of the
  *          board. A machine that faults correctly but reports the wrong cause
  *          sends the next person after the wrong thing, which on a 600 V
  *          inverter is worse than a vague fault.
  *
  *          These are open-loop: they drive the sensors directly rather than
  *          through the PMSM model, because the paths under test - the
  *          self-test and the state machine - read the sensors directly and
  *          never see a rotor. Closed-loop injection against the control path
  *          is a different harness and pmsm.h already carries the flags for it.
  ******************************************************************************
  */

#include "drive.h"
#include "drive_stubs.h"
#include "limits.h"
#include "main.h"
#include "foc.h"
#include "encoder.h"

#include <stdio.h>
#include <string.h>

static int g_pass = 0, g_fail = 0;

static void check(int ok, const char *what, const char *detail)
{
  if (ok) { g_pass++; printf("  PASS  %s\n", what); }
  else    { g_fail++; printf("  FAIL  %s\n          %s\n", what, detail); }
}

/* Bring the drive up from cold with whatever the stub currently reports. */
static void boot(void)
{
  Drive_Init();
  Drive_Step(HAL_GetTick());     /* INIT -> SELFTEST -> READY or FAULT */
}

static const char *cause_name(uint32_t c)
{
  switch (c)
  {
    case DRIVE_FAULT_NONE:        return "NONE";
    case DRIVE_FAULT_OVERCURRENT: return "OVERCURRENT";
    case DRIVE_FAULT_OVERVOLTAGE: return "OVERVOLTAGE";
    case DRIVE_FAULT_UNDERVOLT:   return "UNDERVOLT";
    case DRIVE_FAULT_ENCODER:     return "ENCODER";
    case DRIVE_FAULT_CSENSE:      return "CSENSE";
    case DRIVE_FAULT_SELFTEST:    return "SELFTEST";
    case DRIVE_FAULT_WATCHDOG:    return "WATCHDOG";
    case DRIVE_FAULT_COMMAND:     return "COMMAND";
    default:                      return "?";
  }
}

/* Assert state + cause together, and report BOTH when either is wrong -
 * "expected FAULT/ENCODER, got READY/NONE" localises the failure immediately,
 * where "expected 4, got 2" needs the header open. */
static void expect(uint32_t state, uint32_t cause, const char *what)
{
  char d[192];
  snprintf(d, sizeof(d), "expected %s/%s, got %s/%s",
           Drive_StateName(state), cause_name(cause),
           Drive_StateName(g_drive.state), cause_name(g_drive.fault));
  check(g_drive.state == state && g_drive.fault == cause, what, d);
}

/* ---- a healthy board still arms ----------------------------------------- */

static void test_healthy_reaches_ready(void)
{
  Stub_Reset();
  boot();
  expect(DRIVE_READY, DRIVE_FAULT_NONE, "a healthy board self-tests to READY");
}

/* ---- the injected faults ------------------------------------------------ */

static void test_encoder_silent(void)
{
  Stub_Reset();
  g_stub.enc_status = (int)ENC_ERR_SPI;      /* dead SPI link */
  boot();
  expect(DRIVE_FAULT, DRIVE_FAULT_ENCODER, "a silent encoder faults as ENCODER");
}

static void test_encoder_dead_miso(void)
{
  Stub_Reset();
  g_stub.enc_raw = 0xFFFFU;                  /* what a floating MISO reads as */
  boot();
  expect(DRIVE_FAULT, DRIVE_FAULT_ENCODER,
         "an all-ones encoder frame faults as ENCODER");
}

static void test_current_sense_railed(void)
{
  Stub_Reset();
  boot();                                    /* healthy first, to isolate it */
  Drive_ClearFault();
  g_cs.u_zero = 2048U + DRIVE_CS_ZERO_TOL_CODES + 50U;
  Drive_SelfTest();
  expect(DRIVE_FAULT, DRIVE_FAULT_CSENSE,
         "a current-sense zero outside tolerance faults as CSENSE");
}

static void test_current_sense_just_inside_tolerance(void)
{
  Stub_Reset();
  g_cs.u_zero = 2048U + DRIVE_CS_ZERO_TOL_CODES - 1U;
  boot();
  expect(DRIVE_READY, DRIVE_FAULT_NONE,
         "a zero just inside tolerance is accepted, not faulted");
}

static void test_vbus_collapse_waits_rather_than_latching(void)
{
  Stub_Reset();
  g_stub.vbus_mv = (uint32_t)(LIM_VBUS_MIN_MV - 1000);
  boot();

  char d[160];
  snprintf(d, sizeof(d), "state %s, selftest_fail %s",
           Drive_StateName(g_drive.state), cause_name(g_drive.selftest_fail));
  /* Deliberately NOT a latched fault: the commonest cause is the bench supply
   * not being on yet, and latching there means the drive still refuses to arm
   * after power arrives. It has to record the reason all the same. */
  check(g_drive.state == DRIVE_SELFTEST &&
        g_drive.selftest_fail == (uint32_t)DRIVE_FAULT_UNDERVOLT,
        "an undervoltage bus stays in SELFTEST and records the reason", d);
}

static void test_vbus_recovers_on_its_own(void)
{
  Stub_Reset();
  g_stub.vbus_mv = (uint32_t)(LIM_VBUS_MIN_MV - 1000);
  boot();
  g_stub.vbus_mv = 22000U;                   /* supply arrives */
  Host_AdvanceTick(DRIVE_SELFTEST_TIMEOUT_MS + 1000U);
  Drive_Step(HAL_GetTick());
  expect(DRIVE_READY, DRIVE_FAULT_NONE,
         "the drive becomes READY by itself once the bus arrives");
}

static void test_vbus_overvoltage_latches(void)
{
  Stub_Reset();
  g_stub.vbus_mv = (uint32_t)(LIM_VBUS_MAX_MV + 1000);
  boot();
  expect(DRIVE_FAULT, DRIVE_FAULT_OVERVOLTAGE,
         "an overvoltage bus latches as OVERVOLTAGE");
}

static void test_vbus_read_failure(void)
{
  Stub_Reset();
  g_stub.vbus_rc = -1;                       /* the ADC read itself fails */
  boot();
  expect(DRIVE_FAULT, DRIVE_FAULT_SELFTEST,
         "a failed bus read faults as SELFTEST");
}

/* ---- the bridge, on every fault path ------------------------------------ */

static void test_every_fault_safes_the_bridge(void)
{
  struct { const char *name; int enc; uint16_t raw; uint32_t vbus; int rc; } cases[] = {
    { "silent encoder",  (int)ENC_ERR_SPI, 0x1234U, 22000U, 0  },
    { "dead MISO",       (int)ENC_OK,      0xFFFFU, 22000U, 0  },
    { "overvoltage",     (int)ENC_OK,      0x1234U, (uint32_t)(LIM_VBUS_MAX_MV + 1000), 0 },
    { "bus read fails",  (int)ENC_OK,      0x1234U, 22000U, -1 },
  };

  int all_safe = 1;
  char d[192] = "";
  for (unsigned i = 0; i < sizeof(cases) / sizeof(cases[0]); i++)
  {
    Stub_Reset();
    boot();                                  /* healthy, then arm */
    (void)Drive_Arm();

    g_stub.enc_status = cases[i].enc;
    g_stub.enc_raw    = cases[i].raw;
    g_stub.vbus_mv    = cases[i].vbus;
    g_stub.vbus_rc    = cases[i].rc;
    Drive_SelfTest();

    /* Two separate properties, and the second is the one with teeth.
     *
     * "The bridge ended up down" is also guaranteed by Drive_Enter, which
     * disables the outputs on the way into any non-RUN state - so on its own
     * it cannot tell whether the fault path did its job. Drive_Fault's
     * contract is stronger: freewheel FIRST, unconditionally, before any
     * bookkeeping, because the bookkeeping may not survive. That is
     * MotorPwm_EmergencyStop, and it either got called or it did not. */
    if (g_stub.outputs_live || g_stub.gate_live)
    {
      all_safe = 0;
      snprintf(d, sizeof(d), "%s left outputs_live=%d gate_live=%d",
               cases[i].name, g_stub.outputs_live, g_stub.gate_live);
    }
    else if (g_stub.estop == 0U)
    {
      all_safe = 0;
      snprintf(d, sizeof(d),
               "%s brought the bridge down without an unconditional "
               "EmergencyStop first", cases[i].name);
    }
  }
  check(all_safe, "every fault path leaves the bridge down", d);
}

/* ---- the state machine's own rules -------------------------------------- */

static void test_fault_latches(void)
{
  Stub_Reset();
  g_stub.enc_status = (int)ENC_ERR_SPI;
  boot();
  g_stub.enc_status = (int)ENC_OK;           /* the fault "goes away" */
  Drive_Step(HAL_GetTick());
  Host_AdvanceTick(10000U);
  Drive_Step(HAL_GetTick());
  expect(DRIVE_FAULT, DRIVE_FAULT_ENCODER,
         "FAULT latches even after the cause clears itself");
}

static void test_clear_returns_to_selftest_not_ready(void)
{
  /* The discriminating case is clearing while the cause is STILL TRUE. A
   * clear that re-runs the checks lands back in FAULT; one that goes straight
   * to READY - which is what this used to do a few seconds after an
   * overcurrent - reports a healthy drive with a dead encoder on it.
   *
   * Asserting only the healthy-clear case cannot tell those apart: both end
   * in READY. */
  Stub_Reset();
  g_stub.enc_status = (int)ENC_ERR_SPI;
  boot();
  Drive_ClearFault();                        /* encoder still silent */

  char d[192];
  snprintf(d, sizeof(d), "cleared onto a still-broken encoder and landed in %s",
           Drive_StateName(g_drive.state));
  int refused = (g_drive.state == (uint32_t)DRIVE_FAULT);

  /* And the converse: once the cause is genuinely gone, a clear does reach
   * READY. Without this the test would pass on a drive that never clears. */
  Stub_Reset();
  g_stub.enc_status = (int)ENC_ERR_SPI;
  boot();
  g_stub.enc_status = (int)ENC_OK;
  Drive_ClearFault();
  int recovers = (g_drive.state == (uint32_t)DRIVE_READY);

  check(refused && recovers,
        "clearing a fault re-runs the checks rather than arming", d);
}

static void test_arm_refused_while_faulted(void)
{
  Stub_Reset();
  g_stub.enc_status = (int)ENC_ERR_SPI;
  boot();
  unsigned gate_before = g_stub.gate_en;
  int32_t rc = Drive_Arm();

  char d[160];
  snprintf(d, sizeof(d), "Drive_Arm returned %ld, gate_en went %u -> %u",
           (long)rc, gate_before, g_stub.gate_en);
  check(rc != 0 && g_stub.gate_en == gate_before && !g_stub.gate_live,
        "Drive_Arm is refused while FAULT is latched", d);
}

static void test_first_fault_wins(void)
{
  Stub_Reset();
  g_stub.enc_status = (int)ENC_ERR_SPI;
  boot();                                    /* ENCODER latches */
  g_stub.enc_status = (int)ENC_OK;
  Drive_Fault(DRIVE_FAULT_OVERCURRENT);      /* a second, different trip */

  char d[160];
  snprintf(d, sizeof(d), "cause %s, fault_count %u",
           cause_name(g_drive.fault), (unsigned)g_drive.fault_count);
  /* One value, not a bitmask: what tripped first is what is recoverable. */
  check(g_drive.fault == (uint32_t)DRIVE_FAULT_ENCODER &&
        g_drive.fault_count >= 2U,
        "a later fault is counted, not merged over the first", d);
}

static void test_gate_never_precedes_outputs(void)
{
  Stub_Reset();
  boot();
  (void)Drive_Arm();
  char d[160];
  snprintf(d, sizeof(d), "%u orderings had the gate up first",
           g_stub.gate_before_outputs);
  check(g_stub.gate_before_outputs == 0U,
        "arming never raises the gate drivers before the outputs", d);
}

int main(void)
{
  printf("\ndrive.c fault injection\n-----------------------\n");
  test_healthy_reaches_ready();
  test_encoder_silent();
  test_encoder_dead_miso();
  test_current_sense_railed();
  test_current_sense_just_inside_tolerance();
  test_vbus_collapse_waits_rather_than_latching();
  test_vbus_recovers_on_its_own();
  test_vbus_overvoltage_latches();
  test_vbus_read_failure();
  test_every_fault_safes_the_bridge();
  test_fault_latches();
  test_clear_returns_to_selftest_not_ready();
  test_arm_refused_while_faulted();
  test_first_fault_wins();
  test_gate_never_precedes_outputs();
  printf("-----------------------\n%d passed, %d failed\n\n", g_pass, g_fail);
  return g_fail == 0 ? 0 : 1;
}
