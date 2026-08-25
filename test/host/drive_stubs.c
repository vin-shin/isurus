/**
  ******************************************************************************
  * @file    test/host/drive_stubs.c
  * @brief   Everything drive.c reaches for that is not drive.c.
  *
  *          The same arrangement foc.c's CORDIC has: the file under test is
  *          compiled byte-for-byte as it ships, and the things it calls into
  *          are supplied here rather than conditionally compiled out of it. A
  *          harness that needs the file under test edited is testing a
  *          different program.
  *
  *          The gate-driver calls are RECORDED, not emulated. What these tests
  *          assert is the state machine - which cause latches, whether the
  *          bridge was actually safed, what a clear does - and "was the bridge
  *          taken down, in the right order" is answerable by counting calls.
  *          Whether HRTIM produces the right edges is not a question a host
  *          can answer and this does not pretend to.
  *
  *          The sensors are INPUTS the test sets. That is the fault injection:
  *          a silent encoder is `enc_status = ENC_ERR_SPI`, a collapsed bus is
  *          a low `vbus_mv`, a railed sensor is a `cs_*_zero` far from
  *          mid-scale. pmsm.h carries flags of the same shape for the closed
  *          loop; these are the open-loop equivalents, for the one path that
  *          reads the sensors directly.
  ******************************************************************************
  */

#include "drive_stubs.h"

#include "main.h"
#include "foc.h"
#include "position.h"
#include "csense.h"
#include "encoder.h"

#include <string.h>

DriveStub_t g_stub;

/* ---- peripherals, as plain memory --------------------------------------- */

/* g_rcc_host belongs to cordic_model.c, which the foc.c tests already link -
 * defining it here too would be a duplicate symbol. The other two are new. */
WwdgHostRegs_t   g_wwdg_host;
DbgmcuHostRegs_t g_dbgmcu_host;

/* ---- globals drive.c reaches across for --------------------------------- */

volatile CSenseTelem_t g_cs;
volatile PosState_t    g_pos;
volatile uint32_t      g_faulted;

/* On the target this lives in main.c, which is not host-compilable. drive.c
 * only reads and clears g_foc.enabled, but it is the real FocState_t so that
 * FOC_Reset - which IS linked, from foc.c - operates on the real thing. */
volatile FocState_t    g_foc;

/* ---- host clock ---------------------------------------------------------- */

static uint32_t s_tick_ms;

uint32_t HAL_GetTick(void)      { return s_tick_ms; }
void     Host_SetTick(uint32_t ms) { s_tick_ms = ms; }
void     Host_AdvanceTick(uint32_t ms) { s_tick_ms += ms; }

/* ---- the bridge, recorded ------------------------------------------------ */

void MotorPwm_EnableOutputs(void)
{
  g_stub.outputs_en++;
  g_stub.outputs_live = 1;
}

void MotorPwm_DisableOutputs(void)
{
  g_stub.outputs_dis++;
  g_stub.outputs_live = 0;
}

void MotorPwm_GateEnable(void)
{
  g_stub.gate_en++;
  g_stub.gate_live = 1;
  /* Records an ORDERING error rather than asserting here, so a test can name
   * the violation instead of the harness aborting inside a stub with no
   * context. Enabling the gate drivers while the outputs are still down is
   * not dangerous; the reverse is, and that is checked below. */
  if (!g_stub.outputs_live) { g_stub.gate_before_outputs++; }
}

void MotorPwm_GateDisable(void)
{
  g_stub.gate_dis++;
  g_stub.gate_live = 0;
}

void MotorPwm_EmergencyStop(void)
{
  g_stub.estop++;
  g_stub.outputs_live = 0;
  g_stub.gate_live    = 0;
}

void MotorPwm_SetDutyPermille(uint32_t u, uint32_t v, uint32_t w)
{
  g_stub.duty_calls++;
  g_stub.last_duty_u = u;
  g_stub.last_duty_v = v;
  g_stub.last_duty_w = w;
}

/* ---- sensors, as injectable inputs --------------------------------------- */

Encoder_Status_t Encoder_ReadAngle(uint16_t *raw_counts)
{
  g_stub.enc_reads++;
  if (g_stub.enc_status != (int)ENC_OK)
  {
    return (Encoder_Status_t)g_stub.enc_status;
  }
  if (raw_counts != NULL) { *raw_counts = g_stub.enc_raw; }
  return ENC_OK;
}

int CSense_ReadVbus(CSenseTelem_t *t)
{
  g_stub.vbus_reads++;
  if (g_stub.vbus_rc != 0) { return g_stub.vbus_rc; }
  if (t != NULL) { t->vbus_mv = g_stub.vbus_mv; }
  return 0;
}

/* ---- harness control ----------------------------------------------------- */

void Stub_Reset(void)
{
  memset(&g_stub, 0, sizeof(g_stub));

  /* A HEALTHY board by default, so each test names only the one thing it is
   * breaking. A test that had to spell out every good input in order to
   * inject one bad one would stop saying which input it was about. */
  g_stub.enc_status = (int)ENC_OK;
  g_stub.enc_raw    = 0x1234U;      /* any plausible angle */
  g_stub.vbus_rc    = 0;
  g_stub.vbus_mv    = 22000U;       /* inside the LIM_VBUS window */

  memset((void *)&g_cs, 0, sizeof(g_cs));
  g_cs.u_zero = 2040U;              /* HARDWARE_NOTES section 6 records 2037 */
  g_cs.w_zero = 2041U;              /* and 2039 measured on this board       */
  g_cs.vbus_mv = g_stub.vbus_mv;

  memset((void *)&g_pos, 0, sizeof(g_pos));
  g_faulted = 0U;

  g_rcc_host.CSR = 0U;
  Host_SetTick(0U);
}
