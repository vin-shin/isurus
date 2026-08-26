/**
  ******************************************************************************
  * @file    test/host/drive_stubs.h
  * @brief   The injection surface for the drive.c host tests.
  *
  *          Set the sensor fields to break something, run the state machine,
  *          then read the counters to find out what it did about it.
  ******************************************************************************
  */
#ifndef DRIVE_STUBS_H
#define DRIVE_STUBS_H

#include <stdint.h>

#include "csense.h"
#include "thermal.h"
#include "position.h"

/* Defined in drive_stubs.c. drive.c declares its own externs for these rather
 * than taking them from a header, so the tests need them from somewhere. */
extern volatile CSenseTelem_t g_cs;
extern volatile ThermalTelem_t g_therm;
extern volatile PosState_t    g_pos;
extern volatile uint32_t      g_faulted;

typedef struct {
  /* ---- inputs: what the sensors report ----------------------------------- */
  int      enc_status;    /* ENC_OK, or an Encoder_Status_t error to return   */
  uint16_t enc_raw;       /* angle handed back when enc_status is ENC_OK      */
  int      vbus_rc;       /* non-zero = CSense_ReadVbus itself fails          */
  uint32_t vbus_mv;       /* bus reported when the read succeeds              */
  int      therm_rc;      /* non-zero = Thermal_Read fails (sensor lost)      */
  int32_t  therm_c_x10;   /* winding temperature when the read succeeds      */

  /* ---- outputs: what the drive did about it ------------------------------ */
  unsigned outputs_en, outputs_dis;
  unsigned gate_en, gate_dis;
  unsigned estop;
  unsigned duty_calls;
  uint32_t last_duty_u, last_duty_v, last_duty_w;
  unsigned enc_reads, vbus_reads, therm_reads;

  /* Live state, so a test can ask "is the bridge down NOW" rather than
   * inferring it from call counts. */
  int      outputs_live;
  int      gate_live;

  /* Ordering violations, counted rather than asserted - see the note in
   * MotorPwm_GateEnable. */
  unsigned gate_before_outputs;
} DriveStub_t;

extern DriveStub_t g_stub;

void Stub_Reset(void);
void Host_SetTick(uint32_t ms);
void Host_AdvanceTick(uint32_t ms);

#endif /* DRIVE_STUBS_H */
