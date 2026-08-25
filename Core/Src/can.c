/**
  ******************************************************************************
  * @file    can.c
  * @brief   FDCAN1 transport for the control protocol in can_proto.h.
  ******************************************************************************
  */

#include "can.h"
#include "fdcan.h"
#include "main.h"
#include "position.h"
#include "foc.h"
#include "csense.h"
#include "motor_pwm.h"
#include <string.h>

/* Owned by main.c. */
extern volatile PosState_t   g_pos;
extern volatile FocState_t   g_foc;
extern volatile CSenseTelem_t g_cs;
extern volatile uint32_t     g_faulted;

/* Set by main.c so CAN and the SWD bench block cannot disagree about whether
 * the bridge is up; see Can_ApplyEnable below. */
extern volatile uint32_t g_can_wants_bridge;

static CanTelem_t s_t;
static uint32_t   s_telem_tick;
static uint8_t    s_timed_out;

/* ---- little-endian payload helpers -------------------------------------- *
 *
 * Written out byte by byte rather than casting the payload to an int32_t.
 * The RX buffer has no alignment guarantee, and an unaligned 32-bit load is
 * a fault on some cores and silently slow on others; this also pins the byte
 * order to the spec instead of to whatever the compiler happens to do. */
static int32_t rd_i32(const uint8_t *p)
{
  return (int32_t)((uint32_t)p[0] | ((uint32_t)p[1] << 8) |
                   ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24));
}

static void wr_i32(uint8_t *p, int32_t v)
{
  uint32_t u = (uint32_t)v;
  p[0] = (uint8_t)u; p[1] = (uint8_t)(u >> 8);
  p[2] = (uint8_t)(u >> 16); p[3] = (uint8_t)(u >> 24);
}

static void wr_i16(uint8_t *p, int32_t v)
{
  /* Saturate rather than wrap: a velocity that overflowed would otherwise be
   * reported with the opposite sign, which is worse than being clipped. */
  if (v >  32767) { v =  32767; }
  if (v < -32768) { v = -32768; }
  uint16_t u = (uint16_t)(int16_t)v;
  p[0] = (uint8_t)u; p[1] = (uint8_t)(u >> 8);
}

static void wr_u16(uint8_t *p, uint32_t v)
{
  if (v > 65535U) { v = 65535U; }
  p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8);
}

int32_t Can_Init(uint8_t loopback)
{
  memset(&s_t, 0, sizeof(s_t));

  hfdcan1.Instance                  = FDCAN1;
  hfdcan1.Init.ClockDivider         = FDCAN_CLOCK_DIV1;
  hfdcan1.Init.FrameFormat          = FDCAN_FRAME_CLASSIC;
  hfdcan1.Init.Mode                 = loopback ? FDCAN_MODE_INTERNAL_LOOPBACK
                                               : FDCAN_MODE_NORMAL;
  /* Retransmit on arbitration loss or error. A dropped setpoint is not
   * catastrophic on its own, but a dropped ESTOP is, and the watchdog cannot
   * distinguish "nobody is talking" from "the one frame that mattered was
   * lost". Let the peripheral do what CAN is designed to do. */
  hfdcan1.Init.AutoRetransmission   = ENABLE;
  hfdcan1.Init.TransmitPause        = DISABLE;
  hfdcan1.Init.ProtocolException    = DISABLE;

  /* 1 Mbit/s from a 128 MHz kernel clock (PCLK1, see HAL_FDCAN_MspInit):
   *   128 MHz / 8 = 16 MHz time-quantum clock
   *   1 sync + 12 + 3 = 16 tq per bit -> 1 Mbit/s
   * Sample point at (1+12)/16 = 81.25%, which is what CiA recommends and
   * what every other node on a bus will have been set up for. */
  hfdcan1.Init.NominalPrescaler     = 8;
  hfdcan1.Init.NominalSyncJumpWidth = 3;
  hfdcan1.Init.NominalTimeSeg1      = 12;
  hfdcan1.Init.NominalTimeSeg2      = 3;

  /* Unused in classic mode, but HAL validates them. */
  hfdcan1.Init.DataPrescaler        = 1;
  hfdcan1.Init.DataSyncJumpWidth    = 1;
  hfdcan1.Init.DataTimeSeg1         = 1;
  hfdcan1.Init.DataTimeSeg2         = 1;

  hfdcan1.Init.StdFiltersNbr        = 2;
  hfdcan1.Init.ExtFiltersNbr        = 0;
  hfdcan1.Init.TxFifoQueueMode      = FDCAN_TX_FIFO_OPERATION;

  if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK) { s_t.init_rc = -1; return -1; }

  /* Two acceptance filters, both into FIFO0: one for this node's own command
   * range and one for the broadcast range. Filtering in hardware means a busy
   * bus carrying other drives' traffic costs this one nothing.
   *
   * The range stops at CAN_CMD_LAST rather than spanning the whole 5-bit
   * field, so a node never accepts its own telemetry identifiers. */
  FDCAN_FilterTypeDef f;
  f.IdType       = FDCAN_STANDARD_ID;
  f.FilterIndex  = 0;
  f.FilterType   = FDCAN_FILTER_RANGE;
  f.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  f.FilterID1    = CAN_ID(CAN_NODE_ID, CAN_CMD_FIRST);
  f.FilterID2    = CAN_ID(CAN_NODE_ID, CAN_CMD_LAST);
  if (HAL_FDCAN_ConfigFilter(&hfdcan1, &f) != HAL_OK) { s_t.init_rc = -2; return -2; }

  f.FilterIndex  = 1;
  f.FilterID1    = CAN_ID(CAN_NODE_BROADCAST, CAN_CMD_FIRST);
  f.FilterID2    = CAN_ID(CAN_NODE_BROADCAST, CAN_CMD_LAST);
  if (HAL_FDCAN_ConfigFilter(&hfdcan1, &f) != HAL_OK) { s_t.init_rc = -3; return -3; }

  /* Anything that got past neither filter is not for us. */
  if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_REJECT, FDCAN_REJECT,
                                   FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE) != HAL_OK)
  {
    s_t.init_rc = -4; return -4;
  }

  if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK) { s_t.init_rc = -5; return -5; }

  s_t.init_rc = 0;
  return 0;
}

int32_t Can_Send(uint16_t id, const uint8_t *data, uint8_t len)
{
  static const uint32_t dlc[9] = {
    FDCAN_DLC_BYTES_0, FDCAN_DLC_BYTES_1, FDCAN_DLC_BYTES_2, FDCAN_DLC_BYTES_3,
    FDCAN_DLC_BYTES_4, FDCAN_DLC_BYTES_5, FDCAN_DLC_BYTES_6, FDCAN_DLC_BYTES_7,
    FDCAN_DLC_BYTES_8
  };
  if (len > 8U) { return -1; }

  FDCAN_TxHeaderTypeDef h;
  h.Identifier          = id;
  h.IdType              = FDCAN_STANDARD_ID;
  h.TxFrameType         = FDCAN_DATA_FRAME;
  h.DataLength          = dlc[len];
  h.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  h.BitRateSwitch       = FDCAN_BRS_OFF;
  h.FDFormat            = FDCAN_CLASSIC_CAN;
  h.TxEventFifoControl  = FDCAN_NO_TX_EVENTS;
  h.MessageMarker       = 0;

  if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &h, (uint8_t *)data) != HAL_OK)
  {
    s_t.tx_errors++;
    return -2;
  }
  s_t.tx_frames++;
  return 0;
}

/* Bringing the bridge up from a CAN command has to go through the same
 * ordering main.c enforces - duty, then outputs, then gates - so this asks
 * main.c to do it rather than poking the peripheral from here. */
static void Can_ApplyEnable(uint8_t on)
{
  g_can_wants_bridge = on ? 1U : 0U;
  g_pos.enabled      = on ? 1U : 0U;
  if (!on)
  {
    g_pos.mode   = MOTION_MODE_IDLE;
    g_foc.iq_ref = 0.0f;
    g_foc.id_ref = 0.0f;
  }
}

static void Can_Estop(void)
{
  g_pos.enabled = 0U;
  g_pos.mode    = MOTION_MODE_IDLE;
  g_foc.iq_ref  = 0.0f;
  g_foc.id_ref  = 0.0f;
  g_foc.enabled = 0U;
  g_can_wants_bridge = 0U;
  MotorPwm_EmergencyStop();
}

static void Can_Handle(uint8_t cmd, const uint8_t *d, uint8_t len)
{
  switch (cmd)
  {
    case CAN_CMD_ESTOP:
      Can_Estop();
      break;

    case CAN_CMD_SET_MODE:
      if (len < 1U) { s_t.rx_bad_len++; return; }
      if (d[0] <= MOTION_MODE_POSITION) { g_pos.mode = d[0]; }
      else { s_t.rx_bad_len++; }
      break;

    case CAN_CMD_SET_TORQUE:
      if (len < 4U) { s_t.rx_bad_len++; return; }
      g_pos.torque_cmd_ma = rd_i32(d);
      break;

    case CAN_CMD_SET_VELOCITY:
      if (len < 4U) { s_t.rx_bad_len++; return; }
      g_pos.vel_cmd_dps = rd_i32(d);
      break;

    case CAN_CMD_SET_POSITION:
      if (len < 4U) { s_t.rx_bad_len++; return; }
      g_pos.cmd_deg_x10 = rd_i32(d);
      break;

    case CAN_CMD_SET_LIMITS:
      if (len < 8U) { s_t.rx_bad_len++; return; }
      g_pos.iq_max_ma  = rd_i32(d);
      g_pos.vel_max_dps = rd_i32(d + 4);
      break;

    case CAN_CMD_SET_PROFILE:
      if (len < 8U) { s_t.rx_bad_len++; return; }
      g_pos.accel_max_dps2 = rd_i32(d);
      g_pos.jerk_max_dps3  = rd_i32(d + 4);
      break;

    case CAN_CMD_ZERO_HERE:
      g_pos.zero_here = 1U;
      break;

    case CAN_CMD_CLEAR_FAULT:
      g_faulted = 0U;
      break;

    case CAN_CMD_SET_ENABLE:
      if (len < 1U) { s_t.rx_bad_len++; return; }
      Can_ApplyEnable(d[0]);
      break;

    default:
      s_t.rx_bad_cmd++;
      break;
  }
}

void Can_Poll(void)
{
  if (s_t.init_rc != 0) { return; }

  FDCAN_RxHeaderTypeDef h;
  uint8_t d[8];

  while (HAL_FDCAN_GetRxFifoFillLevel(&hfdcan1, FDCAN_RX_FIFO0) > 0U)
  {
    if (HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &h, d) != HAL_OK) { break; }

    uint8_t node = CAN_ID_NODE(h.Identifier);
    uint8_t cmd  = CAN_ID_CMD(h.Identifier);

    if ((node != CAN_NODE_ID) && (node != CAN_NODE_BROADCAST))
    {
      s_t.rx_ignored++;
      continue;
    }

    /* DataLength is the DLC code; for classic frames up to 8 bytes it maps to
     * the byte count directly, and every command here is <= 8. */
    uint8_t len = (uint8_t)(h.DataLength & 0x0FU);
    if (len > 8U) { len = 8U; }

    s_t.rx_frames++;
    s_t.last_rx_id   = h.Identifier;
    s_t.last_rx_tick = HAL_GetTick();
    s_t.armed        = 1U;
    s_timed_out      = 0U;

    Can_Handle(cmd, d, len);
  }
}

void Can_CheckTimeout(void)
{
  if ((s_t.init_rc != 0) || (s_t.armed == 0U) || (s_timed_out != 0U)) { return; }

  if ((HAL_GetTick() - s_t.last_rx_tick) >= CAN_CMD_TIMEOUT_MS)
  {
    /* Silence is not consent to keep driving. Drop to idle but leave the
     * bridge up, so a host that comes back can resume without re-arming - the
     * motor is making no torque in the meantime. */
    s_timed_out = 1U;
    s_t.timeouts++;
    g_pos.mode   = MOTION_MODE_IDLE;
    g_foc.iq_ref = 0.0f;
  }
}

void Can_PublishTelem(void)
{
  if (s_t.init_rc != 0) { return; }

  uint32_t now = HAL_GetTick();
  if ((now - s_telem_tick) < (1000U / CAN_TELEM_HZ)) { return; }
  s_telem_tick = now;

  uint8_t d[8];

  wr_i32(d,     g_pos.pos_deg_x10);
  wr_i16(d + 4, g_pos.vel_dps);
  wr_i16(d + 6, g_pos.iq_out_ma);
  (void)Can_Send(CAN_ID(CAN_NODE_ID, CAN_MSG_TELEM_MOTION), d, 8);

  uint8_t flags = 0U;
  if (g_pos.enabled)  { flags |= CAN_FLAG_ENABLED; }
  if (g_foc.enabled)  { flags |= CAN_FLAG_FOC_ON; }
  if (g_faulted)      { flags |= CAN_FLAG_FAULTED; }
  if (s_timed_out)    { flags |= CAN_FLAG_CMD_TIMEOUT; }

  d[0] = (uint8_t)g_pos.mode;
  d[1] = flags;
  wr_u16(d + 2, g_cs.vbus_mv);
  wr_i16(d + 4, g_pos.err_deg_x10);
  wr_u16(d + 6, (uint32_t)g_pos.iq_max_ma);
  (void)Can_Send(CAN_ID(CAN_NODE_ID, CAN_MSG_TELEM_STATE), d, 8);
}

void Can_GetTelem(CanTelem_t *t)
{
  *t = s_t;
}
