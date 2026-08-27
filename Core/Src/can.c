/**
  ******************************************************************************
  * @file    can.c
  * @brief   FDCAN2 transport for the control protocol in can_proto.h.
  ******************************************************************************
  */

#include "can.h"
#include "fdcan.h"
#include "main.h"
#include "position.h"
#include "foc.h"
#include "csense.h"
#include "motor_pwm.h"
#include "limits.h"
#include <string.h>

/* Owned by main.c. */
extern volatile PosState_t   g_pos;
extern volatile FocState_t   g_foc;
extern volatile CSenseTelem_t g_cs;
extern volatile uint32_t     g_faulted;
#include "drive.h"

/* Set by main.c so CAN and the SWD bench block cannot disagree about whether
 * the bridge is up; see Can_ApplyEnable below. */
extern volatile uint32_t g_can_wants_bridge;

static CanTelem_t s_t;
static uint32_t   s_telem_tick;
static uint32_t   s_norm_tick;      /* normal-rate slot, for suppression count */
static uint8_t    s_timed_out;
static uint8_t    s_bus_off_seen;   /* sticky; cleared only by Can_Init */
static uint32_t   s_recover_tick;

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
  s_bus_off_seen = 0U;
  s_recover_tick = 0U;
  s_timed_out    = 0U;

  hfdcan2.Instance                  = FDCAN2;
  hfdcan2.Init.ClockDivider         = FDCAN_CLOCK_DIV1;
  hfdcan2.Init.FrameFormat          = FDCAN_FRAME_CLASSIC;
  hfdcan2.Init.Mode                 = loopback ? FDCAN_MODE_INTERNAL_LOOPBACK
                                               : FDCAN_MODE_NORMAL;
  /* SINGLE-SHOT transmission. This was ENABLE, on the reasoning that a
   * dropped ESTOP is catastrophic - but this node never TRANSMITS an ESTOP.
   * ESTOP is host -> drive; everything this node puts on the wire is periodic
   * telemetry, which is replaced 20 ms later by a fresher copy of itself.
   * Retransmission was therefore protecting nothing and costing a great deal:
   * an unacknowledged frame is retried as fast as the bus allows, each attempt
   * adding 8 to the transmit error counter, so a bus with nobody listening
   * drives this node to BUS-OFF in milliseconds - and a bus-off node stops
   * receiving too, ESTOP included.
   *
   * One attempt per frame turns that into 8 counts per published frame, which
   * the backoff in Can_PublishTelem can then actually outrun. The cost is that
   * a frame losing arbitration is dropped rather than retried; at 1.3% bus
   * load per drive that is rare, and the next telemetry frame is 20 ms away. */
  hfdcan2.Init.AutoRetransmission   = DISABLE;
  hfdcan2.Init.TransmitPause        = DISABLE;
  hfdcan2.Init.ProtocolException    = DISABLE;

  /* 1 Mbit/s from a 160 MHz kernel clock (PCLK1, see HAL_FDCAN_MspInit):
   *   160 MHz / 10 = 16 MHz time-quantum clock
   *   1 sync + 12 + 3 = 16 tq per bit -> 1 Mbit/s
   * Sample point at (1+12)/16 = 81.25%, which is what CiA recommends and
   * what every other node on a bus will have been set up for.
   *
   * ONLY the prescaler moved, 8 -> 10, because Mako Longfin's kernel clock
   * was 128 MHz and this board's is 160. The segment split is untouched, so
   * the sample point is bit-for-bit what was validated on the old board.
   *
   * Leaving the prescaler at 8 would have produced 1.25 Mbit/s. That does not
   * fail as a wrong number somewhere - the node simply never acknowledges a
   * frame and every other node on the bus goes error-passive trying to talk
   * to it, which reads as dead hardware. */
  hfdcan2.Init.NominalPrescaler     = 10;
  hfdcan2.Init.NominalSyncJumpWidth = 3;
  hfdcan2.Init.NominalTimeSeg1      = 12;
  hfdcan2.Init.NominalTimeSeg2      = 3;

  /* Unused in classic mode, but HAL validates them. */
  hfdcan2.Init.DataPrescaler        = 1;
  hfdcan2.Init.DataSyncJumpWidth    = 1;
  hfdcan2.Init.DataTimeSeg1         = 1;
  hfdcan2.Init.DataTimeSeg2         = 1;

  hfdcan2.Init.StdFiltersNbr        = 2;
  hfdcan2.Init.ExtFiltersNbr        = 0;
  hfdcan2.Init.TxFifoQueueMode      = FDCAN_TX_FIFO_OPERATION;

  if (HAL_FDCAN_Init(&hfdcan2) != HAL_OK) { s_t.init_rc = -1; return -1; }

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
  if (HAL_FDCAN_ConfigFilter(&hfdcan2, &f) != HAL_OK) { s_t.init_rc = -2; return -2; }

  f.FilterIndex  = 1;
  f.FilterID1    = CAN_ID(CAN_NODE_BROADCAST, CAN_CMD_FIRST);
  f.FilterID2    = CAN_ID(CAN_NODE_BROADCAST, CAN_CMD_LAST);
  if (HAL_FDCAN_ConfigFilter(&hfdcan2, &f) != HAL_OK) { s_t.init_rc = -3; return -3; }

  /* Anything that got past neither filter is not for us. */
  if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan2, FDCAN_REJECT, FDCAN_REJECT,
                                   FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE) != HAL_OK)
  {
    s_t.init_rc = -4; return -4;
  }

  if (HAL_FDCAN_Start(&hfdcan2) != HAL_OK) { s_t.init_rc = -5; return -5; }

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

  if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &h, (uint8_t *)data) != HAL_OK)
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

/* Note a request that lies outside what the hardware can do.
 *
 * The motion loop saturates every command regardless, so this changes no
 * behaviour - it exists so the condition is VISIBLE. A host quietly asking
 * for 40 A and quietly getting 12 looks identical to a host asking for 12,
 * and the difference matters when something does not move as expected. */
static void Can_RangeCheck(int32_t v, int32_t lo, int32_t hi)
{
  if ((v < lo) || (v > hi)) { s_t.rx_out_of_range++; }
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
      Can_RangeCheck(rd_i32(d), -LIM_IQ_MAX_MA, LIM_IQ_MAX_MA);
      g_pos.torque_cmd_ma = rd_i32(d);
      break;

    case CAN_CMD_SET_TORQUE_MNM:
    {
      if (len < 4U) { s_t.rx_bad_len++; return; }

      /* Convert here, at the edge, so everything downstream keeps working in
       * the milliamps it already speaks. The motion loop, the limits and the
       * telemetry do not need to learn a second unit for the same quantity. */
      int32_t mnm = rd_i32(d);
      int32_t ma  = (int32_t)(FOC_TorqueToIq((float)mnm * 0.001f) * 1000.0f);

      /* Range-checked on the CONVERTED value: the bound that exists is a
       * current bound - what the sensors, the FETs and the thermal budget
       * allow - and checking the torque figure against a torque bound derived
       * from the same kt would be the same arithmetic twice. */
      Can_RangeCheck(ma, -LIM_IQ_MAX_MA, LIM_IQ_MAX_MA);
      g_pos.torque_cmd_ma = ma;
      break;
    }

    case CAN_CMD_SET_VELOCITY:
      if (len < 4U) { s_t.rx_bad_len++; return; }
      Can_RangeCheck(rd_i32(d), -LIM_VEL_MAX_DPS, LIM_VEL_MAX_DPS);
      g_pos.vel_cmd_dps = rd_i32(d);
      break;

    case CAN_CMD_SET_POSITION:
      if (len < 4U) { s_t.rx_bad_len++; return; }
      Can_RangeCheck(rd_i32(d), -LIM_POS_MAX_DEG_X10, LIM_POS_MAX_DEG_X10);
      g_pos.cmd_deg_x10 = rd_i32(d);
      break;

    case CAN_CMD_SET_LIMITS:
      if (len < 8U) { s_t.rx_bad_len++; return; }
      Can_RangeCheck(rd_i32(d),     0, LIM_IQ_MAX_MA);
      Can_RangeCheck(rd_i32(d + 4), 1, LIM_VEL_MAX_DPS);
      g_pos.iq_max_ma  = rd_i32(d);
      g_pos.vel_max_dps = rd_i32(d + 4);
      break;

    case CAN_CMD_SET_PROFILE:
      if (len < 8U) { s_t.rx_bad_len++; return; }
      Can_RangeCheck(rd_i32(d),     LIM_ACCEL_MIN_DPS2, LIM_ACCEL_MAX_DPS2);
      Can_RangeCheck(rd_i32(d + 4), 0, LIM_JERK_MAX_DPS3);
      g_pos.accel_max_dps2 = rd_i32(d);
      g_pos.jerk_max_dps3  = rd_i32(d + 4);
      break;

    case CAN_CMD_ZERO_HERE:
      g_pos.zero_here = 1U;
      break;

    case CAN_CMD_CLEAR_FAULT:
      /* Goes through the state machine, which re-runs the self-test rather
       * than simply dropping the latch - see Drive_ClearFault. */
      Drive_ClearFault();
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

  while (HAL_FDCAN_GetRxFifoFillLevel(&hfdcan2, FDCAN_RX_FIFO0) > 0U)
  {
    if (HAL_FDCAN_GetRxMessage(&hfdcan2, FDCAN_RX_FIFO0, &h, d) != HAL_OK) { break; }

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

/* Stand the motion down without dropping the bridge.
 *
 * Silence is not consent to keep driving, but it is not a reason to disarm
 * either: leaving the stage up means a host that comes back resumes without
 * re-arming, and the motor makes no torque in the meantime. Shared by the
 * command watchdog and by bus-off, which are the same event - the control
 * link is gone - discovered two different ways. */
/* Wire-unit conversions. Rounding is symmetric about zero so a signed
 * quantity is not biased in either direction, and the divisors are the ones
 * documented in can_proto.h - 10 mA per count, 10 mV per count. */
static int32_t can_round_div(int32_t v, int32_t d)
{
  return (v >= 0) ? ((v + d / 2) / d) : -(((-v) + d / 2) / d);
}

static int32_t can_ma_to_ca(int32_t ma)   { return can_round_div(ma, 10); }
static int32_t can_mv_to_cv(int32_t mv)   { return can_round_div(mv, 10); }

/* Degrees per second -> rpm. 360 dps is 60 rpm, so the factor is 6. */
static int32_t can_dps_to_rpm(int32_t dps) { return can_round_div(dps, 6); }

static void Can_StandDown(void)
{
  s_timed_out  = 1U;
  g_pos.mode   = MOTION_MODE_IDLE;
  g_foc.iq_ref = 0.0f;
}

void Can_CheckTimeout(void)
{
  if ((s_t.init_rc != 0) || (s_t.armed == 0U) || (s_timed_out != 0U)) { return; }

  if ((HAL_GetTick() - s_t.last_rx_tick) >= CAN_CMD_TIMEOUT_MS)
  {
    s_t.timeouts++;
    Can_StandDown();
  }
}

void Can_CheckBus(void)
{
  if (s_t.init_rc != 0) { return; }

  FDCAN_ProtocolStatusTypeDef ps;
  FDCAN_ErrorCountersTypeDef  ec;

  (void)HAL_FDCAN_GetProtocolStatus(&hfdcan2, &ps);
  (void)HAL_FDCAN_GetErrorCounters(&hfdcan2, &ec);

  s_t.tec         = ec.TxErrorCnt;
  s_t.rec         = ec.RxErrorCnt;
  s_t.bus_warn    = ps.Warning;
  s_t.bus_passive = ps.ErrorPassive;

  if (ps.BusOff == 0U)
  {
    s_t.bus_off = 0U;
    return;
  }

  /* Entering bus-off. Count it once per episode, not once per poll. */
  if (s_t.bus_off == 0U)
  {
    s_t.bus_off = 1U;
    s_t.bus_off_events++;
    s_bus_off_seen = 1U;

    /* The control link is provably gone - a bus-off node receives nothing.
     * Arm-gated exactly like the command watchdog, so a bench drive being
     * driven over SWD with a transceiver attached and no host is not stopped
     * by a bus it was never using. */
    if ((s_t.armed != 0U) && (s_timed_out == 0U))
    {
      Can_StandDown();
    }

    /* Recover immediately the first time; the cooldown only paces retries. */
    s_recover_tick = HAL_GetTick() - CAN_BUSOFF_RETRY_MS;
  }

  if ((HAL_GetTick() - s_recover_tick) < CAN_BUSOFF_RETRY_MS) { return; }
  s_recover_tick = HAL_GetTick();

  /* Bus-off sets CCCR.INIT in hardware; clearing it is what starts the
   * recovery sequence (129 x 11 recessive bits), after which the error
   * counters reset and the node is error-active again.
   *
   * Done by hand rather than through HAL_FDCAN_Start, which refuses unless
   * the handle is in state READY - and after a bus-off the handle is still
   * BUSY, because nothing told the HAL that the hardware stopped. This is the
   * one line of that function that matters here. */
  CLEAR_BIT(hfdcan2.Instance->CCCR, FDCAN_CCCR_INIT);
  s_t.bus_recoveries++;
}

void Can_PublishTelem(void)
{
  if (s_t.init_rc != 0) { return; }

  /* Back off once the error counters say nobody is acknowledging, and stop
   * entirely while bus-off: nothing leaves a bus-off node, and queueing into
   * one only fills the TX FIFO with frames that are stale by the time it
   * recovers.
   *
   * At 50 Hz, single-shot, an unlistened bus costs 8 error counts per frame
   * and 800 per second - bus-off in about a third of a second. At
   * CAN_TELEM_PROBE_HZ that is 16 per second, which turns the same margin into
   * roughly ten, and every successful transmission takes one back off again.
   * The node keeps announcing itself either way. */
  uint32_t now       = HAL_GetTick();
  uint32_t normal_ms = 1000U / CAN_TELEM_HZ;
  uint32_t period_ms = normal_ms;

  if (s_t.bus_off != 0U)                                     { period_ms = 0U; }
  else if ((s_t.bus_warn != 0U) || (s_t.bus_passive != 0U))  { period_ms = 1000U / CAN_TELEM_PROBE_HZ; }

  /* What the backoff costs, counted in FRAMES. This function is called every
   * pass of the main loop - tens of thousands per second - so counting here
   * on entry would count polls, not messages. The normal-rate slot gets its
   * own tick for exactly that reason. */
  if ((now - s_norm_tick) >= normal_ms)
  {
    s_norm_tick = now;
    if (period_ms != normal_ms) { s_t.tx_suppressed += 2U; }
  }

  if (period_ms == 0U) { return; }
  if ((now - s_telem_tick) < period_ms) { return; }
  s_telem_tick = now;

  uint8_t d[8];

  /* Units are NOT the internal ones - see the note in can_proto.h. Internally
   * everything stays in mA and dps; the conversion happens here, at the wire,
   * so nothing upstream has to know the protocol exists.
   *
   * Rounded rather than truncated. Truncation biases every reading toward
   * zero, which on a current telemetry is a systematic under-report. */
  wr_i32(d,     g_pos.pos_deg_x10);
  wr_i16(d + 4, (int32_t)can_dps_to_rpm(g_pos.vel_dps));
  wr_i16(d + 6, (int32_t)can_ma_to_ca(g_pos.iq_out_ma));
  (void)Can_Send(CAN_ID(CAN_NODE_ID, CAN_MSG_TELEM_MOTION), d, 8);

  uint8_t flags = 0U;
  if (g_pos.enabled)     { flags |= CAN_FLAG_ENABLED; }
  if (g_foc.enabled)     { flags |= CAN_FLAG_FOC_ON; }
  if (g_faulted)         { flags |= CAN_FLAG_FAULTED; }
  if (s_timed_out)       { flags |= CAN_FLAG_CMD_TIMEOUT; }
  if (s_t.bus_warn)      { flags |= CAN_FLAG_BUS_WARN; }
  if (s_t.bus_passive)   { flags |= CAN_FLAG_BUS_PASSIVE; }
  if (s_bus_off_seen)    { flags |= CAN_FLAG_BUS_OFF_SEEN; }

  d[0] = (uint8_t)g_pos.mode;
  d[1] = flags;
  wr_u16(d + 2, (uint32_t)can_mv_to_cv(g_cs.vbus_mv));
  wr_i16(d + 4, g_pos.err_deg_x10);
  wr_u16(d + 6, (uint32_t)can_ma_to_ca(g_pos.iq_max_ma));
  (void)Can_Send(CAN_ID(CAN_NODE_ID, CAN_MSG_TELEM_STATE), d, 8);

  /* Drive state and the latched cause. See CAN_MSG_TELEM_DRIVE. */
  d[0] = (uint8_t)g_drive.state;
  d[1] = (uint8_t)g_drive.fault;
  d[2] = (uint8_t)g_drive.selftest_fail;
  d[3] = 0U;
  wr_u16(d + 4, g_drive.fault_count);
  wr_u16(d + 6, (g_drive.ms_in_state > 65535U) ? 65535U : g_drive.ms_in_state);
  (void)Can_Send(CAN_ID(CAN_NODE_ID, CAN_MSG_TELEM_DRIVE), d, 8);
}

void Can_GetTelem(CanTelem_t *t)
{
  *t = s_t;
}
