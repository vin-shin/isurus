/**
  ******************************************************************************
  * @file    can.h
  * @brief   FDCAN1 transport for the control protocol in can_proto.h.
  *
  *          Polled, not interrupt-driven. The 20 kHz control ISR already owns
  *          the tightest deadline on this MCU at priority 0, and a CAN frame
  *          is not urgent by comparison: at 1 Mbit/s a frame takes ~130 us to
  *          arrive and the main loop runs far faster than that, so the RX FIFO
  *          cannot overrun between polls. Keeping CAN out of interrupt context
  *          means it can never delay a control step.
  ******************************************************************************
  */
#ifndef CAN_H
#define CAN_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "can_proto.h"

typedef struct {
  uint32_t rx_frames;      /* addressed frames accepted            */
  uint32_t rx_ignored;     /* frames for another node              */
  uint32_t rx_bad_len;     /* right command, wrong payload length  */
  uint32_t rx_bad_cmd;     /* unknown command id                   */
  uint32_t rx_out_of_range;/* accepted, but a field was out of range and
                            * got saturated - see limits.h. A host that
                            * increments this is asking for more than the
                            * machine can do and should be fixed.   */
  uint32_t tx_frames;      /* telemetry frames queued              */
  uint32_t tx_errors;      /* queue rejected the frame             */
  uint32_t timeouts;       /* times the command watchdog fired     */
  uint32_t last_rx_id;     /* identifier of the last accepted frame */
  uint32_t last_rx_tick;   /* HAL tick of the last accepted frame   */
  uint32_t armed;          /* 1 once a command has ever arrived     */
  int32_t  init_rc;        /* 0 = up                                */
} CanTelem_t;

/* Bring FDCAN1 up at CAN_BITRATE_BPS and start it.
 *
 * `loopback` selects the peripheral's internal loopback mode, where the node
 * receives its own transmissions and never drives the bus. That makes the
 * whole protocol path - filters, framing, decode - testable on a bench with
 * no second node and no transceiver traffic. Pass 0 for normal operation. */
int32_t Can_Init(uint8_t loopback);

/* Drain the RX FIFO and apply whatever arrived. Call from the main loop. */
void Can_Poll(void);

/* Publish telemetry, rate-limited internally to CAN_TELEM_HZ. */
void Can_PublishTelem(void);

/* Enforce the command watchdog; call from the main loop. */
void Can_CheckTimeout(void);

void Can_GetTelem(CanTelem_t *t);

/* Queue one frame. Exposed for the loopback self-test. */
int32_t Can_Send(uint16_t id, const uint8_t *data, uint8_t len);

#ifdef __cplusplus
}
#endif

#endif /* CAN_H */
