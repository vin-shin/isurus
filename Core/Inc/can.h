/**
  ******************************************************************************
  * @file    can.h
  * @brief   FDCAN1 transport for the control protocol in can_proto.h.
  *
  *          Polled, not interrupt-driven. The 30 kHz control ISR already owns
  *          the tightest deadline on this MCU at priority 0, and a CAN frame
  *          is not urgent by comparison: at 1 Mbit/s a frame takes ~130 us to
  *          arrive, and the main loop normally runs far faster than that.
  *          Keeping CAN out of interrupt context means it can never delay a
  *          control step.
  *
  *          The limit of that argument: RX FIFO 0 holds THREE elements on this
  *          part, so the FIFO only survives a main-loop stall shorter than
  *          ~390 us. Telem_Printf blocks until the last character leaves the
  *          UART - about 6 ms for a 70-character line at 115200, or ~45 frame
  *          times - and nothing counts what is lost, because RXF0S is never
  *          read. Do not treat "cannot overrun" as unconditional while the
  *          UART telemetry is compiled in. See docs/CAN_PROTOCOL.md.
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

  /* ---- bus health; see "Bus health" in can_proto.h -------------------- */
  uint32_t tec;            /* transmit error counter, 0..255        */
  uint32_t rec;            /* receive error counter, 0..127         */
  uint32_t bus_warn;       /* 1 = an error counter has reached 96   */
  uint32_t bus_passive;    /* 1 = error-passive                     */
  uint32_t bus_off;        /* 1 = bus-off RIGHT NOW. Not visible to
                            * a host: a bus-off node cannot transmit.
                            * CAN_FLAG_BUS_OFF_SEEN is the wire form. */
  uint32_t bus_off_events; /* times bus-off has been entered        */
  uint32_t bus_recoveries; /* recovery sequences started            */
  uint32_t tx_suppressed;  /* telemetry frames withheld because the
                            * bus was unhealthy - the count that says
                            * backoff is doing something            */
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

/* Publish telemetry, rate-limited internally to CAN_TELEM_HZ - or to
 * CAN_TELEM_PROBE_HZ while the bus is unhealthy, and not at all while it is
 * bus-off. Call from the main loop. */
void Can_PublishTelem(void);

/* Sample the protocol status and error counters, and recover the node if it
 * has counted itself into bus-off. Call from the main loop, BEFORE
 * Can_PublishTelem, so a bus-off is known before anything is queued into it.
 *
 * This is what stops a drive on a dead bus from becoming permanently deaf -
 * including to ESTOP. See "Bus health" in can_proto.h for why that happens at
 * all and what the three lines of defence are. */
void Can_CheckBus(void);

/* Enforce the command watchdog; call from the main loop. */
void Can_CheckTimeout(void);

void Can_GetTelem(CanTelem_t *t);

/* Queue one frame. Exposed for the loopback self-test. */
int32_t Can_Send(uint16_t id, const uint8_t *data, uint8_t len);

#ifdef __cplusplus
}
#endif

#endif /* CAN_H */
