/**
  ******************************************************************************
  * @file    can_proto.h
  * @brief   Wire protocol for controlling this drive over CAN.
  *
  *          This file is the specification. It is deliberately free of any
  *          STM32 or HAL dependency so a host - a Python script, another MCU,
  *          a test rig - can include or transcribe it and agree on the wire
  *          format by construction rather than by comment.
  *
  * ---------------------------------------------------------------------------
  * Physical layer
  * ---------------------------------------------------------------------------
  *   CAN 2.0A: classic CAN frames, 11-bit standard identifiers, 1 Mbit/s.
  *   Not CAN FD. The FDCAN peripheral is configured with FrameFormat set to
  *   FDCAN_FRAME_CLASSIC and IdType to FDCAN_STANDARD_ID, so nothing on the
  *   wire is an FD frame and any 2.0 node can read every frame here.
  *
  *   Classic rather than FD because every message below fits in 8 bytes, and
  *   2.0 talks to every analyser, transceiver and node in existence. Moving to
  *   FD later would change only the frame format, not the protocol.
  *
  *   FDCAN1 on PA12 (TX) and PB8 (RX). Note PB8 is also BOOT0 - see
  *   HARDWARE_NOTES.md section 1 before assuming a dead board is a firmware
  *   problem.
  *
  * ---------------------------------------------------------------------------
  * Addressing
  * ---------------------------------------------------------------------------
  *   The 11-bit identifier splits into a node address and a command:
  *
  *       ID[10:5]  node   1..63, or 0 for broadcast
  *       ID[4:0]   cmd    see CAN_CMD_* and CAN_MSG_* below
  *
  *   The 5-bit command field is split by direction:
  *
  *       0x00..0x0F   host -> drive   commands
  *       0x10..0x1F   drive -> host   telemetry
  *
  *   That split is what lets a drive accept only the command half in its
  *   receive filter. Without it a node accepts its own telemetry
  *   identifiers - harmless on a healthy bus, but it shows up immediately
  *   under loopback, and any echo or bridge on the bus would feed a drive
  *   its own state as though it were a command.
  *
  *   A node accepts frames addressed to its own node id AND to node 0, so a
  *   single frame can stop every drive on the bus.
  *
  *   CAN arbitration gives the numerically lowest identifier the bus, and the
  *   layout above is chosen so that falls out correctly: command 0 is ESTOP,
  *   so a broadcast e-stop is identifier 0x000 - the highest priority frame
  *   that can exist on the bus. Setpoint traffic can never delay it.
  *
  * ---------------------------------------------------------------------------
  * Byte order
  * ---------------------------------------------------------------------------
  *   Little-endian, matching the MCU, so payloads map straight onto the
  *   structs with no swapping on the target. Hosts should use explicit
  *   little-endian packing (Python: struct '<i', '<h', '<H').
  *
  * ---------------------------------------------------------------------------
  * Units
  * ---------------------------------------------------------------------------
  *   Every quantity on the wire is a scaled integer, never a float. Position
  *   is in tenths of a mechanical degree and is MULTI-TURN and signed, so
  *   3600 is one full turn and there is no wrap to reason about. Velocity is
  *   whole mechanical degrees per second. Current is milliamps of q-axis
  *   current, which for this motor is torque-producing current.
  *
  * ---------------------------------------------------------------------------
  * Limits
  * ---------------------------------------------------------------------------
  *   A host cannot command past what the machine can physically do. Every
  *   setpoint and limit is saturated into the bounds in limits.h before it is
  *   used - in the motion loop, not here, so the same bounds apply to the SWD
  *   tools and to anything a debugger writes by hand.
  *
  *   Saturation is silent by nature, so it is also counted. A frame carrying a
  *   value outside the bounds is still ACCEPTED and applied at the limit, and
  *   increments rx_out_of_range. A host that increments that counter is asking
  *   for something impossible and should be fixed; it is not an error the
  *   drive can resolve for you.
  *
  *   The headline bounds, all traceable to hardware:
  *
  *       current    12 A   below the 15 A bench overcurrent trip, itself well
  *                         below the motor's 22 A continuous and the sensors'
  *                         +/-40 A measuring range
  *       bus       50.4 V  a full 12S pack, the motor's ceiling. Exceeding it
  *                         latches the fault path and kills the stage.
  *       speed     3600 deg/s
  *       position  +/-100 turns
  *
  * ---------------------------------------------------------------------------
  * Safety
  * ---------------------------------------------------------------------------
  *   Once a drive has accepted its first command frame it starts requiring
  *   them: if CAN_CMD_TIMEOUT_MS passes with no addressed frame, the drive
  *   drops to idle on its own. A control link that has gone quiet is not the
  *   same as a control link asking for the last setpoint forever, and a
  *   position loop holding a stale setpoint against a jammed shaft is exactly
  *   the failure this exists to prevent.
  *
  *   The timeout only arms after the first frame, so a drive being driven
  *   over SWD with no CAN host present is unaffected.
  *
  * ---------------------------------------------------------------------------
  * Bus health
  * ---------------------------------------------------------------------------
  *   A CAN node that transmits into a bus nobody is acknowledging counts its
  *   way to BUS-OFF, and a bus-off FDCAN sets CCCR.INIT in hardware and stops
  *   transmitting AND RECEIVING. That last part is what makes this a safety
  *   matter rather than a diagnostics one: a drive that has counted itself off
  *   the bus can no longer be reached by an ESTOP.
  *
  *   Nothing about that is hypothetical here. This node publishes telemetry
  *   from boot whether or not a host exists, so an unplugged host, an
  *   unterminated bus or a single-node bench is enough to trigger it.
  *
  *   Three mechanisms keep it survivable, in order of when they act:
  *
  *     1. Transmission is SINGLE-SHOT. A frame gets one attempt; a failure
  *        costs 8 transmit-error counts and the frame is dropped rather than
  *        retried into the same silence.
  *     2. Telemetry BACKS OFF to CAN_TELEM_PROBE_HZ once the error counters
  *        pass the warning limit, which turns a third of a second of margin
  *        into about ten seconds and leaves the node announcing itself.
  *     3. If it reaches bus-off anyway, the drive RECOVERS ITSELF - clearing
  *        INIT starts the hardware recovery sequence - and retries no faster
  *        than every CAN_BUSOFF_RETRY_MS.
  *
  *   All of it is counted and published; see CAN_FLAG_BUS_* below.
  ******************************************************************************
  */
#ifndef CAN_PROTO_H
#define CAN_PROTO_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#define CAN_BITRATE_BPS         1000000U

/* This drive's address. 1..63; 0 is reserved for broadcast. */
#ifndef CAN_NODE_ID
#define CAN_NODE_ID             1U
#endif

#define CAN_NODE_BROADCAST      0U

#define CAN_ID(node, cmd)       ((uint16_t)((((node) & 0x3FU) << 5) | ((cmd) & 0x1FU)))
#define CAN_ID_NODE(id)         ((uint8_t)(((id) >> 5) & 0x3FU))
#define CAN_ID_CMD(id)          ((uint8_t)((id) & 0x1FU))

/* Drop to idle if this long passes with no addressed frame. Only armed once
 * the first command has been seen. */
#define CAN_CMD_TIMEOUT_MS      500U

/* ---- host -> drive ------------------------------------------------------ */
/*
 *  cmd   name           len  payload
 *  ----  -------------  ---  --------------------------------------------
 *  0x00  ESTOP            0  -
 *                           Servo off, FOC off, gate drivers opened. This is
 *                           the big hammer: recovering needs CLEAR_FAULT and
 *                           then ENABLE, which is deliberate.
 *  0x01  SET_MODE         1  u8  mode, MOTION_MODE_* (0 idle .. 3 position)
 *  0x02  SET_TORQUE       4  i32 milliamps of iq
 *  0x03  SET_VELOCITY     4  i32 mechanical degrees per second
 *  0x04  SET_POSITION     4  i32 tenths of a mechanical degree, multi-turn
 *  0x05  SET_LIMITS       8  i32 iq_max milliamps, i32 vel_max deg/s
 *  0x06  SET_PROFILE      8  i32 accel deg/s^2,   i32 jerk deg/s^3 (0 = trapezoid)
 *  0x07  ZERO_HERE        0  make the present rotor position the origin
 *  0x08  CLEAR_FAULT      0  clear a latched overcurrent trip
 *  0x09  SET_ENABLE       1  u8  0 = disengage, 1 = bring the bridge up and engage
 *
 *  A setpoint command does NOT change mode - send SET_MODE for that. The two
 *  are separate so a host can pre-load a setpoint and switch atomically,
 *  rather than the drive guessing which mode a bare number implies.
 */
#define CAN_CMD_ESTOP           0x00U
#define CAN_CMD_SET_MODE        0x01U
#define CAN_CMD_SET_TORQUE      0x02U
#define CAN_CMD_SET_VELOCITY    0x03U
#define CAN_CMD_SET_POSITION    0x04U
#define CAN_CMD_SET_LIMITS      0x05U
#define CAN_CMD_SET_PROFILE     0x06U
#define CAN_CMD_ZERO_HERE       0x07U
#define CAN_CMD_CLEAR_FAULT     0x08U
#define CAN_CMD_SET_ENABLE      0x09U

/* Inclusive bounds of the command half of the identifier space. The receive
 * filter is built from these, so telemetry never loops back in as command. */
#define CAN_CMD_FIRST           0x00U
#define CAN_CMD_LAST            0x0FU

/* ---- drive -> host ------------------------------------------------------ */
/*
 *  cmd   name             len  payload
 *  ----  ---------------  ---  ------------------------------------------
 *  0x10  TELEM_MOTION       8  i32 position, tenths of a degree, multi-turn
 *                              i16 velocity, degrees per second
 *                              i16 iq, milliamps
 *  0x11  TELEM_STATE        8  u8  mode
 *                              u8  flags, CAN_FLAG_*
 *                              u16 bus millivolts
 *                              i16 following error, tenths of a degree
 *                              u16 iq_max milliamps
 *  0x12  TELEM_DRIVE        8  u8  drive state, DriveState_t
 *                              u8  latched fault cause, DriveFault_t
 *                              u8  self-test failure cause, DriveFault_t
 *                              u8  reserved, 0
 *                              u16 fault count since boot
 *                              u16 milliseconds in the present state
 *
 *  TELEM_DRIVE carries what CAN_FLAG_FAULTED cannot: a single "faulted" bit
 *  says something tripped, and the thing a pit crew needs is WHICH. State and
 *  cause are one byte each rather than packed into the flags byte, which is
 *  full, and because an enum that grows is easier to extend than a bitfield
 *  that has to be re-cut.
 *
 *  All three are published at CAN_TELEM_HZ. Velocity and current are 16-bit
 *  because +/-32767 covers every value this drive can reach - 32767 deg/s is
 *  91 rev/s and 32 A is far beyond the bridge - and it keeps each message to
 *  one frame.
 */
#define CAN_MSG_TELEM_MOTION    0x10U
#define CAN_MSG_TELEM_STATE     0x11U
#define CAN_MSG_TELEM_DRIVE     0x12U

#define CAN_TELEM_HZ            50U

/* Reduced telemetry rate used while the bus error counters are past the
 * warning limit - see "Bus health" above. Still fast enough that a host
 * arriving on a repaired bus finds the drive without asking. */
#define CAN_TELEM_PROBE_HZ      1U

/* Minimum interval between bus-off recovery attempts. The hardware recovery
 * sequence itself is 129 x 11 recessive bits, about 1.4 ms at 1 Mbit/s; this
 * is much longer so that a permanently broken bus is retried calmly rather
 * than hammered. */
#define CAN_BUSOFF_RETRY_MS     200U

#define CAN_FLAG_ENABLED        (1U << 0)   /* servo engaged                */
#define CAN_FLAG_FOC_ON         (1U << 1)   /* current loop running         */
#define CAN_FLAG_FAULTED        (1U << 2)   /* overcurrent latch set        */
#define CAN_FLAG_GATES_ON       (1U << 3)   /* gate drivers enabled         */
#define CAN_FLAG_CMD_TIMEOUT    (1U << 4)   /* dropped to idle on silence   */
#define CAN_FLAG_BUS_WARN       (1U << 5)   /* an error counter >= 96       */
#define CAN_FLAG_BUS_PASSIVE    (1U << 6)   /* error-passive                */
#define CAN_FLAG_BUS_OFF_SEEN   (1U << 7)   /* has been bus-off since init  */

/* Note that BUS_OFF_SEEN is STICKY, and deliberately so. A node in bus-off
 * cannot transmit, so a live "I am bus-off right now" bit could never reach a
 * host - the one state most worth reporting is the one state that cannot be
 * reported. The sticky form survives the recovery and tells a host, on the
 * first frame after it, that this drive dropped off the bus and came back.
 * The live state is in CanTelem_t for whoever is on the debugger. */

#ifdef __cplusplus
}
#endif

#endif /* CAN_PROTO_H */
