# CAN protocol — Isurus

Wire format for controlling this drive over CAN.

The normative definition is [`Core/Inc/can_proto.h`](../Core/Inc/can_proto.h).
That header carries no STM32 or HAL dependency, so a host — a Python script,
another MCU, a test rig — can include or transcribe it and agree on the wire
format by construction rather than by comment. This document explains the
framing and how to speak it; where the two disagree, the header wins.

Transport implementation: [`Core/Src/can.c`](../Core/Src/can.c).

## Contents

- [Physical layer](#physical-layer)
- [Frame format](#frame-format)
- [Identifier layout](#identifier-layout)
- [Byte order and units](#byte-order-and-units)
- [Host to drive: commands](#host-to-drive-commands)
- [Drive to host: telemetry](#drive-to-host-telemetry)
- [Worked examples](#worked-examples)
- [Limits and saturation](#limits-and-saturation)
- [Command watchdog](#command-watchdog)
- [Bus loading](#bus-loading)
- [Receive filtering](#receive-filtering)
- [Bus health and recovery](#bus-health-and-recovery)
- [Testing without a second node](#testing-without-a-second-node)
- [Production readiness](#production-readiness)

## Physical layer

| | |
|---|---|
| Protocol | Classic CAN 2.0A. **Not CAN FD.** |
| Identifiers | 11-bit standard |
| Bit rate | 1 Mbit/s (`CAN_BITRATE_BPS`) |
| Sample point | 81.25%, the CiA recommendation |
| Peripheral | FDCAN2 |
| Pins | PB6 (TX), PB5 (RX) |

Bit timing is derived from a 160 MHz kernel clock: prescaler 10 gives a 16 MHz
time-quantum clock, and 1 sync + 12 + 3 = 16 tq per bit is exactly 1 Mbit/s.
The sample point at (1 + 12) / 16 is what other nodes on a bus will have been
set up for.

Only the prescaler differs from Mako Longfin, which ran the same segment split
off a 128 MHz clock with a prescaler of 8. The sample point is therefore
identical to the one validated there. Carrying the old prescaler onto this
board would have given 1.25 Mbit/s, which does not present as a wrong number
anywhere - the node just never acknowledges a frame, and every other node
trying to talk to it goes error-passive.

Classic frames rather than FD because every message here fits in 8 bytes and
2.0 talks to every analyser, transceiver and node in existence. Moving to FD
later would change the frame format only, not the protocol.

> **The BOOT0 conflict does not apply to this board.** On Mako Longfin the CAN
> RX line was PB8, which is also BOOT0, so a chip with factory option bytes
> booted the ST system bootloader instead of the application - the transceiver
> idles RX high. The Mako Desori does not use PB8 at all. The story is still
> worth knowing, because the symptom was a board that looked like broken
> firmware in every respect:
> [HARDWARE_NOTES.md section 1](../HARDWARE_NOTES.md#1-boot0--fdcan1_rx-pin-conflict-critical).

## Telemetry units, and why they are not the internal ones

Commands are in millivolts, milliamps and degrees per second, matching the
firmware's own units. **Telemetry is not.** The bus is reported in centivolts
and currents in deciamps, and velocity in RPM.

That asymmetry is deliberate and it is the fix for a real defect. Four
telemetry fields were sized for Mako Longfin, a 50 V / 12 A bench drive, and
all four overflow on this board:

| field | type | caps at | this board needs |
|---|---|---|---|
| bus | `u16` mV | 65.5 V | 588 V |
| `iq_max` | `u16` mA | 65.5 A | 339 A |
| `iq` | `i16` mA | ±32.8 A | ±339 A |
| velocity | `i16` dps | 32767 | 33600 |

The velocity field is the one worth dwelling on. It overflows by 2.5%, so it
reads correctly across the whole useful speed range and then wraps to a large
negative number near maximum rpm — a channel that lies only at full speed.

Centivolts buy 655 V and deciamps ±3276 A, against a machine that needs 588 V
and 339 A. Currents were briefly centiamps, which caps at 327.67 A — enough
for the 160 A limit in force at the time, and an overflow again as soon as
`LIM_IQ_MAX_MA` was corrected to the motor's 339 A peak. Sizing a wire field
against the current value of a limit, rather than against what the machine can
physically do, is how this protocol collected four overflows to begin with.

Resolution is not the constraint anywhere here: the current chain resolves
314 mA per ADC count and the bus about 320 mV, both coarser than the 100 mA
and 10 mV the wire carries. Velocity is in RPM
because a traction motor's datasheet, limits and every conversation about it
already are.

Commands were left alone because they are 32-bit fields and never overflowed.

**This breaks wire compatibility with Mako Longfin hosts**, deliberately. A
host targets one board — that is what the branch-per-board convention means —
and the alternative, keeping a broken field alongside a working one, leaves
two ways to read the same quantity of which one is wrong.

## Frame format

Every frame is a standard data frame. No remote frames — they are rejected in
the global filter configuration. No extended identifiers.

```
 +-------+------------------+-----+------------+------------------+-----+-----+
 |  SOF  |  11-bit ID       | RTR | IDE r0 DLC |  0..8 data bytes | CRC | ACK |
 | 1 bit |  node<<5 | cmd   |  0  |  0   0   n |  little-endian   |     |     |
 +-------+------------------+-----+------------+------------------+-----+-----+
```

DLC is the true byte count for every message in this protocol (0, 1, 4 or 8).
The receiver reads the DLC field and checks it against the command's expected
length before decoding.

Length handling on receive is **lenient in one direction only**: a frame
carrying *more* bytes than the command needs is accepted and the extra bytes
are ignored; a frame carrying *fewer* is rejected and increments `rx_bad_len`.
Hosts should send the exact length.

## Identifier layout

The 11-bit identifier splits into a node address and a command:

```
    bit  10  9  8  7  6  5   4  3  2  1  0
        +------------------+---------------+
        |   node  (6 bit)  |  cmd  (5 bit) |
        +------------------+---------------+
             1..63,            0x00..0x1F
             0 = broadcast
```

```c
#define CAN_ID(node, cmd)  ((uint16_t)((((node) & 0x3FU) << 5) | ((cmd) & 0x1FU)))
#define CAN_ID_NODE(id)    ((uint8_t)(((id) >> 5) & 0x3FU))
#define CAN_ID_CMD(id)     ((uint8_t)((id) & 0x1FU))
```

The 5-bit command field is split by direction:

| Range | Direction |
|---|---|
| `0x00`–`0x0F` | host → drive, commands |
| `0x10`–`0x1F` | drive → host, telemetry |

**That split is what lets a drive accept only the command half in its receive
filter.** Without it a node accepts its own telemetry identifiers — harmless on
a healthy bus, but it shows up immediately under loopback, and any echo or
bridge on the bus would feed a drive its own state as though it were a command.

A node accepts frames addressed to **its own node id and to node 0**, so one
frame can stop every drive on the bus.

### Priority is not an accident

CAN arbitration gives the numerically lowest identifier the bus, and the layout
above is chosen so that falls out correctly. Command `0x00` is ESTOP, so a
broadcast e-stop is identifier **`0x000`** — the highest-priority frame that can
exist on an 11-bit bus. Setpoint traffic can never delay it.

### Identifier map for `CAN_NODE_ID = 1`

| ID | Direction | Message |
|---|---|---|
| `0x000` | host → all | **ESTOP, broadcast** |
| `0x001`–`0x00F` | host → all | broadcast form of every other command |
| `0x020` | host → node 1 | ESTOP |
| `0x021` | host → node 1 | SET_MODE |
| `0x022` | host → node 1 | SET_TORQUE |
| `0x023` | host → node 1 | SET_VELOCITY |
| `0x024` | host → node 1 | SET_POSITION |
| `0x025` | host → node 1 | SET_LIMITS |
| `0x026` | host → node 1 | SET_PROFILE |
| `0x027` | host → node 1 | ZERO_HERE |
| `0x028` | host → node 1 | CLEAR_FAULT |
| `0x029` | host → node 1 | SET_ENABLE |
| `0x02A` | host → node 1 | SET_TORQUE_MNM |
| `0x030` | node 1 → host | TELEM_MOTION |
| `0x031` | node 1 → host | TELEM_STATE |

Node *n* occupies `n << 5` through `(n << 5) | 0x1F`. Node 63 is the last, at
`0x7E0`–`0x7FF`.

`CAN_NODE_ID` is a compile-time constant, defaulting to 1. A multi-drop bus
needs one build per drive today.


### Two torque commands, and why

`0x02 SET_TORQUE` carries **milliamps of iq**, whatever its name suggests.
`0x0A SET_TORQUE_MNM` carries **milli-newton-metres** and is the one a VCU
should use.

0x02 was not redefined. Same identifier and same four-byte length with a
different meaning is the most dangerous shape a protocol change can take: a
host still sending `1000` would go from asking for 1 A to asking for 1 Nm -
about 12 A on this machine - and nothing anywhere would report an error. A new
identifier makes an old host's frames simply not match.

The conversion is `T = 1.5 * p * lambda_m * iq`, done at the CAN edge so
everything downstream keeps working in milliamps. `id` stays at zero: this is
a surface-magnet machine, the reluctance term is identically zero, and MTPA
collapses to `id = 0`. See the torque interface note in `foc.h`.

## Byte order and units

**Little-endian**, matching the MCU, so payloads map straight onto the structs
with no swapping on the target. Hosts should pack explicitly — in Python,
`struct.pack('<i', ...)`.

The transport reads and writes payloads byte by byte rather than casting the
buffer to an `int32_t`: the RX buffer has no alignment guarantee, an unaligned
32-bit load faults on some cores and is silently slow on others, and doing it
by hand pins the byte order to this spec instead of to whatever the compiler
happens to do.

Every quantity on the wire is a **scaled integer, never a float.**

**Commands and telemetry do not use the same units**, and the reason is in
"Telemetry units" above: command fields are 32-bit and never overflowed, so
they kept the firmware's own units, while four telemetry fields overflowed on
588 V / 160 A hardware and had to be rescaled.

| Quantity | In commands | In telemetry | Notes |
|---|---|---|---|
| Position | tenths of a mechanical degree | same | signed, **multi-turn** — 3600 is one full turn, no wrap to reason about |
| Velocity | degrees per second | **RPM** | signed |
| Current | milliamps of iq | **deciamps** | signed; for this motor iq is torque-producing current |
| Acceleration | deg/s² | — | |
| Jerk | deg/s³ | — | 0 selects a trapezoid profile instead of S-curve |
| Bus voltage | — | **centivolts** | unsigned |

## Host to drive: commands

| cmd | Name | DLC | Payload |
|---|---|---|---|
| `0x00` | `ESTOP` | 0 | — |
| `0x01` | `SET_MODE` | 1 | `u8` mode |
| `0x02` | `SET_TORQUE` | 4 | `i32` milliamps of iq |
| `0x03` | `SET_VELOCITY` | 4 | `i32` deg/s |
| `0x04` | `SET_POSITION` | 4 | `i32` tenths of a degree, multi-turn |
| `0x05` | `SET_LIMITS` | 8 | `i32` iq_max mA, `i32` vel_max deg/s |
| `0x06` | `SET_PROFILE` | 8 | `i32` accel deg/s², `i32` jerk deg/s³ |
| `0x07` | `ZERO_HERE` | 0 | — |
| `0x08` | `CLEAR_FAULT` | 0 | — |
| `0x0A` | `SET_TORQUE_MNM` | 4 | `i32` milli-newton-metres |
| `0x09` | `SET_ENABLE` | 1 | `u8` 0 = disengage, 1 = engage |

Anything in `0x0B`–`0x0F` is unknown and increments `rx_bad_cmd`.

### Mode values

| Value | Mode |
|---|---|
| 0 | idle — no torque |
| 1 | torque |
| 2 | velocity |
| 3 | position |

The firmware also has a haptic mode (4), but `SET_MODE` currently rejects
anything above 3, so it is not reachable over CAN — see
[Production readiness](#production-readiness).

### Command semantics

**`ESTOP`** — servo off, FOC off, gate drivers opened via
`MotorPwm_EmergencyStop()`, which writes DIS high and disconnects every HRTIM
output in two single stores. This is the big hammer. Note that ESTOP does *not*
set the fault latch, so recovery is a `SET_ENABLE 1`; the `CLEAR_FAULT` step
described in `can_proto.h` is not actually required.

**`SET_MODE`** — a setpoint command does **not** change mode. The two are
separate so a host can pre-load a setpoint and switch atomically, rather than
the drive guessing which mode a bare number implies.

**`SET_ENABLE 1`** — asks the main loop to bring the bridge up in the required
order: duty to zero, then HRTIM outputs, then gate drivers. CAN never touches
the peripheral directly, so the CAN path and the SWD bench path cannot disagree
about whether the stage is live. **It also clears the fault latch** — see
[Production readiness](#production-readiness).

**`ZERO_HERE`** — makes the present rotor position the origin, in software.
This is not the encoder's own programmed zero offset, which lives in the A1333's
EEPROM; see
[HARDWARE_NOTES.md section 8](../HARDWARE_NOTES.md#encoder-zero-programmed-into-the-sensor).

**`CLEAR_FAULT`** — clears a latched overcurrent trip. It does not check that
the condition has gone away; if it has not, the fault path re-latches.

## Drive to host: telemetry

Both messages are published at `CAN_TELEM_HZ` = **50 Hz**, unsolicited, from
boot. There is no polling command.

### `0x10` TELEM_MOTION — DLC 8

```
 byte   0        1        2        3        4        5        6        7
      +----------------------------------+-----------------+-----------------+
      | i32  position, tenths of a degree| i16 velocity    | i16 iq          |
      |      multi-turn, signed          |     RPM         |     deciamps    |
      +----------------------------------+-----------------+-----------------+
```

### `0x11` TELEM_STATE — DLC 8

```
 byte   0        1        2        3        4        5        6        7
      +--------+--------+-----------------+-----------------+-----------------+
      | u8     | u8     | u16 bus         | i16 following   | u16 iq_max      |
      | mode   | flags  |     centivolts  |     err, deg*10 |     deciamps    |
      +--------+--------+-----------------+-----------------+-----------------+
```

Velocity and current are 16-bit because, IN THESE UNITS, ±32767 covers
everything this drive can reach — ±32767 rpm against a 5600 rpm machine and
±3276 A against a 339 A limit — and it keeps
each message to a single frame. Both are **saturated, not wrapped**, on the way
out: a velocity that overflowed would otherwise be reported with the opposite
sign, which is worse than being clipped.

### Flags byte

| Bit | Mask | Name | Meaning |
|---|---|---|---|
| 0 | `0x01` | `CAN_FLAG_ENABLED` | servo engaged |
| 1 | `0x02` | `CAN_FLAG_FOC_ON` | current loop running |
| 2 | `0x04` | `CAN_FLAG_FAULTED` | overcurrent latch set |
| 3 | `0x08` | `CAN_FLAG_GATES_ON` | gate drivers enabled — **never set, always reads 0** |
| 4 | `0x10` | `CAN_FLAG_CMD_TIMEOUT` | dropped to idle on silence |
| 5 | `0x20` | `CAN_FLAG_BUS_WARN` | an error counter has reached 96 |
| 6 | `0x40` | `CAN_FLAG_BUS_PASSIVE` | error-passive |
| 7 | `0x80` | `CAN_FLAG_BUS_OFF_SEEN` | **sticky** — has been bus-off since init |

`CAN_FLAG_BUS_OFF_SEEN` is sticky on purpose. A node in bus-off cannot
transmit, so a live "I am bus-off right now" bit could never reach a host — the
one state most worth reporting is the one state that cannot be reported. The
sticky form survives recovery and tells a host, on the first frame after it,
that this drive dropped off the bus and came back. The live state is in
`CanTelem_t` for whoever is on the debugger.

## Worked examples

All for node 1. Bytes are shown in transmission order.

| Intent | ID | DLC | Data |
|---|---|---|---|
| E-stop every drive on the bus | `0x000` | 0 | — |
| E-stop node 1 only | `0x020` | 0 | — |
| Engage the bridge | `0x029` | 1 | `01` |
| Select position mode | `0x021` | 1 | `03` |
| Go to +90.0° | `0x024` | 4 | `84 03 00 00` |
| Go to −1 turn (−3600) | `0x024` | 4 | `F0 F1 FF FF` |
| Hold +2.500 A of iq | `0x022` | 4 | `C4 09 00 00` |
| Spin at 720 deg/s | `0x023` | 4 | `D0 02 00 00` |
| Limit to 6 A and 1800 deg/s | `0x025` | 8 | `70 17 00 00 08 07 00 00` |
| Accel 20000 deg/s², trapezoid | `0x026` | 8 | `20 4E 00 00 00 00 00 00` |
| Zero the position here | `0x027` | 0 | — |
| Disengage | `0x029` | 1 | `00` |

A minimal host, using python-can:

```python
import can, struct

NODE = 1

def cid(cmd):
    return (NODE << 5) | cmd

def i32(v):
    return struct.pack("<i", v)

bus = can.interface.Bus(channel="can0", bustype="socketcan", bitrate=1000000)

def send(cmd, data=b""):
    bus.send(can.Message(arbitration_id=cid(cmd),
                         data=data, is_extended_id=False))

send(0x09, b"\x01")            # SET_ENABLE 1
send(0x01, b"\x03")            # SET_MODE position
send(0x04, i32(900))           # SET_POSITION +90.0 deg

# Keep the watchdog fed: something addressed to this node at least every
# 500 ms. Re-sending the current setpoint is the usual way.

for msg in bus:
    cmd = msg.arbitration_id & 0x1F
    if cmd == 0x10:
        pos, vel, iq = struct.unpack("<ihh", msg.data)
        print("%8.1f deg  %6d deg/s  %6d mA" % (pos / 10.0, vel, iq))
    elif cmd == 0x11:
        mode, flags, vbus, err, iqmax = struct.unpack("<BBHhH", msg.data)
        print("mode %d  flags 0x%02x  %d mV  err %.1f deg" %
              (mode, flags, vbus, err / 10.0))
```

## Limits and saturation

A host cannot command past what the machine can physically do. Every setpoint
and limit is saturated into the bounds in
[`Core/Inc/limits.h`](../Core/Inc/limits.h) — **in the motion loop, not in the
transport**, so the same bounds apply to the SWD tools and to anything a
debugger writes by hand.

| Bound | Value | Traceable to |
|---|---|---|
| Current | 12 A | below the 15 A bench overcurrent trip, itself well below the motor's 22 A continuous and the sensors' ±40 A range |
| Bus | 50.4 V | a full 12S pack. Exceeding it latches the fault path and kills the stage |
| Speed | 3600 deg/s | |
| Position | ±100 turns | |
| Acceleration | 10 – 100000 deg/s² | |
| Jerk | ≤ 2000000 deg/s³ | |

**Saturation is silent by nature, so it is also counted.** A frame carrying a
value outside the bounds is still **accepted** and applied at the limit, and
increments `rx_out_of_range`. A host that increments that counter is asking for
something impossible and should be fixed; it is not an error the drive can
resolve for you.

## Command watchdog

Once a drive has accepted its first command frame it starts requiring them. If
`CAN_CMD_TIMEOUT_MS` = **500 ms** passes with no addressed frame, the drive
drops to idle on its own and sets `CAN_FLAG_CMD_TIMEOUT`.

A control link that has gone quiet is not the same as a control link asking for
the last setpoint forever, and a position loop holding a stale setpoint against
a jammed shaft is exactly the failure this exists to prevent.

Two properties worth knowing:

- **It only arms after the first frame**, so a drive being driven over SWD with
  no CAN host present is unaffected.
- **It drops to idle but leaves the bridge up**, so a host that comes back can
  resume without re-arming. The motor makes no torque in the meantime. The
  timeout clears the moment any addressed frame arrives.

## Bus loading

Each drive transmits 2 frames every 20 ms. A classic 11-bit data frame with 8
bytes is 111 bits before stuffing, about 128 bits worst case, so:

```
100 frames/s * 128 bits = 12.8 kbit/s  =  1.3% of a 1 Mbit/s bus, per drive
```

Ten drives is 13% before any command traffic. Frame time at 1 Mbit/s is roughly
130 us, which is the number behind the polled-receive design below.

That is the healthy-bus figure. A drive whose frames are going unacknowledged
drops to `CAN_TELEM_PROBE_HZ` — see
[Bus health and recovery](#bus-health-and-recovery).

## Receive filtering

Two hardware acceptance filters, both routing into RX FIFO 0:

| Filter | Range | Covers |
|---|---|---|
| 0 | `CAN_ID(CAN_NODE_ID, 0x00)` … `CAN_ID(CAN_NODE_ID, 0x0F)` | this node's commands |
| 1 | `CAN_ID(0, 0x00)` … `CAN_ID(0, 0x0F)` | broadcast commands |

Everything else — including this node's own telemetry identifiers and every
other drive's traffic — is rejected in hardware, so a busy bus costs this node
nothing. Remote frames are rejected outright.

The transport is **polled from the main loop, not interrupt-driven**. The
30 kHz control ISR already owns the tightest deadline on this MCU, and a CAN
frame is not urgent by comparison. Keeping CAN out of interrupt context means it
can never delay a control step. See
[Production readiness](#production-readiness) for the limit of that argument.

### Counters

`Can_GetTelem()` fills a `CanTelem_t`, mirrored to `g_can` for the SWD tools:

| Field | Meaning |
|---|---|
| `rx_frames` | addressed frames accepted |
| `rx_ignored` | frames for another node |
| `rx_bad_len` | right command, payload too short |
| `rx_bad_cmd` | unknown command id |
| `rx_out_of_range` | accepted, but a field was saturated |
| `tx_frames` | telemetry frames queued |
| `tx_errors` | the TX queue rejected the frame |
| `timeouts` | times the command watchdog fired |
| `last_rx_id` / `last_rx_tick` | identifier and HAL tick of the last accepted frame |
| `armed` | 1 once a command has ever arrived |
| `init_rc` | 0 = up |
| `tec` / `rec` | transmit and receive error counters |
| `bus_warn` / `bus_passive` / `bus_off` | live bus state |
| `bus_off_events` | times bus-off has been entered |
| `bus_recoveries` | recovery sequences started |
| `tx_suppressed` | telemetry frames withheld by the backoff |

## Bus health and recovery

A CAN node that transmits into a bus nobody is acknowledging counts its way to
**bus-off**, and a bus-off FDCAN sets `CCCR.INIT` in hardware and stops
transmitting **and receiving**. That last part is what makes this a safety
matter rather than a diagnostic one: a drive that has counted itself off the bus
can no longer be reached by an ESTOP.

Nothing about that is hypothetical here. This node publishes telemetry from boot
whether or not a host exists, so an unplugged host, an unterminated bus or a
single-node bench is enough to trigger it.

Three mechanisms keep it survivable, in the order they act:

**1. Transmission is single-shot.** `AutoRetransmission` is `DISABLE`. A frame
gets one attempt; a failure costs 8 transmit-error counts and the frame is
dropped rather than retried into the same silence.

This was `ENABLE`, on the stated reasoning that "a dropped ESTOP is
catastrophic" — but **this node never transmits an ESTOP.** ESTOP is host →
drive. Everything the drive puts on the wire is periodic telemetry, replaced
20 ms later by a fresher copy of itself, so retransmission was protecting
nothing while costing a great deal: an unacknowledged frame is retried as fast
as the bus allows, each attempt adding another 8, which reaches bus-off in
milliseconds. The cost of single-shot is that a frame losing arbitration is
dropped rather than retried; at 1.3% bus load per drive that is rare, and the
next telemetry frame is 20 ms away.

**2. Telemetry backs off.** Once an error counter reaches the warning limit of
96, publishing drops from `CAN_TELEM_HZ` (50) to `CAN_TELEM_PROBE_HZ` (1).

```
50 Hz, single-shot, nobody listening   100 frames/s * 8 = 800 counts/s   -> bus-off in ~0.3 s
 1 Hz probe                              2 frames/s * 8 =  16 counts/s   -> bus-off in ~10 s
```

Every *successful* transmission takes one count back off again, so a host
returning to a repaired bus walks the counters down. The node keeps announcing
itself either way, just slowly. `tx_suppressed` counts what the backoff costs,
in frames.

**3. The drive recovers itself.** If it reaches bus-off anyway, `Can_CheckBus()`
detects it, stands the motion down, and clears `CCCR.INIT` — which is what
starts the hardware recovery sequence (129 × 11 recessive bits, about 1.4 ms at
1 Mbit/s). Retries are paced no faster than `CAN_BUSOFF_RETRY_MS` (200 ms) so a
permanently broken bus is retried calmly rather than hammered.

`INIT` is cleared directly rather than through `HAL_FDCAN_Start`, which refuses
unless the handle is in state `READY` — and after a bus-off the handle is still
`BUSY`, because nothing told the HAL that the hardware stopped.

The net effect on a dead bus is a cycle of roughly ten seconds of slow probing,
a bus-off, and a recovery measured in milliseconds. **The window in which the
drive cannot hear an ESTOP goes from permanent to about one main-loop pass plus
1.4 ms.**

### Standing down

Bus-off means the control link is provably gone, so it stands the motion down
immediately rather than waiting out the 500 ms command watchdog. It uses the
same path: **idle, bridge left up**, so a host returning to a repaired bus
resumes without re-arming.

It is **arm-gated exactly like the watchdog** — a drive that has never accepted
a CAN frame is not stopped by a bus it was never using. That keeps a bench drive
running over SWD, with a transceiver attached and no host, unaffected.

## Testing without a second node

`Can_Init(1)` selects the peripheral's internal loopback mode, where the node
receives its own transmissions and never drives the bus. That makes the whole
protocol path — filters, framing, decode — testable on a bench with no second
node and no transceiver traffic.

Over SWD: set `g_can_loopback = 1`, then `g_can_reinit = 1`. The `g_can_tx_*`
globals then transmit a real protocol frame at the node, exercising the same
receive path a host would use.

The self-test transmit is triggered by its own `g_can_tx_go` flag rather than by
a non-zero command id — ESTOP is command `0x00`, so keying the trigger off the
command value would have made the one frame most worth testing the one frame
impossible to send.

## Production readiness

The protocol design is sound and the receive path decodes correctly. What
follows is what stands between it and unattended operation on a real bus.

### Resolved

- **Bus-off detection and recovery.** Transmission is now single-shot, telemetry
  backs off to 1 Hz once the error counters pass the warning limit, and
  `Can_CheckBus()` detects bus-off, stands the motion down and clears
  `CCCR.INIT` to start the hardware recovery sequence. The window in which the
  drive cannot hear an ESTOP goes from permanent to about one main-loop pass
  plus 1.4 ms. See [Bus health and recovery](#bus-health-and-recovery).
- **Error-state visibility.** `tec`, `rec` and the live bus state are in
  `CanTelem_t`, and three flag bits carry warning, error-passive and a sticky
  bus-off-seen onto the wire.

> **Neither is verified on hardware yet.** The reasoning and the register
> behaviour are checked against the reference manual and the HAL, but nothing
> has been exercised against a real unterminated bus — internal loopback cannot
> provoke a bus-off, because the node acknowledges itself. What to confirm on a
> bench: `bus_off_events` incrementing, `bus_recoveries` tracking it,
> `tx_suppressed` climbing during the backoff, and a command frame still being
> accepted across the episode.

### Blocking

**1. RX FIFO overflow is possible and uncounted.** RX FIFO 0 holds **3
elements** on this part. `can.h` argues the FIFO cannot overrun between polls
because a frame takes ~130 us and the main loop runs far faster — which is true
until `Telem_Printf` runs. It blocks until the last character leaves the UART,
and a ~70-character line at 115200 baud is about **6 ms**, roughly 45 frame
times. The FIFO holds 3. Frames are then dropped silently: there is no overflow
counter and `RXF0S` is never read. Either drop the UART telemetry from
production builds, make it non-blocking, or add an overflow counter — but the
"cannot overrun" claim should not be relied on as written.

**2. `SET_ENABLE 1` silently clears the fault latch.** On the rising edge of
`g_can_wants_bridge`, `main.c` does `g_faulted = 0U` before bringing the stage
up. A host can therefore re-enable straight through a latched overcurrent
without ever acknowledging it and without sending `CLEAR_FAULT`. Combined with
the fact that ESTOP does not set `g_faulted` at all, the recovery sequence
documented in `can_proto.h` ("CLEAR_FAULT and then ENABLE, which is deliberate")
does not describe what the code does. Decide which behaviour is intended and
make the other one match.

### Should fix before shipping

**3. `CAN_FLAG_GATES_ON` is specified but never populated.** Bit 3 of
TELEM_STATE always reads 0. Either fill it from the PWM telemetry's `gate_en` or
remove it from the protocol; a documented flag that is permanently false is
worse than no flag.

**4. Mode validation increments the wrong counter.** An out-of-range `SET_MODE`
bumps `rx_bad_len`, which will send whoever debugs the host looking at payload
lengths. It needs its own `rx_bad_value`.

**5. Haptic mode is unreachable over CAN.** `MOTION_MODE_HAPTIC` is 4;
`Can_Handle` accepts modes up to `MOTION_MODE_POSITION` only. Either raise the
bound or note in the spec that haptic is SWD-only by design.

**6. Multi-field commands apply non-atomically.** `SET_LIMITS` and
`SET_PROFILE` each write two 32-bit fields that the 30 kHz ISR reads. Individual
words are atomic; the pair is not, so the ISR can observe a new `iq_max` against
an old `vel_max` for one tick. Almost certainly benign here, but it is a real
race and the fix — stage into a struct, set an apply flag the ISR consumes — is
cheap.

**7. The node address is compile-time.** `CAN_NODE_ID` defaults to 1 and needs a
rebuild per drive. Fine for one axis; a multi-drop machine wants an ID strap or
a stored configuration.

### Deliberate, but confirm they match your requirements

- **The watchdog leaves the bridge energised** when it drops to idle. That is
  documented and intentional, and it is the right call for a resumable link. It
  is the wrong call if a silent host should mean a de-energised machine.
- **There is no authentication, sequence numbering or replay protection.**
  Normal for CAN, and appropriate for a private motor bus. It does mean anything
  on the bus can e-stop or drive any node.
- **Overlong frames are accepted.** A `SET_MODE` carrying 8 bytes is honoured
  using byte 0. Lenient by choice.
