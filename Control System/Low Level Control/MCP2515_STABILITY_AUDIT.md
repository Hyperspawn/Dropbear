# MCP2515 multi-actuator CAN stability audit

Generated: 2026-09-26

Scope: the ESP32 + MCP2515 leg controllers, six MyActuator motors per local
bus, the installed `MCP_CAN_lib` 1.5.1 driver, and the Behemoth observation /
control firmware.

## Executive result

The retry storm was real and was amplified by several firmware defects:

- a timed-out frame could outlive its application transaction and later reads
  could spill into the other MCP2515 transmit mailboxes;
- `MCP_CAN_lib::sendMsg()` treats cleared `TXREQ` as success without checking
  `TXERR`, `MLOA`, or `ABTF`, which reports false success in one-shot mode;
- sticky RX overflow and message-error state was handled with repeated full
  controller resets rather than bounded flag cleanup;
- automatic reads, diagnostics, and output did not have one exclusive RTOS
  transport owner; and
- the library's 8 MHz / 1 Mbit/s preset enables triple sampling (`CNF2=0xC0`),
  even though Microchip documents triple sampling as a slower-bus noise aid.

Firmware `.54` owns only TXB0, validates the wire result, performs all runtime
MCP2515/SPI I/O in one priority-5 CAN task, drains one explicitly flagged RX
mailbox at a time, uses deadline queues and timing counters, and changes the
marginal 8 MHz preset to single sampling
(`CNF1/CNF2/CNF3 = 00/80/80`). It also normalizes the vendor `0xB6/0x92`
active-reply slot before polling and retries a latched-offline address only
once every five seconds so a transiently missed motor can rejoin.

There is also a hardware timing limitation: the current configuration is a
1 Mbit/s bus with an 8 MHz MCP2515 clock. Both the installed library preset
`0x00/0xC0/0x80` and the deployed single-sample override `0x00/0x80/0x80`
produce a 1 TQ Phase Segment 2.
Microchip specifies a minimum valid PS2 of 2 TQ. This timing can appear to work
on a favorable bus but is not datasheet-compliant. Firmware can contain the
failure and expose it; a 16 MHz MCP2515 clock is the direct compliant fix while
retaining the motors' 1 Mbit/s bus rate.

## Evidence and decisions

### 1. Transmit lifetime must be owned by the application

- The MCP2515 has three transmit buffers and two receive buffers.
- `MCP_CAN_lib` 1.5.1 waits at most 2500 microseconds for a free transmit
  buffer, then at most another 2500 microseconds for `TXREQ` to clear.
- `CAN_SENDMSGTIMEOUT` means the driver loaded a mailbox but observed `TXREQ`
  still set at its deadline. It does not prove the frame failed, and it does
  not remove the frame.
- In the installed library, `CAN_GETTXBFTIMEOUT` means all three hardware
  mailboxes were still busy. `.54` instead owns TXB0 exclusively, so a busy
  mailbox is an ownership fault and is aborted before another request.
- One-shot mode limits a message to one actual transmit attempt even after
  arbitration loss or an error frame. Before that first attempt, `TXREQ` can
  remain pending while the controller waits for an available bus.

Decision: only one read transaction may be outstanding and only TXB0 may be
used. A deferred transmit owns that mailbox until either its matching reply
arrives or firmware performs the complete ABAT sequence. Priority motion/STOP
traffic cancels a deferred read before transmission.

### 2. Abort must be a complete sequence

Microchip requires ABAT to be reset after pending `TXREQ` bits clear so that
transmission can continue. The installed library's public `abortTX()` sets
ABAT but does not clear it. Firmware therefore performs the sequence under the
same SPI mutex used by the library:

1. set `CANCTRL.ABAT`;
2. wait a bounded interval for all three `TXBnCTRL.TXREQ` bits to clear;
3. clear `CANCTRL.ABAT`;
4. verify both conditions before allowing another transaction.

The operation aborts all mailboxes. That is acceptable for read-only
observation. During motion it is treated as a bus fault: torque state is
cleared, output authority is disarmed, and three bounded full-leg STOP batches
are attempted. Every motor is attempted in each batch; one absent actuator
cannot starve STOP delivery to the others or create an infinite retry storm.

### 3. Receive rollover was already configured

`MCP_CAN_lib` configures `RXB0CTRL.BUKT` in `MCP_ANY` mode, so RXB0 can roll
over into RXB1. Firmware `.54` no longer separates the library's
`checkReceive()` status read from its `readMsgBuf()` status read: it snapshots
`CANINTF`, reads that exact RX buffer in one burst, and clears only that
buffer's flag. This removed ambiguity in mailbox ownership but did not remove
the physical reply bursts, proving that stale application reads were not the
root cause.

Decision: use the active-low MCP2515 INT output to wake the sole receive task,
drain a bounded batch, and immediately continue when INT remains low. A 1 ms
timeout remains for query scheduling and health work, so a broken/noisy bus
cannot starve the rest of the controller.

### 4. 8 MHz / 1 Mbit timing is outside the valid envelope

For the current preset:

- `TQ = 2 × (BRP + 1) / FOSC = 250 ns`;
- Sync = 1 TQ, PropSeg = 1 TQ, PS1 = 1 TQ, PS2 = 1 TQ;
- total = 4 TQ = 1 microsecond = 1 Mbit/s;
- sample point = 75%;
- PS2 = 1 TQ, while the datasheet requires PS2 >= 2 TQ and describes the
  information-processing time as 2 TQ.

The library preset also sets `SAM=1`, so it samples at the nominal 75% point
and at two preceding half-TQ intervals. Microchip states that three-sample
majority mode was intended for noisy buses at slower rates. `.54` explicitly
enters configuration mode, writes and verifies `0x00/0x80/0x80`, then enters
normal mode. This keeps the 75% sample point but samples once.

The library's 16 MHz / 1 Mbit preset is `0x00/0xCA/0x81`, giving eight TQ and
a valid 2 TQ PS2 at the same nominal rate.

Decision: retain the verified single-sample 8 MHz mode to communicate with the
existing hardware and mark CAN timing health `warn`. For production stability,
fit 16 MHz MCP2515 hardware on both legs and change both initialization and
recovery to a verified 16 MHz profile. Do not switch the oscillator selection
until the physical modules are changed or measured.

## Firmware invariants

- One priority-5 task is the sole runtime MCP2515/SPI transport owner.
- Other tasks submit complete frames by value through a bounded deadline queue;
  motion/STOP enters at the front and the caller receives a correlated result.
- The transport checks `TXERR`, `MLOA`, and `ABTF`; cleared `TXREQ` alone is not
  delivery success.
- TXB0 is the only application transmit mailbox.
- Only one automatic `0x92` read is outstanding.
- `CAN_SENDMSGTIMEOUT` is accepted only for a read transaction and starts a
  bounded deferred-mailbox lifetime.
- Any unconfirmed motion transmit fails closed.
- STOP is three bounded full-leg attempts; all six addresses are attempted even
  when an earlier address fails.
- RX is drained before another automatic motor query.
- Missing-response addresses latch after three misses, then receive one
  isolated retry every five seconds; a valid reply clears the latch.
- Recovery re-enables one-shot mode and clears application transaction state.
- RX overflow flags are cleared after draining instead of triggering a reset;
  TXEP/RXEP alone do not trigger a reset loop.
- Diagnostic serial output never blocks the CAN task and every `DBC1` record is
  bounded below the host's 256-byte admission limit.
- Diagnostics expose EFLG, TEC, REC, mailbox outcomes, queue/wire/batch timing,
  RTOS budget misses, interrupt/wakeup counts, oscillator, and CNF readback.

## 2026-09-26 live validation

Both ESP32s were compiled from source SHA-256
`89ea5e04a36f7f4140d346f716a9f93c48647b99a0dbb3d9c9362fb6ede51f82`
and flashed with application binary SHA-256
`50c17e4ca3392ad46f811f2a58010d9eddc95e56b358b4b29e8984786ca590a6`.
SPIFFS remained at `0x290000` and was not written.

- Both report `behemoth-observation-protocol-2026.09.54` and CNF readback
  `00/80/80`.
- Right: the dashboard reports fresh mask `0x2F` (five motors); only configured
  yaw `0x14C` is absent. After more than nine minutes of uptime, a dashboard
  query read `TEC=1, REC=9`, followed by register readback `TEC=0, REC=0`, with
  no breaker trip. Earlier controlled windows showed REC transiently reaching
  115 before recovering. Ten RX overflows were cleared and reply counts can
  exceed request counts, so this remains degraded rather than motion-grade.
- Left: the dashboard reports fresh mask `0x2E` (four configured motors).
  Configured outer calf `0x141` and yaw `0x149` remain absent. The periodic
  retry restored inner calf `0x142` after a transient latch. Dashboard queries
  read `TEC=0, REC=1`, followed by register readback `TEC=0, REC=0`, with no
  breaker trip or RX overflow.
- The dashboard is enabled and running, both serial streams are fresh and
  complete, and the only reported unobserved joints are `left_outer_calf`,
  `left_hip_yaw`, and `right_hip_yaw`.
- Confirmed `0xB6,0x92,0` delivery did not remove right `0x148` reply bursts;
  a passive two-second sniff was quiet. Atomic RX-buffer ownership also did not
  remove them. This rules out a continuously enabled active-reply stream and a
  stale library mailbox as root causes, leaving the out-of-spec timing/ACK path
  as the limiting condition.
- No torque, position, enable, or other motion command was sent during these
  tests.

This run does not pass the five-minute motion-readiness gate: the right bus has
transient receive warnings and cleared overflows, and both yaw controllers are
still silent at their configured IDs. `.54` is accepted for read-only
diagnostics and dashboard observation only.

## Read-only acceptance gate

For each leg, over a minimum five-minute observation window:

1. exact expected firmware version and `one_shot_tx=true`;
2. no serial-record corruption during recovery;
3. no persistent `TXREQ` ownership and zero abort failures;
4. no growth in RX0OVR/RX1OVR after startup;
5. TEC/REC return to and remain near zero on a healthy bus;
6. no repeated five-second recovery loop;
7. each physically present motor has repeatable replies at its actual address;
8. missing configured addresses back off without starving responding motors;
9. play remains false and no motion opcode is sent during this gate.

Passing this gate validates the software containment. It does not make the
8 MHz / 1 Mbit timing profile compliant.

## Primary sources

- Microchip, *MCP2515 Family Data Sheet*, DS20001801K:
  https://ww1.microchip.com/downloads/en/DeviceDoc/MCP2515-Family-Data-Sheet-DS20001801K.pdf
- Microchip, *An In-depth Look at the MCP2510* (AN739), including the guidance
  that triple sampling is intended for noisy, slower buses:
  https://ww1.microchip.com/downloads/en/Appnotes/00739a.pdf
- Cory J. Fowler, official `MCP_CAN_lib` source and 1.5.1 release:
  https://github.com/coryjfowler/MCP_CAN_lib/blob/master/mcp_can.cpp
  and https://github.com/coryjfowler/MCP_CAN_lib/releases/tag/1.5.1
- MYACTUATOR, official RMD-X downloads and current protocol/manual index:
  https://www.myactuator.com/downloads-xseries
