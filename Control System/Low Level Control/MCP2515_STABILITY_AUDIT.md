# MCP2515 multi-actuator CAN stability audit

Generated: 2026-09-26

Scope: the ESP32 + MCP2515 leg controllers, six MyActuator motors per local
bus, the installed `MCP_CAN_lib` 1.5.1 driver, and the Behemoth observation /
control firmware.

## Executive result

The retry storm is real and has two causes. Firmware allowed an MCP2515 frame
that timed out while still marked `TXREQ` to outlive the corresponding
application transaction; subsequent reads could therefore fill all three
hardware transmit buffers. One-shot mode prevents retransmission after a first
attempt, but it does not cancel a frame still waiting for an idle bus.

There is also a hardware timing limitation: the current configuration is a
1 Mbit/s bus with an 8 MHz MCP2515 clock. The installed library uses
`CNF1/CNF2/CNF3 = 0x00/0xC0/0x80`, which produces a 1 TQ Phase Segment 2.
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
- `CAN_GETTXBFTIMEOUT` means all three hardware mailboxes were still busy.
- One-shot mode limits a message to one actual transmit attempt even after
  arbitration loss or an error frame. Before that first attempt, `TXREQ` can
  remain pending while the controller waits for an available bus.

Decision: only one read transaction may be outstanding. A deferred transmit
owns its MCP2515 mailbox until either its matching reply arrives or firmware
performs the complete ABAT sequence. No next read may be submitted first.

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
cleared, output authority is disarmed, and STOP retries remain queued.

### 3. Receive rollover was already configured

`MCP_CAN_lib` configures `RXB0CTRL.BUKT` in `MCP_ANY` mode, so RXB0 can roll
over into RXB1. Its `readMsg()` also clears the consumed RX0IF/RX1IF flag.
The observed RX0 overflow was therefore not caused by missing rollover setup.
It was consistent with delayed/bunched traffic and insufficient receive
service latency.

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

The library's 16 MHz / 1 Mbit preset is `0x00/0xCA/0x81`, giving eight TQ and
a valid 2 TQ PS2 at the same nominal rate.

Decision: retain 8 MHz mode only to communicate with the existing hardware and
mark CAN health `warn`. For production stability, fit 16 MHz MCP2515 hardware
on both legs and change both initialization/recovery calls to `MCP_16MHZ`.
Do not switch this constant until the physical oscillator is verified.

## Firmware invariants

- All MCP2515/SPI operations use one mutex because this driver stores message
  ID, length, flags, and payload in mutable object-level fields.
- Only one automatic `0x92` read is outstanding.
- `CAN_SENDMSGTIMEOUT` is accepted only for a read transaction and starts a
  bounded deferred-mailbox lifetime.
- Any unconfirmed motion transmit fails closed.
- STOP bursts decrement only after all six local stop frames are confirmed by
  the driver; failed bursts remain pending.
- RX is drained before another automatic motor query.
- Missing motors use per-address backoff; they cannot consume every poll slot.
- Recovery re-enables one-shot mode and clears application transaction state.
- Diagnostics expose EFLG, TEC, REC, one-shot state, deferred ownership, abort
  counts, interrupt/wakeup counts, maximum RX batch, oscillator, bitrate, and
  timing-compliance state.

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
- Cory J. Fowler, official `MCP_CAN_lib` source and 1.5.1 release:
  https://github.com/coryjfowler/MCP_CAN_lib/blob/master/mcp_can.cpp
  and https://github.com/coryjfowler/MCP_CAN_lib/releases/tag/1.5.1
- MYACTUATOR, official RMD-X downloads and current protocol/manual index:
  https://www.myactuator.com/downloads-xseries

