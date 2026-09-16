# Dropbear Universal ESP32 Firmware

Unified low-level firmware for the Dropbear robot's ESP32-class subsystem controllers.

This single Arduino sketch is designed to be flashed onto multiple Dropbear ESP32 controllers and then configured, through SPIFFS, USB Serial, or the captive setup portal, to become one of several **mutually exclusive hardware personalities**.

The firmware currently supports:

```text
DROPBEAR UNIVERSAL ESP32 FIRMWARE
│
├── LEFTLEG
│   ├── Standalone / original Dropbear control
│   └── HyperSpawn / ROS2 CAN route
│
├── RIGHTLEG
│   ├── Standalone / original Dropbear control
│   └── HyperSpawn / ROS2 CAN route
│
├── CENTER
│   └── TCA9548A + five-IMU polling stack
│
└── HEADNECK
    └── six-axis A4988 / FastAccelStepper neck controller
```

All personalities share the same:

- SPIFFS configuration system,
- role-aware Wi-Fi captive portal,
- DB1 addressed command router,
- USB Serial transport,
- structured logging,
- diagnostics framework,
- configuration UI,
- reboot/topology semantics,
- safety-oriented command routing.

The role-specific hardware graphs are **never initialized simultaneously**.

> **Important:** `LEFTLEG`, `RIGHTLEG`, `CENTER`, and `HEADNECK` use overlapping GPIOs. Changing hardware role is a reboot-bound topology change. Never treat role selection as a live GPIO-mode switch.

The firmware file documented by this README is:

```text
firmware_full_libs_neck.ino
```

## USB angle telemetry

Leg roles emit a versioned `DB2` line at 50 Hz:

```text
DB2,<controller_ms>,<outer AS5600>,<inner AS5600>,<hip-pitch AS5600>,<knee-actuator AS5600>,<hip-roll AS5600>,<outer motor>,<inner motor>,<hip-pitch motor>,<knee motor>,<hip-yaw motor>,<hip-roll motor>
```

Every angle is in degrees. The five AS5600 fields remain independent absolute
measurements. The six motor fields come from read-only RMD V4.4 `0x92`
multi-turn-angle replies at `0.01°` per least-significant bit; a missing or
stale reply is emitted as `NA`, never replaced with an AS5600 value. Hip yaw has
no dedicated AS5600 and is therefore available only from its motor encoder.

The knee motor field is the upstream actuator-shaft angle at 1:1 scale. The
mechanical linkage or digital twin is responsible for deriving the larger
downstream knee motion from that shaft angle.

---

# Contents

1. [System overview](#1-system-overview)
2. [Why one universal firmware](#2-why-one-universal-firmware)
3. [Boot-time hardware personalities](#3-boot-time-hardware-personalities)
4. [Control routes](#4-control-routes)
5. [DB1 addressed command protocol](#5-db1-addressed-command-protocol)
6. [Target validation and authority](#6-target-validation-and-authority)
7. [First boot and DROPBEAR-SETUP](#7-first-boot-and-dropbear-setup)
8. [Wi-Fi identities](#8-wi-fi-identities)
9. [Common runtime architecture](#9-common-runtime-architecture)
10. [LEFTLEG / RIGHTLEG hardware](#10-leftleg--rightleg-hardware)
11. [AS5600 PWM joint sensing](#11-as5600-pwm-joint-sensing)
12. [Leg actuator map](#12-leg-actuator-map)
13. [Leg Standalone route](#13-leg-standalone-route)
14. [Leg impedance control](#14-leg-impedance-control)
15. [HyperSpawn / ROS2 route](#15-hyperspawn--ros2-route)
16. [HyperSpawn CAN protocol](#16-hyperspawn-can-protocol)
17. [HyperSpawn watchdog and command commit](#17-hyperspawn-watchdog-and-command-commit)
18. [CENTER hardware personality](#18-center-hardware-personality)
19. [TCA9548A / IMU stack](#19-tca9548a--imu-stack)
20. [HEADNECK hardware personality](#20-headneck-hardware-personality)
21. [HEADNECK motion model](#21-headneck-motion-model)
22. [HEADNECK command grammar](#22-headneck-command-grammar)
23. [HEADNECK homing](#23-headneck-homing)
24. [HEADNECK diagnostics and feedback limitations](#24-headneck-diagnostics-and-feedback-limitations)
25. [Captive portal](#25-captive-portal)
26. [Portal API](#26-portal-api)
27. [Diagnostics system](#27-diagnostics-system)
28. [SPIFFS configuration](#28-spiffs-configuration)
29. [Serial command reference](#29-serial-command-reference)
30. [Safety behavior](#30-safety-behavior)
31. [Commissioning](#31-commissioning)
32. [Dependencies](#32-dependencies)
33. [Known limitations](#33-known-limitations)
34. [Repository relationships](#34-repository-relationships)

---

# 1. System overview

The firmware separates **hardware identity**, **control authority**, and **command transport**.

```text
                         INCOMING TRANSPORT
             ┌──────────────┼───────────────┐
             │              │               │
          USB Serial    Captive Portal   Neck Bluetooth
             │              │               │
             └──────────────┼───────────────┘
                            │
                            ▼
                     DB1 ROUTER
                            │
                 target identity check
                            │
                            ▼
                    authority check
                            │
                            ▼
                    role-specific parser
                            │
          ┌─────────────────┼────────────────────┐
          ▼                 ▼                    ▼
     LEFT/RIGHT LEG       CENTER             HEADNECK
          │                 │                    │
          ▼                 ▼                    ▼
    leg controller        IMU stack         stepper stack
```

For leg roles, there is an additional control-route selector:

```text
LEFTLEG / RIGHTLEG
       │
       ├── STANDALONE
       │      └── Portal / Serial owns normal motion
       │
       └── HYPERSPAWN
              └── ROS2 / CAN owns normal motion
```

The underlying actuator/sensor/safety layer remains common.

---

# 2. Why one universal firmware

The goal is to eliminate firmware ambiguity between physical robot subsystems.

A blank ESP32 can be flashed with the same image, expose:

```text
DROPBEAR-SETUP
```

and then be configured as:

```text
LEFTLEG
RIGHTLEG
CENTER
HEADNECK
```

After reboot, it becomes that subsystem.

This has several benefits:

- one firmware artifact to version,
- one configuration format,
- one captive portal framework,
- one diagnostics format,
- one command-addressing protocol,
- one logging model,
- clear hardware-role identity,
- less risk of flashing the wrong source tree onto a physical controller.

This does **not** mean the hardware drivers are shared blindly.

The firmware intentionally uses role-gated initialization because the physical pin graphs overlap.

---

# 3. Boot-time hardware personalities

The internal hardware identity is effectively:

```text
DeviceRole
```

with current values:

```text
UNCONFIGURED
LEFTLEG
RIGHTLEG
CENTER
HEADNECK
```

Reserved DB1 targets also exist for:

```text
LEFTARM
RIGHTARM
```

but arm control is not implemented by this firmware.

An arm-targeted command therefore returns a role-unsupported error rather than falling through into another parser.

## Role switching

Role changes are persistent configuration changes.

Examples:

```text
<DB1:LEFTLEG> role right
<DB1:HEADNECK> role left
<DB1:SETUP> role head
```

A physical-role change should always be followed by reboot.

The firmware marks topology-sensitive changes as:

```text
REBOOT_REQUIRED
```

and disables active runtime state where appropriate.

---

# 4. Control routes

Only leg personalities have a secondary operating-route selection.

Persisted values:

```text
OperatingMode:standalone
```

or:

```text
OperatingMode:hyperspawn
```

Default:

```text
standalone
```

Older configurations that do not contain `OperatingMode:` are treated as Standalone.

## Standalone

Normal motion authority comes from:

```text
USB Serial
Captive portal
```

Features:

- direct torque,
- local impedance,
- calibration,
- manual diagnostics,
- local configuration.

## HyperSpawn

Normal motion authority comes from targeted CAN commands.

Features:

- six-joint position route,
- six-joint torque route,
- atomic two-frame command commit,
- 500 Hz state frames,
- 1 Hz heartbeat,
- command watchdog,
- optional compatibility with upstream legacy brain-origin frames.

Portal and Serial remain available for:

- diagnostics,
- `stop`,
- `zero`,
- configuration,
- route changes,
- SPIFFS access,
- reboot.

Normal local torque/impedance commands are rejected in HyperSpawn mode.

---

# 5. DB1 addressed command protocol

The universal firmware uses an explicit destination envelope:

```text
<DB1:TARGET> payload
```

Examples:

```text
<DB1:LEFTLEG> torque knee 25
<DB1:RIGHTLEG> impedance knee 1 180 0
<DB1:HEADNECK> X10,Y-5,Z15,H30,S1.5,A2,R10,P-5
<DB1:CENTER> status
<DB1:SETUP> role left
```

Recognized targets:

```text
SETUP
LEFTLEG
RIGHTLEG
CENTER
HEADNECK
LEFTARM
RIGHTARM
ALL
```

Aliases recognized by the parser include forms such as:

```text
LEFT_LEG
RIGHT_LEG
HEAD_NECK
HEAD
NECK
```

## Why DB1 exists

The header prevents a command intended for one robot appendage from being executed by another controller.

Example:

```text
RX:
<DB1:LEFTLEG> torque knee 50

actual controller:
RIGHTLEG
```

Result:

```text
ERR|TARGET_MISMATCH|expected=RIGHTLEG|received=LEFTLEG
```

The torque parser is never entered.

## Missing header

Unaddressed commands are rejected by default:

```text
ERR|MISSING_TARGET_HEADER|...
```

SPIFFS contains:

```text
LegacyUnaddressedCommands:0
```

This can be deliberately enabled for migration, but should normally remain off.

## Broadcast

The only permitted broadcast payload is:

```text
<DB1:ALL> stop
```

Motion-producing broadcast commands are rejected.

This means the firmware explicitly forbids patterns such as:

```text
<DB1:ALL> torque knee 100
<DB1:ALL> HOME
<DB1:ALL> play
```

---

# 6. Target validation and authority

A command must pass multiple checks before execution:

```text
raw command
    │
    ▼
DB1 envelope valid?
    │
    ▼
target matches this ESP?
    │
    ▼
target role implemented?
    │
    ▼
source permitted?
    │
    ▼
current operating route permits this command?
    │
    ▼
runtime/safety state permits it?
    │
    ▼
role-specific parser
```

This produces two different classes of protection.

## Target protection

Example:

```text
LEFTLEG command received by RIGHTLEG
```

is rejected as a destination mismatch.

## Authority protection

Example:

```text
<DB1:LEFTLEG> torque knee 40
```

sent over USB while the left leg is in:

```text
OperatingMode:hyperspawn
```

passes the target check but fails motion-authority policy.

The HyperSpawn route owns normal motion.

`stop` remains available as a safety action.

---

# 7. First boot and DROPBEAR-SETUP

If:

```text
/config.txt
```

does not exist or does not contain a valid role, the controller starts unconfigured.

Wi-Fi:

```text
DROPBEAR-SETUP
```

DB1 identity:

```text
SETUP
```

Example first-boot command:

```text
<DB1:SETUP> role left
```

or use the captive configurator.

During unconfigured boot, the firmware intentionally avoids starting the physical motor/IMU topology.

This prevents a blank board from guessing which GPIO graph is connected.

---

# 8. Wi-Fi identities

The SoftAP SSID is derived from the persisted hardware identity.

| Role         | SSID             |
| ------------ | ---------------- |
| Unconfigured | `DROPBEAR-SETUP` |
| Left leg     | `LEFTLEG`        |
| Right leg    | `RIGHTLEG`       |
| Center       | `CENTER`         |
| Head / neck  | `HEADNECK`       |

There is no separate arbitrary SSID setting.

Changing role and rebooting automatically changes the network identity.

Example:

```text
LEFTLEG
  ↓ role right
reboot
  ↓
RIGHTLEG
```

Default captive-portal address:

```text
192.168.4.1
```

The SoftAP is currently open and should be treated as a development/local robot network.

---

# 9. Common runtime architecture

The universal firmware shares the following infrastructure:

```text
SPIFFS
Wi-Fi SoftAP
wildcard DNS
WebServer
DB1 command routing
FreeRTOS synchronization
command logging
configuration persistence
diagnostic snapshots
reboot management
```

Role-specific runtime state is separate.

Conceptually:

```text
setup()
  │
  ├── Serial
  ├── mutexes / queues
  ├── SPIFFS
  ├── load configuration
  ├── determine DeviceRole
  ├── start portal
  │
  └── role switch
       ├── initLegHardware()
       ├── initCenterHardware()
       └── setupNeckHardware()
```

The role-specific GPIO graph is not supposed to leak into another personality.

---

# 10. LEFTLEG / RIGHTLEG hardware

## MCP2515 SPI

| Signal | GPIO |
| ------ | ---: |
| CS     |  `5` |
| INT    | `17` |
| SCK    | `18` |
| MISO   | `19` |
| MOSI   | `23` |

Initialization:

```cpp
SPI.begin(18, 19, 23, 5);
```

CAN configuration:

```text
1,000,000 bit/s
8 MHz MCP2515 crystal
normal mode
11-bit CAN identifiers
```

## External joint sensors

The original physical leg wiring is preserved:

| Joint      | GPIO |
| ---------- | ---: |
| Outer calf | `14` |
| Inner calf | `27` |
| Hip pitch  | `26` |
| Knee       | `25` |
| Hip roll   | `33` |

There is no sixth external hip-yaw AS5600 in the current leg mapping.

---

# 11. AS5600 PWM joint sensing

The universal Wi-Fi build treats each AS5600 OUT signal as a one-wire PWM signal.

The firmware no longer uses `analogRead()` for these five channels.

This is important because on a classic ESP32:

```text
GPIO14
GPIO27
GPIO26
GPIO25
```

belong to ADC2, which conflicts with continuous Wi-Fi operation.

The PWM path avoids that conflict while preserving the existing physical GPIO harness.

## Acquisition

Each pin uses a `CHANGE` interrupt.

The capture tracks:

```text
last rising edge
high duration
period
last edge
frame count
valid state
```

Accepted period window:

```text
800 us .. 12,000 us
```

Signal stale threshold:

```text
100 ms
```

The PWM decoding maps approximately:

```text
2.94% duty -> 0°
97.06% duty -> 360°
```

using:

```text
128 / 4351
4223 / 4351
```

as the useful duty endpoints.

## Compatibility representation

The decoded angle is converted back into a pseudo ADC-style value:

```text
0 .. 4095
```

This allows the existing moving-average, offset, and calibration architecture to remain compatible.

## Sensor processing

Nominal processing period:

```text
1 ms
```

or about:

```text
1000 Hz
```

The firmware retains the ten-sample moving-average layer.

---

# 12. Leg actuator map

The original MyActuator CAN IDs are retained.

| Internal index | Side  | Joint      |  CAN ID |
| -------------: | ----- | ---------- | ------: |
|              0 | Right | Outer calf | `0x144` |
|              1 | Left  | Outer calf | `0x141` |
|              2 | Right | Inner calf | `0x143` |
|              3 | Left  | Inner calf | `0x142` |
|              4 | Right | Knee       | `0x148` |
|              5 | Left  | Knee       | `0x145` |
|              6 | Right | Hip pitch  | `0x147` |
|              7 | Left  | Hip pitch  | `0x146` |
|              8 | Right | Hip yaw    | `0x14C` |
|              9 | Left  | Hip yaw    | `0x149` |
|             10 | Right | Hip roll   | `0x14B` |
|             11 | Left  | Hip roll   | `0x14A` |

The array is intentionally interleaved:

```text
even index = right
odd index  = left
```

A LEFTLEG controller transmits only to:

```text
0x141
0x142
0x145
0x146
0x149
0x14A
```

A RIGHTLEG controller transmits only to:

```text
0x144
0x143
0x148
0x147
0x14C
0x14B
```

---

# 13. Leg Standalone route

Standalone is the default leg-control structure.

Example direct torque:

```text
<DB1:LEFTLEG> torque knee 25
```

Canonical grammar:

```text
<DB1:LEFTLEG|RIGHTLEG> torque <appendage> <torqueValue>
```

Supported appendages:

```text
outer_calf
inner_calf
knee
hip_pitch
hip_yaw
hip_roll
```

The DB1 target determines the leg.

Legacy payloads containing `left` or `right` inside the payload are tolerated only after the outer DB1 destination has already been validated.

## MyActuator output

Normal actuator command:

```text
0xA1
```

Stop command:

```text
0x81
```

The firmware uses one normal periodic CAN writer.

Command parsers update state/setpoints rather than creating independent competing actuator TX loops.

---

# 14. Leg impedance control

Canonical command:

```text
<DB1:LEFTLEG> impedance knee 1 180 0
```

Grammar:

```text
impedance <appendage> <0|1> <desiredPosition> <desiredVelocity>
```

Supported impedance joints:

```text
outer_calf
inner_calf
knee
hip_pitch
hip_roll
```

Hip yaw remains direct-torque only because no external hip-yaw AS5600 is currently mapped.

Conceptual controller:

```text
τ =
    K(q* - q)
  + D(dq* - dq)
  + M(ddq)
```

The resulting value passes through:

```text
direction multiplier
joint constraint behavior
global torque clamp
single CAN-output task
```

before becoming an actuator command.

---

# 15. HyperSpawn / ROS2 route

The HyperSpawn route is the alternative operating structure for LEFTLEG and RIGHTLEG.

Select:

```text
<DB1:LEFTLEG> mode hyperspawn
```

or use the portal.

The change is persisted and treated as reboot-bound.

## Identity

Left leg:

```text
node ID 0x12
limb ID 2
```

Right leg:

```text
node ID 0x13
limb ID 3
```

## Control behavior

### Position command

```text
ROS/CAN position target
        │
        ▼
wire units -> degrees
        │
        ▼
local external-feedback impedance
        │
        ▼
A1 torque
```

### Torque command

```text
ROS/CAN torque
        │
        ▼
HyperSpawn torque bank
        │
        ▼
common safety layer
        │
        ▼
A1 torque
```

This fills the low-level control gap in the higher-level `dropbear_firmware` architecture without abandoning the physical Dropbear actuator/sensor implementation.

---

# 16. HyperSpawn CAN protocol

Message type layout:

```text
CAN ID = (node_id << 4) | message_type
```

Core message types:

```text
0x00 heartbeat
0x01 position base
0x02 torque base
0x03 state base
0x04 position extension
0x05 torque extension
0x06 state extension
0x0F fault
```

## LEFTLEG

|  CAN ID | Meaning              |
| ------: | -------------------- |
| `0x120` | Heartbeat            |
| `0x121` | Position joints 0..3 |
| `0x122` | Torque joints 0..3   |
| `0x123` | State joints 0..3    |
| `0x124` | Position joints 4..5 |
| `0x125` | Torque joints 4..5   |
| `0x126` | State joints 4..5    |
| `0x12F` | Fault                |

## RIGHTLEG

|  CAN ID | Meaning              |
| ------: | -------------------- |
| `0x130` | Heartbeat            |
| `0x131` | Position joints 0..3 |
| `0x132` | Torque joints 0..3   |
| `0x133` | State joints 0..3    |
| `0x134` | Position joints 4..5 |
| `0x135` | Torque joints 4..5   |
| `0x136` | State joints 4..5    |
| `0x13F` | Fault                |

## Six-joint wire order

```text
0 hip_pitch
1 hip_roll
2 hip_yaw
3 knee
4 outer_calf
5 inner_calf
```

Each joint value is a signed 16-bit value encoded big-endian.

## Legacy compatibility

Optional:

```text
HyperspawnLegacyBroadcast:1
```

accepts the earlier brain-origin style such as:

```text
0x011
0x012
```

This is off by default because such frames do not uniquely distinguish left and right limbs on a shared bus.

---

# 17. HyperSpawn watchdog and command commit

## Atomic command commit

A six-joint targeted command requires both frames.

Position:

```text
base 0x121/0x131
extension 0x124/0x134
```

Torque:

```text
base 0x122/0x132
extension 0x125/0x135
```

The firmware stages the fragments and commits all six joint values together.

Fragment timeout:

```text
50 ms
```

A partial command is discarded rather than exposing the control loop to a mixture of old and new trajectory values.

## Command watchdog

Default:

```text
250 ms
```

Persisted:

```text
HyperspawnCommandTimeoutMs:250
```

Valid configured range:

```text
50 .. 10000 ms
```

If the active HyperSpawn control stream goes stale:

```text
control mode -> NONE
pending fragments -> cleared
HyperSpawn torque bank -> zero
stop burst -> queued
fault -> transmitted
```

## Auto-arm

Default:

```text
HyperspawnAutoArm:1
```

With auto-arm enabled, a complete valid command can arm the leg.

Without auto-arm, use:

```text
<DB1:LEFTLEG> play
```

after the route is ready.

## Position scale

Persisted:

```text
HyperspawnPositionUnitsPerDegree:1.0
```

This provides an explicit conversion boundary because the upstream architecture does not establish a final physical unit contract for the raw `int16_t` position field.

---

# 18. CENTER hardware personality

CENTER is a completely different hardware stack.

It does not initialize:

```text
leg MCP2515 control
AS5600 leg sensing
neck FastAccelStepper stack
```

Instead it initializes the shared I²C bus and IMU multiplexer stack.

SSID:

```text
CENTER
```

DB1 identity:

```text
CENTER
```

Example:

```text
<DB1:CENTER> status
```

---

# 19. TCA9548A / IMU stack

I²C:

| Signal | GPIO |
| ------ | ---: |
| SDA    | `21` |
| SCL    | `22` |

Current mux:

```text
TCA9548A @ 0x70
```

Current IMU address:

```text
0x68
```

Channels:

```text
0
1
2
3
4
```

Topology:

```text
ESP32
  │
  ├── SDA21
  └── SCL22
       │
       ▼
    TCA9548A 0x70
       │
       ├── CH0 -> IMU0 @ 0x68
       ├── CH1 -> IMU1 @ 0x68
       ├── CH2 -> IMU2 @ 0x68
       ├── CH3 -> IMU3 @ 0x68
       └── CH4 -> IMU4 @ 0x68
```

The firmware selects a mux channel before reading its IMU.

Current raw read begins at register:

```text
0x3B
```

and consumes 14 bytes for accelerometer, temperature field, and gyroscope layout.

The current firmware reports raw accelerometer and gyroscope state. It does not yet perform full orientation fusion.

---

# 20. HEADNECK hardware personality

HEADNECK is a fourth mutually exclusive physical controller stack.

It uses:

```text
FastAccelStepper
6 × A4988-style STEP/DIR channels
optional shared enable pin
optional Bluetooth Classic
```

SSID:

```text
HEADNECK
```

DB1 target:

```text
HEADNECK
```

Bluetooth name:

```text
NECK_BT
```

when enabled.

## Pinout

| Motor | STEP |  DIR |
| ----- | ---: | ---: |
| M1    | `33` | `32` |
| M2    | `18` | `26` |
| M3    | `23` | `14` |
| M4    | `19` | `27` |
| M5    | `22` | `12` |
| M6    | `21` | `13` |

Optional shared enable:

```text
GPIO25
```

## Why HEADNECK must be boot-exclusive

These pins overlap heavily with leg and center roles.

Examples:

```text
GPIO18
leg    -> MCP2515 SCK
neck   -> M2 STEP

GPIO23
leg    -> MCP2515 MOSI
neck   -> M3 STEP

GPIO21
center -> I²C SDA
neck   -> M6 STEP

GPIO22
center -> I²C SCL
neck   -> M5 STEP

GPIO14
leg    -> outer-calf AS5600
neck   -> M3 DIR
```

Therefore HEADNECK must never initialize leg or center peripherals during the same boot.

---

# 21. HEADNECK motion model

Defaults:

```text
speed              48,000 steps/s
acceleration       36,000 steps/s²
steps/mm           426.67
per-motor min      0 mm
per-motor max      80 mm
```

Persisted fields allow these values to be adjusted.

The controller tracks:

```text
current FastAccelStepper library position
target step position
target mm
running/not running
software homed state
homing state
motion armed state
```

## Direct actuator movement

A direct command specifies actuator extension in millimeters.

Example:

```text
<DB1:HEADNECK> 1:30,2:45,3:20
```

Conversion:

```text
steps = mm × NeckStepsPerMm
```

Then per-actuator software min/max limits are applied.

## Pose movement

Example:

```text
<DB1:HEADNECK> X10,Y-5,Z15,H30,S1.5,A2,R10,P-5
```

Fields:

```text
X
Y
Z
H  height
S  speed multiplier
A  acceleration multiplier
R  roll contribution
P  pitch contribution
```

The current Stewart-style mapping produces six step targets from the pose fields.

The firmware retains the original pose-level clamp equivalent to:

```text
0 .. 80 mm
```

using a pose conversion scale of:

```text
400 steps/mm
```

and then applies the separately configurable per-actuator bounds as a second limit layer.

## Quaternion command

Grammar:

```text
Q:w,x,y,z[,Hn][,Sn][,An]
```

Example:

```text
<DB1:HEADNECK> Q:1,0,0,0,H20,S1,A1
```

The quaternion is normalized and converted to Euler yaw/pitch/roll before being passed into the neck pose controller.

A zero-norm quaternion is rejected.

---

# 22. HEADNECK command grammar

All commands should carry:

```text
<DB1:HEADNECK>
```

unless legacy unaddressed commands have deliberately been enabled.

## Direct motor target

```text
<DB1:HEADNECK> 1:30
<DB1:HEADNECK> 1:30,2:45,3:20
```

Values are millimeters.

## Pose

```text
<DB1:HEADNECK> X10,Y-5,Z15,H30,S1.5,A2,R10,P-5
```

## Quaternion

```text
<DB1:HEADNECK> Q:1,0,0,0,H20,S1,A1
```

## Compound commands

The neck parser supports `|` chained commands.

Example:

```text
<DB1:HEADNECK> 1:20,2:20|3:35,4:35
```

Segments are processed sequentially by the neck parser.

## Health

```text
<DB1:HEADNECK> HEALTH
```

or uppercase compatibility:

```text
<DB1:HEADNECK> STATUS
```

returns the neck health/state string.

The universal lowercase:

```text
<DB1:HEADNECK> status
```

is handled by the common universal status path.

## Motion controls

```text
<DB1:HEADNECK> play
<DB1:HEADNECK> stop
<DB1:HEADNECK> zero
```

Neck-specific equivalents:

```text
<DB1:HEADNECK> neck stop
<DB1:HEADNECK> neck zero
<DB1:HEADNECK> neck status
```

## Speed

```text
<DB1:HEADNECK> neck speed 48000
```

Valid firmware range:

```text
1 .. 200000 Hz
```

## Acceleration

```text
<DB1:HEADNECK> neck accel 36000
```

Valid firmware range:

```text
1 .. 2000000 steps/s²
```

## Bluetooth

```text
<DB1:HEADNECK> neck bluetooth on
<DB1:HEADNECK> neck bluetooth off
```

A Bluetooth topology change requires reboot.

## Boot auto-home

```text
<DB1:HEADNECK> neck autohome on
<DB1:HEADNECK> neck autohome off
```

Unified-firmware default:

```text
OFF
```

This is intentionally safer than automatically driving the mechanism on every boot.

## Per-motor limits

```text
<DB1:HEADNECK> neck limits 1 0 80
```

Grammar:

```text
neck limits <motor 1..6> <min_mm> <max_mm>
```

---

# 23. HEADNECK homing

The firmware preserves two open-loop homing styles.

Commands:

```text
<DB1:HEADNECK> HOME
<DB1:HEADNECK> HOME_BRUTE
<DB1:HEADNECK> HOME_SOFT
```

`HOME` is an alias for brute home.

## Soft home

Current sequence:

```text
height target          -40
speed multiplier       2.0
accel multiplier       2.0
settle                 2200 ms
```

After the settle period:

```text
stepper motion stopped
current library positions -> 0
software homed -> true
```

## Brute home

Preparation phase:

```text
height target          -55
speed multiplier       2.5
accel multiplier       2.5
settle                 2300 ms
```

Then:

```text
150 ms transition gap
```

Final overtravel phase:

```text
height target          -80
speed multiplier       3.0
accel multiplier       3.0
settle                 2600 ms
```

Then software zero is established.

## Critical homing limitation

Current neck homing is:

```text
OPEN LOOP OVERTRAVEL + SOFTWARE ZERO
```

It does **not** verify physical contact using:

- a limit switch,
- an encoder,
- current sensing,
- force sensing,
- a homing sensor.

The homing routine intentionally bypasses ordinary software travel clamps.

Treat it as a mechanical procedure, not as a measured reference acquisition.

---

# 24. HEADNECK diagnostics and feedback limitations

The neck controller currently has no physical actuator-position feedback.

The firmware knows:

```text
what step position FastAccelStepper believes it is at
what target it was commanded to
whether the library reports motion
```

It does not know with certainty:

```text
whether a step was missed
whether the leadscrew stalled
whether a coupling slipped
whether the platform physically reached the target
```

Therefore the portal explicitly treats neck state as:

```text
FEEDBACK=OPEN_LOOP
```

A typical motor diagnostic should be interpreted as:

```text
M3
STEP GPIO          23
DIR GPIO           14
current library    14,200 steps
current derived    33.28 mm
target             14,500 steps
target derived     33.98 mm
running            yes
feedback           OPEN_LOOP
```

Do not interpret `current library position` as independently measured physical extension.

---

# 25. Captive portal

The portal provides a role-aware local interface.

It uses:

- ESP32 SoftAP,
- wildcard DNS,
- captive-check redirects,
- `WebServer`,
- SPIFFS-backed configuration,
- client-side role-specific presentation.

Common portal functions include:

```text
live system state
configuration
diagnostics
terminal / command log
SPIFFS browser
raw config editor
reboot
```

Role-specific panels are shown for:

```text
leg control / AS5600 / CAN
HyperSpawn route
center IMUs
head/neck pose and motor state
```

## Portal command addressing

The browser automatically creates the DB1 envelope.

For example, pressing a knee torque control while connected to LEFTLEG internally produces:

```text
<DB1:LEFTLEG> torque knee 40
```

The target identity is still validated by the backend.

The portal does not bypass DB1.

---

# 26. Portal API

Current endpoints:

```text
GET  /
GET  /api/state
GET  /api/diagnostics

GET  /api/config
POST /api/config
POST /api/config/reload
POST /api/config/raw

POST /api/command
GET  /api/log

GET  /api/spiffs/list
GET  /api/spiffs/read

POST /api/calibrate/sensors
POST /api/reboot
```

Captive-check routes include common Android, Apple, and Windows endpoints.

State-changing API requests are target-aware.

## Command endpoint

```text
POST /api/command
```

parameter:

```text
cmd=<DB1:TARGET> ...
```

The HTTP handler validates the DB1 destination before queueing.

The command is validated **again** when the command queue consumes it.

This is intentional defense in depth.

---

# 27. Diagnostics system

The portal diagnostics wrapper exposes common and role-specific health.

## Common modules

```text
ESP32
heap
uptime
Wi-Fi
SoftAP clients
portal HTTP activity
SPIFFS
command queue
DB1 routing
runtime topology
reboot-required state
```

## Leg modules

```text
SPI
MCP2515
CAN TX
CAN RX
AS5600 channels
joint state
impedance
actuator output
stop state
HyperSpawn protocol
```

## Center modules

```text
I²C
TCA9548A
IMU channels 0..4
read/probe state
raw accel/gyro
```

## Neck modules

```text
FastAccelStepper runtime
six STEP/DIR channels
motion armed state
software homed state
homing state
target/current step state
software limits
Bluetooth
command counts
open-loop feedback warning
```

## DB1 routing diagnostics

The firmware tracks fields such as:

```text
accepted commands
rejected commands
target mismatches
unsupported targets
broadcast STOP count
last routed target
last routed source
last routed payload
last routed command age
```

---

# 28. SPIFFS configuration

Primary configuration:

```text
/config.txt
```

Current version:

```text
ConfigVersion:6
```

Command protocol marker:

```text
CommandProtocol:DB1
```

Default routing policy:

```text
LegacyUnaddressedCommands:0
```

## Representative configuration

```text
LegSide:left

LeftOffsets:32,-26,-4,-17,2
RightOffsets:-28,39,-2,18,-2

DirectionMultipliers:1,1,1,1,1,1,1,1,1,1

outer_calf_left_Constraints Constraints:0,360
outer_calf_right_Constraints Constraints:0,360
inner_calf_left_Constraints Constraints:0,360
inner_calf_right_Constraints Constraints:0,360
knee_left_Constraints Constraints:0,360
knee_right_Constraints Constraints:0,360
hip_pitch_left_Constraints Constraints:0,360
hip_pitch_right_Constraints Constraints:0,360
hip_yaw_left_Constraints Constraints:0,360
hip_yaw_right_Constraints Constraints:0,360
hip_roll_left_Constraints Constraints:0,360
hip_roll_right_Constraints Constraints:0,360

ConfigVersion:6
MaxTorqueLimit:3.000
OperatingMode:standalone
CommandProtocol:DB1
LegacyUnaddressedCommands:0

HyperspawnCommandTimeoutMs:250
HyperspawnLegacyBroadcast:0
HyperspawnAutoArm:1
HyperspawnPositionUnitsPerDegree:1.000000

DeviceRole:left

NeckSpeedHz:48000
NeckAcceleration:36000
NeckStepsPerMm:426.670000
NeckBluetoothEnabled:1
NeckAutoHome:0
NeckUseEnablePin:1
NeckMinMm:0,0,0,0,0,0
NeckMaxMm:80,80,80,80,80,80
```

`LegSide:` is retained near the top for rollback/legacy compatibility.

`DeviceRole:` is the newer explicit hardware-identity field.

## Raw editing

The captive portal can expose `/config.txt` directly.

Raw configuration edits should be treated as topology-affecting operations.

The firmware stops/clears active control state and requires reboot after raw replacement.

---

# 29. Serial command reference

Serial speed:

```text
115200
```

DB1 is mandatory by default.

## Universal

```text
<DB1:TARGET> help
<DB1:TARGET> status
<DB1:TARGET> play
<DB1:TARGET> stop
<DB1:TARGET> zero
<DB1:TARGET> save
<DB1:TARGET> mac
<DB1:TARGET> config
```

Global safety:

```text
<DB1:ALL> stop
```

## Role

```text
<DB1:TARGET> role
<DB1:TARGET> role left
<DB1:TARGET> role right
<DB1:TARGET> role center
<DB1:TARGET> role head
```

First boot:

```text
<DB1:SETUP> role left
```

Compatibility aliases also include:

```text
left
right
center
head
neck
chirality
```

inside a valid DB1 envelope.

## Leg route

```text
<DB1:LEFTLEG> mode
<DB1:LEFTLEG> mode standalone
<DB1:LEFTLEG> mode hyperspawn
```

Aliases:

```text
ros2
hyperspawn on
hyperspawn off
```

## HyperSpawn configuration

```text
hyperspawn status
hyperspawn legacy on
hyperspawn legacy off
hyperspawn autoarm on
hyperspawn autoarm off
hyperspawn timeout <ms>
hyperspawn scale <wire_units_per_degree>
```

Example:

```text
<DB1:RIGHTLEG> hyperspawn timeout 250
```

## Leg direct torque

```text
<DB1:LEFTLEG> torque knee 25
```

## Leg impedance

```text
<DB1:LEFTLEG> impedance knee 1 180 0
```

## Leg sensor/calibration

```text
raw on
raw off
calibrate
calibrate save
resetOffsets
saved
calibrateDirection <joint>
direction <joint> <+|->
constrain <joint> <min> <max>
setJointConstraints
resetSPIFFS
```

## Neck

```text
<DB1:HEADNECK> 1:30,2:45
<DB1:HEADNECK> X10,Y-5,Z15,H30,S1.5,A2,R10,P-5
<DB1:HEADNECK> Q:1,0,0,0,H20,S1,A1

<DB1:HEADNECK> HEALTH
<DB1:HEADNECK> STATUS

<DB1:HEADNECK> HOME
<DB1:HEADNECK> HOME_BRUTE
<DB1:HEADNECK> HOME_SOFT

<DB1:HEADNECK> neck status
<DB1:HEADNECK> neck stop
<DB1:HEADNECK> neck zero
<DB1:HEADNECK> neck speed 48000
<DB1:HEADNECK> neck accel 36000
<DB1:HEADNECK> neck bluetooth on
<DB1:HEADNECK> neck bluetooth off
<DB1:HEADNECK> neck autohome on
<DB1:HEADNECK> neck autohome off
<DB1:HEADNECK> neck limits 1 0 80
```

---

# 30. Safety behavior

## Wrong destination

Rejected before role-specific parsing.

Example:

```text
ERR|TARGET_MISMATCH|expected=HEADNECK|received=LEFTLEG
```

## Unsupported future arm role

Rejected explicitly:

```text
ERR|ROLE_UNSUPPORTED|target=LEFTARM|...
```

## Broadcast restriction

Only:

```text
<DB1:ALL> stop
```

is accepted.

## Leg stop

`stop` disables play and queues real MyActuator stop frames:

```text
0x81
```

to the six actuator IDs belonging to the selected leg.

## HyperSpawn stop

Stopping also clears/disarms HyperSpawn motion state so a subsequent re-arm cannot replay an old trajectory.

## Neck stop

Stops all six FastAccelStepper channels and records current library positions as the new targets.

It does not prove that the physical mechanism is at those positions.

## Neck zero

Stops motion and sets current library positions to zero.

This is a **software zero**, not a physical sensor measurement.

## Topology changes

Changing:

```text
LEFTLEG <-> RIGHTLEG
LEG <-> CENTER
LEG/CENTER <-> HEADNECK
```

requires reboot.

The firmware must not repurpose overlapping GPIOs live while the mechanism is energized.

---

# 31. Commissioning

## Universal first boot

1. Flash `dropbear_unified_behemoth.ino`.

2. Open Serial at `115200`.

3. Connect to:

   ```text
   DROPBEAR-SETUP
   ```

4. Select physical role.

5. Save.

6. Reboot.

7. Confirm expected SSID and DB1 identity.

---

## LEFTLEG / RIGHTLEG commissioning

Before applying meaningful torque:

```text
[ ] controller role matches physical leg
[ ] SSID matches physical leg
[ ] DB1 address matches physical leg
[ ] MCP2515 CS/SPI/INT wiring verified
[ ] CAN termination verified
[ ] MCP2515 crystal is 8 MHz
[ ] CAN bus is 1 Mbps
[ ] AS5600 OUT signals are actually PWM
[ ] five encoder GPIOs produce valid diagnostic pulse streams
[ ] offsets calibrated
[ ] direction multipliers verified
[ ] constraints configured
[ ] maximum torque kept conservative
[ ] physical power disconnect available
```

Test:

```text
<DB1:LEFTLEG> stop
<DB1:LEFTLEG> status
```

Then very small manual torque in Standalone:

```text
<DB1:LEFTLEG> torque knee 5
<DB1:LEFTLEG> play
```

Stop immediately after verifying direction:

```text
<DB1:LEFTLEG> stop
<DB1:LEFTLEG> zero
```

---

## HyperSpawn commissioning

Start mechanically supported.

Verify:

```text
OperatingMode:hyperspawn
correct node ID
legacy broadcast OFF
watchdog = 250 ms
auto-arm behavior understood
```

First targeted command should contain all-zero/low-energy values.

Confirm:

```text
targeted RX increments
completed command increments
state TX increments
heartbeat TX increments
watchdog age resets
```

Then intentionally stop sending commands and verify the watchdog stops the leg.

Do this before testing higher-energy trajectories.

---

## CENTER commissioning

Verify:

```text
SSID = CENTER
DB1 = CENTER
TCA9548A detected at 0x70
five expected mux channels visible
IMU reads present at 0x68
```

Confirm channels individually rather than assuming unique IMU addresses.

---

## HEADNECK commissioning

Before motion:

```text
[ ] role = HEADNECK
[ ] SSID = HEADNECK
[ ] DB1 address = HEADNECK
[ ] six STEP pins match hardware
[ ] six DIR pins match hardware
[ ] GPIO25 enable behavior matches driver wiring
[ ] configured steps/mm is correct
[ ] per-motor min/max are conservative
[ ] auto-home is OFF initially
[ ] platform is mechanically supported
[ ] physical power disconnect available
```

Recommended first sequence:

```text
<DB1:HEADNECK> stop
<DB1:HEADNECK> neck status
<DB1:HEADNECK> play
<DB1:HEADNECK> 1:1
<DB1:HEADNECK> stop
```

Verify one motor/direction at a time before issuing combined Stewart pose commands.

Do not use brute homing until the mechanical overtravel behavior has been deliberately verified.

---

# 32. Dependencies

The sketch includes:

```cpp
#include <Arduino.h>
#include <mcp_can.h>
#include <SPI.h>
#include <SPIFFS.h>
#include <Wire.h>
#include <WiFi.h>
#include <WebServer.h>
#include <DNSServer.h>
#include <FastAccelStepper.h>
#include <BluetoothSerial.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
```

Required environment includes:

- classic ESP32 Arduino core,
- MCP2515 library exposing `MCP_CAN`,
- FastAccelStepper,
- ESP32 BluetoothSerial support,
- SPIFFS-enabled partition layout.

The firmware is intentionally built around the classic ESP32 pin graph currently used by Dropbear.

---

# 33. Known limitations

## AS5600 output-mode assumption

The integrated always-on Wi-Fi leg firmware expects the installed AS5600 OUT pins to be configured for PWM.

If the physical sensors are actually configured for analog voltage output, this acquisition model must be changed.

Do not assume `analog OUT` and `PWM OUT` are interchangeable merely because they share the same AS5600 pin.

## Hip yaw external feedback

The current leg mapping has five external encoders and does not include a dedicated hip-yaw AS5600.

Therefore:

- local hip-yaw impedance is unavailable,
- HyperSpawn hip-yaw position telemetry is not a true independent external measurement,
- torque mode remains available.

## RMD CAN feedback

The firmware has a CAN receive path but does not yet fully decode all MyActuator response data into:

```text
temperature
measured current
motor speed
driver faults
motor-side position
```

Successful CAN TX should not be interpreted as proof of healthy actuator execution.

## Neck feedback

HEADNECK is open-loop.

FastAccelStepper state is not the same as physical-mechanism feedback.

## Neck homing

Current homing uses deliberate open-loop overtravel.

No switch/encoder/force signal terminates the move.

## Portal security

The SoftAP is currently open.

Any client that can join the local network can potentially reach control endpoints.

DB1 prevents wrong-device routing; it is **not authentication**.

## HyperSpawn units

Position scaling is configurable because the raw upstream integer unit is not yet a final standardized physical contract.

## Center IMU processing

The center stack exposes raw sensor reads but does not yet implement full inertial fusion.

---

# 34. Repository relationships

This firmware consolidates functionality and architecture derived from several Dropbear-related repositories.

## Original Dropbear low-level leg controller

```text
https://github.com/Hyperspawn/Dropbear/tree/main/Control%20System/Low%20Level%20Control
```

Provides the historical leg hardware mapping, MyActuator IDs, calibration model, and external joint-control architecture.

## HyperSpawn ROS2/CAN firmware

```text
https://github.com/Hyperspawn/dropbear_firmware
```

Provides the higher-level distributed CAN/ROS2 node architecture used by the optional HyperSpawn leg-control route.

## Captive-portal plumbing

```text
https://github.com/robit-man/esp-captive-chat
```

Provides the local SoftAP/captive-portal design pattern adapted into the universal Dropbear configuration and diagnostics interface.

## Neck assembly/controller

```text
https://github.com/robit-man/Dropbear-Neck-Assembly
```

Provides the six-motor neck pinout, FastAccelStepper control structure, neck pose/direct/quaternion grammar, and homing behavior incorporated into the HEADNECK personality.

## MyActuator reference

```text
https://github.com/Hyperspawn/myactuator-can
```

Provides additional MyActuator CAN reference material. The Dropbear leg controller intentionally continues to use external-feedback torque control rather than replacing its control architecture with a simple internal absolute-position command path.

---

# Quick reference

## Addressing

```text
<DB1:LEFTLEG>
<DB1:RIGHTLEG>
<DB1:CENTER>
<DB1:HEADNECK>
<DB1:SETUP>
<DB1:ALL> stop
```

## SSIDs

```text
DROPBEAR-SETUP
LEFTLEG
RIGHTLEG
CENTER
HEADNECK
```

## Leg CAN

```text
MCP2515 CS    GPIO5
INT           GPIO17
SCK           GPIO18
MISO          GPIO19
MOSI          GPIO23
1 Mbps
8 MHz MCP2515
```

## Leg AS5600

```text
outer calf    GPIO14
inner calf    GPIO27
hip pitch     GPIO26
knee          GPIO25
hip roll      GPIO33
interface     PWM OUT
```

## CENTER

```text
SDA           GPIO21
SCL           GPIO22
TCA9548A      0x70
IMU           0x68
channels      0..4
```

## HEADNECK

```text
M1 STEP 33  DIR 32
M2 STEP 18  DIR 26
M3 STEP 23  DIR 14
M4 STEP 19  DIR 27
M5 STEP 22  DIR 12
M6 STEP 21  DIR 13

ENABLE       GPIO25

default speed          48000 steps/s
default acceleration   36000 steps/s²
default steps/mm       426.67
default travel          0..80 mm
Bluetooth              NECK_BT
```

## Config

```text
ConfigVersion:6
CommandProtocol:DB1
LegacyUnaddressedCommands:0
OperatingMode:standalone
```

---

# License

Use the license of the parent Dropbear project/repository into which this firmware is incorporated.
