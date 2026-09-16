# Dropbear ESP32 Low-Level Control

![Dropbear control hardware](https://github.com/user-attachments/assets/1a4015b7-2bcd-48eb-9b44-fc3f3899f1f7)

Low-level ESP32 firmware for the Dropbear bipedal robot.

For connected-robot visualization, receive-only USB telemetry, raw controller
serial, and guarded compile/upload, use the
[`dropbear_control` dashboard](https://github.com/robit-man/dropbear_control).
The firmware choices in this directory are:

| Source | Intended use |
|---|---|
| `firmware_full_libs_neck.ino` | Recommended universal Behemoth build for left leg, right leg, center, and head/neck roles. |
| `esp32_devkitc_v4_hybrid.ino` | Existing hybrid leg PWM/CAN deployments. |
| `esp32_devkit_v1_observation_safe.ino` | Fail-closed observation migration with no motion command path. |
| `esp32_devkit_v1.ino` | Legacy/development leg implementation retained for compatibility. |

All dashboard builds copy this directory's `partitions.csv`. It provides a
2.5 MiB application region while preserving the standard deployed SPIFFS
settings region at `0x290000` with size `0x160000`. The dashboard uses
`EraseFlash=none`, reads the target partition table before upload, and holds
the upload if the installed SPIFFS offset or size differs.

This controller is designed around **one ESP32 per leg**, an **MCP2515 CAN controller**, MyActuator/RMD-class CAN actuators, and five external analog joint-angle sensors per leg. The firmware performs high-rate joint sensing, optional joint-space impedance control, deterministic CAN torque output, persistent calibration/configuration, and serial command handling.

The current firmware is intentionally a **torque-control architecture**, not an actuator-internal absolute-position architecture. It sends MyActuator `0xA1` torque/current commands during normal operation and `0x81` stop commands when stopping a leg.

The firmware file documented here is:

```text
Control System/
└── Low Level Control/
    └── esp32_devkit_v1.ino
```

---

## Contents

- [System architecture](#system-architecture)
- [Controller roles](#controller-roles)
- [Hardware](#hardware)
- [Pinout](#pinout)
- [CAN configuration](#can-configuration)
- [Actuator IDs](#actuator-ids)
- [External joint sensors](#external-joint-sensors)
- [Boot sequence](#boot-sequence)
- [FreeRTOS runtime architecture](#freertos-runtime-architecture)
- [CAN command arbitration](#can-command-arbitration)
- [Direct torque control](#direct-torque-control)
- [Impedance control](#impedance-control)
- [Joint constraints](#joint-constraints)
- [Sensor calibration](#sensor-calibration)
- [Torque-direction calibration](#torque-direction-calibration)
- [Stop behavior](#stop-behavior)
- [Center / IMU mode](#center--imu-mode)
- [Persistent configuration](#persistent-configuration)
- [Serial interface](#serial-interface)
- [Command reference](#command-reference)
- [Recommended commissioning procedure](#recommended-commissioning-procedure)
- [Safety and current limitations](#safety-and-current-limitations)
- [Dependencies](#dependencies)
- [Relationship to `myactuator-can`](#relationship-to-myactuator-can)
- [Legacy behavior removed or changed](#legacy-behavior-removed-or-changed)

---

# System architecture

The controller is split into three primary real-time paths:

```text
                         HIGH-LEVEL / SERIAL COMMAND
                                   │
                  ┌────────────────┴────────────────┐
                  │                                 │
          direct torque setpoint            impedance target
                  │                                 │
                  │                          q*, dq*
                  │                                 │
                  │                    external joint sensor
                  │                                 │
                  │                          state estimate
                  │                                 │
                  │                     impedance controller
                  │                                 │
                  └──────────────┬──────────────────┘
                                 │
                          command selection
                                 │
                         torque saturation
                                 │
                         single CAN writer
                                 │
                    MyActuator 0xA1 command
                                 │
                              actuator
```

The key architectural rule is:

> **Only the CAN output task performs normal actuator transmission.**

The impedance task computes desired torque but does not transmit CAN frames. Direct torque commands also only update setpoints. The CAN output task is the single periodic owner of normal motor output.

This prevents the previous architecture from having multiple FreeRTOS tasks independently transmit competing torque values to the same actuator.

---

# Controller roles

Each ESP32 stores one of three roles in SPIFFS:

```text
left
right
center
```

## `left`

The controller:

- initializes MCP2515 CAN,
- reads the five local analog joint sensors,
- runs left-leg impedance controllers,
- sends commands only to the six left-leg actuator CAN IDs,
- accepts left-leg torque and impedance commands.

## `right`

The controller:

- initializes MCP2515 CAN,
- reads the five local analog joint sensors,
- runs right-leg impedance controllers,
- sends commands only to the six right-leg actuator CAN IDs,
- accepts right-leg torque and impedance commands.

## `center`

The controller:

- does not initialize the MCP2515 CAN control path,
- does not start leg sensor/control/CAN-output tasks,
- starts the IMU polling task,
- polls I²C addresses `0x68` through `0x6C`,
- prints raw accelerometer and gyroscope data over Serial.

Changing between a leg role and `center` is persisted immediately, but **a reboot is required for the correct task/hardware set to be created**.

Changing directly between `left` and `right` uses the same leg task set and can take effect without changing the physical task topology, although the robot should still be stopped before changing roles.

---

# Hardware

The current low-level leg controller expects:

- ESP32 DevKit V1 / classic ESP32
- MCP2515 CAN controller
- MCP2515 module fitted with an **8 MHz oscillator**
- CAN transceiver associated with the MCP2515 module
- MyActuator/RMD-compatible CAN actuators
- five analog joint-angle signals per leg
- optional I²C IMUs for center mode
- USB serial connection for configuration/debugging

The code uses a 12-bit ADC configuration:

```cpp
analogReadResolution(12);
```

which produces nominal readings from `0` through `4095`.

---

# Pinout

## MCP2515 / SPI

| Function    | ESP32 GPIO | Notes                                                        |
| ----------- | ---------: | ------------------------------------------------------------ |
| MCP2515 CS  |      **5** | Active chip-select used by `MCP_CAN`                         |
| SPI SCK     |     **18** | Explicitly initialized                                       |
| SPI MISO    |     **19** | Explicitly initialized                                       |
| SPI MOSI    |     **23** | Explicitly initialized                                       |
| MCP2515 INT |     **17** | Configured as `INPUT_PULLUP`; current firmware does not use interrupt-driven CAN RX |

SPI initialization:

```cpp
SPI.begin(18, 19, 23, 5);
```

The current controller is transmit-oriented. The MCP2515 interrupt pin is retained in the hardware definition for future receive/interrupt handling, but no ISR is currently attached.

## I²C

| Function | ESP32 GPIO |
| -------- | ---------: |
| SDA      |     **21** |
| SCL      |     **22** |

Initialization:

```cpp
Wire.begin(21, 22);
```

## Analog joint sensors

| Joint measurement | ESP32 GPIO |
| ----------------- | ---------: |
| Outer calf        |     **14** |
| Inner calf        |     **27** |
| Hip pitch         |     **26** |
| Knee              |     **25** |
| Hip roll / butt   |     **33** |

There is currently **no dedicated external analog hip-yaw input**.

---

# CAN configuration

The MCP2515 is configured as:

```text
Bit rate:       1,000,000 bit/s
MCP2515 clock:  8 MHz
Mode:           normal
CAN frame type: standard 11-bit identifier
```

Initialization is equivalent to:

```cpp
CAN.begin(MCP_ANY, CAN_1000KBPS, MCP_8MHZ);
CAN.setMode(MCP_NORMAL);
```

If MCP2515 initialization fails, firmware prints an error and intentionally stops in an infinite delay loop rather than entering operation without CAN.

---

# Actuator IDs

The twelve robot actuator IDs are preserved.

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

The array is deliberately interleaved:

```text
even index -> right leg
odd index  -> left leg
```

This allows the CAN output task to select a leg with a stride of two.

### Left controller transmits only

```text
0x141
0x142
0x145
0x146
0x149
0x14A
```

### Right controller transmits only

```text
0x144
0x143
0x148
0x147
0x14C
0x14B
```

A left ESP therefore no longer sends zero or control traffic to right-leg motors, and a right ESP no longer sends traffic to left-leg motors.

---

# External joint sensors

Five local joint signals are sampled by each leg controller.

The processing chain is:

```text
ADC
 │
 ├─ 12-bit analogRead()
 │
 ├─ 10-sample moving average
 │
 ├─ convert ADC count to 0..360 degrees
 │
 ├─ apply selected-leg offset unless raw mode is enabled
 │
 └─ wrap result into [0, 360)
```

ADC-to-angle conversion is:

```text
angle = ADC × 360 / 4096
```

## Sensor acquisition rate

The sensor task runs every:

```text
1 ms
```

or approximately:

```text
1000 Hz
```

The moving-average window is ten samples, so the filter spans approximately 10 ms at the nominal task rate.

## Serial telemetry rate

When `playMode` is active in leg mode, normalized angles are printed approximately every:

```text
20 ms
```

or:

```text
50 Hz
```

The telemetry format is CSV:

```text
outer,inner,hipPitch,knee,hipRoll
```

Example:

```text
181.2,179.7,176.4,183.1,180.0
```

Command/status text shares the same Serial connection, so consumers of the stream must be able to distinguish CSV telemetry lines from human-readable command responses.

---

# Boot sequence

On reset, the firmware performs the following sequence.

## 1. Serial

Starts:

```text
115200 baud
```

## 2. Synchronization primitives

Creates:

- serial mutex,
- CAN mutex,
- state mutex.

## 3. ADC and I²C

Configures 12-bit ADC resolution and starts I²C on GPIO 21/22.

## 4. SPIFFS

Mounts SPIFFS using:

```cpp
SPIFFS.begin(true)
```

The `true` argument allows formatting if mounting fails.

If SPIFFS still cannot be mounted, the controller halts.

## 5. Configuration

The firmware attempts to load:

```text
/config.txt
```

If no configuration file exists, the controller prompts:

```text
Enter 'left' for left leg, 'right' for right leg, or 'center' for IMU center.
```

The selected role is then saved.

## 6. Role-specific initialization

### Leg role

The controller:

1. configures MCP2515 INT GPIO 17 as input pull-up,
2. initializes SPI,
3. starts MCP2515 at 1 Mbit/s / 8 MHz,
4. switches MCP2515 to normal mode,
5. primes the ten-sample joint sensor filters,
6. starts sensor, impedance, CAN-output, and serial tasks.

### Center role

The controller starts:

- IMU task,
- serial-command task.

CAN is not initialized in center mode.

## 7. Play state

At the end of setup:

```cpp
playMode = true;
```

Therefore:

- a leg controller begins its normal 100 Hz CAN output loop immediately,
- all initial torque setpoints are zero, so initial normal output is zero `0xA1` torque,
- a center controller begins IMU polling immediately.

This startup behavior is intentional and preserves the previous controller's automatic active state while ensuring the initial commanded torque values are zero.

---

# FreeRTOS runtime architecture

## Leg controller tasks

| Task                   | Core | Priority |      Period | Responsibility                                             |
| ---------------------- | ---: | -------: | ----------: | ---------------------------------------------------------- |
| `readAndComputeTask`   |    0 |        2 |        1 ms | ADC acquisition, filtering, normalization, 50 Hz telemetry |
| `impedanceControlTask` |    1 |        2 |       10 ms | Compute impedance torque setpoints                         |
| `canOutputTask`        |    1 |        3 |       10 ms | Sole periodic actuator CAN transmitter                     |
| `checkChiralityTask`   |    1 |        1 | ~10 ms poll | Serial command parser                                      |

## Center controller tasks

| Task                 | Core | Priority |      Period | Responsibility        |
| -------------------- | ---: | -------: | ----------: | --------------------- |
| `imuReadTask`        |    0 |        1 |       10 ms | Poll raw IMU data     |
| `checkChiralityTask` |    1 |        1 | ~10 ms poll | Serial command parser |

The Arduino `loop()` does not implement control logic. It simply sleeps while FreeRTOS tasks own the runtime.

---

# CAN command arbitration

The CAN output task has an explicit command priority.

```text
1. calibration override
        ↓
2. play-mode command output
        ↓
3. queued stop burst
        ↓
4. no transmission
```

## 1. Calibration override

When torque-direction calibration is active:

- only the selected test joint receives the requested test torque,
- every other motor on the selected leg receives zero torque,
- normal direct/impedance setpoints are ignored until calibration ends.

## 2. Normal play output

For each of the six actuators belonging to this controller:

```text
if impedance is enabled for that joint:
    transmit impedanceTorqueValues[joint]
else:
    transmit torqueValues[joint]
```

The selected torque is clamped before transmission.

## 3. Stop burst

When stopped, three `0x81` stop frames are sent to every motor on the selected leg, one burst per CAN task period.

At a 10 ms CAN task rate, this normally means:

```text
t = 0 ms   stop each of six motors
t = 10 ms  stop each of six motors
t = 20 ms  stop each of six motors
```

After the third burst, the controller remains silent until `play` is enabled again.

---

# Direct torque control

Normal direct motor commands use MyActuator command byte:

```text
0xA1
```

The frame generated by the firmware is:

```text
Byte 0: 0xA1
Byte 1: 0x00
Byte 2: 0x00
Byte 3: 0x00
Byte 4: torque low byte
Byte 5: torque high byte
Byte 6: 0x00
Byte 7: 0x00
```

The torque value is transmitted as a signed 16-bit value.

## Torque limit

The firmware currently defines:

```cpp
float maxTorqueLimit = 3.0f;
```

and converts this to a command clamp of:

```text
±300 firmware torque units
```

because the internal clamp is:

```text
maxTorqueLimit × 100
```

This preserves the scaling behavior of the existing A1 implementation. Actual motor current/torque interpretation depends on the actuator's MyActuator/RMD protocol and motor configuration.

## Manual command example

```text
torque left knee 100
```

sets the left-knee direct torque setpoint to `100`.

The value is not transmitted directly by the command parser. It is stored, and the CAN output task transmits the active setpoint at 100 Hz while `playMode` is enabled.

## Chirality enforcement

If a command addresses the opposite leg, it is rejected.

Example on a controller configured as `left`:

```text
torque right knee 100
```

returns a rejection rather than transmitting to the right knee.

---

# Impedance control

Impedance control is available for joints with external analog joint sensing:

```text
outer_calf
inner_calf
knee
hip_pitch
hip_roll
```

Hip yaw is **direct-torque only** in the current firmware because no external hip-yaw analog sensor is mapped.

## Control law

The implemented controller is:

```text
τ = K(q* - q) + D(dq* - dq) + M(ddq)
```

where:

- `q*` = desired position,
- `q` = measured joint position,
- `dq*` = desired velocity,
- `dq` = estimated velocity,
- `ddq` = estimated acceleration,
- `K` = stiffness,
- `D` = damping,
- `M` = acceleration/feed-forward coefficient used by the existing controller.

The computed result is clamped to the same global torque limit used for manual torque commands.

## Default controller constants

| Joint      | Damping `D` | Stiffness `K` | Mass/accel coefficient `M` |
| ---------- | ----------: | ------------: | -------------------------: |
| Outer calf |         2.5 |          50.0 |                        0.8 |
| Inner calf |         2.5 |          50.0 |                        0.8 |
| Knee       |         3.0 |          60.0 |                       3.55 |
| Hip pitch  |         3.5 |          80.0 |                       9.05 |
| Hip roll   |         3.5 |          80.0 |                       9.05 |

Separate controller instances exist for left and right joints.

## Impedance update rate

The impedance task runs every:

```text
10 ms
```

or approximately:

```text
100 Hz
```

## Derivative initialization

When an impedance controller is first enabled or reset, its previous position and time are initialized before derivative terms are used. This avoids the large first-cycle velocity/acceleration spike produced by treating an uninitialized previous state as valid history.

## Direction multipliers

The calculated impedance torque is multiplied by the saved direction multiplier for the corresponding joint before transmission.

Direction multipliers are currently implemented for:

- outer calf,
- inner calf,
- knee,
- hip pitch,
- hip roll,

on both legs.

Hip yaw has no impedance controller or direction multiplier in the current firmware.

## Impedance command example

```text
impedance left knee 1 180 0
```

means:

```text
leg:              left
joint:            knee
enabled:          yes
desired position: 180 degrees
desired velocity: 0 degrees/s
```

Disable it with:

```text
impedance left knee 0 180 0
```

When impedance is enabled for a joint, its impedance output has precedence over that joint's stored direct torque value.

Disabling impedance causes the CAN output path to fall back to the stored direct torque setpoint for that joint.

---

# Joint constraints

Each joint has:

```text
minimum angle
maximum angle
```

Default constraints are:

```text
0 .. 360 degrees
```

The constraints are persistent.

## Constraint behavior

Constraints currently protect the **impedance controller**.

Before calculating impedance torque:

```text
if actualPosition < minAngle
or actualPosition > maxAngle:
    impedance torque = 0
    controller derivative state is reset
```

Important:

> **Direct `torque` commands do not currently enforce angle constraints.**

Joint constraints therefore must not be treated as a universal hard-stop mechanism.

## Direct constraint command

Example:

```text
constrain knee_left 20 165
```

Supported constraint names:

```text
outer_calf_left
outer_calf_right
inner_calf_left
inner_calf_right
knee_left
knee_right
hip_pitch_left
hip_pitch_right
hip_yaw_left
hip_yaw_right
hip_roll_left
hip_roll_right
```

Hip-yaw constraints are stored but are not currently consumed by an impedance controller because hip yaw has no mapped external sensor.

There is also an interactive:

```text
setJointConstraints
```

command.

---

# Sensor calibration

The `calibrate` command calibrates the five analog joint inputs for the currently selected leg.

The calibration procedure:

1. takes a fresh ten-sample sensor average,
2. converts each sensor to degrees,
3. calculates an offset that makes the current physical pose equal to `180°`,
4. updates the selected leg's offsets in RAM,
5. asks whether those offsets should be written to SPIFFS.

For every sensor:

```text
offset = 180 - rawAngle
```

The five resulting offsets correspond to:

```text
outer calf
inner calf
hip pitch
knee
hip roll
```

The prompt is:

```text
Type 'yes' to save these offsets, anything else to leave them only in RAM.
```

Calibration is unavailable in `center` mode.

## Default hardcoded offsets

If no persisted configuration overrides them, the source contains:

### Left

```text
outer calf:  +32
inner calf:  -26
hip pitch:    -4
knee:        -17
hip roll:     +2
```

### Right

```text
outer calf:  -28
inner calf:  +39
hip pitch:    -2
knee:        +18
hip roll:     -2
```

`resetOffsets` sets **both** left and right stored offset arrays to zero and immediately saves the configuration.

---

# Torque-direction calibration

The firmware can determine the sign multiplier for a sensor-backed joint.

Example:

```text
calibrateDirection left_knee
```

Supported joint names:

```text
left_outer_calf
left_inner_calf
left_knee
left_hip_pitch
left_hip_roll

right_outer_calf
right_inner_calf
right_knee
right_hip_pitch
right_hip_roll
```

The requested joint must belong to the controller's current chirality.

## Calibration algorithm

The firmware:

1. reads the joint's external analog encoder,
2. enables calibration override,
3. tests positive torque commands from `10` through `150` in increments of `10`,
4. holds each test value for approximately `500 ms`,
5. checks whether the encoder moved by more than 10 raw converted degrees/count-equivalent output units,
6. if movement is detected, determines the multiplier from movement direction,
7. otherwise repeats the procedure from `-10` through `-150`,
8. zeros the test motor,
9. leaves calibration override,
10. saves the detected multiplier.

During calibration:

- the test actuator receives the calibration command,
- all other actuators on that leg are actively commanded to zero,
- the normal play/impedance output path does not compete for CAN ownership.

This procedure can physically move a joint for several seconds and must only be performed with the robot mechanically supported and with a physical emergency-stop strategy available.

---

# Stop behavior

The normal stop command is:

```text
stop
```

It performs:

```text
playMode = false
stopBurstRemaining = 3
```

The CAN output task then sends the MyActuator stop command:

```text
0x81
```

to the **actual six actuator CAN IDs for the selected leg**.

This corrects the previous firmware behavior that could iterate numeric values `0..11` rather than the real actuator IDs.

The `0x81` frame is:

```text
81 00 00 00 00 00 00 00
```

After three complete stop bursts, normal CAN transmission stops.

## Resume

Use:

```text
play
```

to resume periodic output.

In leg mode, `play` also clears any pending stop-burst count.

## `zero` is not the same as `stop`

`zero` clears:

- direct torque setpoints,
- current stored impedance torque outputs.

It does **not** disable impedance controllers.

Therefore an enabled impedance controller can calculate a new nonzero torque again on its next 100 Hz update.

For an actual stop condition use:

```text
stop
```

not only:

```text
zero
```

---

# Center / IMU mode

A controller configured as:

```text
center
```

does not start the leg CAN-control path.

The IMU task runs at approximately:

```text
100 Hz
```

and attempts to poll five I²C addresses:

```text
0x68
0x69
0x6A
0x6B
0x6C
```

For each responding address it:

1. writes register address `0x3B`,
2. requests 14 bytes,
3. reads:
   - accelerometer X,
   - accelerometer Y,
   - accelerometer Z,
   - skips temperature,
   - gyroscope X,
   - gyroscope Y,
   - gyroscope Z,
4. prints the raw values over Serial.

Output format:

```text
IMU <index> Acc:<ax>,<ay>,<az> Gyro:<gx>,<gy>,<gz>
```

The firmware does **not** currently:

- configure IMU operating registers,
- convert raw data to SI units,
- perform orientation fusion,
- select channels on an I²C multiplexer,
- process magnetometer data.

It assumes devices are already directly addressable and readable at the addresses above.

---

# Persistent configuration

Configuration is stored in:

```text
/config.txt
```

inside SPIFFS.

The saved data includes:

- controller role (`left`, `right`, or `center`),
- left offsets,
- right offsets,
- ten torque-direction multipliers,
- all left/right joint constraints.

The file is rewritten by `saveConfig()`.

A representative structure is:

```text
LegSide:left
LeftOffsets:32,-26,-4,-17,2
RightOffsets:-28,39,-2,18,-2
DirectionMultipliers:1.000000,1.000000,1.000000,1.000000,1.000000,1.000000,1.000000,1.000000,1.000000,1.000000
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
```

The loader now searches each constraint line for `Constraints:` rather than requiring the line itself to begin with that token. This makes the saved and loaded formats consistent.

## `resetSPIFFS`

Despite its legacy name, the current `resetSPIFFS` command is primarily an **integrity check**.

If `SPIFFS.totalBytes()` is nonzero it reports that SPIFFS is functioning.

If total size is zero, it prompts for confirmation before formatting.

It does **not** normally delete `/config.txt` or reset a healthy filesystem to defaults.

---

# Serial interface

Use:

```text
115200 baud
```

with newline-terminated commands.

The serial command task checks for input approximately every 10 ms.

Normal mode and configuration mode have different accepted command sets.

---

# Command reference

## `help`

Print the full firmware command list.

```text
help
```

---

## `status`

Print current operating state.

```text
status
```

Example fields:

```text
mode
chirality
play
raw
maxTorque
```

In leg mode it also prints the five normalized joint angles.

---

## `chirality`

Print the currently selected controller role.

```text
chirality
```

Possible responses:

```text
left
right
center
```

---

## `left`

Set controller role to left leg and persist it.

```text
left
```

If currently operating as a leg controller, a stop burst is issued first.

---

## `right`

Set controller role to right leg and persist it.

```text
right
```

If currently operating as a leg controller, a stop burst is issued first.

---

## `center`

Set controller role to center/IMU mode and persist it.

```text
center
```

A reboot is required to instantiate the center-mode task topology.

---

## `play`

Enable active output.

```text
play
```

### Leg mode

- cancels any pending stop burst,
- enables 100 Hz torque output.

### Center mode

- enables IMU streaming.

---

## `stop`

Disable play mode and queue three stop bursts.

```text
stop
```

In leg mode this sends `0x81` to all six selected-leg actuator IDs three times.

---

## `zero`

Clear all stored direct and impedance torque values.

```text
zero
```

This does not disable impedance controllers and is not a replacement for `stop`.

---

## `torque`

Set a direct torque command.

Syntax:

```text
torque <legSide> <appendage> <torqueValue>
```

Examples:

```text
torque left knee 100
torque left knee -100
torque right hip_yaw 40
```

Valid appendages:

```text
outer_calf
inner_calf
knee
hip_pitch
hip_yaw
hip_roll
```

The leg side must match the controller's selected chirality.

The command value is clamped to the firmware's current global limit.

---

## `impedance`

Enable/disable impedance control and assign desired state.

Syntax:

```text
impedance <legSide> <appendage> <0|1> <desiredPosition> <desiredVelocity>
```

Examples:

```text
impedance left knee 1 180 0
impedance left hip_pitch 1 170 0
impedance left knee 0 180 0
```

Supported appendages:

```text
outer_calf
inner_calf
knee
hip_pitch
hip_roll
```

Hip yaw is not supported in impedance mode.

The requested leg must match the controller's chirality.

---

## `direction`

Manually set the impedance torque direction multiplier.

Syntax:

```text
direction <joint> <+|->
```

Examples:

```text
direction left_knee +
direction right_outer_calf -
```

Supported joints:

```text
left_outer_calf
left_inner_calf
left_knee
left_hip_pitch
left_hip_roll
right_outer_calf
right_inner_calf
right_knee
right_hip_pitch
right_hip_roll
```

The multiplier is persisted immediately.

Direction multipliers are applied to the **impedance-control output**. Direct `torque` setpoints are sent with the sign supplied by the caller.

---

## `calibrateDirection`

Automatically determine direction multiplier from external joint motion.

Syntax:

```text
calibrateDirection <joint>
```

Example:

```text
calibrateDirection left_knee
```

Only sensor-backed joints are supported.

The joint must belong to the selected leg.

---

## `calibrate`

Calibrate all five analog sensors on the selected leg such that the current pose corresponds to 180°.

```text
calibrate
```

After calculating values, the controller prompts whether to persist them.

---

## `resetOffsets`

Set both left and right sensor offset arrays to zero and save.

```text
resetOffsets
```

---

## `raw on`

Disable offset application.

```text
raw on
```

The sensors remain filtered and converted to degrees, but saved offsets are bypassed.

---

## `raw off`

Re-enable offset application.

```text
raw off
```

---

## `saved`

Print the saved/current offsets for the selected leg.

```text
saved
```

---

## `save`

Rewrite the current configuration to SPIFFS.

```text
save
```

---

## `constrain`

Set and immediately persist one joint's impedance constraint.

Syntax:

```text
constrain <joint> <minAngle> <maxAngle>
```

Example:

```text
constrain knee_left 20 165
```

`minAngle` must be less than or equal to `maxAngle`.

---

## `setJointConstraints`

Interactive constraint setter.

```text
setJointConstraints
```

The controller prompts for:

1. joint name,
2. minimum angle,
3. maximum angle.

---

## `config`

Enter configuration mode.

```text
config
```

Entering configuration mode:

1. requests a three-burst actuator stop,
2. waits approximately 40 ms for the stop frames to be emitted,
3. enables `configMode`.

While in configuration mode, only the following commands are accepted:

```text
exit
left
right
center
calibrate
save
status
```

Other normal-mode commands are rejected as unknown configuration commands.

---

## `exit`

Only used while in configuration mode.

```text
exit
```

Exiting configuration mode does **not** automatically resume actuator output.

Use:

```text
play
```

explicitly when ready.

---

## `mac`

Print the ESP32 Wi-Fi MAC address.

```text
mac
```

Wi-Fi networking is not otherwise used by this firmware.

---

## `resetSPIFFS`

Check SPIFFS integrity and offer formatting only if the filesystem reports zero total bytes.

```text
resetSPIFFS
```

This is not a normal "erase all configuration" command.

---

# Recommended commissioning procedure

The following sequence is recommended whenever flashing a controller or changing actuator/sensor wiring.

## 1. Mechanically support the robot

Do not commission torque-controlled joints while the robot is free-standing.

Remove load where practical and keep a physical power-disconnect / E-stop available.

## 2. Flash the firmware

Upload `esp32_devkit_v1.ino`.

Open Serial Monitor at:

```text
115200
```

with newline line endings.

## 3. Select role on first boot

If `/config.txt` does not exist, enter:

```text
left
```

or:

```text
right
```

for a leg controller.

Use:

```text
center
```

only for the center/IMU controller.

## 4. Verify status

Run:

```text
status
chirality
```

Confirm that the leg matches the physical ESP32.

## 5. Stop before active tests

Run:

```text
stop
```

before calibration or intentional torque testing.

## 6. Verify sensor movement

Use:

```text
raw on
play
```

and observe the five-value CSV telemetry.

Move each mechanism manually and confirm that the expected sensor channel changes.

Then:

```text
stop
raw off
```

## 7. Calibrate offsets

Place the leg in the physical pose that should correspond to `180°` for each sensor and run:

```text
calibrate
```

Review the offsets and answer:

```text
yes
```

only if the pose and values are correct.

## 8. Set constraints

Example:

```text
constrain knee_left 20 165
```

Set realistic per-joint bounds before impedance testing.

Remember that these limits currently guard impedance output only.

## 9. Verify motor direction

With the mechanism unloaded/supported:

```text
calibrateDirection left_knee
```

Repeat for each externally sensed joint on the selected leg.

## 10. Test small direct torque

Example:

```text
torque left knee 10
play
```

Verify the expected direction.

Then:

```text
stop
zero
```

Increase test values gradually only after confirming behavior.

## 11. Test impedance conservatively

Begin near the measured current joint angle with zero desired velocity.

Example:

```text
impedance left knee 1 180 0
play
```

Keep a hardware stop available.

Use:

```text
stop
```

to stop output.

---

# Safety and current limitations

This is low-level robot actuator firmware. Several current properties are important.

## Partial CAN receive-state processing

All leg firmware variants continuously request and decode actuator position
with RMD V4.4 command `0x92`. The five external sensors establish the absolute
joint reference after restart; fresh motor-native feedback then provides the
continuous position used by impedance control. Missing or stale CAN position
fails to zero torque instead of falling back to continuous external-sensor
control.

The firmware does not yet monitor, through CAN:

- actuator velocity,
- actuator current feedback,
- actuator temperature,
- motor fault flags,
- response families beyond position.

## No command watchdog

There is no high-level command heartbeat or timeout that automatically stops a leg when serial/high-level commands disappear.

Once `play` is enabled, the controller continues transmitting the latest selected setpoints until another command changes the state or `stop` is issued.

A production locomotion controller should add a supervisory watchdog.

## Startup enters play mode

At the end of `setup()`, `playMode` is set true.

Initial torque setpoints are zero, but the controller does begin active zero-torque A1 transmission automatically.

## Joint constraints do not protect direct torque mode

Constraints are checked by impedance controllers only.

A direct torque command can continue even when a joint is outside its configured impedance range.

## `zero` does not disable impedance

An enabled impedance controller can repopulate its torque output on the following control update.

Use `stop` for stopping the actuator command path.

## Angle wrap-around

Normalized analog joint angles are wrapped into:

```text
0 <= angle < 360
```

Velocity is currently estimated by direct subtraction of successive wrapped angles.

Crossing `359° -> 0°` or `0° -> 359°` can therefore appear as a large instantaneous position change.

Avoid placing operating trajectories across the wrap boundary until wrapped-angle derivative handling is added.

## Constraints do not currently support wrapped intervals

A simple numeric test is used:

```text
minAngle <= actualPosition <= maxAngle
```

A desired valid interval such as:

```text
330° through 30°
```

cannot currently be represented as a single wrapped constraint.

## Hip yaw has no external joint sensor path

Hip yaw can be directly torque-controlled but cannot currently use the external-sensor impedance loop.

## IMU polling is raw

Center mode does not initialize or fuse IMU devices. It only attempts raw register reads.

## MCP2515 interrupt is not consumed

GPIO17 is reserved/configured for MCP2515 INT, but the current firmware does not attach an interrupt handler or use CAN RX.

## Direction calibration causes intentional motion

`calibrateDirection` can test progressively larger positive and negative commands with roughly 500 ms dwell per step.

Do not run this routine on a loaded or unsupported leg.

---

# Dependencies

The sketch requires:

- Arduino Core for ESP32
- `mcp_can.h` / an MCP2515 CAN library exposing the `MCP_CAN` API
- SPI
- SPIFFS
- Wire / I²C
- WiFi library from the ESP32 Arduino core
- FreeRTOS support provided by the ESP32 Arduino core

The sketch includes:

```cpp
#include <Arduino.h>
#include <mcp_can.h>
#include <SPI.h>
#include <SPIFFS.h>
#include <Wire.h>
#include <WiFi.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <math.h>
```

Use an ESP32 partition configuration that provides SPIFFS storage.

---

# Relationship to `myactuator-can`

The low-level Dropbear controller and the `Hyperspawn/myactuator-can` repository use the same general ESP32 + MCP2515 + MyActuator ecosystem, but the control purposes are different.

`myactuator-can` is useful as a MyActuator CAN protocol/reference implementation.

The Dropbear leg controller intentionally retains a different locomotion architecture:

```text
Dropbear:
external joint sensor
    ↓
ESP32 control loop
    ↓
0xA1 torque command
    ↓
actuator
```

rather than the simpler actuator-internal absolute-position path:

```text
target angle
    ↓
ESP32
    ↓
0xA4 absolute-position command
    ↓
actuator internal position controller
```

For Dropbear's legs, the external joint sensors remain important as restart
absolute references and independent drift checks. Continuous position comes
from the motor-native CAN angle while torque-level output remains under the
ESP32 impedance controller.

The current Dropbear firmware therefore does **not** directly adopt the `myactuator-can` demo motor-ID layout or its `0xA4` position-control behavior.

---

# Legacy behavior removed or changed

This firmware revision changes several important behaviors from the previous low-level controller.

## Correct actuator stop IDs

Old behavior could call the stop function with numeric loop indexes.

Current behavior always sends `0x81` to the actual selected-leg actuator IDs.

## One CAN output authority

The old structure could allow a manual torque task and impedance task to independently send commands.

Current behavior uses one normal periodic CAN writer.

## Per-leg CAN isolation

A leg controller now sends only to its own six actuator IDs.

## Impedance parser fixed

Desired position and desired velocity are parsed as separate arguments.

## Persistent joint constraints fixed

The loader now correctly parses the format written by the saver.

## Center IMU task is actually started

A controller booted as `center` now creates the IMU polling task.

## Telemetry throttled

Sensor acquisition remains approximately 1 kHz, while serial angle telemetry is limited to approximately 50 Hz.

## Calibration output is arbitrated

Direction calibration now uses a dedicated output override rather than racing the normal CAN path.

## Removed legacy commands

The previous README referenced commands such as:

```text
stop_actuator
buzz_motor
```

These are **not commands in the current firmware** and should not be documented as supported.

---

# Development direction

Logical next low-level improvements include:

- command heartbeat/watchdog,
- physical E-stop input,
- bus-off/error recovery,
- actuator temperature/current/fault supervision,
- wrapped-angle-safe derivatives,
- universal hard joint limits applied to every control mode,
- hip-yaw external sensing,
- timestamped binary or structured telemetry,
- high-level command transport independent of debug Serial,
- explicit controller-state machine,
- IMU initialization and sensor fusion,
- deterministic configuration schema/versioning.

---

# License

This project is licensed under the repository's MIT License.


## Observation-only migration image

`esp32_devkit_v1_observation_safe.ino` preserves the reviewed, fail-closed
legacy sensor pinout and emits DB2 dual-angle telemetry with continuous 0x92
motor polling. It is retained as a
separate migration candidate; it is not the Behemoth/universal build and is
not installed on the currently connected controllers. See
`SAFETY_ARCHITECTURE.md` before selecting a firmware image.
