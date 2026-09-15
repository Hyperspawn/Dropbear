# ESP32 bipedal control system

## Current source state

This sketch is the source for each leg ESP32. The reviewed source boots in an
observation-only state: external AS5600 angles continue streaming, CAN motion
is disabled, and legacy serial motion commands are denied. These changes are
not active on an installed controller until that controller is separately
built, reviewed, and flashed.

## Observation behavior

- Five single-turn AS5600 analog outputs are sampled through the ESP32 ADC.
- Ten readings are averaged, stored side offsets are applied, and angles are
  emitted in degrees at 50 Hz.
- Each AS5600 angle is a 1:1 actuator output-shaft measurement.
- The knee AS5600 drives the high-mounted upstream knee actuator joint 1:1.
  The closed-loop linkage produces the larger downstream anatomical knee bend;
  firmware and host code must not apply another knee multiplier.
- A disabled-by-default RMD V4.4 `0x92` query path can add independently
  measured motor-native angles after isolated validation.

## Pinout

- **CAN0_INT**: GPIO 17
- **CAN0_CS**: GPIO 5
- **I2C SDA**: GPIO 21
- **I2C SCL**: GPIO 22
- **Outer calf AS5600**: GPIO 14 analog input
- **Inner calf AS5600**: GPIO 27 analog input
- **Hip pitch AS5600**: GPIO 26 analog input
- **Knee actuator AS5600**: GPIO 25 analog input
- **Hip roll AS5600**: GPIO 33 analog input

## Actuator IDs

| Joint | Left | Right |
|---|---:|---:|
| Outer calf | `0x141` | `0x144` |
| Inner calf | `0x142` | `0x143` |
| Knee actuator | `0x145` | `0x148` |
| Hip pitch | `0x146` | `0x147` |
| Hip yaw | `0x149` | `0x14C` |
| Hip roll | `0x14A` | `0x14B` |

## Telemetry

The installed legacy build emits:

```text
outer_calf,inner_calf,hip_pitch,knee_actuator,hip_roll
```

All five values are external sensor degrees. They are not motor encoder values.

The source also defines a future `DB2` record:

```text
DB2,millis,<5 external degrees>,<6 motor-native degrees or NA>
```

The six motor values are ordered outer calf, inner calf, hip pitch, knee,
hip yaw, hip roll. `MOTOR_FEEDBACK_QUERY_ALLOWED` remains `false` because the
`0x92` request transmits a CAN frame. Validate the request and `request + 0x100`
response ID with an isolated unloaded actuator before enabling it. The browser
supports both formats and never fills a missing motor value from an AS5600.

## Current serial surface

- `mac`, `chirality`, `save`, and read-only configuration inspection remain
  available.
- `play`, `torque`, `impedance`, and `calibrateDirection` are denied by the
  legacy motion gate.
- Changing chirality saves the setting, disables CAN, and requires a reboot.
- Browser software zero is intentionally outside this firmware. It neither
  changes stored offsets nor sends a calibration command.

See [SAFETY_ARCHITECTURE.md](SAFETY_ARCHITECTURE.md) for the controller flow,
failure states, and motor-observation release sequence.

## Dependencies

- Arduino Core for ESP32
- MCP_CAN library
- SPIFFS
- FreeRTOS

## License

This project is licensed under the MIT License.
