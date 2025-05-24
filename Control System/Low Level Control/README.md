# ESP32 Bipedal Control System

![Frame 2(3)](https://github.com/user-attachments/assets/1a4015b7-2bcd-48eb-9b44-fc3f3899f1f7)


## Overview
This project involves an ESP32-based control system designed to manage bipedal robot locomotion. The system handles multiple tasks, including torque control for actuators, sensor readings, IMU data processing, and CAN bus communication. The control logic ensures stability and smooth operation for the robot's legs by continuously adjusting torque based on sensor inputs and predefined commands.

## Features
- **Torque Control**: Manages torque for 12 actuators (6 per leg) through CAN bus communication.
- **IMU Data Processing**: Reads and processes data from multiple IMUs for accurate orientation and motion tracking.
- **Sensor Calibration**: Calibrates sensors and saves offset values for accurate movement control.
- **Command Interface**: Accepts serial commands to control the robot, adjust settings, and retrieve status information.
- **Multi-tasking**: Utilizes FreeRTOS for concurrent execution of tasks such as torque control, sensor reading, and IMU processing.

## Hardware Requirements
- **ESP32 DevKit V1**
- **MCP2515 CAN Module**
- **IMUs (e.g., MPU-9250)**
- **Torque Sensors**
- **Actuators for bipedal robot legs**

## Pinout
- **CAN0_INT**: GPIO 17 - Interrupt pin for CAN bus.
- **CAN0_CS**: GPIO 5 - Chip select pin for CAN bus.
- **I2C SDA**: GPIO 21 - I2C data line for IMU communication.
- **I2C SCL**: GPIO 22 - I2C clock line for IMU communication.
- **Outer Calf AS5600 Encoder**: GPIO 14 - Analog input for outer calf sensor.
- **Inner Calf AS5600 Encoder**: GPIO 27 - Analog input for inner calf sensor.
- **Hip AS5600 Encoder**: GPIO 26 - Analog input for hip sensor.
- **Knee AS5600 Encoder**: GPIO 25 - Analog input for knee sensor.
- **Butt AS5600 Encoder**: GPIO 33 - Analog input for butt sensor.

## Actuator IDs
- **Right Calf Outer**: 0x144
- **Left Calf Outer**: 0x141
- **Right Calf Inner**: 0x143
- **Left Calf Inner**: 0x142
- **Right Knee**: 0x148
- **Left Knee**: 0x145
- **Right Hip Pitch**: 0x147
- **Left Hip Pitch**: 0x146
- **Right Hip Yaw**: 0x14C
- **Left Hip Yaw**: 0x149
- **Right Hip Roll**: 0x14B
- **Left Hip Roll**: 0x14A

## Usage
1. **Setup and Initialization**: 
   - Connect the ESP32 to the sensors and actuators as per the pinout.
   - Power the system and open a serial monitor at 115200 baud.
   
2. **Calibration**: 
   - Use the `calibrate` command to calibrate sensors and save offset values.
   
3. **Operating Modes**:
   - **Play Mode**: Use the `play` command to enable torque control and begin operation.
   - **Stop Mode**: Use the `stop` command to disable torque control and stop all actuators.
   - **Configuration Mode**: Enter configuration mode with the `config` command to change settings.

4. **Serial Commands**:
   - `torque left outer_calf 200`: Set the torque for the left outer calf to 200.
   - `mac`: Print the MAC address of the ESP32.
   - `chirality`: Check the current leg side or center mode.
   - `save`: Save the current configuration to SPIFFS.
   - `play`: Start the torque control loop.
   - `stop`: Stop all actuators.
   - `config`: Enter configuration mode.
   - `left`, `right`, `center`: Set the chirality (left or right leg) or enable center mode for IMU processing.
   - `stop_actuator 0x144`: Stop a specific actuator by its ID.
   - `buzz_motor 0x144 100 0.5`: Buzz a motor with the specified frequency and intensity.

## Dependencies
- **Arduino Core for ESP32**
- **MCP_CAN Library**
- **SPIFFS**
- **FreeRTOS**

## To-Do
Implement correct channels for IMU's present on calves and feet
Implement streaming of IMU data from torso

## License
This project is licensed under the MIT License.

## Acknowledgments
Special thanks to the contributors of the libraries used in this project.
