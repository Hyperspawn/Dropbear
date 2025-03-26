# HX711 Calibration and SPIFFS Persistence Script

## Overview
This script is designed for the ESP32 to interface with up to four HX711 load cell modules using a shared clock pin and individual data pins. It includes a guided calibration routine and saves the calibration factors to SPIFFS, allowing for persistent calibration across device restarts. The script is integrated into a system that also uses other peripherals, so non-conflicting GPIO pins have been selected.


https://github.com/user-attachments/assets/f911e795-2ed4-4077-9165-cb4e07b29ae3

## Key Features

- **Calibration:**  
  Calibrate any one of the four load cells by applying a known force. The calibration routine guides you through:
  - A baseline (no load) measurement.
  - Two measurements with known forces.
  
  The new calibration factor is computed and saved to SPIFFS.

- **Drift Correction (/drift):**  
  Recalculates the baseline offsets (no-load values) by averaging 50 stable readings for each load cell. This is useful to correct for drift over time.

- **Clear Calibration (/clear):**  
  Deletes the calibration file from SPIFFS and resets all calibration factors to the default (2280.0) and baseline offsets to zero.

- **Transient Spike Filtering:**  
  For each load cell, the last 5 readings are stored in a circular buffer. If a new reading deviates from the average of these 5 samples by more than 500 units, it is replaced with the average, preventing sudden spikes from affecting the output.

---

## Available Commands

- **/calibrate:**  
  Initiates the calibration routine for a selected load cell.  
  _Usage:_ Type `/calibrate` in the Serial Monitor.

- **/drift:**  
  Resets the baseline offsets by averaging 50 stable samples from each load cell to correct for drift.  
  _Usage:_ Type `/drift` in the Serial Monitor.

- **/clear:**  
  Clears the calibration data stored in SPIFFS and resets calibration factors and offsets to default values.  
  _Usage:_ Type `/clear` in the Serial Monitor.

---


### GPIO Pin Table

| GPIO Pin | Peripheral/Module            | Description / Usage                                              |
|----------|------------------------------|------------------------------------------------------------------|
| **4**    | HX711 SCK (Shared)           | Shared clock pin for all HX711 modules.                          |
| **32**   | HX711 DOUT (Load Cell 0)     | Data pin for HX711 module connected to Load Cell 0.              |
| **12**   | HX711 DOUT (Load Cell 1)     | Data pin for HX711 module connected to Load Cell 1.              |
| **13**   | HX711 DOUT (Load Cell 2)     | Data pin for HX711 module connected to Load Cell 2.              |
| **15**   | HX711 DOUT (Load Cell 3)     | Data pin for HX711 module connected to Load Cell 3.              |
| **5**    | MCP_CAN CS                   | Chip Select for MCP_CAN module over SPI.                         |
| **17**   | MCP_CAN INT                  | Interrupt pin for MCP_CAN module.                                |
| **21**   | I2C SDA (IMU/Multiplexer)    | SDA line for I2C communications with IMU sensors/multiplexer.      |
| **22**   | I2C SCL (IMU/Multiplexer)    | SCL line for I2C communications with IMU sensors/multiplexer.      |
| **14**   | Analog Input (Outer Calf)    | Reads analog sensor data for the outer calf joint.               |
| **27**   | Analog Input (Inner Calf)    | Reads analog sensor data for the inner calf joint.               |
| **26**   | Analog Input (Hip)           | Reads analog sensor data for the hip joint.                      |
| **25**   | Analog Input (Knee)          | Reads analog sensor data for the knee joint.                     |
| **33**   | Analog Input (Butt/Hip Roll) | Reads analog sensor data for the butt/hip roll joint.            |

---

## Usage Instructions


1. **Upload the Code:**  
   Flash the code to your ESP32 using your preferred IDE.

2. **Open the Serial Monitor:**  
   Set the baud rate to 115200.

3. **Enter Commands:**  
   - Type `/calibrate` to calibrate a load cell.
   - Type `/drift` to reset the baseline (correct drift).
   - Type `/clear` to remove saved calibration data from SPIFFS.

4. **View Output:**  
   The ESP32 continuously prints CSV-formatted weight readings from all four load cells (after calibration, drift correction, and filtering) for monitoring or plotting.
