# HX711 Calibration and SPIFFS Persistence Script

## Overview
This script is designed for the ESP32 to interface with up to four HX711 load cell modules using a shared clock pin and individual data pins. It includes a guided calibration routine and saves the calibration factors to SPIFFS, allowing for persistent calibration across device restarts. The script is integrated into a system that also uses other peripherals, so non-conflicting GPIO pins have been selected.

## Features
- **Multiple HX711 Load Cells:**  
  Supports four load cells with a shared clock (GPIO 4) and dedicated data pins (GPIOs 32, 12, 13, and 15).

- **Calibration Routine:**  
  - Activated via the Serial Monitor by typing `/calibrate`.  
  - Automatically detects which load cell is pressed.  
  - Guides you through three calibration steps:  
    1. **Baseline (No Load):** Ensures a stable reading (50 samples with <5 units variation).  
    2. **First Known Force:** Measures with a known applied force.  
    3. **Second Known Force:** Measures with a different known applied force.
  - Calculates and applies the new calibration factor for the selected load cell.

- **SPIFFS Persistence:**  
  Calibration factors are saved to SPIFFS in `/calibration.txt` so that they are loaded automatically on startup.

- **Serial Data Output:**  
  Outputs CSV-formatted readings from all load cells for easy plotting and monitoring.

## Hardware Setup
- **HX711 Modules:**  
  Connect each HX711 module to its respective data pin (GPIO 32, 12, 13, 15) with all modules sharing the same clock pin (GPIO 4).

- **Other Peripherals:**  
  The following GPIO pins are already used by other modules in your system:
  - **MCP_CAN:**  
    - GPIO 5 (Chip Select)  
    - GPIO 17 (Interrupt)
  - **I2C for IMU/Multiplexer:**  
    - GPIO 21 (SDA)  
    - GPIO 22 (SCL)
  - **Analog Sensors:**  
    - GPIO 14 (Outer Calf sensor)  
    - GPIO 27 (Inner Calf sensor)  
    - GPIO 26 (Hip sensor)  
    - GPIO 25 (Knee sensor)  
    - GPIO 33 (Butt/Hip Roll sensor)

Refer to the [GPIO Pin Table](#gpio-pin-table) above for complete details.

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
   Flash the script to your ESP32 using your preferred IDE.

2. **Open the Serial Monitor:**  
   Set the baud rate to 115200.

3. **Calibrate a Load Cell:**  
   - Type `/calibrate` in the Serial Monitor and press Enter.
   - Apply pressure to one load cell to select it for calibration.
   - Follow the on-screen prompts to:
     - Record a baseline (no load).
     - Apply a known force and input the weight.
     - Apply a different known force and input the weight.
   - The script will validate the stability of readings and calculate a new calibration factor.

4. **View Readings:**  
   The script continuously prints the calibrated readings from all load cells in CSV format.

5. **Persistent Calibration:**  
   Calibration data is stored in SPIFFS and is automatically loaded on subsequent startups.

## Notes
- Adjust thresholds, delays, or messaging as needed for your specific application.
- Ensure that all connections are secure and that the wiring matches the GPIO assignments provided.
- The calibration routine checks for stable readings (variation < 5 units over 50 samples). If unstable, you will be prompted to repeat the measurement.
