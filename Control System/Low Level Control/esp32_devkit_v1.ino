#include <Arduino.h>
#include <mcp_can.h>
#include <SPI.h>
#include <SPIFFS.h>
#include <Wire.h>
#include <WiFi.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

// Function declarations
void setup();
void loop();
void checkChiralityTask(void *parameter);
void readAndComputeTask(void *parameter);
void torqueControlTask(void *parameter);
void imuReadTask(void *parameter);
void buzzMotorTask(long unsigned int actuatorID, int frequency, float intensity);

void readSensors();
void computeAverages();
void normalizeReadings();
void calibrateSensors();
void printReadings();
void saveConfig();
void loadConfig();
void resetSPIFFS();
void promptLegSide();
void printSavedOffsets();
void printMACAddress();
void sendTorqueCommand(long unsigned int actuatorID, int16_t torqueValue);
void sendStopCommand(long unsigned int actuatorID);
void processSerialCommand(String command);
void readIMU();
void handleConfigurationCommand(String command);
void enterConfigurationMode();
void exitConfigurationMode();

// Constants for IMU handling
#define IMU_COUNT 5
#define I2C_SDA_PIN 21
#define I2C_SCL_PIN 22

#define CAN0_INT GPIO_NUM_17  // Set INT to gpio pin 17
MCP_CAN CAN(GPIO_NUM_5);      // Set CS to gpio pin 5

// Actuator IDs for right, left leg and IMUs
const long unsigned int ACTUATOR_ID_RIGHT_CALF_OUTER = 0x144;
const long unsigned int ACTUATOR_ID_LEFT_CALF_OUTER = 0x141;
const long unsigned int ACTUATOR_ID_RIGHT_CALF_INNER = 0x143;
const long unsigned int ACTUATOR_ID_LEFT_CALF_INNER = 0x142;
const long unsigned int ACTUATOR_ID_RIGHT_KNEE = 0x148;
const long unsigned int ACTUATOR_ID_LEFT_KNEE = 0x145;
const long unsigned int ACTUATOR_ID_RIGHT_HIP_PITCH = 0x147;
const long unsigned int ACTUATOR_ID_LEFT_HIP_PITCH = 0x146;
const long unsigned int ACTUATOR_ID_RIGHT_HIP_YAW = 0x14C;
const long unsigned int ACTUATOR_ID_LEFT_HIP_YAW = 0x149;
const long unsigned int ACTUATOR_ID_RIGHT_HIP_ROLL = 0x14B;
const long unsigned int ACTUATOR_ID_LEFT_HIP_ROLL = 0x14A;

// Variables to store torque values for each actuator
int16_t torqueValues[12] = {0};

// Maximum torque limit in amps (can be adjusted as needed)
const float MAX_TORQUE_LIMIT = 5.0; // 5 amps

// Sensor readings
int outerCalf = 14;
int innerCalf = 27;
int hip = 26;
int knee = 25;
int butt = 33;

bool isLeft = true;
bool isCenter = false;
const int numReadings = 10;
int readingsOuter[numReadings], readingsInner[numReadings], readingsHip[numReadings], readingsKnee[numReadings], readingsButt[numReadings];
int totalOuter = 0, totalInner = 0, totalHip = 0, totalKnee = 0, totalButt = 0;
int averageOuter = 0, averageInner = 0, averageHip = 0, averageKnee = 0, averageButt = 0;
int normalizedOuter = 0, normalizedInner = 0, normalizedHip = 0, normalizedKnee = 0, normalizedButt = 0;
int offsetOuter = 0, offsetInner = 0, offsetHip = 0, offsetKnee = 0, offsetButt = 0;
int readIndex = 0;
bool playMode = false;
bool configMode = false;
bool stopCommandsSent[12] = {false}; // To track if stop command was printed for each actuator

SemaphoreHandle_t serialMutex;

// Offset arrays for the left and right legs
int leftLegOffsets[5] = {32, -26, -4, -17, 2};   // Offsets for left leg (Outer, Inner, Hip, Knee, Butt)
int rightLegOffsets[5] = {-28, 39, -2, 18, -2};  // Offsets for right leg (Outer, Inner, Hip, Knee, Butt)

void setup() {
  Serial.begin(115200);
  serialMutex = xSemaphoreCreateMutex();
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);  // Initialize I2C for IMU communication

  if (!SPIFFS.begin(true)) {
    Serial.println("An Error has occurred while mounting SPIFFS");
    return;
  }

  loadConfig();

  for (int i = 0; i < numReadings; i++) {
    readingsOuter[i] = readingsInner[i] = readingsHip[i] = readingsKnee[i] = readingsButt[i] = 0;
  }

  // Initialize CAN bus if not in center mode
  if (!isCenter) {
    if (CAN.begin(MCP_ANY, CAN_1000KBPS, MCP_8MHZ) == CAN_OK) {
      Serial.println("CAN bus initialized at 1000kbps.");
    } else {
      Serial.println("CAN bus initialization failed.");
      while (1);
    }
    CAN.setMode(MCP_NORMAL);
  }

  // Create tasks for different functionalities
  xTaskCreatePinnedToCore(readAndComputeTask, "Read and Compute Task", 4096, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(torqueControlTask, "Torque Control Task", 4096, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(checkChiralityTask, "Check Chirality Task", 4096, NULL, 1, NULL, 1); // Increase stack size from 2048 to 4096

  if (isCenter) {
    xTaskCreatePinnedToCore(imuReadTask, "IMU Read Task", 4096, NULL, 1, NULL, 0);
  }
}

void loop() {
  // Main loop does nothing; tasks handle the operations
}

void checkChiralityTask(void *parameter) {
  while (true) {
    if (Serial.available() > 0) {
      String command = Serial.readStringUntil('\n');
      command.trim();
      if (configMode) {
        handleConfigurationCommand(command);
      } else {
        processSerialCommand(command);
      }
    }
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}

void readAndComputeTask(void *parameter) {
  while (true) {
    if (playMode && !isCenter) {
      readSensors();
      computeAverages();
      normalizeReadings();
      printReadings();
    }
    vTaskDelay(1 / portTICK_PERIOD_MS);  // Small delay to prevent CPU hogging
  }
}

void torqueControlTask(void *parameter) {
  while (true) {
    if (playMode && !isCenter) {
      // Send torque commands to all actuators periodically
      sendTorqueCommand(ACTUATOR_ID_RIGHT_CALF_OUTER, torqueValues[0]);
      sendTorqueCommand(ACTUATOR_ID_LEFT_CALF_OUTER, torqueValues[1]);
      sendTorqueCommand(ACTUATOR_ID_RIGHT_CALF_INNER, torqueValues[2]);
      sendTorqueCommand(ACTUATOR_ID_LEFT_CALF_INNER, torqueValues[3]);
      sendTorqueCommand(ACTUATOR_ID_RIGHT_KNEE, torqueValues[4]);
      sendTorqueCommand(ACTUATOR_ID_LEFT_KNEE, torqueValues[5]);
      sendTorqueCommand(ACTUATOR_ID_RIGHT_HIP_PITCH, torqueValues[6]);
      sendTorqueCommand(ACTUATOR_ID_LEFT_HIP_PITCH, torqueValues[7]);
      sendTorqueCommand(ACTUATOR_ID_RIGHT_HIP_YAW, torqueValues[8]);
      sendTorqueCommand(ACTUATOR_ID_LEFT_HIP_YAW, torqueValues[9]);
      sendTorqueCommand(ACTUATOR_ID_RIGHT_HIP_ROLL, torqueValues[10]);
      sendTorqueCommand(ACTUATOR_ID_LEFT_HIP_ROLL, torqueValues[11]);
    } else {
      // Automatically stop all actuators if playMode is false
      for (int i = 0; i < 12; i++) {
        if (!stopCommandsSent[i]) {
          sendStopCommand(i);
          stopCommandsSent[i] = true;
          // Print a single line indicating stop command sent for this actuator
          if (xSemaphoreTake(serialMutex, portMAX_DELAY) == pdTRUE) {
            Serial.printf("Sent stop command to actuator index: %d\n", i);
            xSemaphoreGive(serialMutex);
          }
        }
      }
    }
    vTaskDelay(10 / portTICK_PERIOD_MS);  // Adjust delay to control command frequency
  }
}

void imuReadTask(void *parameter) {
  while (true) {
    if (playMode && isCenter) {
      // Continuously read IMU data
      readIMU();
    }
    vTaskDelay(10 / portTICK_PERIOD_MS);  // Adjust delay as necessary
  }
}

// SPIFFS Integrity Check and Reset function
void resetSPIFFS() {
    Serial.println("Checking SPIFFS integrity...");

    if (SPIFFS.totalBytes() == 0) {
        Serial.println("SPIFFS appears to be unformatted or corrupted.");
        Serial.println("Type 'yes' to format SPIFFS.");
        while (!Serial.available()) delay(10);
        String response = Serial.readStringUntil('\n');
        response.trim();

        if (response.equalsIgnoreCase("yes")) {
            SPIFFS.format();
            Serial.println("SPIFFS formatted.");
        } else {
            Serial.println("SPIFFS reset aborted.");
        }
    } else {
        Serial.println("SPIFFS is functioning properly.");
    }
}


void processSerialCommand(String command) {
  command.trim();

  if (xSemaphoreTake(serialMutex, portMAX_DELAY) == pdTRUE) {
    if (command == "config") {
      enterConfigurationMode();  // Enter config mode directly
    } else if (command == "resetSPIFFS") {
      resetSPIFFS();
    } else if (command.startsWith("torque")) {
      String params = command.substring(7); // Get parameters after "torque "
      int firstSpace = params.indexOf(' ');
      int secondSpace = params.indexOf(' ', firstSpace + 1);

      if (firstSpace > 0 && secondSpace > firstSpace) {
        String legSide = params.substring(0, firstSpace);
        String appendage = params.substring(firstSpace + 1, secondSpace);
        int torqueValue = params.substring(secondSpace + 1).toInt();

        // Apply the maximum torque limit
        torqueValue = constrain(torqueValue, -MAX_TORQUE_LIMIT * 100, MAX_TORQUE_LIMIT * 100);

        int index = -1;
        if (legSide == "left") {
          if (appendage == "outer_calf") index = 1;
          else if (appendage == "inner_calf") index = 3;
          else if (appendage == "knee") index = 5;
          else if (appendage == "hip_pitch") index = 7;
          else if (appendage == "hip_yaw") index = 9;
          else if (appendage == "hip_roll") index = 11;
        } else if (legSide == "right") {
          if (appendage == "outer_calf") index = 0;
          else if (appendage == "inner_calf") index = 2;
          else if (appendage == "knee") index = 4;
          else if (appendage == "hip_pitch") index = 6;
          else if (appendage == "hip_yaw") index = 8;
          else if (appendage == "hip_roll") index = 10;
        }

        if (index >= 0) {
          torqueValues[index] = torqueValue;
          Serial.printf("Torque for %s %s set to %d\n", legSide.c_str(), appendage.c_str(), torqueValue);
        } else {
          Serial.println("Invalid appendage specified.");
        }
      } else {
        Serial.println("Invalid torque command format.");
      }
    } else if (command == "mac") {
      printMACAddress();
    } else if (command == "chirality") {
      String chirality = isCenter ? "center" : (isLeft ? "left" : "right");
      Serial.println("Current chirality: " + chirality);
    } else if (command == "calibrate") {
      calibrateSensors();
    } else if (command == "save") {
      saveConfig();
    } else if (command == "saved") {
      printSavedOffsets();
    } else if (command == "play") {
      playMode = true;
      Serial.println("Play mode enabled.");
    } else if (command == "stop") {
      playMode = false;
      Serial.println("Play mode disabled. Stopping all actuators.");
      for (int i = 0; i < 12; i++) {
        stopCommandsSent[i] = false;  // Reset stop command tracking
      }
    } else if (command == "left" || command == "right" || command == "center") {
      if (command == "left") {
        isLeft = true;
        isCenter = false;
      } else if (command == "right") {
        isLeft = false;
        isCenter = false;
      } else if (command == "center") {
        isCenter = true;
      }
      saveConfig();
      Serial.println("Chirality set to: " + String(command));
    }
    xSemaphoreGive(serialMutex);
  }
}

// Save configuration function
void saveConfig() {
  File file = SPIFFS.open("/config.txt", FILE_WRITE);
  if (!file) {
    Serial.println("Failed to open file for writing");
    return;
  }

  // Save leg side and offsets in one file
  file.printf("LegSide:%s\n", isCenter ? "center" : (isLeft ? "left" : "right"));
  file.printf("LeftOffsets:%d,%d,%d,%d,%d\n", leftLegOffsets[0], leftLegOffsets[1], leftLegOffsets[2], leftLegOffsets[3], leftLegOffsets[4]);
  file.printf("RightOffsets:%d,%d,%d,%d,%d\n", rightLegOffsets[0], rightLegOffsets[1], rightLegOffsets[2], rightLegOffsets[3], rightLegOffsets[4]);

  file.close();
  Serial.println("Configuration saved.");
}

// Load configuration function
void loadConfig() {
  if (!SPIFFS.exists("/config.txt")) {
    // If no config file exists, use hardcoded defaults and prompt for leg side
    Serial.println("No configuration file found. Using hardcoded offsets.");
    promptLegSide();  // Ask the user for the leg side (left, right, or center)
    return;
  }

  // File exists, load it
  File file = SPIFFS.open("/config.txt");
  if (!file) {
    Serial.println("Failed to open configuration file. Using hardcoded offsets.");
    return;
  }

  String line = file.readStringUntil('\n');
  
  // Determine leg side (left, right, center) from the config
  if (line.startsWith("LegSide:")) {
    String side = line.substring(8);
    side.trim();
    if (side == "left") {
      isLeft = true;
      isCenter = false;
    } else if (side == "right") {
      isLeft = false;
      isCenter = false;
    } else if (side == "center") {
      isCenter = true;
    }
  }

  // Load left leg offsets
  line = file.readStringUntil('\n');
  if (line.startsWith("LeftOffsets:")) {
    sscanf(line.c_str(), "LeftOffsets:%d,%d,%d,%d,%d", &leftLegOffsets[0], &leftLegOffsets[1], &leftLegOffsets[2], &leftLegOffsets[3], &leftLegOffsets[4]);
  } else {
    Serial.println("No left offsets found in SPIFFS, using hardcoded left leg offsets.");
  }

  // Load right leg offsets
  line = file.readStringUntil('\n');
  if (line.startsWith("RightOffsets:")) {
    sscanf(line.c_str(), "RightOffsets:%d,%d,%d,%d,%d", &rightLegOffsets[0], &rightLegOffsets[1], &rightLegOffsets[2], &rightLegOffsets[3], &rightLegOffsets[4]);
  } else {
    Serial.println("No right offsets found in SPIFFS, using hardcoded right leg offsets.");
  }

  file.close();
  Serial.println("Configuration loaded successfully.");
}


void handleConfigurationCommand(String command) {
  command.trim();
  if (command == "exit") {
    exitConfigurationMode();
  } else if (command == "left" || command == "right" || command == "center") {
    if (command == "left") {
      isLeft = true;
      isCenter = false;
    } else if (command == "right") {
      isLeft = false;
      isCenter = false;
    } else if (command == "center") {
      isCenter = true;
    }
    saveConfig();
    Serial.println("Chirality set to: " + String(command));
  } else if (command == "calibrate") {
    calibrateSensors();
    Serial.println("Sensors calibrated.");
  } else if (command == "save") {
    saveConfig();
    Serial.println("Configuration saved.");
  } else {
    Serial.println("Unknown configuration command: " + command);
  }
}

void enterConfigurationMode() {
  playMode = false;  // Ensure play mode is disabled
  Serial.println("Entering Configuration Mode...");

  // Stop all actuators three times to ensure they are stopped
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 12; j++) {
      sendStopCommand(j);
    }
  }
  
  configMode = true;
  Serial.println("Configuration mode entered. Type 'exit' to leave.");
}

void exitConfigurationMode() {
  Serial.println("Exiting Configuration Mode...");
  configMode = false;
  Serial.println("Configuration mode exited.");
}

void readSensors() {
  totalOuter -= readingsOuter[readIndex];
  totalInner -= readingsInner[readIndex];
  totalHip -= readingsHip[readIndex];
  totalKnee -= readingsKnee[readIndex];
  totalButt -= readingsButt[readIndex];

  readingsOuter[readIndex] = analogRead(outerCalf);
  readingsInner[readIndex] = analogRead(innerCalf);
  readingsHip[readIndex] = analogRead(hip);
  readingsKnee[readIndex] = analogRead(knee);
  readingsButt[readIndex] = analogRead(butt);

  totalOuter += readingsOuter[readIndex];
  totalInner += readingsInner[readIndex];
  totalHip += readingsHip[readIndex];
  totalKnee += readingsKnee[readIndex];
  totalButt += readingsButt[readIndex];

  readIndex = (readIndex + 1) % numReadings;
}

void computeAverages() {
  averageOuter = totalOuter / numReadings;
  averageInner = totalInner / numReadings;
  averageHip = totalHip / numReadings;
  averageKnee = totalKnee / numReadings;
  averageButt = totalButt / numReadings;
}

// Function to normalize sensor readings and apply offsets based on chirality
void normalizeReadings() {
  const int* offsets = isLeft ? leftLegOffsets : rightLegOffsets; // Use the appropriate offsets array

  // Apply offsets for each reading
  normalizedOuter = wrapAngleFloat(map(averageOuter, 0, 4096, 0, 3600) / 10.0 + offsets[0]);
  normalizedInner = wrapAngleFloat(map(averageInner, 0, 4096, 0, 3600) / 10.0 + offsets[1]);
  normalizedHip = wrapAngleFloat(map(averageHip, 0, 4096, 0, 3600) / 10.0 + offsets[2]);
  normalizedKnee = wrapAngleFloat(map(averageKnee, 0, 4096, 0, 3600) / 10.0 + offsets[3]);
  normalizedButt = wrapAngleFloat(map(averageButt, 0, 4096, 0, 3600) / 10.0 + offsets[4]);
}

// Ensure angles are between 0.0 and 360.0 degrees, with tenths precision
float wrapAngleFloat(float angle) {
  if (angle >= 360.0) return angle - 360.0;
  if (angle < 0.0) return angle + 360.0;
  return angle;
}

void calibrateSensors() {
    // Map sensor readings to 0-360 degrees with tenths precision
    float rawOuter = map(averageOuter, 0, 4096, 0, 3600) / 10.0;
    float rawInner = map(averageInner, 0, 4096, 0, 3600) / 10.0;
    float rawHip = map(averageHip, 0, 4096, 0, 3600) / 10.0;
    float rawKnee = map(averageKnee, 0, 4096, 0, 3600) / 10.0;
    float rawButt = map(averageButt, 0, 4096, 0, 3600) / 10.0;

    // Calculate the offsets needed to align the readings to 180 degrees
    int offsetOuter = (rawOuter > 180.0) ? 180.0 - rawOuter : 180.0 - rawOuter;
    int offsetInner = (rawInner > 180.0) ? 180.0 - rawInner : 180.0 - rawInner;
    int offsetHip = (rawHip > 180.0) ? 180.0 - rawHip : 180.0 - rawHip;
    int offsetKnee = (rawKnee > 180.0) ? 180.0 - rawKnee : 180.0 - rawKnee;
    int offsetButt = (rawButt > 180.0) ? 180.0 - rawButt : 180.0 - rawButt;

    // Assign the offsets to the correct array (left or right leg)
    if (isLeft) {
        leftLegOffsets[0] = offsetOuter;
        leftLegOffsets[1] = offsetInner;
        leftLegOffsets[2] = offsetHip;
        leftLegOffsets[3] = offsetKnee;
        leftLegOffsets[4] = offsetButt;
    } else {
        rightLegOffsets[0] = offsetOuter;
        rightLegOffsets[1] = offsetInner;
        rightLegOffsets[2] = offsetHip;
        rightLegOffsets[3] = offsetKnee;
        rightLegOffsets[4] = offsetButt;
    }

    // Print the calculated offsets
    Serial.println("Calibration complete. Offsets to align readings to 180 degrees:");
    Serial.printf("Outer Offset: %d\n", offsetOuter);
    Serial.printf("Inner Offset: %d\n", offsetInner);
    Serial.printf("Hip Offset: %d\n", offsetHip);
    Serial.printf("Knee Offset: %d\n", offsetKnee);
    Serial.printf("Butt Offset: %d\n", offsetButt);

    // Prompt the user to save the calibration
    Serial.println("Do you want to save these offsets? Type 'yes' to save, 'no' to discard.");
    
    // Wait for user input
    while (!Serial.available()) delay(10);

    String response = Serial.readStringUntil('\n');
    response.trim();

    if (response.equalsIgnoreCase("yes")) {
        saveConfig();
        Serial.println("Offsets saved.");
    } else {
        Serial.println("Offsets not saved.");
    }
}

// Print normalized readings with tenths precision
void printReadings() {
  if (xSemaphoreTake(serialMutex, portMAX_DELAY) == pdTRUE) {
    Serial.print(normalizedOuter, 1);  // Print with 1 decimal place
    Serial.print(",");
    Serial.print(normalizedInner, 1);
    Serial.print(",");
    Serial.print(normalizedHip, 1);
    Serial.print(",");
    Serial.print(normalizedKnee, 1);
    Serial.print(",");
    Serial.println(normalizedButt, 1);
    xSemaphoreGive(serialMutex);
  }
}

// Ensure angles are between 0 and 360 degrees
int wrapAngle(int angle) {
  if (angle >= 360) return angle - 360;
  if (angle < 0) return angle + 360;
  return angle;
}

void printSavedOffsets() {
  Serial.printf("Offsets - Outer: %d, Inner: %d, Hip: %d, Knee: %d, Butt: %d\n", offsetOuter, offsetInner, offsetHip, offsetKnee, offsetButt);
}

void promptLegSide() {
  Serial.println("Enter 'left' for left leg, 'right' for right leg, or 'center' for IMU center.");
  
  // Wait for input from the user
  while (!Serial.available()) {
    delay(10);
  }

  String side = Serial.readStringUntil('\n');
  side.trim();
  
  // Set the leg side based on user input
  if (side == "left") {
    isLeft = true;
    isCenter = false;
  } else if (side == "right") {
    isLeft = false;
    isCenter = false;
  } else if (side == "center") {
    isCenter = true;
  }

  Serial.println(isCenter ? "Center mode selected." : (isLeft ? "Left leg selected." : "Right leg selected."));
}

void printMACAddress() {
  uint8_t mac[6];
  WiFi.macAddress(mac);
  Serial.printf("MAC Address: %02X:%02X:%02X:%02X:%02X:%02X\n", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}

void sendTorqueCommand(long unsigned int actuatorID, int16_t torqueValue) {
  byte buf[8] = {0xA1, 0x00, 0x00, 0x00, (byte)(torqueValue & 0xFF), (byte)(torqueValue >> 8), 0x00, 0x00};
  CAN.sendMsgBuf(actuatorID, 0, 8, buf);
}

void sendStopCommand(long unsigned int actuatorID) {
  byte buf[8] = {0x81, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  CAN.sendMsgBuf(actuatorID, 0, 8, buf);
}

void readIMU() {
    // Example function for reading from multiple IMUs over I2C
    for (int i = 0; i < IMU_COUNT; i++) {
        Wire.beginTransmission(0x68 + i);  // Replace with actual I2C address of each IMU
        Wire.write(0x3B);  // Start reading at a specific register, example for MPU-9250
        Wire.endTransmission(false);
        Wire.requestFrom(0x68 + i, 14, true);  // Request 14 bytes from the IMU

        int16_t ax = Wire.read() << 8 | Wire.read();
        int16_t ay = Wire.read() << 8 | Wire.read();
        int16_t az = Wire.read() << 8 | Wire.read();
        int16_t gx = Wire.read() << 8 | Wire.read();
        int16_t gy = Wire.read() << 8 | Wire.read();
        int16_t gz = Wire.read() << 8 | Wire.read();

        // Removed 'temp' since it was unused
        // int16_t temp = Wire.read() << 8 | Wire.read(); 

        Serial.print("IMU ");
        Serial.print(i);
        Serial.print(" Acc: ");
        Serial.print(ax);
        Serial.print(", ");
        Serial.print(ay);
        Serial.print(", ");
        Serial.print(az);
        Serial.print(" Gyro: ");
        Serial.print(gx);
        Serial.print(", ");
        Serial.print(gy);
        Serial.print(", ");
        Serial.println(gz);
    }
}

