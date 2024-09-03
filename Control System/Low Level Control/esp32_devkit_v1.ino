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
void saveCalibration();
void loadCalibration();
void promptLegSide();
void saveLegSide();
void loadLegSide();
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

void setup() {
  Serial.begin(115200);
  serialMutex = xSemaphoreCreateMutex();
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);  // Initialize I2C for IMU communication

  if (!SPIFFS.begin(true)) {
    Serial.println("An Error has occurred while mounting SPIFFS");
    return;
  }
  loadCalibration();
  loadLegSide();

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
  xTaskCreatePinnedToCore(checkChiralityTask, "Check Chirality Task", 2048, NULL, 1, NULL, 1);

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

void processSerialCommand(String command) {
  command.trim();

  if (xSemaphoreTake(serialMutex, portMAX_DELAY) == pdTRUE) {
    if (command.startsWith("torque")) {
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
      saveLegSide();
      saveCalibration();
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
    } else if (command == "config") {
      enterConfigurationMode();
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
      saveLegSide();
      Serial.println("Chirality set to: " + String(command));
    } else if (command.startsWith("stop_actuator")) {
      String params = command.substring(14); // Get parameters after "stop_actuator "
      long unsigned int actuatorID = strtol(params.c_str(), NULL, 16); // Assume actuator ID is given as hex
      if (actuatorID) {
        sendStopCommand(actuatorID);
        Serial.printf("Sent stop command to actuator ID: 0x%X\n", actuatorID);
      } else {
        Serial.println("Invalid actuator ID.");
      }
    } else if (command.startsWith("buzz_motor")) {
      String params = command.substring(11); // Get parameters after "buzz_motor "
      int firstSpace = params.indexOf(' ');
      int secondSpace = params.indexOf(' ', firstSpace + 1);

      if (firstSpace > 0 && secondSpace > firstSpace) {
        long unsigned int actuatorID = strtol(params.substring(0, firstSpace).c_str(), NULL, 16);
        int frequency = params.substring(firstSpace + 1, secondSpace).toInt();
        float intensity = params.substring(secondSpace + 1).toFloat();

        xTaskCreatePinnedToCore([](void *param) {
          long unsigned int id = ((long unsigned int*)param)[0];
          int freq = ((int*)param)[1];
          float inten = ((float*)param)[2];
          buzzMotorTask(id, freq, inten);
          vTaskDelete(NULL);
        }, "Buzz Motor Task", 4096, new long unsigned int[3]{actuatorID, frequency, (long unsigned int)intensity}, 1, NULL, 1);
      } else {
        Serial.println("Invalid buzz_motor command format.");
      }
    } else {
      Serial.println("Unknown command: " + command);
    }
    xSemaphoreGive(serialMutex);
  }
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
    saveLegSide();
    Serial.println("Chirality set to: " + String(command));
  } else if (command == "calibrate") {
    calibrateSensors();
    Serial.println("Sensors calibrated.");
  } else if (command == "save") {
    saveLegSide();
    saveCalibration();
    Serial.println("Configuration saved.");
  } else {
    Serial.println("Unknown configuration command: " + command);
  }
}

void enterConfigurationMode() {
  if (xSemaphoreTake(serialMutex, portMAX_DELAY) == pdTRUE) {
    configMode = true;
    Serial.println("Entered configuration mode. Type 'exit' to leave.");
    xSemaphoreGive(serialMutex);
  }
}

void exitConfigurationMode() {
  if (xSemaphoreTake(serialMutex, portMAX_DELAY) == pdTRUE) {
    configMode = false;
    Serial.println("Exited configuration mode.");
    xSemaphoreGive(serialMutex);
  }
}

void buzzMotorTask(long unsigned int actuatorID, int frequency, float intensity) {
  int delayTime = 1000 / (frequency * 2); // Time for each half of the cycle (in milliseconds)
  int16_t torqueValue = constrain(intensity * 100, -MAX_TORQUE_LIMIT * 100, MAX_TORQUE_LIMIT * 100);

  for (int i = 0; i < frequency * 5; ++i) { // Run for 5 seconds
    sendTorqueCommand(actuatorID, torqueValue);
    vTaskDelay(delayTime / portTICK_PERIOD_MS);
    sendTorqueCommand(actuatorID, -torqueValue);
    vTaskDelay(delayTime / portTICK_PERIOD_MS);
  }

  sendStopCommand(actuatorID);
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

void normalizeReadings() {
  normalizedOuter = wrapAngle(map(averageOuter, 0, 4096, 0, 360) + offsetOuter);
  normalizedInner = wrapAngle(map(averageInner, 0, 4096, 0, 360) + offsetInner);
  normalizedHip = wrapAngle(map(averageHip, 0, 4096, 0, 360) + offsetHip);
  normalizedKnee = wrapAngle(map(averageKnee, 0, 4096, 0, 360) + offsetKnee);
  normalizedButt = wrapAngle(map(averageButt, 0, 4096, 0, 360) + offsetButt);
}

void calibrateSensors() {
  // Use raw (un-offset) values for calibration
  offsetOuter = wrapAngle(180 - map(averageOuter, 0, 4096, 0, 360));
  offsetInner = wrapAngle(180 - map(averageInner, 0, 4096, 0, 360));
  offsetHip = wrapAngle(180 - map(averageHip, 0, 4096, 0, 360));
  offsetKnee = wrapAngle(180 - map(averageKnee, 0, 4096, 0, 360));
  offsetButt = wrapAngle(180 - map(averageButt, 0, 4096, 0, 360));
  Serial.println("Sensors calibrated. Offsets applied:");
  printSavedOffsets();
}

void printReadings() {
  if (xSemaphoreTake(serialMutex, portMAX_DELAY) == pdTRUE) {
    Serial.print(normalizedOuter);
    Serial.print(",");
    Serial.print(normalizedInner);
    Serial.print(",");
    Serial.print(normalizedHip);
    Serial.print(",");
    Serial.print(normalizedKnee);
    Serial.print(",");
    Serial.println(normalizedButt);
    xSemaphoreGive(serialMutex);
  }
}

// Ensure angles are between 0 and 360 degrees
int wrapAngle(int angle) {
  if (angle >= 360) return angle - 360;
  if (angle < 0) return angle + 360;
  return angle;
}

void saveCalibration() {
  File file = SPIFFS.open("/calibration.txt", FILE_WRITE);
  if (!file) {
    Serial.println("Failed to open file for writing");
    return;
  }

  file.printf("Outer:%d,Inner:%d,Hip:%d,Knee:%d,Butt:%d\n", offsetOuter, offsetInner, offsetHip, offsetKnee, offsetButt);
  file.close();
  Serial.println("Calibration data saved.");
}

void loadCalibration() {
  File file = SPIFFS.open("/calibration.txt");
  if (!file) {
    Serial.println("No calibration file found");
    return;
  }

  String line = file.readStringUntil('\n');
  sscanf(line.c_str(), "Outer:%d,Inner:%d,Hip:%d,Knee:%d,Butt:%d", &offsetOuter, &offsetInner, &offsetHip, &offsetKnee, &offsetButt);
  file.close();
  Serial.println("Calibration data loaded.");
}

void promptLegSide() {
  Serial.println("Enter 'left' for left leg, 'right' for right leg, or 'center' for IMU center.");
  while (!Serial.available()) delay(10);

  String side = Serial.readStringUntil('\n');
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
  Serial.println(isCenter ? "Center mode selected." : (isLeft ? "Left leg selected." : "Right leg selected."));
}

void saveLegSide() {
  File file = SPIFFS.open("/legSide.txt", FILE_WRITE);
  if (!file) {
    Serial.println("Failed to open file for writing");
    return;
  }

  file.println(isCenter ? "center" : (isLeft ? "left" : "right"));
  file.close();
  Serial.println("Leg side configuration saved.");
}

void loadLegSide() {
  File file = SPIFFS.open("/legSide.txt");
  if (!file) {
    Serial.println("No leg side configuration found. Please set chirality or center mode.");
    promptLegSide();
    return;
  }

  String side = file.readStringUntil('\n');
  if (side == "left") {
    isLeft = true;
    isCenter = false;
  } else if (side == "right") {
    isLeft = false;
    isCenter = false;
  } else if (side == "center") {
    isCenter = true;
  }
  file.close();
  Serial.println("Loaded leg side configuration: " + String(isCenter ? "center" : (isLeft ? "left" : "right")));
}

void printSavedOffsets() {
  Serial.printf("Offsets - Outer: %d, Inner: %d, Hip: %d, Knee: %d, Butt: %d\n", offsetOuter, offsetInner, offsetHip, offsetKnee, offsetButt);
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
    int16_t temp = Wire.read() << 8 | Wire.read();

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
