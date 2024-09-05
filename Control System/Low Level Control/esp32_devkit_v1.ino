#include <Arduino.h>
#include <mcp_can.h>
#include <SPI.h>
#include <SPIFFS.h>
#include <Wire.h>
#include <WiFi.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

// Struct to hold joint constraints
struct JointConstraints {
  int minAngle = 0;    // Default to 0
  int maxAngle = 360;  // Default to 360
};

// Joint constraints for each joint
JointConstraints outerCalfConstraintsLeft;
JointConstraints outerCalfConstraintsRight;
JointConstraints innerCalfConstraintsLeft;
JointConstraints innerCalfConstraintsRight;
JointConstraints kneeConstraintsLeft;
JointConstraints kneeConstraintsRight;
JointConstraints hipPitchConstraintsLeft;
JointConstraints hipPitchConstraintsRight;
JointConstraints hipYawConstraintsLeft;
JointConstraints hipYawConstraintsRight;
JointConstraints hipRollConstraintsLeft;
JointConstraints hipRollConstraintsRight;

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
int16_t torqueValues[12] = { 0 };

// Maximum torque limit in amps (adjustable via command)
float maxTorqueLimit = 3.0;

// Sensor readings
int outerCalf = 14;
int innerCalf = 27;
int hip = 26;
int knee = 25;
int butt = 33;

bool isLeft = true;
bool isCenter = false;
bool rawMode = false;  // To track whether raw mode is enabled

const int numReadings = 10;
int readingsOuter[numReadings], readingsInner[numReadings], readingsHip[numReadings], readingsKnee[numReadings], readingsButt[numReadings];
int totalOuter = 0, totalInner = 0, totalHip = 0, totalKnee = 0, totalButt = 0;
int averageOuter = 0, averageInner = 0, averageHip = 0, averageKnee = 0, averageButt = 0;
int normalizedOuter = 0, normalizedInner = 0, normalizedHip = 0, normalizedKnee = 0, normalizedButt = 0;
int offsetOuter = 0, offsetInner = 0, offsetHip = 0, offsetKnee = 0, offsetButt = 0;
int readIndex = 0;
bool playMode = false;
bool configMode = false;
bool stopCommandsSent[12] = { false };  // To track if stop command was printed for each actuator

SemaphoreHandle_t serialMutex;

// Low-pass filter to smooth the incoming angle readings
float filteredAngle(float currentAngle, float previousAngle, float alpha = 0.1) {
  return alpha * currentAngle + (1 - alpha) * previousAngle;
}

// Offset arrays for the left and right legs
int leftLegOffsets[5] = { 32, -26, -4, -17, 2 };   // Offsets for left leg (Outer, Inner, Hip, Knee, Butt)
int rightLegOffsets[5] = { -28, 39, -2, 18, -2 };  // Offsets for right leg (Outer, Inner, Hip, Knee, Butt)

// Impedance Control Class
class ImpedanceControl {
public:
  float damping;
  float stiffness;
  float mass;
  float desiredPosition;
  float desiredVelocity;
  float torqueOutput;
  unsigned long lastTime;
  float lastPosition;
  float lastVelocity;

  ImpedanceControl(float damping, float stiffness, float mass)
    : damping(damping), stiffness(stiffness), mass(mass),
      desiredPosition(180), desiredVelocity(0), torqueOutput(0),
      lastTime(0), lastPosition(0), lastVelocity(0) {}  // Set default to 180

  void update(float actualPosition, unsigned long currentTime, int minAngle, int maxAngle) {
    // Ensure the position is within constraints
    if (actualPosition < minAngle || actualPosition > maxAngle) {
      torqueOutput = 0;  // Stop torque if out of bounds
      return;
    }

    float deltaTime = (currentTime - lastTime) / 1000.0;
    if (deltaTime > 0) {
      float actualVelocity = (actualPosition - lastPosition) / deltaTime;
      float actualAcceleration = (actualVelocity - lastVelocity) / deltaTime;

      float positionError = desiredPosition - actualPosition;
      float velocityError = desiredVelocity - actualVelocity;

      torqueOutput = stiffness * positionError + damping * velocityError + mass * actualAcceleration;

      applyTorqueSaturation();

      lastPosition = actualPosition;
      lastVelocity = actualVelocity;
      lastTime = currentTime;
    }
  }

  void setDesiredPosition(float position) {
    desiredPosition = position;
  }

  void setDesiredVelocity(float velocity) {
    desiredVelocity = velocity;
  }

private:
  void applyTorqueSaturation() {
    if (torqueOutput > maxTorqueLimit * 100) {
      torqueOutput = maxTorqueLimit * 100;
    } else if (torqueOutput < -maxTorqueLimit * 100) {
      torqueOutput = -maxTorqueLimit * 100;
    }
  }
};

// Sluggish and stable impedance control
ImpedanceControl outerCalfControlRight(2.5, 50.0, 0.8);  // Foot mass
ImpedanceControl outerCalfControlLeft(2.5, 50.0, 0.8);   // Foot mass
ImpedanceControl innerCalfControlRight(2.5, 50.0, 0.8);  // Foot mass
ImpedanceControl innerCalfControlLeft(2.5, 50.0, 0.8);   // Foot mass
ImpedanceControl kneeControlRight(3.0, 60.0, 3.55);      // Calf + foot mass
ImpedanceControl kneeControlLeft(3.0, 60.0, 3.55);       // Calf + foot mass
ImpedanceControl hipPitchControlRight(3.5, 80.0, 9.05);  // Thigh + calf + foot mass
ImpedanceControl hipPitchControlLeft(3.5, 80.0, 9.05);   // Thigh + calf + foot mass
ImpedanceControl hipRollControlRight(3.5, 80.0, 9.05);   // Thigh + calf + foot mass
ImpedanceControl hipRollControlLeft(3.5, 80.0, 9.05);    // Thigh + calf + foot mass

// Direction multipliers for each actuator
float directionMultiplierRightOuterCalf = 1.0;
float directionMultiplierRightInnerCalf = 1.0;
float directionMultiplierLeftOuterCalf = 1.0;
float directionMultiplierLeftInnerCalf = 1.0;
float directionMultiplierRightKnee = 1.0;
float directionMultiplierLeftKnee = 1.0;
float directionMultiplierRightHipPitch = 1.0;
float directionMultiplierLeftHipPitch = 1.0;
float directionMultiplierRightHipRoll = 1.0;
float directionMultiplierLeftHipRoll = 1.0;

// Flags to track whether impedance control is enabled for each actuator
bool impedanceEnabledRightOuterCalf = false;
bool impedanceEnabledRightInnerCalf = false;
bool impedanceEnabledLeftOuterCalf = false;
bool impedanceEnabledLeftInnerCalf = false;
bool impedanceEnabledRightKnee = false;
bool impedanceEnabledLeftKnee = false;
bool impedanceEnabledRightHipPitch = false;
bool impedanceEnabledLeftHipPitch = false;
bool impedanceEnabledRightHipRoll = false;
bool impedanceEnabledLeftHipRoll = false;

// Function to reverse direction if needed (manual command or visual inspection)
void setDirectionMultiplier(String jointName, float value) {
  if (jointName == "right_outer_calf") {
    directionMultiplierRightOuterCalf = value;
  } else if (jointName == "right_inner_calf") {
    directionMultiplierRightInnerCalf = value;
  } else if (jointName == "left_outer_calf") {
    directionMultiplierLeftOuterCalf = value;
  } else if (jointName == "left_inner_calf") {
    directionMultiplierLeftInnerCalf = value;
  } else if (jointName == "right_knee") {
    directionMultiplierRightKnee = value;
  } else if (jointName == "left_knee") {
    directionMultiplierLeftKnee = value;
  } else if (jointName == "right_hip_pitch") {
    directionMultiplierRightHipPitch = value;
  } else if (jointName == "left_hip_pitch") {
    directionMultiplierLeftHipPitch = value;
  } else if (jointName == "right_hip_roll") {
    directionMultiplierRightHipRoll = value;
  } else if (jointName == "left_hip_roll") {
    directionMultiplierLeftHipRoll = value;
  } else {
    Serial.println("Invalid joint name.");
  }
}

void impedanceControlTask(void *parameter) {
  const TickType_t xFrequency = pdMS_TO_TICKS(10);  // Adjusted delay to avoid overloading
  TickType_t xLastWakeTime = xTaskGetTickCount();

  while (true) {
    unsigned long currentTime = millis();

    // Outer Calf Right (with direction multiplier)
    if (impedanceEnabledRightOuterCalf) {
      outerCalfControlRight.update(normalizedOuter, currentTime, outerCalfConstraintsRight.minAngle, outerCalfConstraintsRight.maxAngle);
      sendTorqueCommand(ACTUATOR_ID_RIGHT_CALF_OUTER, outerCalfControlRight.torqueOutput * directionMultiplierRightOuterCalf);
    }

    // Outer Calf Left (with direction multiplier)
    if (impedanceEnabledLeftOuterCalf) {
      outerCalfControlLeft.update(normalizedOuter, currentTime, outerCalfConstraintsLeft.minAngle, outerCalfConstraintsLeft.maxAngle);
      sendTorqueCommand(ACTUATOR_ID_LEFT_CALF_OUTER, outerCalfControlLeft.torqueOutput * directionMultiplierLeftOuterCalf);
    }

    // Inner Calf Right (with direction multiplier)
    if (impedanceEnabledRightInnerCalf) {
      innerCalfControlRight.update(normalizedInner, currentTime, innerCalfConstraintsRight.minAngle, innerCalfConstraintsRight.maxAngle);
      sendTorqueCommand(ACTUATOR_ID_RIGHT_CALF_INNER, innerCalfControlRight.torqueOutput * directionMultiplierRightInnerCalf);
    }

    // Inner Calf Left (with direction multiplier)
    if (impedanceEnabledLeftInnerCalf) {
      innerCalfControlLeft.update(normalizedInner, currentTime, innerCalfConstraintsLeft.minAngle, innerCalfConstraintsLeft.maxAngle);
      sendTorqueCommand(ACTUATOR_ID_LEFT_CALF_INNER, innerCalfControlLeft.torqueOutput * directionMultiplierLeftInnerCalf);
    }

    // Knee Right (with direction multiplier)
    if (impedanceEnabledRightKnee) {
      kneeControlRight.update(normalizedKnee, currentTime, kneeConstraintsRight.minAngle, kneeConstraintsRight.maxAngle);
      sendTorqueCommand(ACTUATOR_ID_RIGHT_KNEE, kneeControlRight.torqueOutput * directionMultiplierRightKnee);
    }

    // Knee Left (with direction multiplier)
    if (impedanceEnabledLeftKnee) {
      kneeControlLeft.update(normalizedKnee, currentTime, kneeConstraintsLeft.minAngle, kneeConstraintsLeft.maxAngle);
      sendTorqueCommand(ACTUATOR_ID_LEFT_KNEE, kneeControlLeft.torqueOutput * directionMultiplierLeftKnee);
    }

    // Hip Pitch Right (with direction multiplier)
    if (impedanceEnabledRightHipPitch) {
      hipPitchControlRight.update(normalizedHip, currentTime, hipPitchConstraintsRight.minAngle, hipPitchConstraintsRight.maxAngle);
      sendTorqueCommand(ACTUATOR_ID_RIGHT_HIP_PITCH, hipPitchControlRight.torqueOutput * directionMultiplierRightHipPitch);
    }

    // Hip Pitch Left (with direction multiplier)
    if (impedanceEnabledLeftHipPitch) {
      hipPitchControlLeft.update(normalizedHip, currentTime, hipPitchConstraintsLeft.minAngle, hipPitchConstraintsLeft.maxAngle);
      sendTorqueCommand(ACTUATOR_ID_LEFT_HIP_PITCH, hipPitchControlLeft.torqueOutput * directionMultiplierLeftHipPitch);
    }

    // Hip Roll Right (with direction multiplier)
    if (impedanceEnabledRightHipRoll) {
      hipRollControlRight.update(normalizedButt, currentTime, hipRollConstraintsRight.minAngle, hipRollConstraintsRight.maxAngle);
      sendTorqueCommand(ACTUATOR_ID_RIGHT_HIP_ROLL, hipRollControlRight.torqueOutput * directionMultiplierRightHipRoll);
    }

    // Hip Roll Left (with direction multiplier)
    if (impedanceEnabledLeftHipRoll) {
      hipRollControlLeft.update(normalizedButt, currentTime, hipRollConstraintsLeft.minAngle, hipRollConstraintsLeft.maxAngle);
      sendTorqueCommand(ACTUATOR_ID_LEFT_HIP_ROLL, hipRollControlLeft.torqueOutput * directionMultiplierLeftHipRoll);
    }

    // Use vTaskDelayUntil for accurate task timing
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

void setup() {
  Serial.begin(115200);
  serialMutex = xSemaphoreCreateMutex();
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);  // Initialize I2C for IMU communication

  if (!SPIFFS.begin(true)) {
    Serial.println("An Error has occurred while mounting SPIFFS");
    return;
  }

  loadConfig();

  // Initialize CAN bus if not in center mode
  if (!isCenter) {
    if (CAN.begin(MCP_ANY, CAN_1000KBPS, MCP_8MHZ) == CAN_OK) {
      Serial.println("CAN bus initialized at 1000kbps.");
    } else {
      Serial.println("CAN bus initialization failed.");
      while (1)
        ;
    }
    CAN.setMode(MCP_NORMAL);
  }

  // Activate play mode to start impedance control at startup
  playMode = true;

  // Create tasks for different functionalities
  xTaskCreatePinnedToCore(readAndComputeTask, "Read and Compute Task", 4096, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(torqueControlTask, "Torque Control Task", 4096, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(checkChiralityTask, "Check Chirality Task", 4096, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(impedanceControlTask, "Impedance Control Task", 4096, NULL, 1, NULL, 1);
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

void calibrateTorqueDirection(String joint) {
  // Remove any prefix such as "n " if it exists
  joint.trim();  // Trimming any leading/trailing spaces
  if (joint.startsWith("n ")) {
    joint = joint.substring(2);  // Remove the "n " prefix
  }

  // Variables for encoder readings
  int encoderBefore, encoderAfter;
  float directionMultiplier = 1.0;
  const int encoderThreshold = 10;  // Threshold to detect minimal movements
  const int maxAttempts = 5;        // Number of attempts to try before reversing torque

  // Get initial encoder reading
  encoderBefore = getEncoderReading(joint);
  Serial.printf("Initial encoder reading for %s: %d\n", joint.c_str(), encoderBefore);

  // Ramp up the positive torque gradually
  for (float testTorque = 10; testTorque <= 150; testTorque += 10) {
    applyTestTorque(joint, testTorque);
    delay(500);  // Short delay to allow for joint movement

    // Get encoder reading after each torque application
    encoderAfter = getEncoderReading(joint);
    Serial.printf("Encoder reading after torque of %.1f for %s: %d\n", testTorque, joint.c_str(), encoderAfter);

    // Check if the joint moved
    if (abs(encoderAfter - encoderBefore) > encoderThreshold) {
      if (encoderAfter > encoderBefore) {
        directionMultiplier = 1.0;  // Positive torque increases encoder value
      } else {
        directionMultiplier = -1.0;  // Positive torque decreases encoder value
      }
      Serial.printf("Torque direction for %s determined with positive torque. Multiplier: %.1f\n", joint.c_str(), directionMultiplier);
      applyTestTorque(joint, 0);  // Apply zero torque to stop the joint
      setDirectionMultiplier(joint, directionMultiplier);
      saveDirectionMultiplierToSPIFFS(joint, directionMultiplier);
      Serial.printf("Torque calibration for %s complete. Multiplier saved to SPIFFS.\n", joint.c_str());
      return;
    }
  }

  // If no movement was detected with positive torque, try ramping up negative torque
  Serial.printf("No movement detected with positive torque on %s. Trying negative torque...\n", joint.c_str());

  encoderBefore = encoderAfter;  // Reset encoder before for negative torque calibration

  for (float testTorque = -10; testTorque >= -150; testTorque -= 10) {
    applyTestTorque(joint, testTorque);
    delay(500);  // Short delay to allow for joint movement

    // Get encoder reading after each torque application
    encoderAfter = getEncoderReading(joint);
    Serial.printf("Encoder reading after torque of %.1f for %s: %d\n", testTorque, joint.c_str(), encoderAfter);

    // Check if the joint moved
    if (abs(encoderAfter - encoderBefore) > encoderThreshold) {
      if (encoderAfter < encoderBefore) {
        directionMultiplier = 1.0;  // Negative torque decreases encoder value (original direction positive)
      } else {
        directionMultiplier = -1.0;  // Negative torque increases encoder value (original direction negative)
      }
      Serial.printf("Torque direction for %s determined with negative torque. Multiplier: %.1f\n", joint.c_str(), directionMultiplier);
      applyTestTorque(joint, 0);  // Apply zero torque to stop the joint
      setDirectionMultiplier(joint, directionMultiplier);
      saveDirectionMultiplierToSPIFFS(joint, directionMultiplier);
      Serial.printf("Torque calibration for %s complete. Multiplier saved to SPIFFS.\n", joint.c_str());
      return;
    }
  }

  // If the joint still does not move, it may be locked
  Serial.printf("Joint %s could not be calibrated after both positive and negative torques. It may be locked.\n", joint.c_str());
  applyTestTorque(joint, 0);  // Remove any applied torque
}

void saveDirectionMultiplierToSPIFFS(String joint, float multiplier) {
  // Open SPIFFS for writing
  File file = SPIFFS.open("/directionMultipliers.txt", FILE_WRITE);
  if (!file) {
    Serial.println("Failed to open file for writing direction multipliers.");
    return;
  }

  // Save the joint name and multiplier
  file.printf("%s:%f\n", joint.c_str(), multiplier);

  file.close();
  Serial.printf("Direction multiplier for %s saved to SPIFFS.\n", joint.c_str());
}

void loadDirectionMultiplierFromSPIFFS() {
  // Open SPIFFS for reading
  if (!SPIFFS.exists("/directionMultipliers.txt")) {
    Serial.println("No direction multiplier file found. Using default multipliers.");
    return;
  }

  File file = SPIFFS.open("/directionMultipliers.txt");
  if (!file) {
    Serial.println("Failed to open direction multipliers file.");
    return;
  }

  // Read each line and parse joint names and multipliers
  while (file.available()) {
    String line = file.readStringUntil('\n');
    int separator = line.indexOf(':');
    if (separator > 0) {
      String joint = line.substring(0, separator);
      float multiplier = line.substring(separator + 1).toFloat();
      setDirectionMultiplier(joint, multiplier);  // Set the loaded multiplier
      Serial.printf("Loaded direction multiplier for %s: %.1f\n", joint.c_str(), multiplier);
    }
  }

  file.close();
}

void applyTestTorque(String joint, float torque) {
  int16_t torqueValue = static_cast<int16_t>(torque);

  // Check joint names and apply the corresponding torque
  if (joint == "right_outer_calf") {
    sendTorqueCommand(ACTUATOR_ID_RIGHT_CALF_OUTER, torqueValue);
  } else if (joint == "right_inner_calf") {
    sendTorqueCommand(ACTUATOR_ID_RIGHT_CALF_INNER, torqueValue);
  } else if (joint == "left_outer_calf") {
    sendTorqueCommand(ACTUATOR_ID_LEFT_CALF_OUTER, torqueValue);
  } else if (joint == "left_inner_calf") {
    sendTorqueCommand(ACTUATOR_ID_LEFT_CALF_INNER, torqueValue);
  } else if (joint == "right_knee") {
    sendTorqueCommand(ACTUATOR_ID_RIGHT_KNEE, torqueValue);
  } else if (joint == "left_knee") {
    sendTorqueCommand(ACTUATOR_ID_LEFT_KNEE, torqueValue);
  } else if (joint == "right_hip_pitch") {
    sendTorqueCommand(ACTUATOR_ID_RIGHT_HIP_PITCH, torqueValue);
  } else if (joint == "left_hip_pitch") {
    sendTorqueCommand(ACTUATOR_ID_LEFT_HIP_PITCH, torqueValue);
  } else if (joint == "right_hip_roll") {
    sendTorqueCommand(ACTUATOR_ID_RIGHT_HIP_ROLL, torqueValue);
  } else if (joint == "left_hip_roll") {
    sendTorqueCommand(ACTUATOR_ID_LEFT_HIP_ROLL, torqueValue);
  } else {
    Serial.println("Invalid joint name in applyTestTorque");
  }
}

// Function to get the encoder reading for each joint using analog pins
int getEncoderReading(String joint) {
  int encoderReading = 0;

  if (joint == "right_outer_calf") {
    encoderReading = analogRead(outerCalf);  // Read from the outer calf sensor (right leg)
  } else if (joint == "right_inner_calf") {
    encoderReading = analogRead(innerCalf);  // Read from the inner calf sensor (right leg)
  } else if (joint == "left_outer_calf") {
    encoderReading = analogRead(outerCalf);  // Read from the outer calf sensor (left leg)
  } else if (joint == "left_inner_calf") {
    encoderReading = analogRead(innerCalf);  // Read from the inner calf sensor (left leg)
  } else if (joint == "right_knee") {
    encoderReading = analogRead(knee);  // Read from the knee sensor (right leg)
  } else if (joint == "left_knee") {
    encoderReading = analogRead(knee);  // Read from the knee sensor (left leg)
  } else if (joint == "right_hip_pitch") {
    encoderReading = analogRead(hip);  // Read from the hip pitch sensor (right leg)
  } else if (joint == "left_hip_pitch") {
    encoderReading = analogRead(hip);  // Read from the hip pitch sensor (left leg)
  } else if (joint == "right_hip_roll") {
    encoderReading = analogRead(butt);  // Read from the butt/hip roll sensor (right leg)
  } else if (joint == "left_hip_roll") {
    encoderReading = analogRead(butt);  // Read from the butt/hip roll sensor (left leg)
  } else {
    Serial.printf("Invalid joint name: %s\n", joint.c_str());
    return -1;  // Invalid joint
  }

  // Optional: Convert the analog reading (0-4096) to a scaled angle if needed (e.g., 0-360 degrees)
  encoderReading = map(encoderReading, 0, 4096, 0, 3600) / 10.0;

  Serial.printf("Encoder reading for %s: %d\n", joint.c_str(), encoderReading);  // Debug print

  return encoderReading;
}

void processSerialCommand(String command) {
  command.trim();

  if (xSemaphoreTake(serialMutex, portMAX_DELAY) == pdTRUE) {
    if (command == "config") {
      enterConfigurationMode();  // Enter config mode directly
    } else if (command.startsWith("calibrateDirection")) {
      String joint = command.substring(17);  // Get the joint name after "calibrateDirection "
      if (joint.length() > 0) {
        calibrateTorqueDirection(joint);
      } else {
        Serial.println("Invalid joint name for direction calibration.");
      }
    } else if (command == "resetOffsets") {
      resetOffsets();  // Reset all offsets to zero
    } else if (command == "raw on") {
      rawMode = true;
      Serial.println("Raw mode enabled. Offsets are bypassed.");
    } else if (command == "raw off") {
      rawMode = false;
      Serial.println("Raw mode disabled. Offsets are applied.");
    } else if (command.startsWith("direction")) {
      String params = command.substring(9);  // Get parameters after "direction "
      int firstSpace = params.indexOf(' ');
      if (firstSpace > 0) {
        String joint = params.substring(0, firstSpace);
        String direction = params.substring(firstSpace + 1);

        // Determine direction multiplier based on input
        float multiplier = 1.0;
        if (direction == "-") {
          multiplier = -1.0;
        } else if (direction == "+") {
          multiplier = 1.0;
        } else {
          Serial.println("Invalid direction format. Use + or -.");
          return;
        }

        // Set direction multiplier for the specified joint
        setDirectionMultiplier(joint, multiplier);
        Serial.printf("Direction for %s set to %s (%.1f)\n", joint.c_str(), direction.c_str(), multiplier);

        // Save the updated configuration
        saveConfig();  // <-- Added this line to save the changes to SPIFFS
      } else {
        Serial.println("Invalid direction command format.");
      }
    } else if (command == "help") {
      printHelp();  // Call the help function
    } else if (command.startsWith("impedance")) {
      String params = command.substring(10);  // Get parameters after "impedance "
      int firstSpace = params.indexOf(' ');
      int secondSpace = params.indexOf(' ', firstSpace + 1);
      int thirdSpace = params.indexOf(' ', secondSpace + 1);

      if (firstSpace > 0 && secondSpace > firstSpace && thirdSpace > secondSpace) {
        String legSide = params.substring(0, firstSpace);
        String appendage = params.substring(firstSpace + 1, secondSpace);
        bool enable = params.substring(secondSpace + 1, thirdSpace).toInt();  // 1 to enable, 0 to disable
        float desiredPosition = params.substring(thirdSpace + 1).toFloat();   // Desired position
        float desiredVelocity = params.substring(thirdSpace + 2).toFloat();   // Desired velocity

        // Enable/disable impedance for the specified joint and set desired position and velocity
        ImpedanceControl *control = nullptr;
        if (legSide == "left") {
          if (appendage == "outer_calf") {
            impedanceEnabledLeftOuterCalf = enable;
            control = &outerCalfControlLeft;
          } else if (appendage == "inner_calf") {
            impedanceEnabledLeftInnerCalf = enable;
            control = &innerCalfControlLeft;
          } else if (appendage == "knee") {
            impedanceEnabledLeftKnee = enable;
            control = &kneeControlLeft;
          } else if (appendage == "hip_pitch") {
            impedanceEnabledLeftHipPitch = enable;
            control = &hipPitchControlLeft;
          } else if (appendage == "hip_roll") {
            impedanceEnabledLeftHipRoll = enable;
            control = &hipRollControlLeft;
          }
        } else if (legSide == "right") {
          if (appendage == "outer_calf") {
            impedanceEnabledRightOuterCalf = enable;
            control = &outerCalfControlRight;
          } else if (appendage == "inner_calf") {
            impedanceEnabledRightInnerCalf = enable;
            control = &innerCalfControlRight;
          } else if (appendage == "knee") {
            impedanceEnabledRightKnee = enable;
            control = &kneeControlRight;
          } else if (appendage == "hip_pitch") {
            impedanceEnabledRightHipPitch = enable;
            control = &hipPitchControlRight;
          } else if (appendage == "hip_roll") {
            impedanceEnabledRightHipRoll = enable;
            control = &hipRollControlRight;
          }
        }

        if (control != nullptr) {
          if (enable) {
            control->setDesiredPosition(desiredPosition);
            control->setDesiredVelocity(desiredVelocity);
          }
          Serial.printf("Impedance for %s %s set to %d, position: %.2f, velocity: %.2f\n",
                        legSide.c_str(), appendage.c_str(), enable, desiredPosition, desiredVelocity);
        } else {
          Serial.println("Invalid appendage specified.");
        }
      } else {
        Serial.println("Invalid impedance command format.");
      }
    } else if (command == "resetSPIFFS") {
      resetSPIFFS();
    } else if (command.startsWith("constrain")) {  // Handle constrain command
      constrainJoint(command);
    } else if (command.startsWith("torque")) {
      String params = command.substring(7);  // Get parameters after "torque "
      int firstSpace = params.indexOf(' ');
      int secondSpace = params.indexOf(' ', firstSpace + 1);

      if (firstSpace > 0 && secondSpace > firstSpace) {
        String legSide = params.substring(0, firstSpace);
        String appendage = params.substring(firstSpace + 1, secondSpace);
        int torqueValue = params.substring(secondSpace + 1).toInt();

        // Apply the maximum torque limit
        torqueValue = constrain(torqueValue, -maxTorqueLimit * 100, maxTorqueLimit * 100);

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
    } else if (command == "setJointConstraints") {
      configureJointConstraintsViaSerial();
    }
    xSemaphoreGive(serialMutex);
  }
}

void constrainJoint(String command) {
  // Parse the command "constrain <jointname> <minval> <maxval>"
  int firstSpace = command.indexOf(' ');
  int secondSpace = command.indexOf(' ', firstSpace + 1);
  int thirdSpace = command.indexOf(' ', secondSpace + 1);

  if (firstSpace == -1 || secondSpace == -1 || thirdSpace == -1) {
    Serial.println("Invalid constrain command format. Expected: constrain <jointname> <minval> <maxval>");
    return;
  }

  String jointName = command.substring(firstSpace + 1, secondSpace);
  int minVal = command.substring(secondSpace + 1, thirdSpace).toInt();
  int maxVal = command.substring(thirdSpace + 1).toInt();

  setJointConstraints(jointName, minVal, maxVal);
}

void saveConfig() {
  File file = SPIFFS.open("/config.txt", FILE_WRITE);
  if (!file) {
    Serial.println("Failed to open file for writing");
    return;
  }

  // Save leg side and offsets
  file.printf("LegSide:%s\n", isCenter ? "center" : (isLeft ? "left" : "right"));
  file.printf("LeftOffsets:%d,%d,%d,%d,%d\n", leftLegOffsets[0], leftLegOffsets[1], leftLegOffsets[2], leftLegOffsets[3], leftLegOffsets[4]);
  file.printf("RightOffsets:%d,%d,%d,%d,%d\n", rightLegOffsets[0], rightLegOffsets[1], rightLegOffsets[2], rightLegOffsets[3], rightLegOffsets[4]);

  // Save direction multipliers
  file.printf("DirectionMultipliers:%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n",
              directionMultiplierRightOuterCalf, directionMultiplierRightInnerCalf,
              directionMultiplierLeftOuterCalf, directionMultiplierLeftInnerCalf,
              directionMultiplierRightKnee, directionMultiplierLeftKnee,
              directionMultiplierRightHipPitch, directionMultiplierLeftHipPitch,
              directionMultiplierRightHipRoll, directionMultiplierLeftHipRoll);

  // Save joint constraints
  saveJointConstraintsToFile(file, outerCalfConstraintsLeft, "outer_calf_left_Constraints");
  saveJointConstraintsToFile(file, outerCalfConstraintsRight, "outer_calf_right_Constraints");
  saveJointConstraintsToFile(file, innerCalfConstraintsLeft, "inner_calf_left_Constraints");
  saveJointConstraintsToFile(file, innerCalfConstraintsRight, "inner_calf_right_Constraints");
  saveJointConstraintsToFile(file, kneeConstraintsLeft, "knee_left_Constraints");
  saveJointConstraintsToFile(file, kneeConstraintsRight, "knee_right_Constraints");
  saveJointConstraintsToFile(file, hipPitchConstraintsLeft, "hip_pitch_left_Constraints");
  saveJointConstraintsToFile(file, hipPitchConstraintsRight, "hip_pitch_right_Constraints");
  saveJointConstraintsToFile(file, hipYawConstraintsLeft, "hip_yaw_left_Constraints");
  saveJointConstraintsToFile(file, hipYawConstraintsRight, "hip_yaw_right_Constraints");
  saveJointConstraintsToFile(file, hipRollConstraintsLeft, "hip_roll_left_Constraints");
  saveJointConstraintsToFile(file, hipRollConstraintsRight, "hip_roll_right_Constraints");

  file.close();
  Serial.println("Configuration saved.");
}


void loadConfig() {
  if (!SPIFFS.exists("/config.txt")) {
    // If no config file exists, use hardcoded defaults and prompt for leg side
    Serial.println("No configuration file found. Using hardcoded offsets and constraints.");
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

  // Load direction multipliers
  line = file.readStringUntil('\n');
  if (line.startsWith("DirectionMultipliers:")) {
    sscanf(line.c_str(), "DirectionMultipliers:%f,%f,%f,%f,%f,%f,%f,%f,%f,%f",
           &directionMultiplierRightOuterCalf, &directionMultiplierRightInnerCalf,
           &directionMultiplierLeftOuterCalf, &directionMultiplierLeftInnerCalf,
           &directionMultiplierRightKnee, &directionMultiplierLeftKnee,
           &directionMultiplierRightHipPitch, &directionMultiplierLeftHipPitch,
           &directionMultiplierRightHipRoll, &directionMultiplierLeftHipRoll);
  } else {
    Serial.println("No direction multipliers found in SPIFFS, using hardcoded direction multipliers.");
  }

  // Load joint constraints
  outerCalfConstraintsLeft = loadJointConstraintsFromFile(file.readStringUntil('\n'), outerCalfConstraintsLeft);
  outerCalfConstraintsRight = loadJointConstraintsFromFile(file.readStringUntil('\n'), outerCalfConstraintsRight);
  innerCalfConstraintsLeft = loadJointConstraintsFromFile(file.readStringUntil('\n'), innerCalfConstraintsLeft);
  innerCalfConstraintsRight = loadJointConstraintsFromFile(file.readStringUntil('\n'), innerCalfConstraintsRight);
  kneeConstraintsLeft = loadJointConstraintsFromFile(file.readStringUntil('\n'), kneeConstraintsLeft);
  kneeConstraintsRight = loadJointConstraintsFromFile(file.readStringUntil('\n'), kneeConstraintsRight);
  hipPitchConstraintsLeft = loadJointConstraintsFromFile(file.readStringUntil('\n'), hipPitchConstraintsLeft);
  hipPitchConstraintsRight = loadJointConstraintsFromFile(file.readStringUntil('\n'), hipPitchConstraintsRight);
  hipYawConstraintsLeft = loadJointConstraintsFromFile(file.readStringUntil('\n'), hipYawConstraintsLeft);
  hipYawConstraintsRight = loadJointConstraintsFromFile(file.readStringUntil('\n'), hipYawConstraintsRight);
  hipRollConstraintsLeft = loadJointConstraintsFromFile(file.readStringUntil('\n'), hipRollConstraintsLeft);
  hipRollConstraintsRight = loadJointConstraintsFromFile(file.readStringUntil('\n'), hipRollConstraintsRight);

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

// Function to reset all offsets
void resetOffsets() {
  // Reset all leg offsets to 0
  for (int i = 0; i < 5; i++) {
    leftLegOffsets[i] = 0;
    rightLegOffsets[i] = 0;
  }
  Serial.println("Offsets reset to zero.");
  saveConfig();  // Save the reset offsets to the config
}

// Modify normalizeReadings to consider raw mode
void normalizeReadings() {
  if (rawMode) {
    // In raw mode, bypass offsets and use raw sensor values directly
    normalizedOuter = map(averageOuter, 0, 4096, 0, 3600) / 10.0;
    normalizedInner = map(averageInner, 0, 4096, 0, 3600) / 10.0;
    normalizedHip = map(averageHip, 0, 4096, 0, 3600) / 10.0;
    normalizedKnee = map(averageKnee, 0, 4096, 0, 3600) / 10.0;
    normalizedButt = map(averageButt, 0, 4096, 0, 3600) / 10.0;
    Serial.println("Using raw sensor values, no offsets applied.");
  } else {
    // Apply offsets for each reading
    const int *offsets = isLeft ? leftLegOffsets : rightLegOffsets;  // Use the appropriate offsets array
    normalizedOuter = wrapAngleFloat(map(averageOuter, 0, 4096, 0, 3600) / 10.0 + offsets[0]);
    normalizedInner = wrapAngleFloat(map(averageInner, 0, 4096, 0, 3600) / 10.0 + offsets[1]);
    normalizedHip = wrapAngleFloat(map(averageHip, 0, 4096, 0, 3600) / 10.0 + offsets[2]);
    normalizedKnee = wrapAngleFloat(map(averageKnee, 0, 4096, 0, 3600) / 10.0 + offsets[3]);
    normalizedButt = wrapAngleFloat(map(averageButt, 0, 4096, 0, 3600) / 10.0 + offsets[4]);
  }
}

float wrapAngleFloat(float angle) {
  if (angle >= 360.0) return angle - 360.0;
  if (angle < 0.0) return angle + 360.0;
  return angle;
}

void calibrateSensors() {
  float rawOuter = map(averageOuter, 0, 4096, 0, 3600) / 10.0;
  float rawInner = map(averageInner, 0, 4096, 0, 3600) / 10.0;
  float rawHip = map(averageHip, 0, 4096, 0, 3600) / 10.0;
  float rawKnee = map(averageKnee, 0, 4096, 0, 3600) / 10.0;
  float rawButt = map(averageButt, 0, 4096, 0, 3600) / 10.0;

  int offsetOuter = (rawOuter > 180.0) ? 180.0 - rawOuter : 180.0 - rawOuter;
  int offsetInner = (rawInner > 180.0) ? 180.0 - rawInner : 180.0 - rawInner;
  int offsetHip = (rawHip > 180.0) ? 180.0 - rawHip : 180.0 - rawHip;
  int offsetKnee = (rawKnee > 180.0) ? 180.0 - rawKnee : 180.0 - rawKnee;
  int offsetButt = (rawButt > 180.0) ? 180.0 - rawButt : 180.0 - rawButt;

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

  Serial.println("Calibration complete. Offsets to align readings to 180 degrees:");
  Serial.printf("Outer Offset: %d\n", offsetOuter);
  Serial.printf("Inner Offset: %d\n", offsetInner);
  Serial.printf("Hip Offset: %d\n", offsetHip);
  Serial.printf("Knee Offset: %d\n", offsetKnee);
  Serial.printf("Butt Offset: %d\n", offsetButt);

  Serial.println("Do you want to save these offsets? Type 'yes' to save, 'no' to discard.");

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

void printReadings() {
  if (xSemaphoreTake(serialMutex, portMAX_DELAY) == pdTRUE) {
    Serial.print(normalizedOuter, 1);
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

  while (!Serial.available()) {
    delay(10);
  }

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

void printMACAddress() {
  uint8_t mac[6];
  WiFi.macAddress(mac);
  Serial.printf("MAC Address: %02X:%02X:%02X:%02X:%02X:%02X\n", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}

void sendTorqueCommand(long unsigned int actuatorID, int16_t torqueValue) {
  byte buf[8] = { 0xA1, 0x00, 0x00, 0x00, (byte)(torqueValue & 0xFF), (byte)(torqueValue >> 8), 0x00, 0x00 };
  CAN.sendMsgBuf(actuatorID, 0, 8, buf);
}

void sendStopCommand(long unsigned int actuatorID) {
  byte buf[8] = { 0x81, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
  CAN.sendMsgBuf(actuatorID, 0, 8, buf);
}

void readIMU() {
  for (int i = 0; i < IMU_COUNT; i++) {
    Wire.beginTransmission(0x68 + i);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(0x68 + i, 14, true);

    int16_t ax = Wire.read() << 8 | Wire.read();
    int16_t ay = Wire.read() << 8 | Wire.read();
    int16_t az = Wire.read() << 8 | Wire.read();
    int16_t gx = Wire.read() << 8 | Wire.read();
    int16_t gy = Wire.read() << 8 | Wire.read();
    int16_t gz = Wire.read() << 8 | Wire.read();

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

void setJointConstraints(String jointName, int minAngle, int maxAngle) {
  if (jointName == "outer_calf_left") {
    outerCalfConstraintsLeft.minAngle = minAngle;
    outerCalfConstraintsLeft.maxAngle = maxAngle;
  } else if (jointName == "outer_calf_right") {
    outerCalfConstraintsRight.minAngle = minAngle;
    outerCalfConstraintsRight.maxAngle = maxAngle;
  } else if (jointName == "inner_calf_left") {
    innerCalfConstraintsLeft.minAngle = minAngle;
    innerCalfConstraintsLeft.maxAngle = maxAngle;
  } else if (jointName == "inner_calf_right") {
    innerCalfConstraintsRight.minAngle = minAngle;
    innerCalfConstraintsRight.maxAngle = maxAngle;
  } else if (jointName == "knee_left") {
    kneeConstraintsLeft.minAngle = minAngle;
    kneeConstraintsLeft.maxAngle = maxAngle;
  } else if (jointName == "knee_right") {
    kneeConstraintsRight.minAngle = minAngle;
    kneeConstraintsRight.maxAngle = maxAngle;
  } else if (jointName == "hip_pitch_left") {
    hipPitchConstraintsLeft.minAngle = minAngle;
    hipPitchConstraintsLeft.maxAngle = maxAngle;
  } else if (jointName == "hip_pitch_right") {
    hipPitchConstraintsRight.minAngle = minAngle;
    hipPitchConstraintsRight.maxAngle = maxAngle;
  } else if (jointName == "hip_yaw_left") {
    hipYawConstraintsLeft.minAngle = minAngle;
    hipYawConstraintsLeft.maxAngle = maxAngle;
  } else if (jointName == "hip_yaw_right") {
    hipYawConstraintsRight.minAngle = minAngle;
    hipYawConstraintsRight.maxAngle = maxAngle;
  } else if (jointName == "hip_roll_left") {
    hipRollConstraintsLeft.minAngle = minAngle;
    hipRollConstraintsLeft.maxAngle = maxAngle;
  } else if (jointName == "hip_roll_right") {
    hipRollConstraintsRight.minAngle = minAngle;
    hipRollConstraintsRight.maxAngle = maxAngle;
  }
  saveConfig();
  Serial.printf("Joint constraints for %s set to min: %d, max: %d\n", jointName.c_str(), minAngle, maxAngle);
}

void configureJointConstraintsViaSerial() {
  Serial.println("Enter joint name:");
  while (!Serial.available()) delay(10);
  String jointName = Serial.readStringUntil('\n');
  jointName.trim();

  Serial.println("Enter min angle:");
  while (!Serial.available()) delay(10);
  int minAngle = Serial.parseInt();

  Serial.println("Enter max angle:");
  while (!Serial.available()) delay(10);
  int maxAngle = Serial.parseInt();

  setJointConstraints(jointName, minAngle, maxAngle);
}

JointConstraints loadJointConstraintsFromFile(String line, JointConstraints defaultConstraints) {
  JointConstraints constraints = defaultConstraints;
  if (line.startsWith("Constraints:")) {
    sscanf(line.c_str(), "Constraints:%d,%d", &constraints.minAngle, &constraints.maxAngle);
  }
  return constraints;
}

void saveJointConstraintsToFile(File &file, JointConstraints constraints, const char *jointName) {
  file.printf("%s Constraints:%d,%d\n", jointName, constraints.minAngle, constraints.maxAngle);
}

void printHelp() {
  Serial.println("Available Commands:");
  
  Serial.println("1. config");
  Serial.println("   Enter configuration mode.");
  Serial.println("   Usage: config");
  
  Serial.println("2. resetOffsets");
  Serial.println("   Reset all offsets to zero.");
  Serial.println("   Usage: resetOffsets");
  
  Serial.println("3. raw");
  Serial.println("   Enable or disable raw mode (bypass offsets).");
  Serial.println("   Usage: raw on / raw off");

  Serial.println("4. direction");
  Serial.println("   Set direction multiplier for the specified joint.");
  Serial.println("   Usage: direction <joint> <+> or <->");
  Serial.println("   Example: direction right_outer_calf +");
  
  Serial.println("5. impedance");
  Serial.println("   Enable or disable impedance control for a joint and set desired position and velocity.");
  Serial.println("   Usage: impedance <legSide> <appendage> <enable (1 or 0)> <desiredPosition> <desiredVelocity>");
  Serial.println("   Example: impedance left knee 1 180 0");

  Serial.println("6. constrain");
  Serial.println("   Set joint angle constraints for the specified joint.");
  Serial.println("   Usage: constrain <joint> <minAngle> <maxAngle>");
  Serial.println("   Example: constrain right_knee 0 180");

  Serial.println("7. torque");
  Serial.println("   Set torque value for the specified joint.");
  Serial.println("   Usage: torque <legSide> <appendage> <torqueValue>");
  Serial.println("   Example: torque left knee 100");

  Serial.println("8. calibrateDirection");
  Serial.println("   Calibrate torque direction for the specified joint.");
  Serial.println("   Valid joints:");
  Serial.println("     right_outer_calf, right_inner_calf, left_outer_calf, left_inner_calf");
  Serial.println("     right_knee, left_knee, right_hip_pitch, left_hip_pitch");
  Serial.println("     right_hip_roll, left_hip_roll");
  Serial.println("   Usage: calibrateDirection <joint>");
  Serial.println("   Example: calibrateDirection right_outer_calf");

  Serial.println("9. mac");
  Serial.println("   Print the device's MAC address.");
  Serial.println("   Usage: mac");

  Serial.println("10. chirality");
  Serial.println("   Print the current chirality setting (left, right, center).");
  Serial.println("   Usage: chirality");

  Serial.println("11. calibrate");
  Serial.println("   Calibrate sensors and set offsets for joints.");
  Serial.println("   Usage: calibrate");

  Serial.println("12. save");
  Serial.println("   Save the current configuration to SPIFFS.");
  Serial.println("   Usage: save");

  Serial.println("13. resetSPIFFS");
  Serial.println("   Reset or format SPIFFS file system.");
  Serial.println("   Usage: resetSPIFFS");

  Serial.println("14. play");
  Serial.println("   Enable play mode (start impedance control).");
  Serial.println("   Usage: play");

  Serial.println("15. stop");
  Serial.println("   Disable play mode (stop all actuators).");
  Serial.println("   Usage: stop");

  Serial.println("16. setJointConstraints");
  Serial.println("   Set joint constraints interactively via serial.");
  Serial.println("   Usage: setJointConstraints");
  
  Serial.println("17. help");
  Serial.println("   Print this help menu.");
  Serial.println("   Usage: help");
}
