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

/*

 * Dropbear ESP32 low-level leg controller
   *
 * Updated architecture:
 * - Preserves existing ESP32/MCP2515/analog pinout.
 * - Preserves MyActuator CAN IDs 0x141..0x14C.
 * - Preserves A1 torque-control and 0x81 stop commands.
 * - Uses one periodic CAN-output task as the normal actuator writer.
 * - Impedance task computes desired torque only; it does not transmit CAN.
 * - Each leg controller transmits only to its selected chirality (6 motors).
 * - Correct stop commands use actual actuator CAN IDs.
 * - Calibration uses an output override instead of racing the CAN task.
 * - Fixes impedance serial parsing and joint-constraint persistence.
 * - Starts the IMU task in center mode.
 * - Sensor acquisition remains 1 kHz; serial telemetry is throttled to 50 Hz.
     */

// -----------------------------------------------------------------------------
// Hardware
// -----------------------------------------------------------------------------

#define I2C_SDA_PIN 21
#define I2C_SCL_PIN 22
#define CAN_CS_PIN 5
#define CAN0_INT GPIO_NUM_17

#define SPI_SCK_PIN 18
#define SPI_MISO_PIN 19
#define SPI_MOSI_PIN 23

#define IMU_COUNT 5

MCP_CAN CAN(CAN_CS_PIN);

// Analog joint sensors (existing Dropbear pinout)
static const int PIN_OUTER_CALF = 14;
static const int PIN_INNER_CALF = 27;
static const int PIN_HIP_PITCH = 26;
static const int PIN_KNEE = 25;
static const int PIN_HIP_ROLL = 33;

// -----------------------------------------------------------------------------
// Actuator mapping
// Index ordering intentionally preserves the original torqueValues[] mapping:
// even indexes = right leg, odd indexes = left leg.
// -----------------------------------------------------------------------------

enum ActuatorIndex : uint8_t {
  RIGHT_OUTER_CALF = 0,
  LEFT_OUTER_CALF = 1,
  RIGHT_INNER_CALF = 2,
  LEFT_INNER_CALF = 3,
  RIGHT_KNEE = 4,
  LEFT_KNEE = 5,
  RIGHT_HIP_PITCH = 6,
  LEFT_HIP_PITCH = 7,
  RIGHT_HIP_YAW = 8,
  LEFT_HIP_YAW = 9,
  RIGHT_HIP_ROLL = 10,
  LEFT_HIP_ROLL = 11,
  ACTUATOR_COUNT = 12
};

static const uint32_t ACTUATOR_IDS[ACTUATOR_COUNT] = {
  0x144, // right outer calf
  0x141, // left outer calf
  0x143, // right inner calf
  0x142, // left inner calf
  0x148, // right knee
  0x145, // left knee
  0x147, // right hip pitch
  0x146, // left hip pitch
  0x14C, // right hip yaw
  0x149, // left hip yaw
  0x14B, // right hip roll
  0x14A  // left hip roll
};

// Backward-compatible named constants.
const unsigned long ACTUATOR_ID_RIGHT_CALF_OUTER = ACTUATOR_IDS[RIGHT_OUTER_CALF];
const unsigned long ACTUATOR_ID_LEFT_CALF_OUTER = ACTUATOR_IDS[LEFT_OUTER_CALF];
const unsigned long ACTUATOR_ID_RIGHT_CALF_INNER = ACTUATOR_IDS[RIGHT_INNER_CALF];
const unsigned long ACTUATOR_ID_LEFT_CALF_INNER = ACTUATOR_IDS[LEFT_INNER_CALF];
const unsigned long ACTUATOR_ID_RIGHT_KNEE = ACTUATOR_IDS[RIGHT_KNEE];
const unsigned long ACTUATOR_ID_LEFT_KNEE = ACTUATOR_IDS[LEFT_KNEE];
const unsigned long ACTUATOR_ID_RIGHT_HIP_PITCH = ACTUATOR_IDS[RIGHT_HIP_PITCH];
const unsigned long ACTUATOR_ID_LEFT_HIP_PITCH = ACTUATOR_IDS[LEFT_HIP_PITCH];
const unsigned long ACTUATOR_ID_RIGHT_HIP_YAW = ACTUATOR_IDS[RIGHT_HIP_YAW];
const unsigned long ACTUATOR_ID_LEFT_HIP_YAW = ACTUATOR_IDS[LEFT_HIP_YAW];
const unsigned long ACTUATOR_ID_RIGHT_HIP_ROLL = ACTUATOR_IDS[RIGHT_HIP_ROLL];
const unsigned long ACTUATOR_ID_LEFT_HIP_ROLL = ACTUATOR_IDS[LEFT_HIP_ROLL];

// -----------------------------------------------------------------------------
// Shared state
// -----------------------------------------------------------------------------

struct JointConstraints {
  int minAngle = 0;
  int maxAngle = 360;
};

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

// Manual/direct torque setpoints, in the same 0.01 A-ish command units used by
// the existing A1 implementation. maxTorqueLimit=3.0 => +/-300 command units.
volatile int16_t torqueValues[ACTUATOR_COUNT] = {0};
volatile int16_t impedanceTorqueValues[ACTUATOR_COUNT] = {0};

float maxTorqueLimit = 3.0f;

bool isLeft = true;
bool isCenter = false;
bool rawMode = false;
bool playMode = false;
bool configMode = false;

// Stop burst is handled by the CAN output task. Three repeated 0x81 frames are
// used to retain the intent of the previous enterConfigurationMode() behavior.
volatile uint8_t stopBurstRemaining = 0;

// Calibration output override. This keeps calibration from racing against the
// normal torque/impedance output path.
volatile bool calibrationOverrideActive = false;
volatile int calibrationActuatorIndex = -1;
volatile int16_t calibrationTorqueValue = 0;

SemaphoreHandle_t serialMutex = nullptr;
SemaphoreHandle_t canMutex = nullptr;
SemaphoreHandle_t stateMutex = nullptr;

// -----------------------------------------------------------------------------
// Sensor filtering / state
// -----------------------------------------------------------------------------

static const int NUM_READINGS = 10;
int readingsOuter[NUM_READINGS] = {0};
int readingsInner[NUM_READINGS] = {0};
int readingsHip[NUM_READINGS] = {0};
int readingsKnee[NUM_READINGS] = {0};
int readingsButt[NUM_READINGS] = {0};

long totalOuter = 0;
long totalInner = 0;
long totalHip = 0;
long totalKnee = 0;
long totalButt = 0;

float averageOuter = 0.0f;
float averageInner = 0.0f;
float averageHip = 0.0f;
float averageKnee = 0.0f;
float averageButt = 0.0f;

volatile float normalizedOuter = 0.0f;
volatile float normalizedInner = 0.0f;
volatile float normalizedHip = 0.0f;
volatile float normalizedKnee = 0.0f;
volatile float normalizedButt = 0.0f;

int readIndex = 0;

// Original known offsets retained.
int leftLegOffsets[5] = {32, -26, -4, -17, 2};
int rightLegOffsets[5] = {-28, 39, -2, 18, -2};

// -----------------------------------------------------------------------------
// Direction multipliers
// -----------------------------------------------------------------------------

float directionMultiplierRightOuterCalf = 1.0f;
float directionMultiplierRightInnerCalf = 1.0f;
float directionMultiplierLeftOuterCalf = 1.0f;
float directionMultiplierLeftInnerCalf = 1.0f;
float directionMultiplierRightKnee = 1.0f;
float directionMultiplierLeftKnee = 1.0f;
float directionMultiplierRightHipPitch = 1.0f;
float directionMultiplierLeftHipPitch = 1.0f;
float directionMultiplierRightHipRoll = 1.0f;
float directionMultiplierLeftHipRoll = 1.0f;

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

// -----------------------------------------------------------------------------
// Utility helpers
// -----------------------------------------------------------------------------

float adcToDegrees(float adc) {
  // Keep 4096.0 scaling to remain very close to the original map(...4096...).
  return adc * (360.0f / 4096.0f);
}

float wrapAngleFloat(float angle) {
  angle = fmodf(angle, 360.0f);
  if (angle < 0.0f) angle += 360.0f;
  return angle;
}

int wrapAngle(int angle) {
  angle %= 360;
  if (angle < 0) angle += 360;
  return angle;
}

int16_t clampTorqueCommand(float value) {
  const float limit = maxTorqueLimit * 100.0f;
  if (value > limit) value = limit;
  if (value < -limit) value = -limit;
  return static_cast<int16_t>(lroundf(value));
}

bool actuatorBelongsToSelectedLeg(int index) {
  if (index < 0 || index >= ACTUATOR_COUNT || isCenter) return false;
  return isLeft ? ((index & 1) == 1) : ((index & 1) == 0);
}

int firstSelectedActuatorIndex() {
  return isLeft ? 1 : 0;
}

// -----------------------------------------------------------------------------
// Impedance controller
// -----------------------------------------------------------------------------

class ImpedanceControl {
public:
  float damping;
  float stiffness;
  float mass;
  float desiredPosition;
  float desiredVelocity;
  float torqueOutput;

  ImpedanceControl(float dampingIn, float stiffnessIn, float massIn)
      : damping(dampingIn), stiffness(stiffnessIn), mass(massIn),
        desiredPosition(180.0f), desiredVelocity(0.0f), torqueOutput(0.0f),
        initialized(false), lastTime(0), lastPosition(0.0f), lastVelocity(0.0f) {}

  void reset(float actualPosition = 0.0f) {
    initialized = false;
    lastTime = 0;
    lastPosition = actualPosition;
    lastVelocity = 0.0f;
    torqueOutput = 0.0f;
  }

  void update(float actualPosition, unsigned long currentTime, int minAngle, int maxAngle) {
    if (actualPosition < static_cast<float>(minAngle) || actualPosition > static_cast<float>(maxAngle)) {
      torqueOutput = 0.0f;
      reset(actualPosition);
      return;
    }

    if (!initialized) {
      initialized = true;
      lastTime = currentTime;
      lastPosition = actualPosition;
      lastVelocity = 0.0f;
      torqueOutput = 0.0f;
      return;
    }
    
    const float deltaTime = static_cast<float>(currentTime - lastTime) / 1000.0f;
    if (deltaTime <= 0.0001f || deltaTime > 0.25f) {
      lastTime = currentTime;
      lastPosition = actualPosition;
      lastVelocity = 0.0f;
      torqueOutput = 0.0f;
      return;
    }
    
    const float actualVelocity = (actualPosition - lastPosition) / deltaTime;
    const float actualAcceleration = (actualVelocity - lastVelocity) / deltaTime;
    const float positionError = desiredPosition - actualPosition;
    const float velocityError = desiredVelocity - actualVelocity;
    
    // Preserve the original controller law. Saturation happens before output.
    torqueOutput = stiffness * positionError +
                   damping * velocityError +
                   mass * actualAcceleration;
    
    const float limit = maxTorqueLimit * 100.0f;
    if (torqueOutput > limit) torqueOutput = limit;
    if (torqueOutput < -limit) torqueOutput = -limit;
    
    lastPosition = actualPosition;
    lastVelocity = actualVelocity;
    lastTime = currentTime;

  }

  void setDesiredPosition(float position) { desiredPosition = position; }
  void setDesiredVelocity(float velocity) { desiredVelocity = velocity; }

private:
  bool initialized;
  unsigned long lastTime;
  float lastPosition;
  float lastVelocity;
};

ImpedanceControl outerCalfControlRight(2.5f, 50.0f, 0.8f);
ImpedanceControl outerCalfControlLeft(2.5f, 50.0f, 0.8f);
ImpedanceControl innerCalfControlRight(2.5f, 50.0f, 0.8f);
ImpedanceControl innerCalfControlLeft(2.5f, 50.0f, 0.8f);
ImpedanceControl kneeControlRight(3.0f, 60.0f, 3.55f);
ImpedanceControl kneeControlLeft(3.0f, 60.0f, 3.55f);
ImpedanceControl hipPitchControlRight(3.5f, 80.0f, 9.05f);
ImpedanceControl hipPitchControlLeft(3.5f, 80.0f, 9.05f);
ImpedanceControl hipRollControlRight(3.5f, 80.0f, 9.05f);
ImpedanceControl hipRollControlLeft(3.5f, 80.0f, 9.05f);

// -----------------------------------------------------------------------------
// CAN primitives
// -----------------------------------------------------------------------------

bool canSend(uint32_t actuatorID, const byte data[8]) {
  if (isCenter) return false;
  if (canMutex == nullptr) return false;

  bool ok = false;
  if (xSemaphoreTake(canMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
    ok = (CAN.sendMsgBuf(actuatorID, 0, 8, const_cast<byte *>(data)) == CAN_OK);
    xSemaphoreGive(canMutex);
  }
  return ok;
}

void sendTorqueCommand(unsigned long actuatorID, int16_t torqueValue) {
  byte buf[8] = {
    0xA1, 0x00, 0x00, 0x00,
    static_cast<byte>(torqueValue & 0xFF),
    static_cast<byte>((static_cast<uint16_t>(torqueValue) >> 8) & 0xFF),
    0x00, 0x00
  };
  canSend(actuatorID, buf);
}

void sendStopCommand(unsigned long actuatorID) {
  byte buf[8] = {0x81, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  canSend(actuatorID, buf);
}

void requestStop(uint8_t repeats = 3) {
  playMode = false;
  stopBurstRemaining = repeats;
}

void clearAllTorqueSetpoints() {
  if (stateMutex && xSemaphoreTake(stateMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
    for (int i = 0; i < ACTUATOR_COUNT; ++i) {
      torqueValues[i] = 0;
      impedanceTorqueValues[i] = 0;
    }
    xSemaphoreGive(stateMutex);
  }
}

bool isImpedanceEnabled(int index) {
  switch (index) {
    case RIGHT_OUTER_CALF: return impedanceEnabledRightOuterCalf;
    case LEFT_OUTER_CALF: return impedanceEnabledLeftOuterCalf;
    case RIGHT_INNER_CALF: return impedanceEnabledRightInnerCalf;
    case LEFT_INNER_CALF: return impedanceEnabledLeftInnerCalf;
    case RIGHT_KNEE: return impedanceEnabledRightKnee;
    case LEFT_KNEE: return impedanceEnabledLeftKnee;
    case RIGHT_HIP_PITCH: return impedanceEnabledRightHipPitch;
    case LEFT_HIP_PITCH: return impedanceEnabledLeftHipPitch;
    case RIGHT_HIP_ROLL: return impedanceEnabledRightHipRoll;
    case LEFT_HIP_ROLL: return impedanceEnabledLeftHipRoll;
    default: return false; // hip yaw has no external impedance sensor in this firmware
  }
}

// -----------------------------------------------------------------------------
// Sensor functions
// -----------------------------------------------------------------------------

void readSensors() {
  totalOuter -= readingsOuter[readIndex];
  totalInner -= readingsInner[readIndex];
  totalHip -= readingsHip[readIndex];
  totalKnee -= readingsKnee[readIndex];
  totalButt -= readingsButt[readIndex];

  readingsOuter[readIndex] = analogRead(PIN_OUTER_CALF);
  readingsInner[readIndex] = analogRead(PIN_INNER_CALF);
  readingsHip[readIndex] = analogRead(PIN_HIP_PITCH);
  readingsKnee[readIndex] = analogRead(PIN_KNEE);
  readingsButt[readIndex] = analogRead(PIN_HIP_ROLL);

  totalOuter += readingsOuter[readIndex];
  totalInner += readingsInner[readIndex];
  totalHip += readingsHip[readIndex];
  totalKnee += readingsKnee[readIndex];
  totalButt += readingsButt[readIndex];

  readIndex = (readIndex + 1) % NUM_READINGS;
}

void computeAverages() {
  averageOuter = static_cast<float>(totalOuter) / NUM_READINGS;
  averageInner = static_cast<float>(totalInner) / NUM_READINGS;
  averageHip = static_cast<float>(totalHip) / NUM_READINGS;
  averageKnee = static_cast<float>(totalKnee) / NUM_READINGS;
  averageButt = static_cast<float>(totalButt) / NUM_READINGS;
}

void normalizeReadings() {
  const float outer = adcToDegrees(averageOuter);
  const float inner = adcToDegrees(averageInner);
  const float hip = adcToDegrees(averageHip);
  const float kneeAngle = adcToDegrees(averageKnee);
  const float butt = adcToDegrees(averageButt);

  if (rawMode) {
    normalizedOuter = wrapAngleFloat(outer);
    normalizedInner = wrapAngleFloat(inner);
    normalizedHip = wrapAngleFloat(hip);
    normalizedKnee = wrapAngleFloat(kneeAngle);
    normalizedButt = wrapAngleFloat(butt);
    return;
  }

  const int *offsets = isLeft ? leftLegOffsets : rightLegOffsets;
  normalizedOuter = wrapAngleFloat(outer + offsets[0]);
  normalizedInner = wrapAngleFloat(inner + offsets[1]);
  normalizedHip = wrapAngleFloat(hip + offsets[2]);
  normalizedKnee = wrapAngleFloat(kneeAngle + offsets[3]);
  normalizedButt = wrapAngleFloat(butt + offsets[4]);
}

void primeSensorFilter() {
  totalOuter = totalInner = totalHip = totalKnee = totalButt = 0;
  readIndex = 0;
  for (int i = 0; i < NUM_READINGS; ++i) {
    readingsOuter[i] = analogRead(PIN_OUTER_CALF);
    readingsInner[i] = analogRead(PIN_INNER_CALF);
    readingsHip[i] = analogRead(PIN_HIP_PITCH);
    readingsKnee[i] = analogRead(PIN_KNEE);
    readingsButt[i] = analogRead(PIN_HIP_ROLL);

    totalOuter += readingsOuter[i];
    totalInner += readingsInner[i];
    totalHip += readingsHip[i];
    totalKnee += readingsKnee[i];
    totalButt += readingsButt[i];
    delay(2);

  }
  computeAverages();
  normalizeReadings();
}

void printReadings() {
  if (serialMutex && xSemaphoreTake(serialMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
    Serial.print(normalizedOuter, 1);
    Serial.print(',');
    Serial.print(normalizedInner, 1);
    Serial.print(',');
    Serial.print(normalizedHip, 1);
    Serial.print(',');
    Serial.print(normalizedKnee, 1);
    Serial.print(',');
    Serial.println(normalizedButt, 1);
    xSemaphoreGive(serialMutex);
  }
}

// -----------------------------------------------------------------------------
// Direction / joint mapping helpers
// -----------------------------------------------------------------------------

void setDirectionMultiplier(String jointName, float value) {
  if (jointName == "right_outer_calf") directionMultiplierRightOuterCalf = value;
  else if (jointName == "right_inner_calf") directionMultiplierRightInnerCalf = value;
  else if (jointName == "left_outer_calf") directionMultiplierLeftOuterCalf = value;
  else if (jointName == "left_inner_calf") directionMultiplierLeftInnerCalf = value;
  else if (jointName == "right_knee") directionMultiplierRightKnee = value;
  else if (jointName == "left_knee") directionMultiplierLeftKnee = value;
  else if (jointName == "right_hip_pitch") directionMultiplierRightHipPitch = value;
  else if (jointName == "left_hip_pitch") directionMultiplierLeftHipPitch = value;
  else if (jointName == "right_hip_roll") directionMultiplierRightHipRoll = value;
  else if (jointName == "left_hip_roll") directionMultiplierLeftHipRoll = value;
  else Serial.println("Invalid joint name.");
}

int jointNameToActuatorIndex(const String &joint) {
  if (joint == "right_outer_calf") return RIGHT_OUTER_CALF;
  if (joint == "left_outer_calf") return LEFT_OUTER_CALF;
  if (joint == "right_inner_calf") return RIGHT_INNER_CALF;
  if (joint == "left_inner_calf") return LEFT_INNER_CALF;
  if (joint == "right_knee") return RIGHT_KNEE;
  if (joint == "left_knee") return LEFT_KNEE;
  if (joint == "right_hip_pitch") return RIGHT_HIP_PITCH;
  if (joint == "left_hip_pitch") return LEFT_HIP_PITCH;
  if (joint == "right_hip_yaw") return RIGHT_HIP_YAW;
  if (joint == "left_hip_yaw") return LEFT_HIP_YAW;
  if (joint == "right_hip_roll") return RIGHT_HIP_ROLL;
  if (joint == "left_hip_roll") return LEFT_HIP_ROLL;
  return -1;
}

int getEncoderPinForJoint(const String &joint) {
  if (joint.endsWith("outer_calf")) return PIN_OUTER_CALF;
  if (joint.endsWith("inner_calf")) return PIN_INNER_CALF;
  if (joint.endsWith("knee")) return PIN_KNEE;
  if (joint.endsWith("hip_pitch")) return PIN_HIP_PITCH;
  if (joint.endsWith("hip_roll")) return PIN_HIP_ROLL;
  return -1; // hip yaw does not have an analog sensor in this pinout
}

int getEncoderReading(String joint) {
  const int pin = getEncoderPinForJoint(joint);
  if (pin < 0) {
    Serial.printf("No external analog encoder mapped for joint: %s\n", joint.c_str());
    return -1;
  }
  const float angle = adcToDegrees(static_cast<float>(analogRead(pin)));
  return static_cast<int>(lroundf(angle));
}

// -----------------------------------------------------------------------------
// RTOS tasks
// -----------------------------------------------------------------------------

void readAndComputeTask(void *parameter) {
  TickType_t lastWake = xTaskGetTickCount();
  unsigned long lastTelemetry = 0;

  while (true) {
    if (!isCenter) {
      readSensors();
      computeAverages();
      normalizeReadings();

      // Preserve high-rate sensing without saturating the serial port.
      if (playMode && millis() - lastTelemetry >= 20) {
        printReadings();
        lastTelemetry = millis();
      }
    }
    vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(1));

  }
}

void impedanceControlTask(void *parameter) {
  TickType_t lastWake = xTaskGetTickCount();

  while (true) {
    if (!isCenter) {
      const unsigned long now = millis();

      if (isLeft) {
        if (impedanceEnabledLeftOuterCalf) {
          outerCalfControlLeft.update(normalizedOuter, now, outerCalfConstraintsLeft.minAngle, outerCalfConstraintsLeft.maxAngle);
          impedanceTorqueValues[LEFT_OUTER_CALF] = clampTorqueCommand(outerCalfControlLeft.torqueOutput * directionMultiplierLeftOuterCalf);
        } else impedanceTorqueValues[LEFT_OUTER_CALF] = 0;
    
        if (impedanceEnabledLeftInnerCalf) {
          innerCalfControlLeft.update(normalizedInner, now, innerCalfConstraintsLeft.minAngle, innerCalfConstraintsLeft.maxAngle);
          impedanceTorqueValues[LEFT_INNER_CALF] = clampTorqueCommand(innerCalfControlLeft.torqueOutput * directionMultiplierLeftInnerCalf);
        } else impedanceTorqueValues[LEFT_INNER_CALF] = 0;
    
        if (impedanceEnabledLeftKnee) {
          kneeControlLeft.update(normalizedKnee, now, kneeConstraintsLeft.minAngle, kneeConstraintsLeft.maxAngle);
          impedanceTorqueValues[LEFT_KNEE] = clampTorqueCommand(kneeControlLeft.torqueOutput * directionMultiplierLeftKnee);
        } else impedanceTorqueValues[LEFT_KNEE] = 0;
    
        if (impedanceEnabledLeftHipPitch) {
          hipPitchControlLeft.update(normalizedHip, now, hipPitchConstraintsLeft.minAngle, hipPitchConstraintsLeft.maxAngle);
          impedanceTorqueValues[LEFT_HIP_PITCH] = clampTorqueCommand(hipPitchControlLeft.torqueOutput * directionMultiplierLeftHipPitch);
        } else impedanceTorqueValues[LEFT_HIP_PITCH] = 0;
    
        if (impedanceEnabledLeftHipRoll) {
          hipRollControlLeft.update(normalizedButt, now, hipRollConstraintsLeft.minAngle, hipRollConstraintsLeft.maxAngle);
          impedanceTorqueValues[LEFT_HIP_ROLL] = clampTorqueCommand(hipRollControlLeft.torqueOutput * directionMultiplierLeftHipRoll);
        } else impedanceTorqueValues[LEFT_HIP_ROLL] = 0;
      } else {
        if (impedanceEnabledRightOuterCalf) {
          outerCalfControlRight.update(normalizedOuter, now, outerCalfConstraintsRight.minAngle, outerCalfConstraintsRight.maxAngle);
          impedanceTorqueValues[RIGHT_OUTER_CALF] = clampTorqueCommand(outerCalfControlRight.torqueOutput * directionMultiplierRightOuterCalf);
        } else impedanceTorqueValues[RIGHT_OUTER_CALF] = 0;
    
        if (impedanceEnabledRightInnerCalf) {
          innerCalfControlRight.update(normalizedInner, now, innerCalfConstraintsRight.minAngle, innerCalfConstraintsRight.maxAngle);
          impedanceTorqueValues[RIGHT_INNER_CALF] = clampTorqueCommand(innerCalfControlRight.torqueOutput * directionMultiplierRightInnerCalf);
        } else impedanceTorqueValues[RIGHT_INNER_CALF] = 0;
    
        if (impedanceEnabledRightKnee) {
          kneeControlRight.update(normalizedKnee, now, kneeConstraintsRight.minAngle, kneeConstraintsRight.maxAngle);
          impedanceTorqueValues[RIGHT_KNEE] = clampTorqueCommand(kneeControlRight.torqueOutput * directionMultiplierRightKnee);
        } else impedanceTorqueValues[RIGHT_KNEE] = 0;
    
        if (impedanceEnabledRightHipPitch) {
          hipPitchControlRight.update(normalizedHip, now, hipPitchConstraintsRight.minAngle, hipPitchConstraintsRight.maxAngle);
          impedanceTorqueValues[RIGHT_HIP_PITCH] = clampTorqueCommand(hipPitchControlRight.torqueOutput * directionMultiplierRightHipPitch);
        } else impedanceTorqueValues[RIGHT_HIP_PITCH] = 0;
    
        if (impedanceEnabledRightHipRoll) {
          hipRollControlRight.update(normalizedButt, now, hipRollConstraintsRight.minAngle, hipRollConstraintsRight.maxAngle);
          impedanceTorqueValues[RIGHT_HIP_ROLL] = clampTorqueCommand(hipRollControlRight.torqueOutput * directionMultiplierRightHipRoll);
        } else impedanceTorqueValues[RIGHT_HIP_ROLL] = 0;
      }
    }
    
    vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(10));

  }
}

void canOutputTask(void *parameter) {
  TickType_t lastWake = xTaskGetTickCount();

  while (true) {
    if (!isCenter) {
      const int start = firstSelectedActuatorIndex();

      if (calibrationOverrideActive) {
        // During direction calibration, command only the selected test joint and
        // explicitly zero every other motor on this leg.
        for (int i = start; i < ACTUATOR_COUNT; i += 2) {
          const int16_t value = (i == calibrationActuatorIndex) ? calibrationTorqueValue : 0;
          sendTorqueCommand(ACTUATOR_IDS[i], value);
        }
      } else if (playMode) {
        for (int i = start; i < ACTUATOR_COUNT; i += 2) {
          int16_t value = torqueValues[i];
          if (isImpedanceEnabled(i)) value = impedanceTorqueValues[i];
          value = clampTorqueCommand(value);
          sendTorqueCommand(ACTUATOR_IDS[i], value);
        }
      } else if (stopBurstRemaining > 0) {
        for (int i = start; i < ACTUATOR_COUNT; i += 2) {
          sendStopCommand(ACTUATOR_IDS[i]);
        }
        --stopBurstRemaining;
      }
    }
    
    vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(10));

  }
}

void checkChiralityTask(void *parameter) {
  while (true) {
    if (Serial.available() > 0) {
      String command = Serial.readStringUntil('\n');
      command.trim();
      if (configMode) handleConfigurationCommand(command);
      else processSerialCommand(command);
    }
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

void imuReadTask(void *parameter) {
  TickType_t lastWake = xTaskGetTickCount();
  while (true) {
    if (isCenter && playMode) readIMU();
    vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(10));
  }
}

// -----------------------------------------------------------------------------
// Torque-direction calibration
// -----------------------------------------------------------------------------

void applyTestTorque(String joint, float torque) {
  const int index = jointNameToActuatorIndex(joint);
  if (index < 0) {
    Serial.println("Invalid joint name in applyTestTorque.");
    return;
  }
  if (!actuatorBelongsToSelectedLeg(index)) {
    Serial.println("Refusing calibration: requested joint does not belong to this controller's selected chirality.");
    return;
  }

  calibrationActuatorIndex = index;
  calibrationTorqueValue = clampTorqueCommand(torque);
  calibrationOverrideActive = true;
}

void endCalibrationOverride() {
  calibrationTorqueValue = 0;
  delay(30);
  calibrationOverrideActive = false;
  calibrationActuatorIndex = -1;
}

void calibrateTorqueDirection(String joint) {
  joint.trim();
  if (joint.startsWith("n ")) joint = joint.substring(2);

  const int index = jointNameToActuatorIndex(joint);
  if (index < 0 || getEncoderPinForJoint(joint) < 0) {
    Serial.printf("Invalid or unsupported joint for direction calibration: %s\n", joint.c_str());
    return;
  }
  if (!actuatorBelongsToSelectedLeg(index)) {
    Serial.println("Calibration joint is on the opposite leg. Change chirality or use the matching controller.");
    return;
  }

  const int encoderThreshold = 10;
  int encoderBefore = getEncoderReading(joint);
  if (encoderBefore < 0) return;

  Serial.printf("Initial encoder reading for %s: %d\n", joint.c_str(), encoderBefore);

  for (float testTorque = 10.0f; testTorque <= 150.0f; testTorque += 10.0f) {
    applyTestTorque(joint, testTorque);
    delay(500);
    const int encoderAfter = getEncoderReading(joint);
    Serial.printf("Encoder reading after torque %.1f for %s: %d\n", testTorque, joint.c_str(), encoderAfter);

    if (abs(encoderAfter - encoderBefore) > encoderThreshold) {
      const float multiplier = (encoderAfter > encoderBefore) ? 1.0f : -1.0f;
      setDirectionMultiplier(joint, multiplier);
      applyTestTorque(joint, 0.0f);
      endCalibrationOverride();
      saveConfig();
      Serial.printf("Torque direction for %s calibrated. Multiplier: %.1f\n", joint.c_str(), multiplier);
      return;
    }

  }

  Serial.printf("No movement detected with positive torque on %s. Trying negative torque.\n", joint.c_str());
  encoderBefore = getEncoderReading(joint);

  for (float testTorque = -10.0f; testTorque >= -150.0f; testTorque -= 10.0f) {
    applyTestTorque(joint, testTorque);
    delay(500);
    const int encoderAfter = getEncoderReading(joint);
    Serial.printf("Encoder reading after torque %.1f for %s: %d\n", testTorque, joint.c_str(), encoderAfter);

    if (abs(encoderAfter - encoderBefore) > encoderThreshold) {
      const float multiplier = (encoderAfter < encoderBefore) ? 1.0f : -1.0f;
      setDirectionMultiplier(joint, multiplier);
      applyTestTorque(joint, 0.0f);
      endCalibrationOverride();
      saveConfig();
      Serial.printf("Torque direction for %s calibrated. Multiplier: %.1f\n", joint.c_str(), multiplier);
      return;
    }

  }

  applyTestTorque(joint, 0.0f);
  endCalibrationOverride();
  Serial.printf("Joint %s could not be calibrated. Check mechanical lock, sensor motion, CAN, and actuator power.\n", joint.c_str());
}

// Compatibility wrappers. Direction multipliers now persist through config.txt.
void saveDirectionMultiplierToSPIFFS(String joint, float multiplier) {
  (void)joint;
  (void)multiplier;
  saveConfig();
}

void loadDirectionMultiplierFromSPIFFS() {
  // Direction multipliers are loaded by loadConfig().
}

// -----------------------------------------------------------------------------
// Configuration persistence
// -----------------------------------------------------------------------------

void saveJointConstraintsToFile(File &file, JointConstraints constraints, const char *jointName) {
  file.printf("%s Constraints:%d,%d\n", jointName, constraints.minAngle, constraints.maxAngle);
}

JointConstraints loadJointConstraintsFromFile(String line, JointConstraints defaultConstraints) {
  JointConstraints constraints = defaultConstraints;
  const int marker = line.indexOf("Constraints:");
  if (marker >= 0) {
    const String values = line.substring(marker + 12);
    const int comma = values.indexOf(',');
    if (comma > 0) {
      constraints.minAngle = values.substring(0, comma).toInt();
      constraints.maxAngle = values.substring(comma + 1).toInt();
    }
  }
  return constraints;
}

void saveConfig() {
  File file = SPIFFS.open("/config.txt", FILE_WRITE);
  if (!file) {
    Serial.println("Failed to open /config.txt for writing.");
    return;
  }

  file.printf("LegSide:%s\n", isCenter ? "center" : (isLeft ? "left" : "right"));
  file.printf("LeftOffsets:%d,%d,%d,%d,%d\n",
              leftLegOffsets[0], leftLegOffsets[1], leftLegOffsets[2], leftLegOffsets[3], leftLegOffsets[4]);
  file.printf("RightOffsets:%d,%d,%d,%d,%d\n",
              rightLegOffsets[0], rightLegOffsets[1], rightLegOffsets[2], rightLegOffsets[3], rightLegOffsets[4]);
  file.printf("DirectionMultipliers:%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n",
              directionMultiplierRightOuterCalf, directionMultiplierRightInnerCalf,
              directionMultiplierLeftOuterCalf, directionMultiplierLeftInnerCalf,
              directionMultiplierRightKnee, directionMultiplierLeftKnee,
              directionMultiplierRightHipPitch, directionMultiplierLeftHipPitch,
              directionMultiplierRightHipRoll, directionMultiplierLeftHipRoll);

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

void promptLegSide() {
  while (true) {
    Serial.println("Enter 'left' for left leg, 'right' for right leg, or 'center' for IMU center.");
    while (!Serial.available()) delay(10);
    String side = Serial.readStringUntil('\n');
    side.trim();

    if (side == "left") {
      isLeft = true;
      isCenter = false;
      break;
    }
    if (side == "right") {
      isLeft = false;
      isCenter = false;
      break;
    }
    if (side == "center") {
      isCenter = true;
      break;
    }
    Serial.println("Invalid selection.");

  }

  Serial.println(isCenter ? "Center mode selected." : (isLeft ? "Left leg selected." : "Right leg selected."));
}

void loadConfig() {
  if (!SPIFFS.exists("/config.txt")) {
    Serial.println("No configuration file found. Using hardcoded offsets and constraints.");
    promptLegSide();
    saveConfig();
    return;
  }

  File file = SPIFFS.open("/config.txt");
  if (!file) {
    Serial.println("Failed to open configuration file. Using defaults.");
    promptLegSide();
    return;
  }

  String line = file.readStringUntil('\n');
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

  line = file.readStringUntil('\n');
  if (line.startsWith("LeftOffsets:")) {
    sscanf(line.c_str(), "LeftOffsets:%d,%d,%d,%d,%d",
           &leftLegOffsets[0], &leftLegOffsets[1], &leftLegOffsets[2], &leftLegOffsets[3], &leftLegOffsets[4]);
  }

  line = file.readStringUntil('\n');
  if (line.startsWith("RightOffsets:")) {
    sscanf(line.c_str(), "RightOffsets:%d,%d,%d,%d,%d",
           &rightLegOffsets[0], &rightLegOffsets[1], &rightLegOffsets[2], &rightLegOffsets[3], &rightLegOffsets[4]);
  }

  line = file.readStringUntil('\n');
  if (line.startsWith("DirectionMultipliers:")) {
    sscanf(line.c_str(), "DirectionMultipliers:%f,%f,%f,%f,%f,%f,%f,%f,%f,%f",
           &directionMultiplierRightOuterCalf, &directionMultiplierRightInnerCalf,
           &directionMultiplierLeftOuterCalf, &directionMultiplierLeftInnerCalf,
           &directionMultiplierRightKnee, &directionMultiplierLeftKnee,
           &directionMultiplierRightHipPitch, &directionMultiplierLeftHipPitch,
           &directionMultiplierRightHipRoll, &directionMultiplierLeftHipRoll);
  }

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

void resetSPIFFS() {
  Serial.println("Checking SPIFFS integrity...");
  if (SPIFFS.totalBytes() == 0) {
    Serial.println("SPIFFS appears unformatted or corrupted. Type 'yes' to format.");
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

// -----------------------------------------------------------------------------
// Joint constraints
// -----------------------------------------------------------------------------

void setJointConstraints(String jointName, int minAngle, int maxAngle) {
  if (minAngle > maxAngle) {
    Serial.println("Constraint rejected: minAngle must be <= maxAngle.");
    return;
  }

  bool valid = true;
  JointConstraints *target = nullptr;
  if (jointName == "outer_calf_left") target = &outerCalfConstraintsLeft;
  else if (jointName == "outer_calf_right") target = &outerCalfConstraintsRight;
  else if (jointName == "inner_calf_left") target = &innerCalfConstraintsLeft;
  else if (jointName == "inner_calf_right") target = &innerCalfConstraintsRight;
  else if (jointName == "knee_left") target = &kneeConstraintsLeft;
  else if (jointName == "knee_right") target = &kneeConstraintsRight;
  else if (jointName == "hip_pitch_left") target = &hipPitchConstraintsLeft;
  else if (jointName == "hip_pitch_right") target = &hipPitchConstraintsRight;
  else if (jointName == "hip_yaw_left") target = &hipYawConstraintsLeft;
  else if (jointName == "hip_yaw_right") target = &hipYawConstraintsRight;
  else if (jointName == "hip_roll_left") target = &hipRollConstraintsLeft;
  else if (jointName == "hip_roll_right") target = &hipRollConstraintsRight;
  else valid = false;

  if (target) {
    target->minAngle = minAngle;
    target->maxAngle = maxAngle;
  }

  if (!valid) {
    Serial.printf("Unknown joint constraint name: %s\n", jointName.c_str());
    return;
  }

  saveConfig();
  Serial.printf("Joint constraints for %s set to min=%d max=%d\n", jointName.c_str(), minAngle, maxAngle);
}

void constrainJoint(String command) {
  const int firstSpace = command.indexOf(' ');
  const int secondSpace = command.indexOf(' ', firstSpace + 1);
  const int thirdSpace = command.indexOf(' ', secondSpace + 1);

  if (firstSpace < 0 || secondSpace < 0 || thirdSpace < 0) {
    Serial.println("Usage: constrain <jointname> <minval> <maxval>");
    return;
  }

  const String jointName = command.substring(firstSpace + 1, secondSpace);
  const int minVal = command.substring(secondSpace + 1, thirdSpace).toInt();
  const int maxVal = command.substring(thirdSpace + 1).toInt();
  setJointConstraints(jointName, minVal, maxVal);
}

void configureJointConstraintsViaSerial() {
  Serial.println("Enter joint name:");
  while (!Serial.available()) delay(10);
  String jointName = Serial.readStringUntil('\n');
  jointName.trim();

  Serial.println("Enter min angle:");
  while (!Serial.available()) delay(10);
  String minString = Serial.readStringUntil('\n');
  minString.trim();

  Serial.println("Enter max angle:");
  while (!Serial.available()) delay(10);
  String maxString = Serial.readStringUntil('\n');
  maxString.trim();

  setJointConstraints(jointName, minString.toInt(), maxString.toInt());
}

// -----------------------------------------------------------------------------
// Sensor calibration
// -----------------------------------------------------------------------------

void resetOffsets() {
  for (int i = 0; i < 5; ++i) {
    leftLegOffsets[i] = 0;
    rightLegOffsets[i] = 0;
  }
  Serial.println("Offsets reset to zero.");
  saveConfig();
}

void calibrateSensors() {
  if (isCenter) {
    Serial.println("Sensor calibration is not available in center mode.");
    return;
  }

  // Acquire a fresh stable average even if play mode is disabled/config mode.
  primeSensorFilter();

  const float rawOuter = adcToDegrees(averageOuter);
  const float rawInner = adcToDegrees(averageInner);
  const float rawHip = adcToDegrees(averageHip);
  const float rawKnee = adcToDegrees(averageKnee);
  const float rawButt = adcToDegrees(averageButt);

  const int offsetOuter = static_cast<int>(lroundf(180.0f - rawOuter));
  const int offsetInner = static_cast<int>(lroundf(180.0f - rawInner));
  const int offsetHip = static_cast<int>(lroundf(180.0f - rawHip));
  const int offsetKnee = static_cast<int>(lroundf(180.0f - rawKnee));
  const int offsetButt = static_cast<int>(lroundf(180.0f - rawButt));

  int *offsets = isLeft ? leftLegOffsets : rightLegOffsets;
  offsets[0] = offsetOuter;
  offsets[1] = offsetInner;
  offsets[2] = offsetHip;
  offsets[3] = offsetKnee;
  offsets[4] = offsetButt;

  Serial.println("Calibration complete. Offsets aligning current pose to 180 degrees:");
  Serial.printf("Outer=%d Inner=%d HipPitch=%d Knee=%d HipRoll=%d\n",
                offsetOuter, offsetInner, offsetHip, offsetKnee, offsetButt);
  Serial.println("Type 'yes' to save these offsets, anything else to leave them only in RAM.");

  while (!Serial.available()) delay(10);
  String response = Serial.readStringUntil('\n');
  response.trim();
  if (response.equalsIgnoreCase("yes")) {
    saveConfig();
    Serial.println("Offsets saved.");
  } else {
    Serial.println("Offsets not persisted.");
  }
}

void printSavedOffsets() {
  const int *offsets = isLeft ? leftLegOffsets : rightLegOffsets;
  Serial.printf("%s offsets - Outer:%d Inner:%d HipPitch:%d Knee:%d HipRoll:%d\n",
                isLeft ? "Left" : "Right",
                offsets[0], offsets[1], offsets[2], offsets[3], offsets[4]);
}

// -----------------------------------------------------------------------------
// IMU / misc
// -----------------------------------------------------------------------------

void readIMU() {
  for (int i = 0; i < IMU_COUNT; ++i) {
    const uint8_t address = 0x68 + i;
    Wire.beginTransmission(address);
    Wire.write(0x3B);
    if (Wire.endTransmission(false) != 0) continue;

    const uint8_t received = Wire.requestFrom(address, static_cast<uint8_t>(14), static_cast<uint8_t>(true));
    if (received < 14) continue;
    
    const int16_t ax = static_cast<int16_t>((Wire.read() << 8) | Wire.read());
    const int16_t ay = static_cast<int16_t>((Wire.read() << 8) | Wire.read());
    const int16_t az = static_cast<int16_t>((Wire.read() << 8) | Wire.read());
    (void)Wire.read(); (void)Wire.read(); // temperature bytes
    const int16_t gx = static_cast<int16_t>((Wire.read() << 8) | Wire.read());
    const int16_t gy = static_cast<int16_t>((Wire.read() << 8) | Wire.read());
    const int16_t gz = static_cast<int16_t>((Wire.read() << 8) | Wire.read());
    
    Serial.printf("IMU %d Acc:%d,%d,%d Gyro:%d,%d,%d\n", i, ax, ay, az, gx, gy, gz);

  }
}

void printMACAddress() {
  uint8_t mac[6];
  WiFi.macAddress(mac);
  Serial.printf("MAC Address: %02X:%02X:%02X:%02X:%02X:%02X\n",
                mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}

void printStatus() {
  Serial.printf("Mode: %s | chirality: %s | play:%d | raw:%d | maxTorque:%.2f A\n",
                configMode ? "config" : "normal",
                isCenter ? "center" : (isLeft ? "left" : "right"),
                playMode, rawMode, maxTorqueLimit);
  if (!isCenter) {
    Serial.printf("Angles: outer=%.1f inner=%.1f hipPitch=%.1f knee=%.1f hipRoll=%.1f\n",
                  normalizedOuter, normalizedInner, normalizedHip, normalizedKnee, normalizedButt);
  }
}

// -----------------------------------------------------------------------------
// Mode handling
// -----------------------------------------------------------------------------

void enterConfigurationMode() {
  Serial.println("Entering Configuration Mode...");
  requestStop(3);
  delay(40); // allow CAN output task to emit the stop burst before configuration
  configMode = true;
  Serial.println("Configuration mode entered. Type 'exit' to leave.");
}

void exitConfigurationMode() {
  configMode = false;
  Serial.println("Configuration mode exited. Use 'play' explicitly to resume actuator output.");
}

void changeChirality(const String &side) {
  if (!isCenter) {
    requestStop(3);
    delay(40);
  }

  clearAllTorqueSetpoints();

  if (side == "left") {
    isLeft = true;
    isCenter = false;
  } else if (side == "right") {
    isLeft = false;
    isCenter = false;
  } else if (side == "center") {
    isCenter = true;
  } else {
    Serial.println("Invalid chirality.");
    return;
  }

  saveConfig();
  Serial.println("Chirality set to: " + side);
  Serial.println("Reboot recommended after changing between center and leg modes so tasks/hardware are reinitialized cleanly.");
}

void handleConfigurationCommand(String command) {
  command.trim();
  if (command == "exit") exitConfigurationMode();
  else if (command == "left" || command == "right" || command == "center") changeChirality(command);
  else if (command == "calibrate") calibrateSensors();
  else if (command == "save") saveConfig();
  else if (command == "status") printStatus();
  else Serial.println("Unknown configuration command: " + command);
}

// -----------------------------------------------------------------------------
// Serial command parser
// -----------------------------------------------------------------------------

void processImpedanceCommand(const String &command) {
  String params = command.substring(10);
  params.trim();

  const int p1 = params.indexOf(' ');
  const int p2 = (p1 >= 0) ? params.indexOf(' ', p1 + 1) : -1;
  const int p3 = (p2 >= 0) ? params.indexOf(' ', p2 + 1) : -1;
  const int p4 = (p3 >= 0) ? params.indexOf(' ', p3 + 1) : -1;

  if (p1 < 0 || p2 < 0 || p3 < 0 || p4 < 0) {
    Serial.println("Usage: impedance <legSide> <appendage> <0|1> <desiredPosition> <desiredVelocity>");
    return;
  }

  const String legSide = params.substring(0, p1);
  const String appendage = params.substring(p1 + 1, p2);
  const bool enable = params.substring(p2 + 1, p3).toInt() != 0;
  const float desiredPosition = params.substring(p3 + 1, p4).toFloat();
  const float desiredVelocity = params.substring(p4 + 1).toFloat();

  if ((isLeft && legSide != "left") || (!isLeft && !isCenter && legSide != "right") || isCenter) {
    Serial.println("Impedance command does not match this controller's selected leg.");
    return;
  }

  ImpedanceControl *control = nullptr;
  bool *enabledFlag = nullptr;
  int index = -1;

  if (legSide == "left") {
    if (appendage == "outer_calf") { control = &outerCalfControlLeft; enabledFlag = &impedanceEnabledLeftOuterCalf; index = LEFT_OUTER_CALF; }
    else if (appendage == "inner_calf") { control = &innerCalfControlLeft; enabledFlag = &impedanceEnabledLeftInnerCalf; index = LEFT_INNER_CALF; }
    else if (appendage == "knee") { control = &kneeControlLeft; enabledFlag = &impedanceEnabledLeftKnee; index = LEFT_KNEE; }
    else if (appendage == "hip_pitch") { control = &hipPitchControlLeft; enabledFlag = &impedanceEnabledLeftHipPitch; index = LEFT_HIP_PITCH; }
    else if (appendage == "hip_roll") { control = &hipRollControlLeft; enabledFlag = &impedanceEnabledLeftHipRoll; index = LEFT_HIP_ROLL; }
  } else if (legSide == "right") {
    if (appendage == "outer_calf") { control = &outerCalfControlRight; enabledFlag = &impedanceEnabledRightOuterCalf; index = RIGHT_OUTER_CALF; }
    else if (appendage == "inner_calf") { control = &innerCalfControlRight; enabledFlag = &impedanceEnabledRightInnerCalf; index = RIGHT_INNER_CALF; }
    else if (appendage == "knee") { control = &kneeControlRight; enabledFlag = &impedanceEnabledRightKnee; index = RIGHT_KNEE; }
    else if (appendage == "hip_pitch") { control = &hipPitchControlRight; enabledFlag = &impedanceEnabledRightHipPitch; index = RIGHT_HIP_PITCH; }
    else if (appendage == "hip_roll") { control = &hipRollControlRight; enabledFlag = &impedanceEnabledRightHipRoll; index = RIGHT_HIP_ROLL; }
  }

  if (!control || !enabledFlag || index < 0) {
    Serial.println("Invalid appendage. Hip yaw remains direct-torque only because this firmware has no hip-yaw analog sensor input.");
    return;
  }

  *enabledFlag = enable;
  impedanceTorqueValues[index] = 0;
  control->reset();
  if (enable) {
    control->setDesiredPosition(desiredPosition);
    control->setDesiredVelocity(desiredVelocity);
  }

  Serial.printf("Impedance %s %s -> %d, position %.2f, velocity %.2f\n",
                legSide.c_str(), appendage.c_str(), enable, desiredPosition, desiredVelocity);
}

void processTorqueCommand(const String &command) {
  String params = command.substring(7);
  params.trim();
  const int firstSpace = params.indexOf(' ');
  const int secondSpace = params.indexOf(' ', firstSpace + 1);

  if (firstSpace < 0 || secondSpace < 0) {
    Serial.println("Usage: torque <legSide> <appendage> <torqueValue>");
    return;
  }

  const String legSide = params.substring(0, firstSpace);
  const String appendage = params.substring(firstSpace + 1, secondSpace);
  const int16_t torqueValue = clampTorqueCommand(params.substring(secondSpace + 1).toFloat());

  int index = -1;
  if (legSide == "left") {
    if (appendage == "outer_calf") index = LEFT_OUTER_CALF;
    else if (appendage == "inner_calf") index = LEFT_INNER_CALF;
    else if (appendage == "knee") index = LEFT_KNEE;
    else if (appendage == "hip_pitch") index = LEFT_HIP_PITCH;
    else if (appendage == "hip_yaw") index = LEFT_HIP_YAW;
    else if (appendage == "hip_roll") index = LEFT_HIP_ROLL;
  } else if (legSide == "right") {
    if (appendage == "outer_calf") index = RIGHT_OUTER_CALF;
    else if (appendage == "inner_calf") index = RIGHT_INNER_CALF;
    else if (appendage == "knee") index = RIGHT_KNEE;
    else if (appendage == "hip_pitch") index = RIGHT_HIP_PITCH;
    else if (appendage == "hip_yaw") index = RIGHT_HIP_YAW;
    else if (appendage == "hip_roll") index = RIGHT_HIP_ROLL;
  }

  if (index < 0) {
    Serial.println("Invalid torque appendage.");
    return;
  }
  if (!actuatorBelongsToSelectedLeg(index)) {
    Serial.println("Torque command rejected: joint is not on this controller's selected leg.");
    return;
  }

  torqueValues[index] = torqueValue;
  Serial.printf("Torque for %s %s set to %d\n", legSide.c_str(), appendage.c_str(), torqueValue);
}

void processSerialCommand(String command) {
  command.trim();
  if (command.length() == 0) return;

  if (serialMutex && xSemaphoreTake(serialMutex, portMAX_DELAY) != pdTRUE) return;

  if (command == "config") {
    xSemaphoreGive(serialMutex);
    enterConfigurationMode();
    return;
  } else if (command.startsWith("calibrateDirection ")) {
    const String joint = command.substring(String("calibrateDirection ").length());
    xSemaphoreGive(serialMutex);
    calibrateTorqueDirection(joint);
    return;
  } else if (command == "resetOffsets") {
    resetOffsets();
  } else if (command == "raw on") {
    rawMode = true;
    Serial.println("Raw mode enabled. Offsets bypassed.");
  } else if (command == "raw off") {
    rawMode = false;
    Serial.println("Raw mode disabled. Offsets enabled.");
  } else if (command.startsWith("direction ")) {
    String params = command.substring(10);
    params.trim();
    const int split = params.indexOf(' ');
    if (split > 0) {
      const String joint = params.substring(0, split);
      const String direction = params.substring(split + 1);
      const float multiplier = (direction == "-") ? -1.0f : ((direction == "+") ? 1.0f : 0.0f);
      if (multiplier == 0.0f) Serial.println("Direction must be + or -.");
      else {
        setDirectionMultiplier(joint, multiplier);
        saveConfig();
        Serial.printf("Direction for %s set to %s\n", joint.c_str(), direction.c_str());
      }
    } else Serial.println("Usage: direction <joint> <+|->");
  } else if (command == "help") {
    printHelp();
  } else if (command.startsWith("impedance ")) {
    processImpedanceCommand(command);
  } else if (command == "resetSPIFFS") {
    xSemaphoreGive(serialMutex);
    resetSPIFFS();
    return;
  } else if (command.startsWith("constrain ")) {
    constrainJoint(command);
  } else if (command.startsWith("torque ")) {
    processTorqueCommand(command);
  } else if (command == "mac") {
    printMACAddress();
  } else if (command == "chirality") {
    Serial.println("Current chirality: " + String(isCenter ? "center" : (isLeft ? "left" : "right")));
  } else if (command == "calibrate") {
    xSemaphoreGive(serialMutex);
    calibrateSensors();
    return;
  } else if (command == "save") {
    saveConfig();
  } else if (command == "saved") {
    printSavedOffsets();
  } else if (command == "play") {
    if (isCenter) {
      playMode = true;
      Serial.println("Center play mode enabled (IMU streaming).");
    } else {
      stopBurstRemaining = 0;
      playMode = true;
      Serial.println("Play mode enabled.");
    }
  } else if (command == "stop") {
    requestStop(3);
    Serial.println("Play mode disabled. Stop burst queued for this leg.");
  } else if (command == "zero") {
    clearAllTorqueSetpoints();
    Serial.println("All direct and impedance torque setpoints cleared.");
  } else if (command == "left" || command == "right" || command == "center") {
    xSemaphoreGive(serialMutex);
    changeChirality(command);
    return;
  } else if (command == "setJointConstraints") {
    xSemaphoreGive(serialMutex);
    configureJointConstraintsViaSerial();
    return;
  } else if (command == "status") {
    printStatus();
  } else {
    Serial.println("Unknown command. Type 'help'.");
  }

  xSemaphoreGive(serialMutex);
}

// -----------------------------------------------------------------------------
// Help
// -----------------------------------------------------------------------------

void printHelp() {
  Serial.println("Available Commands:");
  Serial.println("  config");
  Serial.println("      Enter configuration mode and stop this leg.");
  Serial.println("  resetOffsets");
  Serial.println("      Reset left and right stored offsets to zero.");
  Serial.println("  raw on | raw off");
  Serial.println("      Bypass or apply joint offsets.");
  Serial.println("  direction <joint> <+|->");
  Serial.println("      Example: direction right_outer_calf +");
  Serial.println("  impedance <legSide> <appendage> <0|1> <desiredPosition> <desiredVelocity>");
  Serial.println("      Example: impedance left knee 1 180 0");
  Serial.println("      Supported: outer_calf inner_calf knee hip_pitch hip_roll");
  Serial.println("  constrain <joint> <minAngle> <maxAngle>");
  Serial.println("      Example: constrain right_knee 0 180");
  Serial.println("  torque <legSide> <appendage> <torqueValue>");
  Serial.println("      Example: torque left knee 100");
  Serial.println("  calibrateDirection <joint>");
  Serial.println("      Supported joints with external sensor feedback:");
  Serial.println("      right/left_outer_calf, inner_calf, knee, hip_pitch, hip_roll");
  Serial.println("  mac");
  Serial.println("  chirality");
  Serial.println("  calibrate");
  Serial.println("  save");
  Serial.println("  saved");
  Serial.println("  resetSPIFFS");
  Serial.println("  play");
  Serial.println("  stop");
  Serial.println("  zero");
  Serial.println("      Clear all torque setpoints without changing chirality/config.");
  Serial.println("  status");
  Serial.println("  setJointConstraints");
  Serial.println("  left | right | center");
  Serial.println("      Changes chirality; reboot recommended when switching center/leg roles.");
  Serial.println("  help");
}

// -----------------------------------------------------------------------------
// Arduino setup / loop
// -----------------------------------------------------------------------------

void setup() {
  Serial.begin(115200);
  delay(200);

  serialMutex = xSemaphoreCreateMutex();
  canMutex = xSemaphoreCreateMutex();
  stateMutex = xSemaphoreCreateMutex();

  analogReadResolution(12);
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);

  if (!SPIFFS.begin(true)) {
    Serial.println("ERROR: SPIFFS mount failed.");
    while (true) delay(1000);
  }

  loadConfig();

  if (!isCenter) {
    pinMode(CAN0_INT, INPUT_PULLUP);
    SPI.begin(SPI_SCK_PIN, SPI_MISO_PIN, SPI_MOSI_PIN, CAN_CS_PIN);

    if (CAN.begin(MCP_ANY, CAN_1000KBPS, MCP_8MHZ) != CAN_OK) {
      Serial.println("ERROR: MCP2515 CAN initialization failed at 1 Mbps / 8 MHz.");
      while (true) delay(1000);
    }
    CAN.setMode(MCP_NORMAL);
    Serial.println("CAN initialized: 1 Mbps, MCP2515 8 MHz, CS GPIO5.");
    
    primeSensorFilter();
    
    xTaskCreatePinnedToCore(readAndComputeTask, "sensors", 4096, nullptr, 2, nullptr, 0);
    xTaskCreatePinnedToCore(impedanceControlTask, "impedance", 4096, nullptr, 2, nullptr, 1);
    xTaskCreatePinnedToCore(canOutputTask, "can-output", 4096, nullptr, 3, nullptr, 1);

  } else {
    xTaskCreatePinnedToCore(imuReadTask, "imu", 4096, nullptr, 1, nullptr, 0);
  }

  xTaskCreatePinnedToCore(checkChiralityTask, "serial-command", 4096, nullptr, 1, nullptr, 1);

  // Preserve the previous startup behavior. Torque setpoints are zero by
  // default, so leg mode begins by transmitting zero A1 torque commands.
  playMode = true;

  Serial.printf("Dropbear controller ready: %s. Type 'help'.\n",
                isCenter ? "center" : (isLeft ? "left leg" : "right leg"));
}

void loop() {
  // FreeRTOS tasks own the runtime.
  vTaskDelay(pdMS_TO_TICKS(1000));
}
