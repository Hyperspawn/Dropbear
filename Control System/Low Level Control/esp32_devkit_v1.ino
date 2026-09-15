#include <Arduino.h>
#include <mcp_can.h>
#include <SPI.h>
#include <SPIFFS.h>
#include <Wire.h>
#include <WiFi.h>
#include <WebServer.h>
#include <DNSServer.h>
#include <stdarg.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <math.h>

/*
 * Dropbear ESP32 low-level leg controller
 *
 * Updated architecture:
 *   - Preserves existing ESP32 MCP2515/SPI and I2C pinout.
 *   - Portal-enabled builds move joint sensors to ADC1 GPIOs because classic
 *     ESP32 ADC2 channels cannot be used reliably while Wi-Fi is active.
 *   - Preserves MyActuator CAN IDs 0x141..0x14C.
 *   - Preserves A1 torque-control and 0x81 stop commands.
 *   - Uses one periodic CAN-output task as the normal actuator writer.
 *   - Impedance task computes desired torque only; it does not transmit CAN.
 *   - Each leg controller transmits only to its selected chirality (6 motors).
 *   - Correct stop commands use actual actuator CAN IDs.
 *   - Calibration uses an output override instead of racing the CAN task.
 *   - Fixes impedance serial parsing and joint-constraint persistence.
 *   - Starts the IMU task in center mode.
 *   - Sensor acquisition remains 1 kHz; serial telemetry is throttled to 50 Hz.
 *   - Adds role-aware Wi-Fi SoftAP + captive portal control/configuration.
 *   - LEFTLEG / RIGHTLEG / CENTER SSID follows persisted chirality.
 *   - Existing SPIFFS configuration is loaded into the web configurator.
 *   - Web commands are queued into the same parser used by USB Serial.
 *   - Live state, command log, config, and SPIFFS inspection are exposed locally.
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

// Full-time Wi-Fi and ESP32 ADC2 cannot coexist on classic ESP32.
// Portal-enabled builds therefore use only ADC1 sensor inputs.
//
// WIFI-SAFE ADC1 PINOUT (default):
//   outer calf  -> GPIO32  ADC1_CH4
//   inner calf  -> GPIO34  ADC1_CH6
//   hip pitch   -> GPIO35  ADC1_CH7
//   knee        -> GPIO36  ADC1_CH0
//   hip roll    -> GPIO33  ADC1_CH5
//
// LEGACY DROPBEAR PINOUT:
//   14,27,26,25,33. GPIO14/25/26/27 are ADC2 and are not safe while
//   Wi-Fi is active on classic ESP32.
//
// Set to 0 only if retaining the legacy wiring; doing so disables the Wi-Fi
// captive portal at runtime so ADC2 sensing remains valid.
#define DROPBEAR_ENABLE_WIFI_PORTAL 1

#if DROPBEAR_ENABLE_WIFI_PORTAL
static const int PIN_OUTER_CALF = 32;
static const int PIN_INNER_CALF = 34;
static const int PIN_HIP_PITCH = 35;
static const int PIN_KNEE = 36;
static const int PIN_HIP_ROLL = 33;
#else
static const int PIN_OUTER_CALF = 14;
static const int PIN_INNER_CALF = 27;
static const int PIN_HIP_PITCH = 26;
static const int PIN_KNEE = 25;
static const int PIN_HIP_ROLL = 33;
#endif

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

SemaphoreHandle_t webLogMutex = nullptr;
SemaphoreHandle_t spiffsMutex = nullptr;

// True only when a valid LegSide value has been loaded/saved. A fresh device
// boots into DROPBEAR-SETUP and does not start actuator tasks until configured.
bool configProvisioned = false;
String roleAtBoot = "unconfigured";
volatile bool rebootRequired = false;

// These flags describe the task topology actually instantiated at boot.
// Changing role never silently repurposes already-running actuator tasks.
volatile bool runtimeControlReady = false;
volatile bool runtimeImuReady = false;

// -----------------------------------------------------------------------------
// Captive portal / web command plumbing
// -----------------------------------------------------------------------------

DNSServer dnsServer;
WebServer server(80);

IPAddress portalIP(192, 168, 4, 1);
IPAddress portalGateway(192, 168, 4, 1);
IPAddress portalSubnet(255, 255, 255, 0);

static const uint16_t DNS_PORT = 53;
static const size_t WEB_COMMAND_MAX = 192;
static const size_t WEB_LOG_CAPACITY = 96;

struct WebCommand {
  uint32_t id;
  char text[WEB_COMMAND_MAX];
};

struct WebLogEntry {
  uint32_t seq = 0;
  uint32_t ms = 0;
  String text;
};

QueueHandle_t webCommandQueue = nullptr;
WebLogEntry webLog[WEB_LOG_CAPACITY];
size_t webLogHead = 0;
size_t webLogCount = 0;
uint32_t webLogSequence = 0;
uint32_t nextWebCommandId = 1;

volatile bool portalRestartRequested = false;
volatile bool portalRebootRequested = false;
volatile uint32_t portalRebootAtMs = 0;
bool portalRoutesRegistered = false;
String portalSSID = "DROPBEAR-SETUP";

// Command-output capture is only activated while the single command task is
// executing a web command. All human-readable command responses are mirrored
// into the web log as well as USB Serial.
String *activeCommandCapture = nullptr;

String selectedRoleName();
String desiredPortalSSID();
void requestPortalRestart();
void appendWebLog(const String &line);
void dbPrintln(const String &line);
void dbPrintf(const char *format, ...);
void setupPortal();
void portalTask(void *parameter);
void handleConfigurationCommand(String command);
void processSerialCommand(String command);
void readIMU();
void saveConfig();
void printHelp();


// -----------------------------------------------------------------------------
// Unified human-readable logging
// -----------------------------------------------------------------------------

void appendWebLog(const String &line) {
  if (line.length() == 0) return;
  if (webLogMutex && xSemaphoreTake(webLogMutex, pdMS_TO_TICKS(20)) != pdTRUE) return;

  WebLogEntry &entry = webLog[webLogHead];
  entry.seq = ++webLogSequence;
  entry.ms = millis();
  entry.text = line;

  webLogHead = (webLogHead + 1) % WEB_LOG_CAPACITY;
  if (webLogCount < WEB_LOG_CAPACITY) ++webLogCount;

  if (webLogMutex) xSemaphoreGive(webLogMutex);
}

void captureCommandText(const String &text) {
  if (!activeCommandCapture) return;
  *activeCommandCapture += text;
}

void dbPrintln(const String &line) {
  Serial.println(line);
  captureCommandText(line + "\n");
  appendWebLog(line);
}

void dbPrintf(const char *format, ...) {
  char buffer[512];
  va_list args;
  va_start(args, format);
  vsnprintf(buffer, sizeof(buffer), format, args);
  va_end(args);

  Serial.print(buffer);
  String text(buffer);
  captureCommandText(text);

  int start = 0;
  while (start < (int)text.length()) {
    int nl = text.indexOf('\n', start);
    if (nl < 0) {
      String tail = text.substring(start);
      tail.trim();
      if (tail.length()) appendWebLog(tail);
      break;
    }
    String line = text.substring(start, nl);
    line.trim();
    if (line.length()) appendWebLog(line);
    start = nl + 1;
  }
}

String selectedRoleName() {
  if (!configProvisioned) return "unconfigured";
  if (isCenter) return "center";
  return isLeft ? "left" : "right";
}

String desiredPortalSSID() {
  if (!configProvisioned) return "DROPBEAR-SETUP";
  if (isCenter) return "CENTER";
  return isLeft ? "LEFTLEG" : "RIGHTLEG";
}

void requestPortalRestart() {
  portalRestartRequested = true;
}

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
  else dbPrintln("Invalid joint name.");
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
    dbPrintf("No external analog encoder mapped for joint: %s\n", joint.c_str());
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
    if (runtimeControlReady && !isCenter) {
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

void executeQueuedWebCommand(const WebCommand &item) {
  String command(item.text);
  command.trim();

  String capture;
  capture.reserve(1024);
  activeCommandCapture = &capture;
  appendWebLog(String("WEB #") + item.id + " > " + command);

  if (configMode) handleConfigurationCommand(command);
  else processSerialCommand(command);

  activeCommandCapture = nullptr;
  if (!capture.length()) appendWebLog(String("WEB #") + item.id + " < (no textual response)");
}

void checkChiralityTask(void *parameter) {
  while (true) {
    // USB Serial and web commands intentionally converge here.
    if (Serial.available() > 0) {
      String command = Serial.readStringUntil('\n');
      command.trim();
      if (command.length()) {
        appendWebLog("SERIAL > " + command);
        if (configMode) handleConfigurationCommand(command);
        else processSerialCommand(command);
      }
    }

    if (webCommandQueue) {
      WebCommand item;
      if (xQueueReceive(webCommandQueue, &item, 0) == pdTRUE) {
        executeQueuedWebCommand(item);
      }
    }

    vTaskDelay(pdMS_TO_TICKS(5));
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
    dbPrintln("Invalid joint name in applyTestTorque.");
    return;
  }
  if (!actuatorBelongsToSelectedLeg(index)) {
    dbPrintln("Refusing calibration: requested joint does not belong to this controller's selected chirality.");
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
  if (!runtimeControlReady) {
    dbPrintln("Direction calibration unavailable: actuator control runtime is not active. Reboot after role configuration.");
    return;
  }
  if (joint.startsWith("n ")) joint = joint.substring(2);

  const int index = jointNameToActuatorIndex(joint);
  if (index < 0 || getEncoderPinForJoint(joint) < 0) {
    dbPrintf("Invalid or unsupported joint for direction calibration: %s\n", joint.c_str());
    return;
  }
  if (!actuatorBelongsToSelectedLeg(index)) {
    dbPrintln("Calibration joint is on the opposite leg. Change chirality or use the matching controller.");
    return;
  }

  const int encoderThreshold = 10;
  int encoderBefore = getEncoderReading(joint);
  if (encoderBefore < 0) return;

  dbPrintf("Initial encoder reading for %s: %d\n", joint.c_str(), encoderBefore);

  for (float testTorque = 10.0f; testTorque <= 150.0f; testTorque += 10.0f) {
    applyTestTorque(joint, testTorque);
    delay(500);
    const int encoderAfter = getEncoderReading(joint);
    dbPrintf("Encoder reading after torque %.1f for %s: %d\n", testTorque, joint.c_str(), encoderAfter);

    if (abs(encoderAfter - encoderBefore) > encoderThreshold) {
      const float multiplier = (encoderAfter > encoderBefore) ? 1.0f : -1.0f;
      setDirectionMultiplier(joint, multiplier);
      applyTestTorque(joint, 0.0f);
      endCalibrationOverride();
      saveConfig();
      dbPrintf("Torque direction for %s calibrated. Multiplier: %.1f\n", joint.c_str(), multiplier);
      return;
    }
  }

  dbPrintf("No movement detected with positive torque on %s. Trying negative torque.\n", joint.c_str());
  encoderBefore = getEncoderReading(joint);

  for (float testTorque = -10.0f; testTorque >= -150.0f; testTorque -= 10.0f) {
    applyTestTorque(joint, testTorque);
    delay(500);
    const int encoderAfter = getEncoderReading(joint);
    dbPrintf("Encoder reading after torque %.1f for %s: %d\n", testTorque, joint.c_str(), encoderAfter);

    if (abs(encoderAfter - encoderBefore) > encoderThreshold) {
      const float multiplier = (encoderAfter < encoderBefore) ? 1.0f : -1.0f;
      setDirectionMultiplier(joint, multiplier);
      applyTestTorque(joint, 0.0f);
      endCalibrationOverride();
      saveConfig();
      dbPrintf("Torque direction for %s calibrated. Multiplier: %.1f\n", joint.c_str(), multiplier);
      return;
    }
  }

  applyTestTorque(joint, 0.0f);
  endCalibrationOverride();
  dbPrintf("Joint %s could not be calibrated. Check mechanical lock, sensor motion, CAN, and actuator power.\n", joint.c_str());
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
      const int minValue = values.substring(0, comma).toInt();
      const int maxValue = values.substring(comma + 1).toInt();
      if (minValue <= maxValue) {
        constraints.minAngle = minValue;
        constraints.maxAngle = maxValue;
      }
    }
  }
  return constraints;
}

bool parseFiveInts(const String &line, const char *prefix, int out[5]) {
  if (!line.startsWith(prefix)) return false;
  String values = line.substring(strlen(prefix));
  int cursor = 0;
  for (int i = 0; i < 5; ++i) {
    int comma = values.indexOf(',', cursor);
    String token = (i == 4 || comma < 0) ? values.substring(cursor) : values.substring(cursor, comma);
    token.trim();
    if (!token.length()) return false;
    out[i] = token.toInt();
    if (i < 4) {
      if (comma < 0) return false;
      cursor = comma + 1;
    }
  }
  return true;
}

void applyConstraintLine(const String &line) {
  if (line.startsWith("outer_calf_left_Constraints")) outerCalfConstraintsLeft = loadJointConstraintsFromFile(line, outerCalfConstraintsLeft);
  else if (line.startsWith("outer_calf_right_Constraints")) outerCalfConstraintsRight = loadJointConstraintsFromFile(line, outerCalfConstraintsRight);
  else if (line.startsWith("inner_calf_left_Constraints")) innerCalfConstraintsLeft = loadJointConstraintsFromFile(line, innerCalfConstraintsLeft);
  else if (line.startsWith("inner_calf_right_Constraints")) innerCalfConstraintsRight = loadJointConstraintsFromFile(line, innerCalfConstraintsRight);
  else if (line.startsWith("knee_left_Constraints")) kneeConstraintsLeft = loadJointConstraintsFromFile(line, kneeConstraintsLeft);
  else if (line.startsWith("knee_right_Constraints")) kneeConstraintsRight = loadJointConstraintsFromFile(line, kneeConstraintsRight);
  else if (line.startsWith("hip_pitch_left_Constraints")) hipPitchConstraintsLeft = loadJointConstraintsFromFile(line, hipPitchConstraintsLeft);
  else if (line.startsWith("hip_pitch_right_Constraints")) hipPitchConstraintsRight = loadJointConstraintsFromFile(line, hipPitchConstraintsRight);
  else if (line.startsWith("hip_yaw_left_Constraints")) hipYawConstraintsLeft = loadJointConstraintsFromFile(line, hipYawConstraintsLeft);
  else if (line.startsWith("hip_yaw_right_Constraints")) hipYawConstraintsRight = loadJointConstraintsFromFile(line, hipYawConstraintsRight);
  else if (line.startsWith("hip_roll_left_Constraints")) hipRollConstraintsLeft = loadJointConstraintsFromFile(line, hipRollConstraintsLeft);
  else if (line.startsWith("hip_roll_right_Constraints")) hipRollConstraintsRight = loadJointConstraintsFromFile(line, hipRollConstraintsRight);
}

void saveConfig() {
  if (spiffsMutex && xSemaphoreTake(spiffsMutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
    dbPrintln("Failed to acquire SPIFFS lock for configuration save.");
    return;
  }

  File file = SPIFFS.open("/config.txt", FILE_WRITE);
  if (!file) {
    if (spiffsMutex) xSemaphoreGive(spiffsMutex);
    dbPrintln("Failed to open /config.txt for writing.");
    return;
  }

  // Keep the legacy field order first so a rollback to the pre-portal
  // firmware still reads role, offsets, directions, and all constraints.
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

  // Extended fields are appended after the legacy block.
  file.println("ConfigVersion:3");
  file.printf("MaxTorqueLimit:%.3f\n", maxTorqueLimit);

  file.close();
  if (spiffsMutex) xSemaphoreGive(spiffsMutex);
  configProvisioned = true;
  dbPrintln("Configuration saved.");
}

bool loadConfig() {
  configProvisioned = false;

  if (!SPIFFS.exists("/config.txt")) {
    dbPrintln("No /config.txt found. Starting captive setup portal; actuator tasks remain disabled until configured.");
    return false;
  }

  if (spiffsMutex && xSemaphoreTake(spiffsMutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
    dbPrintln("Failed to acquire SPIFFS lock for configuration load.");
    return false;
  }

  File file = SPIFFS.open("/config.txt", FILE_READ);
  if (!file) {
    if (spiffsMutex) xSemaphoreGive(spiffsMutex);
    dbPrintln("Failed to open /config.txt. Starting unconfigured.");
    return false;
  }

  bool validRole = false;
  while (file.available()) {
    String line = file.readStringUntil('\n');
    line.trim();
    if (!line.length()) continue;

    if (line.startsWith("LegSide:")) {
      String side = line.substring(8);
      side.trim();
      if (side == "left") {
        isLeft = true;
        isCenter = false;
        validRole = true;
      } else if (side == "right") {
        isLeft = false;
        isCenter = false;
        validRole = true;
      } else if (side == "center") {
        isCenter = true;
        validRole = true;
      }
    } else if (line.startsWith("MaxTorqueLimit:")) {
      const float value = line.substring(15).toFloat();
      if (value > 0.0f && value <= 100.0f) maxTorqueLimit = value;
    } else if (line.startsWith("LeftOffsets:")) {
      parseFiveInts(line, "LeftOffsets:", leftLegOffsets);
    } else if (line.startsWith("RightOffsets:")) {
      parseFiveInts(line, "RightOffsets:", rightLegOffsets);
    } else if (line.startsWith("DirectionMultipliers:")) {
      sscanf(line.c_str(), "DirectionMultipliers:%f,%f,%f,%f,%f,%f,%f,%f,%f,%f",
             &directionMultiplierRightOuterCalf, &directionMultiplierRightInnerCalf,
             &directionMultiplierLeftOuterCalf, &directionMultiplierLeftInnerCalf,
             &directionMultiplierRightKnee, &directionMultiplierLeftKnee,
             &directionMultiplierRightHipPitch, &directionMultiplierLeftHipPitch,
             &directionMultiplierRightHipRoll, &directionMultiplierLeftHipRoll);
    } else if (line.indexOf("_Constraints") >= 0 && line.indexOf("Constraints:") >= 0) {
      applyConstraintLine(line);
    }
  }

  file.close();
  if (spiffsMutex) xSemaphoreGive(spiffsMutex);

  configProvisioned = validRole;
  if (validRole) {
    dbPrintln("Configuration loaded successfully.");
  } else {
    dbPrintln("/config.txt exists but has no valid LegSide. Starting captive setup portal with actuator tasks disabled.");
  }
  return validRole;
}

void resetSPIFFS() {
  dbPrintln("Checking SPIFFS integrity...");
  if (SPIFFS.totalBytes() == 0) {
    dbPrintln("SPIFFS appears unformatted or corrupted. Type 'yes' to format.");
    while (!Serial.available()) delay(10);
    String response = Serial.readStringUntil('\n');
    response.trim();
    if (response.equalsIgnoreCase("yes")) {
      SPIFFS.format();
      dbPrintln("SPIFFS formatted.");
    } else {
      dbPrintln("SPIFFS reset aborted.");
    }
  } else {
    dbPrintln("SPIFFS is functioning properly.");
  }
}

// -----------------------------------------------------------------------------
// Joint constraints
// -----------------------------------------------------------------------------

void setJointConstraints(String jointName, int minAngle, int maxAngle) {
  if (minAngle > maxAngle) {
    dbPrintln("Constraint rejected: minAngle must be <= maxAngle.");
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
    dbPrintf("Unknown joint constraint name: %s\n", jointName.c_str());
    return;
  }

  saveConfig();
  dbPrintf("Joint constraints for %s set to min=%d max=%d\n", jointName.c_str(), minAngle, maxAngle);
}

void constrainJoint(String command) {
  const int firstSpace = command.indexOf(' ');
  const int secondSpace = command.indexOf(' ', firstSpace + 1);
  const int thirdSpace = command.indexOf(' ', secondSpace + 1);

  if (firstSpace < 0 || secondSpace < 0 || thirdSpace < 0) {
    dbPrintln("Usage: constrain <jointname> <minval> <maxval>");
    return;
  }

  const String jointName = command.substring(firstSpace + 1, secondSpace);
  const int minVal = command.substring(secondSpace + 1, thirdSpace).toInt();
  const int maxVal = command.substring(thirdSpace + 1).toInt();
  setJointConstraints(jointName, minVal, maxVal);
}

void configureJointConstraintsViaSerial() {
  dbPrintln("Enter joint name:");
  while (!Serial.available()) delay(10);
  String jointName = Serial.readStringUntil('\n');
  jointName.trim();

  dbPrintln("Enter min angle:");
  while (!Serial.available()) delay(10);
  String minString = Serial.readStringUntil('\n');
  minString.trim();

  dbPrintln("Enter max angle:");
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
  dbPrintln("Offsets reset to zero.");
  saveConfig();
}

void calibrateSensors(bool forceSave = false) {
  if (isCenter || !configProvisioned) {
    dbPrintln("Sensor calibration is only available on a configured leg controller.");
    return;
  }

  // Acquire an independent 10-sample calibration average. Do not mutate the
  // live moving-average ring while the 1 kHz sensor task is running.
  long sumOuter = 0, sumInner = 0, sumHip = 0, sumKnee = 0, sumButt = 0;
  for (int i = 0; i < NUM_READINGS; ++i) {
    sumOuter += analogRead(PIN_OUTER_CALF);
    sumInner += analogRead(PIN_INNER_CALF);
    sumHip += analogRead(PIN_HIP_PITCH);
    sumKnee += analogRead(PIN_KNEE);
    sumButt += analogRead(PIN_HIP_ROLL);
    delay(2);
  }

  const float rawOuter = adcToDegrees(static_cast<float>(sumOuter) / NUM_READINGS);
  const float rawInner = adcToDegrees(static_cast<float>(sumInner) / NUM_READINGS);
  const float rawHip = adcToDegrees(static_cast<float>(sumHip) / NUM_READINGS);
  const float rawKnee = adcToDegrees(static_cast<float>(sumKnee) / NUM_READINGS);
  const float rawButt = adcToDegrees(static_cast<float>(sumButt) / NUM_READINGS);

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

  dbPrintln("Calibration complete. Offsets aligning current pose to 180 degrees:");
  dbPrintf("Outer=%d Inner=%d HipPitch=%d Knee=%d HipRoll=%d\n",
           offsetOuter, offsetInner, offsetHip, offsetKnee, offsetButt);

  if (forceSave) {
    saveConfig();
    dbPrintln("Offsets saved.");
    return;
  }

  dbPrintln("Type 'yes' to save these offsets, anything else to leave them only in RAM.");
  while (!Serial.available()) delay(10);
  String response = Serial.readStringUntil('\n');
  response.trim();
  if (response.equalsIgnoreCase("yes")) {
    saveConfig();
    dbPrintln("Offsets saved.");
  } else {
    dbPrintln("Offsets not persisted.");
  }
}

void printSavedOffsets() {
  const int *offsets = isLeft ? leftLegOffsets : rightLegOffsets;
  dbPrintf("%s offsets - Outer:%d Inner:%d HipPitch:%d Knee:%d HipRoll:%d\n",
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
  dbPrintf("MAC Address: %02X:%02X:%02X:%02X:%02X:%02X\n",
                mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}

void printStatus() {
  dbPrintf("Mode: %s | chirality: %s | play:%d | raw:%d | maxTorque:%.2f A\n",
                configMode ? "config" : "normal",
                isCenter ? "center" : (isLeft ? "left" : "right"),
                playMode, rawMode, maxTorqueLimit);
  if (!isCenter) {
    dbPrintf("Angles: outer=%.1f inner=%.1f hipPitch=%.1f knee=%.1f hipRoll=%.1f\n",
                  normalizedOuter, normalizedInner, normalizedHip, normalizedKnee, normalizedButt);
  }
}

// -----------------------------------------------------------------------------
// Mode handling
// -----------------------------------------------------------------------------

void enterConfigurationMode() {
  dbPrintln("Entering Configuration Mode...");
  requestStop(3);
  delay(40); // allow CAN output task to emit the stop burst before configuration
  configMode = true;
  dbPrintln("Configuration mode entered. Type 'exit' to leave.");
}

void exitConfigurationMode() {
  configMode = false;
  dbPrintln("Configuration mode exited. Use 'play' explicitly to resume actuator output.");
}

void changeChirality(const String &side) {
  const String previousRole = selectedRoleName();

  if (configProvisioned && !isCenter) {
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
    dbPrintln("Invalid chirality.");
    return;
  }

  configProvisioned = true;
  saveConfig();

  const String newRole = selectedRoleName();
  dbPrintln("Chirality set to: " + newRole);

  if (newRole != roleAtBoot || previousRole == "unconfigured") {
    rebootRequired = true;
    runtimeControlReady = false;
    runtimeImuReady = false;
    playMode = false;
    dbPrintln("Runtime role changed. Control output is disabled until reboot instantiates the correct task topology.");
  }

  // The SoftAP identity always reflects the selected/persisted role when
  // portal support is compiled in.
  if (DROPBEAR_ENABLE_WIFI_PORTAL) requestPortalRestart();
}

void handleConfigurationCommand(String command) {
  command.trim();
  if (command == "exit") exitConfigurationMode();
  else if (command == "left" || command == "right" || command == "center") changeChirality(command);
  else if (command == "calibrate save") calibrateSensors(true);
  else if (command == "calibrate") calibrateSensors(false);
  else if (command == "save") saveConfig();
  else if (command == "status") printStatus();
  else dbPrintln("Unknown configuration command: " + command);
}

// -----------------------------------------------------------------------------
// Serial command parser
// -----------------------------------------------------------------------------

void processImpedanceCommand(const String &command) {
  if (!runtimeControlReady) {
    dbPrintln("Impedance command rejected: actuator control runtime is not active. Reboot after role configuration.");
    return;
  }

  String params = command.substring(10);
  params.trim();

  const int p1 = params.indexOf(' ');
  const int p2 = (p1 >= 0) ? params.indexOf(' ', p1 + 1) : -1;
  const int p3 = (p2 >= 0) ? params.indexOf(' ', p2 + 1) : -1;
  const int p4 = (p3 >= 0) ? params.indexOf(' ', p3 + 1) : -1;

  if (p1 < 0 || p2 < 0 || p3 < 0 || p4 < 0) {
    dbPrintln("Usage: impedance <legSide> <appendage> <0|1> <desiredPosition> <desiredVelocity>");
    return;
  }

  const String legSide = params.substring(0, p1);
  const String appendage = params.substring(p1 + 1, p2);
  const bool enable = params.substring(p2 + 1, p3).toInt() != 0;
  const float desiredPosition = params.substring(p3 + 1, p4).toFloat();
  const float desiredVelocity = params.substring(p4 + 1).toFloat();

  if ((isLeft && legSide != "left") || (!isLeft && !isCenter && legSide != "right") || isCenter) {
    dbPrintln("Impedance command does not match this controller's selected leg.");
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
    dbPrintln("Invalid appendage. Hip yaw remains direct-torque only because this firmware has no hip-yaw analog sensor input.");
    return;
  }

  *enabledFlag = enable;
  impedanceTorqueValues[index] = 0;
  control->reset();
  if (enable) {
    control->setDesiredPosition(desiredPosition);
    control->setDesiredVelocity(desiredVelocity);
  }

  dbPrintf("Impedance %s %s -> %d, position %.2f, velocity %.2f\n",
                legSide.c_str(), appendage.c_str(), enable, desiredPosition, desiredVelocity);
}

void processTorqueCommand(const String &command) {
  if (!runtimeControlReady) {
    dbPrintln("Torque command rejected: actuator control runtime is not active. Reboot after role configuration.");
    return;
  }

  String params = command.substring(7);
  params.trim();
  const int firstSpace = params.indexOf(' ');
  const int secondSpace = params.indexOf(' ', firstSpace + 1);

  if (firstSpace < 0 || secondSpace < 0) {
    dbPrintln("Usage: torque <legSide> <appendage> <torqueValue>");
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
    dbPrintln("Invalid torque appendage.");
    return;
  }
  if (!actuatorBelongsToSelectedLeg(index)) {
    dbPrintln("Torque command rejected: joint is not on this controller's selected leg.");
    return;
  }

  torqueValues[index] = torqueValue;
  dbPrintf("Torque for %s %s set to %d\n", legSide.c_str(), appendage.c_str(), torqueValue);
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
    dbPrintln("Raw mode enabled. Offsets bypassed.");
  } else if (command == "raw off") {
    rawMode = false;
    dbPrintln("Raw mode disabled. Offsets enabled.");
  } else if (command.startsWith("direction ")) {
    String params = command.substring(10);
    params.trim();
    const int split = params.indexOf(' ');
    if (split > 0) {
      const String joint = params.substring(0, split);
      const String direction = params.substring(split + 1);
      const float multiplier = (direction == "-") ? -1.0f : ((direction == "+") ? 1.0f : 0.0f);
      if (multiplier == 0.0f) dbPrintln("Direction must be + or -.");
      else {
        setDirectionMultiplier(joint, multiplier);
        saveConfig();
        dbPrintf("Direction for %s set to %s\n", joint.c_str(), direction.c_str());
      }
    } else dbPrintln("Usage: direction <joint> <+|->");
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
    dbPrintln("Current chirality: " + String(isCenter ? "center" : (isLeft ? "left" : "right")));
  } else if (command == "calibrate save") {
    xSemaphoreGive(serialMutex);
    calibrateSensors(true);
    return;
  } else if (command == "calibrate") {
    xSemaphoreGive(serialMutex);
    calibrateSensors(false);
    return;
  } else if (command == "save") {
    saveConfig();
  } else if (command == "saved") {
    printSavedOffsets();
  } else if (command == "play") {
    if (isCenter) {
      if (!runtimeImuReady) {
        dbPrintln("Play rejected: center/IMU runtime is not active. Reboot after role configuration.");
      } else {
        playMode = true;
        dbPrintln("Center play mode enabled (IMU streaming).");
      }
    } else {
      if (!runtimeControlReady) {
        dbPrintln("Play rejected: actuator control runtime is not active. Reboot after role configuration.");
      } else {
        stopBurstRemaining = 0;
        playMode = true;
        dbPrintln("Play mode enabled.");
      }
    }
  } else if (command == "stop") {
    requestStop(3);
    dbPrintln("Play mode disabled. Stop burst queued for this leg.");
  } else if (command == "zero") {
    clearAllTorqueSetpoints();
    dbPrintln("All direct and impedance torque setpoints cleared.");
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
    dbPrintln("Unknown command. Type 'help'.");
  }

  xSemaphoreGive(serialMutex);
}

// -----------------------------------------------------------------------------
// Help
// -----------------------------------------------------------------------------

void printHelp() {
  dbPrintln("Available Commands:");
  dbPrintln("  config");
  dbPrintln("      Enter configuration mode and stop this leg.");
  dbPrintln("  resetOffsets");
  dbPrintln("      Reset left and right stored offsets to zero.");
  dbPrintln("  raw on | raw off");
  dbPrintln("      Bypass or apply joint offsets.");
  dbPrintln("  direction <joint> <+|->");
  dbPrintln("      Example: direction right_outer_calf +");
  dbPrintln("  impedance <legSide> <appendage> <0|1> <desiredPosition> <desiredVelocity>");
  dbPrintln("      Example: impedance left knee 1 180 0");
  dbPrintln("      Supported: outer_calf inner_calf knee hip_pitch hip_roll");
  dbPrintln("  constrain <joint> <minAngle> <maxAngle>");
  dbPrintln("      Example: constrain right_knee 0 180");
  dbPrintln("  torque <legSide> <appendage> <torqueValue>");
  dbPrintln("      Example: torque left knee 100");
  dbPrintln("  calibrateDirection <joint>");
  dbPrintln("      Supported joints with external sensor feedback:");
  dbPrintln("      right/left_outer_calf, inner_calf, knee, hip_pitch, hip_roll");
  dbPrintln("  mac");
  dbPrintln("  chirality");
  dbPrintln("  calibrate");
  dbPrintln("      Interactive serial calibration; asks before saving.");
  dbPrintln("  calibrate save");
  dbPrintln("      Non-interactive calibration used by the captive portal; saves immediately.");
  dbPrintln("  save");
  dbPrintln("  saved");
  dbPrintln("  resetSPIFFS");
  dbPrintln("  play");
  dbPrintln("  stop");
  dbPrintln("  zero");
  dbPrintln("      Clear all torque setpoints without changing chirality/config.");
  dbPrintln("  status");
  dbPrintln("  setJointConstraints");
  dbPrintln("  left | right | center");
  dbPrintln("      Changes chirality; reboot recommended when switching center/leg roles.");
  dbPrintln("  help");
}


// -----------------------------------------------------------------------------
// Captive portal
// -----------------------------------------------------------------------------

String jsonEscape(const String &input) {
  String out;
  out.reserve(input.length() + 16);
  for (size_t i = 0; i < input.length(); ++i) {
    const char c = input[i];
    switch (c) {
      case '\\': out += "\\\\"; break;
      case '"': out += "\\\""; break;
      case '\n': out += "\\n"; break;
      case '\r': out += "\\r"; break;
      case '\t': out += "\\t"; break;
      default:
        if ((uint8_t)c >= 0x20) out += c;
        break;
    }
  }
  return out;
}

void sendNoCacheHeaders() {
  server.sendHeader("Cache-Control", "no-store, no-cache, must-revalidate, max-age=0");
  server.sendHeader("Pragma", "no-cache");
  server.sendHeader("Expires", "0");
}

void redirectToPortal() {
  server.sendHeader("Location", String("http://") + WiFi.softAPIP().toString() + "/");
  sendNoCacheHeaders();
  server.send(302, "text/plain", "Redirecting to Dropbear controller...");
}

String readSPIFFSTextFile(const String &path, size_t maxBytes = 32768) {
  if (!path.startsWith("/") || path.indexOf("..") >= 0) return "";
  if (spiffsMutex && xSemaphoreTake(spiffsMutex, pdMS_TO_TICKS(500)) != pdTRUE) return "";

  File file = SPIFFS.open(path, FILE_READ);
  if (!file) {
    if (spiffsMutex) xSemaphoreGive(spiffsMutex);
    return "";
  }

  String data;
  const size_t wanted = min((size_t)file.size(), maxBytes);
  data.reserve(wanted + 1);
  while (file.available() && data.length() < maxBytes) {
    data += (char)file.read();
  }
  file.close();
  if (spiffsMutex) xSemaphoreGive(spiffsMutex);
  return data;
}

String constraintJson(const JointConstraints &c) {
  return "[" + String(c.minAngle) + "," + String(c.maxAngle) + "]";
}

String buildStateJson() {
  String out;
  out.reserve(3200);
  out += "{";
  out += "\"role\":\"" + selectedRoleName() + "\",";
  out += "\"portal_enabled\":" + String(DROPBEAR_ENABLE_WIFI_PORTAL ? "true" : "false") + ",";
  out += "\"sensor_pinout\":\"" + String(DROPBEAR_ENABLE_WIFI_PORTAL ? "adc1_wifi" : "legacy_adc2") + "\",";
  out += "\"sensor_pins\":[" + String(PIN_OUTER_CALF) + "," + String(PIN_INNER_CALF) + "," +
         String(PIN_HIP_PITCH) + "," + String(PIN_KNEE) + "," + String(PIN_HIP_ROLL) + "],";
  out += "\"ssid\":\"" + jsonEscape(portalSSID) + "\",";
  out += "\"ip\":\"" + WiFi.softAPIP().toString() + "\",";
  out += "\"clients\":" + String(WiFi.softAPgetStationNum()) + ",";
  out += "\"configured\":" + String(configProvisioned ? "true" : "false") + ",";
  out += "\"control_ready\":" + String(runtimeControlReady ? "true" : "false") + ",";
  out += "\"imu_ready\":" + String(runtimeImuReady ? "true" : "false") + ",";
  out += "\"reboot_required\":" + String(rebootRequired ? "true" : "false") + ",";
  out += "\"play\":" + String(playMode ? "true" : "false") + ",";
  out += "\"raw\":" + String(rawMode ? "true" : "false") + ",";
  out += "\"config_mode\":" + String(configMode ? "true" : "false") + ",";
  out += "\"max_torque\":" + String(maxTorqueLimit, 3) + ",";
  out += "\"heap\":" + String(ESP.getFreeHeap()) + ",";
  out += "\"uptime_ms\":" + String(millis()) + ",";
  out += "\"stop_burst\":" + String(stopBurstRemaining) + ",";
  out += "\"calibration_override\":" + String(calibrationOverrideActive ? "true" : "false") + ",";

  out += "\"angles\":{";
  out += "\"outer_calf\":" + String(normalizedOuter, 2) + ",";
  out += "\"inner_calf\":" + String(normalizedInner, 2) + ",";
  out += "\"hip_pitch\":" + String(normalizedHip, 2) + ",";
  out += "\"knee\":" + String(normalizedKnee, 2) + ",";
  out += "\"hip_roll\":" + String(normalizedButt, 2);
  out += "},";

  out += "\"torque\":[";
  for (int i = 0; i < ACTUATOR_COUNT; ++i) {
    if (i) out += ",";
    out += String(torqueValues[i]);
  }
  out += "],";

  out += "\"impedance_torque\":[";
  for (int i = 0; i < ACTUATOR_COUNT; ++i) {
    if (i) out += ",";
    out += String(impedanceTorqueValues[i]);
  }
  out += "],";

  out += "\"impedance_enabled\":[";
  for (int i = 0; i < ACTUATOR_COUNT; ++i) {
    if (i) out += ",";
    out += isImpedanceEnabled(i) ? "true" : "false";
  }
  out += "],";

  out += "\"actuator_ids\":[";
  for (int i = 0; i < ACTUATOR_COUNT; ++i) {
    if (i) out += ",";
    out += String(ACTUATOR_IDS[i]);
  }
  out += "]";

  out += "}";
  return out;
}

String buildConfigJson() {
  String out;
  out.reserve(5000);
  out += "{";
  out += "\"role\":\"" + selectedRoleName() + "\",";
  out += "\"configured\":" + String(configProvisioned ? "true" : "false") + ",";
  out += "\"max_torque\":" + String(maxTorqueLimit, 3) + ",";

  out += "\"left_offsets\":[";
  for (int i = 0; i < 5; ++i) { if (i) out += ","; out += String(leftLegOffsets[i]); }
  out += "],\"right_offsets\":[";
  for (int i = 0; i < 5; ++i) { if (i) out += ","; out += String(rightLegOffsets[i]); }
  out += "],";

  const float dirs[10] = {
    directionMultiplierRightOuterCalf, directionMultiplierRightInnerCalf,
    directionMultiplierLeftOuterCalf, directionMultiplierLeftInnerCalf,
    directionMultiplierRightKnee, directionMultiplierLeftKnee,
    directionMultiplierRightHipPitch, directionMultiplierLeftHipPitch,
    directionMultiplierRightHipRoll, directionMultiplierLeftHipRoll
  };
  out += "\"directions\":[";
  for (int i = 0; i < 10; ++i) { if (i) out += ","; out += String(dirs[i], 1); }
  out += "],";

  out += "\"constraints\":{";
  out += "\"outer_calf_left\":" + constraintJson(outerCalfConstraintsLeft) + ",";
  out += "\"outer_calf_right\":" + constraintJson(outerCalfConstraintsRight) + ",";
  out += "\"inner_calf_left\":" + constraintJson(innerCalfConstraintsLeft) + ",";
  out += "\"inner_calf_right\":" + constraintJson(innerCalfConstraintsRight) + ",";
  out += "\"knee_left\":" + constraintJson(kneeConstraintsLeft) + ",";
  out += "\"knee_right\":" + constraintJson(kneeConstraintsRight) + ",";
  out += "\"hip_pitch_left\":" + constraintJson(hipPitchConstraintsLeft) + ",";
  out += "\"hip_pitch_right\":" + constraintJson(hipPitchConstraintsRight) + ",";
  out += "\"hip_yaw_left\":" + constraintJson(hipYawConstraintsLeft) + ",";
  out += "\"hip_yaw_right\":" + constraintJson(hipYawConstraintsRight) + ",";
  out += "\"hip_roll_left\":" + constraintJson(hipRollConstraintsLeft) + ",";
  out += "\"hip_roll_right\":" + constraintJson(hipRollConstraintsRight);
  out += "},";

  out += "\"raw_config\":\"" + jsonEscape(readSPIFFSTextFile("/config.txt")) + "\"";
  out += "}";
  return out;
}

void parseConstraintArg(const char *name, JointConstraints &c) {
  String minKey = String(name) + "_min";
  String maxKey = String(name) + "_max";
  if (!server.hasArg(minKey) || !server.hasArg(maxKey)) return;
  int mn = server.arg(minKey).toInt();
  int mx = server.arg(maxKey).toInt();
  if (mn <= mx) {
    c.minAngle = mn;
    c.maxAngle = mx;
  }
}

bool writeRawConfigWithBackup(const String &body) {
  if (!body.length() || body.length() > 16384) return false;
  if (spiffsMutex && xSemaphoreTake(spiffsMutex, pdMS_TO_TICKS(1000)) != pdTRUE) return false;

  if (SPIFFS.exists("/config.txt")) {
    File srcFile = SPIFFS.open("/config.txt", FILE_READ);
    File bakFile = SPIFFS.open("/config.bak", FILE_WRITE);
    if (srcFile && bakFile) {
      while (srcFile.available()) bakFile.write(srcFile.read());
    }
    if (srcFile) srcFile.close();
    if (bakFile) bakFile.close();
  }

  File file = SPIFFS.open("/config.txt", FILE_WRITE);
  bool ok = false;
  if (file) {
    ok = file.print(body) == body.length();
    file.close();
  }

  if (spiffsMutex) xSemaphoreGive(spiffsMutex);
  return ok;
}

void handleApiState() {
  sendNoCacheHeaders();
  server.send(200, "application/json", buildStateJson());
}

void handleApiConfigGet() {
  sendNoCacheHeaders();
  server.send(200, "application/json", buildConfigJson());
}

void handleApiConfigPost() {
  sendNoCacheHeaders();

  const String oldRole = selectedRoleName();
  if (configProvisioned && !isCenter) {
    requestStop(3);
    delay(35);
  }
  clearAllTorqueSetpoints();

  if (server.hasArg("role")) {
    String role = server.arg("role");
    role.trim();
    if (role == "left") {
      isLeft = true; isCenter = false; configProvisioned = true;
    } else if (role == "right") {
      isLeft = false; isCenter = false; configProvisioned = true;
    } else if (role == "center") {
      isCenter = true; configProvisioned = true;
    }
  }

  if (server.hasArg("max_torque")) {
    float v = server.arg("max_torque").toFloat();
    if (v > 0.0f && v <= 100.0f) maxTorqueLimit = v;
  }

  for (int i = 0; i < 5; ++i) {
    String lk = "lo" + String(i);
    String rk = "ro" + String(i);
    if (server.hasArg(lk)) leftLegOffsets[i] = server.arg(lk).toInt();
    if (server.hasArg(rk)) rightLegOffsets[i] = server.arg(rk).toInt();
  }

  float *dirs[10] = {
    &directionMultiplierRightOuterCalf, &directionMultiplierRightInnerCalf,
    &directionMultiplierLeftOuterCalf, &directionMultiplierLeftInnerCalf,
    &directionMultiplierRightKnee, &directionMultiplierLeftKnee,
    &directionMultiplierRightHipPitch, &directionMultiplierLeftHipPitch,
    &directionMultiplierRightHipRoll, &directionMultiplierLeftHipRoll
  };
  for (int i = 0; i < 10; ++i) {
    String key = "dm" + String(i);
    if (server.hasArg(key)) {
      float v = server.arg(key).toFloat();
      *dirs[i] = (v < 0.0f) ? -1.0f : 1.0f;
    }
  }

  parseConstraintArg("outer_calf_left", outerCalfConstraintsLeft);
  parseConstraintArg("outer_calf_right", outerCalfConstraintsRight);
  parseConstraintArg("inner_calf_left", innerCalfConstraintsLeft);
  parseConstraintArg("inner_calf_right", innerCalfConstraintsRight);
  parseConstraintArg("knee_left", kneeConstraintsLeft);
  parseConstraintArg("knee_right", kneeConstraintsRight);
  parseConstraintArg("hip_pitch_left", hipPitchConstraintsLeft);
  parseConstraintArg("hip_pitch_right", hipPitchConstraintsRight);
  parseConstraintArg("hip_yaw_left", hipYawConstraintsLeft);
  parseConstraintArg("hip_yaw_right", hipYawConstraintsRight);
  parseConstraintArg("hip_roll_left", hipRollConstraintsLeft);
  parseConstraintArg("hip_roll_right", hipRollConstraintsRight);

  saveConfig();
  String newRole = selectedRoleName();
  if (oldRole != newRole || newRole != roleAtBoot) {
    rebootRequired = true;
    runtimeControlReady = false;
    runtimeImuReady = false;
    playMode = false;
  }

  String out = "{\"ok\":true,\"role\":\"" + newRole + "\",\"ssid\":\"" +
               desiredPortalSSID() + "\",\"reboot_required\":" +
               String(rebootRequired ? "true" : "false") + "}";
  server.send(200, "application/json", out);
}

void handleApiConfigReload() {
  sendNoCacheHeaders();
  const String oldRole = selectedRoleName();
  if (runtimeControlReady) {
    requestStop(3);
    delay(35);
  }
  clearAllTorqueSetpoints();
  loadConfig();
  if (selectedRoleName() != oldRole || selectedRoleName() != roleAtBoot) {
    rebootRequired = true;
    runtimeControlReady = false;
    runtimeImuReady = false;
    playMode = false;
  }
  server.send(200, "application/json", buildConfigJson());
}

void handleApiRawConfigPost() {
  sendNoCacheHeaders();
  if (runtimeControlReady) {
    requestStop(3);
    delay(35);
  }
  clearAllTorqueSetpoints();

  String body = server.hasArg("plain") ? server.arg("plain") : "";
  if (!body.length() && server.hasArg("body")) body = server.arg("body");
  if (!body.length()) {
    server.send(400, "application/json", "{\"ok\":false,\"error\":\"empty_body\"}");
    return;
  }

  const String oldRole = selectedRoleName();
  if (!writeRawConfigWithBackup(body)) {
    server.send(500, "application/json", "{\"ok\":false,\"error\":\"write_failed\"}");
    return;
  }

  loadConfig();
  rebootRequired = true;
  runtimeControlReady = false;
  runtimeImuReady = false;
  playMode = false;
  server.send(200, "application/json",
              "{\"ok\":true,\"reboot_required\":true,\"ssid\":\"" +
              desiredPortalSSID() + "\"}");
}

void handleApiCommand() {
  sendNoCacheHeaders();
  if (!server.hasArg("cmd")) {
    server.send(400, "application/json", "{\"ok\":false,\"error\":\"missing_cmd\"}");
    return;
  }

  String command = server.arg("cmd");
  command.trim();

  // Web commands cannot satisfy legacy blocking Serial prompts. Preserve the
  // command surface while converting sensor calibration to its noninteractive
  // save form. Other interactive-only operations are rejected below.
  if (command == "calibrate") command = "calibrate save";

  if (!command.length() || command.length() >= WEB_COMMAND_MAX) {
    server.send(400, "application/json", "{\"ok\":false,\"error\":\"bad_length\"}");
    return;
  }

  // These two legacy interactive commands wait synchronously for USB Serial
  // follow-up input. Equivalent noninteractive web paths exist.
  if (command == "setJointConstraints" || command == "resetSPIFFS") {
    server.send(409, "application/json",
                "{\"ok\":false,\"error\":\"interactive_serial_only\",\"hint\":\"use the Configuration/SPIFFS UI\"}");
    return;
  }

  if (!webCommandQueue) {
    server.send(503, "application/json", "{\"ok\":false,\"error\":\"queue_unavailable\"}");
    return;
  }

  WebCommand item{};
  item.id = nextWebCommandId++;
  command.toCharArray(item.text, sizeof(item.text));

  if (xQueueSend(webCommandQueue, &item, 0) != pdTRUE) {
    server.send(503, "application/json", "{\"ok\":false,\"error\":\"queue_full\"}");
    return;
  }

  server.send(202, "application/json",
              "{\"ok\":true,\"queued\":true,\"id\":" + String(item.id) + "}");
}

void handleApiLog() {
  sendNoCacheHeaders();
  uint32_t since = server.hasArg("since") ? (uint32_t)server.arg("since").toInt() : 0;

  String out;
  out.reserve(8192);
  out += "{\"latest\":" + String(webLogSequence) + ",\"entries\":[";

  if (webLogMutex) xSemaphoreTake(webLogMutex, pdMS_TO_TICKS(100));
  size_t oldest = (webLogHead + WEB_LOG_CAPACITY - webLogCount) % WEB_LOG_CAPACITY;
  bool first = true;
  for (size_t i = 0; i < webLogCount; ++i) {
    const WebLogEntry &entry = webLog[(oldest + i) % WEB_LOG_CAPACITY];
    if (entry.seq <= since) continue;
    if (!first) out += ",";
    first = false;
    out += "{\"seq\":" + String(entry.seq) + ",\"ms\":" + String(entry.ms) +
           ",\"text\":\"" + jsonEscape(entry.text) + "\"}";
  }
  if (webLogMutex) xSemaphoreGive(webLogMutex);

  out += "]}";
  server.send(200, "application/json", out);
}

void handleApiSPIFFSList() {
  sendNoCacheHeaders();
  String out = "{\"total\":" + String(SPIFFS.totalBytes()) +
               ",\"used\":" + String(SPIFFS.usedBytes()) + ",\"files\":[";
  File root = SPIFFS.open("/");
  File file = root.openNextFile();
  bool first = true;
  while (file) {
    if (!first) out += ",";
    first = false;
    out += "{\"name\":\"" + jsonEscape(String(file.name())) +
           "\",\"size\":" + String(file.size()) + "}";
    file = root.openNextFile();
  }
  out += "]}";
  server.send(200, "application/json", out);
}

void handleApiSPIFFSRead() {
  sendNoCacheHeaders();
  String path = server.hasArg("path") ? server.arg("path") : "";
  if (!path.startsWith("/") || path.indexOf("..") >= 0) {
    server.send(400, "text/plain", "Invalid path");
    return;
  }
  if (!SPIFFS.exists(path)) {
    server.send(404, "text/plain", "Not found");
    return;
  }
  server.send(200, "text/plain", readSPIFFSTextFile(path));
}

void handleApiSensorCalibration() {
  sendNoCacheHeaders();
  if (!configProvisioned || isCenter) {
    server.send(409, "application/json", "{\"ok\":false,\"error\":\"not_leg_controller\"}");
    return;
  }
  if (!webCommandQueue) {
    server.send(503, "application/json", "{\"ok\":false}");
    return;
  }

  WebCommand item{};
  item.id = nextWebCommandId++;
  String("calibrate save").toCharArray(item.text, sizeof(item.text));
  if (xQueueSend(webCommandQueue, &item, 0) != pdTRUE) {
    server.send(503, "application/json", "{\"ok\":false,\"error\":\"queue_full\"}");
    return;
  }
  server.send(202, "application/json", "{\"ok\":true,\"queued\":true}");
}

void handleApiReboot() {
  sendNoCacheHeaders();
  if (configProvisioned && !isCenter) requestStop(3);
  portalRebootAtMs = millis() + 800;
  portalRebootRequested = true;
  server.send(202, "application/json",
              "{\"ok\":true,\"rebooting\":true,\"next_ssid\":\"" + desiredPortalSSID() + "\"}");
}

static const char PORTAL_HTML[] PROGMEM = R"DBHTML(
<!doctype html>
<html>
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1,viewport-fit=cover">
<meta name="apple-mobile-web-app-capable" content="yes">
<title>Dropbear Controller</title>
<style>
:root{--bg:#090b0d;--panel:#101316;--panel2:#14181c;--line:#262c31;--text:#e8ecef;--muted:#89939b;--ok:#7ca47d;--warn:#c9a35a;--bad:#c56d6d;--accent:#b9c2c8}
*{box-sizing:border-box}html,body{margin:0;background:var(--bg);color:var(--text);font-family:Inter,ui-sans-serif,system-ui,-apple-system,BlinkMacSystemFont,"Segoe UI",sans-serif}
body{padding:18px;min-height:100vh}.shell{max-width:1180px;margin:0 auto}.top{display:flex;justify-content:space-between;align-items:flex-start;gap:14px;flex-wrap:wrap;margin-bottom:16px}
h1{font-size:19px;letter-spacing:.12em;margin:0 0 6px;font-weight:700}h2{font-size:14px;margin:0 0 12px;letter-spacing:.08em;text-transform:uppercase;color:#cdd4d8}
.sub{color:var(--muted);font-size:12px}.badges{display:flex;gap:7px;flex-wrap:wrap}.badge{border:1px solid var(--line);background:var(--panel);padding:5px 8px;border-radius:4px;font-size:11px}
.badge.ok{border-color:#3c5840;color:#b5d5b6}.badge.bad{border-color:#673f3f;color:#efadad}.badge.warn{border-color:#625032;color:#e6c381}
.tabs{display:flex;border-bottom:1px solid var(--line);margin-bottom:14px;overflow:auto}.tab{border:0;background:none;color:var(--muted);padding:10px 14px;cursor:pointer;border-bottom:2px solid transparent;white-space:nowrap}
.tab.active{color:var(--text);border-bottom-color:#aeb6bb}.view{display:none}.view.active{display:block}
.grid{display:grid;grid-template-columns:repeat(12,1fr);gap:10px}.card{background:var(--panel);border:1px solid var(--line);border-radius:5px;padding:13px}.span12{grid-column:span 12}.span8{grid-column:span 8}.span6{grid-column:span 6}.span4{grid-column:span 4}.span3{grid-column:span 3}
.metric .v{font-size:24px;font-variant-numeric:tabular-nums;margin-top:3px}.metric .k{font-size:10px;color:var(--muted);text-transform:uppercase;letter-spacing:.08em}
.row{display:flex;gap:8px;align-items:center;flex-wrap:wrap}.between{justify-content:space-between}.stack{display:flex;flex-direction:column;gap:9px}
button,.btn{border:1px solid #343b41;background:#1a1f23;color:var(--text);padding:8px 11px;border-radius:4px;cursor:pointer;font-size:12px}
button:hover{background:#22282d}.primary{background:#d4d9dc;color:#0b0d0f;border-color:#d4d9dc}.danger{background:#411f22;border-color:#70373d;color:#ffc5c8}.warn{background:#3a3020;border-color:#665335;color:#efd39d}
input,select,textarea{background:#0c0f11;color:var(--text);border:1px solid #30363b;border-radius:3px;padding:8px;font-size:12px;min-width:0}input[type=number]{width:88px}select{min-width:96px}
textarea{width:100%;min-height:220px;font-family:ui-monospace,SFMono-Regular,Menlo,Consolas,monospace;resize:vertical}.joint{display:grid;grid-template-columns:120px 80px 1fr 1fr;gap:8px;align-items:center;padding:8px 0;border-bottom:1px solid #20252a}
.joint:last-child{border-bottom:0}.jointname{font-size:12px}.canid{color:var(--muted);font-family:monospace;font-size:11px}.tiny{font-size:10px;color:var(--muted)}
table{width:100%;border-collapse:collapse;font-size:11px}th,td{text-align:left;border-bottom:1px solid #22282c;padding:7px 5px}th{color:var(--muted);font-weight:500}
.console{background:#050607;border:1px solid #252a2e;border-radius:4px;height:360px;overflow:auto;padding:10px;font:11px/1.55 ui-monospace,SFMono-Regular,Menlo,Consolas,monospace;white-space:pre-wrap}
.notice{padding:9px;border:1px solid #654d2b;background:#2d2519;color:#e6c58d;border-radius:4px;font-size:11px}.okmsg{padding:9px;border:1px solid #38523b;background:#172219;color:#acd0ae;border-radius:4px;font-size:11px}
.section-title{font-size:11px;color:#aeb7bd;text-transform:uppercase;letter-spacing:.08em;margin:8px 0}.cfgline{display:grid;grid-template-columns:170px repeat(5,1fr);gap:6px;align-items:center}.constraints{display:grid;grid-template-columns:repeat(2,minmax(260px,1fr));gap:8px}
.constraint{display:grid;grid-template-columns:1fr 82px 82px;gap:6px;align-items:center}.file{display:flex;justify-content:space-between;gap:8px;padding:7px 0;border-bottom:1px solid #22282c;font-size:11px}
@media(max-width:800px){.span8,.span6,.span4,.span3{grid-column:span 12}.joint{grid-template-columns:1fr 80px}.joint .controls{grid-column:1/-1}.cfgline{grid-template-columns:1fr repeat(2,1fr)}.cfgline input:nth-of-type(n+3){margin-top:2px}.constraints{grid-template-columns:1fr}}
</style>
</head>
<body>
<div class="shell">
 <div class="top">
  <div><h1>DROPBEAR // <span id="roleTitle">...</span></h1><div class="sub">ESP32 low-level control · captive portal <span id="ipText"></span></div></div>
  <div class="badges">
   <span class="badge" id="playBadge">STATE</span><span class="badge" id="canBadge">ROLE</span><span class="badge" id="clientBadge">0 CLIENTS</span><span class="badge" id="heapBadge">HEAP</span>
  </div>
 </div>
 <div id="globalNotice" class="notice" style="display:none"></div>
 <div class="tabs">
  <button class="tab active" data-tab="control">Control</button>
  <button class="tab" data-tab="config">Configuration</button>
  <button class="tab" data-tab="terminal">Terminal</button>
  <button class="tab" data-tab="spiffs">SPIFFS</button>
 </div>

 <section id="control" class="view active">
  <div class="grid">
   <div class="card span12"><div class="row between"><div class="row"><button class="danger" onclick="cmd('stop')">STOP</button><button class="primary" onclick="cmd('play')">PLAY</button><button onclick="cmd('zero')">ZERO TORQUE</button><button onclick="cmd('status')">STATUS</button></div><div class="tiny">All web commands pass through the same command queue/parser as USB Serial.</div></div></div>
   <div class="card span12">
    <h2>Joint state</h2>
    <div class="grid">
     <div class="metric span3"><div class="k">Outer calf</div><div class="v" id="aOuter">—</div></div>
     <div class="metric span3"><div class="k">Inner calf</div><div class="v" id="aInner">—</div></div>
     <div class="metric span3"><div class="k">Hip pitch</div><div class="v" id="aHip">—</div></div>
     <div class="metric span3"><div class="k">Knee</div><div class="v" id="aKnee">—</div></div>
     <div class="metric span3"><div class="k">Hip roll</div><div class="v" id="aRoll">—</div></div>
    </div>
   </div>
   <div class="card span12">
    <div class="row between"><h2>Actuator control</h2><span class="tiny">Torque values are firmware command units; global clamp follows MaxTorqueLimit × 100.</span></div>
    <div id="jointControls"></div>
   </div>
  </div>
 </section>

 <section id="config" class="view">
  <div class="grid">
   <div class="card span12"><h2>Controller identity</h2><div class="row">
    <label>Role <select id="cfgRole"><option value="left">LEFTLEG</option><option value="right">RIGHTLEG</option><option value="center">CENTER</option></select></label>
    <label>Max torque <input id="cfgMaxTorque" type="number" step=".1" min=".1" max="100"></label>
    <button class="primary" onclick="saveConfig()">Save configuration</button>
    <button onclick="reloadConfig()">Reload SPIFFS</button>
    <button class="warn" onclick="saveAndReboot()">Save + reboot</button>
   </div><div class="tiny" style="margin-top:8px">Changing role changes the AP identity. Runtime task topology is recreated only after reboot.</div></div>

   <div class="card span12"><h2>Sensor offsets</h2>
    <div class="cfgline"><div></div><div>Outer</div><div>Inner</div><div>Hip pitch</div><div>Knee</div><div>Hip roll</div></div>
    <div class="cfgline"><div>Left</div><input id="lo0" type="number"><input id="lo1" type="number"><input id="lo2" type="number"><input id="lo3" type="number"><input id="lo4" type="number"></div>
    <div class="cfgline"><div>Right</div><input id="ro0" type="number"><input id="ro1" type="number"><input id="ro2" type="number"><input id="ro3" type="number"><input id="ro4" type="number"></div>
    <div class="row" style="margin-top:10px"><button onclick="calibrateSensors()">Calibrate selected leg + save</button><button onclick="cmd('raw on')">Raw on</button><button onclick="cmd('raw off')">Raw off</button></div>
   </div>

   <div class="card span12"><h2>Direction multipliers</h2><div id="dirFields" class="constraints"></div></div>
   <div class="card span12"><h2>Joint constraints</h2><div id="constraintFields" class="constraints"></div><div class="tiny" style="margin-top:8px">Current firmware applies these bounds to impedance control; direct torque remains unconstrained by angle.</div></div>

   <div class="card span12"><div class="row between"><h2>Raw /config.txt</h2><button onclick="writeRawConfig()">Write raw file + reload</button></div>
    <textarea id="rawConfig"></textarea>
    <div class="tiny">Writing raw configuration first creates /config.bak. A reboot is required after raw configuration changes.</div>
   </div>
  </div>
 </section>

 <section id="terminal" class="view">
  <div class="card"><div class="row"><input id="termInput" type="text" style="flex:1" placeholder="status, torque left knee 10, impedance left knee 1 180 0 ..."><button class="primary" onclick="sendTerminal()">Send</button></div>
   <div class="row" style="margin:9px 0"><button onclick="cmd('help')">help</button><button onclick="cmd('status')">status</button><button onclick="cmd('saved')">saved</button><button onclick="cmd('chirality')">chirality</button></div>
   <div id="console" class="console"></div>
  </div>
 </section>

 <section id="spiffs" class="view">
  <div class="grid"><div class="card span4"><div class="row between"><h2>Files</h2><button onclick="loadFiles()">Refresh</button></div><div id="fileList"></div></div>
  <div class="card span8"><h2 id="fileTitle">SPIFFS viewer</h2><textarea id="fileViewer" readonly></textarea></div></div>
 </section>
</div>
<script>
const jointNames=['outer_calf','inner_calf','knee','hip_pitch','hip_yaw','hip_roll'];
const idx={right:[0,2,4,6,8,10],left:[1,3,5,7,9,11]};
const ids={right:['0x144','0x143','0x148','0x147','0x14C','0x14B'],left:['0x141','0x142','0x145','0x146','0x149','0x14A']};
const sensed=new Set(['outer_calf','inner_calf','knee','hip_pitch','hip_roll']);
const dirNames=['right_outer_calf','right_inner_calf','left_outer_calf','left_inner_calf','right_knee','left_knee','right_hip_pitch','left_hip_pitch','right_hip_roll','left_hip_roll'];
const constraintNames=['outer_calf_left','outer_calf_right','inner_calf_left','inner_calf_right','knee_left','knee_right','hip_pitch_left','hip_pitch_right','hip_yaw_left','hip_yaw_right','hip_roll_left','hip_roll_right'];
let state=null,config=null,lastLog=0,renderedRole='';

document.querySelectorAll('.tab').forEach(b=>b.onclick=()=>{document.querySelectorAll('.tab').forEach(x=>x.classList.remove('active'));document.querySelectorAll('.view').forEach(x=>x.classList.remove('active'));b.classList.add('active');document.getElementById(b.dataset.tab).classList.add('active')});
document.getElementById('termInput').addEventListener('keydown',e=>{if(e.key==='Enter')sendTerminal()});

function notice(t,good=false){const n=document.getElementById('globalNotice');n.style.display=t?'block':'none';n.className=good?'okmsg':'notice';n.textContent=t||''}
function fmtDeg(v){return Number.isFinite(+v)?(+v).toFixed(1)+'°':'—'}
async function jfetch(url,opt){const r=await fetch(url,opt);let data;const ct=r.headers.get('content-type')||'';data=ct.includes('json')?await r.json():await r.text();if(!r.ok)throw new Error(typeof data==='string'?data:(data.error||r.status));return data}

function renderJoints(){
 const box=document.getElementById('jointControls');
 const side=state&&['left','right'].includes(state.role)?state.role:'left';
 if(renderedRole===side && box.children.length){
  jointNames.forEach((j,k)=>{
   const i=idx[side][k],tn=document.getElementById('tn_'+j);if(tn)tn.textContent=state?.torque?.[i]??0;
   const ie=document.getElementById('ie_'+j);if(ie&&document.activeElement!==ie)ie.checked=!!state?.impedance_enabled?.[i];
  });
  return;
 }
 renderedRole=side;box.innerHTML='';
 jointNames.forEach((j,k)=>{
  const i=idx[side][k], enabled=state?.impedance_enabled?.[i];
  const d=document.createElement('div');d.className='joint';
  d.innerHTML=`<div><div class="jointname">${j.replaceAll('_',' ')}</div><div class="canid">${ids[side][k]}</div></div>
   <div><span class="tiny">τ now</span><br><span id="tn_${j}">${state?.torque?.[i]??0}</span></div>
   <div class="controls row"><input id="t_${j}" type="number" value="${state?.torque?.[i]??0}" placeholder="torque"><button onclick="sendTorque('${j}')">Set torque</button></div>
   <div class="controls row">${sensed.has(j)?`<label><input id="ie_${j}" type="checkbox" ${enabled?'checked':''}> impedance</label><input id="ip_${j}" type="number" step=".1" value="180" placeholder="position"><input id="iv_${j}" type="number" step=".1" value="0" placeholder="velocity"><button onclick="sendImpedance('${j}')">Apply</button>`:'<span class="tiny">Direct torque only</span>'}</div>`;
  box.appendChild(d);
 });
}
async function loadState(){
 try{
  state=await jfetch('/api/state');
  document.getElementById('roleTitle').textContent=(state.role||'unconfigured').toUpperCase();
  document.getElementById('ipText').textContent='@ '+state.ip+' · '+state.ssid+' · sensors GPIO '+(state.sensor_pins||[]).join('/');
  const pb=document.getElementById('playBadge');pb.textContent=state.play?'PLAY':'STOPPED';pb.className='badge '+(state.play?'ok':'bad');
  const cb=document.getElementById('canBadge');const ready=state.control_ready||state.imu_ready;cb.textContent=ready?'RUNTIME READY':(state.configured?'REBOOT REQUIRED':'SETUP');cb.className='badge '+(ready?'ok':'warn');
  document.getElementById('clientBadge').textContent=state.clients+' CLIENT'+(state.clients===1?'':'S');
  document.getElementById('heapBadge').textContent=Math.round(state.heap/1024)+' KB HEAP';
  document.getElementById('aOuter').textContent=fmtDeg(state.angles.outer_calf);document.getElementById('aInner').textContent=fmtDeg(state.angles.inner_calf);document.getElementById('aHip').textContent=fmtDeg(state.angles.hip_pitch);document.getElementById('aKnee').textContent=fmtDeg(state.angles.knee);document.getElementById('aRoll').textContent=fmtDeg(state.angles.hip_roll);
  if(!state.configured)notice('Controller is not configured. Actuator tasks are disabled. Select a role under Configuration, save, then reboot.');
  else if(state.reboot_required)notice('Configuration role differs from the boot topology. Reboot is required before control topology matches the saved role.');
  else notice('');
  renderJoints();
 }catch(e){}
}
async function cmd(c){try{await jfetch('/api/command',{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},body:new URLSearchParams({cmd:c})});}catch(e){notice('Command failed: '+e.message)}}
function sendTerminal(){const e=document.getElementById('termInput');const c=e.value.trim();if(c){cmd(c);e.value=''}}
function sendTorque(j){if(!state||!['left','right'].includes(state.role))return notice('Configure a leg role first.');cmd(`torque ${state.role} ${j} ${document.getElementById('t_'+j).value}`)}
function sendImpedance(j){if(!state||!['left','right'].includes(state.role))return notice('Configure a leg role first.');const on=document.getElementById('ie_'+j).checked?1:0;cmd(`impedance ${state.role} ${j} ${on} ${document.getElementById('ip_'+j).value} ${document.getElementById('iv_'+j).value}`)}

function buildConfigFields(){
 const df=document.getElementById('dirFields');df.innerHTML='';dirNames.forEach((n,i)=>{const d=document.createElement('div');d.className='constraint';d.innerHTML=`<span>${n.replaceAll('_',' ')}</span><select id="dm${i}"><option value="1">+</option><option value="-1">−</option></select><span></span>`;df.appendChild(d)});
 const cf=document.getElementById('constraintFields');cf.innerHTML='';constraintNames.forEach(n=>{const d=document.createElement('div');d.className='constraint';d.innerHTML=`<span>${n.replaceAll('_',' ')}</span><input id="${n}_min" type="number" placeholder="min"><input id="${n}_max" type="number" placeholder="max">`;cf.appendChild(d)});
}
async function loadConfig(){
 try{
  config=await jfetch('/api/config');document.getElementById('cfgRole').value=['left','right','center'].includes(config.role)?config.role:'left';document.getElementById('cfgMaxTorque').value=config.max_torque;
  config.left_offsets.forEach((v,i)=>document.getElementById('lo'+i).value=v);config.right_offsets.forEach((v,i)=>document.getElementById('ro'+i).value=v);config.directions.forEach((v,i)=>document.getElementById('dm'+i).value=v<0?'-1':'1');
  Object.entries(config.constraints).forEach(([n,a])=>{const mn=document.getElementById(n+'_min'),mx=document.getElementById(n+'_max');if(mn){mn.value=a[0];mx.value=a[1]}});
  document.getElementById('rawConfig').value=config.raw_config||'';
 }catch(e){notice('Config read failed: '+e.message)}
}
function configParams(){
 const p=new URLSearchParams({role:document.getElementById('cfgRole').value,max_torque:document.getElementById('cfgMaxTorque').value});
 for(let i=0;i<5;i++){p.set('lo'+i,document.getElementById('lo'+i).value);p.set('ro'+i,document.getElementById('ro'+i).value)}
 for(let i=0;i<10;i++)p.set('dm'+i,document.getElementById('dm'+i).value);
 constraintNames.forEach(n=>{p.set(n+'_min',document.getElementById(n+'_min').value);p.set(n+'_max',document.getElementById(n+'_max').value)});
 return p
}
async function saveConfig(){try{const r=await jfetch('/api/config',{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},body:configParams()});notice('Configuration saved. AP identity: '+r.ssid+(r.reboot_required?' · reboot required':''),true);setTimeout(loadConfig,300)}catch(e){notice('Save failed: '+e.message)}}
async function saveAndReboot(){try{await saveConfig();const r=await jfetch('/api/reboot',{method:'POST'});notice('Rebooting. Reconnect to '+r.next_ssid+'.',true)}catch(e){notice('Reboot request failed: '+e.message)}}
async function reloadConfig(){try{await jfetch('/api/config/reload',{method:'POST'});await loadConfig();notice('Configuration reloaded from SPIFFS.',true)}catch(e){notice('Reload failed: '+e.message)}}
async function calibrateSensors(){if(!confirm('Support the leg in the 180° calibration pose. Calibrate and save current sensor offsets?'))return;try{await jfetch('/api/calibrate/sensors',{method:'POST'});notice('Calibration queued. Watch Terminal for results.',true);setTimeout(loadConfig,1500)}catch(e){notice('Calibration failed: '+e.message)}}
async function writeRawConfig(){if(!confirm('Write raw /config.txt? Current file will be copied to /config.bak.'))return;try{const r=await jfetch('/api/config/raw',{method:'POST',headers:{'Content-Type':'text/plain'},body:document.getElementById('rawConfig').value});notice('Raw config written. Reboot required. Next AP: '+r.ssid,true);setTimeout(loadConfig,300)}catch(e){notice('Raw write failed: '+e.message)}}

async function loadLogs(){try{const d=await jfetch('/api/log?since='+lastLog);const c=document.getElementById('console');d.entries.forEach(e=>{c.textContent+=`[${(e.ms/1000).toFixed(3)}] ${e.text}\n`;lastLog=Math.max(lastLog,e.seq)});if(d.entries.length)c.scrollTop=c.scrollHeight}catch(e){}}
async function loadFiles(){try{const d=await jfetch('/api/spiffs/list');const b=document.getElementById('fileList');b.innerHTML=`<div class="tiny">${d.used} / ${d.total} bytes used</div>`;d.files.forEach(f=>{const e=document.createElement('div');e.className='file';e.innerHTML=`<span>${f.name}<br><span class="tiny">${f.size} bytes</span></span><button>View</button>`;e.querySelector('button').onclick=()=>readFile(f.name);b.appendChild(e)})}catch(e){notice('SPIFFS list failed: '+e.message)}}
async function readFile(path){try{document.getElementById('fileTitle').textContent=path;document.getElementById('fileViewer').value=await jfetch('/api/spiffs/read?path='+encodeURIComponent(path))}catch(e){notice('File read failed: '+e.message)}}

buildConfigFields();loadState();loadConfig();loadFiles();loadLogs();setInterval(loadState,500);setInterval(loadLogs,500);
</script>
</body>
</html>
)DBHTML";

void handlePortalRoot() {
  sendNoCacheHeaders();
  server.send_P(200, "text/html", PORTAL_HTML);
}

void registerPortalRoutes() {
  if (portalRoutesRegistered) return;
  portalRoutesRegistered = true;

  server.on("/", HTTP_GET, handlePortalRoot);
  server.on("/api/state", HTTP_GET, handleApiState);
  server.on("/api/config", HTTP_GET, handleApiConfigGet);
  server.on("/api/config", HTTP_POST, handleApiConfigPost);
  server.on("/api/config/reload", HTTP_POST, handleApiConfigReload);
  server.on("/api/config/raw", HTTP_POST, handleApiRawConfigPost);
  server.on("/api/command", HTTP_POST, handleApiCommand);
  server.on("/api/log", HTTP_GET, handleApiLog);
  server.on("/api/spiffs/list", HTTP_GET, handleApiSPIFFSList);
  server.on("/api/spiffs/read", HTTP_GET, handleApiSPIFFSRead);
  server.on("/api/calibrate/sensors", HTTP_POST, handleApiSensorCalibration);
  server.on("/api/reboot", HTTP_POST, handleApiReboot);

  // Captive portal detection endpoints used by Android, Apple, Windows, etc.
  server.on("/generate_204", HTTP_GET, redirectToPortal);
  server.on("/gen_204", HTTP_GET, redirectToPortal);
  server.on("/hotspot-detect.html", HTTP_GET, redirectToPortal);
  server.on("/library/test/success.html", HTTP_GET, redirectToPortal);
  server.on("/ncsi.txt", HTTP_GET, redirectToPortal);
  server.on("/connecttest.txt", HTTP_GET, redirectToPortal);
  server.on("/success.txt", HTTP_GET, redirectToPortal);
  server.onNotFound(redirectToPortal);
}

void startOrRestartSoftAP() {
  portalSSID = desiredPortalSSID();

  dnsServer.stop();
  server.stop();
  WiFi.softAPdisconnect(true);
  delay(60);

  WiFi.mode(WIFI_AP);
  WiFi.softAPConfig(portalIP, portalGateway, portalSubnet);

  // Open captive network, matching the esp-captive-chat transport model.
  // Because this portal can command actuators, do not expose it beyond the
  // robot's intended local operating environment.
  bool ok = WiFi.softAP(portalSSID.c_str());
  if (!ok) {
    dbPrintln("ERROR: SoftAP start failed.");
    return;
  }

  dnsServer.start(DNS_PORT, "*", WiFi.softAPIP());
  server.begin();

  dbPrintf("Captive portal active: SSID=%s IP=%s\n",
           portalSSID.c_str(), WiFi.softAPIP().toString().c_str());
}

void setupPortal() {
  registerPortalRoutes();
  startOrRestartSoftAP();
}

void portalTask(void *parameter) {
  while (true) {
    if (portalRestartRequested) {
      portalRestartRequested = false;
      startOrRestartSoftAP();
    }

    dnsServer.processNextRequest();
    server.handleClient();

    if (portalRebootRequested && (int32_t)(millis() - portalRebootAtMs) >= 0) {
      portalRebootRequested = false;
      delay(40);
      ESP.restart();
    }

    vTaskDelay(pdMS_TO_TICKS(2));
  }
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
  webLogMutex = xSemaphoreCreateMutex();
  spiffsMutex = xSemaphoreCreateMutex();
  webCommandQueue = xQueueCreate(12, sizeof(WebCommand));

  analogReadResolution(12);
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);

  if (!SPIFFS.begin(true)) {
    dbPrintln("ERROR: SPIFFS mount failed.");
    while (true) delay(1000);
  }

  loadConfig();
  roleAtBoot = selectedRoleName();

  // Full-time captive portal requires the Wi-Fi-safe ADC1 sensor pinout.
  // With the legacy ADC2 pinout selected, Wi-Fi is explicitly kept off.
  if (DROPBEAR_ENABLE_WIFI_PORTAL) {
    dbPrintf("Wi-Fi portal enabled: ADC1 sensor pinout REQUIRED: outer=%d inner=%d hipPitch=%d knee=%d hipRoll=%d\n",
             PIN_OUTER_CALF, PIN_INNER_CALF, PIN_HIP_PITCH, PIN_KNEE, PIN_HIP_ROLL);
    setupPortal();
    xTaskCreatePinnedToCore(portalTask, "portal", 8192, nullptr, 1, nullptr, 0);
  } else {
    WiFi.mode(WIFI_OFF);
    portalSSID = "DISABLED-LEGACY-ADC2";
    dbPrintln("Wi-Fi portal disabled: legacy ADC2 sensor pinout selected.");
  }

  if (configProvisioned && !isCenter) {
    pinMode(CAN0_INT, INPUT_PULLUP);
    SPI.begin(SPI_SCK_PIN, SPI_MISO_PIN, SPI_MOSI_PIN, CAN_CS_PIN);

    if (CAN.begin(MCP_ANY, CAN_1000KBPS, MCP_8MHZ) != CAN_OK) {
      dbPrintln("ERROR: MCP2515 CAN initialization failed at 1 Mbps / 8 MHz.");
      playMode = false;
    } else {
      CAN.setMode(MCP_NORMAL);
      dbPrintln("CAN initialized: 1 Mbps, MCP2515 8 MHz, CS GPIO5.");

      primeSensorFilter();

      // Wi-Fi/networking lives primarily on core 0. Keep the control path on
      // core 1 so captive-portal traffic does not become sensor/control jitter.
      xTaskCreatePinnedToCore(readAndComputeTask, "sensors", 4096, nullptr, 3, nullptr, 1);
      xTaskCreatePinnedToCore(impedanceControlTask, "impedance", 4096, nullptr, 3, nullptr, 1);
      xTaskCreatePinnedToCore(canOutputTask, "can-output", 4096, nullptr, 4, nullptr, 1);
      runtimeControlReady = true;
      runtimeImuReady = false;
      playMode = true;
    }
  } else if (configProvisioned && isCenter) {
    xTaskCreatePinnedToCore(imuReadTask, "imu", 4096, nullptr, 2, nullptr, 1);
    runtimeControlReady = false;
    runtimeImuReady = true;
    playMode = true;
  } else {
    runtimeControlReady = false;
    runtimeImuReady = false;
    playMode = false;
    dbPrintln("Controller is unconfigured. CAN/IMU runtime tasks are intentionally disabled.");
  }

  // One command task consumes both Serial and web-command queue traffic.
  xTaskCreatePinnedToCore(checkChiralityTask, "command", 6144, nullptr, 2, nullptr, 1);

  dbPrintf("Dropbear controller ready: role=%s portal=%s SSID=%s. Type 'help'.\n",
           selectedRoleName().c_str(),
           DROPBEAR_ENABLE_WIFI_PORTAL ? "enabled" : "disabled",
           DROPBEAR_ENABLE_WIFI_PORTAL ? desiredPortalSSID().c_str() : "n/a");
}

void loop() {
  // All runtime ownership is explicit FreeRTOS tasks.
  vTaskDelay(pdMS_TO_TICKS(1000));
}
