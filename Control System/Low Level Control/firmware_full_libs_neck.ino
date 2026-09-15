#include <Arduino.h>
#include <mcp_can.h>
#include <SPI.h>
#include <SPIFFS.h>
#include <Wire.h>
#include <WiFi.h>
#include <WebServer.h>
#include <DNSServer.h>
#include <FastAccelStepper.h>
#include <BluetoothSerial.h>
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
 *   - Preserves the original five AS5600 OUT GPIOs and decodes their one-wire
 *     PWM duty cycle with interrupts, allowing Wi-Fi to remain active.
 *   - Preserves MyActuator CAN IDs 0x141..0x14C.
 *   - Preserves A1 torque-control and 0x81 stop commands.
 *   - Uses one periodic CAN-output task as the normal actuator writer.
 *   - Impedance task computes desired torque only; it does not transmit CAN.
 *   - Each leg controller transmits only to its selected chirality (6 motors).
 *   - Correct stop commands use actual actuator CAN IDs.
 *   - Calibration uses an output override instead of racing the CAN task.
 *   - Fixes impedance serial parsing and joint-constraint persistence.
 *   - Starts the IMU task in center mode.
 *   - AS5600 pulse state is processed at 1 kHz; serial telemetry is throttled to 50 Hz.
 *   - Adds role-aware Wi-Fi SoftAP + captive portal control/configuration.
 *   - LEFTLEG / RIGHTLEG / CENTER SSID follows persisted chirality.
 *   - Existing SPIFFS configuration is loaded into the web configurator.
 *   - Web commands are queued into the same parser used by USB Serial.
 *   - Enforces DB1 destination envelopes (<DB1:LEFTLEG>, <DB1:RIGHTLEG>,
 *     <DB1:CENTER>, <DB1:HEADNECK>, <DB1:SETUP>) before subsystem parsing.
 *   - The only allowed DB1 broadcast command is <DB1:ALL> stop.
 *   - Live state, command log, config, and SPIFFS inspection are exposed locally.
 *   - Adds HEAD_NECK as a fourth mutually-exclusive boot hardware personality.
 *   - HEAD_NECK preserves the Dropbear-Neck-Assembly six A4988 STEP/DIR pinout,
 *     FastAccelStepper motion model, legacy direct/pose/quaternion commands,
 *     Bluetooth name NECK_BT, and adds neck-specific portal state/diagnostics.
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

// The leg encoders are AS5600 OUT signals on independent one-wire GPIOs.
// They are decoded as PWM rather than sampled through ADC2, so the original
// Dropbear wiring can coexist with the always-on Wi-Fi captive portal.
//
// ORIGINAL DROPBEAR SENSOR PINOUT (preserved):
//   outer calf  -> GPIO14
//   inner calf  -> GPIO27
//   hip pitch   -> GPIO26
//   knee        -> GPIO25
//   hip roll    -> GPIO33
//
// AS5600 PWM timing is nominally 115/230/460/920 Hz and represents 0..360°
// between approximately 2.9% and 97.1% duty cycle. The interrupt capture below
// measures period/high-time without consuming ADC2, which avoids the classic
// ESP32 Wi-Fi/ADC2 resource conflict while retaining the existing wiring.
#define DROPBEAR_ENABLE_WIFI_PORTAL 1

static const int PIN_OUTER_CALF = 14;
static const int PIN_INNER_CALF = 27;
static const int PIN_HIP_PITCH = 26;
static const int PIN_KNEE = 25;
static const int PIN_HIP_ROLL = 33;

// -----------------------------------------------------------------------------
// HEAD / NECK hardware personality
// -----------------------------------------------------------------------------
// Exact six-axis Stewart-neck STEP/DIR mapping from Dropbear-Neck-Assembly.
// These pins intentionally overlap leg SPI/I2C/AS5600 pins; HEAD_NECK is
// therefore a mutually-exclusive boot personality and never initializes leg
// hardware in the same boot.
static const uint8_t NECK_MOTOR_COUNT = 6;
static const int NECK_STEP_PINS[NECK_MOTOR_COUNT] = {33, 18, 23, 19, 22, 21};
static const int NECK_DIR_PINS[NECK_MOTOR_COUNT]  = {32, 26, 14, 27, 12, 13};
static const int NECK_ENABLE_PIN = 25;
static const char *NECK_BT_NAME = "NECK_BT";

static const uint32_t NECK_DEFAULT_SPEED_HZ = 48000;
static const uint32_t NECK_DEFAULT_ACCEL = 36000;
static const float NECK_DEFAULT_STEPS_PER_MM = 426.67f;
static const float NECK_DEFAULT_MIN_MM = 0.0f;
static const float NECK_DEFAULT_MAX_MM = 80.0f;
static const float NECK_POSE_HEIGHT_SCALE = 400.0f; // retained from neck firmware

static const uint8_t AS5600_SENSOR_COUNT = 5;
static const uint32_t AS5600_STALE_US = 100000; // 100 ms: safely above 115 Hz frame period
static const float AS5600_PWM_MIN_DUTY = 128.0f / 4351.0f;
static const float AS5600_PWM_MAX_DUTY = 4223.0f / 4351.0f;

struct As5600PwmCapture {
  volatile uint32_t lastRiseUs = 0;
  volatile uint32_t highUs = 0;
  volatile uint32_t periodUs = 0;
  volatile uint32_t lastEdgeUs = 0;
  volatile uint32_t frames = 0;
  volatile bool valid = false;
};

As5600PwmCapture as5600Capture[AS5600_SENSOR_COUNT];
static const int AS5600_PINS[AS5600_SENSOR_COUNT] = {
  PIN_OUTER_CALF, PIN_INNER_CALF, PIN_HIP_PITCH, PIN_KNEE, PIN_HIP_ROLL
};

void IRAM_ATTR handleAs5600Edge(uint8_t index, int pin) {
  const uint32_t now = micros();
  As5600PwmCapture &c = as5600Capture[index];
  const int level = digitalRead(pin);
  c.lastEdgeUs = now;

  if (level) {
    if (c.lastRiseUs != 0) {
      const uint32_t period = now - c.lastRiseUs;
      // Accept all four documented AS5600 PWM frequencies with generous margin.
      if (period >= 800 && period <= 12000) {
        c.periodUs = period;
        if (c.highUs > 0 && c.highUs < period) {
          c.valid = true;
          c.frames++;
        }
      }
    }
    c.lastRiseUs = now;
  } else if (c.lastRiseUs != 0) {
    const uint32_t high = now - c.lastRiseUs;
    if (high > 0 && high < 12000) c.highUs = high;
  }
}

void IRAM_ATTR as5600Isr0() { handleAs5600Edge(0, PIN_OUTER_CALF); }
void IRAM_ATTR as5600Isr1() { handleAs5600Edge(1, PIN_INNER_CALF); }
void IRAM_ATTR as5600Isr2() { handleAs5600Edge(2, PIN_HIP_PITCH); }
void IRAM_ATTR as5600Isr3() { handleAs5600Edge(3, PIN_KNEE); }
void IRAM_ATTR as5600Isr4() { handleAs5600Edge(4, PIN_HIP_ROLL); }

void setupAs5600Inputs() {
  for (int i = 0; i < AS5600_SENSOR_COUNT; ++i) pinMode(AS5600_PINS[i], INPUT);
  attachInterrupt(digitalPinToInterrupt(PIN_OUTER_CALF), as5600Isr0, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_INNER_CALF), as5600Isr1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_HIP_PITCH), as5600Isr2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_KNEE), as5600Isr3, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_HIP_ROLL), as5600Isr4, CHANGE);
}

bool readAs5600Pwm(uint8_t index, float &angleDeg, float &duty, float &freqHz,
                   uint32_t &periodUs, uint32_t &highUs, uint32_t &ageUs) {
  if (index >= AS5600_SENSOR_COUNT) return false;

  noInterrupts();
  const As5600PwmCapture snapshot = as5600Capture[index];
  interrupts();

  const uint32_t nowUs = micros();
  ageUs = snapshot.lastEdgeUs ? (nowUs - snapshot.lastEdgeUs) : UINT32_MAX;
  periodUs = snapshot.periodUs;
  highUs = snapshot.highUs;

  if (!snapshot.valid || periodUs == 0 || highUs == 0 || ageUs > AS5600_STALE_US) {
    duty = 0.0f;
    freqHz = 0.0f;
    angleDeg = 0.0f;
    return false;
  }

  duty = static_cast<float>(highUs) / static_cast<float>(periodUs);
  freqHz = 1000000.0f / static_cast<float>(periodUs);
  float fraction = (duty - AS5600_PWM_MIN_DUTY) /
                   (AS5600_PWM_MAX_DUTY - AS5600_PWM_MIN_DUTY);
  if (fraction < 0.0f) fraction = 0.0f;
  if (fraction > 1.0f) fraction = 1.0f;
  angleDeg = fraction * 360.0f;
  return true;
}

int as5600PseudoRaw(uint8_t index) {
  float angle = 0.0f, duty = 0.0f, freq = 0.0f;
  uint32_t period = 0, high = 0, age = 0;
  if (!readAs5600Pwm(index, angle, duty, freq, period, high, age)) return -1;
  int raw = static_cast<int>(lroundf((angle / 360.0f) * 4095.0f));
  if (raw < 0) raw = 0;
  if (raw > 4095) raw = 4095;
  return raw;
}

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
bool isHead = false;
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
volatile bool runtimeNeckReady = false;

// -----------------------------------------------------------------------------
// DB1 command addressing / routing
// -----------------------------------------------------------------------------
// Every external command is expected to carry a destination envelope:
//   <DB1:LEFTLEG> torque knee 25
//   <DB1:RIGHTLEG> impedance knee 1 180 0
//   <DB1:HEADNECK> X10,Y-5,H30,R5,P-3
//   <DB1:SETUP> role left
//
// The envelope is validated BEFORE any subsystem-specific parser sees the
// payload. This prevents a command delivered to the wrong ESP32 from being
// interpreted as a valid command for that controller. Unaddressed legacy
// commands are disabled by default and can only be re-enabled explicitly.
enum CommandTarget : uint8_t {
  TARGET_INVALID = 0,
  TARGET_SETUP,
  TARGET_LEFTLEG,
  TARGET_RIGHTLEG,
  TARGET_CENTER,
  TARGET_HEADNECK,
  TARGET_LEFTARM,
  TARGET_RIGHTARM,
  TARGET_ALL
};

bool legacyUnaddressedCommands = false;
volatile uint32_t routedCommandsAccepted = 0;
volatile uint32_t routedCommandsRejected = 0;
volatile uint32_t routedMissingHeader = 0;
volatile uint32_t routedTargetMismatch = 0;
volatile uint32_t routedUnsupportedTarget = 0;
volatile uint32_t routedBroadcastStop = 0;
volatile uint32_t routedLegacyAccepted = 0;
volatile uint32_t lastRoutedCommandMs = 0;
String lastRoutedTarget = "";
String lastRoutedSource = "";
String lastRoutedPayload = "";

// -----------------------------------------------------------------------------
// Dual operating structure
// -----------------------------------------------------------------------------
// STANDALONE is the existing Dropbear controller: portal/Serial command
// authority with manual torque + external-sensor impedance control.
// HYPERSPAWN_ROUTE is the parallel ROS2/CAN route derived from
// Hyperspawn/dropbear_firmware. Hardware, sensors, safety and RMD output remain
// shared; only command authority and upstream state transport change.
enum OperatingMode : uint8_t {
  OPERATING_STANDALONE = 0,
  OPERATING_HYPERSPAWN_ROUTE = 1
};

volatile OperatingMode operatingMode = OPERATING_STANDALONE; // default/original path
String operatingModeAtBoot = "standalone";

String operatingModeName() {
  return operatingMode == OPERATING_HYPERSPAWN_ROUTE ? "hyperspawn" : "standalone";
}

// Hyperspawn/dropbear_firmware network constants. The existing repository uses
// node 0x12 for left_leg, 0x13 for right_leg, brain 0x01, 1 Mbps CAN, 500 Hz
// control/state cadence and a 1 Hz heartbeat.
static const uint8_t HS_NODE_BRAIN = 0x01;
static const uint8_t HS_NODE_LEFT_LEG = 0x12;
static const uint8_t HS_NODE_RIGHT_LEG = 0x13;
static const uint8_t HS_LIMB_LEFT_LEG = 2;
static const uint8_t HS_LIMB_RIGHT_LEG = 3;
static const uint8_t HS_MSG_HEARTBEAT = 0x00;
static const uint8_t HS_MSG_CMD_POS = 0x01;
static const uint8_t HS_MSG_CMD_TORQUE = 0x02;
static const uint8_t HS_MSG_STATE = 0x03;
// Extension message types for joints 4-5. CAN 2.0 carries only four int16
// values in one frame, while each Dropbear leg has six joints.
static const uint8_t HS_MSG_CMD_POS_EXT = 0x04;
static const uint8_t HS_MSG_CMD_TORQUE_EXT = 0x05;
static const uint8_t HS_MSG_STATE_EXT = 0x06;
static const uint8_t HS_MSG_FAULT = 0x0F;
static const uint16_t HS_ROUTE_HZ = 500;
static const uint16_t HS_HEARTBEAT_HZ = 1;

// Wire joint order used by this combined firmware:
//   0 hip_pitch, 1 hip_roll, 2 hip_yaw, 3 knee, 4 outer_calf/ankle-A,
//   5 inner_calf/ankle-B.
static const uint8_t HS_JOINT_COUNT = 6;
enum HyperspawnJointIndex : uint8_t {
  HS_HIP_PITCH = 0,
  HS_HIP_ROLL = 1,
  HS_HIP_YAW = 2,
  HS_KNEE = 3,
  HS_OUTER_CALF = 4,
  HS_INNER_CALF = 5
};

enum HyperspawnControlMode : uint8_t {
  HS_CONTROL_NONE = 0,
  HS_CONTROL_POSITION = 1,
  HS_CONTROL_TORQUE = 2
};

volatile HyperspawnControlMode hyperspawnControlMode = HS_CONTROL_NONE;
volatile int16_t hyperspawnPositionSetpoints[HS_JOINT_COUNT] = {0};
volatile int16_t hyperspawnTorqueSetpoints[HS_JOINT_COUNT] = {0};
// Targeted two-frame commands decode into staging banks first. They are copied
// into the active banks only when the full six-joint command is complete.
volatile int16_t hyperspawnPositionPending[HS_JOINT_COUNT] = {0};
volatile int16_t hyperspawnTorquePending[HS_JOINT_COUNT] = {0};
volatile int16_t hyperspawnJointState[HS_JOINT_COUNT] = {0};
volatile uint32_t hyperspawnLastCommandMs = 0;
volatile uint32_t hyperspawnLastPositionMs = 0;
volatile uint32_t hyperspawnLastTorqueMs = 0;
volatile uint32_t hyperspawnRxFrames = 0;
volatile uint32_t hyperspawnRxLegacyFrames = 0;
volatile uint32_t hyperspawnRxTargetedFrames = 0;
volatile uint32_t hyperspawnRxRejectedFrames = 0;
volatile uint32_t hyperspawnStateFrames = 0;
volatile uint32_t hyperspawnHeartbeatFrames = 0;
volatile uint32_t hyperspawnFaultFrames = 0;
volatile uint32_t hyperspawnWatchdogTrips = 0;
volatile bool hyperspawnWatchdogTripped = false;
volatile uint8_t hyperspawnFaultCode = 0;
// Targeted six-joint commands arrive as two CAN 2.0 frames. Arm/update control
// only after both fragments have arrived inside this window.
static const uint32_t HS_FRAGMENT_TIMEOUT_MS = 50;
volatile bool hyperspawnPositionBasePending = false;
volatile bool hyperspawnPositionExtPending = false;
volatile bool hyperspawnTorqueBasePending = false;
volatile bool hyperspawnTorqueExtPending = false;
volatile uint32_t hyperspawnPositionBaseMs = 0;
volatile uint32_t hyperspawnPositionExtMs = 0;
volatile uint32_t hyperspawnTorqueBaseMs = 0;
volatile uint32_t hyperspawnTorqueExtMs = 0;
volatile uint32_t hyperspawnCompletedCommands = 0;
volatile uint32_t hyperspawnFragmentTimeouts = 0;
volatile uint32_t lastHyperspawnTaskMs = 0;
volatile uint32_t hyperspawnTaskLoops = 0;

uint32_t hyperspawnCommandTimeoutMs = 250;
bool hyperspawnLegacyBroadcast = false; // unsafe on a shared multi-limb bus; compatibility opt-in
bool hyperspawnAutoArm = true;
float hyperspawnPositionUnitsPerDegree = 1.0f; // published repo leaves scaling unspecified

TaskHandle_t canRxTaskHandle = nullptr;
TaskHandle_t hyperspawnTaskHandle = nullptr;
volatile uint32_t canRxTaskLoops = 0;
volatile uint32_t lastCanRxTaskMs = 0;
volatile uint32_t canRxFrames = 0;
volatile uint32_t canRxErrors = 0;
volatile uint32_t lastCanRxMs = 0;

// TCA9548A is reserved for the IMU bank; AS5600s never use this bus.
static const uint8_t IMU_MUX_ADDRESS = 0x70;
static const uint8_t IMU_DEVICE_ADDRESS = 0x68;
volatile bool imuMuxSeen = false;
volatile uint32_t imuMuxSelectOk = 0;
volatile uint32_t imuMuxSelectFail = 0;

// -----------------------------------------------------------------------------
// HEAD / NECK runtime
// -----------------------------------------------------------------------------

FastAccelStepperEngine neckEngine = FastAccelStepperEngine();
FastAccelStepper *neckSteppers[NECK_MOTOR_COUNT] = {nullptr, nullptr, nullptr, nullptr, nullptr, nullptr};
BluetoothSerial neckBluetooth;

uint32_t neckSpeedHz = NECK_DEFAULT_SPEED_HZ;
uint32_t neckAcceleration = NECK_DEFAULT_ACCEL;
float neckStepsPerMm = NECK_DEFAULT_STEPS_PER_MM;
bool neckBluetoothEnabled = true;
bool neckBluetoothStarted = false;
bool neckAutoHome = false; // safer unified-firmware default; original standalone firmware homes on boot
bool neckUseEnablePin = true;
volatile bool neckMotionEnabled = true;
volatile bool neckBypassLimits = false;
volatile bool neckSoftwareHomed = false;

float neckMinMm[NECK_MOTOR_COUNT] = {0,0,0,0,0,0};
float neckMaxMm[NECK_MOTOR_COUNT] = {80,80,80,80,80,80};
volatile int32_t neckTargetSteps[NECK_MOTOR_COUNT] = {0,0,0,0,0,0};
volatile uint32_t neckMoveCommands = 0;
volatile uint32_t neckRejectedCommands = 0;
volatile uint32_t neckStopCommands = 0;
volatile uint32_t neckBluetoothCommands = 0;
volatile uint32_t neckPortalCommands = 0;
volatile uint32_t neckSerialCommands = 0;
volatile uint32_t neckLastCommandMs = 0;
volatile uint32_t neckServiceLoops = 0;
volatile uint32_t lastNeckServiceMs = 0;
TaskHandle_t neckTaskHandle = nullptr;

struct NeckPoseState {
  volatile int x = 0;
  volatile int y = 0;
  volatile int z = 0;
  volatile int height = 0;
  volatile int roll = 0;
  volatile int pitch = 0;
  volatile float speedMultiplier = 1.0f;
  volatile float accelMultiplier = 1.0f;
};
NeckPoseState neckLastPose;

enum NeckHomeState : uint8_t {
  NECK_HOME_IDLE = 0,
  NECK_HOME_SOFT_SETTLE,
  NECK_HOME_BRUTE_PREP_SETTLE,
  NECK_HOME_BRUTE_GAP,
  NECK_HOME_BRUTE_FINAL_SETTLE
};
volatile NeckHomeState neckHomeState = NECK_HOME_IDLE;
volatile uint32_t neckHomeDeadlineMs = 0;
volatile bool neckHomePreviousBypass = false;

static const int NECK_SOFT_HOME_HEIGHT_MM = -40;
static const float NECK_SOFT_HOME_SPEED_MULT = 2.0f;
static const float NECK_SOFT_HOME_ACCEL_MULT = 2.0f;
static const uint32_t NECK_SOFT_HOME_SETTLE_MS = 2200;
static const int NECK_BRUTE_PREP_HEIGHT_MM = -55;
static const float NECK_BRUTE_PREP_SPEED_MULT = 2.5f;
static const float NECK_BRUTE_PREP_ACCEL_MULT = 2.5f;
static const uint32_t NECK_BRUTE_PREP_SETTLE_MS = 2300;
static const int NECK_BRUTE_HOME_HEIGHT_MM = -80;
static const float NECK_BRUTE_HOME_SPEED_MULT = 3.0f;
static const float NECK_BRUTE_HOME_ACCEL_MULT = 3.0f;
static const uint32_t NECK_BRUTE_HOME_SETTLE_MS = 2600;

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

// -----------------------------------------------------------------------------
// Runtime diagnostics
// -----------------------------------------------------------------------------

struct SensorDiagnostic {
  volatile int raw = 0; // 0..4095 pseudo-raw derived from decoded PWM angle
  volatile float filtered = 0.0f;
  volatile float angle = 0.0f;
  volatile int minRaw = 4095;
  volatile int maxRaw = 0;
  volatile int lastChangeRaw = -1;
  volatile uint32_t samples = 0;
  volatile uint32_t lastSampleMs = 0;
  volatile uint32_t lastChangeMs = 0;
  volatile bool signalValid = false;
  volatile float duty = 0.0f;
  volatile float frequencyHz = 0.0f;
  volatile uint32_t periodUs = 0;
  volatile uint32_t highUs = 0;
  volatile uint32_t pulseAgeUs = UINT32_MAX;
  volatile uint32_t pwmFrames = 0;
};

struct ActuatorDiagnostic {
  volatile uint32_t txOk = 0;
  volatile uint32_t txFail = 0;
  volatile uint32_t lastTxMs = 0;
  volatile int16_t lastCommand = 0;
  volatile uint8_t lastOpcode = 0;
  volatile int lastResult = 0;
};

struct ImuDiagnostic {
  volatile bool everSeen = false;
  volatile uint32_t readOk = 0;
  volatile uint32_t readFail = 0;
  volatile uint32_t lastProbeMs = 0;
  volatile uint32_t lastSeenMs = 0;
  volatile int16_t ax = 0;
  volatile int16_t ay = 0;
  volatile int16_t az = 0;
  volatile int16_t gx = 0;
  volatile int16_t gy = 0;
  volatile int16_t gz = 0;
};

SensorDiagnostic sensorDiagnostics[5];
ActuatorDiagnostic actuatorDiagnostics[ACTUATOR_COUNT];
ImuDiagnostic imuDiagnostics[IMU_COUNT];

volatile bool spiffsMounted = false;
volatile bool i2cInitialized = false;
volatile bool spiInitialized = false;
volatile bool canInitialized = false;
volatile bool portalOnline = false;

volatile uint32_t spiffsReadOps = 0;
volatile uint32_t spiffsWriteOps = 0;
volatile uint32_t spiffsErrors = 0;
volatile uint32_t lastSpiffsReadMs = 0;
volatile uint32_t lastSpiffsWriteMs = 0;

volatile uint32_t canTxSuccess = 0;
volatile uint32_t canTxFailure = 0;
volatile uint32_t canConsecutiveFailures = 0;
volatile uint32_t canMutexTimeouts = 0;
volatile uint32_t canTorqueFrames = 0;
volatile uint32_t canStopFrames = 0;
volatile uint32_t lastCanTxMs = 0;
volatile uint32_t lastCanFailureMs = 0;
volatile int lastCanResult = 0;

volatile uint32_t sensorTaskLoops = 0;
volatile uint32_t impedanceTaskLoops = 0;
volatile uint32_t canTaskLoops = 0;
volatile uint32_t commandTaskLoops = 0;
volatile uint32_t portalTaskLoops = 0;
volatile uint32_t imuTaskLoops = 0;
volatile uint32_t lastSensorTaskMs = 0;
volatile uint32_t lastImpedanceTaskMs = 0;
volatile uint32_t lastCanTaskMs = 0;
volatile uint32_t lastCommandTaskMs = 0;
volatile uint32_t lastPortalTaskMs = 0;
volatile uint32_t lastImuTaskMs = 0;

volatile uint32_t webCommandsQueued = 0;
volatile uint32_t webCommandsProcessed = 0;
volatile uint32_t webCommandQueueFull = 0;
volatile uint32_t serialCommandsProcessed = 0;
volatile uint32_t portalHttpRequests = 0;
volatile uint32_t portalRedirects = 0;

TaskHandle_t sensorTaskHandle = nullptr;
TaskHandle_t impedanceTaskHandle = nullptr;
TaskHandle_t canTaskHandle = nullptr;
TaskHandle_t commandTaskHandle = nullptr;
TaskHandle_t portalTaskHandle = nullptr;
TaskHandle_t imuTaskHandle = nullptr;

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
void processPayloadCommand(String command, const char *source = "serial");
void processRoutedCommand(String command, const char *source = "serial");
void readIMU();
void saveConfig();
void printHelp();
void hyperspawnRouteTask(void *parameter);
void canReceiveTask(void *parameter);
void processHyperspawnSerialCommand(const String &command);
void printHyperspawnStatus();
void changeOperatingMode(const String &modeName);
void sendHyperspawnFault(uint8_t code);
void setupNeckHardware();
void neckServiceTask(void *parameter);
void neckStopAll();
void neckZeroAll();
bool processNeckCommand(String command, const char *source = "serial");
void printNeckStatus();
void changeDeviceRole(const String &role);


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
  if (isHead) return "head";
  if (isCenter) return "center";
  return isLeft ? "left" : "right";
}

bool isLegRole() {
  return configProvisioned && !isCenter && !isHead;
}

String desiredPortalSSID() {
  if (!configProvisioned) return "DROPBEAR-SETUP";
  if (isHead) return "HEADNECK";
  if (isCenter) return "CENTER";
  return isLeft ? "LEFTLEG" : "RIGHTLEG";
}

String commandTargetName(CommandTarget target) {
  switch (target) {
    case TARGET_SETUP: return "SETUP";
    case TARGET_LEFTLEG: return "LEFTLEG";
    case TARGET_RIGHTLEG: return "RIGHTLEG";
    case TARGET_CENTER: return "CENTER";
    case TARGET_HEADNECK: return "HEADNECK";
    case TARGET_LEFTARM: return "LEFTARM";
    case TARGET_RIGHTARM: return "RIGHTARM";
    case TARGET_ALL: return "ALL";
    default: return "INVALID";
  }
}

CommandTarget currentCommandTarget() {
  if (!configProvisioned) return TARGET_SETUP;
  if (isHead) return TARGET_HEADNECK;
  if (isCenter) return TARGET_CENTER;
  return isLeft ? TARGET_LEFTLEG : TARGET_RIGHTLEG;
}

CommandTarget parseCommandTarget(String token) {
  token.trim();
  token.toUpperCase();
  if (token == "SETUP") return TARGET_SETUP;
  if (token == "LEFTLEG" || token == "LEFT_LEG") return TARGET_LEFTLEG;
  if (token == "RIGHTLEG" || token == "RIGHT_LEG") return TARGET_RIGHTLEG;
  if (token == "CENTER" || token == "CENTRE") return TARGET_CENTER;
  if (token == "HEADNECK" || token == "HEAD_NECK" || token == "HEAD" || token == "NECK") return TARGET_HEADNECK;
  if (token == "LEFTARM" || token == "LEFT_ARM") return TARGET_LEFTARM;
  if (token == "RIGHTARM" || token == "RIGHT_ARM") return TARGET_RIGHTARM;
  if (token == "ALL") return TARGET_ALL;
  return TARGET_INVALID;
}

String currentCommandAddress() {
  return commandTargetName(currentCommandTarget());
}

bool parseDB1Envelope(const String &rawCommand, CommandTarget &target, String &payload, bool &usedLegacy) {
  String command = rawCommand;
  command.trim();
  target = TARGET_INVALID;
  payload = "";
  usedLegacy = false;

  if (!command.startsWith("<DB1:")) {
    if (!legacyUnaddressedCommands) {
      routedMissingHeader++;
      routedCommandsRejected++;
      dbPrintf("ERR|MISSING_TARGET_HEADER|expected=<DB1:%s>|example=<DB1:%s> status\n",
               currentCommandAddress().c_str(), currentCommandAddress().c_str());
      return false;
    }
    usedLegacy = true;
    target = currentCommandTarget();
    payload = command;
    routedLegacyAccepted++;
    return payload.length() > 0;
  }

  const int close = command.indexOf('>');
  if (close < 6) {
    routedCommandsRejected++;
    dbPrintln("ERR|MALFORMED_TARGET_HEADER|expected=<DB1:TARGET>");
    return false;
  }

  String token = command.substring(5, close);
  target = parseCommandTarget(token);
  payload = command.substring(close + 1);
  payload.trim();

  if (target == TARGET_INVALID) {
    routedCommandsRejected++;
    dbPrintf("ERR|UNKNOWN_TARGET|received=%s\n", token.c_str());
    return false;
  }
  if (!payload.length()) {
    routedCommandsRejected++;
    dbPrintln("ERR|EMPTY_COMMAND_PAYLOAD");
    return false;
  }
  return true;
}

bool isAllowedBroadcastPayload(String payload) {
  payload.trim();
  payload.toLowerCase();
  // ALL is intentionally safety-only. Never permit broadcast motion, homing,
  // calibration, role changes, reboot, or configuration writes.
  return payload == "stop";
}

bool validateApiTargetArg() {
  if (!server.hasArg("target")) {
    server.send(409, "application/json",
                "{\"ok\":false,\"error\":\"missing_target\",\"expected\":\"" + currentCommandAddress() + "\"}");
    return false;
  }
  CommandTarget requested = parseCommandTarget(server.arg("target"));
  CommandTarget expected = currentCommandTarget();
  if (requested != expected) {
    routedTargetMismatch++;
    routedCommandsRejected++;
    server.send(409, "application/json",
                "{\"ok\":false,\"error\":\"target_mismatch\",\"expected\":\"" +
                commandTargetName(expected) + "\",\"received\":\"" + commandTargetName(requested) + "\"}");
    return false;
  }
  return true;
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
  if (index < 0 || index >= ACTUATOR_COUNT || isCenter || isHead) return false;
  return isLeft ? ((index & 1) == 1) : ((index & 1) == 0);
}

int firstSelectedActuatorIndex() {
  return isLeft ? 1 : 0;
}

uint32_t diagnosticAgeMs(uint32_t timestamp) {
  if (timestamp == 0) return UINT32_MAX;
  return millis() - timestamp;
}

int actuatorIndexFromCanId(uint32_t actuatorID) {
  for (int i = 0; i < ACTUATOR_COUNT; ++i) {
    if (ACTUATOR_IDS[i] == actuatorID) return i;
  }
  return -1;
}

void updateSensorDiagnosticSample(int index, int raw) {
  if (index < 0 || index >= 5) return;
  SensorDiagnostic &d = sensorDiagnostics[index];
  const uint32_t now = millis();

  float angle = 0.0f, duty = 0.0f, freq = 0.0f;
  uint32_t period = 0, high = 0, ageUs = UINT32_MAX;
  const bool valid = readAs5600Pwm(index, angle, duty, freq, period, high, ageUs);
  d.signalValid = valid;
  d.duty = duty;
  d.frequencyHz = freq;
  d.periodUs = period;
  d.highUs = high;
  d.pulseAgeUs = ageUs;
  d.pwmFrames = as5600Capture[index].frames;

  if (raw < 0) {
    d.lastSampleMs = now;
    return;
  }

  d.raw = raw;
  d.samples++;
  d.lastSampleMs = now;
  if (raw < d.minRaw) d.minRaw = raw;
  if (raw > d.maxRaw) d.maxRaw = raw;
  if (d.lastChangeRaw < 0 || abs(raw - d.lastChangeRaw) >= 3) {
    d.lastChangeRaw = raw;
    d.lastChangeMs = now;
  }
}

void updateSensorDiagnosticProcessed() {
  sensorDiagnostics[0].filtered = averageOuter;
  sensorDiagnostics[1].filtered = averageInner;
  sensorDiagnostics[2].filtered = averageHip;
  sensorDiagnostics[3].filtered = averageKnee;
  sensorDiagnostics[4].filtered = averageButt;

  sensorDiagnostics[0].angle = normalizedOuter;
  sensorDiagnostics[1].angle = normalizedInner;
  sensorDiagnostics[2].angle = normalizedHip;
  sensorDiagnostics[3].angle = normalizedKnee;
  sensorDiagnostics[4].angle = normalizedButt;
}

const char *sensorHealthStatus(const SensorDiagnostic &d) {
  if (!runtimeControlReady || isCenter || isHead) return "inactive";
  const uint32_t age = diagnosticAgeMs(d.lastSampleMs);
  if (age == UINT32_MAX || age > 100 || !d.signalValid || d.pulseAgeUs > AS5600_STALE_US) return "fault";
  if (d.duty < 0.025f || d.duty > 0.975f || d.frequencyHz < 100.0f || d.frequencyHz > 980.0f) return "warn";
  return "ok";
}

const char *taskHealthStatus(bool expectedActive, uint32_t lastMs, uint32_t warnAge, uint32_t faultAge) {
  if (!expectedActive) return "inactive";
  const uint32_t age = diagnosticAgeMs(lastMs);
  if (age == UINT32_MAX || age > faultAge) return "fault";
  if (age > warnAge) return "warn";
  return "ok";
}

uint32_t taskStackWords(TaskHandle_t handle) {
  if (!handle) return 0;
  return (uint32_t)uxTaskGetStackHighWaterMark(handle);
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

bool canSendFrame(uint32_t actuatorID, const byte *data, byte dataLen) {
  if (isCenter || isHead || !canInitialized) return false;
  if (canMutex == nullptr) return false;

  const int actuatorIndex = actuatorIndexFromCanId(actuatorID);
  bool ok = false;
  int result = -2;

  if (xSemaphoreTake(canMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
    result = CAN.sendMsgBuf(actuatorID, 0, dataLen, const_cast<byte *>(data));
    ok = (result == CAN_OK);
    xSemaphoreGive(canMutex);
  } else {
    canMutexTimeouts++;
  }

  const uint32_t now = millis();
  lastCanResult = result;
  if (ok) {
    canTxSuccess++;
    canConsecutiveFailures = 0;
    lastCanTxMs = now;
    if (data[0] == 0xA1) canTorqueFrames++;
    if (data[0] == 0x81) canStopFrames++;
  } else {
    canTxFailure++;
    canConsecutiveFailures++;
    lastCanFailureMs = now;
  }

  if (actuatorIndex >= 0) {
    ActuatorDiagnostic &d = actuatorDiagnostics[actuatorIndex];
    d.lastOpcode = data[0];
    d.lastResult = result;
    d.lastTxMs = now;
    if (data[0] == 0xA1) {
      d.lastCommand = static_cast<int16_t>(static_cast<uint16_t>(data[4]) |
                                           (static_cast<uint16_t>(data[5]) << 8));
    } else if (data[0] == 0x81) {
      d.lastCommand = 0;
    }
    if (ok) d.txOk++;
    else d.txFail++;
  }

  return ok;
}

bool canSend(uint32_t actuatorID, const byte data[8]) {
  return canSendFrame(actuatorID, data, 8);
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

void neckStopAll();

void clearHyperspawnCommandState() {
  hyperspawnControlMode = HS_CONTROL_NONE;
  hyperspawnPositionBasePending = hyperspawnPositionExtPending = false;
  hyperspawnTorqueBasePending = hyperspawnTorqueExtPending = false;
  for (int i = 0; i < HS_JOINT_COUNT; ++i) {
    hyperspawnTorqueSetpoints[i] = 0;
    hyperspawnTorquePending[i] = 0;
  }
}

void requestStop(uint8_t repeats = 3) {
  playMode = false;
  if (isHead) {
    stopBurstRemaining = 0;
    neckStopAll();
    return;
  }
  stopBurstRemaining = repeats;
  // In the HyperSpawn world STOP is a disarm operation. A subsequent arm must
  // come from a new complete ROS/CAN command (auto-arm) or an explicit 'play'.
  if (operatingMode == OPERATING_HYPERSPAWN_ROUTE) clearHyperspawnCommandState();
}

void clearAllTorqueSetpoints() {
  if (stateMutex && xSemaphoreTake(stateMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
    for (int i = 0; i < ACTUATOR_COUNT; ++i) {
      torqueValues[i] = 0;
      impedanceTorqueValues[i] = 0;
    }
    for (int i = 0; i < HS_JOINT_COUNT; ++i) {
      hyperspawnTorqueSetpoints[i] = 0;
      hyperspawnTorquePending[i] = 0;
    }
    xSemaphoreGive(stateMutex);
  }
  if (operatingMode == OPERATING_HYPERSPAWN_ROUTE) {
    hyperspawnControlMode = HS_CONTROL_NONE;
    hyperspawnPositionBasePending = hyperspawnPositionExtPending = false;
    hyperspawnTorqueBasePending = hyperspawnTorqueExtPending = false;
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
// Hyperspawn/dropbear_firmware parallel route
// -----------------------------------------------------------------------------

uint8_t hyperspawnNodeId() {
  return isLeft ? HS_NODE_LEFT_LEG : HS_NODE_RIGHT_LEG;
}

uint8_t hyperspawnLimbId() {
  return isLeft ? HS_LIMB_LEFT_LEG : HS_LIMB_RIGHT_LEG;
}

int hyperspawnJointToActuatorIndex(uint8_t joint) {
  const bool left = isLeft;
  switch (joint) {
    case HS_HIP_PITCH: return left ? LEFT_HIP_PITCH : RIGHT_HIP_PITCH;
    case HS_HIP_ROLL: return left ? LEFT_HIP_ROLL : RIGHT_HIP_ROLL;
    case HS_HIP_YAW: return left ? LEFT_HIP_YAW : RIGHT_HIP_YAW;
    case HS_KNEE: return left ? LEFT_KNEE : RIGHT_KNEE;
    case HS_OUTER_CALF: return left ? LEFT_OUTER_CALF : RIGHT_OUTER_CALF;
    case HS_INNER_CALF: return left ? LEFT_INNER_CALF : RIGHT_INNER_CALF;
    default: return -1;
  }
}

float hyperspawnMeasuredDegrees(uint8_t joint) {
  switch (joint) {
    case HS_HIP_PITCH: return normalizedHip;
    case HS_HIP_ROLL: return normalizedButt;
    case HS_KNEE: return normalizedKnee;
    case HS_OUTER_CALF: return normalizedOuter;
    case HS_INNER_CALF: return normalizedInner;
    case HS_HIP_YAW:
      // There is no external hip-yaw encoder in the original five-sensor leg.
      // Preserve the repository's open-loop state convention by echoing the
      // most recent position target for this one unobserved axis.
      return hyperspawnPositionUnitsPerDegree != 0.0f
                 ? static_cast<float>(hyperspawnPositionSetpoints[HS_HIP_YAW]) / hyperspawnPositionUnitsPerDegree
                 : 0.0f;
    default: return 0.0f;
  }
}

int16_t hyperspawnDegreesToWire(float degrees) {
  float value = degrees * hyperspawnPositionUnitsPerDegree;
  if (value > 32767.0f) value = 32767.0f;
  if (value < -32768.0f) value = -32768.0f;
  return static_cast<int16_t>(lroundf(value));
}

float hyperspawnWireToDegrees(int16_t value) {
  if (fabsf(hyperspawnPositionUnitsPerDegree) < 0.0001f) return static_cast<float>(value);
  return static_cast<float>(value) / hyperspawnPositionUnitsPerDegree;
}

void refreshHyperspawnJointState() {
  for (uint8_t i = 0; i < HS_JOINT_COUNT; ++i) {
    hyperspawnJointState[i] = hyperspawnDegreesToWire(hyperspawnMeasuredDegrees(i));
  }
}

void applyHyperspawnPositionTargets() {
  if (operatingMode != OPERATING_HYPERSPAWN_ROUTE || hyperspawnControlMode != HS_CONTROL_POSITION) return;

  const float hipPitch = hyperspawnWireToDegrees(hyperspawnPositionSetpoints[HS_HIP_PITCH]);
  const float hipRoll = hyperspawnWireToDegrees(hyperspawnPositionSetpoints[HS_HIP_ROLL]);
  const float knee = hyperspawnWireToDegrees(hyperspawnPositionSetpoints[HS_KNEE]);
  const float outer = hyperspawnWireToDegrees(hyperspawnPositionSetpoints[HS_OUTER_CALF]);
  const float inner = hyperspawnWireToDegrees(hyperspawnPositionSetpoints[HS_INNER_CALF]);

  if (isLeft) {
    hipPitchControlLeft.setDesiredPosition(hipPitch);
    hipPitchControlLeft.setDesiredVelocity(0.0f);
    hipRollControlLeft.setDesiredPosition(hipRoll);
    hipRollControlLeft.setDesiredVelocity(0.0f);
    kneeControlLeft.setDesiredPosition(knee);
    kneeControlLeft.setDesiredVelocity(0.0f);
    outerCalfControlLeft.setDesiredPosition(outer);
    outerCalfControlLeft.setDesiredVelocity(0.0f);
    innerCalfControlLeft.setDesiredPosition(inner);
    innerCalfControlLeft.setDesiredVelocity(0.0f);
  } else {
    hipPitchControlRight.setDesiredPosition(hipPitch);
    hipPitchControlRight.setDesiredVelocity(0.0f);
    hipRollControlRight.setDesiredPosition(hipRoll);
    hipRollControlRight.setDesiredVelocity(0.0f);
    kneeControlRight.setDesiredPosition(knee);
    kneeControlRight.setDesiredVelocity(0.0f);
    outerCalfControlRight.setDesiredPosition(outer);
    outerCalfControlRight.setDesiredVelocity(0.0f);
    innerCalfControlRight.setDesiredPosition(inner);
    innerCalfControlRight.setDesiredVelocity(0.0f);
  }
}

void markHyperspawnCommand(HyperspawnControlMode mode) {
  const uint32_t now = millis();
  hyperspawnControlMode = mode;
  hyperspawnLastCommandMs = now;
  hyperspawnWatchdogTripped = false;
  hyperspawnFaultCode = 0;
  if (mode == HS_CONTROL_POSITION) hyperspawnLastPositionMs = now;
  if (mode == HS_CONTROL_TORQUE) hyperspawnLastTorqueMs = now;
  if (hyperspawnAutoArm && runtimeControlReady) {
    stopBurstRemaining = 0;
    playMode = true;
  }
}

void decodeHyperspawnValues(const byte *data, byte len, uint8_t startJoint,
                            volatile int16_t *destination) {
  uint8_t joint = startJoint;
  for (uint8_t offset = 0; offset + 1 < len && joint < HS_JOINT_COUNT; offset += 2, ++joint) {
    destination[joint] = static_cast<int16_t>((static_cast<uint16_t>(data[offset]) << 8) |
                                              static_cast<uint16_t>(data[offset + 1]));
  }
}

bool isHyperspawnTargetedId(uint32_t id, uint8_t &msgType) {
  const uint8_t localNode = hyperspawnNodeId();
  const uint8_t node = static_cast<uint8_t>((id >> 4) & 0xFF);
  msgType = static_cast<uint8_t>(id & 0x0F);
  return node == localNode;
}

void expireHyperspawnFragments(uint32_t now) {
  if (hyperspawnPositionBasePending && now - hyperspawnPositionBaseMs > HS_FRAGMENT_TIMEOUT_MS) {
    hyperspawnPositionBasePending = false;
    hyperspawnPositionExtPending = false;
    hyperspawnFragmentTimeouts++;
  }
  if (hyperspawnPositionExtPending && now - hyperspawnPositionExtMs > HS_FRAGMENT_TIMEOUT_MS) {
    hyperspawnPositionBasePending = false;
    hyperspawnPositionExtPending = false;
    hyperspawnFragmentTimeouts++;
  }
  if (hyperspawnTorqueBasePending && now - hyperspawnTorqueBaseMs > HS_FRAGMENT_TIMEOUT_MS) {
    hyperspawnTorqueBasePending = false;
    hyperspawnTorqueExtPending = false;
    hyperspawnFragmentTimeouts++;
  }
  if (hyperspawnTorqueExtPending && now - hyperspawnTorqueExtMs > HS_FRAGMENT_TIMEOUT_MS) {
    hyperspawnTorqueBasePending = false;
    hyperspawnTorqueExtPending = false;
    hyperspawnFragmentTimeouts++;
  }
}

void completeHyperspawnPositionCommand() {
  // Temporarily disarm the route while swapping the complete staged command so
  // the 100 Hz output loop can observe either the old command or zero, never a
  // mixture of old/new joint values.
  hyperspawnControlMode = HS_CONTROL_NONE;
  for (uint8_t i = 0; i < HS_JOINT_COUNT; ++i) {
    hyperspawnPositionSetpoints[i] = hyperspawnPositionPending[i];
  }
  hyperspawnPositionBasePending = false;
  hyperspawnPositionExtPending = false;
  markHyperspawnCommand(HS_CONTROL_POSITION);
  applyHyperspawnPositionTargets();
  hyperspawnCompletedCommands++;
}

void completeHyperspawnTorqueCommand() {
  hyperspawnControlMode = HS_CONTROL_NONE;
  for (uint8_t i = 0; i < HS_JOINT_COUNT; ++i) {
    hyperspawnTorqueSetpoints[i] = hyperspawnTorquePending[i];
  }
  hyperspawnTorqueBasePending = false;
  hyperspawnTorqueExtPending = false;
  markHyperspawnCommand(HS_CONTROL_TORQUE);
  hyperspawnCompletedCommands++;
}

void handleHyperspawnRxFrame(uint32_t id, const byte *data, byte len) {
  if (operatingMode != OPERATING_HYPERSPAWN_ROUTE || isCenter || isHead) return;

  const uint32_t now = millis();
  expireHyperspawnFragments(now);

  uint8_t msgType = 0;
  const bool targeted = isHyperspawnTargetedId(id, msgType);
  bool legacy = false;

  const uint8_t nodeField = static_cast<uint8_t>((id >> 4) & 0xFF);
  if (!targeted && hyperspawnLegacyBroadcast && nodeField == HS_NODE_BRAIN) {
    msgType = static_cast<uint8_t>(id & 0x0F);
    legacy = true;
  }

  if (!targeted && !legacy) return;

  bool accepted = false;
  if (legacy) {
    // Exact compatibility with the current Hyperspawn/dropbear_firmware brain
    // frames. Those frames can contain at most four int16 values and cannot
    // identify a destination limb on a shared bus. Compatibility is therefore
    // explicit/opt-in. Missing position joints hold their measured position;
    // missing torque joints are forced to zero.
    if (msgType == HS_MSG_CMD_POS) {
      decodeHyperspawnValues(data, len, 0, hyperspawnPositionPending);
      hyperspawnPositionPending[HS_OUTER_CALF] = hyperspawnDegreesToWire(hyperspawnMeasuredDegrees(HS_OUTER_CALF));
      hyperspawnPositionPending[HS_INNER_CALF] = hyperspawnDegreesToWire(hyperspawnMeasuredDegrees(HS_INNER_CALF));
      completeHyperspawnPositionCommand();
      accepted = true;
    } else if (msgType == HS_MSG_CMD_TORQUE) {
      decodeHyperspawnValues(data, len, 0, hyperspawnTorquePending);
      hyperspawnTorquePending[HS_OUTER_CALF] = 0;
      hyperspawnTorquePending[HS_INNER_CALF] = 0;
      completeHyperspawnTorqueCommand();
      accepted = true;
    }
  } else {
    // Targeted extension: IDs are based on this limb node (0x12/0x13), and a
    // command is committed only after both the 4-joint base frame and 2-joint
    // extension frame have arrived within HS_FRAGMENT_TIMEOUT_MS.
    switch (msgType) {
      case HS_MSG_CMD_POS:
        decodeHyperspawnValues(data, len, 0, hyperspawnPositionPending);
        hyperspawnPositionBasePending = true;
        hyperspawnPositionExtPending = false;
        hyperspawnPositionBaseMs = now;
        accepted = true;
        break;
      case HS_MSG_CMD_POS_EXT:
        if (hyperspawnPositionBasePending && now - hyperspawnPositionBaseMs <= HS_FRAGMENT_TIMEOUT_MS) {
          decodeHyperspawnValues(data, len, 4, hyperspawnPositionPending);
          hyperspawnPositionExtPending = true;
          hyperspawnPositionExtMs = now;
          completeHyperspawnPositionCommand();
          accepted = true;
        }
        break;
      case HS_MSG_CMD_TORQUE:
        decodeHyperspawnValues(data, len, 0, hyperspawnTorquePending);
        hyperspawnTorqueBasePending = true;
        hyperspawnTorqueExtPending = false;
        hyperspawnTorqueBaseMs = now;
        accepted = true;
        break;
      case HS_MSG_CMD_TORQUE_EXT:
        if (hyperspawnTorqueBasePending && now - hyperspawnTorqueBaseMs <= HS_FRAGMENT_TIMEOUT_MS) {
          decodeHyperspawnValues(data, len, 4, hyperspawnTorquePending);
          hyperspawnTorqueExtPending = true;
          hyperspawnTorqueExtMs = now;
          completeHyperspawnTorqueCommand();
          accepted = true;
        }
        break;
      default:
        break;
    }
  }

  if (accepted) {
    hyperspawnRxFrames++;
    if (legacy) hyperspawnRxLegacyFrames++;
    if (targeted) hyperspawnRxTargetedFrames++;
  } else {
    hyperspawnRxRejectedFrames++;
  }
}
void sendHyperspawnState() {
  if (!canInitialized || isCenter || isHead || operatingMode != OPERATING_HYPERSPAWN_ROUTE) return;
  refreshHyperspawnJointState();

  byte base[8] = {0};
  for (uint8_t i = 0; i < 4; ++i) {
    const uint16_t raw = static_cast<uint16_t>(hyperspawnJointState[i]);
    base[i * 2] = static_cast<byte>((raw >> 8) & 0xFF);
    base[i * 2 + 1] = static_cast<byte>(raw & 0xFF);
  }
  const uint32_t baseId = (static_cast<uint32_t>(hyperspawnNodeId()) << 4) | HS_MSG_STATE;
  if (canSendFrame(baseId, base, 8)) hyperspawnStateFrames++;

  byte ext[4] = {0};
  for (uint8_t i = 0; i < 2; ++i) {
    const uint16_t raw = static_cast<uint16_t>(hyperspawnJointState[i + 4]);
    ext[i * 2] = static_cast<byte>((raw >> 8) & 0xFF);
    ext[i * 2 + 1] = static_cast<byte>(raw & 0xFF);
  }
  const uint32_t extId = (static_cast<uint32_t>(hyperspawnNodeId()) << 4) | HS_MSG_STATE_EXT;
  if (canSendFrame(extId, ext, 4)) hyperspawnStateFrames++;
}

void sendHyperspawnHeartbeat() {
  byte data[1] = {hyperspawnLimbId()};
  const uint32_t id = (static_cast<uint32_t>(hyperspawnNodeId()) << 4) | HS_MSG_HEARTBEAT;
  if (canSendFrame(id, data, 1)) hyperspawnHeartbeatFrames++;
}

void sendHyperspawnFault(uint8_t code) {
  hyperspawnFaultCode = code;
  byte data[1] = {code};
  const uint32_t id = (static_cast<uint32_t>(hyperspawnNodeId()) << 4) | HS_MSG_FAULT;
  if (canSendFrame(id, data, 1)) hyperspawnFaultFrames++;
}

void canReceiveTask(void *parameter) {
  while (true) {
    canRxTaskLoops++;
    lastCanRxTaskMs = millis();

    uint8_t drained = 0;
    while (canInitialized && drained < 12) {
      bool available = false;
      if (canMutex && xSemaphoreTake(canMutex, pdMS_TO_TICKS(2)) == pdTRUE) {
        available = CAN.checkReceive() == CAN_MSGAVAIL;
        if (available) {
          unsigned long rxId = 0;
          unsigned char len = 0;
          byte data[8] = {0};
          const int result = CAN.readMsgBuf(&rxId, &len, data);
          xSemaphoreGive(canMutex);

          if (result == CAN_OK) {
            canRxFrames++;
            lastCanRxMs = millis();
            handleHyperspawnRxFrame(static_cast<uint32_t>(rxId), data, len);
          } else {
            canRxErrors++;
          }
        } else {
          xSemaphoreGive(canMutex);
        }
      }

      if (!available) break;
      drained++;
    }
    vTaskDelay(pdMS_TO_TICKS(1));
  }
}
void hyperspawnRouteTask(void *parameter) {
  TickType_t lastWake = xTaskGetTickCount();
  uint32_t lastHeartbeat = 0;

  while (true) {
    hyperspawnTaskLoops++;
    lastHyperspawnTaskMs = millis();

    if (runtimeControlReady && operatingMode == OPERATING_HYPERSPAWN_ROUTE && !isCenter && !isHead) {
      const uint32_t now = millis();
      expireHyperspawnFragments(now);

      if (hyperspawnLastCommandMs != 0 &&
          now - hyperspawnLastCommandMs > hyperspawnCommandTimeoutMs &&
          playMode && !hyperspawnWatchdogTripped) {
        hyperspawnWatchdogTripped = true;
        hyperspawnWatchdogTrips++;
        hyperspawnControlMode = HS_CONTROL_NONE;
        hyperspawnPositionBasePending = hyperspawnPositionExtPending = false;
        hyperspawnTorqueBasePending = hyperspawnTorqueExtPending = false;
        for (int i = 0; i < HS_JOINT_COUNT; ++i) hyperspawnTorqueSetpoints[i] = 0;
        requestStop(3);
        sendHyperspawnFault(1); // command timeout
        dbPrintln("HYPERSPAWN WATCHDOG: command timeout; actuator stop burst queued.");
      }

      sendHyperspawnState();
      if (now - lastHeartbeat >= (1000UL / HS_HEARTBEAT_HZ)) {
        sendHyperspawnHeartbeat();
        lastHeartbeat = now;
      }
    }

    vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(1000 / HS_ROUTE_HZ));
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

  const int previous[5] = {
    readingsOuter[readIndex], readingsInner[readIndex], readingsHip[readIndex],
    readingsKnee[readIndex], readingsButt[readIndex]
  };
  int sample[5];
  for (int i = 0; i < 5; ++i) {
    const int decoded = as5600PseudoRaw(i);
    // Keep the last valid sample through a short missing-frame interval; health
    // diagnostics still expose the stale pulse independently.
    sample[i] = decoded >= 0 ? decoded : previous[i];
    updateSensorDiagnosticSample(i, decoded);
  }

  readingsOuter[readIndex] = sample[0];
  readingsInner[readIndex] = sample[1];
  readingsHip[readIndex] = sample[2];
  readingsKnee[readIndex] = sample[3];
  readingsButt[readIndex] = sample[4];

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
    updateSensorDiagnosticProcessed();
    return;
  }

  const int *offsets = isLeft ? leftLegOffsets : rightLegOffsets;
  normalizedOuter = wrapAngleFloat(outer + offsets[0]);
  normalizedInner = wrapAngleFloat(inner + offsets[1]);
  normalizedHip = wrapAngleFloat(hip + offsets[2]);
  normalizedKnee = wrapAngleFloat(kneeAngle + offsets[3]);
  normalizedButt = wrapAngleFloat(butt + offsets[4]);
  updateSensorDiagnosticProcessed();
}

void primeSensorFilter() {
  totalOuter = totalInner = totalHip = totalKnee = totalButt = 0;
  readIndex = 0;

  // Give the slowest supported AS5600 PWM mode (~115 Hz) time to produce at
  // least one complete frame on every encoder. Do not fabricate an ADC sample.
  const uint32_t waitStart = millis();
  bool allValid = false;
  while (millis() - waitStart < 250) {
    allValid = true;
    for (uint8_t sensor = 0; sensor < AS5600_SENSOR_COUNT; ++sensor) {
      if (as5600PseudoRaw(sensor) < 0) {
        allValid = false;
        break;
      }
    }
    if (allValid) break;
    delay(2);
  }

  if (!allValid) {
    dbPrintln("WARNING: one or more AS5600 PWM signals were not valid during filter priming; missing channels start at zero and diagnostics remain FAULT until pulses arrive.");
  }

  int seed[5] = {0, 0, 0, 0, 0};
  for (uint8_t sensor = 0; sensor < AS5600_SENSOR_COUNT; ++sensor) {
    const int raw = as5600PseudoRaw(sensor);
    seed[sensor] = raw >= 0 ? raw : 0;
    updateSensorDiagnosticSample(sensor, raw);
  }

  for (int i = 0; i < NUM_READINGS; ++i) {
    readingsOuter[i] = seed[0];
    readingsInner[i] = seed[1];
    readingsHip[i] = seed[2];
    readingsKnee[i] = seed[3];
    readingsButt[i] = seed[4];

    totalOuter += readingsOuter[i];
    totalInner += readingsInner[i];
    totalHip += readingsHip[i];
    totalKnee += readingsKnee[i];
    totalButt += readingsButt[i];
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
  return -1; // hip yaw does not have an external AS5600 OUT signal in this pinout
}

int getEncoderReading(String joint) {
  int sensorIndex = -1;
  if (joint.endsWith("outer_calf")) sensorIndex = 0;
  else if (joint.endsWith("inner_calf")) sensorIndex = 1;
  else if (joint.endsWith("hip_pitch")) sensorIndex = 2;
  else if (joint.endsWith("knee")) sensorIndex = 3;
  else if (joint.endsWith("hip_roll")) sensorIndex = 4;

  if (sensorIndex < 0) {
    dbPrintf("No external AS5600 one-wire encoder mapped for joint: %s\n", joint.c_str());
    return -1;
  }

  float angle = 0.0f, duty = 0.0f, freq = 0.0f;
  uint32_t period = 0, high = 0, age = 0;
  if (!readAs5600Pwm(sensorIndex, angle, duty, freq, period, high, age)) return -1;
  return static_cast<int>(lroundf(angle));
}

// -----------------------------------------------------------------------------
// RTOS tasks
// -----------------------------------------------------------------------------

void readAndComputeTask(void *parameter) {
  TickType_t lastWake = xTaskGetTickCount();
  unsigned long lastTelemetry = 0;

  while (true) {
    sensorTaskLoops++;
    lastSensorTaskMs = millis();
    if (!isCenter && !isHead) {
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
    impedanceTaskLoops++;
    lastImpedanceTaskMs = millis();
    if (!isCenter && !isHead) {
      const unsigned long now = millis();

      if (operatingMode == OPERATING_HYPERSPAWN_ROUTE && hyperspawnControlMode == HS_CONTROL_POSITION) {
        applyHyperspawnPositionTargets();
      }

      if (isLeft) {
        if ((operatingMode == OPERATING_HYPERSPAWN_ROUTE && hyperspawnControlMode == HS_CONTROL_POSITION) || impedanceEnabledLeftOuterCalf) {
          outerCalfControlLeft.update(normalizedOuter, now, outerCalfConstraintsLeft.minAngle, outerCalfConstraintsLeft.maxAngle);
          impedanceTorqueValues[LEFT_OUTER_CALF] = clampTorqueCommand(outerCalfControlLeft.torqueOutput * directionMultiplierLeftOuterCalf);
        } else impedanceTorqueValues[LEFT_OUTER_CALF] = 0;

        if ((operatingMode == OPERATING_HYPERSPAWN_ROUTE && hyperspawnControlMode == HS_CONTROL_POSITION) || impedanceEnabledLeftInnerCalf) {
          innerCalfControlLeft.update(normalizedInner, now, innerCalfConstraintsLeft.minAngle, innerCalfConstraintsLeft.maxAngle);
          impedanceTorqueValues[LEFT_INNER_CALF] = clampTorqueCommand(innerCalfControlLeft.torqueOutput * directionMultiplierLeftInnerCalf);
        } else impedanceTorqueValues[LEFT_INNER_CALF] = 0;

        if ((operatingMode == OPERATING_HYPERSPAWN_ROUTE && hyperspawnControlMode == HS_CONTROL_POSITION) || impedanceEnabledLeftKnee) {
          kneeControlLeft.update(normalizedKnee, now, kneeConstraintsLeft.minAngle, kneeConstraintsLeft.maxAngle);
          impedanceTorqueValues[LEFT_KNEE] = clampTorqueCommand(kneeControlLeft.torqueOutput * directionMultiplierLeftKnee);
        } else impedanceTorqueValues[LEFT_KNEE] = 0;

        if ((operatingMode == OPERATING_HYPERSPAWN_ROUTE && hyperspawnControlMode == HS_CONTROL_POSITION) || impedanceEnabledLeftHipPitch) {
          hipPitchControlLeft.update(normalizedHip, now, hipPitchConstraintsLeft.minAngle, hipPitchConstraintsLeft.maxAngle);
          impedanceTorqueValues[LEFT_HIP_PITCH] = clampTorqueCommand(hipPitchControlLeft.torqueOutput * directionMultiplierLeftHipPitch);
        } else impedanceTorqueValues[LEFT_HIP_PITCH] = 0;

        if ((operatingMode == OPERATING_HYPERSPAWN_ROUTE && hyperspawnControlMode == HS_CONTROL_POSITION) || impedanceEnabledLeftHipRoll) {
          hipRollControlLeft.update(normalizedButt, now, hipRollConstraintsLeft.minAngle, hipRollConstraintsLeft.maxAngle);
          impedanceTorqueValues[LEFT_HIP_ROLL] = clampTorqueCommand(hipRollControlLeft.torqueOutput * directionMultiplierLeftHipRoll);
        } else impedanceTorqueValues[LEFT_HIP_ROLL] = 0;
      } else {
        if ((operatingMode == OPERATING_HYPERSPAWN_ROUTE && hyperspawnControlMode == HS_CONTROL_POSITION) || impedanceEnabledRightOuterCalf) {
          outerCalfControlRight.update(normalizedOuter, now, outerCalfConstraintsRight.minAngle, outerCalfConstraintsRight.maxAngle);
          impedanceTorqueValues[RIGHT_OUTER_CALF] = clampTorqueCommand(outerCalfControlRight.torqueOutput * directionMultiplierRightOuterCalf);
        } else impedanceTorqueValues[RIGHT_OUTER_CALF] = 0;

        if ((operatingMode == OPERATING_HYPERSPAWN_ROUTE && hyperspawnControlMode == HS_CONTROL_POSITION) || impedanceEnabledRightInnerCalf) {
          innerCalfControlRight.update(normalizedInner, now, innerCalfConstraintsRight.minAngle, innerCalfConstraintsRight.maxAngle);
          impedanceTorqueValues[RIGHT_INNER_CALF] = clampTorqueCommand(innerCalfControlRight.torqueOutput * directionMultiplierRightInnerCalf);
        } else impedanceTorqueValues[RIGHT_INNER_CALF] = 0;

        if ((operatingMode == OPERATING_HYPERSPAWN_ROUTE && hyperspawnControlMode == HS_CONTROL_POSITION) || impedanceEnabledRightKnee) {
          kneeControlRight.update(normalizedKnee, now, kneeConstraintsRight.minAngle, kneeConstraintsRight.maxAngle);
          impedanceTorqueValues[RIGHT_KNEE] = clampTorqueCommand(kneeControlRight.torqueOutput * directionMultiplierRightKnee);
        } else impedanceTorqueValues[RIGHT_KNEE] = 0;

        if ((operatingMode == OPERATING_HYPERSPAWN_ROUTE && hyperspawnControlMode == HS_CONTROL_POSITION) || impedanceEnabledRightHipPitch) {
          hipPitchControlRight.update(normalizedHip, now, hipPitchConstraintsRight.minAngle, hipPitchConstraintsRight.maxAngle);
          impedanceTorqueValues[RIGHT_HIP_PITCH] = clampTorqueCommand(hipPitchControlRight.torqueOutput * directionMultiplierRightHipPitch);
        } else impedanceTorqueValues[RIGHT_HIP_PITCH] = 0;

        if ((operatingMode == OPERATING_HYPERSPAWN_ROUTE && hyperspawnControlMode == HS_CONTROL_POSITION) || impedanceEnabledRightHipRoll) {
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
    canTaskLoops++;
    lastCanTaskMs = millis();
    if (runtimeControlReady && !isCenter && !isHead) {
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
          int16_t value = 0;

          if (operatingMode == OPERATING_HYPERSPAWN_ROUTE) {
            if (hyperspawnControlMode == HS_CONTROL_TORQUE) {
              // Translate the selected leg's local actuator into the six-joint
              // Hyperspawn wire order.
              for (uint8_t j = 0; j < HS_JOINT_COUNT; ++j) {
                if (hyperspawnJointToActuatorIndex(j) == i) {
                  value = hyperspawnTorqueSetpoints[j];
                  break;
                }
              }
            } else if (hyperspawnControlMode == HS_CONTROL_POSITION) {
              // Five externally observed joints use the existing impedance
              // controller. Hip yaw has no external encoder, so position-mode
              // torque remains zero until a verified feedback source is added.
              if (i != (isLeft ? LEFT_HIP_YAW : RIGHT_HIP_YAW)) {
                value = impedanceTorqueValues[i];
              }
            }
          } else {
            value = torqueValues[i];
            if (isImpedanceEnabled(i)) value = impedanceTorqueValues[i];
          }

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
  webCommandsProcessed++;

  processRoutedCommand(command, "web");

  activeCommandCapture = nullptr;
  if (!capture.length()) appendWebLog(String("WEB #") + item.id + " < (no textual response)");
}

void checkChiralityTask(void *parameter) {
  while (true) {
    commandTaskLoops++;
    lastCommandTaskMs = millis();
    // USB Serial and web commands intentionally converge here.
    if (Serial.available() > 0) {
      String command = Serial.readStringUntil('\n');
      command.trim();
      if (command.length()) {
        serialCommandsProcessed++;
        appendWebLog("SERIAL > " + command);
        processRoutedCommand(command, "serial");
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
    imuTaskLoops++;
    lastImuTaskMs = millis();
    if (isCenter && !isHead && playMode) readIMU();
    vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(10));
  }
}

// -----------------------------------------------------------------------------
// HEAD / NECK controller
// -----------------------------------------------------------------------------

FastAccelStepper *neckStepperAt(uint8_t index) {
  return index < NECK_MOTOR_COUNT ? neckSteppers[index] : nullptr;
}

int32_t neckMmToSteps(float mm) {
  return (int32_t)lroundf(mm * neckStepsPerMm);
}

float neckStepsToMm(int32_t steps) {
  return neckStepsPerMm > 0.0001f ? ((float)steps / neckStepsPerMm) : 0.0f;
}

bool neckSetTargetSteps(uint8_t index, int32_t requestedSteps, bool bypass = false) {
  if (!runtimeNeckReady || index >= NECK_MOTOR_COUNT || !neckSteppers[index]) return false;
  if (!neckMotionEnabled && !bypass) {
    neckRejectedCommands++;
    dbPrintln("Neck motion rejected: motion is stopped. Use 'play' to re-arm.");
    return false;
  }

  int32_t target = requestedSteps;
  if (!bypass && !neckBypassLimits) {
    const int32_t minSteps = neckMmToSteps(neckMinMm[index]);
    const int32_t maxSteps = neckMmToSteps(neckMaxMm[index]);
    if (target < minSteps) target = minSteps;
    if (target > maxSteps) target = maxSteps;
  }

  neckTargetSteps[index] = target;
  neckSteppers[index]->moveTo(target);
  neckMoveCommands++;
  neckLastCommandMs = millis();
  return true;
}

bool neckMoveToMotorMm(uint8_t motorOneBased, float mm) {
  if (motorOneBased < 1 || motorOneBased > NECK_MOTOR_COUNT) {
    dbPrintln("Invalid neck motor number; expected 1..6.");
    neckRejectedCommands++;
    return false;
  }
  return neckSetTargetSteps(motorOneBased - 1, neckMmToSteps(mm));
}

void neckApplyMotionProfile(float speedMultiplier, float accelMultiplier) {
  if (speedMultiplier <= 0.0f) speedMultiplier = 1.0f;
  if (accelMultiplier <= 0.0f) accelMultiplier = 1.0f;
  uint32_t speed = (uint32_t)max(1.0f, (float)neckSpeedHz * speedMultiplier);
  uint32_t accel = (uint32_t)max(1.0f, (float)neckAcceleration * accelMultiplier);
  for (uint8_t i = 0; i < NECK_MOTOR_COUNT; ++i) {
    if (neckSteppers[i]) {
      neckSteppers[i]->setSpeedInHz(speed);
      neckSteppers[i]->setAcceleration(accel);
    }
  }
}

void neckMoveHead(int angleX, int angleY, int angleZ, int heightOffset,
                  float speedMultiplier, float accelMultiplier, int roll, int pitch,
                  bool bypass = false) {
  if (!runtimeNeckReady) {
    dbPrintln("Neck command rejected: HEAD_NECK runtime is not active.");
    return;
  }
  if (!neckMotionEnabled && !bypass) {
    dbPrintln("Neck command rejected: motion is stopped. Use 'play' to re-arm.");
    neckRejectedCommands++;
    return;
  }

  const float pitchScale = 10.0f;
  const float rollScale = 10.0f;
  const float yawScale = 10.0f;
  const float rollMovementScale = 10.0f;
  const float pitchMovementScale = 10.0f;

  int32_t moves[6];
  moves[0] = (int32_t)lroundf(-angleX * pitchScale + angleY * rollScale + angleZ * yawScale + pitch * pitchMovementScale + roll * rollMovementScale);
  moves[1] = (int32_t)lroundf( angleX * pitchScale - angleY * rollScale - angleZ * yawScale + pitch * pitchMovementScale + roll * rollMovementScale);
  moves[2] = (int32_t)lroundf(-angleX * pitchScale - angleY * rollScale - angleZ * yawScale - pitch * pitchMovementScale + roll * rollMovementScale);
  moves[3] = (int32_t)lroundf( angleX * pitchScale + angleY * rollScale - angleZ * yawScale - pitch * pitchMovementScale - roll * rollMovementScale);
  moves[4] = (int32_t)lroundf(-angleX * pitchScale + angleY * rollScale - angleZ * yawScale + pitch * pitchMovementScale - roll * rollMovementScale);
  moves[5] = (int32_t)lroundf( angleX * pitchScale - angleY * rollScale + angleZ * yawScale + pitch * pitchMovementScale - roll * rollMovementScale);

  const int32_t heightMovement = (int32_t)lroundf(heightOffset * NECK_POSE_HEIGHT_SCALE);
  for (uint8_t i = 0; i < NECK_MOTOR_COUNT; ++i) moves[i] += heightMovement;

  // Preserve the original Dropbear-Neck-Assembly pose envelope: computed
  // Stewart targets are constrained to 0..80 mm-equivalent at 400 steps/mm
  // during ordinary motion. Per-actuator configurable limits are then applied
  // by neckSetTargetSteps() as a second safety boundary. Homing bypasses both.
  if (!bypass && !neckBypassLimits) {
    const int32_t poseMinSteps = (int32_t)lroundf(NECK_DEFAULT_MIN_MM * NECK_POSE_HEIGHT_SCALE);
    const int32_t poseMaxSteps = (int32_t)lroundf(NECK_DEFAULT_MAX_MM * NECK_POSE_HEIGHT_SCALE);
    for (uint8_t i = 0; i < NECK_MOTOR_COUNT; ++i) {
      if (moves[i] < poseMinSteps) moves[i] = poseMinSteps;
      if (moves[i] > poseMaxSteps) moves[i] = poseMaxSteps;
    }
  }

  neckApplyMotionProfile(speedMultiplier, accelMultiplier);
  bool previousBypass = neckBypassLimits;
  if (bypass) neckBypassLimits = true;
  for (uint8_t i = 0; i < NECK_MOTOR_COUNT; ++i) neckSetTargetSteps(i, moves[i], bypass);
  neckBypassLimits = previousBypass;

  neckLastPose.x = angleX;
  neckLastPose.y = angleY;
  neckLastPose.z = angleZ;
  neckLastPose.height = heightOffset;
  neckLastPose.roll = roll;
  neckLastPose.pitch = pitch;
  neckLastPose.speedMultiplier = speedMultiplier;
  neckLastPose.accelMultiplier = accelMultiplier;
}

void neckZeroAll() {
  for (uint8_t i = 0; i < NECK_MOTOR_COUNT; ++i) {
    if (neckSteppers[i]) {
      neckSteppers[i]->stopMove();
      neckSteppers[i]->setCurrentPosition(0);
    }
    neckTargetSteps[i] = 0;
  }
  neckSoftwareHomed = true;
  dbPrintln("HEAD_NECK software positions zeroed. This is open-loop and does not verify physical actuator position.");
}

void neckStopAll() {
  neckMotionEnabled = false;
  neckHomeState = NECK_HOME_IDLE;
  neckBypassLimits = false;
  for (uint8_t i = 0; i < NECK_MOTOR_COUNT; ++i) {
    if (neckSteppers[i]) {
      neckSteppers[i]->stopMove();
      neckTargetSteps[i] = neckSteppers[i]->getCurrentPosition();
    }
  }
  neckStopCommands++;
}

void neckStartSoftHome() {
  if (!runtimeNeckReady) return;
  playMode = true;
  neckMotionEnabled = true;
  neckSoftwareHomed = false;
  neckHomePreviousBypass = neckBypassLimits;
  neckBypassLimits = true;
  neckMoveHead(0,0,0,NECK_SOFT_HOME_HEIGHT_MM,NECK_SOFT_HOME_SPEED_MULT,NECK_SOFT_HOME_ACCEL_MULT,0,0,true);
  neckHomeState = NECK_HOME_SOFT_SETTLE;
  neckHomeDeadlineMs = millis() + NECK_SOFT_HOME_SETTLE_MS;
  dbPrintln("HEAD_NECK HOME_SOFT started (open-loop overtravel + software zero).");
}

void neckStartBruteHome() {
  if (!runtimeNeckReady) return;
  playMode = true;
  neckMotionEnabled = true;
  neckSoftwareHomed = false;
  neckHomePreviousBypass = neckBypassLimits;
  neckBypassLimits = true;
  neckMoveHead(0,0,0,NECK_BRUTE_PREP_HEIGHT_MM,NECK_BRUTE_PREP_SPEED_MULT,NECK_BRUTE_PREP_ACCEL_MULT,0,0,true);
  neckHomeState = NECK_HOME_BRUTE_PREP_SETTLE;
  neckHomeDeadlineMs = millis() + NECK_BRUTE_PREP_SETTLE_MS;
  dbPrintln("HEAD_NECK HOME_BRUTE started (open-loop overtravel + software zero).");
}

void neckServiceHomeState() {
  if (neckHomeState == NECK_HOME_IDLE) return;
  const uint32_t now = millis();
  if ((int32_t)(now - neckHomeDeadlineMs) < 0) return;

  switch (neckHomeState) {
    case NECK_HOME_SOFT_SETTLE:
      neckZeroAll();
      neckBypassLimits = neckHomePreviousBypass;
      neckHomeState = NECK_HOME_IDLE;
      dbPrintln("HEAD_NECK HOME_SOFT complete; software zero established.");
      break;
    case NECK_HOME_BRUTE_PREP_SETTLE:
      neckHomeState = NECK_HOME_BRUTE_GAP;
      neckHomeDeadlineMs = now + 150;
      break;
    case NECK_HOME_BRUTE_GAP:
      neckMoveHead(0,0,0,NECK_BRUTE_HOME_HEIGHT_MM,NECK_BRUTE_HOME_SPEED_MULT,NECK_BRUTE_HOME_ACCEL_MULT,0,0,true);
      neckHomeState = NECK_HOME_BRUTE_FINAL_SETTLE;
      neckHomeDeadlineMs = now + NECK_BRUTE_HOME_SETTLE_MS;
      break;
    case NECK_HOME_BRUTE_FINAL_SETTLE:
      neckZeroAll();
      neckBypassLimits = neckHomePreviousBypass;
      neckHomeState = NECK_HOME_IDLE;
      dbPrintln("HEAD_NECK HOME_BRUTE complete; software zero established.");
      break;
    default:
      neckHomeState = NECK_HOME_IDLE;
      neckBypassLimits = false;
      break;
  }
}

void neckHandleQuaternion(String command) {
  command = command.substring(1);
  command.trim();
  if (command.startsWith(":")) { command = command.substring(1); command.trim(); }

  String tokens[8];
  int tokenCount = 0;
  int start = 0;
  while (start <= (int)command.length() && tokenCount < 8) {
    int comma = command.indexOf(',', start);
    if (comma < 0) comma = command.length();
    tokens[tokenCount++] = command.substring(start, comma);
    start = comma + 1;
    if (comma >= (int)command.length()) break;
  }
  if (tokenCount < 4) { dbPrintln("Invalid quaternion command: expected Q:w,x,y,z[,Hn][,Sn][,An]"); return; }

  float q[4];
  for (int i = 0; i < 4; ++i) q[i] = tokens[i].toFloat();
  float norm = sqrtf(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
  if (norm <= 0.000001f) { dbPrintln("Invalid quaternion: zero norm."); return; }
  for (int i = 0; i < 4; ++i) q[i] /= norm;

  int height = 0;
  float speed = 1.0f, accel = 1.0f;
  for (int i = 4; i < tokenCount; ++i) {
    tokens[i].trim();
    if (tokens[i].startsWith("H")) height = tokens[i].substring(1).toInt();
    else if (tokens[i].startsWith("S")) speed = tokens[i].substring(1).toFloat();
    else if (tokens[i].startsWith("A")) accel = tokens[i].substring(1).toFloat();
  }

  const float rollRad = atan2f(2.0f*(q[0]*q[1] + q[2]*q[3]), 1.0f - 2.0f*(q[1]*q[1] + q[2]*q[2]));
  float sinPitch = 2.0f*(q[0]*q[2] - q[3]*q[1]);
  sinPitch = max(-1.0f, min(1.0f, sinPitch));
  const float pitchRad = asinf(sinPitch);
  const float yawRad = atan2f(2.0f*(q[0]*q[3] + q[1]*q[2]), 1.0f - 2.0f*(q[2]*q[2] + q[3]*q[3]));
  const int rollDeg = (int)lroundf(rollRad * 180.0f / PI);
  const int pitchDeg = (int)lroundf(pitchRad * 180.0f / PI);
  const int yawDeg = (int)lroundf(yawRad * 180.0f / PI);
  dbPrintf("Quaternion -> yaw=%d pitch=%d roll=%d deg\n", yawDeg, pitchDeg, rollDeg);
  neckMoveHead(yawDeg, pitchDeg, rollDeg, height, speed, accel, 0, 0);
}

void neckHandleDirectCommand(const String &command) {
  int start = 0;
  while (start < (int)command.length()) {
    int comma = command.indexOf(',', start);
    if (comma < 0) comma = command.length();
    String token = command.substring(start, comma);
    token.trim();
    int colon = token.indexOf(':');
    if (colon > 0) {
      uint8_t motor = (uint8_t)token.substring(0, colon).toInt();
      float mm = token.substring(colon + 1).toFloat();
      neckMoveToMotorMm(motor, mm);
    }
    start = comma + 1;
  }
}

void neckHandlePoseCommand(const String &command) {
  int x=0,y=0,z=0,h=0,r=0,p=0;
  float speed=1.0f, accel=1.0f;
  int start = 0;
  while (start < (int)command.length()) {
    int comma = command.indexOf(',', start);
    if (comma < 0) comma = command.length();
    String token = command.substring(start, comma);
    token.trim();
    if (token.length()) {
      char axis = toupper(token.charAt(0));
      float value = token.substring(1).toFloat();
      switch (axis) {
        case 'X': x = (int)lroundf(value); break;
        case 'Y': y = (int)lroundf(value); break;
        case 'Z': z = (int)lroundf(value); break;
        case 'H': h = (int)lroundf(value); break;
        case 'S': speed = value; break;
        case 'A': accel = value; break;
        case 'R': r = (int)lroundf(value); break;
        case 'P': p = (int)lroundf(value); break;
      }
    }
    start = comma + 1;
  }
  neckMoveHead(x,y,z,h,speed,accel,r,p);
}

void printNeckStatus() {
  dbPrintf("HEAD_NECK runtime:%d motion:%d BT:%d homed:%d home_state:%u speed:%luHz accel:%lu steps/s^2 steps/mm:%.3f\n",
           runtimeNeckReady, neckMotionEnabled, neckBluetoothStarted, neckSoftwareHomed,
           (unsigned int)neckHomeState, (unsigned long)neckSpeedHz, (unsigned long)neckAcceleration, neckStepsPerMm);
  for (uint8_t i=0;i<NECK_MOTOR_COUNT;++i) {
    int32_t current = neckSteppers[i] ? neckSteppers[i]->getCurrentPosition() : 0;
    bool moving = neckSteppers[i] ? neckSteppers[i]->isRunning() : false;
    dbPrintf("  M%u STEP=%d DIR=%d current=%ld (%.2fmm) target=%ld (%.2fmm) moving=%d limits=%.2f..%.2fmm\n",
             i+1, NECK_STEP_PINS[i], NECK_DIR_PINS[i], (long)current, neckStepsToMm(current),
             (long)neckTargetSteps[i], neckStepsToMm(neckTargetSteps[i]), moving,
             neckMinMm[i], neckMaxMm[i]);
  }
  dbPrintln("  Feedback: OPEN_LOOP step count only; no physical encoder/limit-switch verification in the neck repository hardware stack.");
}

void neckEmitHealth() {
  String line = "HEALTH|DEVICE=NECK|ROLE=STEWART_NECK|PROTO=1|UPTIME_MS=" + String(millis()) +
                "|BAUD=115200|BT_NAME=" + String(NECK_BT_NAME) +
                "|MOTORS=6|SPEED_HZ=" + String(neckSpeedHz) +
                "|ACCEL=" + String(neckAcceleration) +
                "|BYPASS_CLAMP=" + String(neckBypassLimits ? 1 : 0) +
                "|HOMED=" + String(neckSoftwareHomed ? 1 : 0) +
                "|MOTION=" + String(neckMotionEnabled ? 1 : 0) +
                "|FEEDBACK=OPEN_LOOP";
  dbPrintln(line);
  if (neckBluetoothStarted && neckBluetooth.hasClient()) neckBluetooth.println(line);
}

bool processNeckCommand(String command, const char *source) {
  command.trim();
  if (!isHead || !command.length()) return false;

  // Preserve the source project's parseAndMove() behavior: multiple neck
  // commands may be chained with '|'. Each segment is executed in order.
  if (command.indexOf('|') >= 0) {
    int start = 0;
    while (start <= (int)command.length()) {
      int sep = command.indexOf('|', start);
      if (sep < 0) sep = command.length();
      String part = command.substring(start, sep);
      part.trim();
      if (part.length()) processNeckCommand(part, source);
      if (sep >= (int)command.length()) break;
      start = sep + 1;
    }
    return true;
  }

  String upper = command; upper.toUpperCase();

  if (strcmp(source, "bluetooth") == 0) neckBluetoothCommands++;
  else if (strcmp(source, "web") == 0) neckPortalCommands++;
  else neckSerialCommands++;
  neckLastCommandMs = millis();

  if (upper == "HEALTH" || upper == "STATUS") { neckEmitHealth(); return true; }
  if (upper == "HOME" || upper == "HOME_BRUTE") { neckStartBruteHome(); return true; }
  if (upper == "HOME_SOFT") { neckStartSoftHome(); return true; }
  if (upper == "NECK STOP") { playMode = false; neckStopAll(); dbPrintln("HEAD_NECK stopped."); return true; }
  if (upper == "NECK ZERO") { neckZeroAll(); return true; }
  if (upper == "NECK STATUS") { printNeckStatus(); return true; }

  if (upper.startsWith("NECK SPEED ")) {
    uint32_t v = (uint32_t)command.substring(11).toInt();
    if (v >= 1 && v <= 200000) { neckSpeedHz=v; neckApplyMotionProfile(1,1); saveConfig(); dbPrintf("Neck speed=%lu Hz\n",(unsigned long)v); }
    else dbPrintln("Usage: neck speed <1..200000 Hz>");
    return true;
  }
  if (upper.startsWith("NECK ACCEL ")) {
    uint32_t v = (uint32_t)command.substring(11).toInt();
    if (v >= 1 && v <= 2000000) { neckAcceleration=v; neckApplyMotionProfile(1,1); saveConfig(); dbPrintf("Neck acceleration=%lu steps/s^2\n",(unsigned long)v); }
    else dbPrintln("Usage: neck accel <1..2000000>");
    return true;
  }
  if (upper.startsWith("NECK BLUETOOTH ")) {
    String v=command.substring(15); v.trim(); v.toLowerCase();
    bool requested = (v=="on"||v=="1"||v=="true");
    neckBluetoothEnabled=requested; saveConfig();
    dbPrintln(String("Neck Bluetooth persisted ")+(requested?"ON":"OFF")+"; reboot required for radio topology change.");
    rebootRequired=true;
    return true;
  }
  if (upper.startsWith("NECK AUTOHOME ")) {
    String v=command.substring(14); v.trim(); v.toLowerCase();
    neckAutoHome=(v=="on"||v=="1"||v=="true"); saveConfig();
    dbPrintln(String("Neck boot auto-home ")+(neckAutoHome?"ON":"OFF"));
    return true;
  }
  if (upper.startsWith("NECK LIMITS ")) {
    String args=command.substring(12); args.trim();
    int p1=args.indexOf(' '), p2=p1>=0?args.indexOf(' ',p1+1):-1;
    if (p1<0||p2<0) { dbPrintln("Usage: neck limits <motor 1..6> <min_mm> <max_mm>"); return true; }
    int m=args.substring(0,p1).toInt(); float mn=args.substring(p1+1,p2).toFloat(), mx=args.substring(p2+1).toFloat();
    if (m<1||m>6||mn>mx) { dbPrintln("Invalid neck limits."); return true; }
    neckMinMm[m-1]=mn; neckMaxMm[m-1]=mx; saveConfig();
    dbPrintf("Neck M%d limits %.2f..%.2f mm\n",m,mn,mx); return true;
  }

  if (!runtimeNeckReady) { dbPrintln("HEAD_NECK runtime is not active; reboot after selecting head role."); return true; }
  if (upper.startsWith("Q")) { neckHandleQuaternion(command); return true; }
  if (command.indexOf(':') >= 0) { neckHandleDirectCommand(command); return true; }

  char c=toupper(command.charAt(0));
  if (c=='X'||c=='Y'||c=='Z'||c=='H'||c=='S'||c=='A'||c=='R'||c=='P') {
    neckHandlePoseCommand(command); return true;
  }
  return false;
}

void setupNeckHardware() {
  neckEngine.init();
  uint8_t connected = 0;
  for (uint8_t i=0;i<NECK_MOTOR_COUNT;++i) {
    neckSteppers[i]=neckEngine.stepperConnectToPin(NECK_STEP_PINS[i]);
    if (!neckSteppers[i]) {
      dbPrintf("ERROR: neck M%u failed to attach STEP GPIO%d.\n",i+1,NECK_STEP_PINS[i]);
      continue;
    }
    neckSteppers[i]->setDirectionPin(NECK_DIR_PINS[i]);
    if (neckUseEnablePin) {
      neckSteppers[i]->setEnablePin(NECK_ENABLE_PIN);
      neckSteppers[i]->setAutoEnable(true);
    }
    neckSteppers[i]->setSpeedInHz(neckSpeedHz);
    neckSteppers[i]->setAcceleration(neckAcceleration);
    neckTargetSteps[i]=neckSteppers[i]->getCurrentPosition();
    connected++;
  }
  runtimeNeckReady = connected == NECK_MOTOR_COUNT;
  runtimeControlReady = false;
  runtimeImuReady = false;
  neckMotionEnabled = runtimeNeckReady;
  playMode = runtimeNeckReady;

  if (neckBluetoothEnabled) {
    neckBluetooth.begin(NECK_BT_NAME);
    neckBluetooth.setTimeout(25);
    neckBluetoothStarted = true;
  }

  dbPrintf("HEAD_NECK initialized: %u/6 steppers; SSID=%s; Bluetooth=%s.\n",
           connected, desiredPortalSSID().c_str(), neckBluetoothStarted?NECK_BT_NAME:"disabled");
  if (!runtimeNeckReady) dbPrintln("WARNING: one or more FastAccelStepper channels failed to initialize.");
}

void neckServiceTask(void *parameter) {
  while (true) {
    neckServiceLoops++;
    lastNeckServiceMs=millis();
    if (isHead && runtimeNeckReady) {
      neckServiceHomeState();
      if (neckBluetoothStarted && neckBluetooth.available()) {
        String input=neckBluetooth.readStringUntil('\n'); input.trim();
        if (input.length()) {
          appendWebLog("BT NECK > "+input);
          processRoutedCommand(input, "bluetooth");
        }
      }
    }
    vTaskDelay(pdMS_TO_TICKS(5));
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

bool parseSixFloats(const String &line, const char *prefix, float out[6]) {
  if (!line.startsWith(prefix)) return false;
  String values=line.substring(strlen(prefix));
  int cursor=0;
  for(int i=0;i<6;++i){
    int comma=values.indexOf(',',cursor);
    String token=(i==5||comma<0)?values.substring(cursor):values.substring(cursor,comma);
    token.trim(); if(!token.length()) return false; out[i]=token.toFloat();
    if(i<5){ if(comma<0) return false; cursor=comma+1; }
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
    spiffsErrors++;
    dbPrintln("Failed to acquire SPIFFS lock for configuration save.");
    return;
  }

  File file = SPIFFS.open("/config.txt", FILE_WRITE);
  if (!file) {
    spiffsErrors++;
    if (spiffsMutex) xSemaphoreGive(spiffsMutex);
    dbPrintln("Failed to open /config.txt for writing.");
    return;
  }

  // Keep the legacy field order first so a rollback to the pre-portal
  // firmware still reads role, offsets, directions, and all constraints.
  file.printf("LegSide:%s\n", isHead ? "head" : (isCenter ? "center" : (isLeft ? "left" : "right")));
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
  file.println("ConfigVersion:6");
  file.printf("MaxTorqueLimit:%.3f\n", maxTorqueLimit);
  file.printf("OperatingMode:%s\n", operatingModeName().c_str());
  file.printf("CommandProtocol:DB1\n");
  file.printf("LegacyUnaddressedCommands:%d\n", legacyUnaddressedCommands ? 1 : 0);
  file.printf("HyperspawnCommandTimeoutMs:%lu\n", (unsigned long)hyperspawnCommandTimeoutMs);
  file.printf("HyperspawnLegacyBroadcast:%d\n", hyperspawnLegacyBroadcast ? 1 : 0);
  file.printf("HyperspawnAutoArm:%d\n", hyperspawnAutoArm ? 1 : 0);
  file.printf("HyperspawnPositionUnitsPerDegree:%.6f\n", hyperspawnPositionUnitsPerDegree);
  file.printf("DeviceRole:%s\n", selectedRoleName().c_str());
  file.printf("NeckSpeedHz:%lu\n", (unsigned long)neckSpeedHz);
  file.printf("NeckAcceleration:%lu\n", (unsigned long)neckAcceleration);
  file.printf("NeckStepsPerMm:%.6f\n", neckStepsPerMm);
  file.printf("NeckBluetoothEnabled:%d\n", neckBluetoothEnabled ? 1 : 0);
  file.printf("NeckAutoHome:%d\n", neckAutoHome ? 1 : 0);
  file.printf("NeckUseEnablePin:%d\n", neckUseEnablePin ? 1 : 0);
  file.print("NeckMinMm:"); for (int i=0;i<NECK_MOTOR_COUNT;++i) { if(i) file.print(','); file.print(neckMinMm[i],3); } file.println();
  file.print("NeckMaxMm:"); for (int i=0;i<NECK_MOTOR_COUNT;++i) { if(i) file.print(','); file.print(neckMaxMm[i],3); } file.println();

  file.close();
  spiffsWriteOps++;
  lastSpiffsWriteMs = millis();
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
    spiffsErrors++;
    dbPrintln("Failed to acquire SPIFFS lock for configuration load.");
    return false;
  }

  File file = SPIFFS.open("/config.txt", FILE_READ);
  if (!file) {
    spiffsErrors++;
    if (spiffsMutex) xSemaphoreGive(spiffsMutex);
    dbPrintln("Failed to open /config.txt. Starting unconfigured.");
    return false;
  }

  spiffsReadOps++;
  lastSpiffsReadMs = millis();

  bool validRole = false;
  while (file.available()) {
    String line = file.readStringUntil('\n');
    line.trim();
    if (!line.length()) continue;

    if (line.startsWith("LegSide:")) {
      String side = line.substring(8);
      side.trim();
      if (side == "left") {
        isLeft = true; isCenter = false; isHead = false; validRole = true;
      } else if (side == "right") {
        isLeft = false; isCenter = false; isHead = false; validRole = true;
      } else if (side == "center") {
        isCenter = true; isHead = false; validRole = true;
      } else if (side == "head" || side == "neck" || side == "head_neck") {
        isHead = true; isCenter = false; validRole = true;
      }
    } else if (line.startsWith("MaxTorqueLimit:")) {
      const float value = line.substring(15).toFloat();
      if (value > 0.0f && value <= 100.0f) maxTorqueLimit = value;
    } else if (line.startsWith("OperatingMode:")) {
      String mode = line.substring(14);
      mode.trim();
      operatingMode = mode == "hyperspawn" ? OPERATING_HYPERSPAWN_ROUTE : OPERATING_STANDALONE;
    } else if (line.startsWith("LegacyUnaddressedCommands:")) {
      legacyUnaddressedCommands = line.substring(26).toInt() != 0;
    } else if (line.startsWith("HyperspawnCommandTimeoutMs:")) {
      const uint32_t v = (uint32_t)line.substring(27).toInt();
      if (v >= 50 && v <= 10000) hyperspawnCommandTimeoutMs = v;
    } else if (line.startsWith("HyperspawnLegacyBroadcast:")) {
      hyperspawnLegacyBroadcast = line.substring(26).toInt() != 0;
    } else if (line.startsWith("HyperspawnAutoArm:")) {
      hyperspawnAutoArm = line.substring(18).toInt() != 0;
    } else if (line.startsWith("HyperspawnPositionUnitsPerDegree:")) {
      const float v = line.substring(33).toFloat();
      if (v > 0.0001f && v <= 1000.0f) hyperspawnPositionUnitsPerDegree = v;
    } else if (line.startsWith("DeviceRole:")) {
      String role=line.substring(11); role.trim();
      if(role=="head"||role=="neck"||role=="head_neck") { isHead=true; isCenter=false; validRole=true; }
      else if(role=="center") { isHead=false; isCenter=true; validRole=true; }
      else if(role=="left") { isHead=false; isCenter=false; isLeft=true; validRole=true; }
      else if(role=="right") { isHead=false; isCenter=false; isLeft=false; validRole=true; }
    } else if (line.startsWith("NeckSpeedHz:")) {
      uint32_t v=(uint32_t)line.substring(12).toInt(); if(v>=1&&v<=200000) neckSpeedHz=v;
    } else if (line.startsWith("NeckAcceleration:")) {
      uint32_t v=(uint32_t)line.substring(17).toInt(); if(v>=1&&v<=2000000) neckAcceleration=v;
    } else if (line.startsWith("NeckStepsPerMm:")) {
      float v=line.substring(15).toFloat(); if(v>0.01f&&v<100000.0f) neckStepsPerMm=v;
    } else if (line.startsWith("NeckBluetoothEnabled:")) {
      neckBluetoothEnabled=line.substring(21).toInt()!=0;
    } else if (line.startsWith("NeckAutoHome:")) {
      neckAutoHome=line.substring(13).toInt()!=0;
    } else if (line.startsWith("NeckUseEnablePin:")) {
      neckUseEnablePin=line.substring(17).toInt()!=0;
    } else if (line.startsWith("NeckMinMm:")) {
      parseSixFloats(line,"NeckMinMm:",neckMinMm);
    } else if (line.startsWith("NeckMaxMm:")) {
      parseSixFloats(line,"NeckMaxMm:",neckMaxMm);
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
  if (validRole && (isHead || isCenter)) {
    // ControlRoute applies only to leg hardware personalities. Prevent a stale
    // leg HyperSpawn selection from surfacing as active on HEAD/CENTER.
    operatingMode = OPERATING_STANDALONE;
    hyperspawnControlMode = HS_CONTROL_NONE;
  }
  if (validRole) {
    dbPrintln("Configuration loaded successfully.");
  } else {
    dbPrintln("/config.txt exists but has no valid DeviceRole/LegSide. Starting captive setup portal with motion tasks disabled.");
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
  if (isCenter || isHead || !configProvisioned) {
    dbPrintln("Sensor calibration is only available on a configured leg controller.");
    return;
  }

  // Acquire an independent PWM-derived average. The calibration path uses the
  // same AS5600 OUT decoding as the real-time control path; it never falls back
  // to ADC2/analogRead while Wi-Fi is active.
  long sums[5] = {0, 0, 0, 0, 0};
  int validCounts[5] = {0, 0, 0, 0, 0};
  const uint32_t calibrationDeadline = millis() + 500;
  while (millis() < calibrationDeadline) {
    for (uint8_t sensor = 0; sensor < AS5600_SENSOR_COUNT; ++sensor) {
      if (validCounts[sensor] >= NUM_READINGS) continue;
      const int raw = as5600PseudoRaw(sensor);
      if (raw >= 0) {
        sums[sensor] += raw;
        validCounts[sensor]++;
      }
    }

    bool complete = true;
    for (uint8_t sensor = 0; sensor < AS5600_SENSOR_COUNT; ++sensor) {
      if (validCounts[sensor] < NUM_READINGS) {
        complete = false;
        break;
      }
    }
    if (complete) break;
    // 10 ms guarantees a fresh frame even at the slowest supported 115 Hz mode.
    delay(10);
  }

  for (uint8_t sensor = 0; sensor < AS5600_SENSOR_COUNT; ++sensor) {
    if (validCounts[sensor] < NUM_READINGS) {
      dbPrintf("Calibration aborted: AS5600 sensor %u supplied only %d/%d valid PWM samples.\n",
               sensor, validCounts[sensor], NUM_READINGS);
      return;
    }
  }

  const float rawOuter = adcToDegrees(static_cast<float>(sums[0]) / validCounts[0]);
  const float rawInner = adcToDegrees(static_cast<float>(sums[1]) / validCounts[1]);
  const float rawHip = adcToDegrees(static_cast<float>(sums[2]) / validCounts[2]);
  const float rawKnee = adcToDegrees(static_cast<float>(sums[3]) / validCounts[3]);
  const float rawButt = adcToDegrees(static_cast<float>(sums[4]) / validCounts[4]);

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
  if (!isLegRole()) {
    dbPrintln("Saved leg offsets are only available on LEFT_LEG or RIGHT_LEG roles.");
    return;
  }
  const int *offsets = isLeft ? leftLegOffsets : rightLegOffsets;
  dbPrintf("%s offsets - Outer:%d Inner:%d HipPitch:%d Knee:%d HipRoll:%d\n",
                isLeft ? "Left" : "Right",
                offsets[0], offsets[1], offsets[2], offsets[3], offsets[4]);
}

// -----------------------------------------------------------------------------
// IMU / misc
// -----------------------------------------------------------------------------

bool selectImuMuxChannel(uint8_t channel) {
  if (channel > 7) return false;
  Wire.beginTransmission(IMU_MUX_ADDRESS);
  Wire.write(static_cast<uint8_t>(1U << channel));
  const uint8_t result = Wire.endTransmission();
  if (result == 0) {
    imuMuxSeen = true;
    imuMuxSelectOk++;
    return true;
  }
  imuMuxSelectFail++;
  return false;
}

void readIMU() {
  for (int i = 0; i < IMU_COUNT; ++i) {
    ImuDiagnostic &diag = imuDiagnostics[i];
    diag.lastProbeMs = millis();

    if (!selectImuMuxChannel(i)) {
      diag.readFail++;
      continue;
    }

    Wire.beginTransmission(IMU_DEVICE_ADDRESS);
    Wire.write(0x3B);
    if (Wire.endTransmission(false) != 0) {
      diag.readFail++;
      continue;
    }

    const uint8_t received = Wire.requestFrom((int)IMU_DEVICE_ADDRESS, 14, true);
    if (received < 14) {
      diag.readFail++;
      continue;
    }

    diag.ax = (Wire.read() << 8) | Wire.read();
    diag.ay = (Wire.read() << 8) | Wire.read();
    diag.az = (Wire.read() << 8) | Wire.read();
    Wire.read(); Wire.read(); // temperature
    diag.gx = (Wire.read() << 8) | Wire.read();
    diag.gy = (Wire.read() << 8) | Wire.read();
    diag.gz = (Wire.read() << 8) | Wire.read();
    diag.readOk++;
    diag.everSeen = true;
    diag.lastSeenMs = millis();

    dbPrintf("IMU %d CH%d Acc:%d,%d,%d Gyro:%d,%d,%d\n",
             i, i, diag.ax, diag.ay, diag.az, diag.gx, diag.gy, diag.gz);
  }
}

void printMACAddress() {
  uint8_t mac[6];
  WiFi.macAddress(mac);
  dbPrintf("MAC Address: %02X:%02X:%02X:%02X:%02X:%02X\n",
                mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}

void printStatus() {
  if (isHead) {
    dbPrintf("Device role: HEAD_NECK | Mode:%s | play:%d | SSID:%s\n",
             configMode ? "config" : "normal", playMode, desiredPortalSSID().c_str());
    printNeckStatus();
    return;
  }
  dbPrintf("Operating: %s | Mode: %s | role: %s | play:%d | raw:%d | maxTorque:%.2f A\n",
                operatingModeName().c_str(),
                configMode ? "config" : "normal",
                selectedRoleName().c_str(),
                playMode, rawMode, maxTorqueLimit);
  if (!isCenter) {
    dbPrintf("Angles: outer=%.1f inner=%.1f hipPitch=%.1f knee=%.1f hipRoll=%.1f\n",
                  normalizedOuter, normalizedInner, normalizedHip, normalizedKnee, normalizedButt);
    if (operatingMode == OPERATING_HYPERSPAWN_ROUTE) printHyperspawnStatus();
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

void changeDeviceRole(const String &side) {
  const String previousRole = selectedRoleName();

  if (configProvisioned && (!isCenter || isHead)) {
    requestStop(3);
    delay(40);
  }

  clearAllTorqueSetpoints();

  if (side == "left") {
    isLeft = true; isCenter = false; isHead = false;
  } else if (side == "right") {
    isLeft = false; isCenter = false; isHead = false;
  } else if (side == "center") {
    isCenter = true; isHead = false; operatingMode = OPERATING_STANDALONE;
  } else if (side == "head" || side == "neck" || side == "head_neck") {
    isHead = true; isCenter = false; operatingMode = OPERATING_STANDALONE;
  } else {
    dbPrintln("Invalid role. Use left, right, center, or head.");
    return;
  }

  configProvisioned = true;
  saveConfig();

  const String newRole = selectedRoleName();
  dbPrintln("Device role set to: " + newRole);

  if (newRole != roleAtBoot || previousRole == "unconfigured") {
    rebootRequired = true;
    runtimeControlReady = false;
    runtimeImuReady = false;
    runtimeNeckReady = false;
    playMode = false;
    dbPrintln("Runtime role changed. Control output is disabled until reboot instantiates the correct task topology.");
  }

  // The SoftAP identity always reflects the selected/persisted role when
  // portal support is compiled in.
  if (DROPBEAR_ENABLE_WIFI_PORTAL) requestPortalRestart();
}

void changeOperatingMode(const String &modeName) {
  if (isHead || isCenter) {
    dbPrintln("Operating mode selection applies only to LEFT_LEG and RIGHT_LEG roles.");
    return;
  }
  OperatingMode requested = operatingMode;
  if (modeName == "standalone" || modeName == "original" || modeName == "local") {
    requested = OPERATING_STANDALONE;
  } else if (modeName == "hyperspawn" || modeName == "ros2" || modeName == "route") {
    requested = OPERATING_HYPERSPAWN_ROUTE;
  } else {
    dbPrintln("Usage: mode standalone | hyperspawn");
    return;
  }

  if (requested == operatingMode) {
    dbPrintf("Operating mode already %s.\n", operatingModeName().c_str());
    return;
  }

  if (runtimeControlReady && !isCenter && !isHead) {
    requestStop(3);
    delay(40);
  }
  clearAllTorqueSetpoints();
  hyperspawnControlMode = HS_CONTROL_NONE;
  operatingMode = requested;
  saveConfig();
  rebootRequired = true;
  dbPrintf("Operating mode saved as %s. Reboot required before control authority changes.\n",
           operatingModeName().c_str());
}

void printHyperspawnStatus() {
  dbPrintf("Hyperspawn route: mode=%s node=0x%02X limb=%u control=%s play=%s timeout=%lums legacy=%s autoArm=%s scale=%.4f lastCmdAge=%s\n",
           operatingModeName().c_str(), hyperspawnNodeId(), hyperspawnLimbId(),
           hyperspawnControlMode == HS_CONTROL_POSITION ? "position" :
           (hyperspawnControlMode == HS_CONTROL_TORQUE ? "torque" : "none"),
           playMode ? "on" : "off", (unsigned long)hyperspawnCommandTimeoutMs,
           hyperspawnLegacyBroadcast ? "on" : "off",
           hyperspawnAutoArm ? "on" : "off", hyperspawnPositionUnitsPerDegree,
           hyperspawnLastCommandMs ? String(millis() - hyperspawnLastCommandMs).c_str() : "never");
  dbPrintf("  RX=%lu targeted=%lu legacy=%lu rejected=%lu completed=%lu fragmentTimeouts=%lu stateTX=%lu heartbeatTX=%lu watchdogTrips=%lu fault=%u\n",
           (unsigned long)hyperspawnRxFrames, (unsigned long)hyperspawnRxTargetedFrames,
           (unsigned long)hyperspawnRxLegacyFrames, (unsigned long)hyperspawnRxRejectedFrames,
           (unsigned long)hyperspawnCompletedCommands, (unsigned long)hyperspawnFragmentTimeouts,
           (unsigned long)hyperspawnStateFrames, (unsigned long)hyperspawnHeartbeatFrames,
           (unsigned long)hyperspawnWatchdogTrips, hyperspawnFaultCode);
}

void processHyperspawnSerialCommand(const String &command) {
  if (command == "hyperspawn" || command == "hyperspawn status") {
    printHyperspawnStatus();
    return;
  }
  if (command == "hyperspawn legacy on") {
    hyperspawnLegacyBroadcast = true;
    saveConfig();
    dbPrintln("Hyperspawn legacy brain-broadcast compatibility enabled. WARNING: not limb-targeted on a shared bus.");
    return;
  }
  if (command == "hyperspawn legacy off") {
    hyperspawnLegacyBroadcast = false;
    saveConfig();
    dbPrintln("Hyperspawn legacy brain-broadcast compatibility disabled; targeted route remains active.");
    return;
  }
  if (command == "hyperspawn autoarm on") {
    hyperspawnAutoArm = true;
    saveConfig();
    dbPrintln("Hyperspawn auto-arm on valid command enabled.");
    return;
  }
  if (command == "hyperspawn autoarm off") {
    hyperspawnAutoArm = false;
    saveConfig();
    dbPrintln("Hyperspawn auto-arm disabled; use 'play' to arm output.");
    return;
  }
  if (command.startsWith("hyperspawn timeout ")) {
    const uint32_t v = (uint32_t)command.substring(20).toInt();
    if (v < 50 || v > 10000) dbPrintln("Timeout must be 50..10000 ms.");
    else {
      hyperspawnCommandTimeoutMs = v;
      saveConfig();
      dbPrintf("Hyperspawn command timeout set to %lu ms.\n", (unsigned long)v);
    }
    return;
  }
  if (command.startsWith("hyperspawn scale ")) {
    const float v = command.substring(17).toFloat();
    if (v <= 0.0001f || v > 1000.0f) dbPrintln("Position scale must be >0 and <=1000 wire-units/degree.");
    else {
      hyperspawnPositionUnitsPerDegree = v;
      saveConfig();
      dbPrintf("Hyperspawn position scale set to %.6f wire-units/degree.\n", v);
    }
    return;
  }
  dbPrintln("Hyperspawn commands: status | legacy on/off | autoarm on/off | timeout <ms> | scale <units_per_degree>");
}

void handleConfigurationCommand(String command) {
  command.trim();
  if (command == "exit") exitConfigurationMode();
  else if (command == "left" || command == "right" || command == "center" || command == "head" || command == "neck") changeDeviceRole(command);
  else if (command.startsWith("role ")) { String r=command.substring(5); r.trim(); changeDeviceRole(r); }
  else if (command == "role") dbPrintln("Device role: " + selectedRoleName());
  else if (command == "mode") dbPrintf("Operating mode: %s (boot=%s)%s\n", operatingModeName().c_str(), operatingModeAtBoot.c_str(), rebootRequired ? " REBOOT_REQUIRED" : "");
  else if (command.startsWith("mode ")) { String m = command.substring(5); m.trim(); changeOperatingMode(m); }
  else if (command.startsWith("hyperspawn")) processHyperspawnSerialCommand(command);
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
  if (operatingMode == OPERATING_HYPERSPAWN_ROUTE) {
    dbPrintln("Manual motion command rejected: HyperSpawn/ROS2 route owns actuator authority. Use STOP or switch mode and reboot.");
    return;
  }

  if (!runtimeControlReady || isCenter || isHead) {
    dbPrintln("Impedance command rejected: leg actuator-control runtime is not active.");
    return;
  }

  String params = command.substring(10);
  params.trim();

  // Canonical DB1 grammar derives the leg from the validated destination:
  //   <DB1:LEFTLEG> impedance knee 1 180 0
  // Legacy payloads with an explicit side remain accepted only inside an
  // already-addressed command for migration:
  //   <DB1:LEFTLEG> impedance left knee 1 180 0
  String tokens[5];
  int tokenCount = 0;
  int pos = 0;
  while (tokenCount < 5 && pos < (int)params.length()) {
    while (pos < (int)params.length() && params.charAt(pos) == ' ') pos++;
    if (pos >= (int)params.length()) break;
    int next = params.indexOf(' ', pos);
    if (next < 0) next = params.length();
    tokens[tokenCount++] = params.substring(pos, next);
    pos = next + 1;
  }

  String legSide = isLeft ? "left" : "right";
  String appendage;
  bool enable = false;
  float desiredPosition = 0.0f;
  float desiredVelocity = 0.0f;

  if (tokenCount == 4) {
    appendage = tokens[0];
    enable = tokens[1].toInt() != 0;
    desiredPosition = tokens[2].toFloat();
    desiredVelocity = tokens[3].toFloat();
  } else if (tokenCount == 5) {
    String suppliedSide = tokens[0]; suppliedSide.toLowerCase();
    if (suppliedSide != legSide) {
      dbPrintf("ERR|PAYLOAD_SIDE_MISMATCH|target=%s|payload_side=%s\n",
               currentCommandAddress().c_str(), suppliedSide.c_str());
      return;
    }
    appendage = tokens[1];
    enable = tokens[2].toInt() != 0;
    desiredPosition = tokens[3].toFloat();
    desiredVelocity = tokens[4].toFloat();
  } else {
    dbPrintln("Usage: <DB1:LEFTLEG|RIGHTLEG> impedance <appendage> <0|1> <desiredPosition> <desiredVelocity>");
    return;
  }

  ImpedanceControl *control = nullptr;
  bool *enabledFlag = nullptr;
  int index = -1;

  if (isLeft) {
    if (appendage == "outer_calf") { control = &outerCalfControlLeft; enabledFlag = &impedanceEnabledLeftOuterCalf; index = LEFT_OUTER_CALF; }
    else if (appendage == "inner_calf") { control = &innerCalfControlLeft; enabledFlag = &impedanceEnabledLeftInnerCalf; index = LEFT_INNER_CALF; }
    else if (appendage == "knee") { control = &kneeControlLeft; enabledFlag = &impedanceEnabledLeftKnee; index = LEFT_KNEE; }
    else if (appendage == "hip_pitch") { control = &hipPitchControlLeft; enabledFlag = &impedanceEnabledLeftHipPitch; index = LEFT_HIP_PITCH; }
    else if (appendage == "hip_roll") { control = &hipRollControlLeft; enabledFlag = &impedanceEnabledLeftHipRoll; index = LEFT_HIP_ROLL; }
  } else {
    if (appendage == "outer_calf") { control = &outerCalfControlRight; enabledFlag = &impedanceEnabledRightOuterCalf; index = RIGHT_OUTER_CALF; }
    else if (appendage == "inner_calf") { control = &innerCalfControlRight; enabledFlag = &impedanceEnabledRightInnerCalf; index = RIGHT_INNER_CALF; }
    else if (appendage == "knee") { control = &kneeControlRight; enabledFlag = &impedanceEnabledRightKnee; index = RIGHT_KNEE; }
    else if (appendage == "hip_pitch") { control = &hipPitchControlRight; enabledFlag = &impedanceEnabledRightHipPitch; index = RIGHT_HIP_PITCH; }
    else if (appendage == "hip_roll") { control = &hipRollControlRight; enabledFlag = &impedanceEnabledRightHipRoll; index = RIGHT_HIP_ROLL; }
  }

  if (!control || !enabledFlag || index < 0) {
    dbPrintln("Invalid appendage. Hip yaw remains direct-torque only because no external hip-yaw AS5600 is mapped.");
    return;
  }

  *enabledFlag = enable;
  impedanceTorqueValues[index] = 0;
  control->reset();
  if (enable) {
    control->setDesiredPosition(desiredPosition);
    control->setDesiredVelocity(desiredVelocity);
  }

  dbPrintf("OK|target=%s|command=impedance|joint=%s|enabled=%d|position=%.2f|velocity=%.2f\n",
           currentCommandAddress().c_str(), appendage.c_str(), enable, desiredPosition, desiredVelocity);
}

void processTorqueCommand(const String &command) {
  if (operatingMode == OPERATING_HYPERSPAWN_ROUTE) {
    dbPrintln("Manual motion command rejected: HyperSpawn/ROS2 route owns actuator authority. Use STOP or switch mode and reboot.");
    return;
  }

  if (!runtimeControlReady || isCenter || isHead) {
    dbPrintln("Torque command rejected: leg actuator-control runtime is not active.");
    return;
  }

  String params = command.substring(7);
  params.trim();
  String first, second, third;
  int p1 = params.indexOf(' ');
  if (p1 < 0) {
    dbPrintln("Usage: <DB1:LEFTLEG|RIGHTLEG> torque <appendage> <torqueValue>");
    return;
  }
  first = params.substring(0, p1);
  String rest = params.substring(p1 + 1); rest.trim();
  int p2 = rest.indexOf(' ');

  String appendage;
  String valueText;
  String expectedSide = isLeft ? "left" : "right";
  String firstLower = first; firstLower.toLowerCase();
  if ((firstLower == "left" || firstLower == "right") && p2 >= 0) {
    if (firstLower != expectedSide) {
      dbPrintf("ERR|PAYLOAD_SIDE_MISMATCH|target=%s|payload_side=%s\n",
               currentCommandAddress().c_str(), firstLower.c_str());
      return;
    }
    appendage = rest.substring(0, p2);
    valueText = rest.substring(p2 + 1);
  } else {
    appendage = first;
    valueText = rest;
  }
  appendage.trim(); valueText.trim();
  if (!appendage.length() || !valueText.length()) {
    dbPrintln("Usage: <DB1:LEFTLEG|RIGHTLEG> torque <appendage> <torqueValue>");
    return;
  }

  const int16_t torqueValue = clampTorqueCommand(valueText.toFloat());
  int index = -1;
  if (isLeft) {
    if (appendage == "outer_calf") index = LEFT_OUTER_CALF;
    else if (appendage == "inner_calf") index = LEFT_INNER_CALF;
    else if (appendage == "knee") index = LEFT_KNEE;
    else if (appendage == "hip_pitch") index = LEFT_HIP_PITCH;
    else if (appendage == "hip_yaw") index = LEFT_HIP_YAW;
    else if (appendage == "hip_roll") index = LEFT_HIP_ROLL;
  } else {
    if (appendage == "outer_calf") index = RIGHT_OUTER_CALF;
    else if (appendage == "inner_calf") index = RIGHT_INNER_CALF;
    else if (appendage == "knee") index = RIGHT_KNEE;
    else if (appendage == "hip_pitch") index = RIGHT_HIP_PITCH;
    else if (appendage == "hip_yaw") index = RIGHT_HIP_YAW;
    else if (appendage == "hip_roll") index = RIGHT_HIP_ROLL;
  }

  if (index < 0 || !actuatorBelongsToSelectedLeg(index)) {
    dbPrintln("Invalid torque appendage for this addressed leg.");
    return;
  }

  torqueValues[index] = torqueValue;
  dbPrintf("OK|target=%s|command=torque|joint=%s|value=%d\n",
           currentCommandAddress().c_str(), appendage.c_str(), torqueValue);
}

void processRoutedCommand(String command, const char *source) {
  command.trim();
  if (!command.length()) return;

  CommandTarget target = TARGET_INVALID;
  String payload;
  bool usedLegacy = false;
  if (!parseDB1Envelope(command, target, payload, usedLegacy)) return;

  const CommandTarget expected = currentCommandTarget();

  if (target == TARGET_LEFTARM || target == TARGET_RIGHTARM) {
    routedUnsupportedTarget++;
    routedCommandsRejected++;
    dbPrintf("ERR|ROLE_UNSUPPORTED|target=%s|firmware_role=%s\n",
             commandTargetName(target).c_str(), commandTargetName(expected).c_str());
    return;
  }

  if (target == TARGET_ALL) {
    if (!isAllowedBroadcastPayload(payload)) {
      routedCommandsRejected++;
      dbPrintf("ERR|BROADCAST_FORBIDDEN|payload=%s|allowed=stop\n", payload.c_str());
      return;
    }
    routedBroadcastStop++;
  } else if (target != expected) {
    routedTargetMismatch++;
    routedCommandsRejected++;
    dbPrintf("ERR|TARGET_MISMATCH|expected=%s|received=%s\n",
             commandTargetName(expected).c_str(), commandTargetName(target).c_str());
    return;
  }

  routedCommandsAccepted++;
  lastRoutedCommandMs = millis();
  lastRoutedTarget = commandTargetName(target);
  lastRoutedSource = source ? String(source) : String("unknown");
  lastRoutedPayload = payload;
  if (lastRoutedPayload.length() > 96) lastRoutedPayload = lastRoutedPayload.substring(0, 96);

  // STOP always bypasses configuration-mode restrictions. Broadcast ALL is
  // permitted only for STOP, but an explicitly addressed STOP gets the same
  // safety priority.
  if (payload == "stop") {
    processPayloadCommand(payload, source);
    return;
  }

  if (configMode) handleConfigurationCommand(payload);
  else processPayloadCommand(payload, source);
}

void processPayloadCommand(String command, const char *source) {

  command.trim();
  if (command.length() == 0) return;

  if (serialMutex && xSemaphoreTake(serialMutex, portMAX_DELAY) != pdTRUE) return;

  // Preserve Dropbear-Neck-Assembly command compatibility while keeping shared
  // role/config/safety commands in the universal parser.
  if (isHead && (command.startsWith("neck ") || command.equalsIgnoreCase("HEALTH") ||
                 command.equalsIgnoreCase("HOME") || command.equalsIgnoreCase("HOME_BRUTE") ||
                 command.equalsIgnoreCase("HOME_SOFT") || command == "STATUS" || command.startsWith("Q") ||
                 command.indexOf(':') >= 0 || strchr("XYZHSARP", command.charAt(0)) != nullptr)) {
    bool handled=processNeckCommand(command, source ? source : "serial");
    if (handled) { xSemaphoreGive(serialMutex); return; }
  }

  if (command == "role") {
    dbPrintln("Device role: " + selectedRoleName());
  } else if (command.startsWith("role ")) {
    String r=command.substring(5); r.trim(); xSemaphoreGive(serialMutex); changeDeviceRole(r); return;
  } else if (command == "mode") {
    dbPrintf("Operating mode: %s (boot=%s)%s\n", operatingModeName().c_str(), operatingModeAtBoot.c_str(),
             rebootRequired ? " REBOOT_REQUIRED" : "");
  } else if (command.startsWith("mode ")) {
    String m = command.substring(5);
    m.trim();
    xSemaphoreGive(serialMutex);
    changeOperatingMode(m);
    return;
  } else if (command == "ros2" || command == "hyperspawn on") {
    xSemaphoreGive(serialMutex);
    changeOperatingMode("hyperspawn");
    return;
  } else if (command == "hyperspawn off") {
    xSemaphoreGive(serialMutex);
    changeOperatingMode("standalone");
    return;
  } else if (command.startsWith("hyperspawn")) {
    processHyperspawnSerialCommand(command);
  } else if (command == "config") {
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
    if (isHead) dbPrintln("Current hardware role: head (chirality not applicable).");
    else dbPrintln("Current chirality: " + String(isCenter ? "center" : (isLeft ? "left" : "right")));
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
    if (isHead) {
      if (!runtimeNeckReady) dbPrintln("Play rejected: HEAD_NECK runtime is not active. Reboot after role configuration.");
      else { neckMotionEnabled=true; playMode=true; dbPrintln("HEAD_NECK motion armed."); }
    } else if (isCenter) {
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
    dbPrintln(isHead ? "HEAD_NECK motion stopped." : "Play mode disabled. Stop burst queued for this leg.");
  } else if (command == "zero") {
    if (isHead) neckZeroAll();
    else { clearAllTorqueSetpoints(); dbPrintln("All direct and impedance torque setpoints cleared."); }
  } else if (command == "left" || command == "right" || command == "center" || command == "head" || command == "neck") {
    xSemaphoreGive(serialMutex);
    changeDeviceRole(command);
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
  dbPrintln("DB1 addressed command protocol (mandatory by default):");
  dbPrintf("  This controller address: %s\n", currentCommandAddress().c_str());
  dbPrintln("  Format: <DB1:TARGET> <payload>");
  dbPrintln("  Examples:");
  dbPrintln("    <DB1:LEFTLEG> torque knee 25");
  dbPrintln("    <DB1:RIGHTLEG> impedance knee 1 180 0");
  dbPrintln("    <DB1:HEADNECK> X10,Y-5,H30,R5,P-3");
  dbPrintln("    <DB1:ALL> stop   (the only permitted broadcast command)");
  dbPrintln("    <DB1:SETUP> role left   (fresh/unconfigured board)");
  dbPrintln("  Unaddressed commands are rejected unless LegacyUnaddressedCommands is explicitly enabled.");
  dbPrintln("Available payloads after the DB1 header:");
  dbPrintln("  role | role left | role right | role center | role head");
  dbPrintln("      Select boot-time hardware personality. HEAD_NECK uses an entirely separate STEP/DIR pin graph.");
  dbPrintln("  mode");
  dbPrintln("      Show active operating structure.");
  dbPrintln("  mode standalone | mode hyperspawn");
  dbPrintln("      Persist command-authority path; reboot required. Default is standalone/original.");
  dbPrintln("  hyperspawn status");
  dbPrintln("  hyperspawn legacy on | off");
  dbPrintln("  hyperspawn autoarm on | off");
  dbPrintln("  hyperspawn timeout <ms>");
  dbPrintln("  hyperspawn scale <wire_units_per_degree>");
  dbPrintln("  config");
  dbPrintln("      Enter configuration mode and stop this leg.");
  dbPrintln("  resetOffsets");
  dbPrintln("      Reset left and right stored offsets to zero.");
  dbPrintln("  raw on | raw off");
  dbPrintln("      Bypass or apply joint offsets.");
  dbPrintln("  direction <joint> <+|->");
  dbPrintln("      Example: direction right_outer_calf +");
  dbPrintln("  impedance <appendage> <0|1> <desiredPosition> <desiredVelocity>");
  dbPrintln("      Example: <DB1:LEFTLEG> impedance knee 1 180 0");
  dbPrintln("      Supported: outer_calf inner_calf knee hip_pitch hip_roll");
  dbPrintln("  constrain <joint> <minAngle> <maxAngle>");
  dbPrintln("      Example: constrain right_knee 0 180");
  dbPrintln("  torque <appendage> <torqueValue>");
  dbPrintln("      Example: <DB1:LEFTLEG> torque knee 100");
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
  dbPrintln("  HEAD_NECK commands (only when role=head):");
  dbPrintln("      1:30,2:45,...          direct actuator targets in mm");
  dbPrintln("      X10,Y-5,Z15,H30,S1.5,A2,R10,P-5");
  dbPrintln("      Q:w,x,y,z[,Hn][,Sn][,An]");
  dbPrintln("      HOME | HOME_BRUTE | HOME_SOFT | HEALTH");
  dbPrintln("      neck status | neck stop | neck zero");
  dbPrintln("      neck speed <Hz> | neck accel <steps/s^2>");
  dbPrintln("      neck bluetooth on|off | neck autohome on|off");
  dbPrintln("      neck limits <motor 1..6> <min_mm> <max_mm>");
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
  portalHttpRequests++;
  portalRedirects++;
  server.sendHeader("Location", String("http://") + WiFi.softAPIP().toString() + "/");
  sendNoCacheHeaders();
  server.send(302, "text/plain", "Redirecting to Dropbear controller...");
}

String readSPIFFSTextFile(const String &path, size_t maxBytes = 32768) {
  if (!path.startsWith("/") || path.indexOf("..") >= 0) return "";
  if (spiffsMutex && xSemaphoreTake(spiffsMutex, pdMS_TO_TICKS(500)) != pdTRUE) return "";

  File file = SPIFFS.open(path, FILE_READ);
  if (!file) {
    spiffsErrors++;
    if (spiffsMutex) xSemaphoreGive(spiffsMutex);
    return "";
  }
  spiffsReadOps++;
  lastSpiffsReadMs = millis();

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
  out.reserve(6500);
  out += "{";
  out += "\"role\":\"" + selectedRoleName() + "\",";
  out += "\"command_address\":\"" + currentCommandAddress() + "\",";
  out += "\"command_protocol\":\"DB1\",";
  out += "\"control_stack\":\"" + String(isHead ? "neck_stepper" : (isCenter ? "imu_center" : (operatingMode == OPERATING_HYPERSPAWN_ROUTE ? "hyperspawn" : "standalone_leg"))) + "\",";
  out += "\"operating_mode\":\"" + operatingModeName() + "\",";
  out += "\"portal_enabled\":" + String(DROPBEAR_ENABLE_WIFI_PORTAL ? "true" : "false") + ",";
  out += "\"sensor_pinout\":\"as5600_pwm_one_wire_original_gpio\",";
  out += "\"sensor_pins\":[" + String(PIN_OUTER_CALF) + "," + String(PIN_INNER_CALF) + "," +
         String(PIN_HIP_PITCH) + "," + String(PIN_KNEE) + "," + String(PIN_HIP_ROLL) + "],";
  out += "\"ssid\":\"" + jsonEscape(portalSSID) + "\",";
  out += "\"ip\":\"" + WiFi.softAPIP().toString() + "\",";
  out += "\"clients\":" + String(WiFi.softAPgetStationNum()) + ",";
  out += "\"configured\":" + String(configProvisioned ? "true" : "false") + ",";
  out += "\"control_ready\":" + String(runtimeControlReady ? "true" : "false") + ",";
  out += "\"imu_ready\":" + String(runtimeImuReady ? "true" : "false") + ",";
  out += "\"neck_ready\":" + String(runtimeNeckReady ? "true" : "false") + ",";
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
  out += "],";

  out += "\"hyperspawn\":{";
  out += "\"node_id\":" + String(hyperspawnNodeId()) + ",";
  out += "\"limb_id\":" + String(hyperspawnLimbId()) + ",";
  out += "\"control_mode\":\"" + String(hyperspawnControlMode == HS_CONTROL_POSITION ? "position" : (hyperspawnControlMode == HS_CONTROL_TORQUE ? "torque" : "none")) + "\",";
  out += "\"last_command_age_ms\":" + String(hyperspawnLastCommandMs ? millis() - hyperspawnLastCommandMs : UINT32_MAX) + ",";
  out += "\"watchdog_tripped\":" + String(hyperspawnWatchdogTripped ? "true" : "false") + ",";
  out += "\"fault_code\":" + String(hyperspawnFaultCode) + ",";
  out += "\"rx_frames\":" + String(hyperspawnRxFrames) + ",";
  out += "\"completed_commands\":" + String(hyperspawnCompletedCommands) + ",";
  out += "\"fragment_timeouts\":" + String(hyperspawnFragmentTimeouts) + ",";
  out += "\"state_frames\":" + String(hyperspawnStateFrames) + ",";
  out += "\"heartbeat_frames\":" + String(hyperspawnHeartbeatFrames);
  out += "},";

  out += "\"neck\":{";
  out += "\"active\":" + String(isHead && runtimeNeckReady ? "true" : "false") + ",";
  out += "\"motion_enabled\":" + String(neckMotionEnabled ? "true" : "false") + ",";
  out += "\"software_homed\":" + String(neckSoftwareHomed ? "true" : "false") + ",";
  out += "\"home_state\":" + String((unsigned int)neckHomeState) + ",";
  out += "\"bluetooth_enabled\":" + String(neckBluetoothEnabled ? "true" : "false") + ",";
  out += "\"bluetooth_started\":" + String(neckBluetoothStarted ? "true" : "false") + ",";
  out += "\"speed_hz\":" + String(neckSpeedHz) + ",\"accel\":" + String(neckAcceleration) + ",";
  out += "\"steps_per_mm\":" + String(neckStepsPerMm,4) + ",";
  out += "\"pose\":{";
  out += "\"x\":"+String(neckLastPose.x)+",\"y\":"+String(neckLastPose.y)+",\"z\":"+String(neckLastPose.z)+",\"height\":"+String(neckLastPose.height)+",\"roll\":"+String(neckLastPose.roll)+",\"pitch\":"+String(neckLastPose.pitch)+",\"speed\":"+String(neckLastPose.speedMultiplier,2)+",\"accel\":"+String(neckLastPose.accelMultiplier,2);
  out += "},\"motors\":[";
  for(uint8_t i=0;i<NECK_MOTOR_COUNT;++i){ if(i) out+=","; FastAccelStepper *st=neckSteppers[i]; int32_t cur=st?st->getCurrentPosition():0; bool running=st?st->isRunning():false; out+="{\"index\":"+String(i+1)+",\"step_gpio\":"+String(NECK_STEP_PINS[i])+",\"dir_gpio\":"+String(NECK_DIR_PINS[i])+",\"current_steps\":"+String(cur)+",\"target_steps\":"+String(neckTargetSteps[i])+",\"current_mm\":"+String(neckStepsToMm(cur),3)+",\"target_mm\":"+String(neckStepsToMm(neckTargetSteps[i]),3)+",\"moving\":"+String(running?"true":"false")+",\"min_mm\":"+String(neckMinMm[i],2)+",\"max_mm\":"+String(neckMaxMm[i],2)+"}"; }
  out += "]}";
  out += "}";
  return out;
}

String buildConfigJson() {
  String out;
  out.reserve(7500);
  out += "{";
  out += "\"role\":\"" + selectedRoleName() + "\",";
  out += "\"operating_mode\":\"" + operatingModeName() + "\",";
  out += "\"legacy_unaddressed_commands\":" + String(legacyUnaddressedCommands ? "true" : "false") + ",";
  out += "\"hyperspawn_timeout_ms\":" + String(hyperspawnCommandTimeoutMs) + ",";
  out += "\"hyperspawn_legacy_broadcast\":" + String(hyperspawnLegacyBroadcast ? "true" : "false") + ",";
  out += "\"hyperspawn_auto_arm\":" + String(hyperspawnAutoArm ? "true" : "false") + ",";
  out += "\"hyperspawn_position_units_per_degree\":" + String(hyperspawnPositionUnitsPerDegree, 6) + ",";
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
  out += "\"neck\":{";
  out += "\"speed_hz\":"+String(neckSpeedHz)+",\"accel\":"+String(neckAcceleration)+",\"steps_per_mm\":"+String(neckStepsPerMm,6)+",";
  out += "\"bluetooth_enabled\":"+String(neckBluetoothEnabled?"true":"false")+",\"auto_home\":"+String(neckAutoHome?"true":"false")+",\"use_enable_pin\":"+String(neckUseEnablePin?"true":"false")+",";
  out += "\"min_mm\":["; for(int i=0;i<NECK_MOTOR_COUNT;++i){if(i)out+=",";out+=String(neckMinMm[i],3);} out += "],\"max_mm\":["; for(int i=0;i<NECK_MOTOR_COUNT;++i){if(i)out+=",";out+=String(neckMaxMm[i],3);} out += "]},";

  out += "\"raw_config\":\"" + jsonEscape(readSPIFFSTextFile("/config.txt")) + "\"";
  out += "}";
  return out;
}


const JointConstraints &selectedConstraintForSensor(int index) {
  if (isLeft) {
    switch (index) {
      case 0: return outerCalfConstraintsLeft;
      case 1: return innerCalfConstraintsLeft;
      case 2: return hipPitchConstraintsLeft;
      case 3: return kneeConstraintsLeft;
      default: return hipRollConstraintsLeft;
    }
  }
  switch (index) {
    case 0: return outerCalfConstraintsRight;
    case 1: return innerCalfConstraintsRight;
    case 2: return hipPitchConstraintsRight;
    case 3: return kneeConstraintsRight;
    default: return hipRollConstraintsRight;
  }
}

const char *canHealthStatus() {
  if (!configProvisioned || isCenter || isHead) return "inactive";
  if (!canInitialized || !runtimeControlReady) return "fault";
  if (strcmp(taskHealthStatus(true, lastCanTaskMs, 40, 150), "fault") == 0) return "fault";
  if (canConsecutiveFailures >= 3) return "fault";
  if (canConsecutiveFailures > 0 || diagnosticAgeMs(lastCanFailureMs) < 5000) return "warn";
  return "ok";
}

const char *overallDiagnosticStatus() {
  if (!configProvisioned) return "warn";
  if (rebootRequired) return "warn";
  if (DROPBEAR_ENABLE_WIFI_PORTAL && !portalOnline) return "fault";
  if (spiffsMounted == false) return "fault";

  if (isHead) {
    if (!runtimeNeckReady) return "fault";
    if (strcmp(taskHealthStatus(true, lastNeckServiceMs, 100, 500), "fault") == 0) return "fault";
    uint8_t connected=0; for(uint8_t i=0;i<NECK_MOTOR_COUNT;++i) if(neckSteppers[i]) connected++;
    if(connected<NECK_MOTOR_COUNT) return "fault";
    return neckSoftwareHomed ? "ok" : "warn";
  }

  if (isCenter) {
    if (!runtimeImuReady) return "fault";
    if (strcmp(taskHealthStatus(true, lastImuTaskMs, 100, 300), "fault") == 0) return "fault";
    bool anySeen = false;
    for (int i = 0; i < IMU_COUNT; ++i) anySeen = anySeen || imuDiagnostics[i].everSeen;
    return anySeen ? "ok" : "warn";
  }

  if (!runtimeControlReady || !canInitialized) return "fault";
  if (operatingMode == OPERATING_HYPERSPAWN_ROUTE && hyperspawnWatchdogTripped) return "fault";
  if (strcmp(taskHealthStatus(true, lastSensorTaskMs, 20, 100), "fault") == 0) return "fault";
  if (strcmp(taskHealthStatus(true, lastImpedanceTaskMs, 50, 150), "fault") == 0) return "fault";
  if (strcmp(taskHealthStatus(true, lastCanTaskMs, 50, 150), "fault") == 0) return "fault";
  for (int i = 0; i < 5; ++i) {
    if (strcmp(sensorHealthStatus(sensorDiagnostics[i]), "fault") == 0) return "fault";
  }
  if (strcmp(canHealthStatus(), "fault") == 0) return "fault";
  if (strcmp(canHealthStatus(), "warn") == 0) return "warn";
  return "ok";
}

String ageJsonValue(uint32_t timestamp) {
  const uint32_t age = diagnosticAgeMs(timestamp);
  if (age == UINT32_MAX) return "-1";
  return String(age);
}

String buildDiagnosticsJson() {
  static const char *sensorNames[5] = {"outer_calf", "inner_calf", "hip_pitch", "knee", "hip_roll"};
  static const int sensorPins[5] = {PIN_OUTER_CALF, PIN_INNER_CALF, PIN_HIP_PITCH, PIN_KNEE, PIN_HIP_ROLL};
  static const char *actuatorNames[ACTUATOR_COUNT] = {
    "right_outer_calf", "left_outer_calf", "right_inner_calf", "left_inner_calf",
    "right_knee", "left_knee", "right_hip_pitch", "left_hip_pitch",
    "right_hip_yaw", "left_hip_yaw", "right_hip_roll", "left_hip_roll"
  };

  String out;
  out.reserve(24000);
  const uint32_t now = millis();
  const bool legRuntime = configProvisioned && !isCenter && !isHead && runtimeControlReady;
  const uint32_t queueDepth = webCommandQueue ? (uint32_t)uxQueueMessagesWaiting(webCommandQueue) : 0;

  out += "{";
  out += "\"timestamp_ms\":" + String(now) + ",";
  out += "\"overall\":\"" + String(overallDiagnosticStatus()) + "\",";
  out += "\"role\":\"" + selectedRoleName() + "\",";
  out += "\"role_at_boot\":\"" + jsonEscape(roleAtBoot) + "\",";
  out += "\"reboot_required\":" + String(rebootRequired ? "true" : "false") + ",";

  out += "\"modules\":{";
  out += "\"controller\":{";
  out += "\"status\":\"" + String(configProvisioned ? (rebootRequired ? "warn" : "ok") : "warn") + "\",";
  out += "\"configured\":" + String(configProvisioned ? "true" : "false") + ",";
  out += "\"control_ready\":" + String(runtimeControlReady ? "true" : "false") + ",";
  out += "\"imu_ready\":" + String(runtimeImuReady ? "true" : "false") + ",";
  out += "\"neck_ready\":" + String(runtimeNeckReady ? "true" : "false") + ",";
  out += "\"play\":" + String(playMode ? "true" : "false") + ",";
  out += "\"config_mode\":" + String(configMode ? "true" : "false") + ",";
  out += "\"raw_mode\":" + String(rawMode ? "true" : "false") + ",";
  out += "\"max_torque\":" + String(maxTorqueLimit, 3) + ",";
  out += "\"uptime_ms\":" + String(now) + ",";
  out += "\"free_heap\":" + String(ESP.getFreeHeap());
  out += "},";

  out += "\"wifi\":{";
  out += "\"status\":\"" + String(!DROPBEAR_ENABLE_WIFI_PORTAL ? "inactive" : (portalOnline ? "ok" : "fault")) + "\",";
  out += "\"enabled\":" + String(DROPBEAR_ENABLE_WIFI_PORTAL ? "true" : "false") + ",";
  out += "\"online\":" + String(portalOnline ? "true" : "false") + ",";
  out += "\"ssid\":\"" + jsonEscape(portalSSID) + "\",";
  out += "\"ip\":\"" + WiFi.softAPIP().toString() + "\",";
  out += "\"clients\":" + String(WiFi.softAPgetStationNum()) + ",";
  out += "\"http_requests\":" + String(portalHttpRequests) + ",";
  out += "\"redirects\":" + String(portalRedirects);
  out += "},";

  out += "\"spiffs\":{";
  out += "\"status\":\"" + String(spiffsMounted ? "ok" : "fault") + "\",";
  out += "\"mounted\":" + String(spiffsMounted ? "true" : "false") + ",";
  out += "\"total\":" + String(SPIFFS.totalBytes()) + ",";
  out += "\"used\":" + String(SPIFFS.usedBytes()) + ",";
  out += "\"config_exists\":" + String(SPIFFS.exists("/config.txt") ? "true" : "false") + ",";
  out += "\"backup_exists\":" + String(SPIFFS.exists("/config.bak") ? "true" : "false") + ",";
  out += "\"reads\":" + String(spiffsReadOps) + ",";
  out += "\"writes\":" + String(spiffsWriteOps) + ",";
  out += "\"errors\":" + String(spiffsErrors) + ",";
  out += "\"last_read_age_ms\":" + ageJsonValue(lastSpiffsReadMs) + ",";
  out += "\"last_write_age_ms\":" + ageJsonValue(lastSpiffsWriteMs);
  out += "},";

  out += "\"spi\":{";
  out += "\"status\":\"" + String((!configProvisioned || isCenter || isHead) ? "inactive" : (spiInitialized ? "ok" : "fault")) + "\",";
  out += "\"initialized\":" + String(spiInitialized ? "true" : "false") + ",";
  out += "\"sck\":" + String(SPI_SCK_PIN) + ",\"miso\":" + String(SPI_MISO_PIN) + ",\"mosi\":" + String(SPI_MOSI_PIN) + ",\"cs\":" + String(CAN_CS_PIN);
  out += "},";

  out += "\"encoder_io\":{";
  out += "\"status\":\"" + String((!configProvisioned || isCenter || isHead) ? "inactive" : (runtimeControlReady ? "ok" : "fault")) + "\",";
  out += "\"interface\":\"AS5600 OUT / one-wire PWM\",";
  out += "\"pins\":[" + String(PIN_OUTER_CALF) + "," + String(PIN_INNER_CALF) + "," + String(PIN_HIP_PITCH) + "," + String(PIN_KNEE) + "," + String(PIN_HIP_ROLL) + "],";
  out += "\"duty_min_percent\":2.9,\"duty_max_percent\":97.1,";
  out += "\"supported_nominal_hz\":[115,230,460,920]";
  out += "},";

  out += "\"can\":{";
  out += "\"status\":\"" + String(canHealthStatus()) + "\",";
  out += "\"initialized\":" + String(canInitialized ? "true" : "false") + ",";
  out += "\"bitrate\":1000000,\"oscillator_mhz\":8,";
  out += "\"cs_gpio\":" + String(CAN_CS_PIN) + ",\"int_gpio\":" + String((int)CAN0_INT) + ",";
  out += "\"int_level\":" + String(canInitialized ? digitalRead(CAN0_INT) : -1) + ",";
  out += "\"tx_success\":" + String(canTxSuccess) + ",";
  out += "\"rx_frames\":" + String(canRxFrames) + ",";
  out += "\"rx_errors\":" + String(canRxErrors) + ",";
  out += "\"last_rx_age_ms\":" + ageJsonValue(lastCanRxMs) + ",";
  out += "\"tx_failure\":" + String(canTxFailure) + ",";
  out += "\"consecutive_failures\":" + String(canConsecutiveFailures) + ",";
  out += "\"mutex_timeouts\":" + String(canMutexTimeouts) + ",";
  out += "\"torque_frames\":" + String(canTorqueFrames) + ",";
  out += "\"stop_frames\":" + String(canStopFrames) + ",";
  out += "\"last_result\":" + String(lastCanResult) + ",";
  out += "\"last_tx_age_ms\":" + ageJsonValue(lastCanTxMs) + ",";
  out += "\"last_failure_age_ms\":" + ageJsonValue(lastCanFailureMs) + ",";
  out += "\"feedback\":\"rx_transport_active_rmd_decoder_pending\"";
  out += "},";

  out += "\"i2c\":{";
  out += "\"status\":\"" + String(isHead ? "inactive" : (i2cInitialized ? "ok" : "fault")) + "\",";
  out += "\"initialized\":" + String(i2cInitialized ? "true" : "false") + ",";
  out += "\"sda\":" + String(I2C_SDA_PIN) + ",\"scl\":" + String(I2C_SCL_PIN) + ",";
  out += "\"mux_address\":112,";
  out += "\"mux_seen\":" + String(imuMuxSeen ? "true" : "false") + ",";
  out += "\"mux_select_ok\":" + String(imuMuxSelectOk) + ",";
  out += "\"mux_select_fail\":" + String(imuMuxSelectFail);
  out += "},";

  out += "\"commands\":{";
  out += "\"status\":\"" + String(taskHealthStatus(true, lastCommandTaskMs, 100, 500)) + "\",";
  out += "\"queue_depth\":" + String(queueDepth) + ",";
  out += "\"queue_capacity\":12,";
  out += "\"web_queued\":" + String(webCommandsQueued) + ",";
  out += "\"web_processed\":" + String(webCommandsProcessed) + ",";
  out += "\"queue_full_events\":" + String(webCommandQueueFull) + ",";
  out += "\"serial_processed\":" + String(serialCommandsProcessed) + ",";
  out += "\"protocol\":\"DB1\",";
  out += "\"address\":\"" + currentCommandAddress() + "\",";
  out += "\"legacy_unaddressed\":" + String(legacyUnaddressedCommands ? "true" : "false") + ",";
  out += "\"routed_accepted\":" + String(routedCommandsAccepted) + ",";
  out += "\"routed_rejected\":" + String(routedCommandsRejected) + ",";
  out += "\"missing_header\":" + String(routedMissingHeader) + ",";
  out += "\"target_mismatch\":" + String(routedTargetMismatch) + ",";
  out += "\"unsupported_target\":" + String(routedUnsupportedTarget) + ",";
  out += "\"broadcast_stops\":" + String(routedBroadcastStop) + ",";
  out += "\"legacy_accepted\":" + String(routedLegacyAccepted) + ",";
  out += "\"last_target\":\"" + jsonEscape(lastRoutedTarget) + "\",";
  out += "\"last_source\":\"" + jsonEscape(lastRoutedSource) + "\",";
  out += "\"last_payload\":\"" + jsonEscape(lastRoutedPayload) + "\",";
  out += "\"last_age_ms\":" + ageJsonValue(lastRoutedCommandMs);
  out += "},";

  out += "\"neck\":{";
  out += "\"status\":\""+String(!isHead?"inactive":(runtimeNeckReady?"ok":"fault"))+"\",";
  out += "\"runtime_ready\":"+String(runtimeNeckReady?"true":"false")+",";
  out += "\"motion_enabled\":"+String(neckMotionEnabled?"true":"false")+",";
  out += "\"software_homed\":"+String(neckSoftwareHomed?"true":"false")+",";
  out += "\"feedback\":\"open_loop_step_count\",";
  out += "\"speed_hz\":"+String(neckSpeedHz)+",\"accel\":"+String(neckAcceleration)+",";
  out += "\"bluetooth\":\""+String(neckBluetoothStarted?NECK_BT_NAME:(neckBluetoothEnabled?"configured_not_started":"disabled"))+"\"";
  out += "}";
  out += "},";

  out += "\"hyperspawn_route\":{";
  out += "\"status\":\"" + String(operatingMode != OPERATING_HYPERSPAWN_ROUTE ? "inactive" : (hyperspawnWatchdogTripped ? "fault" : "ok")) + "\",";
  out += "\"active\":" + String(operatingMode == OPERATING_HYPERSPAWN_ROUTE ? "true" : "false") + ",";
  out += "\"node_id\":" + String(hyperspawnNodeId()) + ",";
  out += "\"limb_id\":" + String(hyperspawnLimbId()) + ",";
  out += "\"legacy_broadcast\":" + String(hyperspawnLegacyBroadcast ? "true" : "false") + ",";
  out += "\"timeout_ms\":" + String(hyperspawnCommandTimeoutMs) + ",";
  out += "\"position_units_per_degree\":" + String(hyperspawnPositionUnitsPerDegree, 6) + ",";
  out += "\"control_mode\":\"" + String(hyperspawnControlMode == HS_CONTROL_POSITION ? "position" : (hyperspawnControlMode == HS_CONTROL_TORQUE ? "torque" : "none")) + "\",";
  out += "\"last_command_age_ms\":" + String(hyperspawnLastCommandMs ? millis() - hyperspawnLastCommandMs : UINT32_MAX) + ",";
  out += "\"rx_frames\":" + String(hyperspawnRxFrames) + ",";
  out += "\"targeted_rx\":" + String(hyperspawnRxTargetedFrames) + ",";
  out += "\"legacy_rx\":" + String(hyperspawnRxLegacyFrames) + ",";
  out += "\"rejected_rx\":" + String(hyperspawnRxRejectedFrames) + ",";
  out += "\"state_tx\":" + String(hyperspawnStateFrames) + ",";
  out += "\"heartbeat_tx\":" + String(hyperspawnHeartbeatFrames) + ",";
  out += "\"fault_tx\":" + String(hyperspawnFaultFrames) + ",";
  out += "\"watchdog_trips\":" + String(hyperspawnWatchdogTrips) + ",";
  out += "\"completed_commands\":" + String(hyperspawnCompletedCommands) + ",";
  out += "\"fragment_timeouts\":" + String(hyperspawnFragmentTimeouts) + ",";
  out += "\"fault_code\":" + String(hyperspawnFaultCode);
  out += "},";

  out += "\"safety\":{";
  out += "\"play\":" + String(playMode ? "true" : "false") + ",";
  out += "\"stop_burst_remaining\":" + String(stopBurstRemaining) + ",";
  out += "\"calibration_override\":" + String(calibrationOverrideActive ? "true" : "false") + ",";
  out += "\"calibration_actuator_index\":" + String(calibrationActuatorIndex) + ",";
  out += "\"calibration_torque\":" + String(calibrationTorqueValue) + ",";
  out += "\"reboot_required\":" + String(rebootRequired ? "true" : "false") + ",";
  out += "\"command_watchdog\":\"" + String(operatingMode == OPERATING_HYPERSPAWN_ROUTE ? (hyperspawnWatchdogTripped ? "tripped" : "armed") : "inactive") + "\",";
  out += "\"can_feedback_monitoring\":\"rx_transport_active_decoder_pending\"";
  out += "},";

  out += "\"sensors\":[";
  for (int i = 0; i < 5; ++i) {
    if (i) out += ",";
    const SensorDiagnostic &d = sensorDiagnostics[i];
    const JointConstraints &c = selectedConstraintForSensor(i);
    out += "{";
    out += "\"name\":\"" + String(sensorNames[i]) + "\",";
    out += "\"status\":\"" + String(sensorHealthStatus(d)) + "\",";
    out += "\"gpio\":" + String(sensorPins[i]) + ",";
    out += "\"interface\":\"as5600_pwm_one_wire\",";
    out += "\"raw\":" + String(d.raw) + ",";
    out += "\"filtered_raw\":" + String(d.filtered, 2) + ",";
    out += "\"angle\":" + String(d.angle, 2) + ",";
    out += "\"signal_valid\":" + String(d.signalValid ? "true" : "false") + ",";
    out += "\"duty_percent\":" + String(d.duty * 100.0f, 3) + ",";
    out += "\"frequency_hz\":" + String(d.frequencyHz, 2) + ",";
    out += "\"period_us\":" + String(d.periodUs) + ",";
    out += "\"high_us\":" + String(d.highUs) + ",";
    out += "\"pulse_age_us\":" + String(d.pulseAgeUs) + ",";
    out += "\"pwm_frames\":" + String(d.pwmFrames) + ",";
    out += "\"min_raw_seen\":" + String(d.minRaw == 4095 && d.samples == 0 ? 0 : d.minRaw) + ",";
    out += "\"max_raw_seen\":" + String(d.maxRaw) + ",";
    out += "\"samples\":" + String(d.samples) + ",";
    out += "\"sample_age_ms\":" + ageJsonValue(d.lastSampleMs) + ",";
    out += "\"last_change_age_ms\":" + ageJsonValue(d.lastChangeMs) + ",";
    out += "\"rail_warning\":false,";
    out += "\"constraint_min\":" + String(c.minAngle) + ",\"constraint_max\":" + String(c.maxAngle);
    out += "}";
  }
  out += "],";

  out += "\"actuators\":[";
  for (int i = 0; i < ACTUATOR_COUNT; ++i) {
    if (i) out += ",";
    const bool selected = legRuntime && actuatorBelongsToSelectedLeg(i);
    const ActuatorDiagnostic &d = actuatorDiagnostics[i];
    const uint32_t age = diagnosticAgeMs(d.lastTxMs);
    const bool outputExpected = selected && (playMode || calibrationOverrideActive || stopBurstRemaining > 0);
    const char *status = "inactive";
    if (selected) {
      if (!canInitialized) status = "fault";
      else if (d.lastResult != CAN_OK && d.lastTxMs != 0 && diagnosticAgeMs(d.lastTxMs) < 5000) status = "fault";
      else if (outputExpected && (age == UINT32_MAX || age > 250)) status = "fault";
      else if (!playMode && !calibrationOverrideActive && stopBurstRemaining == 0) status = "idle";
      else status = "ok";
    }
    char idHex[12];
    snprintf(idHex, sizeof(idHex), "0x%03lX", (unsigned long)ACTUATOR_IDS[i]);
    char opHex[10];
    snprintf(opHex, sizeof(opHex), "0x%02X", (unsigned int)d.lastOpcode);
    out += "{";
    out += "\"name\":\"" + String(actuatorNames[i]) + "\",";
    out += "\"status\":\"" + String(status) + "\",";
    out += "\"selected\":" + String(selected ? "true" : "false") + ",";
    out += "\"can_id\":\"" + String(idHex) + "\",";
    out += "\"feedback\":\"can_rx_unparsed\",";
    String commandSource;
    if (operatingMode == OPERATING_HYPERSPAWN_ROUTE) {
      commandSource = hyperspawnControlMode == HS_CONTROL_POSITION ? "hyperspawn_position" :
                      (hyperspawnControlMode == HS_CONTROL_TORQUE ? "hyperspawn_torque" : "hyperspawn_idle");
    } else {
      commandSource = isImpedanceEnabled(i) ? "impedance" : "direct";
    }
    out += "\"command_source\":\"" + commandSource + "\",";
    out += "\"direct_setpoint\":" + String(torqueValues[i]) + ",";
    out += "\"impedance_setpoint\":" + String(impedanceTorqueValues[i]) + ",";
    out += "\"impedance_enabled\":" + String(isImpedanceEnabled(i) ? "true" : "false") + ",";
    out += "\"last_sent\":" + String(d.lastCommand) + ",";
    out += "\"last_opcode\":\"" + String(opHex) + "\",";
    out += "\"last_result\":" + String(d.lastResult) + ",";
    out += "\"tx_ok\":" + String(d.txOk) + ",\"tx_fail\":" + String(d.txFail) + ",";
    out += "\"tx_age_ms\":" + ageJsonValue(d.lastTxMs);
    out += "}";
  }
  out += "],";

  out += "\"neck_steppers\":[";
  for(uint8_t i=0;i<NECK_MOTOR_COUNT;++i){
    if(i) out+=","; FastAccelStepper *st=neckSteppers[i]; int32_t cur=st?st->getCurrentPosition():0; bool moving=st?st->isRunning():false;
    const char *status=!isHead?"inactive":(!st?"fault":(moving?"ok":"idle"));
    out+="{\"motor\":"+String(i+1)+",\"status\":\""+String(status)+"\",\"step_gpio\":"+String(NECK_STEP_PINS[i])+",\"dir_gpio\":"+String(NECK_DIR_PINS[i])+",\"enable_gpio\":"+String(NECK_ENABLE_PIN)+",\"current_steps\":"+String(cur)+",\"target_steps\":"+String(neckTargetSteps[i])+",\"current_mm\":"+String(neckStepsToMm(cur),3)+",\"target_mm\":"+String(neckStepsToMm(neckTargetSteps[i]),3)+",\"moving\":"+String(moving?"true":"false")+",\"min_mm\":"+String(neckMinMm[i],2)+",\"max_mm\":"+String(neckMaxMm[i],2)+",\"feedback\":\"open_loop\"}";
  }
  out += "],";

  out += "\"imus\":[";
  for (int i = 0; i < IMU_COUNT; ++i) {
    if (i) out += ",";
    const ImuDiagnostic &d = imuDiagnostics[i];
    const bool active = configProvisioned && isCenter && runtimeImuReady;
    const uint32_t seenAge = diagnosticAgeMs(d.lastSeenMs);
    const char *status = "inactive";
    if (active) {
      if (!d.everSeen) status = "warn";
      else if (seenAge > 1000) status = "fault";
      else status = "ok";
    }
    char addr[8];
    snprintf(addr, sizeof(addr), "0x%02X", IMU_DEVICE_ADDRESS);
    out += "{";
    out += "\"index\":" + String(i) + ",\"mux_channel\":" + String(i) + ",\"address\":\"" + String(addr) + "\",";
    out += "\"status\":\"" + String(status) + "\",";
    out += "\"ever_seen\":" + String(d.everSeen ? "true" : "false") + ",";
    out += "\"read_ok\":" + String(d.readOk) + ",\"read_fail\":" + String(d.readFail) + ",";
    out += "\"probe_age_ms\":" + ageJsonValue(d.lastProbeMs) + ",";
    out += "\"seen_age_ms\":" + ageJsonValue(d.lastSeenMs) + ",";
    out += "\"accel\":[" + String(d.ax) + "," + String(d.ay) + "," + String(d.az) + "],";
    out += "\"gyro\":[" + String(d.gx) + "," + String(d.gy) + "," + String(d.gz) + "]";
    out += "}";
  }
  out += "],";

  out += "\"tasks\":[";
  struct TaskDiagRow { const char *name; bool active; uint32_t expectedHz; uint32_t loops; uint32_t lastMs; TaskHandle_t handle; uint32_t warnAge; uint32_t faultAge; };
  TaskDiagRow taskRows[] = {
    {"sensors", legRuntime, 1000, sensorTaskLoops, lastSensorTaskMs, sensorTaskHandle, 20, 100},
    {"impedance", legRuntime, 100, impedanceTaskLoops, lastImpedanceTaskMs, impedanceTaskHandle, 50, 150},
    {"can_output", legRuntime, 100, canTaskLoops, lastCanTaskMs, canTaskHandle, 50, 150},
    {"command", true, 200, commandTaskLoops, lastCommandTaskMs, commandTaskHandle, 100, 500},
    {"portal", DROPBEAR_ENABLE_WIFI_PORTAL, 500, portalTaskLoops, lastPortalTaskMs, portalTaskHandle, 100, 500},
    {"can_rx", runtimeControlReady, 1000, canRxTaskLoops, lastCanRxTaskMs, canRxTaskHandle, 20, 100},
    {"hyperspawn", runtimeControlReady && operatingMode == OPERATING_HYPERSPAWN_ROUTE, HS_ROUTE_HZ, hyperspawnTaskLoops, lastHyperspawnTaskMs, hyperspawnTaskHandle, 20, 100},
    {"imu", configProvisioned && isCenter && runtimeImuReady, 100, imuTaskLoops, lastImuTaskMs, imuTaskHandle, 100, 300},
    {"neck_service", configProvisioned && isHead && runtimeNeckReady, 200, neckServiceLoops, lastNeckServiceMs, neckTaskHandle, 100, 500}
  };
  const int taskCount = sizeof(taskRows) / sizeof(taskRows[0]);
  for (int i = 0; i < taskCount; ++i) {
    if (i) out += ",";
    const TaskDiagRow &t = taskRows[i];
    out += "{";
    out += "\"name\":\"" + String(t.name) + "\",";
    out += "\"status\":\"" + String(taskHealthStatus(t.active, t.lastMs, t.warnAge, t.faultAge)) + "\",";
    out += "\"active\":" + String(t.active ? "true" : "false") + ",";
    out += "\"expected_hz\":" + String(t.expectedHz) + ",";
    out += "\"loops\":" + String(t.loops) + ",";
    out += "\"age_ms\":" + ageJsonValue(t.lastMs) + ",";
    out += "\"stack_high_water_words\":" + String(taskStackWords(t.handle));
    out += "}";
  }
  out += "]";

  out += "}";
  return out;
}

void handleApiDiagnostics() {
  portalHttpRequests++;
  sendNoCacheHeaders();
  server.send(200, "application/json", buildDiagnosticsJson());
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
    if (ok) {
      spiffsWriteOps++;
      lastSpiffsWriteMs = millis();
    } else {
      spiffsErrors++;
    }
  } else {
    spiffsErrors++;
  }

  if (spiffsMutex) xSemaphoreGive(spiffsMutex);
  return ok;
}

void handleApiState() {
  portalHttpRequests++;
  sendNoCacheHeaders();
  server.send(200, "application/json", buildStateJson());
}

void handleApiConfigGet() {
  portalHttpRequests++;
  sendNoCacheHeaders();
  server.send(200, "application/json", buildConfigJson());
}

void handleApiConfigPost() {
  portalHttpRequests++;
  sendNoCacheHeaders();
  if (!validateApiTargetArg()) return;

  const String oldRole = selectedRoleName();
  if (configProvisioned && (!isCenter || isHead)) {
    requestStop(3);
    delay(35);
  }
  clearAllTorqueSetpoints();

  if (server.hasArg("role")) {
    String role = server.arg("role");
    role.trim();
    if (role == "left") {
      isLeft = true; isCenter = false; isHead = false; configProvisioned = true;
    } else if (role == "right") {
      isLeft = false; isCenter = false; isHead = false; configProvisioned = true;
    } else if (role == "center") {
      isCenter = true; isHead = false; operatingMode = OPERATING_STANDALONE; configProvisioned = true;
    } else if (role == "head" || role == "neck") {
      isHead = true; isCenter = false; operatingMode = OPERATING_STANDALONE; configProvisioned = true;
    }
  }

  if (!isHead && !isCenter && server.hasArg("operatingMode")) {
    String mode = server.arg("operatingMode");
    mode.trim();
    OperatingMode requested = mode == "hyperspawn" ? OPERATING_HYPERSPAWN_ROUTE : OPERATING_STANDALONE;
    if (requested != operatingMode) {
      if ((runtimeControlReady && !isCenter && !isHead) || runtimeNeckReady) requestStop(3);
      clearAllTorqueSetpoints();
      hyperspawnControlMode = HS_CONTROL_NONE;
      operatingMode = requested;
      rebootRequired = true;
    }
  }
  if (server.hasArg("legacyUnaddressed")) legacyUnaddressedCommands = server.arg("legacyUnaddressed") == "1";
  else legacyUnaddressedCommands = false;

  if (server.hasArg("hsTimeout")) {
    uint32_t v = (uint32_t)server.arg("hsTimeout").toInt();
    if (v >= 50 && v <= 10000) hyperspawnCommandTimeoutMs = v;
  }
  if (server.hasArg("hsLegacy")) hyperspawnLegacyBroadcast = server.arg("hsLegacy") == "1";
  else hyperspawnLegacyBroadcast = false;
  if (server.hasArg("hsAutoArm")) hyperspawnAutoArm = server.arg("hsAutoArm") == "1";
  else hyperspawnAutoArm = false;
  if (server.hasArg("hsScale")) {
    float v = server.arg("hsScale").toFloat();
    if (v > 0.0001f && v <= 1000.0f) hyperspawnPositionUnitsPerDegree = v;
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

  if(server.hasArg("neckSpeed")){uint32_t v=(uint32_t)server.arg("neckSpeed").toInt();if(v>=1&&v<=200000)neckSpeedHz=v;}
  if(server.hasArg("neckAccel")){uint32_t v=(uint32_t)server.arg("neckAccel").toInt();if(v>=1&&v<=2000000)neckAcceleration=v;}
  if(server.hasArg("neckStepsPerMm")){float v=server.arg("neckStepsPerMm").toFloat();if(v>0.01f&&v<100000)neckStepsPerMm=v;}
  neckBluetoothEnabled=server.hasArg("neckBluetooth")&&server.arg("neckBluetooth")=="1";
  neckAutoHome=server.hasArg("neckAutoHome")&&server.arg("neckAutoHome")=="1";
  neckUseEnablePin=server.hasArg("neckUseEnable")&&server.arg("neckUseEnable")=="1";
  for(int i=0;i<NECK_MOTOR_COUNT;++i){String mn="neckMin"+String(i),mx="neckMax"+String(i);if(server.hasArg(mn))neckMinMm[i]=server.arg(mn).toFloat();if(server.hasArg(mx))neckMaxMm[i]=server.arg(mx).toFloat();if(neckMinMm[i]>neckMaxMm[i]){float t=neckMinMm[i];neckMinMm[i]=neckMaxMm[i];neckMaxMm[i]=t;}}

  saveConfig();
  String newRole = selectedRoleName();
  if (oldRole != newRole || newRole != roleAtBoot) {
    rebootRequired = true;
    runtimeControlReady = false;
    runtimeImuReady = false;
    runtimeNeckReady = false;
    playMode = false;
  }

  String out = "{\"ok\":true,\"role\":\"" + newRole + "\",\"ssid\":\"" +
               desiredPortalSSID() + "\",\"reboot_required\":" +
               String(rebootRequired ? "true" : "false") + "}";
  server.send(200, "application/json", out);
}

void handleApiConfigReload() {
  portalHttpRequests++;
  sendNoCacheHeaders();
  if (!validateApiTargetArg()) return;
  const String oldRole = selectedRoleName();
  if (runtimeControlReady || runtimeNeckReady) {
    requestStop(3);
    delay(35);
  }
  clearAllTorqueSetpoints();
  loadConfig();
  if (selectedRoleName() != oldRole || selectedRoleName() != roleAtBoot || operatingModeName() != operatingModeAtBoot) {
    rebootRequired = true;
    runtimeControlReady = false;
    runtimeImuReady = false;
    runtimeNeckReady = false;
    playMode = false;
  }
  server.send(200, "application/json", buildConfigJson());
}

void handleApiRawConfigPost() {
  portalHttpRequests++;
  sendNoCacheHeaders();
  if (!validateApiTargetArg()) return;
  if (runtimeControlReady || runtimeNeckReady) {
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
  runtimeNeckReady = false;
  playMode = false;
  server.send(200, "application/json",
              "{\"ok\":true,\"reboot_required\":true,\"ssid\":\"" +
              desiredPortalSSID() + "\"}");
}

void handleApiCommand() {
  portalHttpRequests++;
  sendNoCacheHeaders();
  if (!server.hasArg("cmd")) {
    server.send(400, "application/json", "{\"ok\":false,\"error\":\"missing_cmd\"}");
    return;
  }

  String command = server.arg("cmd");
  command.trim();

  // First ingress gate: reject malformed or wrong-target web/API commands
  // before they are even queued. processRoutedCommand() repeats the validation
  // when consuming the queue, providing defense in depth.
  if (!command.startsWith("<DB1:")) {
    server.send(400, "application/json",
                "{\"ok\":false,\"error\":\"missing_target_header\",\"expected\":\"<DB1:" + currentCommandAddress() + ">\"}");
    return;
  }
  int ingressClose = command.indexOf('>');
  if (ingressClose < 6) {
    server.send(400, "application/json", "{\"ok\":false,\"error\":\"malformed_target_header\"}");
    return;
  }
  CommandTarget ingressTarget = parseCommandTarget(command.substring(5, ingressClose));
  String ingressPayload = command.substring(ingressClose + 1); ingressPayload.trim();
  if (ingressTarget == TARGET_INVALID) {
    server.send(400, "application/json", "{\"ok\":false,\"error\":\"unknown_target\"}");
    return;
  }
  if (ingressTarget == TARGET_LEFTARM || ingressTarget == TARGET_RIGHTARM) {
    server.send(409, "application/json", "{\"ok\":false,\"error\":\"unsupported_target\"}");
    return;
  }
  if (ingressTarget == TARGET_ALL) {
    if (!isAllowedBroadcastPayload(ingressPayload)) {
      server.send(409, "application/json", "{\"ok\":false,\"error\":\"broadcast_forbidden\",\"allowed\":\"stop\"}");
      return;
    }
  } else if (ingressTarget != currentCommandTarget()) {
    server.send(409, "application/json",
                "{\"ok\":false,\"error\":\"target_mismatch\",\"expected\":\"" + currentCommandAddress() +
                "\",\"received\":\"" + commandTargetName(ingressTarget) + "\"}");
    return;
  }

  // API commands must carry the DB1 envelope explicitly. The browser UI adds
  // it automatically; direct API clients must do the same. Inspect the payload
  // here only to prevent web requests from entering blocking Serial-only flows.
  String apiPayload = command;
  if (command.startsWith("<DB1:")) {
    int close = command.indexOf('>');
    if (close > 5) { apiPayload = command.substring(close + 1); apiPayload.trim(); }
  }
  if (apiPayload == "calibrate") {
    int close = command.indexOf('>');
    if (close > 5) command = command.substring(0, close + 1) + " calibrate save";
  }

  if (!command.length() || command.length() >= WEB_COMMAND_MAX) {
    server.send(400, "application/json", "{\"ok\":false,\"error\":\"bad_length\"}");
    return;
  }

  // These two legacy interactive commands wait synchronously for USB Serial
  // follow-up input. Equivalent noninteractive web paths exist.
  if (apiPayload == "setJointConstraints" || apiPayload == "resetSPIFFS") {
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
    webCommandQueueFull++;
    server.send(503, "application/json", "{\"ok\":false,\"error\":\"queue_full\"}");
    return;
  }
  webCommandsQueued++;

  server.send(202, "application/json",
              "{\"ok\":true,\"queued\":true,\"id\":" + String(item.id) + "}");
}

void handleApiLog() {
  portalHttpRequests++;
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
  portalHttpRequests++;
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
  portalHttpRequests++;
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
  portalHttpRequests++;
  sendNoCacheHeaders();
  if (!validateApiTargetArg()) return;
  if (!configProvisioned || isCenter || isHead) {
    server.send(409, "application/json", "{\"ok\":false,\"error\":\"not_leg_controller\"}");
    return;
  }
  if (!webCommandQueue) {
    server.send(503, "application/json", "{\"ok\":false}");
    return;
  }

  WebCommand item{};
  item.id = nextWebCommandId++;
  String routed = "<DB1:" + currentCommandAddress() + "> calibrate save";
  routed.toCharArray(item.text, sizeof(item.text));
  if (xQueueSend(webCommandQueue, &item, 0) != pdTRUE) {
    webCommandQueueFull++;
    server.send(503, "application/json", "{\"ok\":false,\"error\":\"queue_full\"}");
    return;
  }
  webCommandsQueued++;
  server.send(202, "application/json", "{\"ok\":true,\"queued\":true}");
}

void handleApiReboot() {
  portalHttpRequests++;
  sendNoCacheHeaders();
  if (!validateApiTargetArg()) return;
  if (configProvisioned && (!isCenter || isHead)) requestStop(3);
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
.diagcards{display:grid;grid-template-columns:repeat(3,minmax(220px,1fr));gap:10px}.diagcard{border:1px solid var(--line);background:var(--panel2);border-radius:4px;padding:11px}.diaghead{display:flex;align-items:center;justify-content:space-between;gap:8px;margin-bottom:9px}.diagtitle{font-size:12px;font-weight:650;text-transform:uppercase;letter-spacing:.06em}.health{display:inline-block;border:1px solid var(--line);padding:3px 6px;border-radius:3px;font-size:9px;text-transform:uppercase;letter-spacing:.08em}.health.ok{color:#b5d5b6;border-color:#3c5840}.health.warn,.health.idle{color:#e6c381;border-color:#625032}.health.fault{color:#efadad;border-color:#673f3f}.health.inactive{color:#8d979e;border-color:#343b41}.kv{display:grid;grid-template-columns:1fr auto;gap:5px 12px;font-size:10px}.kv .k{color:var(--muted)}.kv .v{font-family:ui-monospace,SFMono-Regular,Menlo,Consolas,monospace;text-align:right}.diagtable{overflow:auto}.diagtable table{min-width:850px}.diagtable td.mono{font-family:ui-monospace,SFMono-Regular,Menlo,Consolas,monospace}.diag-note{font-size:10px;color:var(--muted);line-height:1.5}.overall{font-size:30px;font-weight:700;letter-spacing:.08em}.overall.ok{color:#b5d5b6}.overall.warn{color:#e6c381}.overall.fault{color:#efadad}
@media(max-width:800px){.diagcards{grid-template-columns:1fr}}
@media(max-width:800px){.span8,.span6,.span4,.span3{grid-column:span 12}.joint{grid-template-columns:1fr 80px}.joint .controls{grid-column:1/-1}.cfgline{grid-template-columns:1fr repeat(2,1fr)}.cfgline input:nth-of-type(n+3){margin-top:2px}.constraints{grid-template-columns:1fr}}
</style>
</head>
<body>
<div class="shell">
 <div class="top">
  <div><h1>DROPBEAR // <span id="roleTitle">...</span></h1><div class="sub">ESP32 low-level control · captive portal · <span id="operatingText">STANDALONE</span> <span id="ipText"></span></div></div>
  <div class="badges">
   <span class="badge" id="diagBadge">DIAG</span><span class="badge" id="addressBadge">DB1</span><span class="badge" id="playBadge">STATE</span><span class="badge" id="canBadge">ROLE</span><span class="badge" id="clientBadge">0 CLIENTS</span><span class="badge" id="heapBadge">HEAP</span>
  </div>
 </div>
 <div id="globalNotice" class="notice" style="display:none"></div>
 <div class="tabs">
  <button class="tab active" data-tab="control">Control</button>
  <button class="tab" data-tab="diagnostics">Diagnostics</button>
  <button class="tab" data-tab="config">Configuration</button>
  <button class="tab" data-tab="terminal">Terminal</button>
  <button class="tab" data-tab="spiffs">SPIFFS</button>
 </div>

 <section id="control" class="view active">
  <div class="grid">
   <div class="card span12"><div class="row between"><div class="row"><button class="danger" onclick="cmd('stop')">STOP</button><button class="primary" onclick="cmd('play')">PLAY</button><button onclick="cmd('zero')">ZERO TORQUE</button><button onclick="cmd('status')">STATUS</button></div><div class="tiny">All web commands pass through the same command queue/parser as USB Serial.</div></div></div>
   <div class="card span12" id="legStateCard">
    <h2>Joint state</h2>
    <div class="grid">
     <div class="metric span3"><div class="k">Outer calf</div><div class="v" id="aOuter">—</div></div>
     <div class="metric span3"><div class="k">Inner calf</div><div class="v" id="aInner">—</div></div>
     <div class="metric span3"><div class="k">Hip pitch</div><div class="v" id="aHip">—</div></div>
     <div class="metric span3"><div class="k">Knee</div><div class="v" id="aKnee">—</div></div>
     <div class="metric span3"><div class="k">Hip roll</div><div class="v" id="aRoll">—</div></div>
    </div>
   </div>
   <div class="card span12" id="legActuatorCard">
    <div class="row between"><h2>Actuator control</h2><span class="tiny">Torque values are firmware command units; global clamp follows MaxTorqueLimit × 100.</span></div>
    <div id="jointControls"></div>
   </div>
   <div class="card span12" id="neckControlCard" style="display:none">
    <div class="row between"><div><h2>Head / neck Stewart platform</h2><div class="tiny">Six A4988/NEMA17 axes via FastAccelStepper. Position is OPEN LOOP step count unless physical feedback is added.</div></div><div class="row"><button class="danger" onclick="cmd('stop')">STOP ALL</button><button onclick="cmd('HOME_SOFT')">HOME SOFT</button><button class="warn" onclick="cmd('HOME_BRUTE')">HOME BRUTE</button><button onclick="cmd('neck zero')">SOFTWARE ZERO</button></div></div>
    <div class="section-title">Pose command</div><div class="row"><label>X <input id="nx" type="number" value="0"></label><label>Y <input id="ny" type="number" value="0"></label><label>Z <input id="nz" type="number" value="0"></label><label>H mm <input id="nh" type="number" value="0"></label><label>Roll <input id="nr" type="number" value="0"></label><label>Pitch <input id="np" type="number" value="0"></label><label>Speed × <input id="ns" type="number" step=".1" value="1"></label><label>Accel × <input id="na" type="number" step=".1" value="1"></label><button class="primary" onclick="sendNeckPose()">Move pose</button></div>
    <div class="section-title">Actuators</div><div id="neckMotors"></div>
   </div>
  </div>
 </section>

 <section id="diagnostics" class="view">
  <div class="grid">
   <div class="card span12">
    <div class="row between"><div><h2>Runtime diagnostic wrapper</h2><div class="diag-note">Health is computed from the actual runtime acquisition/transmit points. Motor rows verify the ESP32→MCP2515 transmit path only; actuator health remains unverified until CAN RX is implemented.</div></div><button onclick="loadDiagnostics()">Refresh now</button></div>
    <div class="row" style="margin-top:12px;gap:16px"><div id="diagOverall" class="overall">—</div><div class="stack tiny"><span id="diagTime">—</span><span id="diagRole">—</span></div></div>
   </div>
   <div class="card span12"><h2>Modules</h2><div id="diagModules" class="diagcards"></div></div>
   <div class="card span12"><div class="row between"><h2>AS5600 one-wire encoders</h2><span class="tiny">Independent GPIO PWM capture: duty, frequency, pulse age, decoded/filtered angle and joint constraints.</span></div><div id="diagSensors" class="diagtable"></div></div>
   <div class="card span12"><div class="row between"><h2>Actuator command paths</h2><span class="tiny">TX path status is not actuator feedback.</span></div><div id="diagActuators" class="diagtable"></div></div>
   <div class="card span12"><div class="row between"><h2>Head / neck stepper channels</h2><span class="tiny">FastAccelStepper position is commanded/open-loop state, not physical encoder feedback.</span></div><div id="diagNeck" class="diagtable"></div></div>
   <div class="card span12"><h2>FreeRTOS / control loops</h2><div id="diagTasks" class="diagtable"></div></div>
   <div class="card span8"><h2>I²C / IMU probes</h2><div id="diagImus" class="diagtable"></div></div>
   <div class="card span4"><h2>Safety state</h2><div id="diagSafety" class="kv"></div></div>
  </div>
 </section>

 <section id="config" class="view">
  <div class="grid">
   <div class="card span12"><h2>Controller identity / operating structure</h2><div class="row">
    <label>Role <select id="cfgRole"><option value="left">LEFTLEG</option><option value="right">RIGHTLEG</option><option value="center">CENTER</option><option value="head">HEAD / NECK</option></select></label>
    <label>Operating path <select id="cfgOperatingMode"><option value="standalone">STANDALONE / ORIGINAL</option><option value="hyperspawn">HYPERSPAWN / ROS2 ROUTE</option></select></label>
    <label>Max torque <input id="cfgMaxTorque" type="number" step=".1" min=".1" max="100"></label>
    <label><input id="cfgLegacyUnaddressed" type="checkbox"> Allow legacy unaddressed commands</label>
    <button class="primary" onclick="saveConfig()">Save configuration</button>
    <button onclick="reloadConfig()">Reload SPIFFS</button>
    <button class="warn" onclick="saveAndReboot()">Save + reboot</button>
   </div><div class="tiny" style="margin-top:8px">Role controls LEFTLEG/RIGHTLEG/CENTER/HEADNECK AP identity and selects a mutually-exclusive hardware pin graph. Every external command uses &lt;DB1:TARGET&gt; addressing. Legacy unaddressed commands are disabled by default. Operating path selects local portal/Serial authority or the parallel HyperSpawn ROS2 CAN route. Topology changes require reboot.</div></div>

   <div class="card span12"><h2>HyperSpawn / ROS2 route</h2><div class="row">
    <label>Command timeout ms <input id="cfgHsTimeout" type="number" min="50" max="10000" step="10"></label>
    <label>Position wire units / degree <input id="cfgHsScale" type="number" min="0.001" max="1000" step="0.001"></label>
    <label><input id="cfgHsAutoArm" type="checkbox"> Auto-arm on valid command</label>
    <label><input id="cfgHsLegacy" type="checkbox"> Legacy brain broadcast compatibility</label>
   </div><div class="tiny" style="margin-top:8px">Targeted route uses node 0x12 for LEFTLEG and 0x13 for RIGHTLEG. Legacy 0x011/0x012 broadcast compatibility is optional because those IDs cannot distinguish two legs on the same shared bus.</div></div>

   <div class="card span12" id="neckConfigCard"><h2>Head / neck controller</h2><div class="row">
    <label>Speed Hz <input id="cfgNeckSpeed" type="number" min="1" max="200000"></label>
    <label>Acceleration <input id="cfgNeckAccel" type="number" min="1" max="2000000"></label>
    <label>Steps / mm <input id="cfgNeckSteps" type="number" step=".01" min=".01"></label>
    <label><input id="cfgNeckBt" type="checkbox"> Bluetooth NECK_BT</label>
    <label><input id="cfgNeckAutoHome" type="checkbox"> Auto HOME_BRUTE on boot</label>
    <label><input id="cfgNeckEnable" type="checkbox"> GPIO25 shared enable</label>
   </div><div class="section-title">Software actuator travel limits (mm)</div><div id="neckLimitFields" class="constraints"></div><div class="tiny" style="margin-top:8px">These are open-loop software limits. HEAD/NECK uses STEP/DIR pins 33/32, 18/26, 23/14, 19/27, 22/12, 21/13. It must reboot when switching to/from a leg or center role.</div></div>

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
  <div class="card"><div class="row"><input id="termInput" type="text" style="flex:1" placeholder="Payload only; portal adds <DB1:CURRENT_TARGET> automatically"><button class="primary" onclick="sendTerminal()">Send</button></div>
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
let state=null,config=null,diagnostics=null,lastLog=0,renderedRole='';

document.querySelectorAll('.tab').forEach(b=>b.onclick=()=>{document.querySelectorAll('.tab').forEach(x=>x.classList.remove('active'));document.querySelectorAll('.view').forEach(x=>x.classList.remove('active'));b.classList.add('active');document.getElementById(b.dataset.tab).classList.add('active')});
document.getElementById('termInput').addEventListener('keydown',e=>{if(e.key==='Enter')sendTerminal()});

function notice(t,good=false){const n=document.getElementById('globalNotice');n.style.display=t?'block':'none';n.className=good?'okmsg':'notice';n.textContent=t||''}
function fmtDeg(v){return Number.isFinite(+v)?(+v).toFixed(1)+'°':'—'}
async function jfetch(url,opt){const r=await fetch(url,opt);let data;const ct=r.headers.get('content-type')||'';data=ct.includes('json')?await r.json():await r.text();if(!r.ok)throw new Error(typeof data==='string'?data:(data.error||r.status));return data}

function healthClass(s){return ['ok','warn','fault','inactive','idle'].includes(s)?s:'inactive'}
function healthPill(s){return `<span class="health ${healthClass(s)}">${String(s||'unknown').toUpperCase()}</span>`}
function age(v){return +v<0?'never':(+v<1000?`${v} ms`:`${(+v/1000).toFixed(1)} s`)}
function boolText(v){return v?'YES':'NO'}
function kvRows(rows){return rows.map(([k,v])=>`<div class="k">${k}</div><div class="v">${v}</div>`).join('')}
function moduleCard(title,m,rows){return `<div class="diagcard"><div class="diaghead"><span class="diagtitle">${title}</span>${healthPill(m?.status)}</div><div class="kv">${kvRows(rows)}</div></div>`}
function renderDiagnostics(){
 const d=diagnostics;if(!d)return;
 const ov=document.getElementById('diagOverall');ov.textContent=String(d.overall||'—').toUpperCase();ov.className='overall '+healthClass(d.overall);
 document.getElementById('diagTime').textContent=`snapshot ${(d.timestamp_ms/1000).toFixed(1)} s uptime`;
 document.getElementById('diagRole').textContent=`role ${d.role} · operating ${state?.operating_mode||'standalone'} · boot topology ${d.role_at_boot}${d.reboot_required?' · REBOOT REQUIRED':''}`;
 const db=document.getElementById('diagBadge');db.textContent='DIAG '+String(d.overall||'').toUpperCase();db.className='badge '+(d.overall==='ok'?'ok':d.overall==='fault'?'bad':'warn');
 const m=d.modules||{};
 document.getElementById('diagModules').innerHTML=
  moduleCard('Controller',m.controller,[['configured',boolText(m.controller?.configured)],['control runtime',boolText(m.controller?.control_ready)],['IMU runtime',boolText(m.controller?.imu_ready)],['play',boolText(m.controller?.play)],['max torque',m.controller?.max_torque],['free heap',Math.round((m.controller?.free_heap||0)/1024)+' KB']])+
  moduleCard('Wi-Fi / portal',m.wifi,[['SSID',m.wifi?.ssid],['IP',m.wifi?.ip],['clients',m.wifi?.clients],['HTTP requests',m.wifi?.http_requests],['captive redirects',m.wifi?.redirects]])+
  moduleCard('SPIFFS',m.spiffs,[['mounted',boolText(m.spiffs?.mounted)],['usage',`${m.spiffs?.used} / ${m.spiffs?.total}`],['config.txt',boolText(m.spiffs?.config_exists)],['config.bak',boolText(m.spiffs?.backup_exists)],['read / write',`${m.spiffs?.reads} / ${m.spiffs?.writes}`],['errors',m.spiffs?.errors]])+
  moduleCard('SPI bus',m.spi,[['initialized',boolText(m.spi?.initialized)],['SCK',m.spi?.sck],['MISO',m.spi?.miso],['MOSI',m.spi?.mosi],['CS',m.spi?.cs]])+
  moduleCard('AS5600 encoder I/O',m.encoder_io,[['interface',m.encoder_io?.interface],['pins',(m.encoder_io?.pins||[]).join(', ')],['nominal Hz',(m.encoder_io?.supported_nominal_hz||[]).join('/')]])+
  moduleCard('MCP2515 / CAN',m.can,[['initialized',boolText(m.can?.initialized)],['bus','1 Mbps @ 8 MHz'],['CS / INT',`${m.can?.cs_gpio} / ${m.can?.int_gpio}`],['TX ok / fail',`${m.can?.tx_success} / ${m.can?.tx_failure}`],['consecutive fail',m.can?.consecutive_failures],['last TX',age(m.can?.last_tx_age_ms)],['feedback','TX ONLY']])+
  moduleCard('I²C / IMU mux',m.i2c,[['initialized',boolText(m.i2c?.initialized)],['SDA',m.i2c?.sda],['SCL',m.i2c?.scl],['TCA9548A',m.i2c?.mux_seen?'0x70 present':'not seen'],['mux ok / fail',`${m.i2c?.mux_select_ok} / ${m.i2c?.mux_select_fail}`]])+
  moduleCard('HyperSpawn / ROS2 route',d.hyperspawn_route,[['active',boolText(d.hyperspawn_route?.active)],['node',d.hyperspawn_route?.node_id],['control',d.hyperspawn_route?.control_mode],['last command',age(d.hyperspawn_route?.last_command_age_ms)],['RX targeted',d.hyperspawn_route?.targeted_rx],['RX legacy',d.hyperspawn_route?.legacy_rx],['completed cmds',d.hyperspawn_route?.completed_commands],['fragment timeouts',d.hyperspawn_route?.fragment_timeouts],['state TX',d.hyperspawn_route?.state_tx],['watchdog trips',d.hyperspawn_route?.watchdog_trips]])+
  moduleCard('Command transport',m.commands,[['queue',`${m.commands?.queue_depth} / ${m.commands?.queue_capacity}`],['web queued',m.commands?.web_queued],['web processed',m.commands?.web_processed],['queue full',m.commands?.queue_full_events],['serial processed',m.commands?.serial_processed]])+
  moduleCard('Head / neck',m.neck,[['runtime',boolText(m.neck?.runtime_ready)],['motion',boolText(m.neck?.motion_enabled)],['software homed',boolText(m.neck?.software_homed)],['feedback',m.neck?.feedback],['speed Hz',m.neck?.speed_hz],['acceleration',m.neck?.accel],['Bluetooth',m.neck?.bluetooth]]);
 const sensors=d.sensors||[];
 document.getElementById('diagSensors').innerHTML=`<table><thead><tr><th>Health</th><th>Sensor</th><th>GPIO</th><th>Raw</th><th>Filtered</th><th>Angle</th><th>PWM duty</th><th>PWM Hz</th><th>Pulse age</th><th>Sample age</th><th>Observed raw range</th><th>Constraint</th><th>Notes</th></tr></thead><tbody>${sensors.map(x=>`<tr><td>${healthPill(x.status)}</td><td>${x.name.replaceAll('_',' ')}</td><td class="mono">${x.gpio} / PWM</td><td class="mono">${x.raw}</td><td class="mono">${Number(x.filtered_raw).toFixed(1)}</td><td class="mono">${Number(x.angle).toFixed(1)}°</td><td class="mono">${Number(x.duty_percent||0).toFixed(2)}%</td><td class="mono">${Number(x.frequency_hz||0).toFixed(1)}</td><td>${x.pulse_age_us===4294967295?'never':((x.pulse_age_us||0)/1000).toFixed(1)+' ms'}</td><td>${age(x.sample_age_ms)}</td><td class="mono">${x.min_raw_seen}…${x.max_raw_seen}</td><td class="mono">${x.constraint_min}…${x.constraint_max}°</td><td>${x.signal_valid?'PWM VALID':'PWM INVALID'} · change ${age(x.last_change_age_ms)}</td></tr>`).join('')}</tbody></table>`;
 const acts=d.actuators||[];
 document.getElementById('diagActuators').innerHTML=`<table><thead><tr><th>TX path</th><th>Actuator</th><th>CAN ID</th><th>Owned</th><th>Source</th><th>Direct</th><th>Impedance</th><th>Last sent</th><th>Opcode</th><th>TX ok/fail</th><th>TX age</th><th>Feedback</th></tr></thead><tbody>${acts.map(x=>`<tr><td>${healthPill(x.status)}</td><td>${x.name.replaceAll('_',' ')}</td><td class="mono">${x.can_id}</td><td>${boolText(x.selected)}</td><td>${x.command_source}</td><td class="mono">${x.direct_setpoint}</td><td class="mono">${x.impedance_setpoint}${x.impedance_enabled?' *':''}</td><td class="mono">${x.last_sent}</td><td class="mono">${x.last_opcode}</td><td class="mono">${x.tx_ok}/${x.tx_fail}</td><td>${age(x.tx_age_ms)}</td><td>${x.feedback==='can_rx_unparsed'?'CAN RX ACTIVE — RMD DECODE PENDING':x.feedback}</td></tr>`).join('')}</tbody></table>`;
 const neck=d.neck_steppers||[];
 document.getElementById('diagNeck').innerHTML=`<table><thead><tr><th>Health</th><th>Motor</th><th>STEP</th><th>DIR</th><th>Current</th><th>Target</th><th>Moving</th><th>Limits</th><th>Feedback</th></tr></thead><tbody>${neck.map(x=>`<tr><td>${healthPill(x.status)}</td><td>M${x.motor}</td><td class="mono">${x.step_gpio}</td><td class="mono">${x.dir_gpio}</td><td class="mono">${Number(x.current_mm).toFixed(2)} mm / ${x.current_steps}</td><td class="mono">${Number(x.target_mm).toFixed(2)} mm / ${x.target_steps}</td><td>${boolText(x.moving)}</td><td class="mono">${x.min_mm}…${x.max_mm} mm</td><td>${x.feedback}</td></tr>`).join('')}</tbody></table>`;
 const tasks=d.tasks||[];
 document.getElementById('diagTasks').innerHTML=`<table><thead><tr><th>Health</th><th>Task</th><th>Active</th><th>Expected rate</th><th>Loop count</th><th>Heartbeat age</th><th>Min free stack</th></tr></thead><tbody>${tasks.map(x=>`<tr><td>${healthPill(x.status)}</td><td>${x.name}</td><td>${boolText(x.active)}</td><td>${x.expected_hz} Hz</td><td class="mono">${x.loops}</td><td>${age(x.age_ms)}</td><td class="mono">${x.stack_high_water_words} words</td></tr>`).join('')}</tbody></table>`;
 const imus=d.imus||[];
 document.getElementById('diagImus').innerHTML=`<table><thead><tr><th>Health</th><th>Index</th><th>Mux CH</th><th>Address</th><th>Seen</th><th>Read ok/fail</th><th>Last seen</th><th>Accel raw</th><th>Gyro raw</th></tr></thead><tbody>${imus.map(x=>`<tr><td>${healthPill(x.status)}</td><td>${x.index}</td><td>${x.mux_channel}</td><td class="mono">${x.address}</td><td>${boolText(x.ever_seen)}</td><td class="mono">${x.read_ok}/${x.read_fail}</td><td>${age(x.seen_age_ms)}</td><td class="mono">${(x.accel||[]).join(', ')}</td><td class="mono">${(x.gyro||[]).join(', ')}</td></tr>`).join('')}</tbody></table>`;
 const sf=d.safety||{};document.getElementById('diagSafety').innerHTML=kvRows([['play',boolText(sf.play)],['stop bursts',sf.stop_burst_remaining],['calibration override',boolText(sf.calibration_override)],['calibration motor',sf.calibration_actuator_index],['calibration torque',sf.calibration_torque],['reboot required',boolText(sf.reboot_required)],['command watchdog',sf.command_watchdog],['CAN feedback monitor',sf.can_feedback_monitoring]]);
}
async function loadDiagnostics(){try{diagnostics=await jfetch('/api/diagnostics');renderDiagnostics()}catch(e){const b=document.getElementById('diagBadge');b.textContent='DIAG OFFLINE';b.className='badge bad'}}

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
  document.getElementById('roleTitle').textContent=(state.role||'unconfigured').toUpperCase();document.getElementById('addressBadge').textContent='DB1:'+String(state.command_address||routeTarget());document.getElementById('operatingText').textContent=state.role==='head'?'NECK STEPPER':(state.operating_mode||'standalone').toUpperCase();
  document.getElementById('ipText').textContent='@ '+state.ip+' · '+state.ssid+(state.role==='head'?' · 6× A4988 STEP/DIR':' · AS5600 GPIO '+(state.sensor_pins||[]).join('/'));
  const pb=document.getElementById('playBadge');pb.textContent=state.play?'PLAY':'STOPPED';pb.className='badge '+(state.play?'ok':'bad');
  const cb=document.getElementById('canBadge');const ready=state.control_ready||state.imu_ready||state.neck_ready;cb.textContent=ready?'RUNTIME READY':(state.configured?'REBOOT REQUIRED':'SETUP');cb.className='badge '+(ready?'ok':'warn');
  document.getElementById('clientBadge').textContent=state.clients+' CLIENT'+(state.clients===1?'':'S');
  document.getElementById('heapBadge').textContent=Math.round(state.heap/1024)+' KB HEAP';
  document.getElementById('aOuter').textContent=fmtDeg(state.angles.outer_calf);document.getElementById('aInner').textContent=fmtDeg(state.angles.inner_calf);document.getElementById('aHip').textContent=fmtDeg(state.angles.hip_pitch);document.getElementById('aKnee').textContent=fmtDeg(state.angles.knee);document.getElementById('aRoll').textContent=fmtDeg(state.angles.hip_roll);
  if(!state.configured)notice('Controller is not configured. Actuator tasks are disabled. Select a role under Configuration, save, then reboot.');
  else if(state.reboot_required)notice('Saved role or operating structure differs from the boot topology. Reboot is required before control authority changes.');
  else if(state.role==='head')notice('HEAD / NECK personality active — FastAccelStepper OPEN-LOOP state. STOP remains available at all times.');
  else if(state.operating_mode==='hyperspawn')notice('HYPERSPAWN / ROS2 ROUTE ACTIVE — portal motion controls are read-only; STOP, diagnostics, configuration and terminal remain available.');
  else notice('');
  renderJoints();renderNeck();
 }catch(e){}
}
function targetForRole(r,configured=true){if(!configured)return 'SETUP';if(r==='left')return 'LEFTLEG';if(r==='right')return 'RIGHTLEG';if(r==='center')return 'CENTER';if(r==='head')return 'HEADNECK';return 'SETUP'}
function routeTarget(){return targetForRole(state?.role,!!state?.configured)}
function routed(c){c=(c||'').trim();if(c.startsWith('<DB1:'))return c;return `<DB1:${routeTarget()}> ${c}`}
async function cmd(c){try{const wire=routed(c);await jfetch('/api/command',{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},body:new URLSearchParams({cmd:wire})});}catch(e){notice('Command failed: '+e.message)}}
function sendTerminal(){const e=document.getElementById('termInput');const c=e.value.trim();if(c){cmd(c);e.value=''}}
function sendTorque(j){if(!state||!['left','right'].includes(state.role))return notice('Configure a leg role first.');if(state.operating_mode==='hyperspawn')return notice('Manual torque is locked: HyperSpawn/ROS2 owns command authority. STOP remains available.');cmd(`torque ${j} ${document.getElementById('t_'+j).value}`)}
function sendImpedance(j){if(!state||!['left','right'].includes(state.role))return notice('Configure a leg role first.');if(state.operating_mode==='hyperspawn')return notice('Manual impedance is locked: HyperSpawn/ROS2 owns command authority.');const on=document.getElementById('ie_'+j).checked?1:0;cmd(`impedance ${j} ${on} ${document.getElementById('ip_'+j).value} ${document.getElementById('iv_'+j).value}`)}
function sendNeckPose(){cmd(`X${document.getElementById('nx').value},Y${document.getElementById('ny').value},Z${document.getElementById('nz').value},H${document.getElementById('nh').value},S${document.getElementById('ns').value},A${document.getElementById('na').value},R${document.getElementById('nr').value},P${document.getElementById('np').value}`)}
function sendNeckMotor(i){const e=document.getElementById('nm_'+i);if(e)cmd(`${i}:${e.value}`)}
function renderNeck(){const isHead=state?.role==='head';document.getElementById('neckControlCard').style.display=isHead?'block':'none';document.getElementById('legStateCard').style.display=isHead?'none':'block';document.getElementById('legActuatorCard').style.display=isHead?'none':'block';if(!isHead)return;const b=document.getElementById('neckMotors');const motors=state?.neck?.motors||[];b.innerHTML=motors.map(m=>`<div class="joint"><div><div class="jointname">Motor ${m.index}</div><div class="canid">STEP ${m.step_gpio} · DIR ${m.dir_gpio}</div></div><div><span class="tiny">current</span><br>${Number(m.current_mm).toFixed(2)} mm</div><div><span class="tiny">target ${Number(m.target_mm).toFixed(2)} mm · ${m.moving?'MOVING':'IDLE'}</span><div style="height:5px;background:#20252a;margin-top:5px"><div style="height:100%;background:#aeb7bd;width:${Math.max(0,Math.min(100,100*(m.current_mm-m.min_mm)/Math.max(.001,m.max_mm-m.min_mm)))}%"></div></div></div><div class="row"><input id="nm_${m.index}" type="number" step=".1" value="${Number(m.target_mm).toFixed(2)}"><button onclick="sendNeckMotor(${m.index})">Move mm</button></div></div>`).join('')}

function buildConfigFields(){
 const df=document.getElementById('dirFields');df.innerHTML='';dirNames.forEach((n,i)=>{const d=document.createElement('div');d.className='constraint';d.innerHTML=`<span>${n.replaceAll('_',' ')}</span><select id="dm${i}"><option value="1">+</option><option value="-1">−</option></select><span></span>`;df.appendChild(d)});
 const cf=document.getElementById('constraintFields');cf.innerHTML='';constraintNames.forEach(n=>{const d=document.createElement('div');d.className='constraint';d.innerHTML=`<span>${n.replaceAll('_',' ')}</span><input id="${n}_min" type="number" placeholder="min"><input id="${n}_max" type="number" placeholder="max">`;cf.appendChild(d)});
 const nf=document.getElementById('neckLimitFields');nf.innerHTML='';for(let i=0;i<6;i++){const d=document.createElement('div');d.className='constraint';d.innerHTML=`<span>Motor ${i+1}</span><input id="neckMin${i}" type="number" step=".1" placeholder="min mm"><input id="neckMax${i}" type="number" step=".1" placeholder="max mm">`;nf.appendChild(d)}
}
async function loadConfig(){
 try{
  config=await jfetch('/api/config');document.getElementById('cfgRole').value=['left','right','center','head'].includes(config.role)?config.role:'left';document.getElementById('cfgMaxTorque').value=config.max_torque;document.getElementById('cfgLegacyUnaddressed').checked=!!config.legacy_unaddressed_commands;document.getElementById('cfgOperatingMode').value=config.operating_mode||'standalone';document.getElementById('cfgHsTimeout').value=config.hyperspawn_timeout_ms||250;document.getElementById('cfgHsScale').value=config.hyperspawn_position_units_per_degree||1;document.getElementById('cfgHsLegacy').checked=!!config.hyperspawn_legacy_broadcast;document.getElementById('cfgHsAutoArm').checked=!!config.hyperspawn_auto_arm;const n=config.neck||{};document.getElementById('cfgNeckSpeed').value=n.speed_hz||48000;document.getElementById('cfgNeckAccel').value=n.accel||36000;document.getElementById('cfgNeckSteps').value=n.steps_per_mm||426.67;document.getElementById('cfgNeckBt').checked=n.bluetooth_enabled!==false;document.getElementById('cfgNeckAutoHome').checked=!!n.auto_home;document.getElementById('cfgNeckEnable').checked=n.use_enable_pin!==false;(n.min_mm||[]).forEach((v,i)=>{const e=document.getElementById('neckMin'+i);if(e)e.value=v});(n.max_mm||[]).forEach((v,i)=>{const e=document.getElementById('neckMax'+i);if(e)e.value=v});
  config.left_offsets.forEach((v,i)=>document.getElementById('lo'+i).value=v);config.right_offsets.forEach((v,i)=>document.getElementById('ro'+i).value=v);config.directions.forEach((v,i)=>document.getElementById('dm'+i).value=v<0?'-1':'1');
  Object.entries(config.constraints).forEach(([n,a])=>{const mn=document.getElementById(n+'_min'),mx=document.getElementById(n+'_max');if(mn){mn.value=a[0];mx.value=a[1]}});
  document.getElementById('rawConfig').value=config.raw_config||'';
 }catch(e){notice('Config read failed: '+e.message)}
}
function configParams(){
 const p=new URLSearchParams({target:routeTarget(),role:document.getElementById('cfgRole').value,operatingMode:document.getElementById('cfgOperatingMode').value,max_torque:document.getElementById('cfgMaxTorque').value,legacyUnaddressed:document.getElementById('cfgLegacyUnaddressed').checked?'1':'0',hsTimeout:document.getElementById('cfgHsTimeout').value,hsScale:document.getElementById('cfgHsScale').value,hsLegacy:document.getElementById('cfgHsLegacy').checked?'1':'0',hsAutoArm:document.getElementById('cfgHsAutoArm').checked?'1':'0',neckSpeed:document.getElementById('cfgNeckSpeed').value,neckAccel:document.getElementById('cfgNeckAccel').value,neckStepsPerMm:document.getElementById('cfgNeckSteps').value,neckBluetooth:document.getElementById('cfgNeckBt').checked?'1':'0',neckAutoHome:document.getElementById('cfgNeckAutoHome').checked?'1':'0',neckUseEnable:document.getElementById('cfgNeckEnable').checked?'1':'0'});
 for(let i=0;i<5;i++){p.set('lo'+i,document.getElementById('lo'+i).value);p.set('ro'+i,document.getElementById('ro'+i).value)}
 for(let i=0;i<10;i++)p.set('dm'+i,document.getElementById('dm'+i).value);
 constraintNames.forEach(n=>{p.set(n+'_min',document.getElementById(n+'_min').value);p.set(n+'_max',document.getElementById(n+'_max').value)});
 for(let i=0;i<6;i++){p.set('neckMin'+i,document.getElementById('neckMin'+i).value);p.set('neckMax'+i,document.getElementById('neckMax'+i).value)}
 return p
}
async function saveConfig(){try{const r=await jfetch('/api/config',{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},body:configParams()});notice('Configuration saved. AP identity: '+r.ssid+(r.reboot_required?' · reboot required':''),true);setTimeout(loadConfig,300);return r}catch(e){notice('Save failed: '+e.message);throw e}}
async function saveAndReboot(){try{const saved=await saveConfig();const nextTarget=targetForRole(saved.role,true);const r=await jfetch('/api/reboot?target='+encodeURIComponent(nextTarget),{method:'POST'});notice('Rebooting. Reconnect to '+r.next_ssid+'.',true)}catch(e){notice('Reboot request failed: '+e.message)}}
async function reloadConfig(){try{await jfetch('/api/config/reload?target='+encodeURIComponent(routeTarget()),{method:'POST'});await loadConfig();notice('Configuration reloaded from SPIFFS.',true)}catch(e){notice('Reload failed: '+e.message)}}
async function calibrateSensors(){if(!confirm('Support the leg in the 180° calibration pose. Calibrate and save current sensor offsets?'))return;try{await jfetch('/api/calibrate/sensors?target='+encodeURIComponent(routeTarget()),{method:'POST'});notice('Calibration queued. Watch Terminal for results.',true);setTimeout(loadConfig,1500)}catch(e){notice('Calibration failed: '+e.message)}}
async function writeRawConfig(){if(!confirm('Write raw /config.txt? Current file will be copied to /config.bak.'))return;try{const r=await jfetch('/api/config/raw?target='+encodeURIComponent(routeTarget()),{method:'POST',headers:{'Content-Type':'text/plain'},body:document.getElementById('rawConfig').value});notice('Raw config written. Reboot required. Next AP: '+r.ssid,true);setTimeout(loadConfig,300)}catch(e){notice('Raw write failed: '+e.message)}}

async function loadLogs(){try{const d=await jfetch('/api/log?since='+lastLog);const c=document.getElementById('console');d.entries.forEach(e=>{c.textContent+=`[${(e.ms/1000).toFixed(3)}] ${e.text}\n`;lastLog=Math.max(lastLog,e.seq)});if(d.entries.length)c.scrollTop=c.scrollHeight}catch(e){}}
async function loadFiles(){try{const d=await jfetch('/api/spiffs/list');const b=document.getElementById('fileList');b.innerHTML=`<div class="tiny">${d.used} / ${d.total} bytes used</div>`;d.files.forEach(f=>{const e=document.createElement('div');e.className='file';e.innerHTML=`<span>${f.name}<br><span class="tiny">${f.size} bytes</span></span><button>View</button>`;e.querySelector('button').onclick=()=>readFile(f.name);b.appendChild(e)})}catch(e){notice('SPIFFS list failed: '+e.message)}}
async function readFile(path){try{document.getElementById('fileTitle').textContent=path;document.getElementById('fileViewer').value=await jfetch('/api/spiffs/read?path='+encodeURIComponent(path))}catch(e){notice('File read failed: '+e.message)}}

buildConfigFields();loadState();loadDiagnostics();loadConfig();loadFiles();loadLogs();setInterval(loadState,500);setInterval(loadLogs,500);setInterval(loadDiagnostics,1000);
</script>
</body>
</html>
)DBHTML";

void handlePortalRoot() {
  portalHttpRequests++;
  sendNoCacheHeaders();
  server.send_P(200, "text/html", PORTAL_HTML);
}

void registerPortalRoutes() {
  if (portalRoutesRegistered) return;
  portalRoutesRegistered = true;

  server.on("/", HTTP_GET, handlePortalRoot);
  server.on("/api/state", HTTP_GET, handleApiState);
  server.on("/api/diagnostics", HTTP_GET, handleApiDiagnostics);
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

  portalOnline = false;
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
    portalOnline = false;
    dbPrintln("ERROR: SoftAP start failed.");
    return;
  }

  dnsServer.start(DNS_PORT, "*", WiFi.softAPIP());
  server.begin();
  portalOnline = true;

  dbPrintf("Captive portal active: SSID=%s IP=%s\n",
           portalSSID.c_str(), WiFi.softAPIP().toString().c_str());
}

void setupPortal() {
  registerPortalRoutes();
  startOrRestartSoftAP();
}

void portalTask(void *parameter) {
  while (true) {
    portalTaskLoops++;
    lastPortalTaskMs = millis();
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

  if (!SPIFFS.begin(true)) {
    spiffsMounted = false;
    spiffsErrors++;
    dbPrintln("ERROR: SPIFFS mount failed.");
    while (true) delay(1000);
  }
  spiffsMounted = true;

  loadConfig();
  roleAtBoot = selectedRoleName();
  operatingModeAtBoot = operatingModeName();
  dbPrintf("DB1 command address: %s | unaddressed legacy commands: %s\n",
           currentCommandAddress().c_str(), legacyUnaddressedCommands ? "ENABLED" : "DISABLED");

  // Hardware graphs are initialized only after the persisted role is known.
  // HEAD_NECK shares many GPIOs with SPI/I2C/AS5600 and must never initialize
  // those leg/center peripherals in the same boot.
  if (configProvisioned && !isHead) {
    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
    i2cInitialized = true;
  }
  if (isLegRole()) {
    setupAs5600Inputs();
    dbPrintf("AS5600 one-wire PWM inputs: outer=%d inner=%d hipPitch=%d knee=%d hipRoll=%d\n",
             PIN_OUTER_CALF, PIN_INNER_CALF, PIN_HIP_PITCH, PIN_KNEE, PIN_HIP_ROLL);
  }

  if (DROPBEAR_ENABLE_WIFI_PORTAL) {
    setupPortal();
    xTaskCreatePinnedToCore(portalTask, "portal", 8192, nullptr, 1, &portalTaskHandle, 0);
  }

  if (configProvisioned && isHead) {
    setupNeckHardware();
    xTaskCreatePinnedToCore(neckServiceTask, "neck-service", 6144, nullptr, 2, &neckTaskHandle, 1);
    if (neckAutoHome && runtimeNeckReady) neckStartBruteHome();
  } else if (configProvisioned && !isCenter) {
    pinMode(CAN0_INT, INPUT_PULLUP);
    SPI.begin(SPI_SCK_PIN, SPI_MISO_PIN, SPI_MOSI_PIN, CAN_CS_PIN);
    spiInitialized = true;

    if (CAN.begin(MCP_ANY, CAN_1000KBPS, MCP_8MHZ) != CAN_OK) {
      canInitialized = false;
      dbPrintln("ERROR: MCP2515 CAN initialization failed at 1 Mbps / 8 MHz.");
      playMode = false;
    } else {
      CAN.setMode(MCP_NORMAL);
      canInitialized = true;
      dbPrintln("CAN initialized: 1 Mbps, MCP2515 8 MHz, CS GPIO5.");

      primeSensorFilter();

      // Wi-Fi/networking lives primarily on core 0. Keep the control path on
      // core 1 so captive-portal traffic does not become sensor/control jitter.
      xTaskCreatePinnedToCore(readAndComputeTask, "sensors", 4096, nullptr, 3, &sensorTaskHandle, 1);
      xTaskCreatePinnedToCore(impedanceControlTask, "impedance", 4096, nullptr, 3, &impedanceTaskHandle, 1);
      xTaskCreatePinnedToCore(canOutputTask, "can-output", 4096, nullptr, 4, &canTaskHandle, 1);
      xTaskCreatePinnedToCore(canReceiveTask, "can-rx", 4096, nullptr, 4, &canRxTaskHandle, 1);
      xTaskCreatePinnedToCore(hyperspawnRouteTask, "hyperspawn", 4096, nullptr, 2, &hyperspawnTaskHandle, 1);
      runtimeControlReady = true;
      runtimeImuReady = false;
      runtimeNeckReady = false;
      playMode = (operatingMode == OPERATING_STANDALONE);
      if (operatingMode == OPERATING_HYPERSPAWN_ROUTE) {
        dbPrintln("HyperSpawn/ROS2 route selected. Waiting for a valid CAN command before actuator output is armed.");
      }
    }
  } else if (configProvisioned && isCenter) {
    xTaskCreatePinnedToCore(imuReadTask, "imu", 4096, nullptr, 2, &imuTaskHandle, 1);
    runtimeControlReady = false;
    runtimeImuReady = true;
    runtimeNeckReady = false;
    playMode = true;
  } else {
    runtimeControlReady = false;
    runtimeImuReady = false;
    runtimeNeckReady = false;
    playMode = false;
    dbPrintln("Controller is unconfigured. CAN/IMU/neck runtime tasks are intentionally disabled.");
  }

  // One command task consumes both Serial and web-command queue traffic.
  xTaskCreatePinnedToCore(checkChiralityTask, "command", 6144, nullptr, 2, &commandTaskHandle, 1);

  dbPrintf("Dropbear controller ready: role=%s stack=%s portal=%s SSID=%s. Type 'help'.\n",
           selectedRoleName().c_str(), isHead ? "neck" : (isCenter ? "imu" : operatingModeName().c_str()),
           DROPBEAR_ENABLE_WIFI_PORTAL ? "enabled" : "disabled",
           DROPBEAR_ENABLE_WIFI_PORTAL ? desiredPortalSSID().c_str() : "n/a");
}

void loop() {
  // All runtime ownership is explicit FreeRTOS tasks.
  vTaskDelay(pdMS_TO_TICKS(1000));
}
