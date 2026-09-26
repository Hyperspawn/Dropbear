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
#include <freertos/queue.h>
#include <math.h>
#include "dropbear_motor_protocol.h"

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
 *   - Uses each AS5600 to establish the corresponding motor-native CAN angle
 *     reference after restart, then uses fresh RMD angle feedback for control.
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

static const char *DROPBEAR_FIRMWARE_VERSION =
  "behemoth-observation-protocol-2026.09.41";
static const char *DROPBEAR_CAPABILITY_SCHEMA = "DBV1";
static const char *DROPBEAR_COMMAND_PROTOCOL = "DB1";
static const char *DROPBEAR_TELEMETRY_PROTOCOL = "DB3";
static const char *DROPBEAR_CAPABILITIES =
  "version-v1;health-v1;observe-stream-v1;db1-required;"
  "motor-profile-v1;motor-angle-rmd-v17-v42-0x92;"
  "motor-control-aligned-v1;as5600-crosscheck-v1;"
  "boot-observability-v1;portal-safety-v1;can-read-passthrough-v1;"
  "can-discovered-read-v1;"
  "can-discovery-v1;can-bus-recovery-v1;can-transaction-scheduler-v1;"
  "can-oneshot-tx-v1;"
  "can-deferred-tx-abort-v1;can-interrupt-rx-v1;"
  "can-timing-observability-v1;can-wire-tx-status-v1;"
  "can-overflow-recovery-v1;can-poll-control-v1;"
  "can-register-snapshot-v1;can-rtos-deadline-v1;"
  "can-nonblocking-diagnostic-queue-v1;can-single-mailbox-v1;"
  "can-offline-backoff-v1;can-bounded-records-v1;"
  "can-single-sample-1mbps-v1;"
  "config-records-v1;calibration-result-v1";

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

// Arduino 1.8 generates prototypes before later sketch declarations. Keep this
// incomplete declaration above the first function so its generated prototype
// for loadJointConstraintsFromFile() is valid; the full definition follows in
// Shared state.
struct JointConstraints;
enum CommandTarget : uint8_t;

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

// Explicit installed-motor profiles keep wire layout, physical reduction and
// command opcodes out of control logic. RMD 0x92 reports output-shaft angle on
// both installed families, so the 9:1 and 7:1 reductions are documented but
// intentionally do not divide the reported joint angle. A future motor-shaft
// codec can select ANGLE_REFERENCE_MOTOR_SHAFT and reuse the same conversion.
static const dropbear::MotorProfile MOTOR_PROFILE_X8_V17 = {
  "MyActuator RMD-X8 Pro 1:9", "V1.7", 9.0f,
  dropbear::ANGLE_SIGNED_56_LE_BYTES_1_TO_7,
  dropbear::ANGLE_REFERENCE_OUTPUT_SHAFT,
  0.01f, 0x92, 0xA1, 0x81
};
static const dropbear::MotorProfile MOTOR_PROFILE_X10_V42 = {
  "MyActuator RMD-X10 1:7", "V4.2+", 7.0f,
  dropbear::ANGLE_SIGNED_32_LE_BYTES_4_TO_7,
  dropbear::ANGLE_REFERENCE_OUTPUT_SHAFT,
  0.01f, 0x92, 0xA1, 0x81
};

static const dropbear::MotorProfile *const ACTUATOR_MOTOR_PROFILES[ACTUATOR_COUNT] = {
  &MOTOR_PROFILE_X8_V17,  &MOTOR_PROFILE_X8_V17,  // outer calves
  &MOTOR_PROFILE_X8_V17,  &MOTOR_PROFILE_X8_V17,  // inner calves
  &MOTOR_PROFILE_X10_V42, &MOTOR_PROFILE_X10_V42, // knees
  &MOTOR_PROFILE_X10_V42, &MOTOR_PROFILE_X10_V42, // hip pitch
  &MOTOR_PROFILE_X10_V42, &MOTOR_PROFILE_X10_V42, // hip yaw
  &MOTOR_PROFILE_X10_V42, &MOTOR_PROFILE_X10_V42  // hip roll
};

const dropbear::MotorProfile *motorProfileForActuator(int actuatorIndex) {
  if (actuatorIndex < 0 || actuatorIndex >= ACTUATOR_COUNT) return nullptr;
  return ACTUATOR_MOTOR_PROFILES[actuatorIndex];
}

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

// Read-only RMD 0x92 multi-turn angle polling is decoded through the installed
// motor profile for each CAN ID. One motor is queried per slot to avoid a
// six-frame burst. Replies are emitted beside the five
// external AS5600 angles in the versioned DB3 serial record. This request
// cannot command motion, but it does add bounded CAN traffic.
static const bool MOTOR_NATIVE_FEEDBACK_ENABLED = true;
// The installed MCP2515 modules are documented as 8 MHz while the actuator bus
// is fixed at 1 Mbit/s. Microchip requires PS2 >= 2 TQ; even the single-sample
// CNF1/CNF2/CNF3 = 00/80/80 override below has PS2 = 1 TQ. Keep the deployed
// bitrate for compatibility with the motors, but report this hardware timing
// limitation explicitly. A 16 MHz MCP2515 clock is the compliant hardware fix.
static const uint32_t CAN_CONTROLLER_CLOCK_HZ = 8000000;
static const uint32_t CAN_BUS_BITRATE_BPS = 1000000;
static const bool CAN_BIT_TIMING_DATASHEET_COMPLIANT = false;
static const uint8_t CAN_CNF1_8MHZ_1MBPS = 0x00;
static const uint8_t CAN_CNF2_8MHZ_1MBPS_SINGLE_SAMPLE = 0x80;
static const uint8_t CAN_CNF3_8MHZ_1MBPS = 0x80;
static const uint32_t MOTOR_NATIVE_QUERY_SLOT_MS = 12;
static const uint32_t MOTOR_NATIVE_QUERY_BACKOFF_MS = 50;
static const uint32_t MOTOR_NATIVE_REPLY_TIMEOUT_MS = 10;
static const uint8_t MOTOR_NATIVE_OFFLINE_AFTER_MISSES = 3;
// A missing node costs eight TEC counts per no-ACK attempt. Re-probing every
// 500 ms can keep an otherwise healthy multi-node controller error-passive,
// especially when two configured addresses are absent. Five seconds still
// discovers a repaired connection promptly without poisoning healthy reads.
static const uint32_t MOTOR_NATIVE_OFFLINE_RETRY_MS = 5000;
static const uint32_t MOTOR_NATIVE_STALE_MS = 750;
static const uint8_t CAN_RX_BURST_LIMIT = 12;
static const uint32_t CAN_DEFERRED_TX_ABORT_MS = 12;
static const uint32_t CAN_TX_ABORT_TIMEOUT_US = 2000;
static const uint32_t CAN_TX_COMPLETION_TIMEOUT_US = 3500;
static const uint32_t CAN_TX_QUEUE_DEADLINE_US = 8000;
static const uint32_t CAN_TX_CALLER_TIMEOUT_MS = 12;
static const uint8_t CAN_TX_QUEUE_DEPTH = 12;
static const uint32_t CAN_OUTPUT_PERIOD_US = 10000;
static const uint32_t CAN_IO_LOOP_BUDGET_US = 5000;
static const uint8_t CAN_DIAGNOSTIC_LINE_QUEUE_DEPTH = 16;
static const size_t CAN_DIAGNOSTIC_LINE_MAX = 384;
static const uint32_t CAN_DIAGNOSTIC_DEFAULT_CAPTURE_MS = 1000;
static const uint32_t CAN_DIAGNOSTIC_MAX_CAPTURE_MS = 10000;
static const uint32_t CAN_DIAGNOSTIC_SNIFF_MAX_MS = 2000;
static const uint16_t CAN_DIAGNOSTIC_SNIFF_FRAME_LIMIT = 128;
static const uint32_t RMD_DISCOVERY_FIRST_ID = 0x141;
static const uint32_t RMD_DISCOVERY_LAST_ID = 0x160;
static const uint32_t RMD_DISCOVERY_REPLY_WAIT_MS = 40;
static const uint32_t CAN_ERROR_SAMPLE_MS = 250;
// A persistent wiring/bitrate fault must not turn recovery into a one-second
// reset/log storm. Five seconds still recovers promptly after a cable repair.
static const uint32_t CAN_RECOVERY_INTERVAL_MS = 5000;
static const uint32_t CAN_RECOVERY_FAILURE_THRESHOLD = 8;
static const uint32_t CAN_RECOVERY_RX_QUIET_MS = 500;
static const uint8_t MOTOR_BOOT_ZERO_SAMPLE_COUNT = 8;
static const float MOTOR_BOOT_ZERO_MAX_SPREAD_DEG = 2.0f;
static const float MOTOR_AS5600_DIVERGENCE_LIMIT_DEG = 12.0f;
static const uint8_t MOTOR_AS5600_DIVERGENCE_SAMPLE_COUNT = 3;
static const uint8_t RIGHT_MOTOR_TELEMETRY_ORDER[6] = {
  RIGHT_OUTER_CALF, RIGHT_INNER_CALF, RIGHT_HIP_PITCH,
  RIGHT_KNEE, RIGHT_HIP_YAW, RIGHT_HIP_ROLL
};
static const uint8_t LEFT_MOTOR_TELEMETRY_ORDER[6] = {
  LEFT_OUTER_CALF, LEFT_INNER_CALF, LEFT_HIP_PITCH,
  LEFT_KNEE, LEFT_HIP_YAW, LEFT_HIP_ROLL
};

volatile float motorNativeDegrees[ACTUATOR_COUNT] = {0.0f};
volatile uint32_t motorNativeReceivedMs[ACTUATOR_COUNT] = {0};
volatile bool motorNativeValid[ACTUATOR_COUNT] = {false};
// Five AS5600-equipped axes are aligned once after boot. The resulting offset
// maps the RMD multi-turn angle into the calibrated joint coordinate. AS5600
// remains an independent diagnostic after that point and never replaces stale
// CAN feedback in the active controller.
volatile bool motorControlZeroed[ACTUATOR_COUNT] = {false};
volatile bool motorControlAlignmentFault[ACTUATOR_COUNT] = {false};
volatile float motorControlZeroOffsetDegrees[ACTUATOR_COUNT] = {0.0f};
volatile float motorControlDegrees[ACTUATOR_COUNT] = {0.0f};
volatile float motorAs5600ErrorDegrees[ACTUATOR_COUNT] = {0.0f};
volatile uint8_t motorBootZeroSamples[ACTUATOR_COUNT] = {0};
volatile uint8_t motorAs5600DivergenceSamples[ACTUATOR_COUNT] = {0};
float motorBootZeroOffsetSum[ACTUATOR_COUNT] = {0.0f};
float motorBootZeroOffsetMin[ACTUATOR_COUNT] = {0.0f};
float motorBootZeroOffsetMax[ACTUATOR_COUNT] = {0.0f};
volatile uint32_t motorNativeQueries = 0;
volatile uint32_t motorNativeQueryFailures = 0;
volatile uint32_t motorNativeResponses = 0;
volatile uint32_t motorNativeMalformedResponses = 0;
// Exactly one automatic 0x92 transaction may be outstanding. This prevents a
// delayed reply or a congested MCP2515 TX mailbox from being compounded by the
// next polling slot. Diagnostic reads pause this scheduler independently.
volatile int8_t motorNativePendingIndex = -1;
volatile uint32_t motorNativePendingSinceMs = 0;
volatile uint32_t motorNativeReplyTimeouts = 0;
volatile uint32_t motorNativeOutOfOrderResponses = 0;
volatile uint32_t motorNativeOfflineSkips = 0;
volatile uint8_t motorNativeMissStreak[ACTUATOR_COUNT] = {0};
volatile uint32_t motorNativeNextEligibleMs[ACTUATOR_COUNT] = {0};
// MCP_CAN_lib returns CAN_SENDMSGTIMEOUT after 2.5 ms while TXREQ can remain
// loaded. One-shot mode prevents retries only after a first transmit attempt;
// it does not cancel a frame that is still waiting for an idle bus. Track that
// ownership explicitly so timeout cleanup occurs before another read is sent.
volatile bool canDeferredTxPending = false;
volatile uint32_t canDeferredTxSinceMs = 0;
volatile bool canDeferredTxAbortAttempted = false;
// This switch controls only automatic 0x92 polling. It never arms motion and
// lets diagnostics prove whether controller errors occur on an otherwise idle
// bus. Diagnostic transactions pause and restore it explicitly.
volatile bool motorNativePollingEnabled = true;

struct CanTxRequest {
  uint32_t id;
  byte data[8];
  byte length;
  bool allowDeferredRead;
  bool highPriority;
  uint32_t token;
  uint32_t enqueuedUs;
  uint32_t deadlineUs;
  TaskHandle_t requester;
};

struct CanDiagnosticLineRecord {
  char text[CAN_DIAGNOSTIC_LINE_MAX];
};

QueueHandle_t canTxQueue = nullptr;
QueueHandle_t canDiagnosticLineQueue = nullptr;
volatile uint32_t canTxQueueToken = 0;
volatile uint32_t canTxQueueAccepted = 0;
volatile uint32_t canTxQueueCompleted = 0;
volatile uint32_t canTxQueueFull = 0;
volatile uint32_t canTxQueueExpired = 0;
volatile uint32_t canTxCallerTimeouts = 0;
volatile uint32_t canTxQueueLatencyLastUs = 0;
volatile uint32_t canTxQueueLatencyMaxUs = 0;
volatile uint32_t canTxExecutionLastUs = 0;
volatile uint32_t canTxExecutionMaxUs = 0;
volatile uint32_t canDiagnosticLinesQueued = 0;
volatile uint32_t canDiagnosticLinesDrained = 0;
volatile uint32_t canDiagnosticLineDrops = 0;

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

// The debug bridge is deliberately read-only.  It captures one configured
// motor at a time and recognizes both legacy same-ID and newer ID+0x100 reply
// conventions without changing the stricter closed-loop feedback decoder.
volatile bool canDiagnosticCaptureActive = false;
volatile bool canDiagnosticCaptureAll = false;
volatile bool canDiagnosticScanActive = false;
volatile uint32_t canDiagnosticRequestId = 0;
volatile uint32_t canDiagnosticCaptureUntilMs = 0;
volatile uint16_t canDiagnosticCaptureFramesRemaining = 0;
volatile uint32_t canDiagnosticRxFrames = 0;
volatile uint32_t canDiagnosticSerialDrops = 0;
volatile uint32_t canDiagnosticScanFoundMask = 0;
volatile uint32_t canDiagnosticScanSameIdMask = 0;
volatile uint32_t canDiagnosticScanOffsetIdMask = 0;
volatile uint16_t canDiagnosticScanResponses = 0;
volatile uint16_t canDiagnosticScanTxFailures = 0;

float maxTorqueLimit = 3.0f;

bool isLeft = true;
bool isCenter = false;
bool isHead = false;
bool rawMode = false;
bool playMode = false;
// Observation is independent of actuator play. `observe on` enables only
// serial telemetry; it never arms torque output or changes playMode.
volatile bool telemetryStreamingEnabled = false;
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
// The SPI mutex serializes MCP2515 register access. This short critical lock
// separately makes counters/state transitions atomic across the RX, output,
// diagnostic, and HyperSpawn tasks after SPI ownership has been released.
portMUX_TYPE canStateMux = portMUX_INITIALIZER_UNLOCKED;

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
volatile bool runtimeInitializationComplete = false;

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
volatile uint32_t canRxInterrupts = 0;
volatile uint32_t canRxWakeups = 0;
volatile uint8_t canRxMaxBurst = 0;

void IRAM_ATTR onCanInterrupt() {
  canRxInterrupts++;
  if (canRxTaskHandle == nullptr) return;
  BaseType_t higherPriorityTaskWoken = pdFALSE;
  vTaskNotifyGiveFromISR(canRxTaskHandle, &higherPriorityTaskWoken);
  if (higherPriorityTaskWoken == pdTRUE) portYIELD_FROM_ISR();
}

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
static const uint32_t PORTAL_MOTION_LEASE_MS = 90000;
static const uint32_t PORTAL_TORQUE_TEST_MAX_MS = 500;
static const int16_t PORTAL_TORQUE_TEST_MAX_COMMAND = 25;

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
volatile uint8_t portalSafetyStage = 0;
volatile uint32_t portalMotionLeaseUntilMs = 0;
volatile uint32_t portalSafetyUnlocks = 0;
volatile uint32_t portalSafetyRejects = 0;
volatile bool portalTorqueTestActive = false;
volatile int portalTorqueTestActuatorIndex = -1;
volatile int16_t portalTorqueTestValue = 0;
volatile uint32_t portalTorqueTestUntilMs = 0;

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
volatile bool canOneShotEnabled = false;
volatile bool portalOnline = false;

volatile uint32_t spiffsReadOps = 0;
volatile uint32_t spiffsWriteOps = 0;
volatile uint32_t spiffsErrors = 0;
volatile uint32_t lastSpiffsReadMs = 0;
volatile uint32_t lastSpiffsWriteMs = 0;

volatile uint32_t canTxSuccess = 0;
volatile uint32_t canTxFailure = 0;
volatile uint32_t canDeferredReadSubmissions = 0;
volatile uint32_t canConsecutiveFailures = 0;
volatile uint32_t canMutexTimeouts = 0;
volatile uint32_t canTorqueFrames = 0;
volatile uint32_t canStopFrames = 0;
volatile uint32_t lastCanTxMs = 0;
volatile uint32_t lastCanFailureMs = 0;
volatile int lastCanResult = 0;
volatile uint8_t canErrorFlags = 0;
volatile uint8_t canTxErrorCount = 0;
volatile uint8_t canRxErrorCount = 0;
volatile uint32_t canRecoveryAttempts = 0;
volatile uint32_t canRecoverySuccesses = 0;
volatile uint32_t lastCanRecoveryMs = 0;
volatile uint32_t canTxAbortAttempts = 0;
volatile uint32_t canTxAbortSuccesses = 0;
volatile uint32_t canTxAbortFailures = 0;
volatile uint32_t lastCanTxAbortReportMs = 0;
volatile uint32_t canMotionFailClosed = 0;
volatile uint32_t canTxWireErrors = 0;
volatile uint32_t canTxArbitrationLosses = 0;
volatile uint32_t canTxControllerAborts = 0;
volatile uint32_t canRxOverflowEvents = 0;
volatile uint32_t canRxOverflowClears = 0;
volatile uint32_t canOutputBatchLastUs = 0;
volatile uint32_t canOutputBatchMaxUs = 0;
volatile uint32_t canOutputDeadlineMisses = 0;
volatile uint32_t canStopBatchFailures = 0;
volatile uint32_t canIoLoopLastUs = 0;
volatile uint32_t canIoLoopMaxUs = 0;
volatile uint32_t canIoLoopBudgetMisses = 0;
volatile uint32_t canRxDrainLastUs = 0;
volatile uint32_t canRxDrainMaxUs = 0;
volatile uint8_t canControllerMode = MODE_CONFIG;
volatile uint8_t canControlRegister = 0;
volatile uint8_t canInterruptFlags = 0;
volatile uint8_t canTxBufferControl[3] = {0, 0, 0};
volatile uint8_t canBitTimingRegisters[3] = {0, 0, 0};
volatile uint8_t canInterruptEnable = 0;
volatile uint8_t canRxBufferControl[2] = {0, 0};

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
String commandTargetName(CommandTarget target);
CommandTarget currentCommandTarget();
CommandTarget parseCommandTarget(String token);
bool parseDB1Envelope(const String &rawCommand, CommandTarget &target,
                      String &payload, bool &usedLegacy);
void requestPortalRestart();
void lockPortalMotion(bool stopOutputs, const char *reason);
bool portalMotionAuthorized();
bool portalPayloadRequiresMotionUnlock(String payload);
void appendWebLog(const String &line);
void dbPrintln(const String &line);
void dbPrintf(const char *format, ...);
void setupPortal();
bool canSendFrame(uint32_t actuatorID, const byte *data, byte dataLen);
int canSendFrameResult(uint32_t actuatorID, const byte *data, byte dataLen,
                       bool allowDeferredRead);
void sampleCanControllerErrors();
bool recoverCanController();
bool abortPendingCanTx(const char *reason);
bool clearCanRxOverflowFlags();
bool quiesceMotorNativePolling();
void restoreMotorNativePolling(bool wasEnabled);
void IRAM_ATTR onCanInterrupt();
void printCanBusStatus();
void printCanRegisterSnapshot();
bool processCanDiagnosticCommand(String command);
void captureCanDiagnosticFrame(uint32_t responseId, const byte *data, byte len);
void portalTask(void *parameter);
const char *sensorHealthStatus(const SensorDiagnostic &d);
void markHyperspawnCommand(HyperspawnControlMode mode);
void executeQueuedWebCommand(const WebCommand &item);
void saveJointConstraintsToFile(File &file, JointConstraints constraints,
                                const char *jointName);
JointConstraints loadJointConstraintsFromFile(
  String line, JointConstraints defaultConstraints);
String constraintJson(const JointConstraints &c);
const JointConstraints &selectedConstraintForSensor(int index);
void parseConstraintArg(const char *name, JointConstraints &c);
void handleConfigurationCommand(String command);
void processPayloadCommand(String command, const char *source = "serial");
void processRoutedCommand(String command, const char *source = "serial");
void readIMU();
void saveConfig();
void printHelp();
void printVersionRecord();
void printHealthRecord();
void printConfigurationRecords();
bool processConfigurationSetCommand(const String &command);
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

const char *commandTargetCString(CommandTarget target) {
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

String commandTargetName(CommandTarget target) {
  return String(commandTargetCString(target));
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

const uint8_t *selectedMotorTelemetryOrder() {
  return isLeft ? LEFT_MOTOR_TELEMETRY_ORDER : RIGHT_MOTOR_TELEMETRY_ORDER;
}

int actuatorIndexFromMotorFeedbackId(uint32_t responseID) {
  // Installed V1.7 X8 calves answer on the request ID itself, while the newer
  // X10 firmware answers on request+0x100. The MCP2515 in normal mode does not
  // enqueue its own transmitted request, and the profile check prevents a
  // direct-ID X10 frame from being admitted as feedback.
  const int directIndex = actuatorIndexFromCanId(responseID);
  if (actuatorBelongsToSelectedLeg(directIndex) &&
      motorProfileForActuator(directIndex) == &MOTOR_PROFILE_X8_V17) {
    return directIndex;
  }
  if (responseID < 0x100) return -1;
  const int offsetIndex = actuatorIndexFromCanId(responseID - 0x100);
  return actuatorBelongsToSelectedLeg(offsetIndex) ? offsetIndex : -1;
}

int as5600SensorIndexForActuator(int actuatorIndex) {
  switch (actuatorIndex) {
    case RIGHT_OUTER_CALF:
    case LEFT_OUTER_CALF: return 0;
    case RIGHT_INNER_CALF:
    case LEFT_INNER_CALF: return 1;
    case RIGHT_HIP_PITCH:
    case LEFT_HIP_PITCH: return 2;
    case RIGHT_KNEE:
    case LEFT_KNEE: return 3;
    case RIGHT_HIP_ROLL:
    case LEFT_HIP_ROLL: return 4;
    default: return -1;
  }
}

float motorFeedbackDirectionForActuator(int actuatorIndex) {
  // Position scale is 1:1. Sign follows the same persisted mechanical mapping
  // already verified for positive joint torque on each actuator.
  switch (actuatorIndex) {
    case RIGHT_OUTER_CALF: return directionMultiplierRightOuterCalf;
    case LEFT_OUTER_CALF: return directionMultiplierLeftOuterCalf;
    case RIGHT_INNER_CALF: return directionMultiplierRightInnerCalf;
    case LEFT_INNER_CALF: return directionMultiplierLeftInnerCalf;
    case RIGHT_KNEE: return directionMultiplierRightKnee;
    case LEFT_KNEE: return directionMultiplierLeftKnee;
    case RIGHT_HIP_PITCH: return directionMultiplierRightHipPitch;
    case LEFT_HIP_PITCH: return directionMultiplierLeftHipPitch;
    case RIGHT_HIP_ROLL: return directionMultiplierRightHipRoll;
    case LEFT_HIP_ROLL: return directionMultiplierLeftHipRoll;
    default: return 1.0f;
  }
}

float normalizedAs5600Degrees(uint8_t sensorIndex) {
  switch (sensorIndex) {
    case 0: return normalizedOuter;
    case 1: return normalizedInner;
    case 2: return normalizedHip;
    case 3: return normalizedKnee;
    case 4: return normalizedButt;
    default: return 0.0f;
  }
}

bool readFreshAs5600Reference(int actuatorIndex, float &degrees) {
  const int sensorIndex = as5600SensorIndexForActuator(actuatorIndex);
  if (sensorIndex < 0) return false;
  const SensorDiagnostic &diagnostic = sensorDiagnostics[sensorIndex];
  const uint32_t age = diagnosticAgeMs(diagnostic.lastSampleMs);
  if (!diagnostic.signalValid || diagnostic.pulseAgeUs > AS5600_STALE_US ||
      age == UINT32_MAX || age > 100) return false;
  degrees = normalizedAs5600Degrees(static_cast<uint8_t>(sensorIndex));
  return isfinite(degrees);
}

float shortestAngleDifference(float value, float reference) {
  float difference = fmodf(value - reference + 540.0f, 360.0f) - 180.0f;
  return difference;
}

void resetMotorBootZeroAccumulator(int actuatorIndex) {
  motorBootZeroSamples[actuatorIndex] = 0;
  motorBootZeroOffsetSum[actuatorIndex] = 0.0f;
  motorBootZeroOffsetMin[actuatorIndex] = 0.0f;
  motorBootZeroOffsetMax[actuatorIndex] = 0.0f;
}

void updateMotorControlReference(int actuatorIndex, float nativeDegrees) {
  const int sensorIndex = as5600SensorIndexForActuator(actuatorIndex);
  if (motorControlAlignmentFault[actuatorIndex]) return;

  const float directedNative = nativeDegrees * motorFeedbackDirectionForActuator(actuatorIndex);
  if (sensorIndex < 0) {
    // Hip yaw has no AS5600. Establish a boot-relative zero from the first
    // verified RMD 0x92 response, then preserve the continuous motor delta.
    if (!motorControlZeroed[actuatorIndex]) {
      motorControlZeroOffsetDegrees[actuatorIndex] = -directedNative;
      motorControlZeroed[actuatorIndex] = true;
    }
    motorControlDegrees[actuatorIndex] =
      directedNative + motorControlZeroOffsetDegrees[actuatorIndex];
    return;
  }

  float externalDegrees = 0.0f;
  if (!readFreshAs5600Reference(actuatorIndex, externalDegrees)) return;

  if (motorControlZeroed[actuatorIndex]) {
    const float aligned = directedNative + motorControlZeroOffsetDegrees[actuatorIndex];
    motorControlDegrees[actuatorIndex] = aligned;
    const float disagreement = fabsf(shortestAngleDifference(aligned, externalDegrees));
    motorAs5600ErrorDegrees[actuatorIndex] = disagreement;
    if (disagreement > MOTOR_AS5600_DIVERGENCE_LIMIT_DEG) {
      if (motorAs5600DivergenceSamples[actuatorIndex] < UINT8_MAX) {
        motorAs5600DivergenceSamples[actuatorIndex]++;
      }
      if (motorAs5600DivergenceSamples[actuatorIndex] >=
          MOTOR_AS5600_DIVERGENCE_SAMPLE_COUNT) {
        // Latch until restart. Re-zeroing while the leg may be moving would
        // create a discontinuous position reference.
        motorControlAlignmentFault[actuatorIndex] = true;
        motorControlZeroed[actuatorIndex] = false;
        impedanceTorqueValues[actuatorIndex] = 0;
      }
    } else {
      motorAs5600DivergenceSamples[actuatorIndex] = 0;
    }
    return;
  }

  float candidateOffset = externalDegrees - directedNative;
  const uint8_t samples = motorBootZeroSamples[actuatorIndex];
  if (samples > 0) {
    const float referenceOffset = motorBootZeroOffsetSum[actuatorIndex] /
                                  static_cast<float>(samples);
    candidateOffset = referenceOffset +
      shortestAngleDifference(candidateOffset, referenceOffset);
  }

  if (samples == 0) {
    motorBootZeroOffsetMin[actuatorIndex] = candidateOffset;
    motorBootZeroOffsetMax[actuatorIndex] = candidateOffset;
  } else {
    motorBootZeroOffsetMin[actuatorIndex] = min(motorBootZeroOffsetMin[actuatorIndex], candidateOffset);
    motorBootZeroOffsetMax[actuatorIndex] = max(motorBootZeroOffsetMax[actuatorIndex], candidateOffset);
  }
  motorBootZeroOffsetSum[actuatorIndex] += candidateOffset;
  motorBootZeroSamples[actuatorIndex] = samples + 1;

  if (motorBootZeroSamples[actuatorIndex] < MOTOR_BOOT_ZERO_SAMPLE_COUNT) return;

  const float spread = motorBootZeroOffsetMax[actuatorIndex] -
                       motorBootZeroOffsetMin[actuatorIndex];
  if (spread > MOTOR_BOOT_ZERO_MAX_SPREAD_DEG) {
    resetMotorBootZeroAccumulator(actuatorIndex);
    return;
  }

  const float zeroOffset = motorBootZeroOffsetSum[actuatorIndex] /
                           static_cast<float>(motorBootZeroSamples[actuatorIndex]);
  motorControlZeroOffsetDegrees[actuatorIndex] = zeroOffset;
  motorControlDegrees[actuatorIndex] = directedNative + zeroOffset;
  motorAs5600ErrorDegrees[actuatorIndex] = fabsf(
    shortestAngleDifference(motorControlDegrees[actuatorIndex], externalDegrees));
  motorAs5600DivergenceSamples[actuatorIndex] = 0;
  motorControlZeroed[actuatorIndex] = true;
}

bool readMotorControlDegrees(int actuatorIndex, float &degrees) {
  if (actuatorIndex < 0 || actuatorIndex >= ACTUATOR_COUNT ||
      !motorControlZeroed[actuatorIndex] || motorControlAlignmentFault[actuatorIndex] ||
      !motorNativeValid[actuatorIndex] ||
      millis() - motorNativeReceivedMs[actuatorIndex] > MOTOR_NATIVE_STALE_MS) return false;
  degrees = motorNativeDegrees[actuatorIndex] * motorFeedbackDirectionForActuator(actuatorIndex) +
            motorControlZeroOffsetDegrees[actuatorIndex];
  motorControlDegrees[actuatorIndex] = degrees;
  return isfinite(degrees);
}

bool ingestMotorNativeFeedback(uint32_t responseID, const byte *data, byte len) {
  if (!MOTOR_NATIVE_FEEDBACK_ENABLED) return false;
  const int index = actuatorIndexFromMotorFeedbackId(responseID);
  // Shared CAN buses can carry HyperSpawn traffic, motor replies for the other
  // leg, and vendor frames for commands other than 0x92. Those are not
  // malformed angle replies and must remain available to the next decoder.
  if (index < 0) return false;

  const dropbear::MotorProfile *profile = motorProfileForActuator(index);
  if (profile == nullptr || data == nullptr || len == 0 ||
      data[0] != profile->readMultiTurnOpcode) return false;
  double outputShaftDegrees = 0.0;
  const dropbear::DecodeStatus decodeStatus =
    dropbear::decodeMultiTurnAngle(*profile, data, len, &outputShaftDegrees);
  if (decodeStatus != dropbear::DECODE_OK) {
    motorNativeMalformedResponses++;
    return false;
  }
  const float decodedDegrees = static_cast<float>(outputShaftDegrees);
  if (!isfinite(decodedDegrees)) {
    motorNativeMalformedResponses++;
    return false;
  }
  motorNativeDegrees[index] = decodedDegrees;
  motorNativeReceivedMs[index] = millis();
  motorNativeValid[index] = true;
  if (motorNativePendingIndex == index) {
    motorNativePendingIndex = -1;
    motorNativePendingSinceMs = 0;
    canDeferredTxPending = false;
    canDeferredTxSinceMs = 0;
    canDeferredTxAbortAttempted = false;
    motorNativeMissStreak[index] = 0;
    motorNativeNextEligibleMs[index] = 0;
  } else if (motorNativePendingIndex >= 0) {
    // Keep valid late feedback, but never let it complete a different motor's
    // transaction. The outstanding request must receive its own reply or time
    // out before the scheduler advances.
    motorNativeOutOfOrderResponses++;
  }
  updateMotorControlReference(index, motorNativeDegrees[index]);
  motorNativeResponses++;
  return true;
}

void recordMotorNativeMiss(uint8_t actuatorIndex, uint32_t now,
                           bool replyTimeout) {
  if (actuatorIndex >= ACTUATOR_COUNT) return;
  if (motorNativeMissStreak[actuatorIndex] < UINT8_MAX) {
    motorNativeMissStreak[actuatorIndex]++;
  }
  if (replyTimeout) {
    motorNativeReplyTimeouts++;
    motorNativeQueryFailures++;
  }
  const uint32_t retryDelay =
    motorNativeMissStreak[actuatorIndex] >= MOTOR_NATIVE_OFFLINE_AFTER_MISSES
      ? MOTOR_NATIVE_OFFLINE_RETRY_MS
      : MOTOR_NATIVE_QUERY_BACKOFF_MS;
  motorNativeNextEligibleMs[actuatorIndex] = now + retryDelay;
}

bool requestMotorNativeFeedback(uint8_t actuatorIndex) {
  if (!MOTOR_NATIVE_FEEDBACK_ENABLED || !runtimeControlReady ||
      !canInitialized || !actuatorBelongsToSelectedLeg(actuatorIndex) ||
      motorNativePendingIndex >= 0 || canDeferredTxPending) return false;
  const dropbear::MotorProfile *profile = motorProfileForActuator(actuatorIndex);
  if (profile == nullptr) {
    motorNativeQueryFailures++;
    return false;
  }
  byte request[8];
  dropbear::encodeReadMultiTurnAngle(*profile, request);
  const int result = canSendFrameResult(ACTUATOR_IDS[actuatorIndex], request, 8, true);
  // A SEND_MSG_TIMEOUT can leave the owned mailbox loaded even though the
  // frame later wins arbitration and receives a valid motor reply. Keep that
  // transaction pending. A busy mailbox means stale ownership and is aborted
  // before the scheduler is allowed to probe another node.
  if (result != CAN_OK && result != CAN_SENDMSGTIMEOUT) {
    if (result == CAN_GETTXBFTIMEOUT) {
      abortPendingCanTx("read_mailbox_busy");
    }
    motorNativeQueryFailures++;
    return false;
  }
  motorNativePendingIndex = static_cast<int8_t>(actuatorIndex);
  motorNativePendingSinceMs = millis();
  motorNativeQueries++;
  return true;
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

void updateMotorReferencedImpedance(int actuatorIndex, ImpedanceControl &controller,
                                    const JointConstraints &constraints,
                                    float outputDirection, bool enabled,
                                    unsigned long now);

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

bool isReadOnlyCanDiagnosticOpcode(uint8_t opcode) {
  switch (opcode) {
    case 0x30:  // read PID parameters
    case 0x42:  // read acceleration
    case 0x90:  // read encoder position
    case 0x92:  // read multi-turn angle
    case 0x94:  // read single-turn angle
    case 0x9A:  // read status 1 / fault state
    case 0x9C:  // read status 2 / live state
    case 0x9D:  // read status 3 / phase currents
      return true;
    default:
      return false;
  }
}

bool isExpectedCanDiagnosticReplyId(uint32_t requestId, uint32_t responseId) {
  if (requestId > 0x7FFU || responseId > 0x7FFU) return false;
  return responseId == requestId ||
         (requestId <= 0x6FFU && responseId == requestId + 0x100U);
}

bool nextCanCommandToken(const String &input, int &cursor, String &token) {
  while (cursor < static_cast<int>(input.length()) &&
         (input.charAt(cursor) == ' ' || input.charAt(cursor) == '\t')) cursor++;
  if (cursor >= static_cast<int>(input.length())) {
    token = "";
    return false;
  }
  const int start = cursor;
  while (cursor < static_cast<int>(input.length()) &&
         input.charAt(cursor) != ' ' && input.charAt(cursor) != '\t') cursor++;
  token = input.substring(start, cursor);
  return true;
}

bool parseCanHexToken(String token, uint32_t maximum, uint32_t &value) {
  token.trim();
  if (token.startsWith("0x") || token.startsWith("0X")) token = token.substring(2);
  if (token.length() == 0 || token.length() > 8) return false;
  for (uint16_t i = 0; i < token.length(); ++i) {
    const char c = token.charAt(i);
    const bool hex = (c >= '0' && c <= '9') ||
                     (c >= 'a' && c <= 'f') ||
                     (c >= 'A' && c <= 'F');
    if (!hex) return false;
  }
  char *end = nullptr;
  const unsigned long parsed = strtoul(token.c_str(), &end, 16);
  if (end == token.c_str() || *end != '\0' || parsed > maximum) return false;
  value = static_cast<uint32_t>(parsed);
  return true;
}

bool parseCanDurationToken(String token, uint32_t &value) {
  token.trim();
  if (token.length() == 0 || token.length() > 5) return false;
  for (uint16_t i = 0; i < token.length(); ++i) {
    if (token.charAt(i) < '0' || token.charAt(i) > '9') return false;
  }
  char *end = nullptr;
  const unsigned long parsed = strtoul(token.c_str(), &end, 10);
  if (end == token.c_str() || *end != '\0' || parsed < 100 ||
      parsed > CAN_DIAGNOSTIC_MAX_CAPTURE_MS) return false;
  value = static_cast<uint32_t>(parsed);
  return true;
}

bool selectedDiagnosticMotorId(uint32_t requestId) {
  // Passive discovery is specifically used to find motors whose stored CAN ID
  // no longer matches this controller's configured leg map.  Permit read-only
  // diagnostic opcodes across the bounded discovery range on the local bus.
  // Motion still routes exclusively through actuatorBelongsToSelectedLeg().
  return isLegRole() && requestId >= RMD_DISCOVERY_FIRST_ID &&
         requestId <= RMD_DISCOVERY_LAST_ID;
}

void emitCanDiagnosticCStringNow(const char *line, bool countDrop) {
  if (line == nullptr || line[0] == '\0') return;
  if (serialMutex && xSemaphoreTake(serialMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
    Serial.println(line);
    xSemaphoreGive(serialMutex);
    appendWebLog(String(line));
    return;
  }
  if (countDrop) canDiagnosticSerialDrops++;
}

void emitCanDiagnosticCString(const char *line, bool countDrop = false) {
  if (line == nullptr || line[0] == '\0') return;
  // Serial and the web-log mutex are not real-time transports. The sole CAN
  // I/O task may only copy a bounded record into a non-blocking queue; core 0
  // formats/drains it later. This prevents diagnostics from delaying RX drain,
  // TX completion, or error recovery on core 1.
  if (canRxTaskHandle != nullptr &&
      xTaskGetCurrentTaskHandle() == canRxTaskHandle) {
    if (canDiagnosticLineQueue != nullptr) {
      CanDiagnosticLineRecord record = {};
      snprintf(record.text, sizeof(record.text), "%s", line);
      if (xQueueSend(canDiagnosticLineQueue, &record, 0) == pdTRUE) {
        canDiagnosticLinesQueued++;
        return;
      }
    }
    canDiagnosticLineDrops++;
    if (countDrop) canDiagnosticSerialDrops++;
    return;
  }
  emitCanDiagnosticCStringNow(line, countDrop);
}

void emitCanDiagnosticLine(const String &line, bool countDrop = false) {
  emitCanDiagnosticCString(line.c_str(), countDrop);
}

void emitCanDiagnosticFrame(const char *direction, uint32_t timestampMs,
                            uint32_t id, const byte *data, byte len,
                            bool countDrop = false) {
  if (direction == nullptr || data == nullptr || len > 8) return;
  char line[192] = {0};
  int used = snprintf(line, sizeof(line), "DBC1,%s,%s,%lu,0x%03lX,%u",
                      commandTargetCString(currentCommandTarget()), direction,
                      static_cast<unsigned long>(timestampMs),
                      static_cast<unsigned long>(id),
                      static_cast<unsigned int>(len));
  if (used < 0) return;
  size_t offset = static_cast<size_t>(used);
  for (byte index = 0; index < len && offset < sizeof(line); ++index) {
    const int appended = snprintf(line + offset, sizeof(line) - offset,
                                  ",%02X", data[index]);
    if (appended < 0 || static_cast<size_t>(appended) >= sizeof(line) - offset) {
      line[sizeof(line) - 1] = '\0';
      break;
    }
    offset += static_cast<size_t>(appended);
  }
  emitCanDiagnosticCString(line, countDrop);
}

void flushDeferredCanDiagnosticLines(uint8_t limit) {
  if (canDiagnosticLineQueue == nullptr) return;
  CanDiagnosticLineRecord record;
  for (uint8_t index = 0; index < limit; ++index) {
    if (xQueueReceive(canDiagnosticLineQueue, &record, 0) != pdTRUE) break;
    canDiagnosticLinesDrained++;
    emitCanDiagnosticCStringNow(record.text, true);
  }
}

String canDiagnosticFrameLine(const char *direction, uint32_t id,
                              const byte *data, byte len) {
  char idText[8];
  snprintf(idText, sizeof(idText), "0x%03lX", static_cast<unsigned long>(id));
  String line = "DBC1," + currentCommandAddress() + "," + String(direction) + "," +
                String(millis()) + "," + String(idText) + "," + String(len);
  for (byte index = 0; index < len && index < 8; ++index) {
    char byteText[4];
    snprintf(byteText, sizeof(byteText), "%02X", data[index]);
    line += "," + String(byteText);
  }
  return line;
}

void armCanDiagnosticCapture(uint32_t requestId, uint32_t durationMs) {
  canDiagnosticCaptureAll = false;
  canDiagnosticRequestId = requestId;
  canDiagnosticCaptureUntilMs = millis() + durationMs;
  canDiagnosticCaptureFramesRemaining = UINT16_MAX;
  canDiagnosticCaptureActive = true;
}

void armCanDiagnosticSniff(uint32_t durationMs) {
  canDiagnosticCaptureAll = true;
  canDiagnosticRequestId = 0;
  canDiagnosticCaptureUntilMs = millis() + durationMs;
  canDiagnosticCaptureFramesRemaining = CAN_DIAGNOSTIC_SNIFF_FRAME_LIMIT;
  canDiagnosticCaptureActive = true;
}

void captureCanDiagnosticFrame(uint32_t responseId, const byte *data, byte len) {
  if (!canDiagnosticCaptureActive || data == nullptr || len > 8) return;
  const uint32_t now = millis();
  if (static_cast<int32_t>(canDiagnosticCaptureUntilMs - now) <= 0) {
    canDiagnosticCaptureActive = false;
    return;
  }
  if (!canDiagnosticCaptureAll &&
      !isExpectedCanDiagnosticReplyId(canDiagnosticRequestId, responseId)) return;
  if (canDiagnosticCaptureFramesRemaining == 0) {
    canDiagnosticCaptureActive = false;
    return;
  }
  if (canDiagnosticScanActive &&
      canDiagnosticRequestId >= RMD_DISCOVERY_FIRST_ID &&
      canDiagnosticRequestId <= RMD_DISCOVERY_LAST_ID) {
    const uint8_t bit = static_cast<uint8_t>(canDiagnosticRequestId -
                                              RMD_DISCOVERY_FIRST_ID);
    const uint32_t mask = 1UL << bit;
    canDiagnosticScanFoundMask |= mask;
    if (responseId == canDiagnosticRequestId) canDiagnosticScanSameIdMask |= mask;
    else canDiagnosticScanOffsetIdMask |= mask;
    canDiagnosticScanResponses++;
  }
  canDiagnosticRxFrames++;
  emitCanDiagnosticFrame("RX", now, responseId, data, len, true);
  if (canDiagnosticCaptureFramesRemaining != UINT16_MAX) {
    canDiagnosticCaptureFramesRemaining--;
    if (canDiagnosticCaptureFramesRemaining == 0) canDiagnosticCaptureActive = false;
  }
}

bool sendReadOnlyDiagnosticFrame(uint32_t requestId, const byte payload[8]) {
  if (!isReadOnlyCanDiagnosticOpcode(payload[0])) {
    emitCanDiagnosticLine("DBC1," + currentCommandAddress() + ",REJECT," +
                          String(millis()) + ",unsafe_opcode");
    return false;
  }
  // Diagnostics pause the automatic poller and own no deferred mailbox. A
  // timeout is completed/aborted by the CAN I/O task before this call returns.
  const int result = canSendFrameResult(requestId, payload, 8, false);
  const bool sent = result == CAN_OK;
  if (canDiagnosticScanActive && !sent) canDiagnosticScanTxFailures++;
  emitCanDiagnosticLine(canDiagnosticFrameLine(sent ? "TX" : "TX_FAIL",
                                               requestId, payload, 8));
  return sent;
}

bool quiesceMotorNativePolling() {
  const bool wasEnabled = motorNativePollingEnabled;
  motorNativePollingEnabled = false;
  // The CAN I/O task owns timeout/abort cleanup. Waiting here lets an existing
  // automatic request close without any command task touching MCP2515 SPI.
  vTaskDelay(pdMS_TO_TICKS(MOTOR_NATIVE_REPLY_TIMEOUT_MS +
                           CAN_DEFERRED_TX_ABORT_MS + 2));
  return wasEnabled;
}

void restoreMotorNativePolling(bool wasEnabled) {
  if (wasEnabled) motorNativePollingEnabled = true;
}

String canDiscoveryIdList(uint32_t mask) {
  if (mask == 0) return "none";
  String ids;
  for (uint8_t bit = 0; bit <= RMD_DISCOVERY_LAST_ID - RMD_DISCOVERY_FIRST_ID; ++bit) {
    if ((mask & (1UL << bit)) == 0) continue;
    char idText[8];
    snprintf(idText, sizeof(idText), "0x%03lX",
             static_cast<unsigned long>(RMD_DISCOVERY_FIRST_ID + bit));
    if (ids.length() > 0) ids += "|";
    ids += idText;
  }
  return ids;
}

void printCanDiagnosticStatus() {
  const uint32_t now = millis();
  const bool active = canDiagnosticCaptureActive &&
    static_cast<int32_t>(canDiagnosticCaptureUntilMs - now) > 0;
  if (!active) canDiagnosticCaptureActive = false;
  char idText[8];
  snprintf(idText, sizeof(idText), "0x%03lX",
           static_cast<unsigned long>(canDiagnosticRequestId));
  const uint32_t remaining = active ? canDiagnosticCaptureUntilMs - now : 0;
  const String captureTarget = canDiagnosticCaptureAll ? String("all") : String(idText);
  emitCanDiagnosticLine("DBC1," + currentCommandAddress() + ",STATUS," +
                        String(now) + "," + (active ? "on" : "off") + "," +
                        captureTarget + "," +
                        String(remaining) + "," +
                        String(canDiagnosticRxFrames) + "," +
                        String(canDiagnosticSerialDrops));
}

bool processCanDiagnosticCommand(String command) {
  command.trim();
  String normalized = command;
  normalized.toLowerCase();
  if (normalized == "can monitor off") {
    canDiagnosticCaptureActive = false;
    canDiagnosticCaptureAll = false;
    printCanDiagnosticStatus();
    return true;
  }
  if (normalized == "can monitor status") {
    printCanDiagnosticStatus();
    return true;
  }
  if (normalized == "can bus") {
    printCanBusStatus();
    return true;
  }
  if (normalized == "can registers") {
    printCanRegisterSnapshot();
    return true;
  }
  if (normalized == "can poll status") {
    emitCanDiagnosticLine("DBC1," + currentCommandAddress() + ",POLL," +
                          String(millis()) + "," +
                          String(motorNativePollingEnabled ? "on" : "off"));
    return true;
  }
  if (normalized == "can poll off") {
    quiesceMotorNativePolling();
    emitCanDiagnosticLine("DBC1," + currentCommandAddress() + ",POLL," +
                          String(millis()) + ",off");
    return true;
  }
  if (normalized == "can poll on") {
    motorNativePollingEnabled = true;
    emitCanDiagnosticLine("DBC1," + currentCommandAddress() + ",POLL," +
                          String(millis()) + ",on");
    return true;
  }
  if (!runtimeControlReady || !canInitialized || !isLegRole()) {
    emitCanDiagnosticLine("DBC1," + currentCommandAddress() + ",REJECT," +
                          String(millis()) + ",can_runtime_not_ready");
    return true;
  }

  if (normalized == "can scan") {
    const bool restorePolling = quiesceMotorNativePolling();
    byte payload[8] = {0x9A, 0, 0, 0, 0, 0, 0, 0};
    canDiagnosticScanFoundMask = 0;
    canDiagnosticScanSameIdMask = 0;
    canDiagnosticScanOffsetIdMask = 0;
    canDiagnosticScanResponses = 0;
    canDiagnosticScanTxFailures = 0;
    canDiagnosticScanActive = true;
    emitCanDiagnosticLine("DBC1," + currentCommandAddress() + ",SCAN," +
                          String(millis()) + ",start,0x141,0x160,opcode=9A");
    for (uint32_t requestId = RMD_DISCOVERY_FIRST_ID;
         requestId <= RMD_DISCOVERY_LAST_ID; ++requestId) {
      armCanDiagnosticCapture(requestId, RMD_DISCOVERY_REPLY_WAIT_MS);
      sendReadOnlyDiagnosticFrame(requestId, payload);
      vTaskDelay(pdMS_TO_TICKS(RMD_DISCOVERY_REPLY_WAIT_MS));
    }
    canDiagnosticCaptureActive = false;
    canDiagnosticScanActive = false;
    restoreMotorNativePolling(restorePolling);
    emitCanDiagnosticLine("DBC1," + currentCommandAddress() + ",SCAN," +
                          String(millis()) + ",complete,0x141,0x160,opcode=9A," +
                          "found=" + canDiscoveryIdList(canDiagnosticScanFoundMask) + "," +
                          "same_id=" + canDiscoveryIdList(canDiagnosticScanSameIdMask) + "," +
                          "offset_id=" + canDiscoveryIdList(canDiagnosticScanOffsetIdMask) + "," +
                          "responses=" + String(canDiagnosticScanResponses) + "," +
                          "tx_failures=" + String(canDiagnosticScanTxFailures));
    return true;
  }

  if (normalized == "can sniff" || normalized.startsWith("can sniff ")) {
    uint32_t duration = CAN_DIAGNOSTIC_DEFAULT_CAPTURE_MS;
    if (normalized.startsWith("can sniff ")) {
      String durationToken = command.substring(String("can sniff ").length());
      durationToken.trim();
      if (!parseCanDurationToken(durationToken, duration) ||
          duration > CAN_DIAGNOSTIC_SNIFF_MAX_MS) {
        emitCanDiagnosticLine("DBC1," + currentCommandAddress() + ",REJECT," +
                              String(millis()) + ",sniff_duration_must_be_100_to_2000_ms");
        return true;
      }
    }
    armCanDiagnosticSniff(duration);
    printCanDiagnosticStatus();
    return true;
  }

  String prefix;
  if (normalized.startsWith("can probe ")) prefix = "can probe ";
  else if (normalized.startsWith("can info ")) prefix = "can info ";
  else if (normalized.startsWith("can monitor ")) prefix = "can monitor ";
  else if (normalized.startsWith("can tx-read ")) prefix = "can tx-read ";
  else {
    emitCanDiagnosticLine("DBC1," + currentCommandAddress() + ",REJECT," +
                          String(millis()) + ",usage");
    return true;
  }

  const String arguments = command.substring(prefix.length());
  int cursor = 0;
  String token;
  uint32_t requestId = 0;
  if (!nextCanCommandToken(arguments, cursor, token) ||
      !parseCanHexToken(token, 0x7FF, requestId) ||
      !selectedDiagnosticMotorId(requestId)) {
    emitCanDiagnosticLine("DBC1," + currentCommandAddress() + ",REJECT," +
                          String(millis()) + ",id_outside_diagnostic_range");
    return true;
  }

  if (prefix == "can monitor ") {
    uint32_t duration = CAN_DIAGNOSTIC_DEFAULT_CAPTURE_MS;
    if (nextCanCommandToken(arguments, cursor, token) &&
        !parseCanDurationToken(token, duration)) {
      emitCanDiagnosticLine("DBC1," + currentCommandAddress() + ",REJECT," +
                            String(millis()) + ",duration_must_be_100_to_10000_ms");
      return true;
    }
    if (nextCanCommandToken(arguments, cursor, token)) {
      emitCanDiagnosticLine("DBC1," + currentCommandAddress() + ",REJECT," +
                            String(millis()) + ",too_many_arguments");
      return true;
    }
    armCanDiagnosticCapture(requestId, duration);
    printCanDiagnosticStatus();
    return true;
  }

  if (prefix == "can info ") {
    if (nextCanCommandToken(arguments, cursor, token)) {
      emitCanDiagnosticLine("DBC1," + currentCommandAddress() + ",REJECT," +
                            String(millis()) + ",too_many_arguments");
      return true;
    }
    const bool restorePolling = quiesceMotorNativePolling();
    static const byte INFO_OPCODES[] = {0x9A, 0x9C, 0x9D, 0x92, 0x90, 0x30, 0x42};
    armCanDiagnosticCapture(requestId, 2000);
    for (byte index = 0; index < sizeof(INFO_OPCODES); ++index) {
      byte payload[8] = {0};
      payload[0] = INFO_OPCODES[index];
      sendReadOnlyDiagnosticFrame(requestId, payload);
      vTaskDelay(pdMS_TO_TICKS(8));
    }
    vTaskDelay(pdMS_TO_TICKS(20));
    canDiagnosticCaptureActive = false;
    restoreMotorNativePolling(restorePolling);
    return true;
  }

  byte payload[8] = {0};
  if (prefix == "can probe ") {
    uint32_t opcode = 0;
    if (!nextCanCommandToken(arguments, cursor, token) ||
        !parseCanHexToken(token, 0xFF, opcode) ||
        nextCanCommandToken(arguments, cursor, token)) {
      emitCanDiagnosticLine("DBC1," + currentCommandAddress() + ",REJECT," +
                            String(millis()) + ",probe_requires_id_and_opcode");
      return true;
    }
    payload[0] = static_cast<byte>(opcode);
  } else {
    for (byte index = 0; index < 8; ++index) {
      uint32_t value = 0;
      if (!nextCanCommandToken(arguments, cursor, token) ||
          !parseCanHexToken(token, 0xFF, value)) {
        emitCanDiagnosticLine("DBC1," + currentCommandAddress() + ",REJECT," +
                              String(millis()) + ",tx_read_requires_exactly_8_bytes");
        return true;
      }
      payload[index] = static_cast<byte>(value);
    }
    if (nextCanCommandToken(arguments, cursor, token)) {
      emitCanDiagnosticLine("DBC1," + currentCommandAddress() + ",REJECT," +
                            String(millis()) + ",tx_read_requires_exactly_8_bytes");
      return true;
    }
  }

  if (!isReadOnlyCanDiagnosticOpcode(payload[0])) {
    emitCanDiagnosticLine("DBC1," + currentCommandAddress() + ",REJECT," +
                          String(millis()) + ",unsafe_opcode");
    return true;
  }
  armCanDiagnosticCapture(requestId, CAN_DIAGNOSTIC_DEFAULT_CAPTURE_MS);
  sendReadOnlyDiagnosticFrame(requestId, payload);
  return true;
}

uint8_t mcp2515ReadRegisterDirect(uint8_t address) {
  SPI.beginTransaction(SPISettings(10000000, MSBFIRST, SPI_MODE0));
  digitalWrite(CAN_CS_PIN, LOW);
  SPI.transfer(MCP_READ);
  SPI.transfer(address);
  const uint8_t value = SPI.transfer(0x00);
  digitalWrite(CAN_CS_PIN, HIGH);
  SPI.endTransaction();
  return value;
}

void mcp2515WriteRegisterDirect(uint8_t address, uint8_t value) {
  SPI.beginTransaction(SPISettings(10000000, MSBFIRST, SPI_MODE0));
  digitalWrite(CAN_CS_PIN, LOW);
  SPI.transfer(MCP_WRITE);
  SPI.transfer(address);
  SPI.transfer(value);
  digitalWrite(CAN_CS_PIN, HIGH);
  SPI.endTransaction();
}

void mcp2515WriteRegistersDirect(uint8_t address, const uint8_t *values,
                                 uint8_t length) {
  SPI.beginTransaction(SPISettings(10000000, MSBFIRST, SPI_MODE0));
  digitalWrite(CAN_CS_PIN, LOW);
  SPI.transfer(MCP_WRITE);
  SPI.transfer(address);
  for (uint8_t index = 0; index < length; ++index) SPI.transfer(values[index]);
  digitalWrite(CAN_CS_PIN, HIGH);
  SPI.endTransaction();
}

void mcp2515BitModifyDirect(uint8_t address, uint8_t mask, uint8_t value) {
  SPI.beginTransaction(SPISettings(10000000, MSBFIRST, SPI_MODE0));
  digitalWrite(CAN_CS_PIN, LOW);
  SPI.transfer(MCP_BITMOD);
  SPI.transfer(address);
  SPI.transfer(mask);
  SPI.transfer(value);
  digitalWrite(CAN_CS_PIN, HIGH);
  SPI.endTransaction();
}

int configureCanTimingAndNormalMode() {
  // MCP_CAN_lib 1.5.1 uses CNF2=0xC0 for 8 MHz / 1 Mbit/s, enabling
  // triple sampling. Microchip documents that mode as a slow/noisy-bus aid;
  // at this four-TQ bit time the extra samples occur only 0.5 and 1 TQ before
  // the nominal 75% sample point. Own the complete register tuple and sample
  // once at 75% (CNF2=0x80). The 8 MHz PS2=1 TQ limitation remains reported
  // as out-of-spec; production hardware should still migrate to 16 MHz.
  if (CAN.setMode(MODE_CONFIG) != CAN_OK) return CAN_FAIL;
  mcp2515WriteRegisterDirect(MCP_CNF1, CAN_CNF1_8MHZ_1MBPS);
  mcp2515WriteRegisterDirect(MCP_CNF2, CAN_CNF2_8MHZ_1MBPS_SINGLE_SAMPLE);
  mcp2515WriteRegisterDirect(MCP_CNF3, CAN_CNF3_8MHZ_1MBPS);
  const bool verified =
    mcp2515ReadRegisterDirect(MCP_CNF1) == CAN_CNF1_8MHZ_1MBPS &&
    mcp2515ReadRegisterDirect(MCP_CNF2) == CAN_CNF2_8MHZ_1MBPS_SINGLE_SAMPLE &&
    mcp2515ReadRegisterDirect(MCP_CNF3) == CAN_CNF3_8MHZ_1MBPS;
  if (!verified) return CAN_FAIL;
  return CAN.setMode(MCP_NORMAL);
}

// MCP_CAN_lib::sendMsg() considers a cleared TXREQ bit successful without
// examining ABTF, MLOA, or TXERR. That is incorrect in one-shot mode: TXREQ
// also clears after an arbitration loss or transmit error. Keep the library
// for controller setup/RX, but own standard-frame transmission here so every
// caller receives the actual wire outcome. Caller must hold canMutex.
int mcp2515SendStandardFrameDirect(uint32_t id, const byte *data, byte length) {
  if (id > 0x7FFU || data == nullptr || length > 8) return CAN_FAILTX;

  // The firmware intentionally supports only one outstanding CAN transaction.
  // Own TXB0 exclusively so a stale TXREQ can never spill later frames into
  // TXB1/TXB2 and silently turn one request into three in-flight mailboxes.
  const uint8_t controlAddress = MCP_TXB0CTRL;
  const uint32_t freeStartedUs = micros();
  uint8_t control = mcp2515ReadRegisterDirect(controlAddress);
  while ((control & MCP_TXB_TXREQ_M) != 0 &&
         micros() - freeStartedUs < CAN_TX_COMPLETION_TIMEOUT_US) {
    delayMicroseconds(10);
    control = mcp2515ReadRegisterDirect(controlAddress);
  }
  if ((control & MCP_TXB_TXREQ_M) != 0) return CAN_GETTXBFTIMEOUT;

  // Clear stale completion/error flags before this mailbox is reused.
  mcp2515WriteRegisterDirect(controlAddress, 0);
  mcp2515BitModifyDirect(MCP_CANINTF,
                         MCP_TX0IF | MCP_ERRIF | MCP_MERRF, 0);
  uint8_t frame[13] = {0};
  frame[0] = static_cast<uint8_t>((id >> 3) & 0xFF);  // SIDH
  frame[1] = static_cast<uint8_t>((id & 0x07) << 5); // SIDL, standard ID
  frame[4] = length & 0x0F;                          // DLC
  for (uint8_t index = 0; index < length; ++index) frame[5 + index] = data[index];
  mcp2515WriteRegistersDirect(controlAddress + 1, frame, sizeof(frame));
  mcp2515BitModifyDirect(controlAddress, MCP_TXB_TXREQ_M, MCP_TXB_TXREQ_M);

  const uint32_t transmitStartedUs = micros();
  control = MCP_TXB_TXREQ_M;
  do {
    control = mcp2515ReadRegisterDirect(controlAddress);
    if ((control & MCP_TXB_TXREQ_M) == 0) break;
    delayMicroseconds(10);
  } while (micros() - transmitStartedUs < CAN_TX_COMPLETION_TIMEOUT_US);

  canTxBufferControl[0] = control;
  mcp2515BitModifyDirect(MCP_CANINTF,
                         MCP_TX0IF | MCP_ERRIF | MCP_MERRF, 0);
  if ((control & MCP_TXB_TXREQ_M) != 0) return CAN_SENDMSGTIMEOUT;
  bool failed = false;
  if ((control & MCP_TXB_TXERR_M) != 0) {
    canTxWireErrors++;
    failed = true;
  }
  if ((control & MCP_TXB_MLOA_M) != 0) {
    canTxArbitrationLosses++;
    failed = true;
  }
  if ((control & MCP_TXB_ABTF_M) != 0) {
    canTxControllerAborts++;
    failed = true;
  }
  return failed ? CAN_FAILTX : CAN_OK;
}

bool abortPendingCanTx(const char *reason) {
  if (!canInitialized || canMutex == nullptr) return false;
  canTxAbortAttempts++;
  bool cleared = false;
  if (xSemaphoreTake(canMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
    // ABAT aborts every pending mailbox. It must be cleared again before the
    // controller is allowed to transmit. The installed MCP_CAN_lib::abortTX()
    // sets ABAT but never clears it, so perform the complete datasheet sequence
    // here while holding the same SPI mutex used by every library operation.
    mcp2515BitModifyDirect(MCP_CANCTRL, ABORT_TX, ABORT_TX);
    const uint32_t startedUs = micros();
    uint8_t pending = MCP_TXB_TXREQ_M;
    while (pending != 0 && micros() - startedUs < CAN_TX_ABORT_TIMEOUT_US) {
      pending = (mcp2515ReadRegisterDirect(MCP_TXB0CTRL) |
                 mcp2515ReadRegisterDirect(MCP_TXB1CTRL) |
                 mcp2515ReadRegisterDirect(MCP_TXB2CTRL)) & MCP_TXB_TXREQ_M;
      if (pending != 0) delayMicroseconds(10);
    }
    mcp2515BitModifyDirect(MCP_CANCTRL, ABORT_TX, 0);
    const uint8_t remaining =
      (mcp2515ReadRegisterDirect(MCP_TXB0CTRL) |
       mcp2515ReadRegisterDirect(MCP_TXB1CTRL) |
       mcp2515ReadRegisterDirect(MCP_TXB2CTRL)) & MCP_TXB_TXREQ_M;
    const uint8_t control = mcp2515ReadRegisterDirect(MCP_CANCTRL);
    cleared = remaining == 0 && (control & ABORT_TX) == 0;
    xSemaphoreGive(canMutex);
  } else {
    canMutexTimeouts++;
  }

  if (cleared) {
    canTxAbortSuccesses++;
    canDeferredTxPending = false;
    canDeferredTxSinceMs = 0;
    canDeferredTxAbortAttempted = false;
  } else {
    canTxAbortFailures++;
    portENTER_CRITICAL(&canStateMux);
    if (canConsecutiveFailures < CAN_RECOVERY_FAILURE_THRESHOLD) {
      canConsecutiveFailures = CAN_RECOVERY_FAILURE_THRESHOLD;
    }
    portEXIT_CRITICAL(&canStateMux);
  }
  const uint32_t reportNow = millis();
  if (!cleared || lastCanTxAbortReportMs == 0 ||
      reportNow - lastCanTxAbortReportMs >= 1000) {
    lastCanTxAbortReportMs = reportNow;
    emitCanDiagnosticLine(
      "DBC1," + currentCommandAddress() + ",TX_ABORT," + String(reportNow) + "," +
      (cleared ? "ok" : "failed") + ",reason=" + String(reason ? reason : "unknown")
    );
  }
  return cleared;
}

void sampleCanControllerErrors() {
  if (!canInitialized || canMutex == nullptr) return;
  if (xSemaphoreTake(canMutex, pdMS_TO_TICKS(5)) != pdTRUE) return;
  canControllerMode = mcp2515ReadRegisterDirect(MCP_CANSTAT) & MODE_MASK;
  canControlRegister = mcp2515ReadRegisterDirect(MCP_CANCTRL);
  canBitTimingRegisters[0] = mcp2515ReadRegisterDirect(MCP_CNF1);
  canBitTimingRegisters[1] = mcp2515ReadRegisterDirect(MCP_CNF2);
  canBitTimingRegisters[2] = mcp2515ReadRegisterDirect(MCP_CNF3);
  canInterruptEnable = mcp2515ReadRegisterDirect(MCP_CANINTE);
  canInterruptFlags = mcp2515ReadRegisterDirect(MCP_CANINTF);
  canErrorFlags = mcp2515ReadRegisterDirect(MCP_EFLG);
  canTxErrorCount = mcp2515ReadRegisterDirect(MCP_TEC);
  canRxErrorCount = mcp2515ReadRegisterDirect(MCP_REC);
  canTxBufferControl[0] = mcp2515ReadRegisterDirect(MCP_TXB0CTRL);
  canTxBufferControl[1] = mcp2515ReadRegisterDirect(MCP_TXB1CTRL);
  canTxBufferControl[2] = mcp2515ReadRegisterDirect(MCP_TXB2CTRL);
  canRxBufferControl[0] = mcp2515ReadRegisterDirect(MCP_RXB0CTRL);
  canRxBufferControl[1] = mcp2515ReadRegisterDirect(MCP_RXB1CTRL);
  xSemaphoreGive(canMutex);
}

bool clearCanRxOverflowFlags() {
  if (!canInitialized || canMutex == nullptr) return false;
  const uint8_t overflowMask = MCP_EFLG_RX0OVR | MCP_EFLG_RX1OVR;
  bool cleared = false;
  if (xSemaphoreTake(canMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
    const uint8_t before = mcp2515ReadRegisterDirect(MCP_EFLG);
    if ((before & overflowMask) != 0) {
      canRxOverflowEvents++;
      // RXnOVR is sticky and the datasheet requires the MCU to clear it. The
      // receive task has already drained both mailboxes; a full controller
      // reset here only destroys good state and creates a five-second storm.
      mcp2515BitModifyDirect(MCP_EFLG, overflowMask, 0);
      mcp2515BitModifyDirect(MCP_CANINTF, MCP_ERRIF, 0);
      const uint8_t after = mcp2515ReadRegisterDirect(MCP_EFLG);
      canErrorFlags = after;
      cleared = (after & overflowMask) == 0;
      if (cleared) canRxOverflowClears++;
    }
    xSemaphoreGive(canMutex);
  }
  return cleared;
}

const char *canResultName(int result) {
  switch (result) {
    case CAN_OK: return "OK";
    case CAN_FAILINIT: return "FAIL_INIT";
    case CAN_FAILTX: return "FAIL_TX";
    case CAN_MSGAVAIL: return "MSG_AVAILABLE";
    case CAN_NOMSG: return "NO_MSG";
    case CAN_CTRLERROR: return "CONTROLLER_ERROR";
    case CAN_GETTXBFTIMEOUT: return "GET_TX_BUFFER_TIMEOUT";
    case CAN_SENDMSGTIMEOUT: return "SEND_MSG_TIMEOUT";
    case CAN_FAIL: return "FAIL";
    case -2: return "MUTEX_TIMEOUT";
    default: return "UNKNOWN";
  }
}

String canErrorFlagNames(uint8_t flags) {
  if (flags == 0) return "none";
  String names;
  if (flags & MCP_EFLG_EWARN) names += "EWARN";
  if (flags & MCP_EFLG_RXWAR) names += names.length() ? "|RXWAR" : "RXWAR";
  if (flags & MCP_EFLG_TXWAR) names += names.length() ? "|TXWAR" : "TXWAR";
  if (flags & MCP_EFLG_RXEP) names += names.length() ? "|RXEP" : "RXEP";
  if (flags & MCP_EFLG_TXEP) names += names.length() ? "|TXEP" : "TXEP";
  if (flags & MCP_EFLG_TXBO) names += names.length() ? "|TXBO" : "TXBO";
  if (flags & MCP_EFLG_RX0OVR) names += names.length() ? "|RX0OVR" : "RX0OVR";
  if (flags & MCP_EFLG_RX1OVR) names += names.length() ? "|RX1OVR" : "RX1OVR";
  return names;
}

void printCanBusStatus() {
  // HardwareObservationManager admits at most 256 bytes per serial record.
  // Keep state, error counts, and timing in separate bounded records rather
  // than producing one oversized line that cannot be delivered intact.
  char line[256];
  const char *address = commandTargetCString(currentCommandTarget());
  const unsigned long now = static_cast<unsigned long>(millis());
  snprintf(line, sizeof(line),
           "DBC1,%s,BUS,%lu,ready=%u,eflg=%02X,tec=%u,rec=%u,fail=%lu,recovery=%lu/%lu,last=%d,result=%s,oneshot=%u,deferred=%u,poll=%s,mode=%02X,timing=%s",
           address, now, canInitialized ? 1U : 0U, canErrorFlags,
           canTxErrorCount, canRxErrorCount,
           static_cast<unsigned long>(canConsecutiveFailures),
           static_cast<unsigned long>(canRecoverySuccesses),
           static_cast<unsigned long>(canRecoveryAttempts), lastCanResult,
           canResultName(lastCanResult), canOneShotEnabled ? 1U : 0U,
           canDeferredTxPending ? 1U : 0U,
           motorNativePollingEnabled ? "on" : "off", canControllerMode,
           CAN_BIT_TIMING_DATASHEET_COMPLIANT ? "compliant" : "out_of_spec_8mhz_1mbps");
  emitCanDiagnosticCString(line);

  const String flagNames = canErrorFlagNames(canErrorFlags);
  snprintf(line, sizeof(line),
           "DBC1,%s,ERRORS,%lu,flags=%s,tx_abort=%lu/%lu,wire=%lu,arb=%lu,abort=%lu,overflow=%lu/%lu,rx_irq=%lu,rx_wake=%lu,rx_burst=%u",
           address, now, flagNames.c_str(),
           static_cast<unsigned long>(canTxAbortSuccesses),
           static_cast<unsigned long>(canTxAbortAttempts),
           static_cast<unsigned long>(canTxWireErrors),
           static_cast<unsigned long>(canTxArbitrationLosses),
           static_cast<unsigned long>(canTxControllerAborts),
           static_cast<unsigned long>(canRxOverflowClears),
           static_cast<unsigned long>(canRxOverflowEvents),
           static_cast<unsigned long>(canRxInterrupts),
           static_cast<unsigned long>(canRxWakeups), canRxMaxBurst);
  emitCanDiagnosticCString(line);

  snprintf(line, sizeof(line),
           "DBC1,%s,TIMING,%lu,queue_us=%lu/%lu,tx_us=%lu/%lu,batch_us=%lu/%lu,batch_miss=%lu,io_us=%lu/%lu,io_miss=%lu,rx_us=%lu/%lu,diag_depth=%u,diag_drop=%lu",
           address, now,
           static_cast<unsigned long>(canTxQueueLatencyLastUs),
           static_cast<unsigned long>(canTxQueueLatencyMaxUs),
           static_cast<unsigned long>(canTxExecutionLastUs),
           static_cast<unsigned long>(canTxExecutionMaxUs),
           static_cast<unsigned long>(canOutputBatchLastUs),
           static_cast<unsigned long>(canOutputBatchMaxUs),
           static_cast<unsigned long>(canOutputDeadlineMisses),
           static_cast<unsigned long>(canIoLoopLastUs),
           static_cast<unsigned long>(canIoLoopMaxUs),
           static_cast<unsigned long>(canIoLoopBudgetMisses),
           static_cast<unsigned long>(canRxDrainLastUs),
           static_cast<unsigned long>(canRxDrainMaxUs),
           static_cast<unsigned int>(canDiagnosticLineQueue ?
             uxQueueMessagesWaiting(canDiagnosticLineQueue) : 0),
           static_cast<unsigned long>(canDiagnosticLineDrops));
  emitCanDiagnosticCString(line);
}

void printCanRegisterSnapshot() {
  if (!canInitialized) return;
  // The CAN I/O task refreshes this cache. Command/portal tasks never touch
  // MCP2515 SPI, preserving single-owner timing and avoiding diagnostic jitter.
  char line[320];
  snprintf(line, sizeof(line),
           "DBC1,%s,REGISTERS,%lu,CANSTAT=%02X,CANCTRL=%02X,CNF1=%02X,CNF2=%02X,CNF3=%02X,CANINTE=%02X,CANINTF=%02X,EFLG=%02X,TEC=%u,REC=%u,RXB0CTRL=%02X,RXB1CTRL=%02X,TXB0CTRL=%02X,TXB1CTRL=%02X,TXB2CTRL=%02X",
           currentCommandAddress().c_str(), static_cast<unsigned long>(millis()),
           canControllerMode, canControlRegister, canBitTimingRegisters[0],
           canBitTimingRegisters[1], canBitTimingRegisters[2], canInterruptEnable,
           canInterruptFlags, canErrorFlags, canTxErrorCount, canRxErrorCount,
           canRxBufferControl[0], canRxBufferControl[1], canTxBufferControl[0],
           canTxBufferControl[1], canTxBufferControl[2]);
  emitCanDiagnosticLine(String(line));
}

bool recoverCanController() {
  if (!isLegRole() || canMutex == nullptr || !spiInitialized) return false;
  const uint32_t now = millis();
  if (now - lastCanRecoveryMs < CAN_RECOVERY_INTERVAL_MS) return false;
  lastCanRecoveryMs = now;
  canRecoveryAttempts++;

  bool recovered = false;
  int beginResult = -1;
  int modeResult = -1;
  int oneShotResult = -1;
  // MCP_CAN_lib prints during begin()/setMode(). Hold the serial mutex so a
  // recovery cannot splice those messages into a DB3/DBH1 record.
  const bool serialHeld = serialMutex == nullptr ||
    xSemaphoreTake(serialMutex, pdMS_TO_TICKS(50)) == pdTRUE;
  if (serialHeld && xSemaphoreTake(canMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
    beginResult = CAN.begin(MCP_ANY, CAN_1000KBPS, MCP_8MHZ);
    if (beginResult == CAN_OK) {
      modeResult = configureCanTimingAndNormalMode();
      if (modeResult == CAN_OK) oneShotResult = CAN.enOneShotTX();
      recovered = modeResult == CAN_OK && oneShotResult == CAN_OK;
    }
    canErrorFlags = CAN.getError();
    canTxErrorCount = CAN.errorCountTX();
    canRxErrorCount = CAN.errorCountRX();
    xSemaphoreGive(canMutex);
  }
  if (serialHeld && serialMutex != nullptr) xSemaphoreGive(serialMutex);

  if (recovered) {
    canInitialized = true;
    canOneShotEnabled = true;
    portENTER_CRITICAL(&canStateMux);
    canConsecutiveFailures = 0;
    portEXIT_CRITICAL(&canStateMux);
    canErrorFlags = 0;
    canTxErrorCount = 0;
    canRxErrorCount = 0;
    motorNativePendingIndex = -1;
    motorNativePendingSinceMs = 0;
    canDeferredTxPending = false;
    canDeferredTxSinceMs = 0;
    canDeferredTxAbortAttempted = false;
    canRecoverySuccesses++;
  } else if (beginResult != -1) {
    canOneShotEnabled = false;
  }
  emitCanDiagnosticLine(
    "DBC1," + currentCommandAddress() + ",RECOVERY," + String(now) + "," +
    (recovered ? "ok" : "failed") + "," + String(beginResult) + "," +
    String(modeResult) + "," + String(canErrorFlags) + "," +
    String(canTxErrorCount) + "," + String(canRxErrorCount) + "," +
    "eflg=" + canErrorFlagNames(canErrorFlags) + ",oneshot_result=" +
    String(canResultName(oneShotResult)) + ",oneshot_enabled=" +
    String(canOneShotEnabled ? 1 : 0)
  );
  return recovered;
}

uint8_t encodeCanResultForNotification(int result) {
  return result < 0 ? 0xFF : static_cast<uint8_t>(result);
}

int decodeCanResultFromNotification(uint8_t encoded) {
  return encoded == 0xFF ? -2 : static_cast<int>(encoded);
}

uint32_t nextCanTxToken() {
  portENTER_CRITICAL(&canStateMux);
  canTxQueueToken = (canTxQueueToken + 1) & 0x00FFFFFFUL;
  if (canTxQueueToken == 0) canTxQueueToken = 1;
  const uint32_t token = canTxQueueToken;
  portEXIT_CRITICAL(&canStateMux);
  return token;
}

int performCanTransmit(uint32_t id, const byte *data, byte length) {
  const uint32_t startedUs = micros();
  if (canMutex == nullptr ||
      xSemaphoreTake(canMutex, pdMS_TO_TICKS(5)) != pdTRUE) {
    canMutexTimeouts++;
    const uint32_t executionUs = micros() - startedUs;
    canTxExecutionLastUs = executionUs;
    if (executionUs > canTxExecutionMaxUs) canTxExecutionMaxUs = executionUs;
    return -2;
  }
  const int result = mcp2515SendStandardFrameDirect(id, data, length);
  xSemaphoreGive(canMutex);
  const uint32_t executionUs = micros() - startedUs;
  canTxExecutionLastUs = executionUs;
  if (executionUs > canTxExecutionMaxUs) canTxExecutionMaxUs = executionUs;
  return result;
}

bool serviceCanTxQueueOne() {
  if (canTxQueue == nullptr) return false;
  CanTxRequest request;
  if (xQueueReceive(canTxQueue, &request, 0) != pdTRUE) return false;

  const uint32_t serviceStartedUs = micros();
  const uint32_t queueLatencyUs = serviceStartedUs - request.enqueuedUs;
  canTxQueueLatencyLastUs = queueLatencyUs;
  if (queueLatencyUs > canTxQueueLatencyMaxUs) {
    canTxQueueLatencyMaxUs = queueLatencyUs;
  }
  int result = -2;
  if (static_cast<int32_t>(serviceStartedUs - request.deadlineUs) > 0) {
    canTxQueueExpired++;
  } else {
    if (request.highPriority && canDeferredTxPending) {
      const int8_t interruptedRead = motorNativePendingIndex;
      if (abortPendingCanTx("priority_preempts_deferred_read") &&
          interruptedRead >= 0) {
        motorNativePendingIndex = -1;
        motorNativePendingSinceMs = 0;
        recordMotorNativeMiss(static_cast<uint8_t>(interruptedRead), millis(), true);
      }
    }
    result = performCanTransmit(request.id, request.data, request.length);
    if ((result == CAN_SENDMSGTIMEOUT || result == CAN_GETTXBFTIMEOUT) &&
        !request.allowDeferredRead) {
      abortPendingCanTx(request.highPriority ? "queued_priority_tx_timeout"
                                             : "queued_tx_timeout");
    }
  }
  canTxQueueCompleted++;
  const uint32_t response = (request.token << 8) |
    encodeCanResultForNotification(result);
  if (request.requester != nullptr) {
    xTaskNotify(request.requester, response, eSetValueWithOverwrite);
  }
  return true;
}

int canSendFrameResult(uint32_t actuatorID, const byte *data, byte dataLen,
                       bool allowDeferredRead) {
  if (isCenter || isHead || !canInitialized || canMutex == nullptr ||
      data == nullptr || dataLen > 8) return -2;

  const int actuatorIndex = actuatorIndexFromCanId(actuatorID);
  const dropbear::MotorProfile *profile = motorProfileForActuator(actuatorIndex);
  const bool motionFrame = profile != nullptr && dataLen > 0 &&
    (data[0] == profile->torqueOpcode || data[0] == profile->stopOpcode);
  int result = -2;

  if (xTaskGetCurrentTaskHandle() == canRxTaskHandle) {
    // Automatic request/response scheduling already runs in the sole CAN I/O
    // task, so it can execute directly without queueing back to itself.
    result = performCanTransmit(actuatorID, data, dataLen);
  } else if (canTxQueue != nullptr && canRxTaskHandle != nullptr) {
    CanTxRequest request = {};
    request.id = actuatorID;
    request.length = dataLen;
    memcpy(request.data, data, dataLen);
    request.allowDeferredRead = allowDeferredRead;
    request.highPriority = motionFrame;
    request.token = nextCanTxToken();
    request.enqueuedUs = micros();
    request.deadlineUs = request.enqueuedUs + CAN_TX_QUEUE_DEADLINE_US;
    request.requester = xTaskGetCurrentTaskHandle();

    uint32_t discarded = 0;
    xTaskNotifyWait(0, UINT32_MAX, &discarded, 0);
    const BaseType_t queued = motionFrame
      ? xQueueSendToFront(canTxQueue, &request, pdMS_TO_TICKS(1))
      : xQueueSendToBack(canTxQueue, &request, pdMS_TO_TICKS(1));
    if (queued == pdTRUE) {
      canTxQueueAccepted++;
      xTaskNotifyGive(canRxTaskHandle);
      const uint32_t waitStartedMs = millis();
      bool responseReceived = false;
      while (true) {
        const uint32_t elapsedMs = millis() - waitStartedMs;
        if (elapsedMs > CAN_TX_CALLER_TIMEOUT_MS) break;
        uint32_t response = 0;
        const TickType_t remaining = pdMS_TO_TICKS(
          CAN_TX_CALLER_TIMEOUT_MS - elapsedMs + 1);
        if (xTaskNotifyWait(0, UINT32_MAX, &response, remaining) == pdTRUE &&
            (response >> 8) == request.token) {
          result = decodeCanResultFromNotification(response & 0xFF);
          responseReceived = true;
          break;
        }
      }
      if (!responseReceived) canTxCallerTimeouts++;
    } else {
      canTxQueueFull++;
    }
  }

  const uint32_t now = millis();
  const bool ok = result == CAN_OK;
  const bool deferredRead = allowDeferredRead && data != nullptr && dataLen > 0 &&
                            result == CAN_SENDMSGTIMEOUT;
  portENTER_CRITICAL(&canStateMux);
  lastCanResult = result;
  if (ok) {
    canTxSuccess++;
    canConsecutiveFailures = 0;
    lastCanTxMs = now;
    if (profile != nullptr && data[0] == profile->torqueOpcode) canTorqueFrames++;
    if (profile != nullptr && data[0] == profile->stopOpcode) canStopFrames++;
  } else if (deferredRead) {
    // TXREQ is already loaded. Completion is resolved by the matching reply or
    // the transaction timeout instead of queuing another request behind it.
    // One-shot does not clear TXREQ until a first bus attempt occurs, so the
    // timeout path must explicitly abort this mailbox if the bus never idles.
    canDeferredReadSubmissions++;
    canDeferredTxPending = true;
    canDeferredTxSinceMs = now;
    canDeferredTxAbortAttempted = false;
    lastCanTxMs = now;
  } else {
    canTxFailure++;
    canConsecutiveFailures++;
    lastCanFailureMs = now;
  }

  // Read diagnostics must not overwrite the last motion-command result used by
  // actuator safety/status reporting.
  if (actuatorIndex >= 0 && motionFrame) {
    ActuatorDiagnostic &d = actuatorDiagnostics[actuatorIndex];
    d.lastOpcode = data[0];
    d.lastResult = result;
    d.lastTxMs = now;
    if (profile != nullptr && data[0] == profile->torqueOpcode) {
      d.lastCommand = static_cast<int16_t>(static_cast<uint16_t>(data[4]) |
                                           (static_cast<uint16_t>(data[5]) << 8));
    } else if (profile != nullptr && data[0] == profile->stopOpcode) {
      d.lastCommand = 0;
    }
    if (ok) d.txOk++; else d.txFail++;
  }
  portEXIT_CRITICAL(&canStateMux);

  return result;
}

bool canSendFrame(uint32_t actuatorID, const byte *data, byte dataLen) {
  return canSendFrameResult(actuatorID, data, dataLen, false) == CAN_OK;
}

bool canSend(uint32_t actuatorID, const byte data[8]) {
  return canSendFrame(actuatorID, data, 8);
}

bool sendTorqueCommand(unsigned long actuatorID, int16_t torqueValue) {
  const dropbear::MotorProfile *profile =
    motorProfileForActuator(actuatorIndexFromCanId(actuatorID));
  if (profile == nullptr) return false;
  byte buf[8];
  dropbear::encodeTorqueCommand(*profile, torqueValue, buf);
  return canSend(actuatorID, buf);
}

bool sendStopCommand(unsigned long actuatorID) {
  const dropbear::MotorProfile *profile =
    motorProfileForActuator(actuatorIndexFromCanId(actuatorID));
  if (profile == nullptr) return false;
  byte buf[8];
  dropbear::encodeStopCommand(*profile, buf);
  return canSend(actuatorID, buf);
}

void paceCanCommandBurst() {
  // Each synchronous submission is completed by the higher-priority CAN I/O
  // task. That task returns to RX draining before it accepts the next queued
  // frame. A one-tick delay here would add at least 6 ms to every six-motor
  // batch and can make a nominal 10 ms output cycle miss its own deadline.
  // Yield without sleeping; the transport owner's priority provides pacing.
  taskYIELD();
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

void failClosedCanMotion(const char *reason) {
  canMotionFailClosed++;
  portalTorqueTestActive = false;
  portalTorqueTestActuatorIndex = -1;
  portalTorqueTestValue = 0;
  portalTorqueTestUntilMs = 0;
  calibrationOverrideActive = false;
  calibrationActuatorIndex = -1;
  calibrationTorqueValue = 0;
  clearAllTorqueSetpoints();
  requestStop(3);
  appendWebLog("CAN MOTION FAIL-CLOSED: " + String(reason ? reason : "tx failure") +
               "; torque cleared and stop burst retained until delivered");
}

bool portalMotionAuthorized() {
  if (portalSafetyStage != 3 || portalMotionLeaseUntilMs == 0) return false;
  return (int32_t)(portalMotionLeaseUntilMs - millis()) > 0;
}

uint32_t portalMotionRemainingMs() {
  if (!portalMotionAuthorized()) return 0;
  return portalMotionLeaseUntilMs - millis();
}

void lockPortalMotion(bool stopOutputs, const char *reason) {
  const bool wasArmed = portalSafetyStage == 3 || portalTorqueTestActive;
  portalSafetyStage = 0;
  portalMotionLeaseUntilMs = 0;
  portalTorqueTestActive = false;
  portalTorqueTestActuatorIndex = -1;
  portalTorqueTestValue = 0;
  portalTorqueTestUntilMs = 0;
  if (stopOutputs && wasArmed) {
    clearAllTorqueSetpoints();
    requestStop(3);
  }
  if (reason && reason[0]) appendWebLog(String("PORTAL SAFETY LOCK: ") + reason);
}

void expirePortalMotionLeaseIfNeeded() {
  if (portalSafetyStage == 3 && !portalMotionAuthorized()) {
    lockPortalMotion(true, "motion lease expired");
  }
}

String payloadFromRoutedCommand(String command) {
  command.trim();
  if (!command.startsWith("<DB1:")) return command;
  const int close = command.indexOf('>');
  if (close < 0) return command;
  String payload = command.substring(close + 1);
  payload.trim();
  return payload;
}

bool portalPayloadRequiresMotionUnlock(String payload) {
  payload.trim();
  String lower = payload;
  lower.toLowerCase();
  if (lower == "play" || lower.startsWith("torque ") ||
      lower.startsWith("impedance ") || lower.startsWith("test_torque ") ||
      lower.startsWith("calibratedirection ")) return true;
  if (isHead) {
    if (lower == "home" || lower == "home_soft" || lower == "home_brute" ||
        lower.startsWith("neck ") || payload.indexOf(':') >= 0) return true;
    if (payload.length() && strchr("XYZHSARP", payload.charAt(0)) != nullptr) return true;
  }
  return false;
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
  const int actuatorIndex = hyperspawnJointToActuatorIndex(joint);
  float motorDegrees = 0.0f;
  if (readMotorControlDegrees(actuatorIndex, motorDegrees)) return motorDegrees;

  // Before the boot reference is ready, retain the calibrated AS5600 value for
  // staging a hold target. The torque controller itself remains at zero until
  // aligned, fresh CAN-native feedback is available.
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
  while (!runtimeInitializationComplete) vTaskDelay(pdMS_TO_TICKS(10));
  while (true) {
    const uint32_t loopStartedUs = micros();
    canRxTaskLoops++;
    lastCanRxTaskMs = millis();

    uint8_t drained = 0;
    const uint32_t drainStartedUs = micros();
    // A busy or electrically degraded bus must not monopolize core 1 and
    // starve sensor/DB3 publication. Drain a bounded batch, then yield at the
    // RTOS tick below so equal-priority control tasks get CPU time.
    while (canInitialized && drained < CAN_RX_BURST_LIMIT) {
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
            captureCanDiagnosticFrame(static_cast<uint32_t>(rxId), data, len);
            if (!ingestMotorNativeFeedback(static_cast<uint32_t>(rxId), data, len)) {
              handleHyperspawnRxFrame(static_cast<uint32_t>(rxId), data, len);
            }
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
    const uint32_t drainDurationUs = micros() - drainStartedUs;
    canRxDrainLastUs = drainDurationUs;
    if (drainDurationUs > canRxDrainMaxUs) canRxDrainMaxUs = drainDurationUs;
    if (drained > canRxMaxBurst) canRxMaxBurst = drained;
    // All non-I/O tasks submit complete frames through this queue. Service one
    // request per pass, then return to RX draining so a command's immediate
    // motor reply cannot sit behind a long transmit batch.
    const bool servicedQueuedTx = serviceCanTxQueueOne();
    static uint32_t lastMotorQueryMs = 0;
    static uint32_t lastCanErrorSampleMs = 0;
    static uint8_t motorQuerySlot = 0;
    const uint32_t now = millis();
    if (now - lastCanErrorSampleMs >= CAN_ERROR_SAMPLE_MS) {
      sampleCanControllerErrors();
      lastCanErrorSampleMs = now;
    }
    const bool canRxQuiet = lastCanRxMs == 0 ||
      now - lastCanRxMs >= CAN_RECOVERY_RX_QUIET_MS;
    if ((canErrorFlags & (MCP_EFLG_RX0OVR | MCP_EFLG_RX1OVR)) != 0) {
      clearCanRxOverflowFlags();
    }
    const bool controllerModeFault = canControllerMode != MCP_NORMAL;
    // TXEP/RXEP are degraded operating states, not reset requests. Repeatedly
    // resetting on those sticky thresholds was discarding good RX state every
    // five seconds. Reinitialize only after bus-off, an invalid mode, or a
    // proven run of transport failures while RX is quiet.
    if (!canDiagnosticScanActive &&
        (((canErrorFlags & MCP_EFLG_TXBO) != 0 || controllerModeFault) ||
         (canConsecutiveFailures >= CAN_RECOVERY_FAILURE_THRESHOLD && canRxQuiet))) {
      recoverCanController();
    }

    if (canDeferredTxPending && !canDeferredTxAbortAttempted &&
        now - canDeferredTxSinceMs >= CAN_DEFERRED_TX_ABORT_MS) {
      canDeferredTxAbortAttempted = true;
      abortPendingCanTx("deferred_read_timeout");
    }

    // A reply timeout closes the transaction before another request may use a
    // TX mailbox. This is intentionally independent of telemetry freshness:
    // control still fails closed at MOTOR_NATIVE_STALE_MS.
    if (motorNativePendingIndex >= 0 &&
        now - motorNativePendingSinceMs >= MOTOR_NATIVE_REPLY_TIMEOUT_MS) {
      const uint8_t timedOutIndex = static_cast<uint8_t>(motorNativePendingIndex);
      motorNativePendingIndex = -1;
      motorNativePendingSinceMs = 0;
      recordMotorNativeMiss(timedOutIndex, now, true);
    }

    const bool controllerDegraded =
      (canErrorFlags & (MCP_EFLG_EWARN | MCP_EFLG_TXWAR | MCP_EFLG_RXWAR |
                        MCP_EFLG_TXEP | MCP_EFLG_RXEP)) != 0;
    const uint32_t queryInterval = controllerDegraded || canConsecutiveFailures >= 3
      ? MOTOR_NATIVE_QUERY_BACKOFF_MS
      : MOTOR_NATIVE_QUERY_SLOT_MS;
    if (MOTOR_NATIVE_FEEDBACK_ENABLED && motorNativePollingEnabled &&
        runtimeControlReady &&
        !canDiagnosticScanActive &&
        !servicedQueuedTx &&
        motorNativePendingIndex < 0 &&
        now - lastMotorQueryMs >= queryInterval) {
      const uint8_t *order = selectedMotorTelemetryOrder();
      int selectedIndex = -1;
      for (uint8_t attempt = 0; attempt < 6; ++attempt) {
        const uint8_t candidate = order[motorQuerySlot];
        motorQuerySlot = static_cast<uint8_t>((motorQuerySlot + 1) % 6);
        const uint32_t eligibleAt = motorNativeNextEligibleMs[candidate];
        if (eligibleAt == 0 || static_cast<int32_t>(now - eligibleAt) >= 0) {
          selectedIndex = candidate;
          break;
        }
        motorNativeOfflineSkips++;
      }
      if (selectedIndex >= 0 &&
          !requestMotorNativeFeedback(static_cast<uint8_t>(selectedIndex))) {
        recordMotorNativeMiss(static_cast<uint8_t>(selectedIndex), now, false);
      }
      lastMotorQueryMs = now;
    }
    const uint32_t loopDurationUs = micros() - loopStartedUs;
    canIoLoopLastUs = loopDurationUs;
    if (loopDurationUs > canIoLoopMaxUs) canIoLoopMaxUs = loopDurationUs;
    if (loopDurationUs > CAN_IO_LOOP_BUDGET_US) canIoLoopBudgetMisses++;
    // CANINT stays low until every enabled receive flag is cleared. If the
    // bounded batch ended while it is still low, yield once and immediately
    // continue draining; otherwise sleep until the falling edge or the 1 ms
    // scheduler/health deadline. This sharply reduces two-buffer overflow risk
    // without allowing a noisy bus to monopolize the core.
    if (drained >= CAN_RX_BURST_LIMIT && digitalRead(CAN0_INT) == LOW) {
      taskYIELD();
    } else if (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(1)) > 0) {
      canRxWakeups++;
    }
  }
}
void hyperspawnRouteTask(void *parameter) {
  while (!runtimeInitializationComplete) vTaskDelay(pdMS_TO_TICKS(10));
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
    const uint32_t now = millis();
    if (MOTOR_NATIVE_FEEDBACK_ENABLED) {
      Serial.print("DB3,");
      Serial.print(now);
      Serial.print(',');
    }
    Serial.print(normalizedOuter, 1);
    Serial.print(',');
    Serial.print(normalizedInner, 1);
    Serial.print(',');
    Serial.print(normalizedHip, 1);
    Serial.print(',');
    Serial.print(normalizedKnee, 1);
    Serial.print(',');
    Serial.print(normalizedButt, 1);
    if (MOTOR_NATIVE_FEEDBACK_ENABLED) {
      const uint8_t *order = selectedMotorTelemetryOrder();
      for (uint8_t slot = 0; slot < 6; ++slot) {
        const uint8_t index = order[slot];
        Serial.print(',');
        // Preserve the last successfully decoded position for observability.
        // The freshMask below remains the authority for freshness, and the
        // control path independently rejects stale feedback.
        if (motorNativeValid[index]) {
          Serial.print(motorNativeDegrees[index], 2);
        } else {
          Serial.print("NA");
        }
      }
      uint8_t freshMask = 0;
      uint8_t controlMask = 0;
      uint8_t alignmentFaultMask = 0;
      for (uint8_t slot = 0; slot < 6; ++slot) {
        const uint8_t index = order[slot];
        const bool fresh = motorNativeValid[index] &&
          now - motorNativeReceivedMs[index] <= MOTOR_NATIVE_STALE_MS;
        if (fresh) freshMask |= static_cast<uint8_t>(1U << slot);
        float controlDegrees = 0.0f;
        const bool controlReady = readMotorControlDegrees(index, controlDegrees);
        Serial.print(',');
        if (controlReady) {
          controlMask |= static_cast<uint8_t>(1U << slot);
          Serial.print(controlDegrees, 2);
        } else {
          Serial.print("NA");
        }
        if (motorControlAlignmentFault[index]) {
          alignmentFaultMask |= static_cast<uint8_t>(1U << slot);
        }
      }
      Serial.print(',');
      Serial.print(freshMask);
      Serial.print(',');
      Serial.print(controlMask);
      Serial.print(',');
      Serial.print(alignmentFaultMask);
    }
    Serial.println();
    xSemaphoreGive(serialMutex);
  }
}

void printVersionRecord() {
  if (serialMutex && xSemaphoreTake(serialMutex, pdMS_TO_TICKS(20)) != pdTRUE) return;
  Serial.print(DROPBEAR_CAPABILITY_SCHEMA);
  Serial.print(',');
  Serial.print(currentCommandAddress());
  Serial.print(',');
  Serial.print(DROPBEAR_FIRMWARE_VERSION);
  Serial.print(',');
  Serial.print(DROPBEAR_COMMAND_PROTOCOL);
  Serial.print(',');
  Serial.print(DROPBEAR_TELEMETRY_PROTOCOL);
  Serial.print(',');
  Serial.println(DROPBEAR_CAPABILITIES);
  xSemaphoreGive(serialMutex);
}

void printHealthRecord() {
  if (serialMutex && xSemaphoreTake(serialMutex, pdMS_TO_TICKS(20)) != pdTRUE) return;
  const uint32_t now = millis();
  uint8_t sensorMask = 0;
  uint8_t freshMask = 0;
  uint8_t controlMask = 0;
  uint8_t alignmentFaultMask = 0;
  if (isLegRole()) {
    for (uint8_t sensor = 0; sensor < AS5600_SENSOR_COUNT; ++sensor) {
      const SensorDiagnostic &diagnostic = sensorDiagnostics[sensor];
      if (diagnostic.signalValid && diagnostic.pulseAgeUs <= AS5600_STALE_US) {
        sensorMask |= static_cast<uint8_t>(1U << sensor);
      }
    }
    const uint8_t *order = selectedMotorTelemetryOrder();
    for (uint8_t slot = 0; slot < 6; ++slot) {
      const uint8_t index = order[slot];
      if (motorNativeValid[index] &&
          now - motorNativeReceivedMs[index] <= MOTOR_NATIVE_STALE_MS) {
        freshMask |= static_cast<uint8_t>(1U << slot);
      }
      float ignored = 0.0f;
      if (readMotorControlDegrees(index, ignored)) {
        controlMask |= static_cast<uint8_t>(1U << slot);
      }
      if (motorControlAlignmentFault[index]) {
        alignmentFaultMask |= static_cast<uint8_t>(1U << slot);
      }
    }
  }
  const bool runtimeReady = isHead ? runtimeNeckReady :
    (isCenter ? runtimeImuReady : runtimeControlReady);
  const bool canReady = !isLegRole() || canInitialized;
  const bool hardFault = !runtimeReady || !canReady || alignmentFaultMask != 0;
  const bool degraded = isLegRole() && (sensorMask != 0x1F || freshMask != 0x3F ||
                                        controlMask != 0x3F || canConsecutiveFailures > 0);
  const char *overall = hardFault ? "fault" : (degraded ? "warn" : "ok");
  Serial.print("DBH1,");
  Serial.print(currentCommandAddress());
  Serial.print(','); Serial.print(now);
  Serial.print(','); Serial.print(overall);
  Serial.print(','); Serial.print(runtimeReady ? 1 : 0);
  Serial.print(','); Serial.print(canReady ? 1 : 0);
  Serial.print(','); Serial.print(sensorMask);
  Serial.print(','); Serial.print(freshMask);
  Serial.print(','); Serial.print(controlMask);
  Serial.print(','); Serial.print(alignmentFaultMask);
  Serial.print(','); Serial.print(motorNativeQueries);
  Serial.print(','); Serial.print(motorNativeResponses);
  Serial.print(','); Serial.print(motorNativeQueryFailures);
  Serial.print(','); Serial.print(motorNativeMalformedResponses);
  Serial.print(','); Serial.println(canConsecutiveFailures);
  xSemaphoreGive(serialMutex);
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
  while (!runtimeInitializationComplete) vTaskDelay(pdMS_TO_TICKS(10));
  // Finish boot and bring up the command/diagnostic path before sampling PWM.
  // A noisy or absent AS5600 channel must never prevent version/health queries.
  vTaskDelay(pdMS_TO_TICKS(40));
  primeSensorFilter();
  dbPrintln("BOOT|phase=as5600-prime|status=complete");

  TickType_t lastWake = xTaskGetTickCount();
  unsigned long lastTelemetry = 0;
  unsigned long lastHealth = 0;

  while (true) {
    sensorTaskLoops++;
    lastSensorTaskMs = millis();
    if (!isCenter && !isHead) {
      readSensors();
      computeAverages();
      normalizeReadings();

      // Preserve high-rate sensing without saturating the serial port.
      if (telemetryStreamingEnabled && millis() - lastTelemetry >= 20) {
        printReadings();
        lastTelemetry = millis();
      }
      if (telemetryStreamingEnabled && millis() - lastHealth >= 1000) {
        printHealthRecord();
        lastHealth = millis();
      }
    }
    vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(1));
  }
}

void updateMotorReferencedImpedance(int actuatorIndex, ImpedanceControl &controller,
                                    const JointConstraints &constraints,
                                    float outputDirection, bool enabled,
                                    unsigned long now) {
  if (!enabled) {
    impedanceTorqueValues[actuatorIndex] = 0;
    return;
  }

  float measuredDegrees = 0.0f;
  if (!readMotorControlDegrees(actuatorIndex, measuredDegrees)) {
    // Position feedback never falls back to an AS5600 once boot zeroing has
    // begun. Missing/stale/untrusted CAN feedback is a zero-torque condition.
    controller.reset();
    impedanceTorqueValues[actuatorIndex] = 0;
    return;
  }

  controller.update(measuredDegrees, now, constraints.minAngle, constraints.maxAngle);
  impedanceTorqueValues[actuatorIndex] = clampTorqueCommand(
    controller.torqueOutput * outputDirection);
}

void impedanceControlTask(void *parameter) {
  while (!runtimeInitializationComplete) vTaskDelay(pdMS_TO_TICKS(10));
  TickType_t lastWake = xTaskGetTickCount();

  while (true) {
    impedanceTaskLoops++;
    lastImpedanceTaskMs = millis();
    if (!isCenter && !isHead) {
      const unsigned long now = millis();

      if (operatingMode == OPERATING_HYPERSPAWN_ROUTE && hyperspawnControlMode == HS_CONTROL_POSITION) {
        applyHyperspawnPositionTargets();
      }

      const bool routedPosition = operatingMode == OPERATING_HYPERSPAWN_ROUTE &&
                                  hyperspawnControlMode == HS_CONTROL_POSITION;
      if (isLeft) {
        updateMotorReferencedImpedance(LEFT_OUTER_CALF, outerCalfControlLeft,
          outerCalfConstraintsLeft, directionMultiplierLeftOuterCalf,
          routedPosition || impedanceEnabledLeftOuterCalf, now);
        updateMotorReferencedImpedance(LEFT_INNER_CALF, innerCalfControlLeft,
          innerCalfConstraintsLeft, directionMultiplierLeftInnerCalf,
          routedPosition || impedanceEnabledLeftInnerCalf, now);
        updateMotorReferencedImpedance(LEFT_KNEE, kneeControlLeft,
          kneeConstraintsLeft, directionMultiplierLeftKnee,
          routedPosition || impedanceEnabledLeftKnee, now);
        updateMotorReferencedImpedance(LEFT_HIP_PITCH, hipPitchControlLeft,
          hipPitchConstraintsLeft, directionMultiplierLeftHipPitch,
          routedPosition || impedanceEnabledLeftHipPitch, now);
        updateMotorReferencedImpedance(LEFT_HIP_ROLL, hipRollControlLeft,
          hipRollConstraintsLeft, directionMultiplierLeftHipRoll,
          routedPosition || impedanceEnabledLeftHipRoll, now);
      } else {
        updateMotorReferencedImpedance(RIGHT_OUTER_CALF, outerCalfControlRight,
          outerCalfConstraintsRight, directionMultiplierRightOuterCalf,
          routedPosition || impedanceEnabledRightOuterCalf, now);
        updateMotorReferencedImpedance(RIGHT_INNER_CALF, innerCalfControlRight,
          innerCalfConstraintsRight, directionMultiplierRightInnerCalf,
          routedPosition || impedanceEnabledRightInnerCalf, now);
        updateMotorReferencedImpedance(RIGHT_KNEE, kneeControlRight,
          kneeConstraintsRight, directionMultiplierRightKnee,
          routedPosition || impedanceEnabledRightKnee, now);
        updateMotorReferencedImpedance(RIGHT_HIP_PITCH, hipPitchControlRight,
          hipPitchConstraintsRight, directionMultiplierRightHipPitch,
          routedPosition || impedanceEnabledRightHipPitch, now);
        updateMotorReferencedImpedance(RIGHT_HIP_ROLL, hipRollControlRight,
          hipRollConstraintsRight, directionMultiplierRightHipRoll,
          routedPosition || impedanceEnabledRightHipRoll, now);
      }
    }

    vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(10));
  }
}

void canOutputTask(void *parameter) {
  while (!runtimeInitializationComplete) vTaskDelay(pdMS_TO_TICKS(10));
  TickType_t lastWake = xTaskGetTickCount();

  while (true) {
    canTaskLoops++;
    lastCanTaskMs = millis();
    if (runtimeControlReady && !isCenter && !isHead) {
      const uint32_t batchStartedUs = micros();
      const int start = firstSelectedActuatorIndex();
      bool motionBatchAttempted = false;
      bool motionBatchOk = true;
      bool stopBatchAttempted = false;

      if (portalTorqueTestActive) {
        motionBatchAttempted = true;
        if ((int32_t)(portalTorqueTestUntilMs - millis()) <= 0) {
          portalTorqueTestActive = false;
          portalTorqueTestActuatorIndex = -1;
          portalTorqueTestValue = 0;
          portalTorqueTestUntilMs = 0;
          for (int i = start; i < ACTUATOR_COUNT; i += 2) {
            if (!sendTorqueCommand(ACTUATOR_IDS[i], 0)) {
              motionBatchOk = false;
              break;
            }
            paceCanCommandBurst();
          }
          requestStop(3);
          if (motionBatchOk) {
            appendWebLog("TORQUE PULSE COMPLETE: zero command + stop burst queued");
          }
        } else {
          // Bounded commissioning pulse: one selected motor receives the small
          // test command; every other motor owned by this leg receives zero.
          for (int i = start; i < ACTUATOR_COUNT; i += 2) {
            const int16_t value = (i == portalTorqueTestActuatorIndex)
              ? portalTorqueTestValue : 0;
            if (!sendTorqueCommand(ACTUATOR_IDS[i], value)) {
              motionBatchOk = false;
              break;
            }
            paceCanCommandBurst();
          }
        }
      } else if (calibrationOverrideActive) {
        motionBatchAttempted = true;
        // During direction calibration, command only the selected test joint and
        // explicitly zero every other motor on this leg.
        for (int i = start; i < ACTUATOR_COUNT; i += 2) {
          const int16_t value = (i == calibrationActuatorIndex) ? calibrationTorqueValue : 0;
          if (!sendTorqueCommand(ACTUATOR_IDS[i], value)) {
            motionBatchOk = false;
            break;
          }
          paceCanCommandBurst();
        }
      } else if (playMode) {
        motionBatchAttempted = true;
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
          if (!sendTorqueCommand(ACTUATOR_IDS[i], value)) {
            motionBatchOk = false;
            break;
          }
          paceCanCommandBurst();
        }
      } else if (stopBurstRemaining > 0) {
        stopBatchAttempted = true;
        bool stopBatchOk = true;
        for (int i = start; i < ACTUATOR_COUNT; i += 2) {
          if (!sendStopCommand(ACTUATOR_IDS[i])) {
            stopBatchOk = false;
          }
          paceCanCommandBurst();
        }
        // A missing actuator must not prevent STOP delivery to every other
        // motor or create an unbounded retry storm. Complete the configured
        // three full-leg attempts and expose any unconfirmed batch.
        if (!stopBatchOk) canStopBatchFailures++;
        --stopBurstRemaining;
      }
      if (motionBatchAttempted && !motionBatchOk) {
        failClosedCanMotion("unconfirmed MCP2515 transmit");
      }
      if (motionBatchAttempted || stopBatchAttempted) {
        const uint32_t batchDurationUs = micros() - batchStartedUs;
        canOutputBatchLastUs = batchDurationUs;
        if (batchDurationUs > canOutputBatchMaxUs) {
          canOutputBatchMaxUs = batchDurationUs;
        }
        if (batchDurationUs > CAN_OUTPUT_PERIOD_US) {
          canOutputDeadlineMisses++;
        }
      }
    }

    vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(10));
  }
}

void executeQueuedWebCommand(const WebCommand &item) {
  String command(item.text);
  command.trim();
  const String payload = payloadFromRoutedCommand(command);

  String capture;
  capture.reserve(1024);
  activeCommandCapture = &capture;
  appendWebLog(String("WEB #") + item.id + " > " + command);
  webCommandsProcessed++;

  expirePortalMotionLeaseIfNeeded();
  if (portalPayloadRequiresMotionUnlock(payload) && !portalMotionAuthorized()) {
    portalSafetyRejects++;
    appendWebLog(String("WEB #") + item.id + " < ERR|PORTAL_MOTION_LOCKED");
    activeCommandCapture = nullptr;
    return;
  }

  processRoutedCommand(command, "web");
  if (payload == "stop") lockPortalMotion(false, "STOP command accepted");

  activeCommandCapture = nullptr;
  if (!capture.length()) appendWebLog(String("WEB #") + item.id + " < (no textual response)");
}

void checkChiralityTask(void *parameter) {
  while (true) {
    commandTaskLoops++;
    lastCommandTaskMs = millis();
    // Bounded core-0 drain for records produced by the real-time CAN task.
    // Never print or take the web-log mutex from the CAN transport owner.
    flushDeferredCanDiagnosticLines(4);
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
  file.println("ConfigVersion:7");
  file.printf("MaxTorqueLimit:%.3f\n", maxTorqueLimit);
  file.printf("RawMode:%d\n", rawMode ? 1 : 0);
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
    } else if (line.startsWith("RawMode:")) {
      rawMode = line.substring(8).toInt() != 0;
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
      dbPrintf("DBCAL1,%s,error,valid_counts,%d,%d,%d,%d,%d\n",
               currentCommandAddress().c_str(), validCounts[0], validCounts[1],
               validCounts[2], validCounts[3], validCounts[4]);
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
    dbPrintf("DBCAL1,%s,ok,offsets,%d,%d,%d,%d,%d\n",
             currentCommandAddress().c_str(), offsetOuter, offsetInner,
             offsetHip, offsetKnee, offsetButt);
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

void printConfigurationRecords() {
  if (!isLegRole()) {
    dbPrintln("DBCFG1 configuration inspection is currently available for leg controllers only.");
    return;
  }
  const String role = currentCommandAddress();
  const int *offsets = isLeft ? leftLegOffsets : rightLegOffsets;
  const float directions[5] = {
    isLeft ? directionMultiplierLeftOuterCalf : directionMultiplierRightOuterCalf,
    isLeft ? directionMultiplierLeftInnerCalf : directionMultiplierRightInnerCalf,
    isLeft ? directionMultiplierLeftKnee : directionMultiplierRightKnee,
    isLeft ? directionMultiplierLeftHipPitch : directionMultiplierRightHipPitch,
    isLeft ? directionMultiplierLeftHipRoll : directionMultiplierRightHipRoll
  };
  dbPrintf("DBCFG1,%s,meta,%d,%s,%.3f,%d,%d,%d,%s\n",
           role.c_str(), configProvisioned ? 1 : 0, operatingModeName().c_str(),
           maxTorqueLimit, legacyUnaddressedCommands ? 1 : 0, rawMode ? 1 : 0,
           rebootRequired ? 1 : 0, DROPBEAR_FIRMWARE_VERSION);
  dbPrintf("DBCFG1,%s,hyperspawn,%lu,%d,%d,%.6f\n",
           role.c_str(), (unsigned long)hyperspawnCommandTimeoutMs,
           hyperspawnLegacyBroadcast ? 1 : 0, hyperspawnAutoArm ? 1 : 0,
           hyperspawnPositionUnitsPerDegree);
  dbPrintf("DBCFG1,%s,offsets,%d,%d,%d,%d,%d\n", role.c_str(),
           offsets[0], offsets[1], offsets[2], offsets[3], offsets[4]);
  dbPrintf("DBCFG1,%s,directions,%.1f,%.1f,%.1f,%.1f,%.1f\n", role.c_str(),
           directions[0], directions[1], directions[2], directions[3], directions[4]);

  const JointConstraints *constraints[6] = {
    isLeft ? &outerCalfConstraintsLeft : &outerCalfConstraintsRight,
    isLeft ? &innerCalfConstraintsLeft : &innerCalfConstraintsRight,
    isLeft ? &kneeConstraintsLeft : &kneeConstraintsRight,
    isLeft ? &hipPitchConstraintsLeft : &hipPitchConstraintsRight,
    isLeft ? &hipYawConstraintsLeft : &hipYawConstraintsRight,
    isLeft ? &hipRollConstraintsLeft : &hipRollConstraintsRight
  };
  const char *names[6] = {
    "outer_calf", "inner_calf", "knee", "hip_pitch", "hip_yaw", "hip_roll"
  };
  for (uint8_t i = 0; i < 6; ++i) {
    dbPrintf("DBCFG1,%s,constraint,%s,%d,%d\n", role.c_str(), names[i],
             constraints[i]->minAngle, constraints[i]->maxAngle);
  }
  dbPrintf("DBCFG1,%s,end\n", role.c_str());
}

bool processConfigurationSetCommand(const String &command) {
  if (!isLegRole()) {
    dbPrintln("Configuration mutation is currently available for leg controllers only.");
    return true;
  }
  if (command.startsWith("config set max_torque ")) {
    const float value = command.substring(22).toFloat();
    if (value <= 0.0f || value > 100.0f) {
      dbPrintln("Max torque must be >0 and <=100.");
      return true;
    }
    maxTorqueLimit = value;
    saveConfig();
    dbPrintf("DBCFG1,%s,updated,max_torque,%.3f\n",
             currentCommandAddress().c_str(), maxTorqueLimit);
    return true;
  }
  if (command.startsWith("config set offset ")) {
    String params = command.substring(18);
    params.trim();
    const int split = params.indexOf(' ');
    if (split <= 0) {
      dbPrintln("Usage: config set offset <outer_calf|inner_calf|hip_pitch|knee|hip_roll> <-720..720>");
      return true;
    }
    const String joint = params.substring(0, split);
    const String valueText = params.substring(split + 1);
    const int value = valueText.toInt();
    if ((value == 0 && valueText != "0") || value < -720 || value > 720) {
      dbPrintln("Offset must be an integer from -720 through 720.");
      return true;
    }
    int index = -1;
    if (joint == "outer_calf") index = 0;
    else if (joint == "inner_calf") index = 1;
    else if (joint == "hip_pitch") index = 2;
    else if (joint == "knee") index = 3;
    else if (joint == "hip_roll") index = 4;
    if (index < 0) {
      dbPrintln("Offset joint must be outer_calf, inner_calf, hip_pitch, knee, or hip_roll.");
      return true;
    }
    int *offsets = isLeft ? leftLegOffsets : rightLegOffsets;
    offsets[index] = value;
    saveConfig();
    dbPrintf("DBCFG1,%s,updated,offset,%s,%d\n",
             currentCommandAddress().c_str(), joint.c_str(), value);
    return true;
  }
  return false;
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

    const size_t received = Wire.requestFrom(
      static_cast<uint8_t>(IMU_DEVICE_ADDRESS), static_cast<size_t>(14), true
    );
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
  else if (command == "config show") printConfigurationRecords();
  else if (command.startsWith("config set ") && processConfigurationSetCommand(command)) {}
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

void processTorquePulseTestCommand(const String &command) {
  if (operatingMode == OPERATING_HYPERSPAWN_ROUTE || !runtimeControlReady ||
      isCenter || isHead || calibrationOverrideActive) {
    dbPrintln("Torque pulse rejected: standalone leg runtime must be ready and calibration idle.");
    return;
  }

  String params = command.substring(String("test_torque ").length());
  params.trim();
  const int firstSpace = params.indexOf(' ');
  const int secondSpace = firstSpace < 0 ? -1 : params.indexOf(' ', firstSpace + 1);
  if (firstSpace < 1 || secondSpace < firstSpace + 2) {
    dbPrintln("Usage: test_torque <joint> <command -25..25> <duration_ms 50..500>");
    return;
  }

  String joint = params.substring(0, firstSpace);
  String torqueText = params.substring(firstSpace + 1, secondSpace);
  String durationText = params.substring(secondSpace + 1);
  joint.trim(); torqueText.trim(); durationText.trim();
  const int index = jointNameToActuatorIndex(joint);
  const int requestedTorque = torqueText.toInt();
  const uint32_t durationMs = (uint32_t)durationText.toInt();
  if (index < 0 || !actuatorBelongsToSelectedLeg(index)) {
    dbPrintln("Torque pulse rejected: joint is not owned by this controller.");
    return;
  }
  if (requestedTorque == 0 || abs(requestedTorque) > PORTAL_TORQUE_TEST_MAX_COMMAND ||
      durationMs < 50 || durationMs > PORTAL_TORQUE_TEST_MAX_MS) {
    dbPrintln("Torque pulse rejected: require nonzero command -25..25 and duration 50..500 ms.");
    return;
  }

  bool feedbackReady = false;
  float measuredDegrees = 0.0f;
  if (as5600SensorIndexForActuator(index) >= 0) {
    feedbackReady = readMotorControlDegrees(index, measuredDegrees);
  } else {
    feedbackReady = motorNativeValid[index] &&
      millis() - motorNativeReceivedMs[index] <= MOTOR_NATIVE_STALE_MS;
    if (feedbackReady) measuredDegrees = motorNativeDegrees[index];
  }
  if (!feedbackReady) {
    dbPrintln("Torque pulse rejected: fresh verified motor-native angle feedback is required.");
    return;
  }

  clearAllTorqueSetpoints();
  playMode = false;
  stopBurstRemaining = 0;
  portalTorqueTestActuatorIndex = index;
  portalTorqueTestValue = clampTorqueCommand((float)requestedTorque);
  portalTorqueTestUntilMs = millis() + durationMs;
  portalTorqueTestActive = true;
  dbPrintf("OK|target=%s|command=test_torque|joint=%s|value=%d|duration_ms=%lu|position_deg=%.2f\n",
           currentCommandAddress().c_str(), joint.c_str(), portalTorqueTestValue,
           (unsigned long)durationMs, measuredDegrees);
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

  String payloadKind = payload;
  payloadKind.toLowerCase();
  if (payloadKind.startsWith("can ")) {
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

  if (command.equalsIgnoreCase("version") || command.equalsIgnoreCase("/version") ||
      command.equalsIgnoreCase("capabilities")) {
    xSemaphoreGive(serialMutex);
    printVersionRecord();
    return;
  }
  if (command.equalsIgnoreCase("health")) {
    xSemaphoreGive(serialMutex);
    printHealthRecord();
    return;
  }
  if (command.equalsIgnoreCase("observe on")) {
    telemetryStreamingEnabled = isLegRole();
    dbPrintf("DBO1,%s,%s\n", currentCommandAddress().c_str(),
             telemetryStreamingEnabled ? "on" : "unsupported");
    xSemaphoreGive(serialMutex);
    printVersionRecord();
    printHealthRecord();
    return;
  }
  if (command.equalsIgnoreCase("observe off")) {
    telemetryStreamingEnabled = false;
    dbPrintf("DBO1,%s,off\n", currentCommandAddress().c_str());
    xSemaphoreGive(serialMutex);
    return;
  }
  String commandKind = command;
  commandKind.toLowerCase();
  if (commandKind.startsWith("can ")) {
    xSemaphoreGive(serialMutex);
    processCanDiagnosticCommand(command);
    return;
  }

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
  } else if (command == "config show") {
    printConfigurationRecords();
  } else if (command.startsWith("config set ") && processConfigurationSetCommand(command)) {
    // Strict configuration setters persist one validated field at a time.
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
    saveConfig();
    dbPrintln("Raw mode enabled. Offsets bypassed.");
  } else if (command == "raw off") {
    rawMode = false;
    saveConfig();
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
  } else if (command.startsWith("test_torque ")) {
    processTorquePulseTestCommand(command);
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
  dbPrintln("  version | capabilities | health");
  dbPrintln("      Machine-readable DBV1/DBH1 identity and live subsystem health.");
  dbPrintln("  observe on | observe off");
  dbPrintln("      Start/stop DB3 state telemetry without enabling actuator output.");
  dbPrintln("  can info <motor_id>");
  dbPrintln("      Send the safe status/angle/encoder/PID/acceleration read suite.");
  dbPrintln("  can probe <motor_id> <read_opcode>");
  dbPrintln("  can tx-read <motor_id> <byte0> ... <byte7>");
  dbPrintln("      Send one allowlisted read frame; hex tokens may include 0x.");
  dbPrintln("  can monitor <motor_id> [100..10000_ms] | can monitor status | can monitor off");
  dbPrintln("      Emit matching same-ID or ID+0x100 RX frames as DBC1 records.");
  dbPrintln("  can sniff [100..2000_ms]");
  dbPrintln("      Passively emit up to 128 RX frames from every observed CAN ID.");
  dbPrintln("  can scan");
  dbPrintln("      Pause normal polling, read status 0x9A from RMD IDs 0x141..0x160,");
  dbPrintln("      and summarize same-ID versus ID+0x100 replies and TX failures.");
  dbPrintln("  can bus");
  dbPrintln("      Emit MCP2515 EFLG/TEC/REC, decoded fault names, and recovery counters.");
  dbPrintln("  can registers");
  dbPrintln("      Emit the CAN I/O task's cached MCP2515 mode/timing/mailbox registers.");
  dbPrintln("  can poll on | can poll off | can poll status");
  dbPrintln("      Start/stop only automatic 0x92 reads; actuator output remains disarmed.");
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
  dbPrintln("  config show");
  dbPrintln("      Emit complete leg configuration as bounded DBCFG1 records.");
  dbPrintln("  config set max_torque <0..100>");
  dbPrintln("  config set offset <outer_calf|inner_calf|hip_pitch|knee|hip_roll> <-720..720>");
  dbPrintln("      Persist one validated setting; intended for guarded dashboard use.");
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
  out += "\"firmware_version\":\"" + String(DROPBEAR_FIRMWARE_VERSION) + "\",";
  out += "\"command_address\":\"" + currentCommandAddress() + "\",";
  out += "\"command_protocol\":\"" + String(DROPBEAR_COMMAND_PROTOCOL) + "\",";
  out += "\"telemetry_protocol\":\"" + String(DROPBEAR_TELEMETRY_PROTOCOL) + "\",";
  out += "\"observation_streaming\":" + String(telemetryStreamingEnabled ? "true" : "false") + ",";
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
  out += "\"position_feedback_policy\":\"as5600_boot_zero_then_rmd_0x92\",";
  out += "\"max_torque\":" + String(maxTorqueLimit, 3) + ",";
  out += "\"heap\":" + String(ESP.getFreeHeap()) + ",";
  out += "\"uptime_ms\":" + String(millis()) + ",";
  out += "\"stop_burst\":" + String(stopBurstRemaining) + ",";
  out += "\"calibration_override\":" + String(calibrationOverrideActive ? "true" : "false") + ",";
  out += "\"portal_safety\":{";
  out += "\"stage\":" + String(portalSafetyStage) + ",";
  out += "\"motion_unlocked\":" + String(portalMotionAuthorized() ? "true" : "false") + ",";
  out += "\"lease_remaining_ms\":" + String(portalMotionRemainingMs()) + ",";
  out += "\"expected_phrase\":\"ENABLE " + currentCommandAddress() + "\",";
  out += "\"torque_test_active\":" + String(portalTorqueTestActive ? "true" : "false") + ",";
  out += "\"torque_test_actuator\":" + String(portalTorqueTestActuatorIndex) + ",";
  out += "\"torque_test_value\":" + String(portalTorqueTestValue) + ",";
  out += "\"unlocks\":" + String(portalSafetyUnlocks) + ",";
  out += "\"rejects\":" + String(portalSafetyRejects);
  out += "},";

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
  out += "\"raw_mode\":" + String(rawMode ? "true" : "false") + ",";

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
  if (!canInitialized || !canOneShotEnabled || !runtimeControlReady) return "fault";
  if (strcmp(taskHealthStatus(true, lastCanTaskMs, 40, 150), "fault") == 0) return "fault";
  if ((canErrorFlags & (MCP_EFLG_TXBO | MCP_EFLG_TXEP | MCP_EFLG_RXEP)) != 0) return "fault";
  if (canConsecutiveFailures >= 3) return "fault";
  if ((canErrorFlags & (MCP_EFLG_EWARN | MCP_EFLG_TXWAR | MCP_EFLG_RXWAR |
                        MCP_EFLG_RX0OVR | MCP_EFLG_RX1OVR)) != 0 ||
      canConsecutiveFailures > 0 || diagnosticAgeMs(lastCanFailureMs) < 5000) return "warn";
  if (!CAN_BIT_TIMING_DATASHEET_COMPLIANT) return "warn";
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
  static const char *actuatorJointNames[ACTUATOR_COUNT] = {
    "outer_calf", "outer_calf", "inner_calf", "inner_calf",
    "knee", "knee", "hip_pitch", "hip_pitch",
    "hip_yaw", "hip_yaw", "hip_roll", "hip_roll"
  };

  String out;
  out.reserve(24000);
  const uint32_t now = millis();
  const bool legRuntime = configProvisioned && !isCenter && !isHead && runtimeControlReady;
  const uint32_t queueDepth = webCommandQueue ? (uint32_t)uxQueueMessagesWaiting(webCommandQueue) : 0;

  out += "{";
  out += "\"timestamp_ms\":" + String(now) + ",";
  out += "\"firmware_version\":\"" + String(DROPBEAR_FIRMWARE_VERSION) + "\",";
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
  out += "\"one_shot_tx\":" + String(canOneShotEnabled ? "true" : "false") + ",";
  out += "\"bitrate\":" + String(CAN_BUS_BITRATE_BPS) + ",";
  out += "\"oscillator_hz\":" + String(CAN_CONTROLLER_CLOCK_HZ) + ",";
  out += "\"bit_timing_datasheet_compliant\":" +
         String(CAN_BIT_TIMING_DATASHEET_COMPLIANT ? "true" : "false") + ",";
  out += "\"bit_timing_profile\":\"dropbear_8MHz_1Mbps_CNF_00_80_80_single_sample\",";
  out += "\"cs_gpio\":" + String(CAN_CS_PIN) + ",\"int_gpio\":" + String((int)CAN0_INT) + ",";
  out += "\"int_level\":" + String(canInitialized ? digitalRead(CAN0_INT) : -1) + ",";
  out += "\"rx_interrupts\":" + String(canRxInterrupts) + ",";
  out += "\"rx_wakeups\":" + String(canRxWakeups) + ",";
  out += "\"rx_max_burst\":" + String(canRxMaxBurst) + ",";
  out += "\"tx_success\":" + String(canTxSuccess) + ",";
  out += "\"rx_frames\":" + String(canRxFrames) + ",";
  out += "\"rx_errors\":" + String(canRxErrors) + ",";
  out += "\"last_rx_age_ms\":" + ageJsonValue(lastCanRxMs) + ",";
  out += "\"tx_failure\":" + String(canTxFailure) + ",";
  out += "\"deferred_read_submissions\":" + String(canDeferredReadSubmissions) + ",";
  out += "\"deferred_tx_pending\":" + String(canDeferredTxPending ? "true" : "false") + ",";
  out += "\"tx_abort_attempts\":" + String(canTxAbortAttempts) + ",";
  out += "\"tx_abort_successes\":" + String(canTxAbortSuccesses) + ",";
  out += "\"tx_abort_failures\":" + String(canTxAbortFailures) + ",";
  out += "\"wire_errors\":" + String(canTxWireErrors) + ",";
  out += "\"arbitration_losses\":" + String(canTxArbitrationLosses) + ",";
  out += "\"controller_aborts\":" + String(canTxControllerAborts) + ",";
  out += "\"rx_overflow_events\":" + String(canRxOverflowEvents) + ",";
  out += "\"rx_overflow_clears\":" + String(canRxOverflowClears) + ",";
  out += "\"polling_enabled\":" + String(motorNativePollingEnabled ? "true" : "false") + ",";
  out += "\"controller_mode\":" + String(canControllerMode) + ",";
  out += "\"tx_queue_depth\":" + String(canTxQueue ? uxQueueMessagesWaiting(canTxQueue) : 0) + ",";
  out += "\"tx_queue_accepted\":" + String(canTxQueueAccepted) + ",";
  out += "\"tx_queue_completed\":" + String(canTxQueueCompleted) + ",";
  out += "\"tx_queue_full\":" + String(canTxQueueFull) + ",";
  out += "\"tx_queue_expired\":" + String(canTxQueueExpired) + ",";
  out += "\"tx_caller_timeouts\":" + String(canTxCallerTimeouts) + ",";
  out += "\"tx_queue_latency_last_us\":" + String(canTxQueueLatencyLastUs) + ",";
  out += "\"tx_queue_latency_max_us\":" + String(canTxQueueLatencyMaxUs) + ",";
  out += "\"tx_execution_last_us\":" + String(canTxExecutionLastUs) + ",";
  out += "\"tx_execution_max_us\":" + String(canTxExecutionMaxUs) + ",";
  out += "\"output_batch_last_us\":" + String(canOutputBatchLastUs) + ",";
  out += "\"output_batch_max_us\":" + String(canOutputBatchMaxUs) + ",";
  out += "\"output_deadline_misses\":" + String(canOutputDeadlineMisses) + ",";
  out += "\"stop_batch_failures\":" + String(canStopBatchFailures) + ",";
  out += "\"io_loop_last_us\":" + String(canIoLoopLastUs) + ",";
  out += "\"io_loop_max_us\":" + String(canIoLoopMaxUs) + ",";
  out += "\"io_loop_budget_misses\":" + String(canIoLoopBudgetMisses) + ",";
  out += "\"rx_drain_last_us\":" + String(canRxDrainLastUs) + ",";
  out += "\"rx_drain_max_us\":" + String(canRxDrainMaxUs) + ",";
  out += "\"diagnostic_queue_depth\":" + String(canDiagnosticLineQueue ? uxQueueMessagesWaiting(canDiagnosticLineQueue) : 0) + ",";
  out += "\"diagnostic_lines_queued\":" + String(canDiagnosticLinesQueued) + ",";
  out += "\"diagnostic_lines_drained\":" + String(canDiagnosticLinesDrained) + ",";
  out += "\"diagnostic_line_drops\":" + String(canDiagnosticLineDrops) + ",";
  out += "\"motion_fail_closed\":" + String(canMotionFailClosed) + ",";
  out += "\"consecutive_failures\":" + String(canConsecutiveFailures) + ",";
  out += "\"error_flags\":" + String(canErrorFlags) + ",";
  out += "\"tx_error_count\":" + String(canTxErrorCount) + ",";
  out += "\"rx_error_count\":" + String(canRxErrorCount) + ",";
  out += "\"recovery_attempts\":" + String(canRecoveryAttempts) + ",";
  out += "\"recovery_successes\":" + String(canRecoverySuccesses) + ",";
  out += "\"last_recovery_age_ms\":" + ageJsonValue(lastCanRecoveryMs) + ",";
  out += "\"mutex_timeouts\":" + String(canMutexTimeouts) + ",";
  out += "\"torque_frames\":" + String(canTorqueFrames) + ",";
  out += "\"stop_frames\":" + String(canStopFrames) + ",";
  out += "\"last_result\":" + String(lastCanResult) + ",";
  out += "\"last_tx_age_ms\":" + ageJsonValue(lastCanTxMs) + ",";
  out += "\"last_failure_age_ms\":" + ageJsonValue(lastCanFailureMs) + ",";
  out += "\"feedback\":\"rmd_v17_v42_0x92_multi_turn\",";
  out += "\"position_feedback_policy\":\"as5600_boot_zero_then_can_continuous\",";
  out += "\"motor_angle_queries\":" + String(motorNativeQueries) + ",";
  out += "\"motor_angle_query_failures\":" + String(motorNativeQueryFailures) + ",";
  out += "\"motor_angle_responses\":" + String(motorNativeResponses) + ",";
  out += "\"motor_angle_malformed\":" + String(motorNativeMalformedResponses) + ",";
  out += "\"motor_angle_reply_timeouts\":" + String(motorNativeReplyTimeouts) + ",";
  out += "\"motor_angle_out_of_order\":" + String(motorNativeOutOfOrderResponses) + ",";
  out += "\"motor_angle_offline_skips\":" + String(motorNativeOfflineSkips) + ",";
  out += "\"motor_angle_pending_index\":" + String(motorNativePendingIndex);
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
  out += "\"portal_safety_stage\":" + String(portalSafetyStage) + ",";
  out += "\"portal_motion_unlocked\":" + String(portalMotionAuthorized() ? "true" : "false") + ",";
  out += "\"portal_lease_remaining_ms\":" + String(portalMotionRemainingMs()) + ",";
  out += "\"portal_unlocks\":" + String(portalSafetyUnlocks) + ",";
  out += "\"portal_rejects\":" + String(portalSafetyRejects) + ",";
  out += "\"torque_test_active\":" + String(portalTorqueTestActive ? "true" : "false") + ",";
  out += "\"torque_test_actuator_index\":" + String(portalTorqueTestActuatorIndex) + ",";
  out += "\"torque_test_value\":" + String(portalTorqueTestValue) + ",";
  out += "\"reboot_required\":" + String(rebootRequired ? "true" : "false") + ",";
  out += "\"command_watchdog\":\"" + String(operatingMode == OPERATING_HYPERSPAWN_ROUTE ? (hyperspawnWatchdogTripped ? "tripped" : "armed") : "inactive") + "\",";
  out += "\"can_feedback_monitoring\":\"rmd_v17_v42_0x92_multi_turn\"";
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
    const bool motorFeedbackFresh = motorNativeValid[i] &&
      now - motorNativeReceivedMs[i] <= MOTOR_NATIVE_STALE_MS;
    const dropbear::MotorProfile *profile = motorProfileForActuator(i);
    float controlPositionDegrees = 0.0f;
    const bool controlFeedbackReady = readMotorControlDegrees(i, controlPositionDegrees);
    const int sensorIndex = as5600SensorIndexForActuator(i);
    const bool hasExternalReference = sensorIndex >= 0;
    const bool externalValueBelongsToActuator = hasExternalReference &&
      actuatorBelongsToSelectedLeg(i);
    const SensorDiagnostic *external = externalValueBelongsToActuator
      ? &sensorDiagnostics[sensorIndex] : nullptr;
    const bool externalFresh = external != nullptr && external->signalValid &&
      external->pulseAgeUs <= AS5600_STALE_US &&
      diagnosticAgeMs(external->lastSampleMs) <= 100;
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
    out += "\"side\":\"" + String((i & 1) ? "left" : "right") + "\",";
    out += "\"appendage\":\"" + String(actuatorJointNames[i]) + "\",";
    out += "\"status\":\"" + String(status) + "\",";
    out += "\"selected\":" + String(selected ? "true" : "false") + ",";
    out += "\"can_id\":\"" + String(idHex) + "\",";
    out += "\"motor_model\":\"" + String(profile ? profile->model : "unknown") + "\",";
    out += "\"motor_protocol\":\"" + String(profile ? profile->protocolVersion : "unknown") + "\",";
    out += "\"gear_ratio\":" + String(profile ? profile->reductionRatio : 0.0f, 2) + ",";
    out += "\"angle_reference\":\"" + String(profile
      ? dropbear::angleReferenceName(profile->angleReference) : "unknown") + "\",";
    out += "\"angle_payload\":\"" + String(profile
      ? dropbear::angleLayoutName(profile->angleLayout) : "unknown") + "\",";
    out += "\"feedback\":\"" + String(motorFeedbackFresh ? "measured" :
      (motorNativeValid[i] ? "stale" : "unavailable")) + "\",";
    out += "\"motor_position_deg\":";
    out += motorNativeValid[i] ? String(motorNativeDegrees[i], 2) : String("null");
    out += ",";
    out += "\"feedback_age_ms\":" + ageJsonValue(motorNativeReceivedMs[i]) + ",";
    out += "\"query_miss_streak\":" + String(motorNativeMissStreak[i]) + ",";
    out += "\"query_pending\":" + String(motorNativePendingIndex == i ? "true" : "false") + ",";
    out += "\"control_feedback\":\"" + String(
      motorControlAlignmentFault[i] ? "alignment_fault" :
      (controlFeedbackReady ? "motor_native_zeroed" :
       (hasExternalReference ? "zeroing_from_as5600" : "motor_telemetry_only"))) + "\",";
    out += "\"control_position_deg\":";
    out += controlFeedbackReady ? String(controlPositionDegrees, 2) : String("null");
    out += ",";
    out += "\"boot_zero_offset_deg\":";
    out += motorControlZeroed[i] ? String(motorControlZeroOffsetDegrees[i], 2) : String("null");
    out += ",";
    out += "\"as5600_crosscheck_error_deg\":";
    out += hasExternalReference && motorControlZeroed[i]
      ? String(motorAs5600ErrorDegrees[i], 2) : String("null");
    out += ",";
    out += "\"as5600\":{";
    out += "\"present\":" + String(hasExternalReference ? "true" : "false") + ",";
    out += "\"sensor\":\"" + String(hasExternalReference ? sensorNames[sensorIndex] : "none") + "\",";
    out += "\"gpio\":" + String(hasExternalReference ? sensorPins[sensorIndex] : -1) + ",";
    out += "\"status\":\"" + String(!hasExternalReference ? "not_installed" :
      (!externalValueBelongsToActuator ? "other_controller" :
       (externalFresh ? "measured" : "unavailable"))) + "\",";
    out += "\"signal_valid\":" + String(external != nullptr && external->signalValid ? "true" : "false") + ",";
    out += "\"angle_deg\":";
    out += externalFresh ? String(normalizedAs5600Degrees(sensorIndex), 2) : String("null");
    out += ",\"sample_age_ms\":";
    out += external != nullptr ? ageJsonValue(external->lastSampleMs) : String("-1");
    out += ",\"pulse_age_us\":" + String(external != nullptr ? external->pulseAgeUs : UINT32_MAX);
    out += "},";
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

String buildVersionJson() {
  String out;
  out.reserve(640);
  out += "{\"schema\":\"DBV1\",";
  out += "\"role\":\"" + currentCommandAddress() + "\",";
  out += "\"firmware\":\"" + String(DROPBEAR_FIRMWARE_VERSION) + "\",";
  out += "\"command_protocol\":\"" + String(DROPBEAR_COMMAND_PROTOCOL) + "\",";
  out += "\"telemetry_protocol\":\"" + String(DROPBEAR_TELEMETRY_PROTOCOL) + "\",";
  out += "\"capabilities\":\"" + String(DROPBEAR_CAPABILITIES) + "\",";
  out += "\"observation_streaming\":" + String(telemetryStreamingEnabled ? "true" : "false");
  out += "}";
  return out;
}

void handleApiVersion() {
  portalHttpRequests++;
  sendNoCacheHeaders();
  server.send(200, "application/json", buildVersionJson());
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

  expirePortalMotionLeaseIfNeeded();
  if (portalPayloadRequiresMotionUnlock(apiPayload) && !portalMotionAuthorized()) {
    portalSafetyRejects++;
    server.send(423, "application/json",
                "{\"ok\":false,\"error\":\"portal_motion_locked\",\"required\":\"three_stage_unlock\"}");
    return;
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

String portalSafetyJson() {
  return "{\"ok\":true,\"stage\":" + String(portalSafetyStage) +
         ",\"motion_unlocked\":" + String(portalMotionAuthorized() ? "true" : "false") +
         ",\"lease_remaining_ms\":" + String(portalMotionRemainingMs()) +
         ",\"expected_phrase\":\"ENABLE " + currentCommandAddress() + "\"}";
}

void handleApiSafetyAdvance() {
  portalHttpRequests++;
  sendNoCacheHeaders();
  if (!validateApiTargetArg()) return;
  expirePortalMotionLeaseIfNeeded();
  const int requestedStage = server.hasArg("stage") ? server.arg("stage").toInt() : 0;

  bool accepted = false;
  if (requestedStage == 1 && portalSafetyStage == 0) {
    portalSafetyStage = 1;
    accepted = true;
  } else if (requestedStage == 2 && portalSafetyStage == 1) {
    portalSafetyStage = 2;
    accepted = true;
  } else if (requestedStage == 3 && portalSafetyStage == 2) {
    const String expected = "ENABLE " + currentCommandAddress();
    const String phrase = server.hasArg("phrase") ? server.arg("phrase") : "";
    const bool runtimeReady = isHead ? runtimeNeckReady :
      ((!isCenter) && runtimeControlReady && operatingMode == OPERATING_STANDALONE);
    if (phrase == expected && runtimeReady && !rebootRequired) {
      portalSafetyStage = 3;
      portalMotionLeaseUntilMs = millis() + PORTAL_MOTION_LEASE_MS;
      portalSafetyUnlocks++;
      accepted = true;
      appendWebLog("PORTAL SAFETY UNLOCK: 90-second motion lease granted");
    }
  }

  if (!accepted) {
    portalSafetyRejects++;
    lockPortalMotion(true, "invalid or out-of-order safety acknowledgement");
    server.send(409, "application/json",
                "{\"ok\":false,\"error\":\"safety_stage_rejected\",\"stage\":" +
                String(portalSafetyStage) + "}");
    return;
  }
  server.send(200, "application/json", portalSafetyJson());
}

void handleApiSafetyLock() {
  portalHttpRequests++;
  sendNoCacheHeaders();
  if (!validateApiTargetArg()) return;
  lockPortalMotion(true, "operator locked portal motion");
  server.send(200, "application/json", portalSafetyJson());
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
button:disabled,input:disabled{opacity:.42;cursor:not-allowed}.safety-steps{display:grid;grid-template-columns:repeat(3,minmax(180px,1fr));gap:8px}.safety-step{border:1px solid var(--line);padding:10px;border-radius:4px}.safety-step.done{border-color:#3c5840;background:#172219}
input,select,textarea{background:#0c0f11;color:var(--text);border:1px solid #30363b;border-radius:3px;padding:8px;font-size:12px;min-width:0}input[type=number]{width:88px}select{min-width:96px}
textarea{width:100%;min-height:220px;font-family:ui-monospace,SFMono-Regular,Menlo,Consolas,monospace;resize:vertical}.joint{display:grid;grid-template-columns:120px 80px 1fr 1fr;gap:8px;align-items:center;padding:8px 0;border-bottom:1px solid #20252a}
.joint:last-child{border-bottom:0}.jointname{font-size:12px}.canid{color:var(--muted);font-family:monospace;font-size:11px}.tiny{font-size:10px;color:var(--muted)}
table{width:100%;border-collapse:collapse;font-size:11px}th,td{text-align:left;border-bottom:1px solid #22282c;padding:7px 5px}th{color:var(--muted);font-weight:500}
.console{background:#050607;border:1px solid #252a2e;border-radius:4px;height:360px;overflow:auto;padding:10px;font:11px/1.55 ui-monospace,SFMono-Regular,Menlo,Consolas,monospace;white-space:pre-wrap}
.notice{padding:9px;border:1px solid #654d2b;background:#2d2519;color:#e6c58d;border-radius:4px;font-size:11px}.okmsg{padding:9px;border:1px solid #38523b;background:#172219;color:#acd0ae;border-radius:4px;font-size:11px}
.section-title{font-size:11px;color:#aeb7bd;text-transform:uppercase;letter-spacing:.08em;margin:8px 0}.cfgline{display:grid;grid-template-columns:170px repeat(5,1fr);gap:6px;align-items:center}.constraints{display:grid;grid-template-columns:repeat(2,minmax(260px,1fr));gap:8px}
.constraint{display:grid;grid-template-columns:1fr 82px 82px;gap:6px;align-items:center}.file{display:flex;justify-content:space-between;gap:8px;padding:7px 0;border-bottom:1px solid #22282c;font-size:11px}
.diagcards{display:grid;grid-template-columns:repeat(3,minmax(220px,1fr));gap:10px}.diagcard{border:1px solid var(--line);background:var(--panel2);border-radius:4px;padding:11px}.diaghead{display:flex;align-items:center;justify-content:space-between;gap:8px;margin-bottom:9px}.diagtitle{font-size:12px;font-weight:650;text-transform:uppercase;letter-spacing:.06em}.health{display:inline-block;border:1px solid var(--line);padding:3px 6px;border-radius:3px;font-size:9px;text-transform:uppercase;letter-spacing:.08em}.health.ok{color:#b5d5b6;border-color:#3c5840}.health.warn,.health.idle{color:#e6c381;border-color:#625032}.health.fault{color:#efadad;border-color:#673f3f}.health.inactive{color:#8d979e;border-color:#343b41}.kv{display:grid;grid-template-columns:1fr auto;gap:5px 12px;font-size:10px}.kv .k{color:var(--muted)}.kv .v{font-family:ui-monospace,SFMono-Regular,Menlo,Consolas,monospace;text-align:right}.diagtable{overflow:auto}.diagtable table{min-width:850px}.diagtable td.mono{font-family:ui-monospace,SFMono-Regular,Menlo,Consolas,monospace}.diag-note{font-size:10px;color:var(--muted);line-height:1.5}.overall{font-size:30px;font-weight:700;letter-spacing:.08em}.overall.ok{color:#b5d5b6}.overall.warn{color:#e6c381}.overall.fault{color:#efadad}
.telemetry-sources{display:grid;grid-template-columns:1fr;gap:12px}.telemetry-source{border:1px solid var(--line);background:var(--panel2);border-radius:4px;overflow:hidden}.telemetry-source-head{display:flex;justify-content:space-between;gap:12px;align-items:flex-start;padding:11px 12px;border-bottom:1px solid var(--line)}.telemetry-source-head h3{font-size:11px;letter-spacing:.08em;text-transform:uppercase;margin:0 0 4px}.telemetry-source-head p{font-size:10px;color:var(--muted);line-height:1.45;margin:0}.telemetry-source .diagtable{padding:0 7px 7px}.telemetry-source .diagtable table{min-width:760px}.source-kind{font:600 9px ui-monospace,SFMono-Regular,Menlo,Consolas,monospace;color:var(--muted);border:1px solid var(--line);padding:4px 6px;white-space:nowrap}.value-main{font:600 12px ui-monospace,SFMono-Regular,Menlo,Consolas,monospace}.value-sub{display:block;color:var(--muted);font-size:9px;margin-top:2px}.model-name{font-weight:650}.model-detail{display:block;color:var(--muted);font-size:9px;margin-top:2px}
@media(max-width:800px){.diagcards{grid-template-columns:1fr}}
@media(max-width:800px){.span8,.span6,.span4,.span3{grid-column:span 12}.joint{grid-template-columns:1fr 80px}.joint .controls{grid-column:1/-1}.cfgline{grid-template-columns:1fr repeat(2,1fr)}.cfgline input:nth-of-type(n+3){margin-top:2px}.constraints,.safety-steps{grid-template-columns:1fr}}
</style>
</head>
<body>
<div class="shell">
 <div class="top">
  <div><h1>DROPBEAR // <span id="roleTitle">...</span></h1><div class="sub">ESP32 low-level control · captive portal · <span id="operatingText">STANDALONE</span> · <span id="firmwareText">FW …</span> <span id="ipText"></span></div></div>
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
   <div class="card span12">
    <div class="row between"><div><h2>Portal motion interlock</h2><div class="tiny">Motion starts locked after every boot. Complete the stages in order for a 90-second lease. STOP remains available at all times.</div></div><span id="safetyLease" class="health fault">LOCKED</span></div>
    <div class="safety-steps" style="margin-top:10px">
     <div id="safetyStep1" class="safety-step"><div class="tiny">STAGE 1</div><div>Robot is mechanically supported.</div><button id="safetyButton1" onclick="advanceSafety(1)">CONFIRM SUPPORT</button></div>
     <div id="safetyStep2" class="safety-step"><div class="tiny">STAGE 2</div><div>E-stop and power disconnect are ready.</div><button id="safetyButton2" onclick="advanceSafety(2)" disabled>CONFIRM E-STOP</button></div>
     <div id="safetyStep3" class="safety-step"><div class="tiny">STAGE 3</div><div>Type the exact controller phrase.</div><input id="safetyPhrase" autocomplete="off" spellcheck="false" placeholder="ENABLE LEFTLEG"><button id="safetyButton3" onclick="advanceSafety(3)" disabled>UNLOCK 90 SECONDS</button></div>
    </div>
    <div class="row" style="margin-top:10px"><button class="danger" onclick="lockSafety()">LOCK + STOP NOW</button><span id="safetyDetail" class="tiny">No web motion authority.</span></div>
   </div>
   <div class="card span12"><div class="row between"><div class="row"><button class="danger" onclick="cmd('stop')">STOP</button><button class="primary requires-motion-unlock" onclick="cmd('play')">PLAY</button><button onclick="cmd('zero')">ZERO TORQUE</button><button onclick="cmd('status')">STATUS</button></div><div class="tiny">All web commands pass through the same command queue/parser as USB Serial.</div></div></div>
   <div class="card span12" id="legStateCard">
    <div class="row between"><div><h2>Live appendage state</h2><div class="diag-note">Motor CAN position and AS5600 PWM position are independent measurements. Aligned control angle is shown separately and is never substituted for stale motor feedback.</div></div><span id="liveRoleBadge" class="health inactive">WAITING</span></div>
    <div class="telemetry-sources" style="margin-top:11px">
     <section class="telemetry-source">
      <div class="telemetry-source-head"><div><h3>Motor output-shaft state</h3><p>Every owned CAN actuator, decoded by its installed model/firmware profile.</p></div><span class="source-kind">RMD CAN · 0x92</span></div>
      <div id="liveMotorState" class="diagtable"></div>
     </section>
     <section class="telemetry-source">
      <div class="telemetry-source-head"><div><h3>AS5600 encoder state</h3><p>Five independent one-wire PWM sensors. Hip yaw has no AS5600.</p></div><span class="source-kind">GPIO PWM</span></div>
      <div id="liveEncoderState" class="diagtable"></div>
     </section>
    </div>
   </div>
   <div class="card span12" id="legActuatorCard">
    <div class="row between"><h2>Actuator control</h2><span class="tiny">Torque values are firmware command units; global clamp follows MaxTorqueLimit × 100.</span></div>
    <div id="jointControls"></div>
   </div>
   <div class="card span12" id="neckControlCard" style="display:none">
    <div class="row between"><div><h2>Head / neck Stewart platform</h2><div class="tiny">Six A4988/NEMA17 axes via FastAccelStepper. Position is OPEN LOOP step count unless physical feedback is added.</div></div><div class="row"><button class="danger" onclick="cmd('stop')">STOP ALL</button><button class="requires-motion-unlock" onclick="cmd('HOME_SOFT')">HOME SOFT</button><button class="warn requires-motion-unlock" onclick="cmd('HOME_BRUTE')">HOME BRUTE</button><button onclick="cmd('neck zero')">SOFTWARE ZERO</button></div></div>
    <div class="section-title">Pose command</div><div class="row"><label>X <input id="nx" type="number" value="0"></label><label>Y <input id="ny" type="number" value="0"></label><label>Z <input id="nz" type="number" value="0"></label><label>H mm <input id="nh" type="number" value="0"></label><label>Roll <input id="nr" type="number" value="0"></label><label>Pitch <input id="np" type="number" value="0"></label><label>Speed × <input id="ns" type="number" step=".1" value="1"></label><label>Accel × <input id="na" type="number" step=".1" value="1"></label><button class="primary requires-motion-unlock" onclick="sendNeckPose()">Move pose</button></div>
    <div class="section-title">Actuators</div><div id="neckMotors"></div>
   </div>
  </div>
 </section>

 <section id="diagnostics" class="view">
  <div class="grid">
   <div class="card span12">
    <div class="row between"><div><h2>Runtime diagnostic wrapper</h2><div class="diag-note">Health is computed from actual runtime acquisition and transmit points. Motor rows include RMD 0x92 CAN replies, AS5600 boot alignment, continuous motor-native control angle, cross-check error, command source, and CAN TX status.</div></div><button onclick="loadDiagnostics()">Refresh now</button></div>
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
function renderLiveTelemetry(){
 const d=diagnostics;if(!d)return;
 const role=String(d.role||'').toLowerCase();
 const side=role.includes('left')?'left':role.includes('right')?'right':'';
 const badge=document.getElementById('liveRoleBadge');
 if(badge){badge.textContent=side?`${side.toUpperCase()}LEG · ${d.overall==='ok'?'LIVE':'CHECK'}`:'NO LEG ROLE';badge.className='health '+(side?healthClass(d.overall):'inactive')}
 const motors=(d.actuators||[]).filter(x=>x.side===side);
 const motorRows=motors.map(x=>{
  const feedbackHealth=x.control_feedback==='alignment_fault'||x.feedback==='stale'?'fault':x.feedback==='measured'?(x.control_position_deg==null?'warn':'ok'):'fault';
  const ratio=Number(x.gear_ratio)>0?`${Number(x.gear_ratio).toFixed(0)}:1`:'—';
  return `<tr><td>${healthPill(feedbackHealth)}</td><td><span class="model-name">${x.appendage.replaceAll('_',' ')}</span><span class="model-detail">${x.can_id}</span></td><td><span class="model-name">${x.motor_model}</span><span class="model-detail">FW ${x.motor_protocol} · ${ratio} · ${x.angle_reference.replaceAll('_',' ')}</span></td><td><span class="value-main">${x.motor_position_deg==null?'—':fmtDeg(x.motor_position_deg)}</span><span class="value-sub">${x.feedback.toUpperCase()} · ${age(x.feedback_age_ms)}</span></td><td><span class="value-main">${x.control_position_deg==null?'—':fmtDeg(x.control_position_deg)}</span><span class="value-sub">${x.control_feedback.replaceAll('_',' ')}</span></td><td><span class="value-main">${x.last_sent}</span><span class="value-sub">${x.command_source.replaceAll('_',' ')} · ${x.last_opcode}</span></td></tr>`
 }).join('');
 const motorBox=document.getElementById('liveMotorState');
 if(motorBox)motorBox.innerHTML=`<table><thead><tr><th>State</th><th>Appendage / CAN</th><th>Installed motor</th><th>Raw motor 0x92</th><th>Aligned control</th><th>Command</th></tr></thead><tbody>${motorRows||'<tr><td colspan="6">No active LEFTLEG or RIGHTLEG role.</td></tr>'}</tbody></table>`;
 const sensors=d.sensors||[];
 const sensorRows=sensors.map(x=>`<tr><td>${healthPill(x.status)}</td><td><span class="model-name">${x.name.replaceAll('_',' ')}</span><span class="model-detail">independent joint encoder</span></td><td class="mono">GPIO ${x.gpio}</td><td><span class="value-main">${x.signal_valid?fmtDeg(x.angle):'—'}</span><span class="value-sub">raw ${x.raw} · filtered ${Number(x.filtered_raw).toFixed(1)}</span></td><td><span class="value-main">${x.signal_valid?Number(x.frequency_hz||0).toFixed(1)+' Hz':'NO SIGNAL'}</span><span class="value-sub">duty ${Number(x.duty_percent||0).toFixed(2)}% · pulse ${x.pulse_age_us===4294967295?'never':((x.pulse_age_us||0)/1000).toFixed(1)+' ms'}</span></td><td>${age(x.sample_age_ms)}</td><td>${x.signal_valid?'PWM VALID':'PWM INVALID'}</td></tr>`).join('');
 const sensorBox=document.getElementById('liveEncoderState');
 if(sensorBox)sensorBox.innerHTML=`<table><thead><tr><th>State</th><th>Appendage</th><th>Input</th><th>AS5600 angle</th><th>PWM signal</th><th>Sample age</th><th>Signal</th></tr></thead><tbody>${sensorRows}</tbody></table>`;
}
function renderDiagnostics(){
 const d=diagnostics;if(!d)return;
 renderLiveTelemetry();
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
  moduleCard('MCP2515 / CAN',m.can,[['initialized',boolText(m.can?.initialized)],['bus','1 Mbps @ 8 MHz'],['CS / INT',`${m.can?.cs_gpio} / ${m.can?.int_gpio}`],['TX ok / fail',`${m.can?.tx_success} / ${m.can?.tx_failure}`],['RX frames',m.can?.rx_frames],['angle query ok / fail',`${m.can?.motor_angle_queries} / ${m.can?.motor_angle_query_failures}`],['angle responses',m.can?.motor_angle_responses],['last TX',age(m.can?.last_tx_age_ms)],['feedback',m.can?.feedback]])+
  moduleCard('I²C / IMU mux',m.i2c,[['initialized',boolText(m.i2c?.initialized)],['SDA',m.i2c?.sda],['SCL',m.i2c?.scl],['TCA9548A',m.i2c?.mux_seen?'0x70 present':'not seen'],['mux ok / fail',`${m.i2c?.mux_select_ok} / ${m.i2c?.mux_select_fail}`]])+
  moduleCard('HyperSpawn / ROS2 route',d.hyperspawn_route,[['active',boolText(d.hyperspawn_route?.active)],['node',d.hyperspawn_route?.node_id],['control',d.hyperspawn_route?.control_mode],['last command',age(d.hyperspawn_route?.last_command_age_ms)],['RX targeted',d.hyperspawn_route?.targeted_rx],['RX legacy',d.hyperspawn_route?.legacy_rx],['completed cmds',d.hyperspawn_route?.completed_commands],['fragment timeouts',d.hyperspawn_route?.fragment_timeouts],['state TX',d.hyperspawn_route?.state_tx],['watchdog trips',d.hyperspawn_route?.watchdog_trips]])+
  moduleCard('Command transport',m.commands,[['queue',`${m.commands?.queue_depth} / ${m.commands?.queue_capacity}`],['web queued',m.commands?.web_queued],['web processed',m.commands?.web_processed],['queue full',m.commands?.queue_full_events],['serial processed',m.commands?.serial_processed]])+
  moduleCard('Head / neck',m.neck,[['runtime',boolText(m.neck?.runtime_ready)],['motion',boolText(m.neck?.motion_enabled)],['software homed',boolText(m.neck?.software_homed)],['feedback',m.neck?.feedback],['speed Hz',m.neck?.speed_hz],['acceleration',m.neck?.accel],['Bluetooth',m.neck?.bluetooth]]);
 const sensors=d.sensors||[];
 document.getElementById('diagSensors').innerHTML=`<table><thead><tr><th>Health</th><th>Sensor</th><th>GPIO</th><th>Raw</th><th>Filtered</th><th>Angle</th><th>PWM duty</th><th>PWM Hz</th><th>Pulse age</th><th>Sample age</th><th>Observed raw range</th><th>Constraint</th><th>Notes</th></tr></thead><tbody>${sensors.map(x=>`<tr><td>${healthPill(x.status)}</td><td>${x.name.replaceAll('_',' ')}</td><td class="mono">${x.gpio} / PWM</td><td class="mono">${x.raw}</td><td class="mono">${Number(x.filtered_raw).toFixed(1)}</td><td class="mono">${Number(x.angle).toFixed(1)}°</td><td class="mono">${Number(x.duty_percent||0).toFixed(2)}%</td><td class="mono">${Number(x.frequency_hz||0).toFixed(1)}</td><td>${x.pulse_age_us===4294967295?'never':((x.pulse_age_us||0)/1000).toFixed(1)+' ms'}</td><td>${age(x.sample_age_ms)}</td><td class="mono">${x.min_raw_seen}…${x.max_raw_seen}</td><td class="mono">${x.constraint_min}…${x.constraint_max}°</td><td>${x.signal_valid?'PWM VALID':'PWM INVALID'} · change ${age(x.last_change_age_ms)}</td></tr>`).join('')}</tbody></table>`;
 const acts=d.actuators||[];
 document.getElementById('diagActuators').innerHTML=`<table><thead><tr><th>TX path</th><th>Actuator</th><th>Motor profile</th><th>CAN ID</th><th>Owned</th><th>Source</th><th>Direct</th><th>Impedance</th><th>Last sent</th><th>Opcode</th><th>TX ok/fail</th><th>TX age</th><th>Raw motor</th><th>Control angle</th><th>Boot offset</th><th>AS5600 error</th><th>Feedback state</th></tr></thead><tbody>${acts.map(x=>`<tr><td>${healthPill(x.status)}</td><td>${x.name.replaceAll('_',' ')}</td><td><span class="model-name">${x.motor_model}</span><span class="model-detail">FW ${x.motor_protocol} · ${Number(x.gear_ratio).toFixed(0)}:1 · ${x.angle_payload}</span></td><td class="mono">${x.can_id}</td><td>${boolText(x.selected)}</td><td>${x.command_source}</td><td class="mono">${x.direct_setpoint}</td><td class="mono">${x.impedance_setpoint}${x.impedance_enabled?' *':''}</td><td class="mono">${x.last_sent}</td><td class="mono">${x.last_opcode}</td><td class="mono">${x.tx_ok}/${x.tx_fail}</td><td>${age(x.tx_age_ms)}</td><td class="mono">${x.motor_position_deg==null?'—':fmtDeg(x.motor_position_deg)}</td><td class="mono">${x.control_position_deg==null?'—':fmtDeg(x.control_position_deg)}</td><td class="mono">${x.boot_zero_offset_deg==null?'—':fmtDeg(x.boot_zero_offset_deg)}</td><td class="mono">${x.as5600_crosscheck_error_deg==null?'—':fmtDeg(x.as5600_crosscheck_error_deg)}</td><td>${x.control_feedback} · ${x.feedback} · ${age(x.feedback_age_ms)}</td></tr>`).join('')}</tbody></table>`;
 const neck=d.neck_steppers||[];
 document.getElementById('diagNeck').innerHTML=`<table><thead><tr><th>Health</th><th>Motor</th><th>STEP</th><th>DIR</th><th>Current</th><th>Target</th><th>Moving</th><th>Limits</th><th>Feedback</th></tr></thead><tbody>${neck.map(x=>`<tr><td>${healthPill(x.status)}</td><td>M${x.motor}</td><td class="mono">${x.step_gpio}</td><td class="mono">${x.dir_gpio}</td><td class="mono">${Number(x.current_mm).toFixed(2)} mm / ${x.current_steps}</td><td class="mono">${Number(x.target_mm).toFixed(2)} mm / ${x.target_steps}</td><td>${boolText(x.moving)}</td><td class="mono">${x.min_mm}…${x.max_mm} mm</td><td>${x.feedback}</td></tr>`).join('')}</tbody></table>`;
 const tasks=d.tasks||[];
 document.getElementById('diagTasks').innerHTML=`<table><thead><tr><th>Health</th><th>Task</th><th>Active</th><th>Expected rate</th><th>Loop count</th><th>Heartbeat age</th><th>Min free stack</th></tr></thead><tbody>${tasks.map(x=>`<tr><td>${healthPill(x.status)}</td><td>${x.name}</td><td>${boolText(x.active)}</td><td>${x.expected_hz} Hz</td><td class="mono">${x.loops}</td><td>${age(x.age_ms)}</td><td class="mono">${x.stack_high_water_words} words</td></tr>`).join('')}</tbody></table>`;
 const imus=d.imus||[];
 document.getElementById('diagImus').innerHTML=`<table><thead><tr><th>Health</th><th>Index</th><th>Mux CH</th><th>Address</th><th>Seen</th><th>Read ok/fail</th><th>Last seen</th><th>Accel raw</th><th>Gyro raw</th></tr></thead><tbody>${imus.map(x=>`<tr><td>${healthPill(x.status)}</td><td>${x.index}</td><td>${x.mux_channel}</td><td class="mono">${x.address}</td><td>${boolText(x.ever_seen)}</td><td class="mono">${x.read_ok}/${x.read_fail}</td><td>${age(x.seen_age_ms)}</td><td class="mono">${(x.accel||[]).join(', ')}</td><td class="mono">${(x.gyro||[]).join(', ')}</td></tr>`).join('')}</tbody></table>`;
 const sf=d.safety||{};document.getElementById('diagSafety').innerHTML=kvRows([['play',boolText(sf.play)],['stop bursts',sf.stop_burst_remaining],['portal stage',sf.portal_safety_stage],['portal unlocked',boolText(sf.portal_motion_unlocked)],['lease remaining',`${Math.ceil((sf.portal_lease_remaining_ms||0)/1000)} s`],['portal unlocks / rejects',`${sf.portal_unlocks||0} / ${sf.portal_rejects||0}`],['torque pulse',sf.torque_test_active?`motor ${sf.torque_test_actuator_index} @ ${sf.torque_test_value}`:'inactive'],['calibration override',boolText(sf.calibration_override)],['calibration motor',sf.calibration_actuator_index],['calibration torque',sf.calibration_torque],['reboot required',boolText(sf.reboot_required)],['command watchdog',sf.command_watchdog],['CAN feedback monitor',sf.can_feedback_monitoring]]);
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
   <div class="controls row"><input class="requires-motion-unlock" id="t_${j}" type="number" min="-25" max="25" value="${state?.torque?.[i]??0}" placeholder="torque"><button class="requires-motion-unlock" onclick="sendTorque('${j}')">Set torque</button><button class="requires-motion-unlock" onclick="sendTorquePulse('${j}')">Pulse 250 ms</button></div>
   <div class="controls row">${sensed.has(j)?`<label><input class="requires-motion-unlock" id="ie_${j}" type="checkbox" ${enabled?'checked':''}> impedance</label><input class="requires-motion-unlock" id="ip_${j}" type="number" step=".1" value="180" placeholder="position"><input class="requires-motion-unlock" id="iv_${j}" type="number" step=".1" value="0" placeholder="velocity"><button class="requires-motion-unlock" onclick="sendImpedance('${j}')">Apply</button>`:'<span class="tiny">Direct torque only · fresh RMD feedback required for pulse</span>'}</div>`;
  box.appendChild(d);
 });
}
async function loadState(){
 try{
  state=await jfetch('/api/state');
  document.getElementById('roleTitle').textContent=(state.role||'unconfigured').toUpperCase();document.getElementById('addressBadge').textContent='DB1:'+String(state.command_address||routeTarget());document.getElementById('operatingText').textContent=state.role==='head'?'NECK STEPPER':(state.operating_mode||'standalone').toUpperCase();document.getElementById('firmwareText').textContent='FW '+String(state.firmware_version||'unknown')+' · '+String(state.telemetry_protocol||'unknown');
  document.getElementById('ipText').textContent='@ '+state.ip+' · '+state.ssid+(state.role==='head'?' · 6× A4988 STEP/DIR':' · AS5600 GPIO '+(state.sensor_pins||[]).join('/'));
  const pb=document.getElementById('playBadge');pb.textContent=state.play?'PLAY':'STOPPED';pb.className='badge '+(state.play?'ok':'bad');
  const cb=document.getElementById('canBadge');const ready=state.control_ready||state.imu_ready||state.neck_ready;cb.textContent=ready?'RUNTIME READY':(state.configured?'REBOOT REQUIRED':'SETUP');cb.className='badge '+(ready?'ok':'warn');
  document.getElementById('clientBadge').textContent=state.clients+' CLIENT'+(state.clients===1?'':'S');
  document.getElementById('heapBadge').textContent=Math.round(state.heap/1024)+' KB HEAP';
  if(!state.configured)notice('Controller is not configured. Actuator tasks are disabled. Select a role under Configuration, save, then reboot.');
  else if(state.reboot_required)notice('Saved role or operating structure differs from the boot topology. Reboot is required before control authority changes.');
  else if(state.role==='head')notice('HEAD / NECK personality active — FastAccelStepper OPEN-LOOP state. STOP remains available at all times.');
  else if(state.operating_mode==='hyperspawn')notice('HYPERSPAWN / ROS2 ROUTE ACTIVE — portal motion controls are read-only; STOP, diagnostics, configuration and terminal remain available.');
  else notice('');
  renderJoints();renderNeck();renderPortalSafety();
 }catch(e){}
}
function targetForRole(r,configured=true){if(!configured)return 'SETUP';if(r==='left')return 'LEFTLEG';if(r==='right')return 'RIGHTLEG';if(r==='center')return 'CENTER';if(r==='head')return 'HEADNECK';return 'SETUP'}
function routeTarget(){return targetForRole(state?.role,!!state?.configured)}
function routed(c){c=(c||'').trim();if(c.startsWith('<DB1:'))return c;return `<DB1:${routeTarget()}> ${c}`}
async function cmd(c){try{const wire=routed(c);await jfetch('/api/command',{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},body:new URLSearchParams({cmd:wire})});}catch(e){notice('Command failed: '+e.message)}}
function renderPortalSafety(){const s=state?.portal_safety||{};const stage=Number(s.stage||0),unlocked=!!s.motion_unlocked;for(let i=1;i<=3;i++){document.getElementById('safetyStep'+i).classList.toggle('done',stage>=i);document.getElementById('safetyButton'+i).disabled=unlocked||(i===1?stage!==0:stage!==i-1)}const lease=document.getElementById('safetyLease');lease.textContent=unlocked?`UNLOCKED ${Math.ceil((s.lease_remaining_ms||0)/1000)} s`:'LOCKED';lease.className='health '+(unlocked?'ok':'fault');document.getElementById('safetyPhrase').placeholder=s.expected_phrase||'ENABLE LEFTLEG';document.getElementById('safetyPhrase').disabled=unlocked||stage!==2;document.getElementById('safetyDetail').textContent=unlocked?`Web motion authority active for ${Math.ceil((s.lease_remaining_ms||0)/1000)} seconds. Torque pulse ${s.torque_test_active?'ACTIVE':'idle'}.`:'No web motion authority. Motion requests receive HTTP 423.';document.querySelectorAll('.requires-motion-unlock').forEach(e=>e.disabled=!unlocked)}
async function advanceSafety(stage){try{const p=new URLSearchParams({target:routeTarget(),stage:String(stage)});if(stage===3)p.set('phrase',document.getElementById('safetyPhrase').value);await jfetch('/api/safety/advance',{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},body:p});await loadState()}catch(e){notice('Safety unlock reset: '+e.message);await loadState()}}
async function lockSafety(){try{await jfetch('/api/safety/lock?target='+encodeURIComponent(routeTarget()),{method:'POST'});notice('Portal motion locked and stop requested.',true);await loadState()}catch(e){notice('Safety lock failed: '+e.message)}}
function sendTerminal(){const e=document.getElementById('termInput');const c=e.value.trim();if(c){cmd(c);e.value=''}}
function sendTorque(j){if(!state||!['left','right'].includes(state.role))return notice('Configure a leg role first.');if(state.operating_mode==='hyperspawn')return notice('Manual torque is locked: HyperSpawn/ROS2 owns command authority. STOP remains available.');cmd(`torque ${j} ${document.getElementById('t_'+j).value}`)}
function sendTorquePulse(j){if(!state?.portal_safety?.motion_unlocked)return notice('Complete the three-stage portal motion unlock first.');cmd(`test_torque ${j} ${document.getElementById('t_'+j).value} 250`)}
function sendImpedance(j){if(!state||!['left','right'].includes(state.role))return notice('Configure a leg role first.');if(state.operating_mode==='hyperspawn')return notice('Manual impedance is locked: HyperSpawn/ROS2 owns command authority.');const on=document.getElementById('ie_'+j).checked?1:0;cmd(`impedance ${j} ${on} ${document.getElementById('ip_'+j).value} ${document.getElementById('iv_'+j).value}`)}
function sendNeckPose(){cmd(`X${document.getElementById('nx').value},Y${document.getElementById('ny').value},Z${document.getElementById('nz').value},H${document.getElementById('nh').value},S${document.getElementById('ns').value},A${document.getElementById('na').value},R${document.getElementById('nr').value},P${document.getElementById('np').value}`)}
function sendNeckMotor(i){const e=document.getElementById('nm_'+i);if(e)cmd(`${i}:${e.value}`)}
function renderNeck(){const isHead=state?.role==='head';document.getElementById('neckControlCard').style.display=isHead?'block':'none';document.getElementById('legStateCard').style.display=isHead?'none':'block';document.getElementById('legActuatorCard').style.display=isHead?'none':'block';if(!isHead)return;const b=document.getElementById('neckMotors');const motors=state?.neck?.motors||[];b.innerHTML=motors.map(m=>`<div class="joint"><div><div class="jointname">Motor ${m.index}</div><div class="canid">STEP ${m.step_gpio} · DIR ${m.dir_gpio}</div></div><div><span class="tiny">current</span><br>${Number(m.current_mm).toFixed(2)} mm</div><div><span class="tiny">target ${Number(m.target_mm).toFixed(2)} mm · ${m.moving?'MOVING':'IDLE'}</span><div style="height:5px;background:#20252a;margin-top:5px"><div style="height:100%;background:#aeb7bd;width:${Math.max(0,Math.min(100,100*(m.current_mm-m.min_mm)/Math.max(.001,m.max_mm-m.min_mm)))}%"></div></div></div><div class="row"><input class="requires-motion-unlock" id="nm_${m.index}" type="number" step=".1" value="${Number(m.target_mm).toFixed(2)}"><button class="requires-motion-unlock" onclick="sendNeckMotor(${m.index})">Move mm</button></div></div>`).join('')}

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
  server.on("/api/version", HTTP_GET, handleApiVersion);
  server.on("/api/state", HTTP_GET, handleApiState);
  server.on("/api/diagnostics", HTTP_GET, handleApiDiagnostics);
  server.on("/api/config", HTTP_GET, handleApiConfigGet);
  server.on("/api/config", HTTP_POST, handleApiConfigPost);
  server.on("/api/config/reload", HTTP_POST, handleApiConfigReload);
  server.on("/api/config/raw", HTTP_POST, handleApiRawConfigPost);
  server.on("/api/command", HTTP_POST, handleApiCommand);
  server.on("/api/safety/advance", HTTP_POST, handleApiSafetyAdvance);
  server.on("/api/safety/lock", HTTP_POST, handleApiSafetyLock);
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
  lockPortalMotion(true, "SoftAP starting or restarting");
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
    expirePortalMotionLeaseIfNeeded();

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
  Serial.print("FIRMWARE:");
  Serial.println(DROPBEAR_FIRMWARE_VERSION);

  serialMutex = xSemaphoreCreateMutex();
  canMutex = xSemaphoreCreateMutex();
  stateMutex = xSemaphoreCreateMutex();
  webLogMutex = xSemaphoreCreateMutex();
  spiffsMutex = xSemaphoreCreateMutex();
  webCommandQueue = xQueueCreate(12, sizeof(WebCommand));
  canTxQueue = xQueueCreate(CAN_TX_QUEUE_DEPTH, sizeof(CanTxRequest));
  canDiagnosticLineQueue = xQueueCreate(CAN_DIAGNOSTIC_LINE_QUEUE_DEPTH,
                                        sizeof(CanDiagnosticLineRecord));

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
  printVersionRecord();

  // Establish the read-only diagnostic path before any role-specific hardware
  // initialization. This keeps both leg ports structurally observable even if
  // a sensor or CAN peripheral stalls during boot.
  if (xTaskCreatePinnedToCore(
        checkChiralityTask, "command", 6144, nullptr, 2,
        &commandTaskHandle, 0) != pdPASS) {
    commandTaskHandle = nullptr;
    dbPrintln("BOOT|phase=command-task|status=fault");
  } else {
    dbPrintln("BOOT|phase=command-task|status=ready");
  }

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

    const int canBeginResult = CAN.begin(MCP_ANY, CAN_1000KBPS, MCP_8MHZ);
    const int canModeResult = canBeginResult == CAN_OK
      ? configureCanTimingAndNormalMode() : CAN_FAIL;
    const int canOneShotResult = canModeResult == CAN_OK
      ? CAN.enOneShotTX() : CAN_FAIL;
    if (canBeginResult != CAN_OK || canModeResult != CAN_OK ||
        canOneShotResult != CAN_OK) {
      canInitialized = false;
      canOneShotEnabled = false;
      dbPrintf("ERROR: MCP2515 CAN initialization failed: begin=%d mode=%d oneshot=%d.\n",
               canBeginResult, canModeResult, canOneShotResult);
      playMode = false;
    } else {
      canInitialized = true;
      canOneShotEnabled = true;
      dbPrintln("CAN initialized: 1 Mbps, MCP2515 8 MHz, single-sample RX, one-shot TX, CS GPIO5.");
      if (!CAN_BIT_TIMING_DATASHEET_COMPLIANT) {
        dbPrintln("WARNING: MCP2515 8 MHz / 1 Mbps uses out-of-spec PS2=1 TQ; use a 16 MHz MCP2515 clock for compliant 1 Mbps timing.");
      }

      dbPrintln("BOOT|phase=as5600-prime|status=deferred");

      // Wi-Fi/networking lives primarily on core 0. Keep the control path on
      // core 1 so captive-portal traffic does not become sensor/control jitter.
      xTaskCreatePinnedToCore(readAndComputeTask, "sensors", 4096, nullptr, 3, &sensorTaskHandle, 1);
      xTaskCreatePinnedToCore(impedanceControlTask, "impedance", 4096, nullptr, 3, &impedanceTaskHandle, 1);
      xTaskCreatePinnedToCore(canOutputTask, "can-output", 4096, nullptr, 4, &canTaskHandle, 1);
      // The CAN I/O task is the sole runtime MCP2515 transport owner. It must
      // preempt sensing/output long enough to drain the two hardware RX
      // mailboxes and complete one deadline-bound TX, then yields explicitly.
      xTaskCreatePinnedToCore(canReceiveTask, "can-io", 5120, nullptr, 5, &canRxTaskHandle, 1);
      if (canRxTaskHandle != nullptr) {
        attachInterrupt(digitalPinToInterrupt(CAN0_INT), onCanInterrupt, FALLING);
      }
      xTaskCreatePinnedToCore(hyperspawnRouteTask, "hyperspawn", 4096, nullptr, 2, &hyperspawnTaskHandle, 1);
      runtimeControlReady = sensorTaskHandle && impedanceTaskHandle &&
        canTaskHandle && canRxTaskHandle && hyperspawnTaskHandle && canTxQueue &&
        canDiagnosticLineQueue;
      runtimeImuReady = false;
      runtimeNeckReady = false;
      if (!runtimeControlReady) {
        dbPrintln("BOOT|phase=leg-runtime-tasks|status=fault");
      } else {
        dbPrintln("BOOT|phase=leg-runtime-tasks|status=ready");
      }
      // A reboot never arms physical leg output. Serial, portal, or the
      // HyperSpawn route must explicitly establish its own authority.
      playMode = false;
      dbPrintln("Leg output starts STOPPED after boot; explicit arming is required.");
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

  runtimeInitializationComplete = true;

  dbPrintf("Dropbear controller ready: role=%s stack=%s portal=%s SSID=%s. Type 'help'.\n",
           selectedRoleName().c_str(), isHead ? "neck" : (isCenter ? "imu" : operatingModeName().c_str()),
           DROPBEAR_ENABLE_WIFI_PORTAL ? "enabled" : "disabled",
           DROPBEAR_ENABLE_WIFI_PORTAL ? desiredPortalSSID().c_str() : "n/a");
}

void loop() {
  // All runtime ownership is explicit FreeRTOS tasks.
  vTaskDelay(pdMS_TO_TICKS(1000));
}
