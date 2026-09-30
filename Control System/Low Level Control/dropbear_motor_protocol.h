#ifndef DROPBEAR_MOTOR_PROTOCOL_H
#define DROPBEAR_MOTOR_PROTOCOL_H

#include <stddef.h>
#include <stdint.h>

// Pure, allocation-free MyActuator/RMD protocol helpers.  This header has no
// Arduino dependencies so the exact wire codecs can also be tested on a host.
namespace dropbear {

enum AnglePayloadLayout : uint8_t {
  ANGLE_SIGNED_56_LE_BYTES_1_TO_7 = 0,
  ANGLE_SIGNED_32_LE_BYTES_4_TO_7 = 1,
};

enum AngleReference : uint8_t {
  ANGLE_REFERENCE_OUTPUT_SHAFT = 0,
  ANGLE_REFERENCE_MOTOR_SHAFT = 1,
};

enum DecodeStatus : uint8_t {
  DECODE_OK = 0,
  DECODE_INVALID_ARGUMENT,
  DECODE_INVALID_LENGTH,
  DECODE_WRONG_OPCODE,
  DECODE_RESERVED_BYTES_NONZERO,
  DECODE_INVALID_REDUCTION,
};

struct MotorProfile {
  const char *model;
  const char *protocolVersion;
  float reductionRatio;
  AnglePayloadLayout angleLayout;
  AngleReference angleReference;
  float angleLsbDegrees;
  uint8_t readMultiTurnOpcode;
  uint8_t torqueOpcode;
  uint8_t shutdownOpcode;
  uint8_t holdOpcode;
};

inline const char *angleLayoutName(AnglePayloadLayout layout) {
  switch (layout) {
    case ANGLE_SIGNED_56_LE_BYTES_1_TO_7:
      return "signed56_le_bytes_1_7";
    case ANGLE_SIGNED_32_LE_BYTES_4_TO_7:
      return "signed32_le_bytes_4_7";
    default:
      return "unknown";
  }
}

inline const char *angleReferenceName(AngleReference reference) {
  return reference == ANGLE_REFERENCE_MOTOR_SHAFT ? "motor_shaft" : "output_shaft";
}

inline int64_t decodeSigned56LittleEndian(const uint8_t *bytes) {
  uint64_t raw = 0;
  for (uint8_t index = 0; index < 7; ++index) {
    raw |= static_cast<uint64_t>(bytes[index]) << (index * 8);
  }
  if ((raw & (1ULL << 55)) == 0) return static_cast<int64_t>(raw);
  const uint64_t magnitude = ((~raw) & ((1ULL << 56) - 1ULL)) + 1ULL;
  return -static_cast<int64_t>(magnitude);
}

inline int64_t decodeSigned32LittleEndian(const uint8_t *bytes) {
  const uint32_t raw = static_cast<uint32_t>(bytes[0]) |
                       (static_cast<uint32_t>(bytes[1]) << 8) |
                       (static_cast<uint32_t>(bytes[2]) << 16) |
                       (static_cast<uint32_t>(bytes[3]) << 24);
  return (raw & 0x80000000UL)
    ? static_cast<int64_t>(raw) - 0x100000000LL
    : static_cast<int64_t>(raw);
}

inline DecodeStatus outputShaftDegrees(const MotorProfile &profile,
                                       double reportedDegrees,
                                       double *outputDegrees) {
  if (outputDegrees == nullptr) return DECODE_INVALID_ARGUMENT;
  if (profile.angleReference == ANGLE_REFERENCE_OUTPUT_SHAFT) {
    *outputDegrees = reportedDegrees;
    return DECODE_OK;
  }
  if (!(profile.reductionRatio > 0.0f)) return DECODE_INVALID_REDUCTION;
  *outputDegrees = reportedDegrees / static_cast<double>(profile.reductionRatio);
  return DECODE_OK;
}

inline DecodeStatus decodeMultiTurnAngle(const MotorProfile &profile,
                                         const uint8_t *payload,
                                         size_t length,
                                         double *outputDegrees) {
  if (payload == nullptr || outputDegrees == nullptr) return DECODE_INVALID_ARGUMENT;
  if (length != 8) return DECODE_INVALID_LENGTH;
  if (payload[0] != profile.readMultiTurnOpcode) return DECODE_WRONG_OPCODE;

  int64_t rawAngle = 0;
  switch (profile.angleLayout) {
    case ANGLE_SIGNED_56_LE_BYTES_1_TO_7:
      rawAngle = decodeSigned56LittleEndian(payload + 1);
      break;
    case ANGLE_SIGNED_32_LE_BYTES_4_TO_7:
      if (payload[1] != 0 || payload[2] != 0 || payload[3] != 0) {
        return DECODE_RESERVED_BYTES_NONZERO;
      }
      rawAngle = decodeSigned32LittleEndian(payload + 4);
      break;
    default:
      return DECODE_INVALID_ARGUMENT;
  }

  return outputShaftDegrees(
    profile,
    static_cast<double>(rawAngle) * static_cast<double>(profile.angleLsbDegrees),
    outputDegrees
  );
}

inline DecodeStatus decodeMultiTurnAngleWithLayout(
    const MotorProfile &profile,
    AnglePayloadLayout layout,
    const uint8_t *payload,
    size_t length,
    double *outputDegrees) {
  MotorProfile detected = profile;
  detected.angleLayout = layout;
  return decodeMultiTurnAngle(detected, payload, length, outputDegrees);
}

inline void encodeReadMultiTurnAngle(const MotorProfile &profile, uint8_t payload[8]) {
  for (uint8_t index = 0; index < 8; ++index) payload[index] = 0;
  payload[0] = profile.readMultiTurnOpcode;
}

inline void encodeTorqueCommand(const MotorProfile &profile,
                                int16_t torqueValue,
                                uint8_t payload[8]) {
  for (uint8_t index = 0; index < 8; ++index) payload[index] = 0;
  const uint16_t wireValue = static_cast<uint16_t>(torqueValue);
  payload[0] = profile.torqueOpcode;
  payload[4] = static_cast<uint8_t>(wireValue & 0xFF);
  payload[5] = static_cast<uint8_t>((wireValue >> 8) & 0xFF);
}

inline void encodeShutdownCommand(const MotorProfile &profile, uint8_t payload[8]) {
  for (uint8_t index = 0; index < 8; ++index) payload[index] = 0;
  payload[0] = profile.shutdownOpcode;
}

inline void encodeHoldCommand(const MotorProfile &profile, uint8_t payload[8]) {
  for (uint8_t index = 0; index < 8; ++index) payload[index] = 0;
  payload[0] = profile.holdOpcode;
}

// Source compatibility for callers that historically called the closed-loop
// 0x81 operation "stop". It is a hold, not an output shutdown.
inline void encodeStopCommand(const MotorProfile &profile, uint8_t payload[8]) {
  encodeHoldCommand(profile, payload);
}

// 0xB3 communication-loss protection (RMD-X protocol V3.9 p74-75, V4.2 p78):
// the timeout in milliseconds is a little-endian uint32 in DATA[4..7];
// 0 disables the protection. Writing it to DATA[2..3] leaves DATA[4..7] = 0,
// which silently disables the protection.
inline void encodeCommLossProtection(uint32_t timeoutMs, uint8_t payload[8]) {
  for (uint8_t index = 0; index < 8; ++index) payload[index] = 0;
  payload[0] = 0xB3;
  payload[4] = static_cast<uint8_t>(timeoutMs & 0xFF);
  payload[5] = static_cast<uint8_t>((timeoutMs >> 8) & 0xFF);
  payload[6] = static_cast<uint8_t>((timeoutMs >> 16) & 0xFF);
  payload[7] = static_cast<uint8_t>((timeoutMs >> 24) & 0xFF);
}

}  // namespace dropbear

#endif  // DROPBEAR_MOTOR_PROTOCOL_H
