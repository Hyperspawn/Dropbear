#include <assert.h>
#include <math.h>
#include <stdint.h>

#include "dropbear_motor_protocol.h"

namespace {

const dropbear::MotorProfile kX8V17 = {
  "RMD-X8 Pro 1:9", "V1.7", 9.0f,
  dropbear::ANGLE_SIGNED_56_LE_BYTES_1_TO_7,
  dropbear::ANGLE_REFERENCE_OUTPUT_SHAFT,
  0.01f, 0x92, 0xA1, 0x80, 0x81
};

const dropbear::MotorProfile kX10V42 = {
  "RMD-X10 1:7", "V4.2+", 7.0f,
  dropbear::ANGLE_SIGNED_32_LE_BYTES_4_TO_7,
  dropbear::ANGLE_REFERENCE_OUTPUT_SHAFT,
  0.01f, 0x92, 0xA1, 0x80, 0x81
};

void encode56(int64_t value, uint8_t payload[8]) {
  payload[0] = 0x92;
  const uint64_t raw = static_cast<uint64_t>(value) & ((1ULL << 56) - 1ULL);
  for (uint8_t index = 0; index < 7; ++index) {
    payload[index + 1] = static_cast<uint8_t>((raw >> (index * 8)) & 0xFF);
  }
}

void encode32(int32_t value, uint8_t payload[8]) {
  payload[0] = 0x92;
  payload[1] = payload[2] = payload[3] = 0;
  const uint32_t raw = static_cast<uint32_t>(value);
  for (uint8_t index = 0; index < 4; ++index) {
    payload[index + 4] = static_cast<uint8_t>((raw >> (index * 8)) & 0xFF);
  }
}

void expectNear(double actual, double expected) {
  assert(fabs(actual - expected) < 1e-4);
}

}  // namespace

int main() {
  uint8_t payload[8] = {0};
  double degrees = 0.0;

  encode56(12345, payload);
  assert(dropbear::decodeMultiTurnAngle(kX8V17, payload, 8, &degrees) == dropbear::DECODE_OK);
  expectNear(degrees, 123.45);
  encode56(-12345, payload);
  assert(dropbear::decodeMultiTurnAngle(kX8V17, payload, 8, &degrees) == dropbear::DECODE_OK);
  expectNear(degrees, -123.45);

  encode56(-(1LL << 55), payload);
  assert(dropbear::decodeSigned56LittleEndian(payload + 1) == -(1LL << 55));
  assert(dropbear::decodeMultiTurnAngle(kX8V17, payload, 8, &degrees) == dropbear::DECODE_OK);
  assert(degrees < 0.0);

  encode32(54321, payload);
  assert(dropbear::decodeMultiTurnAngle(kX10V42, payload, 8, &degrees) == dropbear::DECODE_OK);
  expectNear(degrees, 543.21);
  assert(dropbear::decodeMultiTurnAngleWithLayout(
           kX8V17, dropbear::ANGLE_SIGNED_32_LE_BYTES_4_TO_7,
           payload, 8, &degrees) == dropbear::DECODE_OK);
  expectNear(degrees, 543.21);

  encode56(-54321, payload);
  assert(dropbear::decodeMultiTurnAngleWithLayout(
           kX10V42, dropbear::ANGLE_SIGNED_56_LE_BYTES_1_TO_7,
           payload, 8, &degrees) == dropbear::DECODE_OK);
  expectNear(degrees, -543.21);

  encode32(-54321, payload);
  assert(dropbear::decodeMultiTurnAngle(kX10V42, payload, 8, &degrees) == dropbear::DECODE_OK);
  expectNear(degrees, -543.21);
  payload[2] = 1;
  assert(dropbear::decodeMultiTurnAngle(kX10V42, payload, 8, &degrees) ==
         dropbear::DECODE_RESERVED_BYTES_NONZERO);
  payload[2] = 0;
  payload[0] = 0x91;
  assert(dropbear::decodeMultiTurnAngle(kX10V42, payload, 8, &degrees) ==
         dropbear::DECODE_WRONG_OPCODE);

  dropbear::MotorProfile rotorProfile = kX10V42;
  rotorProfile.angleReference = dropbear::ANGLE_REFERENCE_MOTOR_SHAFT;
  encode32(700, payload);
  assert(dropbear::decodeMultiTurnAngle(rotorProfile, payload, 8, &degrees) == dropbear::DECODE_OK);
  expectNear(degrees, 1.0);

  dropbear::encodeReadMultiTurnAngle(kX8V17, payload);
  assert(payload[0] == 0x92 && payload[1] == 0 && payload[7] == 0);
  dropbear::encodeTorqueCommand(kX8V17, -300, payload);
  assert(payload[0] == 0xA1 && payload[4] == 0xD4 && payload[5] == 0xFE);
  dropbear::encodeShutdownCommand(kX8V17, payload);
  assert(payload[0] == 0x80 && payload[1] == 0 && payload[7] == 0);
  dropbear::encodeHoldCommand(kX8V17, payload);
  assert(payload[0] == 0x81 && payload[1] == 0 && payload[7] == 0);
  dropbear::encodeStopCommand(kX8V17, payload);
  assert(payload[0] == 0x81);
  return 0;
}
