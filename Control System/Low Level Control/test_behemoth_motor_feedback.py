"""Static safety contract for Behemoth motor-native observation telemetry."""

from pathlib import Path
import unittest


SOURCE = (Path(__file__).parent / "firmware_full_libs_neck.ino").read_text()
PROTOCOL = (Path(__file__).parent / "dropbear_motor_protocol.h").read_text()


class BehemothMotorFeedbackContract(unittest.TestCase):
    def test_read_only_rmd_query_and_decoder_are_present(self):
        self.assertIn("dropbear::encodeReadMultiTurnAngle(*profile, request)", SOURCE)
        self.assertIn("dropbear::decodeMultiTurnAngle(*profile, data, len", SOURCE)
        self.assertIn("responseID - 0x100", SOURCE)
        self.assertIn("ANGLE_SIGNED_56_LE_BYTES_1_TO_7", PROTOCOL)
        self.assertIn("ANGLE_SIGNED_32_LE_BYTES_4_TO_7", PROTOCOL)
        self.assertIn("DECODE_RESERVED_BYTES_NONZERO", PROTOCOL)

    def test_motor_profiles_are_explicit_per_actuator(self):
        self.assertIn('"MyActuator RMD-X8 Pro 1:9", "V1.7", 9.0f', SOURCE)
        self.assertIn('"MyActuator RMD-X10 1:7", "V4.2+", 7.0f', SOURCE)
        self.assertIn("ACTUATOR_MOTOR_PROFILES[ACTUATOR_COUNT]", SOURCE)
        self.assertIn("ANGLE_REFERENCE_OUTPUT_SHAFT", SOURCE)
        self.assertIn("motorProfileForActuator", SOURCE)

    def test_command_encoding_uses_motor_profile_library(self):
        self.assertIn("dropbear::encodeTorqueCommand(*profile, torqueValue, buf)", SOURCE)
        self.assertIn("dropbear::encodeStopCommand(*profile, buf)", SOURCE)

    def test_motor_feedback_is_consumed_before_control_route_frames(self):
        native = SOURCE.index("ingestMotorNativeFeedback(static_cast<uint32_t>(rxId), data, len)")
        route = SOURCE.index("handleHyperspawnRxFrame(static_cast<uint32_t>(rxId), data, len)", native)
        self.assertLess(native, route)

    def test_db3_keeps_external_and_raw_motor_fields_then_adds_aligned_motor_fields(self):
        self.assertIn('Serial.print("DB3,")', SOURCE)
        self.assertIn("for (uint8_t slot = 0; slot < 6; ++slot)", SOURCE)
        for external in (
            "normalizedOuter",
            "normalizedInner",
            "normalizedHip",
            "normalizedKnee",
            "normalizedButt",
        ):
            self.assertIn(f"Serial.print({external}, 1)", SOURCE)
        self.assertIn('Serial.print("NA")', SOURCE)
        self.assertIn("readMotorControlDegrees(index, controlDegrees)", SOURCE)
        self.assertIn("freshMask", SOURCE)
        self.assertIn("controlMask", SOURCE)
        self.assertIn("alignmentFaultMask", SOURCE)

    def test_identity_and_diagnostics_name_the_feedback_protocol(self):
        self.assertIn("behemoth-observation-protocol-2026.09.33", SOURCE)
        self.assertIn("motor-profile-v1", SOURCE)
        self.assertIn("motor-angle-rmd-v17-v42-0x92", SOURCE)
        self.assertIn("boot-observability-v1", SOURCE)
        self.assertIn("can-read-passthrough-v1", SOURCE)
        self.assertIn("can-discovery-v1", SOURCE)
        self.assertIn("can-bus-recovery-v1", SOURCE)

    def test_can_debug_bridge_is_targeted_read_only_and_raw(self):
        self.assertIn("isReadOnlyCanDiagnosticOpcode(payload[0])", SOURCE)
        self.assertIn("selectedDiagnosticMotorId(requestId)", SOURCE)
        self.assertIn("can tx-read <motor_id> <byte0> ... <byte7>", SOURCE)
        self.assertIn("can info <motor_id>", SOURCE)
        self.assertIn("can monitor <motor_id>", SOURCE)
        self.assertIn("captureCanDiagnosticFrame(static_cast<uint32_t>(rxId), data, len)", SOURCE)
        self.assertIn("isExpectedCanDiagnosticReplyId", SOURCE)
        self.assertIn("responseId == requestId + 0x100U", SOURCE)
        for unsafe_opcode in ("case 0x81", "case 0xA1", "case 0xA4", "case 0xC4"):
            opcode_filter = SOURCE[
                SOURCE.index("bool isReadOnlyCanDiagnosticOpcode"):
                SOURCE.index("bool isExpectedCanDiagnosticReplyId")
            ]
            self.assertNotIn(unsafe_opcode, opcode_filter)
        self.assertIn('normalized == "can scan"', SOURCE)
        self.assertIn('normalized == "can sniff"', SOURCE)
        self.assertIn("RMD_DISCOVERY_FIRST_ID = 0x141", SOURCE)
        self.assertIn("RMD_DISCOVERY_LAST_ID = 0x160", SOURCE)
        self.assertIn("byte payload[8] = {0x9A, 0, 0, 0, 0, 0, 0, 0}", SOURCE)
        self.assertIn("!canDiagnosticScanActive", SOURCE)
        self.assertIn("motorProfileForActuator(directIndex) == &MOTOR_PROFILE_X8_V17", SOURCE)
        self.assertIn("CAN.getError()", SOURCE)
        self.assertIn("CAN.errorCountTX()", SOURCE)
        self.assertIn("recoverCanController()", SOURCE)
        self.assertIn("MCP_EFLG_TXBO", SOURCE)
        self.assertIn("CAN_RECOVERY_RX_QUIET_MS", SOURCE)
        self.assertIn("GET_TX_BUFFER_TIMEOUT", SOURCE)
        self.assertIn("canDiagnosticScanFoundMask", SOURCE)
        self.assertIn("same_id=", SOURCE)
        self.assertIn("tx_failures=", SOURCE)

    def test_diagnostic_task_precedes_deferred_sensor_priming(self):
        setup = SOURCE[SOURCE.index("void setup()") : SOURCE.index("void loop()")]
        self.assertLess(setup.index("checkChiralityTask"), setup.index("status=deferred"))
        self.assertNotIn("primeSensorFilter();", setup)
        sensor_start = SOURCE.index("void readAndComputeTask(void *parameter) {")
        sensor_task = SOURCE[
            sensor_start : SOURCE.index("void updateMotorReferencedImpedance", sensor_start)
        ]
        self.assertIn("primeSensorFilter();", sensor_task)
        self.assertIn("while (!runtimeInitializationComplete)", sensor_task)
        setup = SOURCE[SOURCE.index("void setup()") : SOURCE.index("void loop()")]
        self.assertIn("&commandTaskHandle, 0", setup)
        self.assertIn("rmd_v17_v42_0x92_multi_turn", SOURCE)

    def test_observation_protocol_is_addressed_and_does_not_enable_play(self):
        self.assertIn('DROPBEAR_COMMAND_PROTOCOL = "DB1"', SOURCE)
        self.assertIn("version-v1;health-v1;observe-stream-v1;db1-required", SOURCE)
        self.assertIn('command.equalsIgnoreCase("observe on")', SOURCE)
        observation_block = SOURCE[
            SOURCE.index('if (command.equalsIgnoreCase("observe on")'):
            SOURCE.index('if (command.equalsIgnoreCase("observe off")')
        ]
        self.assertNotIn("playMode = true", observation_block)
        self.assertIn('server.on("/api/version", HTTP_GET, handleApiVersion)', SOURCE)

    def test_as5600_boot_zero_then_can_feedback_drives_impedance(self):
        self.assertIn("MOTOR_BOOT_ZERO_SAMPLE_COUNT", SOURCE)
        self.assertIn("updateMotorControlReference(index, motorNativeDegrees[index])", SOURCE)
        self.assertIn("readMotorControlDegrees(actuatorIndex, measuredDegrees)", SOURCE)
        self.assertIn("controller.update(measuredDegrees", SOURCE)
        self.assertNotIn("outerCalfControlLeft.update(normalizedOuter", SOURCE)
        self.assertIn("as5600_boot_zero_then_rmd_0x92", SOURCE)

    def test_portal_exposes_motor_profiles_and_independent_sensor_state(self):
        for field in (
            "motor_model",
            "motor_protocol",
            "gear_ratio",
            "angle_reference",
            "angle_payload",
            "as5600",
        ):
            self.assertIn(field, SOURCE)
        self.assertIn("Live appendage state", SOURCE)
        self.assertIn("Motor output-shaft state", SOURCE)
        self.assertIn("AS5600 encoder state", SOURCE)

    def test_stale_or_divergent_can_feedback_fails_to_zero_torque(self):
        self.assertIn("MOTOR_AS5600_DIVERGENCE_LIMIT_DEG", SOURCE)
        self.assertIn("motorControlAlignmentFault[actuatorIndex] = true", SOURCE)
        self.assertIn("impedanceTorqueValues[actuatorIndex] = 0", SOURCE)
        self.assertIn("millis() - motorNativeReceivedMs[actuatorIndex] > MOTOR_NATIVE_STALE_MS", SOURCE)

    def test_busy_can_bus_cannot_starve_sensor_telemetry(self):
        self.assertIn("CAN_RX_BURST_LIMIT", SOURCE)
        self.assertIn("drained < CAN_RX_BURST_LIMIT", SOURCE)
        self.assertIn("MOTOR_NATIVE_QUERY_BACKOFF_MS", SOURCE)
        self.assertIn("canConsecutiveFailures >= 3", SOURCE)
        self.assertIn(
            'xTaskCreatePinnedToCore(canReceiveTask, "can-rx", 4096, nullptr, 3',
            SOURCE,
        )
        self.assertIn("if (index < 0) return false;", SOURCE)
        self.assertIn("data[0] != profile->readMultiTurnOpcode", SOURCE)


if __name__ == "__main__":
    unittest.main()
