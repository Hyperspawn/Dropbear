"""Static safety contract for Behemoth motor-native observation telemetry."""

from pathlib import Path
import unittest


SOURCE = (Path(__file__).parent / "firmware_full_libs_neck.ino").read_text()
PROTOCOL = (Path(__file__).parent / "dropbear_motor_protocol.h").read_text()


class BehemothMotorFeedbackContract(unittest.TestCase):
    def test_read_only_rmd_query_and_decoder_are_present(self):
        self.assertIn("dropbear::encodeReadMultiTurnAngle(*profile, request)", SOURCE)
        self.assertIn("dropbear::decodeMultiTurnAngleWithLayout(", SOURCE)
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
        self.assertIn("dropbear::encodeHoldCommand(*profile, buf)", SOURCE)
        self.assertIn("encodeShutdownCommand", PROTOCOL)
        self.assertIn("shutdownOpcode", PROTOCOL)
        self.assertIn("holdOpcode", PROTOCOL)

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
        self.assertIn("behemoth-observation-protocol-2026.09.43", SOURCE)
        self.assertIn("motor-profile-v1", SOURCE)
        self.assertIn("motor-angle-rmd-v17-v42-0x92", SOURCE)
        self.assertIn("boot-observability-v1", SOURCE)
        self.assertIn("can-read-passthrough-v1", SOURCE)
        self.assertIn("can-discovered-read-v1", SOURCE)
        self.assertIn("can-discovery-v1", SOURCE)
        self.assertIn("can-bus-recovery-v1", SOURCE)
        self.assertIn("can-transaction-scheduler-v1", SOURCE)
        self.assertIn("can-oneshot-tx-v1", SOURCE)
        self.assertIn("can-deferred-tx-abort-v1", SOURCE)
        self.assertIn("can-interrupt-rx-v1", SOURCE)
        self.assertIn("can-timing-observability-v1", SOURCE)
        self.assertIn("motor-identity-discovery-v1", SOURCE)
        self.assertIn("motor-runtime-codec-detection-v1", SOURCE)
        self.assertIn("motor-active-reply-normalization-v1", SOURCE)

    def test_can_debug_bridge_is_targeted_read_only_and_raw(self):
        self.assertIn("isReadOnlyCanDiagnosticOpcode(payload[0])", SOURCE)
        self.assertIn("selectedDiagnosticMotorId(requestId)", SOURCE)
        diagnostic_selector = SOURCE[
            SOURCE.index("bool selectedDiagnosticMotorId"):
            SOURCE.index("void emitCanDiagnosticLine")
        ]
        self.assertIn("requestId >= RMD_DISCOVERY_FIRST_ID", diagnostic_selector)
        self.assertIn("requestId <= RMD_DISCOVERY_LAST_ID", diagnostic_selector)
        self.assertNotIn("actuatorBelongsToSelectedLeg(index)", diagnostic_selector)
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
        self.assertIn("if (actuatorBelongsToSelectedLeg(directIndex)) return directIndex", SOURCE)
        self.assertIn("CAN.getError()", SOURCE)
        self.assertIn("CAN.errorCountTX()", SOURCE)
        self.assertIn("recoverCanController()", SOURCE)
        self.assertIn("MCP_EFLG_TXBO", SOURCE)
        self.assertIn("CAN_RECOVERY_RX_QUIET_MS", SOURCE)
        self.assertIn("GET_TX_BUFFER_TIMEOUT", SOURCE)
        self.assertIn("canDiagnosticScanFoundMask", SOURCE)
        self.assertIn("same_id=", SOURCE)
        self.assertIn("tx_failures=", SOURCE)

    def test_identity_discovery_is_cross_generation_and_motion_safe(self):
        self.assertIn('normalized == "can discover"', SOURCE)
        self.assertIn('normalized.startsWith("can identify ")', SOURCE)
        self.assertIn("payload[0] = 0xB2", SOURCE)
        self.assertIn("payload[0] = 0xB5", SOURCE)
        self.assertIn("payload[1] = 0x01", SOURCE)
        self.assertIn("tenDigitDateRevision", SOURCE)
        self.assertIn('"DBM1," + currentCommandAddress()', SOURCE)
        opcode_filter = SOURCE[
            SOURCE.index("bool isReadOnlyCanDiagnosticOpcode"):
            SOURCE.index("bool isExpectedCanDiagnosticReplyId")
        ]
        self.assertIn("case 0xB2", opcode_filter)
        self.assertIn("case 0xB5", opcode_filter)
        self.assertNotIn("case 0xB6", opcode_filter)
        self.assertIn("disableCanMotorActiveReplies", SOURCE)
        self.assertIn("{0xB6, ACTIVE_REPLY_OPCODES[index], 0x00", SOURCE)

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
            'xTaskCreatePinnedToCore(canReceiveTask, "can-io", 5120, nullptr, 5',
            SOURCE,
        )
        self.assertIn("serviceCanTxQueueOne();", SOURCE)
        self.assertIn("xQueueSendToFront(canTxQueue", SOURCE)
        self.assertIn("if (index < 0) return false;", SOURCE)
        self.assertIn("data[0] != profile->readMultiTurnOpcode", SOURCE)

    def test_can_reads_use_one_outstanding_transaction_and_offline_backoff(self):
        self.assertIn("motorNativePendingIndex", SOURCE)
        self.assertIn("MOTOR_NATIVE_REPLY_TIMEOUT_MS", SOURCE)
        self.assertIn("MOTOR_NATIVE_OFFLINE_AFTER_MISSES", SOURCE)
        self.assertIn("MOTOR_NATIVE_OFFLINE_RETRY_MS", SOURCE)
        self.assertIn("recordMotorNativeMiss", SOURCE)
        scheduler = SOURCE[
            SOURCE.index("void canReceiveTask(void *parameter) {"):
            SOURCE.index("void hyperspawnRouteTask(void *parameter) {")
        ]
        self.assertIn("motorNativePendingIndex < 0", scheduler)
        self.assertIn("motorNativePendingSinceMs", scheduler)
        self.assertIn("motorNativeNextEligibleMs", scheduler)
        request = SOURCE[
            SOURCE.index("bool requestMotorNativeFeedback"):
            SOURCE.index("void updateSensorDiagnosticSample")
        ]
        self.assertIn("result != CAN_SENDMSGTIMEOUT", request)
        sender = SOURCE[
            SOURCE.index("int canSendFrameResult"):
            SOURCE.index("bool canSend(uint32_t")
        ]
        self.assertIn("deferredRead", sender)
        self.assertIn("canDeferredReadSubmissions++", sender)
        self.assertIn("if (actuatorIndex >= 0 && motionFrame)", sender)

    def test_error_passive_backs_off_without_reset_and_overflow_is_cleared(self):
        scheduler = SOURCE[
            SOURCE.index("void canReceiveTask(void *parameter) {"):
            SOURCE.index("void hyperspawnRouteTask(void *parameter) {")
        ]
        for flag in (
            "MCP_EFLG_TXBO",
            "MCP_EFLG_TXEP",
            "MCP_EFLG_RXEP",
            "MCP_EFLG_RX0OVR",
            "MCP_EFLG_RX1OVR",
        ):
            self.assertIn(flag, scheduler)
        recovery_condition = scheduler[
            scheduler.index("const bool controllerModeFault"):
            scheduler.index("if (canDeferredTxPending")
        ]
        self.assertIn("MCP_EFLG_TXBO", recovery_condition)
        self.assertNotIn("MCP_EFLG_TXEP) != 0", recovery_condition)
        self.assertIn("clearCanRxOverflowFlags();", scheduler)
        output = SOURCE[
            SOURCE.index("void canOutputTask(void *parameter) {"):
            SOURCE.index("void executeQueuedWebCommand(const WebCommand &item) {")
        ]
        self.assertGreaterEqual(output.count("paceCanCommandBurst();"), 5)
        pacing = SOURCE[
            SOURCE.index("void paceCanCommandBurst()"):
            SOURCE.index("void neckStopAll();", SOURCE.index("void paceCanCommandBurst()"))
        ]
        self.assertIn("taskYIELD();", pacing)
        self.assertNotIn("vTaskDelay", pacing)
        self.assertGreaterEqual(SOURCE.count("CAN.enOneShotTX()"), 2)
        self.assertIn("abortPendingCanTx(\"deferred_read_timeout\")", scheduler)
        self.assertIn("MCP_TXB0CTRL", SOURCE)
        self.assertIn("MCP_TXB1CTRL", SOURCE)
        self.assertIn("MCP_TXB2CTRL", SOURCE)
        self.assertIn("mcp2515BitModifyDirect(MCP_CANCTRL, ABORT_TX, 0)", SOURCE)
        self.assertIn("ulTaskNotifyTake", scheduler)
        self.assertIn("digitalRead(CAN0_INT) == LOW", scheduler)
        self.assertIn("attachInterrupt(digitalPinToInterrupt(CAN0_INT)", SOURCE)
        self.assertIn("bit_timing_datasheet_compliant", SOURCE)
        self.assertIn("out_of_spec_8mhz_1mbps", SOURCE)
        recovery_start = SOURCE.index("bool recoverCanController() {")
        recovery_end = SOURCE.index("uint8_t encodeCanResultForNotification", recovery_start)
        recovery = SOURCE[recovery_start:recovery_end]
        self.assertIn("xSemaphoreTake(serialMutex", recovery)
        self.assertIn("canOneShotEnabled = true", recovery)

    def test_one_shot_transmit_checks_wire_outcome_and_uses_deadline_queue(self):
        direct_start = SOURCE.index("int mcp2515SendStandardFrameDirect")
        direct_end = SOURCE.index("bool abortPendingCanTx", direct_start)
        direct = SOURCE[direct_start:direct_end]
        self.assertIn("MCP_TXB_TXERR_M", direct)
        self.assertIn("MCP_TXB_MLOA_M", direct)
        self.assertIn("MCP_TXB_ABTF_M", direct)
        self.assertIn("CAN_TX_COMPLETION_TIMEOUT_US", direct)
        self.assertIn("const uint8_t controlAddress = MCP_TXB0CTRL", direct)
        self.assertNotIn("TX_CONTROL[3]", direct)
        self.assertIn("MCP_TX0IF | MCP_ERRIF | MCP_MERRF", direct)
        sender_start = SOURCE.index("int canSendFrameResult(uint32_t actuatorID", direct_end)
        sender_end = SOURCE.index("bool canSendFrame(uint32_t", sender_start)
        sender = SOURCE[sender_start:sender_end]
        self.assertIn("xQueueSendToFront(canTxQueue", sender)
        self.assertIn("CAN_TX_QUEUE_DEADLINE_US", sender)
        self.assertIn("CAN_TX_CALLER_TIMEOUT_MS", sender)
        self.assertIn("xTaskGetCurrentTaskHandle() == canRxTaskHandle", sender)

    def test_missing_nodes_back_off_and_serial_diagnostics_are_bounded(self):
        self.assertIn("MOTOR_NATIVE_OFFLINE_RETRY_MS = 5000", SOURCE)
        request = SOURCE[
            SOURCE.index("bool requestMotorNativeFeedback"):
            SOURCE.index("void updateSensorDiagnosticSample")
        ]
        self.assertIn('abortPendingCanTx("read_mailbox_busy")', request)
        bus_start = SOURCE.index("void printCanBusStatus()", SOURCE.index("const char *canResultName"))
        bus = SOURCE[
            bus_start:SOURCE.index("void printCanRegisterSnapshot()", bus_start)
        ]
        self.assertIn("char line[256];", bus)
        self.assertIn('DBC1,%s,BUS,', bus)
        self.assertIn('DBC1,%s,ERRORS,', bus)
        self.assertIn('DBC1,%s,TIMING,', bus)
        self.assertNotIn('emitCanDiagnosticLine(', bus)

    def test_8mhz_1mbps_override_disables_library_triple_sampling(self):
        timing = SOURCE[
            SOURCE.index("int configureCanTimingAndNormalMode()"):
            SOURCE.index("int mcp2515SendStandardFrameDirect")
        ]
        self.assertIn("CAN.setMode(MODE_CONFIG)", timing)
        self.assertIn("CAN_CNF1_8MHZ_1MBPS = 0x00", SOURCE)
        self.assertIn("CAN_CNF2_8MHZ_1MBPS_SINGLE_SAMPLE = 0x80", SOURCE)
        self.assertIn("CAN_CNF3_8MHZ_1MBPS = 0x80", SOURCE)
        self.assertIn("mcp2515ReadRegisterDirect(MCP_CNF2)", timing)
        self.assertIn("configureCanTimingAndNormalMode()", SOURCE[SOURCE.index("void setup()") :])
        recovery = SOURCE[
            SOURCE.index("bool recoverCanController()"):
            SOURCE.index("uint8_t encodeCanResultForNotification")
        ]
        self.assertIn("configureCanTimingAndNormalMode()", recovery)

    def test_rtos_transport_has_measured_deadlines_and_nonblocking_diagnostics(self):
        scheduler = SOURCE[
            SOURCE.index("void canReceiveTask(void *parameter) {"):
            SOURCE.index("void hyperspawnRouteTask(void *parameter) {")
        ]
        self.assertIn("const bool servicedQueuedTx = serviceCanTxQueueOne();", scheduler)
        self.assertIn("!servicedQueuedTx", scheduler)
        self.assertIn("canIoLoopBudgetMisses++", scheduler)
        self.assertIn("canRxDrainMaxUs", scheduler)
        sender_start = SOURCE.index("int performCanTransmit(uint32_t id")
        sender = SOURCE[
            sender_start:SOURCE.index("bool canSendFrame(uint32_t", sender_start)
        ]
        self.assertIn("canTxQueueLatencyMaxUs", sender)
        self.assertIn("canTxExecutionMaxUs", sender)
        output = SOURCE[
            SOURCE.index("void canOutputTask(void *parameter) {"):
            SOURCE.index("void executeQueuedWebCommand", SOURCE.index("void canOutputTask"))
        ]
        self.assertIn("canOutputDeadlineMisses++", output)
        self.assertIn("canStopBatchFailures++", output)
        stop_batch = output[output.index("} else if (stopBurstRemaining > 0)"):]
        self.assertNotIn("break;", stop_batch.split("if (motionBatchAttempted", 1)[0])
        diagnostic = SOURCE[
            SOURCE.index("void emitCanDiagnosticCStringNow"):
            SOURCE.index("bool sendReadOnlyDiagnosticFrame")
        ]
        self.assertIn("xQueueSend(canDiagnosticLineQueue", diagnostic)
        self.assertIn("emitCanDiagnosticFrame(\"RX\"", diagnostic)
        self.assertIn("flushDeferredCanDiagnosticLines(4);", SOURCE)

    def test_polling_can_be_quiesced_for_non_interleaved_diagnostics(self):
        self.assertIn('normalized == "can poll off"', SOURCE)
        self.assertIn('normalized == "can poll on"', SOURCE)
        self.assertIn('normalized == "can registers"', SOURCE)
        self.assertIn("quiesceMotorNativePolling();", SOURCE)
        self.assertIn("MOTOR_NATIVE_FEEDBACK_ENABLED && motorNativePollingEnabled", SOURCE)

    def test_stale_observation_retains_last_value_but_control_remains_fail_closed(self):
        readings = SOURCE[
            SOURCE.index("void printReadings() {"):
            SOURCE.index("void printVersionRecord() {")
        ]
        self.assertIn("if (motorNativeValid[index])", readings)
        self.assertIn("const bool fresh = motorNativeValid[index]", readings)
        self.assertIn(
            "millis() - motorNativeReceivedMs[actuatorIndex] > MOTOR_NATIVE_STALE_MS",
            SOURCE,
        )


if __name__ == "__main__":
    unittest.main()
