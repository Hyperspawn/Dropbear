from __future__ import annotations

import unittest
from pathlib import Path


SOURCE = Path(__file__).with_name("firmware_full_libs_neck.ino").read_text()


class BehemothPortalSafetyContract(unittest.TestCase):
    def test_leg_output_starts_stopped(self):
        self.assertIn("Leg output starts STOPPED after boot", SOURCE)
        self.assertNotIn("playMode = (operatingMode == OPERATING_STANDALONE);", SOURCE)

    def test_motion_commands_require_server_side_timed_unlock(self):
        self.assertIn("PORTAL_MOTION_LEASE_MS = 90000", SOURCE)
        self.assertIn('server.on("/api/safety/advance", HTTP_POST', SOURCE)
        self.assertIn('server.on("/api/safety/lock", HTTP_POST', SOURCE)
        self.assertGreaterEqual(SOURCE.count("portalPayloadRequiresMotionUnlock"), 3)
        self.assertIn('server.send(423, "application/json"', SOURCE)
        self.assertIn('"ENABLE " + currentCommandAddress()', SOURCE)

    def test_each_leg_motor_has_a_bounded_feedback_gated_pulse(self):
        self.assertIn(
            "const jointNames=['outer_calf','inner_calf','knee','hip_pitch','hip_yaw','hip_roll']",
            SOURCE,
        )
        self.assertIn("PORTAL_TORQUE_TEST_MAX_MS = 500", SOURCE)
        self.assertIn("PORTAL_TORQUE_TEST_MAX_COMMAND = 25", SOURCE)
        self.assertIn("fresh verified motor-native angle feedback is required", SOURCE)
        self.assertIn("sendTorqueCommand(ACTUATOR_IDS[i], 0)", SOURCE)
        self.assertIn("requestStop(3);", SOURCE)

    def test_live_diagnostics_expose_both_angle_sources_and_alignment(self):
        for field in (
            "motor_position_deg",
            "control_position_deg",
            "boot_zero_offset_deg",
            "as5600_crosscheck_error_deg",
            "control_feedback",
            "portal_motion_unlocked",
            "torque_test_active",
        ):
            self.assertIn(field, SOURCE)


if __name__ == "__main__":
    unittest.main()
