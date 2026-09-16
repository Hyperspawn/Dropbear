"""Static contracts for continuous motor-native position across leg firmware."""

from pathlib import Path
import unittest


ROOT = Path(__file__).parent
LEG_FIRMWARE = {
    name: (ROOT / name).read_text()
    for name in (
        "firmware_full_libs_neck.ino",
        "esp32_devkitc_v4_hybrid.ino",
        "esp32_devkit_v1.ino",
        "esp32_devkit_v1_observation_safe.ino",
    )
}


class ContinuousMotorFeedbackContract(unittest.TestCase):
    def test_every_leg_image_requests_and_decodes_rmd_position(self):
        for name, source in LEG_FIRMWARE.items():
            with self.subTest(firmware=name):
                self.assertIn("0x92", source)
                self.assertIn("motorNativeDegrees", source)
                self.assertIn("MOTOR_FEEDBACK_STALE_MS" if "observation_safe" in name else "MOTOR_NATIVE_STALE_MS", source)

    def test_every_leg_image_emits_versioned_dual_angle_telemetry(self):
        for name, source in LEG_FIRMWARE.items():
            with self.subTest(firmware=name):
                self.assertIn('Serial.print("DB2,")', source)
                self.assertIn("normalizedOuter", source)
                self.assertIn("motorNativeDegrees", source)
                self.assertIn('Serial.print("NA")', source)

    def test_active_controllers_use_zeroed_can_and_fail_closed(self):
        for name in (
            "firmware_full_libs_neck.ino",
            "esp32_devkitc_v4_hybrid.ino",
            "esp32_devkit_v1.ino",
        ):
            source = LEG_FIRMWARE[name]
            with self.subTest(firmware=name):
                self.assertIn("MOTOR_BOOT_ZERO_SAMPLE_COUNT", source)
                self.assertIn("readMotorControlDegrees(actuatorIndex, measuredDegrees)", source)
                self.assertIn("controller.update(measuredDegrees", source)
                self.assertIn("motorControlAlignmentFault[actuatorIndex] = true", source)
                self.assertIn("impedanceTorqueValues[actuatorIndex] = 0", source)

    def test_observation_image_explicitly_enables_non_motion_queries(self):
        source = LEG_FIRMWARE["esp32_devkit_v1_observation_safe.ino"]
        self.assertIn("const bool OBSERVATION_ONLY_FIRMWARE = true", source)
        self.assertIn("const bool MOTOR_FEEDBACK_QUERY_ALLOWED = true", source)
        self.assertIn("const bool LEGACY_SERIAL_MOTION_ALLOWED = false", source)


if __name__ == "__main__":
    unittest.main()
