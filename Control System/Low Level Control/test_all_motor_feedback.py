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
                self.assertIn("motor-angle-rmd-v17-v42-0x92", source)
                if name == "firmware_full_libs_neck.ino":
                    self.assertIn('#include "dropbear_motor_protocol.h"', source)
                    self.assertIn("decodeMultiTurnAngle", source)
                else:
                    self.assertIn("0xFF00000000000000ULL", source)
                    self.assertIn("static_cast<double>(signedRaw) * 0.01", source)
                    self.assertIn("static_cast<float>(signedRaw) * 0.01f", source)
                self.assertIn("MOTOR_FEEDBACK_STALE_MS" if "observation_safe" in name else "MOTOR_NATIVE_STALE_MS", source)

    def test_every_leg_image_emits_versioned_dual_angle_telemetry(self):
        for name, source in LEG_FIRMWARE.items():
            with self.subTest(firmware=name):
                self.assertIn("DROPBEAR_FIRMWARE_VERSION", source)
                self.assertIn('Serial.print("FIRMWARE:")', source)
                expected_schema = "DB2" if "observation_safe" in name else "DB3"
                self.assertIn(f'"{expected_schema}"', source)
                self.assertIn("normalizedOuter", source)
                self.assertIn("motorNativeDegrees", source)
                self.assertIn('Serial.print("NA")', source)

    def test_every_leg_image_reports_version_health_and_observation_capability(self):
        for name, source in LEG_FIRMWARE.items():
            with self.subTest(firmware=name):
                self.assertIn("version-v1", source)
                self.assertIn("health-v1", source)
                self.assertIn("observe-stream-v1", source)
                self.assertIn("DBV1", source)
                self.assertIn('"DBH1,', source)
                self.assertIn('"observe on"', source)
                self.assertIn('"observe off"', source)

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

    def test_active_controllers_publish_aligned_can_angles_and_masks(self):
        for name in (
            "firmware_full_libs_neck.ino",
            "esp32_devkitc_v4_hybrid.ino",
            "esp32_devkit_v1.ino",
        ):
            source = LEG_FIRMWARE[name]
            with self.subTest(firmware=name):
                self.assertIn('DROPBEAR_TELEMETRY_PROTOCOL = "DB3"', source)
                self.assertIn("readMotorControlDegrees(index, controlDegrees)", source)
                self.assertIn("alignmentFaultMask", source)

    def test_observation_image_explicitly_enables_non_motion_queries(self):
        source = LEG_FIRMWARE["esp32_devkit_v1_observation_safe.ino"]
        self.assertIn("const bool OBSERVATION_ONLY_FIRMWARE = true", source)
        self.assertIn("const bool MOTOR_FEEDBACK_QUERY_ALLOWED = true", source)
        self.assertIn("const bool LEGACY_SERIAL_MOTION_ALLOWED = false", source)


if __name__ == "__main__":
    unittest.main()
