from pathlib import Path
import re
import unittest


SOURCE = Path(__file__).with_name("esp32_devkit_v1_observation_safe.ino").read_text()


class LowLevelSafetySourceTests(unittest.TestCase):
    def test_default_build_is_observation_only(self):
        self.assertIn("const bool OBSERVATION_ONLY_FIRMWARE = true;", SOURCE)
        self.assertIn("const bool LEGACY_SERIAL_MOTION_ALLOWED = false;", SOURCE)
        self.assertIn("const bool MOTOR_FEEDBACK_QUERY_ALLOWED = true;", SOURCE)
        self.assertIn("byte frame[8] = { 0x92", SOURCE)
        self.assertRegex(SOURCE, r"bool playMode\s*=\s*false;")

    def test_every_motion_sender_has_local_fail_closed_guard(self):
        for function in ("sendTorqueCommand", "sendStopCommand"):
            body = re.search(
                rf"void {function}\([^)]*\)\s*\{{(?P<body>.*?)\n\}}",
                SOURCE,
                re.DOTALL,
            )
            self.assertIsNotNone(body, function)
            self.assertIn("OBSERVATION_ONLY_FIRMWARE", body.group("body"))
            self.assertIn("!canReady", body.group("body"))
            self.assertIn("!isKnownActuatorId", body.group("body"))

    def test_stop_calls_use_real_can_ids(self):
        self.assertNotRegex(SOURCE, r"sendStopCommand\(\s*[ij]\s*\)")
        self.assertIn("sendStopCommand(ACTUATOR_IDS[i])", SOURCE)
        self.assertIn("sendStopCommand(ACTUATOR_IDS[j])", SOURCE)

    def test_observation_survives_can_and_play_state(self):
        sensor_task = re.search(
            r"void readAndComputeTask\([^)]*\)\s*\{(?P<body>.*?)\n\}",
            SOURCE,
            re.DOTALL,
        )
        self.assertIsNotNone(sensor_task)
        self.assertNotIn("playMode &&", sensor_task.group("body"))
        self.assertIn("printReadings();", sensor_task.group("body"))
        self.assertNotRegex(SOURCE, r"CAN bus initialization failed\.\"\);\s*while")

    def test_motor_feedback_is_separate_and_versioned(self):
        self.assertIn("byte frame[8] = { 0x92", SOURCE)
        self.assertIn("responseID - 0x100", SOURCE)
        self.assertIn("motorNativeDegrees", SOURCE)
        self.assertIn('Serial.print("DB2,")', SOURCE)
        self.assertIn('Serial.print("NA")', SOURCE)


if __name__ == "__main__":
    unittest.main()
