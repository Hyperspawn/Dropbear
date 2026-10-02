from pathlib import Path
import re
import unittest

SOURCE = Path(__file__).with_name("esp32_devkitc_v4_hybrid.ino").read_text()


class CommLossWiringTests(unittest.TestCase):
    def test_header_included(self):
        self.assertIn('#include "dropbear_motor_protocol.h"', SOURCE)

    def test_arm_function_uses_encoder_for_selected_leg(self):
        body = re.search(r"void armMotorCommLossProtection\(\)\s*\{(.*?)\n\}", SOURCE, re.DOTALL)
        self.assertIsNotNone(body)
        self.assertIn("dropbear::encodeCommLossProtection(MOTOR_COMM_LOSS_TIMEOUT_MS", body.group(1))
        self.assertIn("actuatorBelongsToSelectedLeg", body.group(1))

    def test_armed_at_boot_before_control_tasks_start(self):
        arm = SOURCE.index("      armMotorCommLossProtection();")
        first_task = SOURCE.index('xTaskCreatePinnedToCore(readAndComputeTask')
        self.assertLess(arm, first_task)

    def test_timeout_is_nonzero(self):
        m = re.search(r"MOTOR_COMM_LOSS_TIMEOUT_MS\s*=\s*(\d+)", SOURCE)
        self.assertGreater(int(m.group(1)), 0)


if __name__ == "__main__":
    unittest.main()
