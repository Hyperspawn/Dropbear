"""Static safety contract for Behemoth motor-native observation telemetry."""

from pathlib import Path
import unittest


SOURCE = (Path(__file__).parent / "firmware_full_libs_neck.ino").read_text()


class BehemothMotorFeedbackContract(unittest.TestCase):
    def test_read_only_rmd_query_and_decoder_are_present(self):
        self.assertIn("const byte request[8] = {0x92, 0, 0, 0, 0, 0, 0, 0}", SOURCE)
        self.assertIn("data[0] != 0x92", SOURCE)
        self.assertIn("responseID - 0x100", SOURCE)
        self.assertIn("data[1] != 0 || data[2] != 0 || data[3] != 0", SOURCE)
        self.assertIn("static_cast<float>(signedRaw) * 0.01f", SOURCE)

    def test_motor_feedback_is_consumed_before_control_route_frames(self):
        native = SOURCE.index("ingestMotorNativeFeedback(static_cast<uint32_t>(rxId), data, len)")
        route = SOURCE.index("handleHyperspawnRxFrame(static_cast<uint32_t>(rxId), data, len)", native)
        self.assertLess(native, route)

    def test_db2_keeps_five_external_and_six_motor_fields(self):
        self.assertIn('Serial.print("DB2,")', SOURCE)
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

    def test_identity_and_diagnostics_name_the_feedback_protocol(self):
        self.assertIn("behemoth-db2-native-angle-2026.09.15", SOURCE)
        self.assertIn("rmd_v44_0x92_multi_turn", SOURCE)


if __name__ == "__main__":
    unittest.main()
