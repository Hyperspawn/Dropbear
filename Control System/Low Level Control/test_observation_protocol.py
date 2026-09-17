"""Static contracts for the non-motion USB observation protocol."""

from pathlib import Path
import unittest


ROOT = Path(__file__).parent
SOURCES = {
    name: (ROOT / name).read_text()
    for name in (
        "firmware_full_libs_neck.ino",
        "esp32_devkitc_v4_hybrid.ino",
        "esp32_devkit_v1.ino",
        "esp32_devkit_v1_observation_safe.ino",
    )
}


class ObservationProtocolContract(unittest.TestCase):
    def test_protocol_records_are_available_on_every_image(self):
        for name, source in SOURCES.items():
            with self.subTest(firmware=name):
                self.assertIn("void printVersionRecord()", source)
                self.assertIn("void printHealthRecord()", source)
                self.assertIn("DBV1", source)
                self.assertIn("DBH1", source)
                self.assertIn("telemetryStreamingEnabled", source)

    def test_observation_stream_is_independent_of_play(self):
        for name, source in SOURCES.items():
            with self.subTest(firmware=name):
                start = source.index('command.equalsIgnoreCase("observe on")') if "observation_safe" not in name else source.index('command == "observe on"')
                end = source.index("observe off", start)
                self.assertNotIn("playMode = true", source[start:end])

    def test_behemoth_requires_db1_and_legacy_images_advertise_bare_commands(self):
        self.assertIn('DROPBEAR_COMMAND_PROTOCOL = "DB1"', SOURCES["firmware_full_libs_neck.ino"])
        for name in (
            "esp32_devkitc_v4_hybrid.ino",
            "esp32_devkit_v1.ino",
            "esp32_devkit_v1_observation_safe.ino",
        ):
            self.assertIn('DROPBEAR_COMMAND_PROTOCOL = "LEGACY"', SOURCES[name])

    def test_portal_firmwares_expose_version_api(self):
        for name in (
            "firmware_full_libs_neck.ino",
            "esp32_devkitc_v4_hybrid.ino",
            "esp32_devkit_v1.ino",
        ):
            self.assertIn('server.on("/api/version", HTTP_GET, handleApiVersion)', SOURCES[name])


if __name__ == "__main__":
    unittest.main()
