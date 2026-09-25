"""Host-compiled numerical tests for the reusable RMD wire codecs."""

from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest


ROOT = Path(__file__).parent


class MotorProtocolTests(unittest.TestCase):
    def test_wire_codec_vectors(self):
        compiler = shutil.which("g++")
        if compiler is None:
            self.skipTest("g++ is unavailable")
        with tempfile.TemporaryDirectory() as temporary:
            binary = Path(temporary) / "test_dropbear_motor_protocol"
            subprocess.run(
                [
                    compiler,
                    "-std=c++11",
                    "-Wall",
                    "-Wextra",
                    "-Werror",
                    "-I",
                    str(ROOT),
                    str(ROOT / "test_dropbear_motor_protocol.cpp"),
                    "-o",
                    str(binary),
                ],
                check=True,
                capture_output=True,
                text=True,
            )
            subprocess.run([str(binary)], check=True)


if __name__ == "__main__":
    unittest.main()
