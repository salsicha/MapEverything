import importlib.util
from pathlib import Path
import sqlite3
import sys
import tempfile
import unittest
from unittest.mock import patch


spec = importlib.util.spec_from_file_location(
    "bag_converter", Path(__file__).resolve().parents[1] / "mapeverything-local-bag-to-ros2.py"
)
converter = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = converter
spec.loader.exec_module(converter)


class ConverterOutputSafetyTests(unittest.TestCase):
    def test_overlapping_outputs_preserve_input_before_loading_ros(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            session = root / "session"
            session.mkdir()
            chunk = session / "recording.db3"
            with sqlite3.connect(chunk) as database:
                database.execute("CREATE TABLE evidence (value TEXT)")
                database.execute("INSERT INTO evidence VALUES ('original recording')")
            original_bytes = chunk.read_bytes()
            alias = root / "alias"
            alias.symlink_to(session, target_is_directory=True)
            input_alias = root / "input.db3"
            input_alias.symlink_to(chunk)

            for source, output in [
                (chunk, session),
                (chunk, root),
                (chunk, chunk),
                (chunk, session / "unused" / ".."),
                (chunk, alias),
                (input_alias, session),
            ]:
                with self.subTest(source=source, output=output):
                    with patch.object(converter.shutil, "rmtree") as remove:
                        with self.assertRaisesRegex(converter.BagConversionError, "Output contains an input"):
                            converter.convert([source], output, "sqlite3", {}, set(), False, True, False)
                        remove.assert_not_called()
                    self.assertEqual(chunk.read_bytes(), original_bytes)

    def test_all_input_chunks_are_checked(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            output = root / "second-session"
            with self.assertRaisesRegex(converter.BagConversionError, "Output contains an input"):
                converter.convert(
                    [root / "first.db3", output / "second.db3"],
                    output, "sqlite3", {}, set(), False, True, False,
                )

    def test_separate_output_proceeds_to_conversion(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            # A deliberately unavailable ROS dependency marks the point after
            # path validation without requiring ROS to be installed in CI.
            with patch.dict(sys.modules, {"rclpy.serialization": None}):
                with self.assertRaisesRegex(converter.BagConversionError, "Unable to import rclpy"):
                    converter.convert(
                        [root / "input" / "recording.db3"], root / "output",
                        "sqlite3", {}, set(), False, True, False,
                    )


if __name__ == "__main__":
    unittest.main()
