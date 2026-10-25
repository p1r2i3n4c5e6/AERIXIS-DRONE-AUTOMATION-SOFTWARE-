"""Onboard model library: board lock rules, upload checks, active model, launcher fallback."""
import io
import json
import os
import shutil
import sys
import tempfile
import unittest
import zipfile
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
import model_library as ml  # noqa: E402

PERSON = ROOT / "model"
ALL = dict(ncnn=True, onnx=True, rknn=False, tensorrt=False, hef=False)


def zip_bytes(files):
    buffer = io.BytesIO()
    with zipfile.ZipFile(buffer, "w") as z:
        for name, data in files.items():
            z.writestr(name, data)
    return buffer.getvalue()


def ncnn_folder_zip(prefix="best_ncnn_model/", names="names:\n  0: person\n  1: dog\n"):
    return zip_bytes({prefix + "model.ncnn.param": (PERSON / "model.ncnn.param").read_bytes(),
                      prefix + "model.ncnn.bin": (PERSON / "model.ncnn.bin").read_bytes(),
                      prefix + "metadata.yaml": "task: detect\nimgsz:\n- 384\n- 640\n" + names})


class LibraryCase(unittest.TestCase):
    def setUp(self):
        self.dir = tempfile.mkdtemp()
        self.env = patch.dict(os.environ, {"A8_MODEL_LIBRARY": self.dir})
        self.env.start()

    def tearDown(self):
        self.env.stop()
        shutil.rmtree(self.dir, ignore_errors=True)


class LockRuleTests(LibraryCase):
    def models(self, board, available=ALL, policy=None, threads=2):
        return {m["name"]: m for m in ml.list_models(board, threads, available, policy)}

    def test_shipped_models_on_a_bare_pi5(self):
        models = self.models("pi5")
        self.assertTrue(models["person-n512-640x384"]["active"])
        self.assertFalse(models["person-n512-320x192"]["locked"])
        self.assertFalse(models["person-n512-640x384"]["locked"])
        big = models["person-n640-960x544"]
        self.assertTrue(big["locked"])                         # ~4.7 Hz estimated on 2 Pi 5 threads
        self.assertIn("detection", big["reasons"][0])
        self.assertEqual(big["speed"]["capture"], [960, 540])   # needs the bigger video, not upscaled 640

    def test_faster_board_or_lower_minimum_changes_the_lock(self):
        # The 960x544 model on the Axon's 4 CPU cores reaches ~9 Hz: enough at 3 m/s, not on a Pi 5.
        self.assertFalse(self.models("axon", threads=4, policy=dict(speed_ms=3))["person-n640-960x544"]["locked"])
        self.assertTrue(self.models("pi5", policy=dict(speed_ms=3))["person-n640-960x544"]["locked"])
        self.assertTrue(self.models("pi5", policy=dict(min_detect_hz=12))["person-n512-640x384"]["locked"])
        slow = dict(min_detect_hz=4, speed_ms=2)               # hovering / slow search: 4 Hz is enough
        self.assertFalse(self.models("pi5", policy=slow)["person-n640-960x544"]["locked"])

    def test_detection_needed_grows_with_drone_speed(self):
        self.assertEqual(ml.required_detect_hz(dict(min_detect_hz=5, speed_ms=2)), 5)
        self.assertEqual(ml.required_detect_hz(dict(min_detect_hz=5, speed_ms=5)), 10)
        self.assertEqual(ml.required_detect_hz(dict(min_detect_hz=5, speed_ms=8)), 16)
        fast = self.models("pi5", policy=dict(speed_ms=8))
        self.assertTrue(fast["person-n512-640x384"]["locked"])          # ~10 Hz on a Pi 5
        self.assertFalse(fast["person-n512-320x192"]["locked"])         # ~41 Hz
        self.assertIn("8 m/s", fast["person-n512-640x384"]["reasons"][0])
        self.assertEqual(ml.DEFAULT_POLICY["min_track_fps"], 19.0)

    def test_missing_runtime_or_wrong_format_locks(self):
        models = self.models("pi5", available=dict(ALL, ncnn=False))
        self.assertTrue(all(m["locked"] for m in models.values()))
        self.assertIn("runtime", models["person-n512-640x384"]["reasons"][0])

    def test_measured_speed_overrides_the_estimate(self):
        ml.save_benchmark("person-n640-960x544", "pi5", dict(detect_ms=150, detect_hz=6.7, track_fps=25,
                                                              capture=[960, 540]))
        big = self.models("pi5", policy=dict(speed_ms=3))["person-n640-960x544"]   # 6 Hz needed
        self.assertTrue(big["speed"]["measured"])
        self.assertFalse(big["locked"])

    def test_board_warning_when_the_choice_and_the_hardware_disagree(self):
        real = dict(guess="pi5", accelerators=[])
        self.assertTrue(ml.board_warnings("pi5-hailo8l", real))
        self.assertEqual(ml.board_warnings("pi5", real), [])
        self.assertTrue(ml.board_warnings("axon", real))

    def test_pi5_and_its_ai_hat_are_recognised(self):
        files = {"/proc/device-tree/model": "", "/proc/cpuinfo": "processor\t: 0\nCPU part\t: 0xd0b\n"
                 "Model\t\t: Raspberry Pi 5 Model B Rev 1.0\n", "/proc/meminfo": "MemTotal: 8000000 kB\n"}
        fake_read = lambda path, default="": files.get(str(path), default)
        with patch.object(ml, "_read", fake_read), patch.object(ml, "_exists", lambda p: False):
            board = ml.detect_board()
        self.assertEqual((board["guess"], board["model"], board["cpu"]), ("pi5", "Raspberry Pi 5 Model B Rev 1.0", "Cortex-A76"))
        with patch.object(ml, "_read", fake_read), patch.object(ml, "_exists", lambda p: p == "/dev/hailo0"), \
                patch.object(ml, "_hailo_arch", lambda: "hailo8"):
            self.assertEqual(ml.detect_board()["guess"], "pi5-hailo8")
        ml.set_board("auto")
        self.assertEqual(ml.load_active()["board"], "")

    def test_real_board_detection_runs_here(self):
        board = ml.detect_board()
        self.assertIn(board["guess"], ml.BOARDS)
        self.assertGreaterEqual(board["threads"], board["cores"])
        budget = ml.cpu_budget(0.2)
        self.assertTrue(1 <= budget["detector_threads"] <= 4)


class ActiveModelTests(LibraryCase):
    def test_locked_model_cannot_be_made_active(self):
        with self.assertRaisesRegex(ValueError, "locked"):
            ml.set_active("person-n640-960x544", [], [], "pi5", 2, ALL)

    def test_classes_must_exist_and_lock_classes_must_be_detected(self):
        with self.assertRaisesRegex(ValueError, "dog"):
            ml.set_active("person-n512-640x384", ["dog"], [], "pi5", 2, ALL)
        with self.assertRaisesRegex(ValueError, "among the detected"):
            ml.set_active("objects80-n-640x384", ["person"], ["dog"], "pi5", 2, ALL)
        data = ml.set_active("objects80-n-640x384", ["person", "dog"], ["dog"], "pi5", 2, ALL)
        self.assertEqual(ml.load_active()["lock_classes"], ["dog"])
        self.assertEqual(data["threads"], 2)

    def test_launcher_uses_the_active_model_and_falls_back_loudly(self):
        ml.set_active("objects80-n-640x384", ["person", "dog"], [], "pi5", 2, ALL)
        folder, classes, lock, capture, note = ml.active_launch(PERSON)
        self.assertEqual((folder.name, classes, capture, note), ("objects80-n-640x384", ["person", "dog"], (640, 360), ""))
        active = json.loads(Path(self.dir, "active.json").read_text())
        active["model"] = "deleted-model"
        Path(self.dir, "active.json").write_text(json.dumps(active))
        folder, classes, _, _, note = ml.active_launch(PERSON)
        self.assertEqual((folder, classes), (PERSON, []))
        self.assertIn("unusable", note)

    def test_tracker_wrapper_passes_model_classes_and_capture_size(self):
        sys.path.insert(0, str(ROOT.parent))
        from as2_a8_tracker import tracker_process
        ml.set_active("objects80-n-640x384", ["person", "dog"], ["dog"], "pi5", 2, ALL)
        args = SimpleNamespace(rtsp="rtsp://x", camera_ip="192.168.144.25", threads=2, command_cap=35,
                               min_person_height=24, telemetry_port=9090, control_port=9091, log_dir="/tmp/x",
                               cores=[0, 1], detector_cores=[2, 3], stream=0, mode="track")
        with patch.object(tracker_process, "tracker_home", return_value=ROOT):
            command = tracker_process.build_command(args, [])
        text = " ".join(command)
        self.assertIn("--model %s" % (ROOT / "models" / "objects80-n-640x384"), text)
        self.assertIn("--nano-dir %s" % (ROOT / "model"), text)
        self.assertIn("--classes person dog", text)
        self.assertIn("--lock-classes dog", text)
        self.assertIn("--threads 2", text)
        self.assertIn("--res 640x360", text)


class UploadTests(LibraryCase):
    def test_zip_of_an_export_folder_is_added_and_deleted(self):
        folder = ml.add_model("pet-n-640x384", "best_ncnn_model.zip", ncnn_folder_zip())
        self.assertTrue((folder / "model.ncnn.bin").exists())
        names = {m["name"]: m for m in ml.list_models("pi5", 2, ALL)}
        self.assertEqual(names["pet-n-640x384"]["names"], {0: "person", 1: "dog"})
        with self.assertRaisesRegex(ValueError, "already exists"):
            ml.add_model("pet-n-640x384", "a.zip", ncnn_folder_zip())
        ml.delete_model("pet-n-640x384")
        self.assertFalse(folder.exists())

    def test_missing_metadata_is_filled_from_the_form(self):
        data = ncnn_folder_zip(names="")
        with self.assertRaisesRegex(ValueError, "Class names unknown"):
            ml.add_model("nonames", "a.zip", data)
        folder = ml.add_model("named", "a.zip", data, class_names=["boat", "person"])
        self.assertIn("boat", (folder / "metadata.yaml").read_text())
        with self.assertRaisesRegex(ValueError, "multiples of 32"):
            ml.add_model("badsize", "a.zip", data, class_names=["x"], imgsz=[100, 640])

    def test_unsafe_or_wrong_uploads_are_refused(self):
        evil = zip_bytes({"../escape/model.ncnn.param": "x", "../escape/model.ncnn.bin": "x"})
        with self.assertRaisesRegex(ValueError, "Unsafe path"):
            ml.add_model("evil", "evil.zip", evil)
        self.assertFalse(Path(self.dir).parent.joinpath("escape").exists())
        with self.assertRaisesRegex(ValueError, "export it first"):
            ml.add_model("pt", "best.pt", b"x")
        with self.assertRaisesRegex(ValueError, "letters, digits"):
            ml.add_model("../x", "a.zip", ncnn_folder_zip())
        two = zip_bytes({"a/model.onnx": "x", "b/model.onnx": "x"})
        with self.assertRaisesRegex(ValueError, "several model folders"):
            ml.add_model("two", "two.zip", two)
        with self.assertRaisesRegex(ValueError, "No model file"):
            ml.add_model("empty", "a.zip", zip_bytes({"readme.txt": "hi"}))
        self.assertEqual([p.name for p in Path(self.dir).iterdir()], [])   # nothing left behind

    def test_shipped_and_active_models_cannot_be_deleted(self):
        with self.assertRaisesRegex(ValueError, "shipped"):
            ml.delete_model("person-n512-320x192")
        ml.add_model("mine", "a.zip", ncnn_folder_zip())
        ml.set_active("mine", [], [], "pi5", 2, ALL)
        with self.assertRaisesRegex(ValueError, "active model"):
            ml.delete_model("mine")


if __name__ == "__main__":
    unittest.main()
