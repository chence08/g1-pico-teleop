import importlib.util
from pathlib import Path
import unittest
import numpy as np

path = Path(__file__).resolve().parents[1] / "outputs/g1_arm_preview.py"
spec = importlib.util.spec_from_file_location("preview", path)
preview = importlib.util.module_from_spec(spec)
spec.loader.exec_module(preview)


class PreviewRegressionTests(unittest.TestCase):
    def test_short_gap_freezes_without_forcing_grip_reset(self):
        self.assertEqual(preview.input_mode(0.3, [True, True], True), ("gap", True))
        self.assertEqual(preview.input_mode(0.02, [True, True], True), ("follow", True))

    def test_long_gap_requires_release(self):
        self.assertEqual(preview.input_mode(1.1, [True, True], True), ("lost", False))
        self.assertEqual(preview.input_mode(0.02, [True, True], False), ("release", False))
        self.assertEqual(preview.input_mode(0.02, [False, False], False), ("clutch", True))

    def test_one_grip_release_clutches(self):
        self.assertEqual(preview.input_mode(0.02, [True, False], True)[0], "clutch")

    def test_complete_packet_is_decoded_atomically(self):
        left, right = np.eye(4), np.eye(4)
        left[:3, 3] = [0.2, 1.1, -0.4]
        packet = {"left": left.flatten(order="F").tolist(),
                  "right": right.flatten(order="F").tolist(),
                  "leftState": {"squeeze": True}, "rightState": {"squeezeValue": 1}}
        poses, grips = preview.decode_controller_packet(packet)
        np.testing.assert_allclose(poses[0], left)
        self.assertEqual(grips, [True, True])
        del packet["rightState"]
        with self.assertRaises(ValueError):
            preview.decode_controller_packet(packet)

    def test_no_jump_on_engagement_or_gap_reanchor(self):
        poses = [np.eye(4), np.eye(4)]
        robot = [np.eye(4), np.eye(4)]
        robot[0][:3, 3] = [0.2, 0.1, 0.3]
        for result, expected in zip(preview.relative_targets(poses, poses, robot, 0.5), robot):
            np.testing.assert_allclose(result, expected)

    def test_head_rotation_cannot_change_fixed_reference(self):
        poses = [np.eye(4), np.eye(4)]
        poses[0][2, 3] = -0.2
        out = preview.controller_reference(poses, np.eye(3))
        np.testing.assert_allclose(out[0][:3, 3], [0.2, 0, 0])
        # Transform depends only on controller poses and the engagement anchor.
        again = preview.controller_reference(poses, np.eye(3))
        np.testing.assert_allclose(again, out)


if __name__ == "__main__":
    unittest.main()
