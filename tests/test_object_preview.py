import json
from pathlib import Path
import sys
import unittest
from unittest.mock import MagicMock

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "outputs"))
from pico_tracking_probe import motion_poses, ObjectViewer


def raw(count=2):
    return json.dumps({"value": {"Motion": {"len": count, "joints": [
        {"sn": "a", "p": "0,0,0,0,0,0,1"},
        {"sn": "b", "p": "0.2,0,0,0,0,0,1"}]}}})


class ObjectPreviewTests(unittest.TestCase):
    def test_count_prevents_old_retained_joints_from_appearing(self):
        self.assertEqual(len(motion_poses(raw(0))), 0)
        self.assertEqual(len(motion_poses(raw(1))), 1)
        self.assertEqual(len(motion_poses(raw())), 2)
        self.assertEqual(motion_poses('{"value":{}}'), [])

    def test_invalid_pose_is_rejected(self):
        with self.assertRaises(ValueError):
            motion_poses(raw().replace("0,0,0,0,0,0,1", "nan,0,0,0,0,0,1"))

    def test_stale_and_removed_markers_hide_with_stable_identity(self):
        viewer = ObjectViewer.__new__(ObjectViewer)
        viewer.viewer = MagicMock()
        viewer.geometry = MagicMock()
        viewer.identities, viewer.origin = {}, None
        poses = motion_poses(raw())
        viewer.update(poses, 10, 10)
        self.assertEqual(viewer.identities, {"a": 0, "b": 1})
        np.testing.assert_allclose(viewer.origin, [0, -.1, 0])
        viewer.viewer.reset_mock()
        viewer.update(poses[::-1], 10, 10.2)
        self.assertEqual(viewer.identities, {"a": 0, "b": 1})
        viewer.update(poses, 10, 10.6)
        viewer.viewer["raw_trackers"].set_property.assert_called_with("visible", False)
        viewer.update([], 11, 11)
        viewer.viewer["raw_trackers"].set_property.assert_called_with("visible", False)


if __name__ == "__main__":
    unittest.main()
