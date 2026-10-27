import json
from pathlib import Path
import socket
import sys
import time
import unittest
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "outputs"))
from pico_native_input import NativeReceiver, pose7_matrix, sdk_controller_event, BODY_NAMES
from g1_arm_preview import decode_controller_packet
from pico_native_bridge import effective_body_timestamp


def packet(stamp=1, held=False):
    body = [[0, 0, 0, 0, 0, 0, 1] for _ in range(24)]
    return {"schema": "pico-native-sdk-snapshot-v1", "session_id": "test",
            "host_poll_monotonic_s": time.monotonic(), "source_timestamp_ns": stamp,
            "head_pose": [0, 1.6, 0, 0, 0, 0, 1],
            "left_controller_pose": [-.2, 1.1, -.3, 0, 0, 0, 1],
            "right_controller_pose": [.2, 1.1, -.3, 0, 0, 0, 1],
            "left_grip": float(held), "right_grip": float(held),
            "body": {"source_timestamp_ns": stamp, "poses": body}}


class NativeTests(unittest.TestCase):
    def test_missing_body_timestamp_uses_real_joint_time(self):
        self.assertEqual(effective_body_timestamp(0, [0, 10, 12]), (12, "max_body_joint_timestamp"))
        self.assertEqual(effective_body_timestamp(20, [10, 12]), (20, "sdk_body_timestamp"))
        self.assertEqual(effective_body_timestamp(0, [0] * 24), (0, "unavailable"))

    def setUp(self):
        self.events, self.resets = [], []
        self.receiver = NativeReceiver(lambda event, t: self.events.append((event, t)),
                                       lambda: self.resets.append(True), port=0)

    def tearDown(self):
        self.receiver.close()

    def test_pose_and_grips_match_existing_preview_decoder(self):
        event = sdk_controller_event(packet(held=True))
        poses, grips = decode_controller_packet(event)
        np.testing.assert_allclose(poses[0][:3, 3], [-.2, 1.1, -.3])
        self.assertEqual(grips, [True, True])

    def test_invalid_quaternion_is_rejected(self):
        with self.assertRaises(ValueError):
            pose7_matrix([0] * 7)

    def test_stream_start_requires_grip_release(self):
        self.receiver.consume(packet(held=True), time.monotonic())
        self.assertFalse(self.events)
        self.receiver.consume(packet(2, held=False), time.monotonic())
        self.receiver.consume(packet(3, held=True), time.monotonic())
        self.assertEqual(len(self.events), 2)

    def test_repeated_sdk_timestamp_cannot_refresh_arm_input(self):
        self.receiver.consume(packet(), time.monotonic())
        row = packet()
        row["body"]["source_timestamp_ns"] = 2
        self.receiver.consume(row, time.monotonic())
        self.assertEqual(len(self.events), 1)
        self.assertEqual(self.receiver.body_stamp, 2)

    def test_stale_bridge_packet_is_rejected(self):
        row = packet()
        row["host_poll_monotonic_s"] -= 2
        with self.assertRaises(ValueError):
            self.receiver.consume(row, time.monotonic())

    def test_triggers_are_independent_optional_and_cannot_refresh_on_regressed_stamp(self):
        row=packet(10);row.update(left_trigger=.2,right_trigger=.8)
        self.receiver.consume(row,time.monotonic())
        self.assertEqual([self.events[-1][0][s+'State']['triggerValue'] for s in ('left','right')],[.2,.8])
        old=packet(9);old.update(left_trigger=1.,right_trigger=1.)
        self.receiver.consume(old,time.monotonic());self.assertEqual(len(self.events),1)
        event=sdk_controller_event(packet(11))
        self.assertNotIn('triggerValue',event['leftState'])
        for x in (float('nan'),-.1,1.1):
            with self.assertRaises(ValueError):sdk_controller_event(dict(row,left_trigger=x))

    def test_bridge_restart_requires_new_release(self):
        self.receiver.consume(packet(), time.monotonic())
        row = packet(2, held=True)
        row["session_id"] = "restarted"
        self.receiver.consume(row, time.monotonic())
        self.assertEqual(len(self.events), 1)
        self.assertEqual(len(self.resets), 2)

    def test_new_session_cannot_reuse_old_body_as_fresh_elbow_hint(self):
        self.receiver.consume(packet(10), time.monotonic())
        row=packet(11);row['session_id']='new';row['body']=None
        self.receiver.consume(row,time.monotonic())
        points,_=self.receiver.get_body_arm_points()
        self.assertIsNone(points)
        self.assertEqual(self.receiver.body_stamp,0)

    def test_regressed_body_timestamp_cannot_replace_current_estimate(self):
        self.receiver.consume(packet(10),time.monotonic())
        row=packet(11);row['body']['source_timestamp_ns']=9
        row['body']['poses'][18][0]=10
        self.receiver.consume(row,time.monotonic())
        self.assertEqual(self.receiver.body_stamp,10)
        self.assertEqual(self.receiver.body[18,0],0)

    def test_udp_transport_delivers_to_preview(self):
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sender:
            sender.sendto(json.dumps(packet()).encode(), self.receiver.sock.getsockname())
        deadline = time.monotonic() + 1
        while not self.events and time.monotonic() < deadline:
            time.sleep(.01)
        self.assertEqual(len(self.events), 1)

    def test_ankles_are_named_as_estimated_body_joints(self):
        self.assertEqual(BODY_NAMES[7:9], ["left_ankle", "right_ankle"])

    def test_leg_data_requires_two_updates_and_expires_independently(self):
        self.receiver.consume(packet(1),time.monotonic())
        self.assertIsNone(self.receiver.get_body_leg_data()[0])
        row=packet(2);row['body']['poses'][7][:3]=[1,2,3]
        self.receiver.consume(row,time.monotonic())
        poses,age,seq=self.receiver.get_body_leg_data()
        self.assertEqual(seq,2);np.testing.assert_allclose(poses[7,:3,3],[-3,-1,2])
        self.receiver.body_seen-=1
        row=packet(3);row['body']['source_timestamp_ns']=2
        self.receiver.consume(row,time.monotonic())
        self.assertIsNone(self.receiver.get_body_leg_data()[0])
        row=packet(4);row['session_id']='restart'
        self.receiver.consume(row,time.monotonic())
        self.assertEqual(self.receiver.body_sequence,1);self.assertIsNone(self.receiver.get_body_leg_data()[0])

    def test_invalid_body_quaternion_cannot_advance_body(self):
        self.receiver.consume(packet(1),time.monotonic())
        row=packet(2);row['body']['poses'][7][3:]=[0,0,0,0]
        self.receiver.consume(row,time.monotonic())
        self.assertEqual(self.receiver.body_sequence,1)


if __name__ == "__main__":
    unittest.main()
