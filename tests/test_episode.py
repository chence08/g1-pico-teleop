import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'outputs'))
import g1_episode as rec
class Tests(unittest.TestCase):
    def test_clock_bounds_cover_asymmetric_network(self):
        # True offset100ns; outbound10ns,remoteprocessing5ns,return30ns.
        r=rec.clock_bounds(1000,1110,1115,1045)
        self.assertEqual((r['offset_lower_ns'],r['offset_upper_ns']),(70,110))
        self.assertLessEqual(r['offset_lower_ns'],100);self.assertGreaterEqual(r['offset_upper_ns'],100)
        with self.assertRaises(ValueError):rec.clock_bounds(100,50,100,101)
    def test_index_rejects_future_and_stale_and_keeps_per_hand_times(self):
        with tempfile.TemporaryDirectory() as d:
            p=Path(d);(p/'pc_logs').mkdir()
            def rows(name,value): (p/name).write_text(''.join(json.dumps(x)+'\n' for x in value))
            rows('camera_frames.jsonl',[{'video_index':i,'host_receipt_monotonic_ns':t} for i,t in enumerate([99_000_000,110_000_000,201_000_000])])
            rows('state.jsonl',[{'host_receipt_monotonic_ns':100_000_000},{'host_receipt_monotonic_ns':200_000_000,'delivery_excess_ns':60_000_000}])
            rows('commands.jsonl',[{'topic':'rt/arm_sdk','host_receipt_monotonic_ns':105_000_000}])
            rows('pc_logs/hands.jsonl',[{'record':{'measurement_monotonic_s':[.1,.12]}}])
            summary=rec.build_index(p);data=rec.read_rows(p/'frame_index.jsonl')
            self.assertEqual(summary['frames_missing_recent_state'],2);self.assertIsNone(data[0]['state_row'])
            self.assertEqual(data[1]['state_row'],0);self.assertEqual(data[1]['hand_rows'],[0,None])
            self.assertIsNone(data[0]['command_rows']['rt/arm_sdk'])
    def test_incomplete_inspect_returns_nonzero(self):
        with tempfile.TemporaryDirectory() as d:
            p=Path(d);rec.save_json(p/'manifest.json',{'remote_exit_code':1})
            result=subprocess.run([sys.executable,str(rec.PROJECT/'outputs/g1_episode.py'),'inspect',str(p)],capture_output=True,text=True)
            self.assertEqual(result.returncode,1);self.assertFalse(rec.read_json(p/'quality.json')['capture_complete'])
    def test_hash_stream_matches_reference(self):
        with tempfile.TemporaryDirectory() as d:
            p=Path(d)/'data';p.write_bytes(b'hello'*400000)
            self.assertEqual(rec.file_hash(p),rec.hashlib.sha256(p.read_bytes()).hexdigest())
if __name__=='__main__':unittest.main()
