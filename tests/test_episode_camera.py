import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import Mock
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'outputs'))
from g1_episode_camera import CameraWriter,CapturedFrame,FrameStats,quality_issues,verify_video
class Tests(unittest.TestCase):
    def test_bounded_queue(self):
        w=CameraWriter(Path('/unused'),1,False,None,None,queue_size=2);f=CapturedFrame(None,None,{})
        self.assertTrue(w.submit(f));self.assertTrue(w.submit(f));self.assertFalse(w.submit(f));self.assertEqual(w.dropped,1)
    def test_discontinuities(self):
        stats=FrameStats()
        for number,stamp,t,domain in [(10,100.,1000000000,'hardware'),(13,200.,1100000000,'hardware'),(13,200.,1200000000,'hardware'),(14,50.,1300000000,'system')]:
            stats.observe(dict(frame_number=number,device_timestamp_ms=stamp,host_receipt_monotonic_ns=t,timestamp_domain=domain))
        s=stats.summary();self.assertEqual(s['frame_number_gaps'],2);self.assertEqual(s['nonincreasing_frame_numbers'],1)
        self.assertEqual(s['nonincreasing_device_timestamps'],1);self.assertEqual(s['timestamp_domain_changes'],1)
    def test_video_count(self):
        runner=Mock(return_value=Mock(stdout=json.dumps({'streams':[dict(nb_read_frames='3',width=1280,height=720)]}),stderr=''))
        self.assertEqual(verify_video(Path('rgb.mp4'),3,runner)['decoded_frames'],3)
        with self.assertRaises(RuntimeError):verify_video(Path('rgb.mp4'),4,runner)
        self.assertIn('-count_frames',runner.call_args.args[0])
    def test_decode_error(self):
        runner=Mock(return_value=Mock(stdout=json.dumps({'streams':[dict(nb_read_frames='3',width=1280,height=720)]}),stderr='corrupt frame'))
        with self.assertRaises(RuntimeError):verify_video(Path('rgb.mp4'),3,runner)
    def test_incomplete(self):
        r=dict(errors=[],frames_written=10,queue_drops=0,capture_timeouts=0,rgb_capture=FrameStats().summary(),depth_capture=None,video_verification={'decoded_frames':10})
        self.assertEqual(quality_issues(r),[]);r['queue_drops']=1;self.assertTrue(quality_issues(r))
        r['queue_drops']=0;r['video_verification']=None;self.assertTrue(quality_issues(r))
    def test_encoder_failure(self):
        cv2=Mock();v=cv2.VideoWriter.return_value;v.isOpened.return_value=True;v.write.side_effect=OSError('disk full')
        with tempfile.TemporaryDirectory() as d:
            w=CameraWriter(Path(d),1,False,cv2,None);w.submit(CapturedFrame(object(),None,{'host_receipt_monotonic_ns':1}))
            w.capture_done.set();w.thread.start();w.thread.join(2)
            self.assertFalse(w.thread.is_alive());self.assertTrue(any('disk full' in e for e in w.errors));self.assertEqual(w.written,0);v.release.assert_called_once()
if __name__=='__main__':unittest.main()
