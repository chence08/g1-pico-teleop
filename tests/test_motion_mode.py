import math,os,sys,threading,unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'outputs'))
from g1_motion_mode import MotionModeMonitor,classify_mode

class MotionModeTests(unittest.TestCase):
    def row(self,**changes):
        row=dict(code=0,name='',receipt_s=9.9,last_success_s=9.9,release_requested=False,release_result=None,worker_alive=True)
        row.update(changes);return row
    def test_fresh_empty_name_is_released(self):self.assertEqual(classify_mode(self.row(),10),'released')
    def test_normal_ai_is_active(self):self.assertEqual(classify_mode(self.row(name='ai'),10),'active')
    def test_release_ack_does_not_confirm_release(self):
        self.assertEqual(classify_mode(self.row(name='ai',release_requested=True,release_result=dict(code=0,receipt_s=9.8)),10),'active')
    def test_dead_worker_cannot_reuse_confirmation(self):self.assertEqual(classify_mode(self.row(worker_alive=False),10),'fault')
    def test_stale_receipt_or_success_is_fault(self):
        self.assertEqual(classify_mode(self.row(receipt_s=9,last_success_s=9),10),'fault')
        self.assertEqual(classify_mode(self.row(code=3104,name=None,last_success_s=8.9),10),'fault')
    def test_timeout_never_reuses_empty_name(self):self.assertEqual(classify_mode(self.row(code=3104,name=None),10),'wait')
    def test_other_rpc_failure_is_fault(self):self.assertEqual(classify_mode(self.row(code=5,name=None),10),'fault')
    def test_malformed_and_future_fail_closed(self):
        for row in (self.row(name=None),self.row(code=True),self.row(receipt_s=math.nan),self.row(receipt_s=10.1,last_success_s=10.1),self.row(last_success_s=10.1)):
            self.assertEqual(classify_mode(row,10),'fault')
    def test_request_is_idempotent_single_byte(self):
        r,w=os.pipe();monitor=MotionModeMonitor.__new__(MotionModeMonitor)
        monitor._lock=threading.Lock();monitor._closed=monitor._release_sent=False
        monitor.process=SimpleNamespace(poll=lambda:None,stdin=SimpleNamespace(fileno=lambda:w))
        try:
            self.assertTrue(monitor.request_release());self.assertFalse(monitor.request_release())
            os.set_blocking(r,False);self.assertEqual(os.read(r,8),b'R')
        finally:os.close(r);os.close(w)
    def test_failed_request_not_delivered(self):
        monitor=MotionModeMonitor.__new__(MotionModeMonitor)
        monitor._lock=threading.Lock();monitor._closed=monitor._release_sent=False
        monitor.process=SimpleNamespace(poll=lambda:None,stdin=SimpleNamespace(fileno=lambda:123))
        with patch('g1_motion_mode.os.write',side_effect=BlockingIOError):
            with self.assertRaises(RuntimeError):monitor.request_release()
        self.assertFalse(monitor._release_sent)
