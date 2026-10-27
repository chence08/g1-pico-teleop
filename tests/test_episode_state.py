import json
from pathlib import Path
import queue
import sys
import tempfile
import time
from types import SimpleNamespace as NS
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'outputs'))
import g1_episode_state as rec

def state():
    return NS(motor_state=[NS(q=i*.01,dq=i*.02,tau_est=-i*.03,mode=1,motorstate=i,temperature=[30+i,31+i],vol=48.) for i in range(35)],mode_pr=0,mode_machine=6,tick=2**32-2)
def command():
    motors=[NS(mode=1,q=i*.1,dq=-i*.01,tau=.3,kp=60.,kd=1.5) for i in range(35)];motors[29].q=.73
    return NS(motor_cmd=motors,mode_pr=0,mode_machine=6,crc=2**32-1)
class Tests(unittest.TestCase):
    def test_measured_and_clocks(self):
        row=rec.serialize_sample('rt/lowstate',state(),1793007271662311656,1234567890123,1793007271669000001,7,99)
        self.assertEqual(len(row['q']),29);self.assertAlmostEqual(row['tau_est'][28],-.84)
        self.assertEqual(row['motor_temperature'][0],[30,31]);self.assertEqual(row['motor_status'][28],28)
        self.assertEqual(row['motor_voltage'],[48.]*29);self.assertEqual(row['dds_source_timestamp_ns'],1793007271662311656)
        self.assertEqual(row['host_receipt_monotonic_ns'],1234567890123);self.assertEqual(row['dds_publication_handle'],99)
    def test_commands_not_execution(self):
        for topic in ('rt/lowcmd','rt/arm_sdk'):
            row=rec.serialize_sample(topic,command(),100,200,300,1)
            self.assertEqual(len(row['motor_cmd']['q']),35);self.assertAlmostEqual(row['motor_cmd']['q'][34],3.4)
            self.assertEqual(row['arm_weight_slot29'],.73);self.assertIs(row['execution_confirmed'],False)
    def test_invalid(self):
        msg=state();msg.motor_state[7].q=float('nan');row=rec.serialize_sample('rt/lowstate',msg,100,200,300,1)
        self.assertIsNone(row['q'][7]);self.assertEqual(row['invalid_numeric_fields'],['q[7]']);json.dumps(row,allow_nan=False)
        with self.assertRaises(ValueError):rec.serialize_sample('rt/lowstate',msg,0,200,300,1)
    def test_per_writer_dedup_reset(self):
        s=rec.TopicStats('rt/lowcmd');self.assertIsNotNone(s.observe(1000,10000,1));self.assertIsNone(s.observe(1000,10001,1))
        self.assertIsNotNone(s.observe(1000,10002,2));reset=s.observe(2,10003,1)
        self.assertTrue(reset['source_clock_regressed']);self.assertEqual(reset['source_clock_epoch'],1)
        self.assertEqual(s.duplicates,1);self.assertEqual(s.regressions,1)
    def test_delay_variation(self):
        s=rec.TopicStats('rt/lowstate');stamp=1_793_000_000_000_000_000
        self.assertEqual(s.observe(stamp,1_000_000_000,1)['delivery_excess_ns'],0)
        self.assertEqual(s.observe(stamp+2_000_000,1_003_000_000,1)['delivery_excess_ns'],1_000_000)
        self.assertEqual(s.observe(stamp+4_000_000,1_004_000_000,1)['delivery_excess_ns'],0)
    def test_bounded_queue(self):
        w=rec.BoundedWriter.__new__(rec.BoundedWriter);w.queue=queue.Queue(maxsize=1);w.stats={t:rec.TopicStats(t) for t in rec.TOPICS}
        self.assertTrue(w.submit({'topic':'rt/lowstate'}));self.assertFalse(w.submit({'topic':'rt/lowstate'}));self.assertEqual(w.stats['rt/lowstate'].queue_drops,1)
    def test_quality_idle_commands_vs_missing_state(self):
        stats={t:rec.TopicStats(t) for t in rec.TOPICS};s=stats['rt/lowstate'];s.observe(1,1_000_000_000,1);s.observe(2,1_002_000_000,1);s.written=2
        w=NS(ready=True,error=None,queue=queue.Queue(maxsize=1))
        self.assertTrue(rec.result_summary(stats,1_000_000_000,1_003_000_000,'STOP file',None,w)['complete'])
        self.assertFalse(rec.result_summary(stats,1_000_000_000,1_203_000_000,'STOP file',None,w)['complete'])
    def test_writer_readiness_and_split(self):
        with tempfile.TemporaryDirectory() as folder:
            output=Path(folder);stats={t:rec.TopicStats(t) for t in rec.TOPICS};w=rec.BoundedWriter(output,stats)
            msg=state();msg.motor_state[0].q=float('nan');w.submit(rec.serialize_sample('rt/lowstate',msg,100,200,300,1))
            deadline=time.monotonic()+1
            while stats['rt/lowstate'].written<1 and time.monotonic()<deadline:time.sleep(.001)
            self.assertFalse((output/'state_ready.json').exists());w.submit(rec.serialize_sample('rt/lowstate',state(),101,201,301,2))
            w.submit(rec.serialize_sample('rt/arm_sdk',command(),102,202,302,1));w.finish();self.assertIsNone(w.error)
            self.assertEqual(len((output/'state.jsonl').read_text().splitlines()),2);self.assertEqual(len((output/'commands.jsonl').read_text().splitlines()),1)
            self.assertEqual(json.loads((output/'state_ready.json').read_text())['dds_source_timestamp_ns'],101)
    def test_never_overwrite(self):
        with tempfile.TemporaryDirectory() as folder:
            p=Path(folder)/'state.jsonl';p.write_text('sentinel')
            with self.assertRaises(ValueError):rec.record(folder,1,'unused')
            self.assertEqual(p.read_text(),'sentinel')
if __name__=='__main__':unittest.main()
