from pathlib import Path
import sys,threading,time,unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'outputs'))
from pico_native_bridge import AsyncDiagnosticLog,control_snapshot
from pico_native_input import NativeReceiver


class BridgePipelineTests(unittest.TestCase):
    def test_compact_packet_preserves_receiver_and_acquisition_timestamps(self):
        identity=[0,0,0,0,0,0,1]
        row={'schema':'pico-native-sdk-snapshot-v1','session_id':'compact',
             'source_timestamp_ns':123,'host_poll_monotonic_s':time.monotonic(),
             'head_pose':identity,'left_controller_pose':identity,'right_controller_pose':identity,
             'left_grip':0.,'right_grip':0.,'left_trigger':.2,'right_trigger':.8,
             'body':{'source_timestamp_ns':120,'poses':[identity]*24,'velocities':[[999]*6]*24},
             'motion':{'unconsumed':'metadata'},'buttons':{'A':True}}
        small=control_snapshot(row)
        self.assertEqual((small['left_trigger'],small['right_trigger']),(.2,.8))
        self.assertNotIn('motion',small);self.assertNotIn('velocities',small['body'])
        events=[];receiver=NativeReceiver(lambda e,t:events.append(t),lambda:None,port=0)
        try:
            receiver.consume(small,time.monotonic())
            self.assertEqual(events,[row['host_poll_monotonic_s']])
            self.assertEqual(receiver.body_stamp,120)
            self.assertEqual(receiver.last_stamp,123)
        finally:receiver.close()

    def test_disk_backpressure_drops_diagnostics_without_blocking_submission(self):
        entered=threading.Event();resume=threading.Event()
        class Stream:
            closed=False
            def __init__(self):self.lines=[]
            def write(self,line):
                entered.set();resume.wait(timeout=2);self.lines.append(line)
            def flush(self):pass
            def close(self):self.closed=True
        stream=Stream()
        class FakePath:
            def open(self,*args,**kwargs):return stream
        with AsyncDiagnosticLog(FakePath()) as log:
            log.write('first\n');self.assertTrue(entered.wait(timeout=1))
            for _ in range(500):log.write('next\n')
            self.assertGreater(log.dropped,0)
            self.assertEqual(log.queue.qsize(),128)
            resume.set()
        self.assertTrue(stream.closed)
        self.assertEqual(len(stream.lines)+log.dropped,501)
