"""Real controller loop with fake DDS/motion services; never connects hardware."""
import argparse,copy,io,json,signal,sys,threading,time,types,unittest
from pathlib import Path
from unittest.mock import patch
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'outputs'))
import g1_suspended_body as body
import g1_arm_datagram as codec
from g1_arm_link import ArmLink
from g1_leg_input import LegMotionGate


class ValidationTests(unittest.TestCase):
    def test_equal_body_sequence_cannot_renew_lifetime(self):
        clock=body.LegObservationClock()
        self.assertAlmostEqual(clock.accept(2,.01,10),.01)
        self.assertAlmostEqual(clock.accept(2,0,10.05),.06)
        with self.assertRaises(ValueError):clock.accept(2,0,10.16)
        clock.accept(3,.01,10.17)
        with self.assertRaises(ValueError):clock.accept(2,0,10.18)

    def test_submit_rejects_nan_and_invalid_sequence(self):
        link=ArmLink.__new__(ArmLink);link.lock=threading.Lock()
        for stamp,seq,verified in [(float('nan'),2,True),(time.monotonic()+1,2,True),(0,True,True),(0,2,1)]:
            with self.assertRaises(ValueError):link.submit_legs([0.]*12,stamp,seq,verified)

    def test_motion_check_is_pelvis_relative_and_requires_both_ankles(self):
        gate=LegMotionGate();poses=np.repeat(np.eye(4)[None],24,axis=0)
        self.assertFalse(gate.observe(poses,2))
        poses[:,:3,3]+=[1,2,3]
        self.assertFalse(gate.observe(poses,3))
        poses[7,2,3]+=.04
        self.assertFalse(gate.observe(poses,4))
        poses[8,2,3]+=.04
        self.assertTrue(gate.observe(poses,5));gate.reset();self.assertFalse(gate.verified)

    def test_shutdown_gain_never_increases(self):
        values=[body.damping_gain(.2,t) for t in [0,.1,.5,1,2]]
        self.assertEqual(values[0],.2);self.assertEqual(values[-1],0)
        self.assertTrue(all(a>=b for a,b in zip(values,values[1:])))


class RuntimeTests(unittest.TestCase):
    def run_scenario(self,live=True,scenario='normal'):
        writes=[];rows=[];events=[];stopped=threading.Event();eof=threading.Event();callbacks={};destination=[]
        initial=np.zeros(29);initial[:12]=[-.23,0,0,.48,-.24,0,-.25,0,0,.47,-.19,0]
        initial[15:]=[.29,.21,0,.98,.15,0,0,.29,-.21,0,.98,-.18,0,0]
        def motor(q=0.):return types.SimpleNamespace(q=float(q),dq=0.,tau_est=0.,mode=0,tau=0.,kp=0.,kd=0.)
        state=types.SimpleNamespace(motor_state=[motor(q) for q in initial]+[motor() for _ in range(6)],mode_machine=6,tick=0)
        class Capture:
            def __init__(self,_interface,callback):
                def produce():
                    while not stopped.is_set():
                        state.capture_received_s=time.monotonic();state.capture_delay_s=0.
                        state.source_timestamp_ns=time.time_ns();state.state_ipc_drops=0;state.tick+=1;callback(state);stopped.wait(.001)
                threading.Thread(target=produce,daemon=True).start()
            def Close(self):stopped.set()
        class Subscriber:
            def __init__(self,topic,_type):self.topic=topic
            def Init(self,callback,_q):callbacks[self.topic]=callback
            def Close(self):pass
        class Commands:
            def __init__(self,_interface,callback):
                callbacks['rt/lowcmd']=lambda msg:callback(1,msg.crc,time.monotonic())
                callbacks['rt/arm_sdk']=lambda msg:callback(2,msg.crc,time.monotonic())
            def healthy(self):return True
            def Close(self):pass
        class Mode:
            def __init__(self,_interface):self.requested=None
            def snapshot(self,timeout=0):
                now=time.monotonic();released=self.requested is not None and now-self.requested>.07
                return dict(status='released' if released else 'active',name='' if released else 'ai',code=0,last_success_age_s=0.,
                            receipt_s=now,release_result={'code':0,'receipt_s':self.requested+.06} if released else None)
            def request_release(self):
                self.requested=time.monotonic();events.append(('release',self.requested))
                if scenario=='handoff_tracking_bias':
                    state.motor_state[10].q=float(initial[10]-.25-.025)
                if scenario=='eof_pending':eof.set()
            def close(self):events.append(('mode_close',time.monotonic()))
        class Publisher:
            def __init__(self,topic,_type):assert topic=='rt/lowcmd'
            def Init(self):time.sleep(.02);events.append(('init',time.monotonic()))
            def Write(self,cmd):
                if scenario=='write_failure' and not any(e[0]=='write_failure' for e in events):
                    events.append(('write_failure',time.monotonic()));return False
                writes.append((time.monotonic(),copy.deepcopy(cmd)));callbacks['rt/lowcmd'](cmd)
                if scenario=='foreign' and len(writes)==170:
                    events.append(('foreign',time.monotonic()));callbacks['rt/lowcmd'](types.SimpleNamespace(crc=-999))
                return True
            def Close(self):events.append(('publisher_close',time.monotonic()))
        class CRC:
            count=0
            def Crc(self,cmd):self.count+=1;return self.count
        def factory():return types.SimpleNamespace(motor_cmd=[motor() for _ in range(35)],crc=0)
        class Geometry:
            def __init__(self,_urdf):pass
            def limits(self,q):
                lo=q-1.;hi=q+1.;lo[12:15]=hi[12:15]=q[12:15]
                if scenario=='handoff_tracking_bias':
                    lo[10]=q[10]-.25;hi[10]=q[10]+.25
                return lo,hi
            def leg_clear(self,q):return True
        class Workspace:
            def __init__(self,_path):pass
            def clearance(self,q):return np.ones(2)
            def interpolate(self,q,v,p,pv,measured):
                if scenario=='workspace_exception':raise RuntimeError('injected workspace exception')
                return p,pv,[False,False],[1.,1.]
        class Gravity:
            def __init__(self,*_):self.calibration_clipped=np.zeros(14,dtype=bool)
            def evaluate(self,q,return_info=False):return np.zeros(14),dict(saturated=np.zeros(14,dtype=bool))
        class Log:
            def __init__(self,_path):self.dropped=0
            def write(self,row):pass
            def close(self):pass
        class Transport:
            def __init__(self,output):self.count=0;self.start=None
            def read(self,incoming,ending):
                destination.append(incoming)
                while not ending.wait(.01):pass
            def send(self,row):
                rows.append(row);now=time.monotonic()
                if not destination:return
                if self.start is None:self.start=now
                self.count+=1;elapsed=now-self.start
                follow=elapsed>.08
                if scenario=='handoff_tracking_bias' and elapsed>.3:follow=False
                if scenario=='withdraw_pending' and events and any(e[0]=='release' for e in events):follow=False
                command=dict(seq=self.count,challenge=row['challenge'],q=initial[15:].tolist(),input_age_s=.005,follow=follow,
                             leg_q=initial[:12].tolist(),body_input_age_s=.005,body_sequence=self.count+2,body_motion_verified=True)
                command['q'][0]+=.05;command['leg_q'][3]+=.05
                destination[0].append((command,now))
            def close(self):pass
        class Input:
            def read(self,n):eof.wait(5);return ''
        leaves={
            'unitree_sdk2py.core.channel':dict(ChannelFactoryInitialize=lambda *_:None,ChannelSubscriber=Subscriber,ChannelPublisher=Publisher),
            'unitree_sdk2py.idl.unitree_hg.msg.dds_':dict(LowCmd_=object),
            'unitree_sdk2py.idl.default':dict(unitree_hg_msg_dds__LowCmd_=factory),
            'unitree_sdk2py.utils.crc':dict(CRC=CRC)}
        modules={}
        for name,values in leaves.items():
            parts=name.split('.')
            for i in range(1,len(parts)+1):
                prefix='.'.join(parts[:i])
                if prefix not in modules:modules[prefix]=types.ModuleType(prefix);modules[prefix].__path__=[]
            vars(modules[name]).update(values)
        handlers={s:signal.getsignal(s) for s in (signal.SIGINT,signal.SIGTERM)};output=io.StringIO()
        try:
            with patch.dict(sys.modules,modules),patch.object(sys,'stdin',Input()),patch.object(sys,'stdout',output), \
                    patch.object(body,'LowStateMonitor',Capture),patch.object(body,'MotionModeMonitor',Mode), \
                    patch.object(body,'CommandMonitor',Commands), \
                    patch.object(body,'BodyGeometry',Geometry),patch('g1_arm_workspace.ArmWorkspace',Workspace), \
                    patch.object(body,'CalibratedGravity',Gravity),patch.object(body,'DiagnosticLog',Log), \
                    patch.object(body,'ServerDatagrams',Transport),patch.object(body,'walking_client_connected',return_value=False):
                body.server(argparse.Namespace(interface='fake',live=live,seconds=1.1))
        finally:
            stopped.set();eof.set()
            for s,h in handlers.items():signal.signal(s,h)
        return writes,rows,events,output.getvalue()

    def test_shadow_never_releases_mode_or_writes_motors(self):
        writes,rows,events,_=self.run_scenario(live=False)
        self.assertFalse(writes);self.assertFalse(any(e[0]=='release' for e in events));self.assertTrue(rows)

    def test_live_owns_29_motors_and_ends_with_damping_dwell(self):
        writes,rows,events,_=self.run_scenario()
        self.assertGreater(len(writes),200)
        release=next(t for e,t in events if e=='release');self.assertGreater(writes[0][0],release+.07)
        for _,cmd in writes:
            self.assertEqual(cmd.mode_pr,0)
            self.assertTrue(all(m.mode==1 for m in cmd.motor_cmd[:29]))
            self.assertTrue(all(m.mode==0 and m.q==0 for m in cmd.motor_cmd[29:]))
        zero=[t for t,cmd in writes if cmd.motor_cmd[3].kp==0]
        self.assertGreaterEqual(len(zero),20)
        self.assertEqual(writes[-1][1].motor_cmd[3].kp,0)
        for row in rows:
            packet=codec.encode_state(row,bytes(range(32)),row['state_sequence'])
            self.assertLessEqual(len(packet),1200)

    def test_cancelled_handoff_still_damps_before_mode_worker_closes(self):
        for scenario in ('withdraw_pending','eof_pending'):
            with self.subTest(scenario=scenario):
                writes,rows,events,_=self.run_scenario(scenario=scenario)
                self.assertGreater(len(writes),150)
                self.assertTrue(all(cmd.motor_cmd[3].kp==0 for _,cmd in writes))
                self.assertGreater(writes[-1][0]-writes[0][0],.8)
                self.assertGreater(next(t for e,t in events if e=='mode_close'),writes[-1][0])

    def test_handoff_tracking_offset_never_escapes_command_bounds(self):
        writes,rows,events,_=self.run_scenario(scenario='handoff_tracking_bias')
        self.assertGreater(len(writes),200)
        lower=np.array(rows[0]['leg_lower_rad']);upper=np.array(rows[0]['leg_upper_rad'])
        self.assertLess(min(r['motor_q29'][10] for r in rows),lower[10]-.02)
        self.assertAlmostEqual(writes[0][1].motor_cmd[10].q,lower[10],places=7)
        self.assertEqual(writes[0][1].motor_cmd[10].kp,0)
        for _,cmd in writes:
            q=np.array([m.q for m in cmd.motor_cmd[:12]])
            self.assertTrue(np.all(q>=lower-1e-8));self.assertTrue(np.all(q<=upper+1e-8))
        self.assertEqual(writes[-1][1].motor_cmd[10].kp,0)

    def test_runtime_exception_and_write_failure_reach_emergency_damping(self):
        for scenario in ('workspace_exception','write_failure'):
            with self.subTest(scenario=scenario):
                writes,rows,events,output=self.run_scenario(scenario=scenario)
                self.assertGreater(len(writes),100)
                self.assertTrue(all(m.kp==0 and m.tau==0 for m in writes[-1][1].motor_cmd[:29]))
                self.assertIn('controller exception',output)

    def test_actual_foreign_lowcmd_stops_publishing_immediately(self):
        writes,rows,events,_=self.run_scenario(scenario='foreign')
        foreign=next(t for e,t in events if e=='foreign')
        self.assertTrue(all(t<=foreign for t,_ in writes))

if __name__=='__main__':unittest.main()
