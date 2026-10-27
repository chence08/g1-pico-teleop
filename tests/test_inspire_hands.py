import io
import json
import os
from pathlib import Path
import struct
import sys
import threading
from types import SimpleNamespace
import unittest
from unittest.mock import patch
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'outputs'))
from g1_inspire_serial import frame,parse_frame,InspireSerial
from g1_inspire_teleop import trigger_target,finger_step,validate_input,hand_server
from g1_arm_datagram import encode_command,decode_command


class InspireTests(unittest.TestCase):
    def test_protocol_measured_reply_and_corruption(self):
        reply=bytes.fromhex('90eb010f110a06e803e803e803e803e703de03a8')
        self.assertEqual(struct.unpack('<6H',parse_frame(reply,0x11,0x060a,12)),(1000,1000,1000,1000,999,990))
        for i in range(len(reply)):
            bad=bytearray(reply);bad[i]^=1
            with self.assertRaises(ValueError):parse_frame(bytes(bad),0x11,0x060a,12)
        self.assertEqual(frame(0x11,0x060a,b'\x0c').hex(),'eb900104110a060c32')

    def test_negative_write_ack_is_not_success(self):
        hand=InspireSerial.__new__(InspireSerial)
        hand.transaction=lambda *args:b'\x00'
        with self.assertRaises(RuntimeError):hand.write_angles([.5]*6)

    def test_independent_triggers_preserve_thumb_rotation_and_rate(self):
        initial=[.8]*6+[.7]*6;initial[5]=.43;initial[11]=.64
        target=trigger_target([1.,0.],initial)
        self.assertEqual(target[5],.43);self.assertEqual(target[11],.64)
        self.assertAlmostEqual(target[0],.2);self.assertEqual(target[6],1.)
        result=finger_step(initial,target,.01)
        self.assertLess(result[0],initial[0]);self.assertGreater(result[6],initial[6])
        self.assertLessEqual(max(abs(a-b) for a,b in zip(initial,result)),.01200001)
        self.assertEqual(result[5],initial[5]);self.assertEqual(result[11],initial[11])

    def test_delayed_command_and_invalid_triggers_are_rejected(self):
        row=dict(triggers=[0.,1.],challenge='test',input_age_s=.01)
        validate_input(row,[('test',10.)],10.02)
        with self.assertRaises(ValueError):validate_input(row,[('test',10.)],10.16)
        for t in ([float('nan'),0.],[0.,1.1],[0.],None):
            with self.assertRaises((ValueError,TypeError)):validate_input(dict(row,triggers=t),[('test',10.)],10.02)

    def test_stale_source_keeps_feedback_and_recovers_without_stale_writes(self):
        now=[100.];key=[None];pending=[];states=[];writes=[]
        source_pause=[None];recovery=[None];injected=[False];sequence=[0]
        gate=threading.Event()
        def queue_row(row):
            sequence[0]+=1;row['seq']=sequence[0]
            pending.append(encode_command(row,key[0],sequence[0]))
        class Hand:
            def __init__(self,port):self.port=port
            def read_angles(self):now[0]+=.03;return [.7]*6
            def write_angles(self,q):writes.append((now[0],self.port,list(q)))
            def close(self):pass
        class Sock:
            def bind(self,_):pass
            def getsockname(self):return ('127.0.0.1',12345)
            def close(self):pass
            def recvfrom(self,_):
                if not pending:raise BlockingIOError()
                return pending.pop(0),('127.0.0.1',2222)
            def sendto(self,data,_peer):
                _,state=decode_command(data,key[0]);states.append(state)
                command=dict(challenge=state['challenge'],triggers=[0.,0.],input_age_s=0.)
                if len(states)==1:queue_row(command)
                elif len(states)==2:queue_row(dict(command,triggers=[1.,1.]))
                elif source_pause[0] is None:
                    source_pause[0]=now[0];queue_row(dict(type='link_ping'))
                elif now[0]-source_pause[0]>=1.35:
                    if recovery[0] is None:recovery[0]=now[0]
                    queue_row(command)
                elif now[0]-source_pause[0]>=.35 and not injected[0]:
                    injected[0]=True;queue_row(dict(command,input_age_s=.2))
                else:queue_row(dict(type='link_ping'))
        class Output(io.StringIO):
            def write(self,text):
                if text.startswith('{'):
                    key[0]=bytes.fromhex(json.loads(text)['key'])
                    queue_row(dict(type='link_ping'))
                return super().write(text)
        class Input:
            def read(self,_):gate.wait(1);return ''
        try:
            with patch('g1_inspire_serial.InspireSerial',Hand),patch('g1_inspire_teleop.datagram_socket',return_value=Sock()), \
                 patch('g1_inspire_teleop.time.monotonic',side_effect=lambda:now[0]), \
                 patch.dict(os.environ,{'SSH_CONNECTION':'127.0.0.1 22 127.0.0.1 22'}), \
                 patch.object(sys,'stdin',Input()),patch.object(sys,'stdout',Output()):
                hand_server(SimpleNamespace(live=True,seconds=2.2))
        finally:gate.set()
        self.assertIsNotNone(recovery[0],'Healthy worker must survive a >1s source pause')
        stale=[r for r in states if source_pause[0]+.25<r['server_monotonic_s']<recovery[0]]
        self.assertGreater(len(stale),10)
        self.assertTrue(all(r['status']=='input stale: holding last grasp' for r in stale))
        self.assertEqual(len({tuple(r['command_q']) for r in stale}),1)
        self.assertFalse(any(source_pause[0]+.25<t<recovery[0] for t,_,_ in writes))
        self.assertTrue(any(t>recovery[0] for t,_,_ in writes),'Fresh input must recover control')
        self.assertTrue(any(r.get('last_rejection')=='Expired hand command' for r in states))

    def test_serial_delays_cannot_execute_expired_input(self):
        now=[100.];writes=[];key=[None];pending=[];gate=threading.Event()
        class Hand:
            def __init__(self,port):self.port=port
            def read_angles(self):now[0]+=.11;return [.7]*6
            def write_angles(self,q):writes.append((self.port,q))
            def close(self):pass
        class Sock:
            def bind(self,_):pass
            def getsockname(self):return ('127.0.0.1',12345)
            def close(self):pass
            def recvfrom(self,_):
                if not pending:raise BlockingIOError()
                return pending.pop(0),('127.0.0.1',2222)
            def sendto(self,data,_peer):
                _,state=decode_command(data,key[0])
                pending.append(encode_command(dict(seq=state['seq']+1,
                    challenge=state['challenge'],triggers=[0.,0.],input_age_s=0.),key[0],state['seq']+1))
        class Output(io.StringIO):
            def write(self,text):
                if text.startswith('{'):
                    row=json.loads(text);key[0]=bytes.fromhex(row['key'])
                    pending.append(encode_command(dict(type='link_ping',seq=1),key[0],1))
                return super().write(text)
        class Input:
            def read(self,_):gate.wait(1);return ''
        try:
            with patch('g1_inspire_serial.InspireSerial',Hand),patch('g1_inspire_teleop.datagram_socket',return_value=Sock()),\
                 patch('g1_inspire_teleop.time.monotonic',side_effect=lambda:now[0]),\
                 patch.dict(os.environ,{'SSH_CONNECTION':'127.0.0.1 22 127.0.0.1 22'}),\
                 patch.object(sys,'stdin',Input()),patch.object(sys,'stdout',Output()):
                hand_server(SimpleNamespace(live=True,seconds=.5))
        finally:gate.set()
        self.assertEqual(writes,[],'Expired trigger must never reach a finger after blocking serial reads')


if __name__=='__main__':unittest.main()
