"""PICO trigger control of Inspire DFX hands, independent of the arm servo.

Index triggers command closure; side grips remain the arm clutch. Serial ports
use configured physical LEFT/RIGHT aliases. No arm publisher.
"""
import argparse
from collections import deque
from contextlib import ExitStack
import ipaddress
import json
import math
import os
from pathlib import Path
import select
import socket
import subprocess
import sys
import threading
import time

from g1_arm_datagram import encode_command, decode_command, MAX_DATAGRAM
from g1_arm_transport import datagram_socket

from g1_config import REMOTE_PYTHON, REMOTE_TOOLS, ssh
REMOTE_SCRIPT=REMOTE_TOOLS+'/g1_inspire_teleop.py'


def trigger_target(triggers, initial):
    if len(triggers)!=2 or any(type(x) not in (int,float) or not math.isfinite(x) or not 0<=x<=1 for x in triggers):
        raise ValueError('Expected two triggers in [0,1]')
    if len(initial)!=12 or any(not math.isfinite(x) or not 0<=x<=1 for x in initial):
        raise ValueError('Expected twelve initial hand angles')
    result=list(initial)
    for side,value in enumerate(triggers):
        # Four fingers + thumb bend; preserve measured thumb opposition.
        result[side*6:side*6+5]=[1.-.8*value]*5
    return result


def finger_step(command,target,dt,speed=1.2):
    maximum=speed*min(.06,max(0.,dt))
    return [a+max(-maximum,min(maximum,b-a)) for a,b in zip(command,target)]


def validate_input(row,challenges,now):
    triggers=row.get('triggers')
    trigger_target(triggers,[1.]*12)
    age=row.get('input_age_s')
    if type(age) not in (int,float) or not math.isfinite(age) or age<0:
        raise ValueError('Invalid hand input age')
    issued=next((issued for token,issued in reversed(challenges) if token==row.get('challenge')),None)
    if issued is None or not 0<=now-issued<=.2 or age+now-issued>.15:
        raise ValueError('Expired hand command')
    return triggers,age+now-issued


def hand_server(args):
    from g1_inspire_serial import InspireSerial,LEFT_PORT,RIGHT_PORT
    connection=os.environ.get('SSH_CONNECTION','').split()
    if len(connection)!=4:raise RuntimeError('Hands require private SSH bootstrap')
    peer_ip=str(ipaddress.IPv4Address(connection[0]));host=str(ipaddress.IPv4Address(connection[2]))
    ending=threading.Event()
    def watch_parent():
        sys.stdin.read(1);ending.set()
    threading.Thread(target=watch_parent,daemon=True).start()
    with ExitStack() as resources:
        hands=[];initial=[]
        # Both ports must return valid measurements before any position write.
        for port in (LEFT_PORT,RIGHT_PORT):
            hand=InspireSerial(port);resources.callback(hand.close)
            initial.extend(hand.read_angles());hands.append(hand)
        sock=datagram_socket();resources.callback(sock.close);sock.bind((host,0))
        key=os.urandom(32)
        print(json.dumps(dict(type='hand_bootstrap',host=host,port=sock.getsockname()[1],key=key.hex())),flush=True)
        peer=None;last_seq=-1;state_seq=0;accepted=rejected=0
        challenges=deque(maxlen=12);command=list(initial);target=list(initial)
        armed=False;ever_input=False;last_input=0.;last_meta={};last_triggers=None;last_rejection=None
        start=last_tick=time.monotonic();reason='release BOTH index triggers to arm fingers'
        while not ending.is_set() and time.monotonic()-start<args.seconds:
            tick=time.monotonic();dt=tick-last_tick;last_tick=tick
            for _ in range(64):
                try:data,address=sock.recvfrom(MAX_DATAGRAM+1)
                except BlockingIOError:break
                if address[0]!=peer_ip or (peer is not None and address!=peer):continue
                try:
                    seq,row=decode_command(data,key,last_seq=last_seq)
                    last_seq=seq;peer=address
                    if row.get('type')=='link_ping':continue
                    now=time.monotonic();triggers,upper_age=validate_input(row,challenges,now)
                    accepted+=1;last_triggers=triggers
                    if not armed and max(triggers)<=.1:armed=True
                    if armed:
                        target=trigger_target(triggers,initial)
                        last_input=now-upper_age;ever_input=True
                        reason='following triggers'
                    last_meta=dict(command_id=seq,pc_sent_s=row.get('pc_sent_s'),
                                   input_age_upper_s=upper_age,triggers=triggers)
                except (ValueError,TypeError,KeyError,OverflowError) as exc:
                    rejected+=1;last_rejection=str(exc)[:120]
            fresh=armed and tick-last_input<=.2
            if not fresh:
                target=list(command)
                reason='input stale: holding last grasp' if ever_input else 'release BOTH index triggers to arm fingers'
            measured=[];measurement_times=[]
            for hand in hands:
                measured.extend(hand.read_angles());measurement_times.append(time.monotonic())
            fresh=armed and time.monotonic()-last_input<=.2 and not ending.is_set()
            if fresh:
                proposed=finger_step(command,target,dt)
                if args.live:
                    for index,hand in enumerate(hands):
                        if ending.is_set() or time.monotonic()-last_input>.2:
                            target=list(command);reason='input stale: holding last grasp';break
                        values=proposed[index*6:index*6+6]
                        if max(abs(a-b) for a,b in zip(values,command[index*6:index*6+6]))>=.0005:
                            hand.write_angles(values)
                            command[index*6:index*6+6]=values
                else:command=proposed
            state_seq+=1;now=time.monotonic();token=os.urandom(12).hex();challenges.append((token,now))
            row=dict(type='hand_state',seq=state_seq,challenge=token,
                     server_monotonic_s=now,measurement_monotonic_s=measurement_times,
                     measured_q=measured,command_q=command,target_q=target,
                     initial_q=initial,live=args.live,armed=armed,status=reason,
                     accepted=accepted,rejected=rejected,last_rejection=last_rejection,physical_order=['left','right'],
                     serial_ports=[LEFT_PORT,RIGHT_PORT],fault=False,**last_meta)
            # Device resolution is 0.001; retain that precision on the wire
            # rather than fragmenting long decimal interpolation values.
            for field in ('measured_q','command_q','target_q','initial_q'):
                row[field]=[round(x,3) for x in row[field]]
            if peer is not None:
                try:sock.sendto(encode_command(row,key,state_seq),peer)
                except BlockingIOError:pass
            # Source expiry freezes finger writes, not feedback. Keep this
            # worker available for fresh trigger input until the parent closes
            # it, serial I/O fails, or the bounded session duration expires.
            ending.wait(max(0,.04-(time.monotonic()-tick)))
        # No opening on loss/exit: the hands retain their last grasp locally.


class InspireHandLink:
    def __init__(self,live=False):
        from pico_native_bridge import AsyncDiagnosticLog
        project=Path(__file__).resolve().parents[1]
        self.log_path=project/'work'/f'hand_telemetry_{time.time_ns()}.jsonl'
        self.log=AsyncDiagnosticLog(self.log_path)
        self.lock=threading.Lock();self.stop=threading.Event();self.ready=threading.Event()
        self.state=None;self.received=0.;self.target=None;self.socket=None
        self.errors=[];self.offset_min=None;self.last_state_seq=-1
        remote=[REMOTE_PYTHON,'-u',REMOTE_SCRIPT,'--server','--seconds','900']+(['--live'] if live else [])
        self.process=subprocess.Popen(ssh(remote),
                                      stdin=subprocess.PIPE,stdout=subprocess.PIPE,text=True,bufsize=1)
        threading.Thread(target=self._bootstrap,daemon=True).start()
        if not self.ready.wait(12) or self.socket is None:
            self.close();raise RuntimeError('No Inspire hand bootstrap; check serial permission/power')
        threading.Thread(target=self._receive,daemon=True).start()
        threading.Thread(target=self._write,daemon=True).start()
        deadline=time.monotonic()+5
        while self.state is None and time.monotonic()<deadline and self.process.poll() is None:time.sleep(.02)
        if self.state is None:
            self.close();raise RuntimeError('No authenticated Inspire feedback')
        print(f'Inspire telemetry: {self.log_path}',flush=True)

    def _bootstrap(self):
        try:
            for line in self.process.stdout:
                row=json.loads(line)
                if row.get('type')!='hand_bootstrap':continue
                key=bytes.fromhex(row['key']);host=str(ipaddress.IPv4Address(row['host']));port=row['port']
                if len(key)!=32 or type(port) is not int or not 1<=port<=65535:raise ValueError('Bad hand bootstrap')
                self.key=key;self.socket=datagram_socket();self.socket.connect((host,port));self.ready.set()
        except (OSError,ValueError,KeyError):self.errors.append('Invalid private hand bootstrap')
        finally:self.ready.set()

    def _receive(self):
        try:
            while not self.stop.is_set():
                if not select.select([self.socket],[],[],.05)[0]:continue
                latest=None
                for _ in range(64):
                    try:data=self.socket.recv(MAX_DATAGRAM+1)
                    except BlockingIOError:break
                    try:
                        seq,row=decode_command(data,self.key,last_seq=self.last_state_seq)
                        if row.get('type')!='hand_state' or len(row['measured_q'])!=12:raise ValueError('Bad hand state')
                        if any(not math.isfinite(x) or not 0<=x<=1 for x in row['measured_q']):raise ValueError('Bad measured fingers')
                        self.last_state_seq=seq;latest=row,time.monotonic()
                    except (ValueError,KeyError,TypeError):continue
                if latest is None:continue
                row,received=latest;offset=received-row['server_monotonic_s']
                self.offset_min=offset if self.offset_min is None else min(self.offset_min,offset)
                excess=max(0,offset-self.offset_min)
                row.update(pc_monotonic_s=received,feedback_network_excess_s=excess,
                           packet_bytes=len(data))
                if row.get('pc_sent_s') is not None:row['pc_send_to_ack_ms']=1000*(received-row['pc_sent_s'])
                with self.lock:self.state,self.received=row,received-excess
                self.log.write(json.dumps(row,allow_nan=False)+'\n')
        except OSError:pass

    def submit(self,triggers,sampled):
        trigger_target(triggers,[1.]*12)
        with self.lock:self.target=list(triggers),float(sampled)

    def snapshot(self):
        with self.lock:
            if self.state is None:return None
            return dict(self.state,link_age_s=time.monotonic()-self.received)

    def _write(self):
        seq=0
        while not self.stop.wait(.02):
            with self.lock:state=self.state;received=self.received;target=self.target
            seq+=1;row=dict(type='link_ping',seq=seq)
            if state and target and time.monotonic()-received<.15:
                triggers,sampled=target;age=time.monotonic()-sampled
                if 0<=age<.15:
                    row=dict(seq=seq,triggers=triggers,input_age_s=age,
                             challenge=state['challenge'],pc_sent_s=time.monotonic())
            try:self.socket.send(encode_command(row,self.key,seq))
            except (OSError,ValueError):return

    def close(self):
        self.stop.set()
        try:self.process.stdin.close()
        except (OSError,AttributeError):pass
        try:self.process.wait(timeout=3)
        except subprocess.TimeoutExpired:self.process.terminate()
        if self.socket is not None:self.socket.close()
        self.log.__exit__(None)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--server',action='store_true',help=argparse.SUPPRESS)
    parser.add_argument('--live',action='store_true')
    parser.add_argument('--seconds',type=float,default=180)
    parser.add_argument('--native-port',type=int,default=8013)
    args=parser.parse_args()
    if args.server:return hand_server(args)
    from pico_native_input import NativeReceiver
    latest=[None]
    def event(value,sampled=None):latest[0]=(value,sampled)
    receiver=None;link=None
    try:
        link=InspireHandLink(live=args.live)
        receiver=NativeReceiver(event,lambda:latest.__setitem__(0,None),args.native_port)
        print('HANDS ONLY: arms remain stationary. Release both grips once, and BOTH index triggers.',flush=True)
        print('Then squeeze each index trigger to curl its matching hand; release to open. Ctrl+C holds last grasp.',flush=True)
        start=time.monotonic();next_report=start
        while time.monotonic()-start<args.seconds:
            sample=latest[0]
            if sample:
                value,sampled=sample
                states=[value[s+'State'] for s in ('left','right')]
                if all('triggerValue' in x for x in states):link.submit([x['triggerValue'] for x in states],sampled)
            state=link.snapshot()
            if state and (state['link_age_s']>1 or link.process.poll() is not None):
                raise RuntimeError('Hand feedback lost; last grasp retained')
            if state and time.monotonic()>=next_report:
                print('HANDS',json.dumps({k:state.get(k) for k in ('armed','status','triggers','measured_q','command_q','link_age_s','accepted','rejected')}),flush=True)
                next_report=time.monotonic()+2
            time.sleep(.01)
    except KeyboardInterrupt:print('Hands stopped; last grasp retained.',flush=True)
    finally:
        if receiver is not None:receiver.close()
        if link is not None:link.close()


if __name__=='__main__':main()
