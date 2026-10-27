"""Suspended-only full-body owner. Shadow default; --live permits mode handoff.

Never walk or balance. High-level control is released only after fresh,
verified body input and a released-then-held clutch. Exit damps the suspended
body; it NEVER selects a normal/walking mode automatically.
"""
from g1_config import REMOTE_ROOT, DDS_INTERFACE
import argparse
from collections import deque
from contextlib import ExitStack
import gc,json,math,os,queue,signal,sys,threading,time
import numpy as np
from g1_arm_link import (ARM_IDS,operating_joint_limits,bounded_joint_step,
    validate_target,walking_client_connected,REMOTE_GRAVITY_URDF)
from g1_arm_transport import ServerDatagrams
from g1_latest_state import LowStateMonitor
from g1_motion_mode import MotionModeMonitor
from g1_command_monitor import CommandMonitor
from g1_gravity_comp import MOTOR_NAMES_29,CalibratedGravity

LEG_RANGE=np.tile([.6,.35,.35,.6,.25,.15],2)
KP=np.array([60,60,60,100,40,40]*2+[60,40,40]+[60,60,60,60,40,40,40]*2,dtype=float)
KD=np.array([1,1,1,2,1,1]*2+[1,1,1]+[1.5]*14,dtype=float)


class BodyGeometry:
    def __init__(self,urdf):
        import pinocchio as pin
        self.pin=pin;self.model=pin.buildModelFromUrdf(urdf);self.data=self.model.createData()
        self.indices=np.array([self.model.joints[self.model.getJointId(n)].idx_q for n in MOTOR_NAMES_29])
        self.leg_ids=[self.model.getJointId(f'{s}_{joint}_joint') for s in ('left','right') for joint in ('knee','ankle_pitch')]
    def limits(self,initial):
        lo=self.model.lowerPositionLimit[self.indices].copy();hi=self.model.upperPositionLimit[self.indices].copy()
        lo[:12]=np.maximum(lo[:12],initial[:12]-LEG_RANGE);hi[:12]=np.minimum(hi[:12],initial[:12]+LEG_RANGE)
        lo[12:15]=hi[12:15]=initial[12:15]
        lo[15:],hi[15:]=operating_joint_limits()
        return lo,hi
    def leg_clear(self,q29):
        q=self.pin.neutral(self.model);q[self.indices]=q29
        self.pin.forwardKinematics(self.model,self.data,q)
        points=[self.data.oMi[i].translation for i in self.leg_ids]
        # A 5-mm runtime margin below IK's individual medial planes.
        return bool(points[0][1]>=.055 and points[1][1]>=.055 and points[2][1]<=-.055 and points[3][1]<=-.055)


class LegObservationClock:
    """Equal source sequences cannot renew the original observation lifetime."""
    def __init__(self):self.sequence=0;self.observed=0.
    def accept(self,sequence,age,now):
        if sequence<self.sequence:raise ValueError('Regressed body sequence; restart B after bridge restart')
        observed=now-age
        if sequence==self.sequence:observed=min(observed,self.observed)
        age=now-observed
        if age>.15:raise ValueError('Original body observation expired')
        self.sequence,self.observed=sequence,observed
        return age


def validate_leg_input(row,challenges,now,lower,upper,clock=None):
    q=row.get('leg_q');age=row.get('body_input_age_s');sequence=row.get('body_sequence')
    if not isinstance(q,list) or len(q)!=12 or any(type(x) not in (int,float) or not math.isfinite(x) for x in q):raise ValueError('Invalid twelve leg targets')
    if np.any(np.array(q)<lower) or np.any(np.array(q)>upper):raise ValueError('Leg target outside initial envelope/URDF')
    if type(sequence) is not int or sequence<2 or row.get('body_motion_verified') is not True:raise ValueError('Leg observation/motion not verified')
    issued=next((issued for token,issued in reversed(challenges) if token==row.get('challenge')),None)
    if type(age) not in (int,float) or not math.isfinite(age) or age<0 or issued is None or not 0<=now-issued<=.2 or age+now-issued>.15:raise ValueError('Stale estimated body bundle')
    age=age+now-issued
    if clock is not None:age=clock.accept(sequence,age,now)
    return np.array(q),age,sequence


def publish_body(publisher,factory,crc,own_crcs,mode_machine,command,velocity,tau,gain):
    cmd=factory();cmd.mode_pr=0;cmd.mode_machine=mode_machine
    for i in range(29):
        m=cmd.motor_cmd[i];m.mode=1;m.q=float(command[i]);m.dq=float(velocity[i])
        m.tau=float(tau[i])*gain;m.kp=float(KP[i])*gain;m.kd=float(KD[i])
    cmd.crc=crc.Crc(cmd);own_crcs.append(cmd.crc)
    if publisher.Write(cmd) is False:raise RuntimeError('DDS lowcmd write failed')


def damping_gain(start_gain,elapsed):
    return max(0.,start_gain*(1.-max(0.,elapsed)))


class DiagnosticLog:
    def __init__(self,path):
        self.stream=open(path,'w');self.queue=queue.Queue(maxsize=128);self.dropped=0
        self.ending=threading.Event();self.thread=threading.Thread(target=self.run,daemon=True);self.thread.start()
    def write(self,row):
        try:self.queue.put_nowait(row)
        except queue.Full:self.dropped+=1
    def run(self):
        while not self.ending.is_set() or not self.queue.empty():
            try:row=self.queue.get(timeout=.05)
            except queue.Empty:continue
            self.stream.write(json.dumps(row,allow_nan=False)+'\n')
        self.stream.flush()
    def close(self):
        self.ending.set();self.thread.join(timeout=3)
        if not self.thread.is_alive():self.stream.close()


def server(args):
    from unitree_sdk2py.core.channel import ChannelFactoryInitialize,ChannelPublisher
    from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_
    from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
    from unitree_sdk2py.utils.crc import CRC
    from g1_arm_workspace import ArmWorkspace
    output=sys.stdout;sys.stdout=sys.stderr
    ending=threading.Event();lock=threading.Lock();latest=[None,0.];history=deque(maxlen=500)
    incoming=deque(maxlen=1);outgoing=deque(maxlen=1);own_crcs=deque(maxlen=1000)
    foreign_low=[0.];foreign_arm=[0.];ownership=[False,0.]
    def state_callback(msg):
        with lock:latest[:]=[msg,msg.capture_received_s];history.append(msg)
    def command_callback(kind,crc,received):
        if kind==1 and crc not in own_crcs:foreign_low[0]=received
        elif kind==2:foreign_arm[0]=received
    with ExitStack() as resources:
        ChannelFactoryInitialize(0,args.interface)
        capture=LowStateMonitor(args.interface,state_callback);resources.callback(capture.Close)
        mode=MotionModeMonitor(args.interface);resources.callback(mode.close)
        initial_mode=mode.snapshot(timeout=3)
        if initial_mode['status'] not in ('active','released'):raise RuntimeError('No fresh motion-mode confirmation')
        deadline=time.monotonic()+3
        while (latest[0] is None or len(history)<80) and time.monotonic()<deadline:time.sleep(.01)
        if latest[0] is None or time.monotonic()-latest[1]>.1:raise RuntimeError('No fresh measured body state')
        msg=latest[0];initial=np.array([m.q for m in msg.motor_state[:29]])
        if not np.isfinite(initial).all():raise RuntimeError('Invalid measured body pose')
        geometry=BodyGeometry(REMOTE_GRAVITY_URDF);lower,upper=geometry.limits(initial)
        if np.any(initial<lower) or np.any(initial>upper) or not geometry.leg_clear(initial):raise RuntimeError('Initial legs outside verified envelope/medial planes')
        workspace=ArmWorkspace(REMOTE_GRAVITY_URDF)
        if np.any(workspace.clearance(initial)<0):raise RuntimeError('Initial arm geometry intersects torso')
        if walking_client_connected() or time.monotonic()-foreign_arm[0]<.5:raise RuntimeError('Competing control app/arm publisher')
        samples=list(history);qs=np.array([[m.motor_state[i].q for i in range(29)] for m in samples])
        taus=np.array([[m.motor_state[i].tau_est for i in ARM_IDS] for m in samples])
        if np.max(np.ptp(qs[:,15:],axis=0))>.02:raise RuntimeError('Hold arms still for startup torque calibration')
        gravity=CalibratedGravity(REMOTE_GRAVITY_URDF,np.median(qs,axis=0),np.median(taus,axis=0))
        if np.any(gravity.calibration_clipped):raise RuntimeError('Holding torque exceeds arm calibration limit')
        # Model/mesh loading can hold the parent GIL. Begin event observation
        # afterwards so startup work cannot create an artificial IPC backlog.
        commands=CommandMonitor(args.interface,command_callback);resources.callback(commands.Close)
        deadline=time.monotonic()+2
        while not commands.healthy() and time.monotonic()<deadline:time.sleep(.01)
        if not commands.healthy():raise RuntimeError('Command ownership observer unavailable')
        transport=ServerDatagrams(output);resources.callback(transport.close)
        log_path=f'{REMOTE_ROOT}/body_telemetry_{time.time_ns()}.jsonl'
        log=DiagnosticLog(log_path);resources.callback(log.close)
        print('Suspended body diagnostics:',log_path,flush=True)
        def watch_parent():sys.stdin.read(1);ending.set()
        def writer():
            while not ending.wait(.005):
                if outgoing:
                    try:transport.send(outgoing.pop())
                    except Exception as exc:print('Body output failed:',str(exc),flush=True);ending.set();return
        def poll_ownership():
            while not ending.wait(.1):ownership[:]=[walking_client_connected(),time.monotonic()]
        ownership[:]=[walking_client_connected(),time.monotonic()]
        threading.Thread(target=watch_parent,daemon=True).start()
        threading.Thread(target=writer,daemon=True).start()
        threading.Thread(target=transport.read,args=(incoming,ending),daemon=True).start()
        threading.Thread(target=poll_ownership,daemon=True).start()
        signal.signal(signal.SIGINT,lambda *_:ending.set());signal.signal(signal.SIGTERM,lambda *_:ending.set())
        command=initial.copy();desired=initial.copy();velocity=np.zeros(29);tau=np.zeros(29)
        released=False;following=False;leg_follow=False;leg_valid=False;last_seq=accepted=rejected=0
        last_input=last_leg=0.;body_sequence=0;body_age=None;meta={};challenges=deque(maxlen=30)
        publisher=None;active=False;virtual=False;handoff=None;active_at=None;shutdown=None
        shutdown_start_gain=0.;gain=0.;damping_done=False;ownership_lost=False
        body_clock=LegObservationClock();blocked=[False,False];clearance=None;gravity_saturated=[False]*14
        fault=False;reasons=[];status='waiting for verified body and clutch';state_sequence=0;loop_max=0.
        crc=CRC();start=last_tick=next_state=time.monotonic();mode_machine=msg.mode_machine
        if args.live:
            # SDK Init may block. Prepare before examining any engage packet;
            # no command is written until the release/quiet confirmation.
            publisher=ChannelPublisher('rt/lowcmd',LowCmd_);publisher.Init();resources.callback(publisher.Close)
            start=last_tick=next_state=time.monotonic()
        gc_enabled=gc.isenabled();gc.disable()
        try:
            while True:
                now=time.monotonic();dt=min(.02,max(0,now-last_tick));loop_max=max(loop_max,now-last_tick);last_tick=now
                with lock:msg,received=latest
                measured=np.array([m.q for m in msg.motor_state[:29]]);age=max(0,now-received)+msg.capture_delay_s
                mode_state=mode.snapshot();mode_status=mode_state['status'];mode_ok=mode_status=='released' if active else mode_status in ('active','released')
                bad=[]
                if not np.isfinite(measured).all() or age>.5:bad.append('invalid/stale body feedback')
                if msg.mode_machine!=mode_machine:bad.append('motor mode_machine changed')
                if mode_status=='fault':bad.append('motion mode query unavailable')
                if not commands.healthy():bad.append('command ownership observer unavailable')
                ownership_lost=bool((active and mode_status=='active') or (active and now-foreign_low[0]<.05))
                if active and mode_status=='active':bad.append('another high-level controller became active')
                if now-foreign_arm[0]<.1:bad.append('external arm publisher')
                if ownership[0] or (shutdown is None and not ending.is_set() and now-ownership[1]>.5):bad.append('external walking client/ownership unknown')
                if active and now-foreign_low[0]<.05:bad.append('external lowcmd publisher after takeover')
                if bad:
                    fault=True
                    for reason in bad:
                        if reason not in reasons:reasons.append(reason)
                if incoming:
                    row,input_received=incoming.pop()
                    if row.get('type')!='link_ping':
                        try:
                            seq,arms,follow=validate_target(row,initial[15:].tolist(),challenges,now,last_seq,0)
                            if np.any(np.array(arms)<lower[15:]) or np.any(np.array(arms)>upper[15:]):raise ValueError('Arm limits')
                            challenge_age=min(now-issued for token,issued in challenges if token==row['challenge'])
                            last_seq=seq;accepted+=1;last_input=now-row['input_age_s']-challenge_age
                            meta={k:row[k] for k in ('pc_sampled_s','pc_ready_s','pc_sent_s','source_sequence','ik_ms') if k in row}
                            meta.update(command_id=seq,robot_input_received_s=input_received,robot_accepted_s=now,
                                        robot_challenge_age_s=challenge_age,robot_input_age_upper_s=row['input_age_s']+challenge_age)
                            if all(row.get(k) is not None for k in ('pc_sent_s','pc_sampled_s','pc_ready_s')):
                                meta.update(pc_input_to_send_ms=1000*(row['pc_sent_s']-row['pc_sampled_s']),pc_ready_to_send_ms=1000*(row['pc_sent_s']-row['pc_ready_s']))
                            try:
                                legs,body_age,body_sequence=validate_leg_input(row,challenges,now,lower[:12],upper[:12],body_clock)
                                last_leg=now-body_age;leg_valid=True
                            except ValueError:leg_valid=False
                            if not follow:released=True;desired=command.copy()
                            following=bool(follow and released);leg_follow=bool(following and leg_valid)
                            if following:desired[15:]=arms
                            if leg_follow:desired[:12]=legs
                            if following and leg_valid and handoff is None and not active and not fault:
                                if args.live:
                                    handoff=now;mode.request_release();status='releasing internal control for suspended ownership'
                                else:virtual=True
                        except (ValueError,TypeError,KeyError):rejected+=1
                if handoff is not None and not active:
                    if now-handoff>3:reasons.append('suspended handoff did not complete');fault=True
                    if mode_status=='released' and now-foreign_low[0]>.1 and not ownership_lost:
                        command=np.clip(measured,lower,upper);initial[12:15]=measured[12:15]
                        lower[12:15]=upper[12:15]=measured[12:15]
                        desired[12:15]=measured[12:15];velocity[:]=0
                        active=True;active_at=now;status='suspended controller owns all 29 body motors'
                        if fault or ending.is_set() or not following or not leg_valid or now-last_input>.15 or now-last_leg>.15:
                            fault=True;reasons.append('engagement withdrawn during handoff; damping')
                if now-last_input>.2:following=False;desired[15:]=command[15:]
                if now-last_leg>.2:leg_follow=False;desired[:12]=command[:12]
                if not leg_follow:desired[:12]=command[:12];velocity[:12]=0
                if age>.1 or not mode_ok:following=leg_follow=False;desired=command.copy()
                if (active or virtual) and (fault or ending.is_set() or now-start>=args.seconds or now-last_input>1):
                    if shutdown is None:shutdown=now;shutdown_start_gain=gain
                if not active and not virtual and (fault or ending.is_set() or now-start>=args.seconds):break
                if ownership_lost:break  # Relinquish immediately; never fight another full-body owner.
                gain=min(1.,max(0.,(now-active_at)/.5)) if active else (1. if virtual else 0.)
                if shutdown is not None:
                    gain=damping_gain(shutdown_start_gain,now-shutdown);desired=command.copy();velocity[:]=0;tau[:]=0;status='damping suspended body'
                elif active or virtual:
                    if gain<1:command=np.clip(measured,lower,upper);command[12:15]=initial[12:15];velocity[:]=0
                    elif following:
                        speed=np.r_[np.full(12,1.2),np.zeros(3),np.full(14,2.)]
                        acceleration=np.r_[np.full(12,6.),np.zeros(3),np.full(14,10.)]
                        proposal,pv=bounded_joint_step(command,velocity,desired,dt,lower,upper,speed,acceleration,.02,True)
                        lead=np.r_[np.full(12,.15),np.full(3,.1),np.full(14,.1)]
                        trust_lower=np.maximum(lower,measured-lead);trust_upper=np.minimum(upper,measured+lead)
                        if np.any(trust_lower>trust_upper):raise RuntimeError('Measured body outside feasible session/trust intersection')
                        protected=np.clip(proposal,trust_lower,trust_upper);pv[protected!=proposal]=0
                        if not geometry.leg_clear(protected):protected[:12]=command[:12];pv[:12]=0
                        arms,av,blocked,clearance=workspace.interpolate(command[15:],velocity[15:],protected[15:],pv[15:],measured)
                        protected[15:]=arms;pv[15:]=av;command,velocity=protected,pv
                    else:velocity[:]=0
                    tau[15:],gravity_info=gravity.evaluate(measured,return_info=True)
                    gravity_saturated=gravity_info['saturated'].tolist()
                    if gain>.99 and np.max(np.abs(command-measured))>.25:
                        fault=True;reasons.append('full-body tracking error exceeds 0.25 rad')
                if publisher is not None and active:
                    publish_body(publisher,unitree_hg_msg_dds__LowCmd_,crc,own_crcs,mode_machine,command,velocity,tau,gain)
                    if meta and 'robot_first_published_s' not in meta:meta['robot_first_published_s']=time.monotonic()
                if now>=next_state:
                    state_sequence+=1;token=os.urandom(12).hex();challenges.append((token,now))
                    if shutdown is None:status='following' if following else ('clutch: holding' if active or virtual else 'waiting for verified body and clutch')
                    row=dict(type='arm_state',challenge=token,state_sequence=state_sequence,q=measured[15:].tolist(),motor_q29=measured.tolist(),
                        initial_q=initial[15:].tolist(),q_command=command[15:].tolist(),q_target=desired[15:].tolist(),dq_command=velocity[15:].tolist(),
                        dq=[m.dq for m in msg.motor_state[15:29]],state_age_s=age,fsm=None,mode_machine=mode_machine,mode_query_ok=mode_ok,
                        mode_query_code=mode_state['code'],mode_last_success_age_s=mode_state['last_success_age_s'],joint_range_rad=0.,profile='responsive',
                        operating_lower_rad=lower[15:].tolist(),operating_upper_rad=upper[15:].tolist(),speed_limit_rad_s=2.,acceleration_limit_rad_s2=10.,
                        tracking_lead_limit_rad=.10,tracking_limited=(np.abs(command[15:]-measured[15:])>=.0999).tolist(),
                        leg_tracking_limited=(np.abs(command[:12]-measured[:12])>=.1499).tolist(),workspace_blocked=list(blocked),
                        workspace_proposed_clearance_m=clearance,velocity_reference=True,takeover_active=active and gain<1,
                        tau_measured=[m.tau_est for m in msg.motor_state[15:29]],gravity_comp='calibrated',tau_command=(tau[15:]*gain).tolist(),
                        gravity_calibration=None,gravity_saturated=gravity_saturated,live=args.live,weight=gain,status=status,accepted=accepted,rejected=rejected,
                        fault=fault,fault_reasons=reasons.copy(),link_pings=0,server_monotonic_s=now,loop_max_gap_s=loop_max,
                        robot_tick=int(msg.tick),state_capture_received_s=received,state_delivery_delay_s=msg.capture_delay_s,
                        state_source_timestamp_ns=msg.source_timestamp_ns,state_ipc_drops=msg.state_ipc_drops,walking_client_connected=ownership[0],
                        control_scope='suspended_body',leg_initial_q=initial[:12].tolist(),leg_lower_rad=lower[:12].tolist(),leg_upper_rad=upper[:12].tolist(),
                        leg_command_q=command[:12].tolist(),leg_target_q=desired[:12].tolist(),leg_command_dq=velocity[:12].tolist(),
                        body_input_age_s=None if not last_leg else now-last_leg,body_sequence=body_sequence)
                    row.update(meta)
                    if 'robot_first_published_s' in meta:row['robot_receive_to_publish_ms']=1000*(meta['robot_first_published_s']-meta['robot_input_received_s'])
                    outgoing.append(row)
                    log.write(dict(row,command_q29=command.tolist(),target_q29=desired.tolist(),command_dq29=velocity.tolist(),
                                   measured_dq29=[m.dq for m in msg.motor_state[:29]],measured_tau29=[m.tau_est for m in msg.motor_state[:29]],
                                   command_tau29=(tau*gain).tolist(),command_kp29=(KP*gain).tolist(),command_kd29=KD.tolist(),
                                   gain=gain,motion_mode=mode_state['name'],diagnostic_dropped=log.dropped))
                    next_state=now+1/60
                if shutdown is not None and now-shutdown>=1.2:damping_done=True;break
                time.sleep(max(0.,.004-(time.monotonic()-now)))
        except Exception as exc:
            fault=True;reasons.append('controller exception: '+str(exc));print(reasons[-1],flush=True)
        finally:
            ending.set()
            # An exception must not bypass shutdown. Position/torque are zero
            # in this emergency path; damping never overrides a competing owner.
            if publisher is not None and not ownership_lost and not damping_done:
                # Release is asynchronous. Await its result before deciding
                # whether a just-cancelled handoff needs damping.
                deadline=time.monotonic()+1.5
                while handoff is not None and not active and time.monotonic()<deadline:
                    pending=mode.snapshot()
                    if pending.get('release_result') is not None and pending['status']=='released':break
                    if pending.get('release_result') is not None and pending['status']=='active' and pending.get('receipt_s',0)>pending['release_result']['receipt_s']:break
                    time.sleep(.01)
                mode_now=mode.snapshot()
                can_damp=active or (handoff is not None and mode_now['status']=='released' and time.monotonic()-foreign_low[0]>.1)
                if can_damp:
                    try:
                        until=time.monotonic()+1.
                        while time.monotonic()<until:
                            if mode.snapshot()['status']=='active' or time.monotonic()-foreign_low[0]<.05:break
                            publish_body(publisher,unitree_hg_msg_dds__LowCmd_,crc,own_crcs,mode_machine,command,np.zeros(29),np.zeros(29),0.)
                            time.sleep(.004)
                        else:damping_done=True
                    except Exception as exc:print('Emergency damping delivery failed:',str(exc),flush=True)
            if gc_enabled:gc.enable()
            if handoff is not None:print('Suspended body ended; damping stream completed='+str(damping_done)+'. After B exits, run bash outputs/pico_native.sh restore to restore AI ownership, then use the handheld while supported before grounding.',flush=True)
            if fault:output.write(json.dumps(dict(type='arm_error',fault_reasons=reasons))+'\n');output.flush()


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--live',action='store_true');p.add_argument('--interface',default=DDS_INTERFACE);p.add_argument('--seconds',type=float,default=900)
    p.add_argument('--joint-range',type=float,default=0);p.add_argument('--profile',default='responsive');p.add_argument('--gravity-comp',default='calibrated')
    p.add_argument('--transport',default='udp');p.add_argument('--velocity-reference',default='on')
    args=p.parse_args()
    if args.profile!='responsive' or args.transport!='udp' or args.joint_range!=0:p.error('Suspended mode requires responsive/UDP and absolute arm limits')
    server(args)
