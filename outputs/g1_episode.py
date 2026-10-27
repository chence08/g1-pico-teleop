"""Recording-only episode orchestration; never starts control or publishes commands.

Camera and DDS workers share the Orin monotonic clock. Device capture clocks
remain explicit; receipt-aligned playback is not a claim of sensor synchronization.
"""
import argparse
import bisect
from datetime import datetime, timezone
import hashlib
import fcntl
import json
import os
from pathlib import Path
import platform
import shlex
import signal
import shutil
import subprocess
import sys
import time
import uuid
from g1_files import upload_files, download_tree

PROJECT = Path(__file__).resolve().parent.parent
from g1_config import REMOTE_TOOLS, REMOTE_ROOT as ROBOT_ROOT, REMOTE_PYTHON as ROBOT_PYTHON, DDS_INTERFACE, HOST, remote_command, ssh
REMOTE_ROOT = ROBOT_ROOT + '/episodes'
WORKERS = ('g1_episode.py', 'g1_episode_camera.py', 'g1_episode_state.py', 'g1_config.py', 'g1_files.py')
BODY_JOINT_NAMES = ([side+'_'+joint+'_joint' for side in ('left','right') for joint in
                    ('hip_pitch','hip_roll','hip_yaw','knee','ankle_pitch','ankle_roll')]
                   + ['waist_'+joint+'_joint' for joint in ('yaw','roll','pitch')]
                   + [side+'_'+joint+'_joint' for side in ('left','right') for joint in
                      ('shoulder_pitch','shoulder_roll','shoulder_yaw','elbow','wrist_roll','wrist_pitch','wrist_yaw')])


def save_json(path, value):
    path = Path(path)
    temporary = path.with_suffix(path.suffix + '.tmp')
    temporary.write_text(json.dumps(value, indent=2, allow_nan=False) + '\n')
    temporary.replace(path)


def read_json(path):
    return json.loads(Path(path).read_text())


def host_identity():
    return dict(hostname=platform.node(), boot_id=Path('/proc/sys/kernel/random/boot_id').read_text().strip(),
                monotonic_ns=time.monotonic_ns(), wall_ns=time.time_ns())


def ssh_command(host, args):
    return ssh(args,host=host)


def clock_bounds(t0, t1, t2, t3):
    if not t0 <= t3 or not t1 <= t2:
        raise ValueError('Clock regression')
    low, high = t2-t3, t1-t0
    if low > high:
        raise ValueError('Inconsistent clock exchange')
    return dict(offset_lower_ns=low, offset_upper_ns=high,
                offset_midpoint_ns=(low+high)//2, uncertainty_ns=(high-low)//2,
                network_round_trip_ns=high-low, t0=t0, t1=t1, t2=t2, t3=t3)


def clock_worker():
    identity = host_identity()
    for line in sys.stdin:
        t1 = time.monotonic_ns()
        request = json.loads(line)
        row = dict(id=request['id'], identity=identity, t1=t1, wall_ns=time.time_ns())
        row['t2'] = time.monotonic_ns()
        print(json.dumps(row), flush=True)


def sample_clocks(host, samples=20):
    import select
    proc = subprocess.Popen(ssh_command(host, ['python3', REMOTE_TOOLS+'/g1_episode.py', 'clock']),
                            stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                            text=True, bufsize=1)
    rows = []
    try:
        for index in range(samples):
            t0 = time.monotonic_ns()
            proc.stdin.write(json.dumps({'id': index})+'\n'); proc.stdin.flush()
            if not select.select([proc.stdout], [], [], 12)[0]:
                raise TimeoutError('SSH clock worker did not respond')
            line = proc.stdout.readline(); t3 = time.monotonic_ns()
            if not line:
                raise RuntimeError('SSH clock worker exited: '+proc.stderr.read())
            reply = json.loads(line)
            if reply['id'] != index:
                raise ValueError('Clock reply mismatch')
            rows.append(dict(clock_bounds(t0, reply['t1'], reply['t2'], t3),
                             remote_identity=reply['identity']))
    finally:
        proc.stdin.close()
        try: proc.wait(timeout=5)
        except subprocess.TimeoutExpired: proc.terminate(); proc.wait(timeout=5)
    return dict(samples=rows, best=min(rows, key=lambda row: row['network_round_trip_ns']),
                semantics='Orin monotonic minus PC monotonic; bounds assume nonnegative transport delay; not camera capture synchronization')


def remote_record(args):
    output = Path(args.output)
    if not output.is_absolute(): raise ValueError('Remote output must be absolute')
    output.parent.mkdir(parents=True,exist_ok=True)
    lease=(output.parent/'.recording.lock').open('a')
    fcntl.flock(lease,fcntl.LOCK_EX|fcntl.LOCK_NB)
    output.mkdir(parents=True, exist_ok=False)
    model=Path(REMOTE_TOOLS)/'g1_body29_hand14.urdf'
    if model.exists(): shutil.copyfile(model,output/'robot_model.urdf')
    start = host_identity()
    save_json(output/'remote_manifest.json', dict(status='starting', host=start, seconds=args.seconds,
              task=args.task, recording_only=True))
    children = []; logs = []; errors = []; interrupted = []
    def stop(signum, frame):
        interrupted.append(signum); (output/'STOP').touch()
    signal.signal(signal.SIGTERM, stop); signal.signal(signal.SIGINT, stop)
    try:
        for name, python, extra in [('state', ROBOT_PYTHON, ['--interface', args.interface]),
                                    ('camera', 'python3', ['--depth'] if args.depth else [])]:
            log = (output/(name+'_worker.log')).open('x'); logs.append(log)
            duration = args.seconds+100 if name == 'state' else args.seconds
            cmd = [python, str(Path(__file__).with_name('g1_episode_'+name+'.py')),
                   '--output', str(output), '--seconds', str(duration), *extra]
            children.append((name, subprocess.Popen(cmd, stdout=log, stderr=subprocess.STDOUT)))
        deadline = time.monotonic()+20
        while not all((output/(name+'_ready.json')).exists() for name, _ in children):
            if interrupted: raise RuntimeError('Recording interrupted during startup')
            if any(p.poll() is not None for _, p in children): raise RuntimeError('Worker exited before both streams were ready')
            if time.monotonic()>deadline: raise TimeoutError('Camera/state startup exceeded20seconds')
            time.sleep(.1)
        print(json.dumps(dict(type='episode_ready', output=str(output), recording_only=True)), flush=True)
        deadline = time.monotonic()+args.seconds+90
        while children[1][1].poll() is None:
            if children[0][1].poll() is not None: raise RuntimeError('State worker stopped before camera')
            if time.monotonic()>deadline: raise TimeoutError('Camera worker exceeded deadline')
            time.sleep(.1)
    except Exception as exc:
        errors.append(str(exc))
    finally:
        (output/'STOP').touch()
        for name, proc in children:
            try: proc.wait(timeout=90 if name=='camera' else 15)
            except subprocess.TimeoutExpired:
                proc.terminate()
                try: proc.wait(timeout=5)
                except subprocess.TimeoutExpired: proc.kill(); proc.wait()
                errors.append(name+' required forced shutdown; incomplete capture')
            if proc.returncode: errors.append(name+' exit '+str(proc.returncode))
        for log in logs: log.close()
        save_json(output/'remote_manifest.json', dict(status='incomplete' if errors or interrupted else 'closed',
                  host=start, ended=host_identity(), task=args.task, recording_only=True,
                  errors=errors, interrupted=interrupted, worker_exit_codes={name:p.returncode for name,p in children}))
        lease.close()
    print(json.dumps(dict(type='episode_closed', output=str(output), errors=errors)), flush=True)
    return 1 if errors or interrupted else 0


def read_rows(path):
    if not Path(path).exists(): return []
    rows = []
    with Path(path).open() as source:
        for line in source:
            if line.strip(): rows.append(json.loads(line))
    return rows


def iter_rows(path):
    if Path(path).exists():
        with Path(path).open() as source:
            for line in source:
                if line.strip(): yield json.loads(line)


def file_hash(path):
    digest=hashlib.sha256()
    with Path(path).open('rb') as source:
        for block in iter(lambda:source.read(1024*1024),b''): digest.update(block)
    return digest.hexdigest()


def attach_pc_logs(output, start_ns, end_ns, start_wall_ns):
    """Copy existing complete log records; no SDK reader and no UDP consumer."""
    attached = {}; destination = output/'pc_logs'; destination.mkdir(exist_ok=True)
    patterns = {'pico': ('pico_native/logs/*.jsonl', 'host_poll_monotonic_s'),
                'arms': ('arm_telemetry_*.jsonl', 'pc_monotonic_s'),
                'hands': ('hand_telemetry_*.jsonl', 'pc_monotonic_s')}
    for kind, (pattern, stamp_key) in patterns.items():
        count = invalid = 0; sources = []; ranges = []; drop_values = []
        with (destination/(kind+'.jsonl')).open('x') as dest:
            for path in sorted((PROJECT/'work').glob(pattern)):
                if path.stat().st_mtime_ns < start_wall_ns-5_000_000_000: continue
                kept = 0
                with path.open() as src:
                    for line in src:
                        if not line.endswith('\n'): continue  # a writer may still be appending
                        try:
                            row = json.loads(line); stamp = row.get(stamp_key)
                            if not isinstance(stamp, (int,float)) or not start_ns <= int(stamp*1e9) <= end_ns: continue
                        except (ValueError, TypeError): invalid += 1; continue
                        dest.write(json.dumps(dict(source_file=str(path), record=row), allow_nan=False)+'\n')
                        kept += 1; count += 1; ranges.append(stamp)
                        if 'diagnostic_dropped_snapshots' in row: drop_values.append(row['diagnostic_dropped_snapshots'])
                if kept: sources.append(dict(path=str(path), rows=kept))
        attached[kind] = dict(rows=count, invalid_rows=invalid, sources=sources,
                              coverage_s=max(ranges)-min(ranges) if ranges else 0,
                              known_drop_counter_increase=max(drop_values)-min(drop_values) if drop_values else None,
                              completeness='diagnostic subset; source writer may drop; no claim of complete action history')
    return attached


def at_or_before(times, t, max_age_ns):
    index = bisect.bisect_right(times, t)-1
    if index < 0 or t-times[index] > max_age_ns: return None
    return index


def build_index(output):
    """Conservative receipt-time associations; never borrow future observations."""
    frames = read_rows(output/'camera_frames.jsonl')
    state_samples = [(row['host_receipt_monotonic_ns'],row.get('delivery_excess_ns',0)) for row in iter_rows(output/'state.jsonl')]
    state_times = [t for t,_ in state_samples]
    if any(b<a for a,b in zip(state_times, state_times[1:])): raise ValueError('State receipt clock regressed')
    topics = {}
    for index,row in enumerate(iter_rows(output/'commands.jsonl')):
        topics.setdefault(row['topic'], []).append((row['host_receipt_monotonic_ns'], index))
    for values in topics.values(): values.sort()
    topic_times={name:[a for a,_ in values] for name,values in topics.items()}
    hands = read_rows(output/'pc_logs'/'hands.jsonl')
    hand_samples = [[], []]
    for index,wrapper in enumerate(hands):
        for side,t in enumerate(wrapper['record'].get('measurement_monotonic_s', [])[:2]):
            if isinstance(t,(int,float)): hand_samples[side].append((int(t*1e9),index))
    for values in hand_samples: values.sort()
    hand_times=[[a for a,_ in values] for values in hand_samples]
    missing = 0; ages=[]; hand_missing=[0,0]
    with (output/'frame_index.jsonl').open('w') as dest:
        for frame in frames:
            t=frame['host_receipt_monotonic_ns']; state_index=at_or_before(state_times,t,50_000_000)
            if state_index is not None and state_samples[state_index][1]>50_000_000: state_index=None
            row=dict(video_index=frame['video_index'],orin_receipt_monotonic_ns=t,state_row=state_index,
                     command_rows={},hand_rows=[],alignment='Orin host receipt; not exposure time',execution_confirmed=False)
            if state_index is None: missing += 1
            else:
                row['state_age_ms']=(t-state_times[state_index])/1e6; ages.append(row['state_age_ms'])
            for topic,values in topics.items():
                i=at_or_before(topic_times[topic],t,100_000_000)
                row['command_rows'][topic]=None if i is None else values[i][1]
            for side,values in enumerate(hand_samples):
                i=at_or_before(hand_times[side],t,120_000_000)
                row['hand_rows'].append(None if i is None else values[i][1])
                if i is None: hand_missing[side]+=1
            dest.write(json.dumps(row)+'\n')
    return dict(frames=len(frames),frames_missing_recent_state=missing,
                max_associated_state_age_ms=max(ages) if ages else None,
                frames_missing_hand_measurements=hand_missing,command_topics=list(topics),
                physical_capture_alignment_verified=False)


def inspect_episode(output):
    output=Path(output); index=build_index(output); issues=[]
    for name in ('camera','state'):
        path=output/(name+'_result.json')
        if not path.exists(): issues.append(name+' result missing'); continue
        result=read_json(path)
        if not result.get('complete',False): issues.append(name+' capture incomplete (see result)')
    if not index['frames']: issues.append('no camera frames')
    if index['frames_missing_recent_state']: issues.append('some frames have no preceding state within50ms')
    manifest=read_json(output/'manifest.json')
    if manifest.get('error') or manifest.get('remote_exit_code') != 0: issues.append('coordinator failed or was interrupted')
    if manifest.get('pc',{}).get('boot_id') != manifest.get('pc_end',{}).get('boot_id'): issues.append('PC rebooted during capture')
    pre=manifest.get('clock_pre',{}).get('best'); post=manifest.get('clock_post',{}).get('best')
    if pre and post:
        if pre['remote_identity']['boot_id']!=post['remote_identity']['boot_id']: issues.append('Orin rebooted during capture')
        clock_summary=dict(pre_uncertainty_ms=pre['uncertainty_ns']/1e6,post_uncertainty_ms=post['uncertainty_ns']/1e6,
                           midpoint_offset_change_ms=(post['offset_midpoint_ns']-pre['offset_midpoint_ns'])/1e6,
                           pc_alignment='bounded estimates at endpoints; drift/asymmetry not silently removed')
    else: issues.append('PC-Orin clock calibration missing'); clock_summary={}
    if manifest.get('kind')=='rehearsal':
        for name in ('pico','arms','hands'):
            if not manifest.get('pc_logs',{}).get(name,{}).get('rows'): issues.append(name+' teleoperation diagnostics missing')
        if any(index['frames_missing_hand_measurements']): issues.append('some video frames have no recent hand measurements')
    remote=read_json(output/'remote_manifest.json') if (output/'remote_manifest.json').exists() else {}
    if remote.get('status')!='closed': issues.append('remote recorder did not close normally')
    report=dict(capture_complete=not issues,issues=issues,index=index,clock=clock_summary,
                training_ready=False,review_required=['hands/box visible','task-specific motion accuracy',
                'video/state timing review','teleoperation latency under recording load','outcome label'],
                note='A closed recording is not automatically a training demonstration. DDS commands are publications, not proof of execution.')
    save_json(output/'quality.json',report)
    return report


def await_remote_closed(host,remote,timeout=110):
    deadline=time.monotonic()+timeout
    while time.monotonic()<deadline:
        status=subprocess.run(ssh_command(host,['cat',remote+'/remote_manifest.json']),capture_output=True,text=True,timeout=15)
        if status.returncode==0:
            result=json.loads(status.stdout)
            if result.get('status') in ('closed','incomplete') and len(result.get('worker_exit_codes',{}))==2:
                return result
        time.sleep(1)
    raise TimeoutError('Recorder is not finalized; preserve remote capture and retry fetch later')


def control_configuration(output):
    import xml.etree.ElementTree as ET
    first=next(iter_rows(output/'pc_logs'/'arms.jsonl'),{}).get('record',{})
    keys=('profile','control_scope','initial_q','joint_range_rad','operating_lower_rad','operating_upper_rad',
          'speed_limit_rad_s','acceleration_limit_rad_s2','tracking_lead_limit_rad','gravity_comp',
          'gravity_calibration','leg_initial_q','leg_lower_rad','leg_upper_rad','arm_transport')
    config={key:first.get(key) for key in keys}
    config.update(joint_position_unit='rad',joint_velocity_unit='rad/s',torque_unit='Nm',
                  motor_ids=list(range(29)),body_joint_names_in_motor_order=BODY_JOINT_NAMES,
                  hand_order=['left','right'],hand_actuator_order=['pinky','ring','middle','index','thumb_bend','thumb_rotation'],
                  hand_position_unit='normalized device angle, 1=open',
                  unknown_configuration=['exact B command line','controller mounting transform','operator calibration history'],
                  action_history='Latest observed DDS publications plus sampled B diagnostics; not every servo update')
    model=output/'robot_model.urdf'
    if model.exists():
        config['urdf_sha256']=file_hash(model)
        config['urdf_movable_joint_names']=[j.attrib['name'] for j in ET.parse(model).getroot().findall('joint') if j.attrib.get('type')!='fixed']
        config['urdf_order_note']='Use body_joint_names_in_motor_order; URDF traversal also includes hand joints.'
        if not set(BODY_JOINT_NAMES).issubset(config['urdf_movable_joint_names']): raise ValueError('Body motor names do not match recorded URDF')
    save_json(output/'control_configuration.json',config)


def fetch(output,host=None):
    manifest=read_json(output/'manifest.json'); remote=manifest['remote_path']
    host=host or manifest.get('host',HOST)
    try:
        await_remote_closed(host,remote)
        print('FETCHING '+remote,flush=True)
        download_tree(host,remote,output,timeout=600)
    except Exception as exc:
        manifest.update(stage='recovery_pending',fetch_error=str(exc));save_json(output/'manifest.json',manifest)
        raise
    if not (output/'pc_logs').exists():
        manifest['pc_logs']=attach_pc_logs(output,manifest['pc_start_ns'],manifest['pc_end_ns'],manifest['pc']['wall_ns'])
    manifest['stage']='closed';save_json(output/'manifest.json',manifest)
    control_configuration(output)
    report=inspect_episode(output)
    manifest['artifact_sha256']={str(p.relative_to(output)):file_hash(p) for p in output.rglob('*') if p.is_file() and p.name!='manifest.json'}
    save_json(output/'manifest.json',manifest)
    return report


def hashes(root):
    return {str(path.relative_to(root)):file_hash(path)
            for path in sorted((root/'outputs').glob('*.py'))}


def record(args):
    episode_id=datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')+'_'+uuid.uuid4().hex[:8]
    output=PROJECT/'work'/'episodes'/episode_id; output.mkdir(parents=True,exist_ok=False)
    remote=REMOTE_ROOT+'/'+episode_id
    manifest=dict(schema='g1-raw-episode-v1',episode_id=episode_id,task=args.task,kind=args.kind,outcome='unreviewed',
                  stage='starting',pc=host_identity(),host=args.host,remote_path=remote,seconds=args.seconds,depth=args.depth,
                  source_sha256=hashes(PROJECT),recording_only=True,
                  clocks={'camera_device':'original timestamp/domain preserved', 'robot_sensor':'DDSsource/tick preserved',
                          'orin_monotonic':'camera/DDS receipt and per-hand read completion',
                          'pc_monotonic':'PICO SDK polling and existing B diagnostics'},
                  action_semantics='DDS command publications and existing hand diagnostic commands; not guaranteed actuator acceptance')
    save_json(output/'manifest.json',manifest)
    (output/'software').mkdir()
    for path in (PROJECT/'outputs').glob('*.py'): shutil.copyfile(path,output/'software'/path.name)
    # Deploy recording-only files; motor controller files are not modified.
    subprocess.run(ssh_command(args.host,['mkdir','-p',REMOTE_TOOLS,REMOTE_ROOT]),check=True,timeout=20)
    upload_files(args.host,[PROJECT/'outputs'/p for p in WORKERS],REMOTE_TOOLS,timeout=30)
    start_wall=time.time_ns(); start_pc=time.monotonic_ns(); result=None
    try:
        manifest['clock_pre']=sample_clocks(args.host)
        save_json(output/'manifest.json',manifest)
        command=['python3',REMOTE_TOOLS+'/g1_episode.py','remote','--output',remote,
                 '--seconds',str(args.seconds),'--task',args.task,'--interface',args.interface]
        if args.depth: command.append('--depth')
        print('EPISODE '+str(output),flush=True)
        result=subprocess.run(ssh_command(args.host,command),timeout=args.seconds+140)
        manifest['clock_post']=sample_clocks(args.host)
    except (Exception,KeyboardInterrupt) as exc:
        manifest['error']=str(exc) or type(exc).__name__
        try: subprocess.run(ssh_command(args.host,['touch',remote+'/STOP']),timeout=15)
        except Exception: pass
    finally:
        end_pc=time.monotonic_ns()
        manifest.update(pc_start_ns=start_pc,pc_end_ns=end_pc,remote_exit_code=result.returncode if result else None,
                        stage='collecting',pc_end=host_identity())
        save_json(output/'manifest.json',manifest)
    # Allow diagnostic writers' two-second flush interval before attaching complete lines.
    time.sleep(2.2)
    if args.kind=='rehearsal':
        # Bulk video/depth transfer shares Wi-Fi with control; defer until B exits.
        manifest['pc_logs']=attach_pc_logs(output,start_pc,end_pc,start_wall)
        manifest['stage']='recorded_remote_pending_download';save_json(output/'manifest.json',manifest)
        print(json.dumps(dict(output=str(output),remote_path=remote,download_deferred=True,
                              instruction='After B exits: python3 outputs/g1_episode.py fetch '+str(output))),flush=True)
        return 0 if result is not None and result.returncode==0 else 1
    report=fetch(output,args.host)
    print(json.dumps(dict(output=str(output),**report),indent=2),flush=True)
    return 0 if report['capture_complete'] else 1


def main():
    parser=argparse.ArgumentParser(description=__doc__); commands=parser.add_subparsers(dest='command',required=True)
    for mode in ('record','remote'):
        p=commands.add_parser(mode);p.add_argument('--seconds',type=float,default=30)
        p.add_argument('--task',default='Receive an empty cardboard box from a human')
        p.add_argument('--depth',action='store_true');p.add_argument('--interface',default=DDS_INTERFACE)
        if mode=='remote': p.add_argument('--output',required=True)
        else:
            p.add_argument('--host',default=HOST);p.add_argument('--kind',choices=('commissioning','rehearsal'),default='commissioning')
    commands.add_parser('clock')
    p=commands.add_parser('inspect');p.add_argument('output',type=Path)
    p=commands.add_parser('fetch');p.add_argument('output',type=Path);p.add_argument('--host',help='Override host saved in episode manifest')
    p=commands.add_parser('label');p.add_argument('output',type=Path)
    p.add_argument('--outcome',choices=('success','failure','aborted','commissioning'),required=True);p.add_argument('--notes',default='')
    args=parser.parse_args()
    if hasattr(args,'seconds') and not 1<=args.seconds<=300: parser.error('seconds must be1..300')
    if args.command=='clock': clock_worker(); return 0
    if args.command=='remote': return remote_record(args)
    if args.command=='record': return record(args)
    if args.command in ('inspect','fetch'):
        report=inspect_episode(args.output) if args.command=='inspect' else fetch(args.output,args.host)
        print(json.dumps(report,indent=2));return 0 if report['capture_complete'] else 1
    manifest=read_json(args.output/'manifest.json');manifest.update(outcome=args.outcome,notes=args.notes,labelled_utc=datetime.now(timezone.utc).isoformat())
    save_json(args.output/'manifest.json',manifest); print('Label saved; training readiness still requires quality review.');return 0


if __name__=='__main__': sys.exit(main())
