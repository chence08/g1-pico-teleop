"""Read-only Orin DDS recorder. Observed publications do not prove execution."""
from g1_config import DDS_INTERFACE
import argparse
from collections import deque
import json
import math
import os
from pathlib import Path
import queue
import signal
import threading
import time

TOPICS=('rt/lowstate','rt/lowcmd','rt/arm_sdk')
OWNED=('state.jsonl','commands.jsonl','state_ready.json','state_result.json')

def optional(value):
    if isinstance(value,(list,tuple)):return [optional(x) for x in value]
    if value is None or isinstance(value,(bool,int,str)):return value
    try:
        value=float(value)
        return value if math.isfinite(value) else None
    except (ValueError,TypeError,OverflowError):return None

def serialize_sample(topic,msg,source_ns,mono_ns,wall_ns,sequence,publication_handle=0):
    if topic not in TOPICS or type(source_ns) is not int or source_ns<=0:raise ValueError('Invalid topic or source timestamp')
    invalid=[]
    def number(value,label):
        try:
            value=float(value)
            if math.isfinite(value):return value
        except (ValueError,TypeError,OverflowError):pass
        invalid.append(label);return None
    row=dict(schema='g1_episode_dds_v1',topic=topic,sequence=int(sequence),dds_source_timestamp_ns=source_ns,
             dds_publication_handle=int(publication_handle),host_receipt_monotonic_ns=int(mono_ns),
             host_receipt_wall_ns=int(wall_ns),clock_domain='g1_orin',receipt_kind='post_take_deserialization',
             mode_pr=optional(getattr(msg,'mode_pr',None)),mode_machine=optional(getattr(msg,'mode_machine',None)))
    if topic=='rt/lowstate':
        motors=list(msg.motor_state)
        if len(motors)<29:raise ValueError('Fewer than29measured motors')
        motors=motors[:29];row.update(kind='measured_joint_state',tick=int(msg.tick),motor_ids=list(range(29)))
        for name in ('q','dq','tau_est'):
            row[name]=[number(getattr(m,name),f'{name}[{i}]') for i,m in enumerate(motors)]
        for field,attr in [('motor_mode','mode'),('motor_status','motorstate'),('motor_temperature','temperature'),('motor_voltage','vol')]:
            if any(hasattr(m,attr) for m in motors):row[field]=[optional(getattr(m,attr,None)) for m in motors]
    else:
        motors=list(msg.motor_cmd)
        if len(motors)<29:raise ValueError('Fewer than29commanded motors')
        columns={'mode':[optional(getattr(m,'mode',None)) for m in motors]}
        for name in ('q','dq','tau','kp','kd'):
            columns[name]=[number(getattr(m,name),f'{name}[{i}]') for i,m in enumerate(motors)]
        row.update(kind='observed_command_publication',motor_ids=list(range(len(motors))),motor_cmd=columns,
                   execution_confirmed=False,command_topic_class=topic.split('/')[-1],
                   arm_weight_slot29=columns['q'][29] if len(motors)>29 else None,
                   arm_weight_semantics='arm weight only on rt/arm_sdk; otherwise raw slot29.q',crc=optional(getattr(msg,'crc',None)))
    row['invalid_numeric_fields']=invalid
    return row

def percentile(values,p):
    if not values:return None
    a=sorted(values);x=(len(a)-1)*p;i=int(x)
    return a[i]+(a[min(i+1,len(a)-1)]-a[i])*(x-i)

class TopicStats:
    def __init__(self,topic):
        self.topic=topic;self.samples=self.written=self.duplicates=self.regressions=0
        self.invalid=self.queue_drops=self.nonfinite_samples=0
        self.sample_lost=self.sample_rejected=self.lifecycle_events=0;self.first_receipt=self.last_receipt=None
        self.max_gap_ns=self.max_excess_ns=self.stale_gaps=0;self.gaps=deque(maxlen=8192);self.writers={}
    def observe(self,source_ns,received_ns,writer):
        if source_ns<=0:self.invalid+=1;return None
        if writer not in self.writers:
            if len(self.writers)>=64:self.invalid+=1;return None
            self.writers[writer]=[None,None,0]
        previous,minimum,epoch=self.writers[writer]
        if source_ns==previous:self.duplicates+=1;return None
        regression=previous is not None and source_ns<previous
        if regression:self.regressions+=1;epoch+=1;minimum=None
        offset=received_ns-source_ns;minimum=offset if minimum is None else min(minimum,offset);excess=max(0,offset-minimum)
        self.writers[writer]=[source_ns,minimum,epoch]
        if self.last_receipt is None:self.first_receipt=received_ns
        else:
            gap=max(0,received_ns-self.last_receipt);self.gaps.append(gap)
            self.max_gap_ns=max(self.max_gap_ns,gap);self.stale_gaps+=gap>100_000_000
        self.last_receipt=received_ns;self.max_excess_ns=max(self.max_excess_ns,excess);self.samples+=1
        return dict(source_clock_epoch=epoch,source_clock_regressed=regression,delivery_excess_ns=excess)
    def summary(self,end_ns):
        span=0 if self.first_receipt is None else (self.last_receipt-self.first_receipt)/1e9
        return dict(samples=self.samples,written=self.written,observed_hz=(self.samples-1)/span if span>0 else 0.,
                    duplicates=self.duplicates,source_clock_regressions=self.regressions,invalid_samples=self.invalid,
                    nonfinite_samples=self.nonfinite_samples,writer_queue_drops=self.queue_drops,
                    dds_reported_sample_lost=self.sample_lost,dds_reported_sample_rejected=self.sample_rejected,
                    dds_lifecycle_events=self.lifecycle_events,
                    last_receipt_age_s=None if self.last_receipt is None else max(0,end_ns-self.last_receipt)/1e9,
                    receipt_gap_max_s=self.max_gap_ns/1e9,
                    receipt_gap_p50_s=None if not self.gaps else percentile(self.gaps,.5)/1e9,
                    receipt_gap_p95_s=None if not self.gaps else percentile(self.gaps,.95)/1e9,
                    percentile_scope='last8192advancing-sample intervals',receipt_gaps_over100ms=self.stale_gaps,
                    delivery_excess_max_s=self.max_excess_ns/1e9,writer_count=len(self.writers))

def atomic_json(path,value):
    temporary=path.with_name(path.name+'.tmp')
    with temporary.open('x') as file:
        json.dump(value,file,allow_nan=False,separators=(',',':'));file.write('\n');file.flush();os.fsync(file.fileno())
    os.replace(temporary,path)

class BoundedWriter:
    def __init__(self,output,stats,capacity=2048):
        self.output,self.stats=output,stats;self.queue=queue.Queue(maxsize=capacity)
        self.ending=threading.Event();self.error=None;self.ready=False
        self.state_file=(output/'state.jsonl').open('x',buffering=65536)
        try:self.command_file=(output/'commands.jsonl').open('x',buffering=65536)
        except BaseException:self.state_file.close();raise
        self.thread=threading.Thread(target=self.run,daemon=True);self.thread.start()
    def submit(self,row):
        try:self.queue.put_nowait(row);return True
        except queue.Full:self.stats[row['topic']].queue_drops+=1;return False
    def run(self):
        try:
            flushed=time.monotonic()
            while not self.ending.is_set() or not self.queue.empty():
                try:row=self.queue.get(timeout=.05)
                except queue.Empty:row=None
                if row is not None:
                    file=self.state_file if row['topic']=='rt/lowstate' else self.command_file
                    file.write(json.dumps(row,allow_nan=False,separators=(',',':'))+'\n');self.stats[row['topic']].written+=1
                    if row['topic']=='rt/lowstate' and not self.ready and not row['invalid_numeric_fields']:
                        file.flush();atomic_json(self.output/'state_ready.json',dict(read_only=True,pid=os.getpid(),
                            host_receipt_monotonic_ns=row['host_receipt_monotonic_ns'],host_receipt_wall_ns=row['host_receipt_wall_ns'],
                            dds_source_timestamp_ns=row['dds_source_timestamp_ns']));self.ready=True
                if time.monotonic()-flushed>=.25:
                    self.state_file.flush();self.command_file.flush();flushed=time.monotonic()
            for file in (self.state_file,self.command_file):file.flush();os.fsync(file.fileno())
        except BaseException as exc:self.error=f'{type(exc).__name__}: {exc}'
    def finish(self):
        self.ending.set();self.thread.join(timeout=5)
        if self.thread.is_alive():self.error=self.error or 'Writer did not drain within5seconds'
        else:self.state_file.close();self.command_file.close()

def result_summary(stats,start_ns,end_ns,stop_reason,error,writer):
    topics={name:s.summary(end_ns) for name,s in stats.items()};issues=[];state=topics['rt/lowstate']
    if not writer.ready:issues.append('no valid recorded lowstate')
    if state['receipt_gap_max_s']>.1 or state['delivery_excess_max_s']>.1 or (state['last_receipt_age_s'] is not None and state['last_receipt_age_s']>.1):
        issues.append('lowstate observation stale/gap exceeds100ms')
    for name,s in topics.items():
        if any(s[k] for k in ('source_clock_regressions','invalid_samples','nonfinite_samples','writer_queue_drops','dds_reported_sample_lost','dds_reported_sample_rejected')):
            issues.append(name+': recording quality loss')
    if error or writer.error:issues.append('recorder error')
    if any(s.samples!=s.written for s in stats.values()):issues.append('not all observed samples written')
    return dict(schema='g1_episode_dds_result_v1',read_only=True,status='incomplete' if issues else 'clean',complete=not issues,
                quality_issues=issues,stop_reason=stop_reason,error=error or writer.error,host_start_monotonic_ns=start_ns,
                host_end_monotonic_ns=end_ns,duration_s=(end_ns-start_ns)/1e9,topics=topics,dds_history_depth=1,
                writer_queue_capacity=writer.queue.maxsize,pending_unwritten=writer.queue.qsize(),
                sample_policy='Latest available sample per topic at up to200Hz; superseded DDS samples are intentionally not a lossless servo trace',
                requested_sample_hz=200,
                loss_accounting='Local queue drops and DDS reported loss only; best-effort/history overwrites may be unobservable',
                clock_note='DDS source and Orin receipt clocks preserved; delivery excess is relative lower-envelope delay',
                command_note='Observed publications do not prove robot acceptance/execution')

def record(output,seconds,interface):
    output=Path(output).resolve(strict=True)
    if not output.is_dir() or any((output/name).exists() or (output/(name+'.tmp')).exists() for name in OWNED):
        raise ValueError('Output must be an existing episode directory without recorder files')
    stats={name:TopicStats(name) for name in TOPICS};writer=BoundedWriter(output,stats)
    ending=threading.Event();stop_reason=['duration']
    def stop(signum,_frame):stop_reason[0]=signal.Signals(signum).name;ending.set()
    previous={s:signal.getsignal(s) for s in (signal.SIGINT,signal.SIGTERM)}
    for s in previous:signal.signal(s,stop)
    start=time.monotonic_ns();error=None;readers=[];topics=[];listeners=[];participant=None
    try:
        from unitree_sdk2py.core.channel import ChannelFactoryInitialize
        from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_,LowCmd_
        from cyclonedds.domain import DomainParticipant
        from cyclonedds.topic import Topic
        from cyclonedds.sub import DataReader
        from cyclonedds.core import Listener
        from cyclonedds.qos import Qos,Policy
        from cyclonedds.internal import InvalidSample
        ChannelFactoryInitialize(0,interface);participant=DomainParticipant(0)
        qos=Qos(Policy.History.KeepLast(1),Policy.Reliability.BestEffort,
                Policy.ResourceLimits(max_samples=1,max_instances=1,max_samples_per_instance=1))
        for name in TOPICS:
            counter=stats[name]
            def lost(_reader,status,counter=counter):counter.sample_lost=max(counter.sample_lost,int(status.total_count))
            def rejected(_reader,status,counter=counter):counter.sample_rejected=max(counter.sample_rejected,int(status.total_count))
            listener=Listener(on_sample_lost=lost,on_sample_rejected=rejected)
            topic=Topic(participant,name,LowState_ if name=='rt/lowstate' else LowCmd_)
            listeners.append(listener);topics.append(topic);readers.append((name,DataReader(participant,topic,qos,listener)))
        while not ending.is_set() and (time.monotonic_ns()-start)/1e9<seconds:
            loop_start=time.monotonic()
            if (output/'STOP').exists():stop_reason[0]='STOP file';break
            if writer.error:raise RuntimeError(writer.error)
            had_data=False
            for name,reader in readers:
                for msg in reader.take(1):
                    mono,wall=time.monotonic_ns(),time.time_ns();had_data=True
                    # DDS writer disposal/no-writers notifications carry no payload;
                    # they are lifecycle events, not malformed sensor observations.
                    if isinstance(msg,InvalidSample) or not msg.sample_info.valid_data:stats[name].lifecycle_events+=1;continue
                    info=msg.sample_info;source=int(info.source_timestamp);handle=int(getattr(info,'publication_handle',0))
                    quality=stats[name].observe(source,mono,handle)
                    if quality is None:continue
                    try:row=serialize_sample(name,msg,source,mono,wall,stats[name].samples,handle)
                    except (AttributeError,ValueError,TypeError,OverflowError):stats[name].invalid+=1;continue
                    row.update(quality);stats[name].nonfinite_samples+=bool(row['invalid_numeric_fields']);writer.submit(row)
            ending.wait(max(0.,.005-(time.monotonic()-loop_start)))
    except BaseException as exc:stop_reason[0]='error';error=f'{type(exc).__name__}: {exc}'
    finally:
        end=time.monotonic_ns();readers.clear();topics.clear();listeners.clear();participant=None;writer.finish()
        for s,handler in previous.items():signal.signal(s,handler)
    result=result_summary(stats,start,end,stop_reason[0],error,writer);atomic_json(output/'state_result.json',result)
    return result

def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--seconds',type=float,required=True);parser.add_argument('--interface',default=DDS_INTERFACE);args=parser.parse_args()
    if not math.isfinite(args.seconds) or not 0<args.seconds<=400 or not args.interface:parser.error('Use positive duration up to400seconds and interface')
    result=record(args.output,args.seconds,args.interface)
    print(json.dumps(dict(status=result['status'],stop_reason=result['stop_reason'],state_rows=result['topics']['rt/lowstate']['written'])),flush=True)
    return 0 if result['status']=='clean' else 2

if __name__=='__main__':raise SystemExit(main())
