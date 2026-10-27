"""Report measured software-stage latency and joint tracking from arm telemetry.

PC input age begins at bridge polling, not headset capture. PC send->ACK is
round-trip plus publication/telemetry scheduling, not one-way motor latency.
"""
import argparse,json
from pathlib import Path
import numpy as np


def percentiles(values):
    v=np.asarray([x for x in values if x is not None and np.isfinite(x)],dtype=float)
    if not len(v):return None
    return dict(zip(('p50','p95','p99','max'),np.percentile(v,[50,95,99,100]).tolist()))


def motion_lag(rows, reference):
    """Descriptive cross-correlation on continuously following, moving segments."""
    follow=[r for r in rows if r.get('status')=='following' and r.get('weight',0)>.99]
    if len(follow)<60:return []
    t=np.array([r['server_monotonic_s'] for r in follow]);meas=np.array([r['q'] for r in follow]);ref=np.array([r[reference] for r in follow])
    # New traces preserve acquisition age. Avoid interpreting IPC/reader queues
    # as motor dynamics; a fixed unknown source-to-host offset still remains.
    mt=t-np.array([r.get('robot_state_age_s',r.get('state_age_s',0)) if 'state_capture_received_s' in r else 0 for r in follow])
    unique=np.r_[True,np.diff(mt)>0]
    mt=mt[unique];meas=meas[unique]
    if len(mt)<60:return []
    out=[]
    for j in range(14):
        if np.std(ref[:,j])<.02:continue
        estimates=[]
        for lag in np.arange(0,.505,.005):
            query=t+lag;ids=np.searchsorted(mt,query)
            valid=(ids>0)&(ids<len(mt));ii=np.clip(ids,1,len(mt)-1)
            valid&=(mt[ii]-mt[ii-1]<.1)
            if np.sum(valid)<60:continue
            a=ref[valid,j];b=np.interp(query[valid],mt,meas[:,j])
            if np.std(a)<.01 or np.std(b)<.01:continue
            correlation=float(np.corrcoef(a,b)[0,1])
            estimates.append((correlation,float(lag*1000)))
        if estimates:
            corr,lag=max(estimates)
            out.append({'motor_id':j+15,'lag_estimate_ms':lag,'correlation':corr,
                        'usable_excitation':bool(corr>.8 and lag<500)})
    return out


def analyze(path):
    rows=[json.loads(x) for x in Path(path).read_text().splitlines() if x.strip()]
    active=[r for r in rows if r.get('weight',0)>.99]
    following=[r for r in active if r.get('status')=='following']
    errors=[max(abs(a-b) for a,b in zip(r['q_command'],r['q'])) for r in following]
    settled=[r for r in following if max(abs(v) for v in r['dq_command'])<.03 and
             max(abs(a-b) for a,b in zip(r['q_command'],r['q_target']))<.02]
    return {'telemetry':str(path),'profile':rows[-1].get('profile'),'transport':rows[-1].get('arm_transport','ssh'),'rows':len(rows),'following_rows':len(following),
            'span_s':rows[-1]['pc_monotonic_s']-rows[0]['pc_monotonic_s'],
            'pc_input_to_send_ms':percentiles([r.get('pc_input_to_send_ms') for r in following]),
            'pc_ready_to_send_ms':percentiles([r.get('pc_ready_to_send_ms') for r in following]),
            'pc_send_to_ack_ms':percentiles([r.get('pc_send_to_ack_ms') for r in following]),
            'robot_receive_to_dds_publish_ms':percentiles([r.get('robot_receive_to_publish_ms') for r in following]),
            'feedback_age_ms':percentiles([1000*r['state_age_s'] for r in rows]),
            'robot_local_feedback_age_ms':percentiles([1000*r['robot_state_age_s'] for r in rows if 'robot_state_age_s' in r]),
            'feedback_network_excess_delay_ms':percentiles([1000*r['feedback_network_delay_s'] for r in rows if 'feedback_network_delay_s' in r]),
            'robot_challenge_roundtrip_age_ms':percentiles([1000*r['robot_challenge_age_s'] for r in following if 'robot_challenge_age_s' in r]),
            'robot_input_age_upper_bound_ms':percentiles([1000*r['robot_input_age_upper_s'] for r in following if 'robot_input_age_upper_s' in r]),
            'transport_missing_states':max((r.get('transport_missing',0) for r in rows),default=0),
            'transport_max_packet_bytes':max((r.get('transport_packet_bytes',0) for r in rows),default=0),
            'feedback_delivery_delay_ms':percentiles([1000*r['state_delivery_delay_s'] for r in rows if 'state_delivery_delay_s' in r]),
            'feedback_ipc_drops_total':max((r.get('state_ipc_drops',0) for r in rows),default=0),
            'ik_compute_ms':percentiles([r.get('ik_duration_ms') for r in following]),
            'max_joint_tracking_error_rad':percentiles(errors),
            'settled_tracking_error_rad':percentiles([max(abs(a-b) for a,b in zip(r['q_command'],r['q'])) for r in settled]),
            'peak_commanded_joint_speed_rad_s':max(max(abs(v) for v in r['dq_command']) for r in rows),
            'at_speed_cap_fraction':sum(any(abs(v)>.999*r.get('speed_limit_rad_s',1.) for v in r['dq_command']) for r in following)/max(1,len(following)),
            'tracking_limited_fraction':sum(any(r.get('tracking_limited',[])) for r in following)/max(1,len(following)),
            'interpolation_workspace_blocked_fraction':sum(any(r.get('workspace_blocked',[])) for r in following)/max(1,len(following)),
            'workspace_limited_fraction':sum('workspace limited' in r.get('pc_status','') for r in following)/max(1,len(following)),
            'fault_rows':sum(r.get('fault',False) for r in rows),
            'gravity_saturated_rows':sum(any(r.get('gravity_saturated',[])) for r in following),
            'command_to_motion_lag':motion_lag(following,'q_command'),
            'ik_goal_to_motion_lag':motion_lag(following,'q_target') if following else [],
            'measurement_limits':'No headset-capture clock sync. ACK is PC round-trip, publication is not hardware acceptance. Lag estimates need moving excitation and can be biased by saturation or changing gravity.'}


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('telemetry',type=Path);p.add_argument('--out',type=Path);a=p.parse_args()
    result=analyze(a.telemetry);text=json.dumps(result,indent=2);print(text)
    if a.out:a.out.write_text(text)
