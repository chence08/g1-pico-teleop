"""One bounded synthetic joint check; --live enables physical arm output.

No PICO input is used. Fresh, locally generated test targets drive one shoulder
through +0.12 rad and back. Robot-side range, mode, ownership, speed, feedback,
and target-expiry checks remain active. User must watch with the stop ready.
"""
import argparse
import json
from pathlib import Path
import time
from g1_arm_link import ArmLink


def test_offset(elapsed):
    # Three-second takeover at measured pose, two-second outward move,
    # three-second pause, two-second return, two-second rest.
    if elapsed < 3:
        return 0.
    if elapsed < 5:
        return .12 * (elapsed - 3) / 2
    if elapsed < 8:
        return .12
    if elapsed < 10:
        return .12 * (10 - elapsed) / 2
    return 0.


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--live', action='store_true')
    parser.add_argument('--profile',choices=('commissioning','responsive'),default='commissioning')
    parser.add_argument('--gravity-comp',choices=('off','calibrated'),default='off')
    parser.add_argument('--velocity-reference',choices=('on','off'),default='off')
    parser.add_argument('--step-test',action='store_true')
    args = parser.parse_args()
    path = Path(__file__).resolve().parents[1] / 'work' / f'arm_synthetic_{time.time_ns()}.jsonl'
    link = ArmLink(live=args.live,profile=args.profile,gravity_comp=args.gravity_comp,
                   velocity_reference=args.velocity_reference=='on')
    try:
        state = link.snapshot(timeout=12)
        initial = list(state['q'])
        print(f"{'LIVE' if args.live else 'SHADOW'} synthetic check: motor 16 (left shoulder roll), +{.08 if args.step_test else .12:g} rad then return", flush=True)
        print('Full state log:', path, flush=True)
        with path.open('w') as log:
            start = time.monotonic()
            next_print = start
            while time.monotonic() - start < (10.4 if args.step_test else 12.4):
                now = time.monotonic()
                elapsed = now - start
                state = link.snapshot()
                if (state['fault'] or state['link_age_s'] > .2 or state['state_age_s'] > .1
                        or not state.get('mode_query_ok') or link.process.poll() is not None):
                    raise RuntimeError(f"Stopped: fault={state['fault']}, link_age={state['link_age_s']:.3f}, "
                                       f"mode_ok={state.get('mode_query_ok')}, reasons={state.get('fault_reasons')}")
                target = initial.copy()
                target[1] += (.08 if 2 <= elapsed < 4 or 6 <= elapsed < 8 else 0.) if args.step_test else test_offset(max(0, elapsed - .4))
                # Initial explicit release, then a freshly generated synthetic
                # intent. This timestamp describes test generation, not PICO.
                link.submit(target, now, follow=elapsed >= .4)
                log.write(json.dumps(dict(source='synthetic_joint_check', elapsed_s=elapsed,
                                          generated_target_rad=target, **state)) + '\n')
                if now >= next_print:
                    print(f"t={elapsed:.1f}s weight={state['weight']:.2f} "
                          f"left_shoulder_delta_deg: requested={(state['q_command'][1]-initial[1])*57.29578:.2f} "
                          f"measured={(state['q'][1]-initial[1])*57.29578:.2f} "
                          f"status={state['status']}", flush=True)
                    next_print = now + 1
                time.sleep(.02)
        print('Sequence finished; releasing arm control.', flush=True)
    finally:
        link.close()
    print('Arm link closed.', flush=True)


if __name__ == '__main__':
    main()
