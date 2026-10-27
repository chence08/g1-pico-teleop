from collections import deque
import json
from pathlib import Path
import sys
import threading
from types import SimpleNamespace
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'outputs'))
from g1_arm_preview import arm_feedback_mode
from g1_arm_link import ArmLink
from g1_arm_link import fsm_monitor_mode


class FeedbackTests(unittest.TestCase):
    def test_acknowledged_body_shutdown_is_distinct_from_a_fault(self):
        state=dict(link_age_s=.6,state_age_s=.6,fault=False,
                   control_scope='suspended_body',status='damping suspended body')
        self.assertEqual(arm_feedback_mode(state),'shutdown')
        self.assertEqual(arm_feedback_mode(state,process_exit=0),'shutdown')
        self.assertEqual(arm_feedback_mode(state,process_exit=1),'fault')
        self.assertEqual(arm_feedback_mode(dict(state,fault=True)),'fault')
        self.assertEqual(arm_feedback_mode(dict(state,control_scope='arms')),'fault')

    def test_rpc_timeout_holds_without_treating_unknown_mode_as_standing(self):
        self.assertEqual(fsm_monitor_mode(3104, None, 1.5), 'wait')
        self.assertEqual(fsm_monitor_mode(3104, None, 2.01), 'fault')
        self.assertEqual(fsm_monitor_mode(0, 1, .01), 'fault')
        self.assertEqual(fsm_monitor_mode(0, 500, .01), 'valid')
        self.assertEqual(fsm_monitor_mode(3102, None, .01), 'fault')
        state = {'link_age_s': .01, 'state_age_s': .001, 'fault': False, 'mode_query_ok': False}
        self.assertEqual(arm_feedback_mode(state), 'mode_wait')

    def test_brief_feedback_gap_holds_and_prolonged_loss_exits(self):
        state = {'link_age_s': .289, 'state_age_s': .006, 'fault': False}
        self.assertEqual(arm_feedback_mode(state), 'gap')
        state['link_age_s'] = 1.01
        self.assertEqual(arm_feedback_mode(state), 'lost')
        state['link_age_s'] = .01
        self.assertEqual(arm_feedback_mode(state), 'fresh')

    def test_fault_and_stale_robot_state_are_never_treated_as_brief_link_gap(self):
        state = {'link_age_s': .289, 'state_age_s': .006, 'fault': True}
        self.assertEqual(arm_feedback_mode(state), 'fault')
        state.update(fault=False, state_age_s=.101)
        state['link_age_s'] = .01
        self.assertEqual(arm_feedback_mode(state), 'state_wait')
        state['state_age_s'] = .501
        self.assertEqual(arm_feedback_mode(state), 'fault')
        state['state_age_s'] = .006
        self.assertEqual(arm_feedback_mode(state, process_exit=0), 'fault')

    def test_final_robot_fault_replaces_previous_nonfault_status(self):
        link = ArmLink.__new__(ArmLink)
        link.lock = threading.Lock()
        link.errors = deque(maxlen=5)
        link.state, link.received = None, 0
        link.process = SimpleNamespace(stdout=[
            json.dumps({'type': 'arm_state', 'fault': False, 'status': 'clutch: holding'}),
            json.dumps({'type': 'arm_error', 'fault_reasons': ['FSM query failed']})])
        link._read()
        self.assertTrue(link.state['fault'])
        self.assertEqual(link.state['fault_reasons'], ['FSM query failed'])


if __name__ == '__main__':
    unittest.main()
