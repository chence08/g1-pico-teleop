import sys,unittest
from pathlib import Path
from unittest.mock import Mock
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'outputs'))
from g1_motion_mode import restore_ai

class RestoreAiTests(unittest.TestCase):
    def test_busy_ack_is_confirmed_without_repeating_selection(self):
        client=Mock()
        client.CheckMode.side_effect=[(0,{'name':''}),(3104,None),(0,{'name':'ai'})]
        client.SelectMode.return_value=(3104,None)
        result=restore_ai(client,wait=lambda _:None)
        self.assertTrue(result['restored']);client.SelectMode.assert_called_once_with('ai')

    def test_existing_ai_does_not_restart_controller(self):
        client=Mock();client.CheckMode.return_value=(0,{'name':'ai'})
        self.assertFalse(restore_ai(client)['restored']);client.SelectMode.assert_not_called()

    def test_unknown_or_competing_ownership_is_not_changed(self):
        for result in [(3102,None),(0,{}),(0,{'name':'other'})]:
            client=Mock();client.CheckMode.return_value=result
            with self.assertRaises(RuntimeError):restore_ai(client,wait=lambda _:None)
            client.SelectMode.assert_not_called()

    def test_no_blind_retry_when_restoration_is_unconfirmed(self):
        client=Mock();client.CheckMode.return_value=(0,{'name':''})
        with self.assertRaises(RuntimeError):restore_ai(client,wait=lambda _:None)
        client.SelectMode.assert_called_once_with('ai')
