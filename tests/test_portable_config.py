"""Migration regressions: configuration reaches remote workers as literal data."""
from pathlib import Path
import json
import os
import shlex
import subprocess
import sys
import tempfile
import unittest
from unittest import mock
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'outputs'))
import g1_config as config

class PortableConfigTests(unittest.TestCase):
    def test_remote_arguments_are_quoted_without_expansion(self):
        strange="/tmp/robot data/$(printf surprise);*.py"
        args=["python", strange, "--task", "receive 'box'"]
        self.assertEqual(shlex.split(config.remote_command(args))[-len(args):],args)
    def test_custom_config_propagates_to_workers(self):
        env=dict(os.environ,G1_REMOTE_ROOT='/srv/robot tools',G1_REMOTE_PYTHON='/env/custom/bin/python',G1_DDS_INTERFACE='robot0',INSPIRE_LEFT_PORT='/dev/left_hand',G1_HOST='robot-alias')
        code='import json,g1_config as c;print(json.dumps([c.HOST,c.REMOTE_TOOLS,c.robot_settings(),c.ssh(["python","x.py"])]))'
        env['PYTHONPATH']=str(ROOT/'outputs')
        result=json.loads(subprocess.check_output([sys.executable,'-c',code],env=env,text=True))
        self.assertEqual(result[0],'robot-alias');self.assertEqual(result[1],'/srv/robot tools/tools')
        self.assertEqual(result[2]['G1_DDS_INTERFACE'],'robot0');self.assertEqual(result[3][-2],'robot-alias')
        self.assertIn('INSPIRE_LEFT_PORT=/dev/left_hand',shlex.split(result[3][-1]))
    def test_deployed_config_is_machine_local_and_excluded(self):
        import shutil
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory);shutil.copy(ROOT/'outputs/g1_config.py',path/'g1_config.py')
            (path/'deployment.json').write_text(json.dumps({'G1_REMOTE_ROOT':'/srv/robot','G1_DDS_INTERFACE':'robot0'}))
            env={k:v for k,v in os.environ.items() if not k.startswith('G1_')};env['PYTHONPATH']=directory
            result=subprocess.check_output([sys.executable,'-c','import g1_config as c;print(c.REMOTE_TOOLS,c.DDS_INTERFACE)'],env=env,cwd=directory,text=True)
            self.assertEqual(result.strip(),'/srv/robot/tools robot0')
if __name__=='__main__':unittest.main()
