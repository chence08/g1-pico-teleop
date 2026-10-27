"""Explicit normal-controller ownership restoration. No posture selection."""
from pathlib import Path
import subprocess
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'outputs'))
from g1_config import REMOTE_TOOLS, REMOTE_PYTHON, ssh
if __name__=='__main__':
    if len(sys.argv)>1: raise SystemExit('Usage: bash outputs/pico_native.sh restore')
    raise SystemExit(subprocess.call(ssh([REMOTE_PYTHON,'-u',REMOTE_TOOLS+'/g1_motion_mode.py','--restore-ai'])))
