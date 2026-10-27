"""Portable deployment settings. Shell entrypoints load config/local.env.

No discovery, connections, or robot commands occur at import. Deployment copies
this module and an explicit JSON settings file to the Orin.
"""
import json
import os
from pathlib import Path
import shlex

PROJECT = Path(__file__).resolve().parent.parent
_SETTINGS = Path(__file__).with_name('deployment.json')
_settings = json.loads(_SETTINGS.read_text()) if _SETTINGS.exists() else {}

def setting(name, default):
    return os.environ.get(name, _settings.get(name, default))

HOST = setting('G1_HOST', 'g1')
REMOTE_ROOT = setting('G1_REMOTE_ROOT', '/home/unitree/g1_dataset')
REMOTE_TOOLS = REMOTE_ROOT + '/tools'
REMOTE_PYTHON = setting('G1_REMOTE_PYTHON', '/home/unitree/miniconda3/envs/unitree_deploy/bin/python')
DDS_INTERFACE = setting('G1_DDS_INTERFACE', 'enP8p1s0')
XR_REPO = Path(setting('XR_TELEOPERATE_REPO', str(PROJECT/'third_party/xr_teleoperate'))).expanduser()
LEFT_PORT = setting('INSPIRE_LEFT_PORT', '/dev/g1_inspire_left')
RIGHT_PORT = setting('INSPIRE_RIGHT_PORT', '/dev/g1_inspire_right')

def robot_settings():
    return dict(G1_REMOTE_ROOT=REMOTE_ROOT, G1_REMOTE_PYTHON=REMOTE_PYTHON,
                G1_DDS_INTERFACE=DDS_INTERFACE, INSPIRE_LEFT_PORT=LEFT_PORT, INSPIRE_RIGHT_PORT=RIGHT_PORT)

def remote_command(args):
    """Quote each remote shell argument, including deployment paths with spaces."""
    return shlex.join(['env', *[f'{k}={v}' for k,v in robot_settings().items()], *map(str,args)])

def ssh(args, host=HOST):
    return ['ssh','-4','-T','-o','BatchMode=yes','-o','ConnectTimeout=10',host,remote_command(args)]
