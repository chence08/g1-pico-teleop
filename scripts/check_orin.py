"""Read-only Orin runtime checks over SSH; no SDK initialization or device opens.

Imports dependencies in separate Python processes and inspects paths/permissions.
This does not prove camera streaming, DDS connectivity, or physical hand mapping.
"""
import argparse
import json
from pathlib import Path
import subprocess
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'outputs'))
from g1_config import HOST, REMOTE_PYTHON, REMOTE_TOOLS, DDS_INTERFACE, LEFT_PORT, RIGHT_PORT, ssh


IMPORT_PROBE = r'''
import contextlib, importlib, importlib.metadata, io, json, re, sys
kind = sys.argv[1]
rows = {}
captured = io.StringIO()
def module(name):
    try:
        with contextlib.redirect_stdout(captured):
            value = importlib.import_module(name)
        version = getattr(value, '__version__', None)
        if version is None:
            try: version = importlib.metadata.version(name.split('.')[0])
            except importlib.metadata.PackageNotFoundError: pass
        rows[name] = dict(ok=True, version=str(version) if version is not None else None)
        return value
    except Exception as error:
        rows[name] = dict(ok=False, error=type(error).__name__ + ': ' + str(error)[:250])
        return None
module('numpy')
if kind == 'robot':
    module('pinocchio')
    collision = module('coal')
    if collision is None:
        collision = module('hppfcl')
    if collision is not None and rows.get('coal', {}).get('ok') is False:
        rows.pop('coal')
    module('cyclonedds')
    module('unitree_sdk2py')
    module('unitree_sdk2py.core.channel')
    module('unitree_sdk2py.idl.unitree_hg.msg.dds_')
else:
    camera = module('cv2')
    module('pyrealsense2')
    rows['opencv_gstreamer'] = dict(ok=bool(camera is not None and
        re.search(r'^\s*GStreamer\s*:\s*YES\b', camera.getBuildInformation(), re.M)))
report = dict(ok=all(row['ok'] for row in rows.values()), python=sys.version.split()[0],
              executable=sys.executable, checks=rows)
print('G1_RUNTIME_CHECK_JSON=' + json.dumps(report))
'''


REMOTE_CHECK = r'''
import json, os, pathlib, shutil, stat, subprocess, sys
robot_python, interface, left, right, tools, after, probe = sys.argv[1:]
environment = dict(os.environ, PYTHONDONTWRITEBYTECODE='1', GST_REGISTRY_UPDATE='no')
groups = {}
def python_check(executable, kind):
    try:
        process = subprocess.run([executable, '-B', '-c', probe, kind], capture_output=True,
                                 text=True, timeout=30, env=environment)
        prefix = 'G1_RUNTIME_CHECK_JSON='
        lines = [line[len(prefix):] for line in process.stdout.splitlines() if line.startswith(prefix)]
        if process.returncode or len(lines) != 1:
            raise RuntimeError('Python probe exit ' + str(process.returncode) + ': ' + process.stderr[-400:])
        return json.loads(lines[0])
    except Exception as error:
        return dict(ok=False, error=type(error).__name__ + ': ' + str(error)[:400])
groups['robot_python'] = python_check(robot_python, 'robot')
groups['camera_python'] = python_check('python3', 'camera')
commands = {}
for name, command in [('ffprobe', ['ffprobe', '-version']),
                      ('nvv4l2h264enc', ['gst-inspect-1.0', 'nvv4l2h264enc']),
                      ('nvvidconv', ['gst-inspect-1.0', 'nvvidconv'])]:
    executable = shutil.which(command[0])
    if executable is None:
        commands[name] = dict(ok=False, error=command[0] + ' not found')
        continue
    try:
        process = subprocess.run(command, capture_output=True, text=True, timeout=10, env=environment)
        row = dict(ok=process.returncode == 0, executable=executable)
        if process.returncode: row['error'] = process.stderr[-250:]
        commands[name] = row
    except Exception as error:
        commands[name] = dict(ok=False, error=type(error).__name__ + ': ' + str(error)[:250])
groups['camera_tools'] = dict(ok=all(row['ok'] for row in commands.values()), checks=commands)
network = pathlib.Path('/sys/class/net') / interface
valid_interface = bool(interface) and pathlib.Path(interface).name == interface and network.is_dir()
network_row = dict(ok=valid_interface, interface=interface)
if valid_interface:
    try: network_row['operstate'] = (network / 'operstate').read_text().strip()
    except OSError: network_row['operstate'] = 'unavailable'
groups['dds_interface'] = network_row
ports = {}
for side, port in [('left', left), ('right', right)]:
    try:
        is_character = stat.S_ISCHR(os.stat(port).st_mode)
        readable, writable = os.access(port, os.R_OK), os.access(port, os.W_OK)
        ports[side] = dict(ok=is_character and readable and writable, path=port,
                           character_device=is_character, readable=readable, writable=writable)
    except OSError as error:
        ports[side] = dict(ok=False, path=port, error=str(error))
groups['hand_permissions'] = dict(ok=all(row['ok'] for row in ports.values()), checks=ports,
                                 note='Permissions only; ports were not opened and physical side mapping was not tested')
if after == '1':
    root = pathlib.Path(tools)
    names = ['g1_body29_hand14.urdf', 'meshes/torso_link_rev_1_0.STL']
    names += ['meshes/' + side + '_' + joint + '_link.STL' for side in ('left', 'right')
              for joint in ('shoulder_yaw', 'elbow', 'wrist_roll', 'wrist_pitch', 'wrist_yaw')]
    missing = []
    for name in names:
        path = root / name
        try: valid = path.is_file() and path.stat().st_size > 0
        except OSError: valid = False
        if not valid: missing.append(name)
    groups['deployed_assets'] = dict(ok=not missing, root=str(root), checked_files=len(names), missing=missing)
else:
    groups['deployed_assets'] = dict(ok=True, skipped=True, note='Use --after-deploy to require robot model and workspace meshes')
result = dict(ok=all(group['ok'] for group in groups.values()), read_only=True, groups=groups,
              note='No SDK factory initialized, no DDS participant created, no camera/serial device opened, no motion commanded')
print('G1_ORIN_CHECK_JSON=' + json.dumps(result))
'''


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--after-deploy', action='store_true', help='Also require deployed G1 URDF and workspace meshes')
    args = parser.parse_args(argv)
    command = ['python3', '-B', '-c', REMOTE_CHECK, REMOTE_PYTHON, DDS_INTERFACE,
               LEFT_PORT, RIGHT_PORT, REMOTE_TOOLS, '1' if args.after_deploy else '0', IMPORT_PROBE]
    try:
        process = subprocess.run(ssh(command), capture_output=True, text=True, timeout=110)
        prefix = 'G1_ORIN_CHECK_JSON='
        lines = [line[len(prefix):] for line in process.stdout.splitlines() if line.startswith(prefix)]
        if process.returncode or len(lines) != 1:
            raise RuntimeError('Remote check exit ' + str(process.returncode) + ': ' + process.stderr[-500:])
        result = json.loads(lines[0])
        result['host'] = HOST
    except Exception as error:
        result = dict(ok=False, read_only=True, host=HOST, error=type(error).__name__ + ': ' + str(error))
    print(json.dumps(result, indent=2))
    return 0 if result['ok'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
