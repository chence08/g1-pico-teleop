#!/usr/bin/env python3
"""Offline test groups. No group connects to a robot, camera, or PICO SDK.

unit: stdlib + NumPy; local loopback and Unix sockets are used.
model: additionally needs Pinocchio with CasADi bindings, CasADi, hpp-fcl/coal,
       and XR_TELEOPERATE_REPO (default: third_party/xr_teleoperate).
G1_TEST_FIXTURES optionally names an external directory containing the two
recorded arm telemetry fixtures. Missing files fail when explicitly enabled.
check: compile Python and check shell syntax without importing implementations.
"""
import argparse
import importlib
import importlib.abc
import os
from pathlib import Path
import subprocess
import sys
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'outputs'))
sys.path.insert(0, str(ROOT / 'tests'))

UNIT_MODULES = (
    'test_arm_datagram', 'test_arm_feedback', 'test_arm_link',
    'test_arm_transport', 'test_bridge_pipeline', 'test_command_monitor',
    'test_elbow_reference', 'test_episode', 'test_episode_camera',
    'test_episode_state', 'test_inspire_hands', 'test_latest_state',
    'test_motion_mode', 'test_object_preview', 'test_pico_native',
    'test_portable_config', 'test_file_transfer',
    'test_preview_regressions', 'test_restore_ai', 'test_sdk_lease',
    'test_smooth_motion', 'test_workspace_tracking',
)
MODEL_MODULES = (
    'test_gravity_comp', 'test_leg_ik', 'test_arm_runtime_simulated',
    'test_suspended_body',
)
RECORDED_MODEL = ('test_workspace_tracking.TrackingWorkspaceTests.'
                  'test_recorded_stall_is_excluded_and_interpolation_blocks_that_arm')


class UnitDependencyGuard(importlib.abc.MetaPathFinder):
    """Catch accidental runtime/hardware dependencies in the minimal group."""
    blocked = frozenset(('pinocchio', 'casadi', 'hppfcl', 'coal', 'meshcat',
                         'cv2', 'pyrealsense2', 'serial', 'cyclonedds',
                         'unitree_sdk2py', 'XRoboToolkit_PyBind', 'xrobotoolkit_sdk'))

    def find_spec(self, fullname, path=None, target=None):
        if fullname.split('.')[0] in self.blocked:
            raise ImportError(f'{fullname} is outside the NumPy-only unit group')
        return None


def check_sources():
    paths = sorted(path for folder in ('outputs', 'scripts', 'tests')
                   for path in (ROOT / folder).rglob('*.py'))
    for path in paths:
        compile(path.read_bytes(), str(path), 'exec')
    shells = sorted(path for folder in ('outputs', 'scripts')
                    for path in (ROOT / folder).rglob('*.sh'))
    for path in shells:
        subprocess.run(['bash', '-n', str(path)], check=True)
    print(f'Syntax checked {len(paths)} Python and {len(shells)} shell files; '
          'implementations were not imported. This does not verify robot hardware.')


def require_models():
    from _support import URDF
    try:
        for module in ('numpy', 'pinocchio', 'pinocchio.casadi', 'casadi'):
            importlib.import_module(module)
        try:
            importlib.import_module('hppfcl')
        except ImportError:
            importlib.import_module('coal')
    except ImportError as exc:
        raise RuntimeError('Model tests need Pinocchio with CasADi bindings, '
                           'CasADi, and hpp-fcl/coal in the selected Python environment') from exc
    urdf = Path(URDF)
    mesh_names = ['torso_link_rev_1_0.STL'] + [
        f'{side}_{joint}_link.STL' for side in ('left', 'right')
        for joint in ('shoulder_yaw', 'elbow', 'wrist_roll', 'wrist_pitch', 'wrist_yaw')]
    missing = [path for path in [urdf, *(urdf.parent / 'meshes' / name for name in mesh_names)]
               if not path.is_file()]
    if missing:
        raise RuntimeError(f'Model assets missing: {missing[0]}. Set '
                           'XR_TELEOPERATE_REPO or install third_party/xr_teleoperate.')


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('group', nargs='?', choices=('unit', 'model', 'all', 'check'), default='unit')
    parser.add_argument('-v', '--verbose', action='store_true')
    args = parser.parse_args(argv)
    if args.group == 'check':
        check_sources()
        return 0
    # Keep deployment configuration outside the offline tests. No subprocess
    # in these groups is permitted to start a robot or SDK connection.
    os.environ['G1_TEST_MODELS'] = '1' if args.group in ('model', 'all') else '0'
    if args.group in ('model', 'all'):
        try:
            require_models()
        except RuntimeError as exc:
            parser.error(str(exc))
    else:
        sys.meta_path.insert(0, UnitDependencyGuard())
    names = []
    if args.group in ('unit', 'all'):
        names.extend(UNIT_MODULES)
    if args.group in ('model', 'all'):
        names.extend(MODEL_MODULES)
        if args.group == 'model':
            names.append(RECORDED_MODEL)
    suite = unittest.defaultTestLoader.loadTestsFromNames(names)
    result = unittest.TextTestRunner(verbosity=2 if args.verbose else 1).run(suite)
    return 0 if result.wasSuccessful() else 1


if __name__ == '__main__':
    raise SystemExit(main())
