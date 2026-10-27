"""Portable paths; recorded data is opt-in and is never bundled with tests."""
import os
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def configured_path(name, default=None):
    value = os.environ.get(name)
    if not value:
        return default
    path = Path(os.path.expandvars(value)).expanduser()
    return (ROOT / path).resolve() if not path.is_absolute() else path.resolve()


XR_REPO = configured_path('XR_TELEOPERATE_REPO', ROOT / 'third_party/xr_teleoperate')
URDF = str(XR_REPO / 'assets/g1/g1_body29_hand14.urdf')
FIXTURE_DIR = configured_path('G1_TEST_FIXTURES')
MODELS_ENABLED = os.environ.get('G1_TEST_MODELS') == '1'


def fixture(name):
    if FIXTURE_DIR is None:
        raise RuntimeError('Set G1_TEST_FIXTURES to an external directory of recorded fixtures')
    return FIXTURE_DIR / name
