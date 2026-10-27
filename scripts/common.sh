#!/usr/bin/env bash
# Source from repo entrypoints; trusted local configuration is shell syntax.
G1_PROJECT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ -f "$G1_PROJECT/config/local.env" ]]; then
  set -a
  source "$G1_PROJECT/config/local.env"
  set +a
fi
export XR_TELEOPERATE_REPO="${XR_TELEOPERATE_REPO:-$G1_PROJECT/third_party/xr_teleoperate}"
run_ik() {
  if [[ -n "${IK_PYTHON:-}" ]]; then "$IK_PYTHON" "$@";
  else conda run --no-capture-output -n g1-ik python "$@"; fi
}
run_native() {
  if [[ -n "${NATIVE_PYTHON:-}" ]]; then "$NATIVE_PYTHON" "$@";
  else conda run --no-capture-output -n g1-pico python "$@"; fi
}
