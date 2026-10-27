#!/usr/bin/env bash
set -euo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/../scripts/common.sh"
mode="${1:-help}"
if [[ $# -gt 0 ]]; then shift; fi
cd "$G1_PROJECT"
case "$mode" in
  verify)
    run_native "$G1_PROJECT/scripts/doctor.py" --pico
    run_ik "$G1_PROJECT/scripts/doctor.py" --ik
    ;;
  bridge|probe|trackers)
    mkdir -p work/pico_native/runtime
    cd work/pico_native/runtime
    if [[ "$mode" == bridge ]]; then
      run_native "$G1_PROJECT/outputs/pico_native_bridge.py" "$@"
    elif [[ "$mode" == trackers ]]; then
      run_ik "$G1_PROJECT/outputs/pico_tracking_probe.py" --viewer "$@"
    else
      run_native "$G1_PROJECT/outputs/pico_tracking_probe.py" "$@"
    fi
    ;;
  preview) run_ik "$G1_PROJECT/outputs/g1_arm_preview.py" "$@" ;;
  hands) run_ik "$G1_PROJECT/outputs/g1_inspire_teleop.py" "$@" ;;
  record) python3 "$G1_PROJECT/outputs/g1_episode.py" record "$@" ;;
  episode) python3 "$G1_PROJECT/outputs/g1_episode.py" "$@" ;;
  restore) python3 "$G1_PROJECT/scripts/restore_ai.py" "$@" ;;
  service)
    : "${PICO_SERVICE_SCRIPT:?Set PICO_SERVICE_SCRIPT in config/local.env to the native service launcher}"
    exec bash "$PICO_SERVICE_SCRIPT" "$@"
    ;;
  install-app)
    : "${PICO_APK:?Set PICO_APK to an official APK; verify the release checksum before installing}"
    if [[ -n "${PICO_SERIAL:-}" ]]; then device=(-s "$PICO_SERIAL"); else device=(-d); fi
    exec adb "${device[@]}" install -r "$PICO_APK"
    ;;
  *) echo 'Usage: bash outputs/pico_native.sh verify|service|bridge|preview|hands|record|episode|restore|probe|trackers|install-app [options]' ;;
esac
