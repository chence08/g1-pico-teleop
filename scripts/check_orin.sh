#!/usr/bin/env bash
set -euo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/common.sh"
exec python3 -B "$G1_PROJECT/scripts/check_orin.py" "$@"
