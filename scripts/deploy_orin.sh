#!/usr/bin/env bash
set -euo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/common.sh"
exec python3 "$G1_PROJECT/scripts/deploy_orin.py" "$@"
