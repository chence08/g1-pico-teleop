#!/usr/bin/env bash
# Provision workstation dependencies only. No service, SDK client, or robot is started.
set -euo pipefail
repo_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
checkout_only=0
skip_envs=0
skip_sdk=0
build_service=0
qt_prefix="${QT_PREFIX:-}"
jobs="${G1_BUILD_JOBS:-2}"

usage() {
  cat <<'EOF'
Usage: bash scripts/bootstrap_workstation.sh [options]

  --deps-only          Fetch exact source checkouts; do not create envs or build.
  --skip-envs           Reuse existing g1-ik and g1-pico environments.
  --skip-sdk            Do not build the native PICO SDK/Python extension.
  --build-service       Also compile the native PC Service (never start it).
  --qt-prefix PATH      Qt6 prefix for --build-service, e.g. /usr or a Qt SDK.
  --jobs N              Build parallelism; default 2 to limit compiler memory.
  -h, --help            Show this text without modifying anything.

Requires git, python3, a native C/C++ compiler, and Conda/Miniforge (except
--deps-only). Service builds additionally need Qt6 Core, Network, Core5Compat
development files described in docs/SETUP.md. Workstation: Linux x86_64 only.
No apt, sudo, service startup, robot connection, or actuator command is run.
EOF
}

while (($#)); do
  case "$1" in
    --deps-only) checkout_only=1; shift ;;
    --skip-envs) skip_envs=1; shift ;;
    --skip-sdk) skip_sdk=1; shift ;;
    --build-service) build_service=1; shift ;;
    --qt-prefix) [[ $# -ge 2 ]] || { usage >&2; exit 2; }; qt_prefix="$2"; shift 2 ;;
    --jobs) [[ $# -ge 2 ]] || { usage >&2; exit 2; }; jobs="$2"; shift 2 ;;
    -h|--help) usage; exit 0 ;;
    *) printf 'Unknown option: %s\n' "$1" >&2; usage >&2; exit 2 ;;
  esac
done
[[ "$jobs" =~ ^[1-9][0-9]*$ ]] || { printf 'jobs must be a positive integer\n' >&2; exit 2; }
[[ "$(uname -s)" == Linux ]] || { printf 'This bootstrap requires Linux.\n' >&2; exit 2; }
[[ "$(uname -m)" == x86_64 ]] || { printf 'This workstation setup requires Linux x86_64.\n' >&2; exit 2; }
for executable in git python3; do
  command -v "$executable" >/dev/null || { printf 'Missing prerequisite: %s\n' "$executable" >&2; exit 2; }
done
if [[ "$build_service" == 1 && -z "$qt_prefix" ]]; then
  printf -- '--build-service requires --qt-prefix PATH; setup does not install Qt.\n' >&2
  exit 2
fi

# Never reset a user's source edits. The one allowed tracked change is a library
# rebuilt by this script into the SDK's x86 output directory.
check_clean() {
  local path="$1" name="$2"
  if [[ "$name" == pico_service ]]; then
    git -C "$path" diff --quiet HEAD -- . \
      ':(exclude)RoboticsService/SDK/linux/64/libPXREARobotSDK.so'
  else
    git -C "$path" diff --quiet HEAD -- .
  fi
}

dependency_rows="$(python3 - "$repo_root/dependencies.lock.json" <<'PY'
import json,sys
for name,item in json.load(open(sys.argv[1]))['repositories'].items():
    print(name,item['url'],item['revision'],item['path'],sep='\t')
PY
)"
while IFS=$'\t' read -r name url revision relative_path; do
  checkout="$repo_root/$relative_path"
  new_checkout=0
  if [[ -e "$checkout" && ! -d "$checkout/.git" ]]; then
    printf 'Refusing non-Git dependency directory: %s\n' "$checkout" >&2; exit 2
  fi
  if [[ ! -d "$checkout/.git" ]]; then
    mkdir -p "$(dirname -- "$checkout")"
    git clone --no-checkout -- "$url" "$checkout"
    new_checkout=1
  fi
  [[ "$(git -C "$checkout" remote get-url origin)" == "$url" ]] || {
    printf 'Unexpected origin for %s; inspect it before retrying.\n' "$checkout" >&2; exit 2;
  }
  if [[ "$new_checkout" == 0 ]]; then
    check_clean "$checkout" "$name" || {
      printf 'Dependency has source edits; preserving them: %s\n' "$checkout" >&2; exit 2;
    }
  fi
  if ! git -C "$checkout" cat-file -e "$revision^{commit}" 2>/dev/null; then
    git -C "$checkout" fetch origin "$revision"
  fi
  if [[ "$new_checkout" == 1 || "$(git -C "$checkout" rev-parse HEAD 2>/dev/null || true)" != "$revision" ]]; then
    git -C "$checkout" checkout --detach "$revision"
  fi
  # Native IK imports only this checkout's arm IK and weighted filter modules.
  # Browser, image-streaming and finger-retargeting submodules are not needed.
done <<< "$dependency_rows"

printf 'Pinned x86 workstation source revisions checked out.\n'
if [[ "$checkout_only" == 1 ]]; then exit 0; fi
conda_bin="${CONDA_EXE:-}"
if [[ -z "$conda_bin" ]]; then conda_bin="$(command -v conda || true)"; fi
if [[ -z "$conda_bin" || ! -x "$conda_bin" ]]; then
  printf 'Conda/Miniforge is required. Activate its shell or set CONDA_EXE to its executable.\n' >&2; exit 2
fi
for executable in c++ make; do
  command -v "$executable" >/dev/null || { printf 'Install native build prerequisite: %s\n' "$executable" >&2; exit 2; }
done
if [[ "$skip_envs" == 0 ]]; then
  for spec in ik pico; do
    environment=g1-ik; [[ "$spec" == pico ]] && environment=g1-pico
    if "$conda_bin" run -n "$environment" python -c 'import sys' >/dev/null 2>&1; then
      "$conda_bin" env update -n "$environment" --file "$repo_root/envs/$spec.yml"
    else
      "$conda_bin" env create --file "$repo_root/envs/$spec.yml"
    fi
  done
fi

pybind="$repo_root/third_party/XRoboToolkit-PC-Service-Pybind"
service="$repo_root/third_party/XRoboToolkit-PC-Service/RoboticsService"
build_root="$repo_root/.build/x86_64"
mkdir -p "$build_root"
if [[ "$skip_sdk" == 0 ]]; then
  grpc_root="$service/Redistributable/linux/grpc"
  for required in include/grpcpp/grpcpp.h lib/libgrpc++.a lib/libprotobuf.a; do
    [[ -f "$grpc_root/$required" ]] || {
      printf 'Pinned native gRPC prerequisite missing: %s\n' "$grpc_root/$required" >&2; exit 1;
    }
  done
  "$conda_bin" run --no-capture-output -n g1-pico cmake \
    -S "$service/PXREARobotSDK" -B "$build_root/pico-sdk" -DCMAKE_BUILD_TYPE=Release
  "$conda_bin" run --no-capture-output -n g1-pico cmake \
    --build "$build_root/pico-sdk" --parallel "$jobs"
  # CMake produces this file directly and also installs it into SDK/linux/64.
  # Both paths are native build results, not binaries copied from the old PC.
  sdk_built="$build_root/pico-sdk/libPXREARobotSDK.so"
  [[ -f "$sdk_built" ]] || { printf 'Native SDK build did not produce %s\n' "$sdk_built" >&2; exit 1; }
  mkdir -p "$pybind/lib" "$pybind/include"
  cp -- "$sdk_built" "$pybind/lib/libPXREARobotSDK.so"
  cp -- "$service/PXREARobotSDK/PXREARobotSDK.h" "$pybind/include/"
  cp -R -- "$service/PXREARobotSDK/nlohmann" "$pybind/include/"
  pybind_cmake="$("$conda_bin" run -n g1-pico python -m pybind11 --cmakedir)"
  # Build directories are CPU/Python-specific: do not reuse another host's cache.
  CMAKE_PREFIX_PATH="$pybind_cmake${CMAKE_PREFIX_PATH:+:$CMAKE_PREFIX_PATH}" \
    "$conda_bin" run --no-capture-output -n g1-pico python -m pip install \
      --no-build-isolation --no-deps --no-cache-dir --force-reinstall "$pybind"
fi

if [[ "$build_service" == 1 ]]; then
  # Build only the service target; no desktop demos are launched. Release avoids
  # upstream's RelWithDebInfo rules copying hardcoded Qt6.6.2 filenames.
  "$conda_bin" run --no-capture-output -n g1-pico cmake \
    -S "$service" -B "$build_root/pico-service" -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_PREFIX_PATH="$qt_prefix" -DBUILD_LIB_PATH:STRING="$qt_prefix"
  "$conda_bin" run --no-capture-output -n g1-pico cmake \
    --build "$build_root/pico-service" --target RoboticsServiceProcess --parallel "$jobs"
  [[ -f "$service/bin/RoboticsServiceProcess" ]] || {
    printf 'PC Service build did not produce its executable.\n' >&2; exit 1;
  }
  # Generate a foreground launcher. This file is ignored build output, not a
  # replacement for an installed service and not an instruction to start one.
  python3 - "$build_root/launch_service.sh" "$service/bin" "$qt_prefix" <<'PY'
from pathlib import Path
import shlex,sys
launcher,service_bin,qt_prefix=map(Path,sys.argv[1:])
q=shlex.quote
launcher.write_text('#!/usr/bin/env bash\nset -euo pipefail\n'
    +'service_bin='+q(str(service_bin))+'\nqt_prefix='+q(str(qt_prefix))+'\n'
    +'export QT_QPA_PLATFORM="${QT_QPA_PLATFORM:-offscreen}"\n'
    +'export LD_LIBRARY_PATH="$service_bin:$service_bin/lib:$qt_prefix/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"\n'
    +'export QT_PLUGIN_PATH="$qt_prefix/plugins${QT_PLUGIN_PATH:+:$QT_PLUGIN_PATH}"\n'
    +'cd "$service_bin"\nexec "$service_bin/RoboticsServiceProcess"\n')
launcher.chmod(0o755)
PY
  printf 'Built PC Service. Set PICO_SERVICE_SCRIPT=%q in config/local.env.\n' "$build_root/launch_service.sh"
fi

printf 'Set PICO_SDK_LIBRARY=%q in config/local.env.\n' "$pybind/lib/libPXREARobotSDK.so"
printf 'Run offline validation:\n'
printf '  conda run -n g1-ik python scripts/doctor.py --ik\n'
printf '  conda run -n g1-pico python scripts/doctor.py --pico\n'
printf 'Setup finished. Nothing was started and no robot connection was opened.\n'
