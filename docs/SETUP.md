# x86 workstation setup

Use Linux **x86_64**, Conda/Miniforge, Git, Python3, a C/C++ compiler and Make.
The workstation runs CPU IK; CUDA and training frameworks are not required.
The G1's onboard Orin keeps its existing robot and camera runtime.

## Install

```bash
bash scripts/bootstrap_workstation.sh
cp config/example.env config/local.env
```

The bootstrap fetches three source revisions from `dependencies.lock.json`,
creates `g1-ik` and `g1-pico`, and builds the native PICO SDK/Python extension.
No vendor submodules, browser pipeline or robot SDK are installed on the PC.
It does not start PC Service or connect to the robot.

Use `--deps-only` to fetch sources, `--skip-envs` to reuse provisioned environments,
or `--jobs N` to change build parallelism. Environment versions are defined in
`envs/ik.yml` and `envs/pico.yml`; do not copy the old PC's Conda directories or
compiled Python extension.

## Configure

Edit `config/local.env`; it is excluded from Git.

| Setting | Value |
| --- | --- |
| `G1_HOST` | Your working SSH alias for the Orin; default `g1` |
| `G1_REMOTE_PYTHON` | Existing Orin Python containing robot dependencies |
| `G1_DDS_INTERFACE` | Robot DDS Ethernet interface |
| `INSPIRE_LEFT_PORT`, `INSPIRE_RIGHT_PORT` | Stable serial-device paths on Orin |
| `PICO_SERVICE_SCRIPT` | Native x86 PC Service launcher |
| `PICO_SDK_LIBRARY` | Native SDK library path printed by the bootstrap |

`XR_TELEOPERATE_REPO`, `IK_PYTHON` and `NATIVE_PYTHON` are optional overrides;
defaults use the pinned checkout and the two Conda environments. Configure the
new workstation's own SSH access separately. Keep SSH keys out of the repository.

## PC Service

Install the matching Ubuntu22.04/24.04 **amd64** `.deb` from the official
[PC Service v1.0.0 release](https://github.com/XR-Robotics/XRoboToolkit-PC-Service/releases/tag/v1.0.0),
or reuse an existing installation. Set:

```bash
PICO_SERVICE_SCRIPT=/opt/apps/roboticsservice/runService.sh
```

For an optional source build, provide Qt6 Core, Network and Core5Compat development
files, then run:

```bash
bash scripts/bootstrap_workstation.sh --skip-envs --build-service --qt-prefix /usr
```

Use your Qt SDK prefix if different. The script prints a foreground launcher
supporting headless operation; setup does not start it. SDK and service builds are
separate.

## Verify and deploy

```bash
conda run -n g1-ik python scripts/doctor.py --ik
conda run -n g1-pico python scripts/doctor.py --pico
bash scripts/check_orin.sh
bash scripts/deploy_orin.sh
bash scripts/deploy_orin.sh --apply
bash scripts/check_orin.sh --after-deploy
```

The workstation checks exercise model IK, collision queries and PICO binding
imports without initializing the XR SDK. Deployment prints a plan first; stop B
and recorders before applying it. Existing Orin tools are backed up.

The robot Python needs NumPy, Pinocchio, coal or hpp-fcl, Unitree SDK2 Python and
CycloneDDS. The direct hand adapter uses the Python standard library. Camera capture uses system Python with RealSense and
OpenCV/GStreamer, NVIDIA GStreamer encoding plugins and ffprobe.
`check_orin.sh` inspects the existing runtime without opening camera/serial devices
or publishing commands. Observed robot versions are recorded separately in the
source lock; workstation pins do not replace the Orin environment.

If persistent Inspire access is missing, run the deployed installer **on Orin**
with your adapter serial and robot login, replacing the placeholders:

```bash
sudo bash /home/unitree/g1_dataset/tools/install_inspire_access.sh \
  --serial ADAPTER_SERIAL --user ROBOT_USER
```

Adjust the tool path if `G1_REMOTE_ROOT` differs. The default USB interfaces are
`01` for left and `02` for right; override `--left-interface`/`--right-interface`
to match verified wiring. The installer creates `/dev/g1_inspire_left` and
`/dev/g1_inspire_right` without opening the hands or issuing movement commands.

Continue with [OPERATIONS.md](OPERATIONS.md) for the Ethernet/network check and
terminal A/B launch sequence.
