# G1 PICO teleoperation and recording

Teleoperate a Unitree G1 EDU 29-DOF with Inspire hands using a consumer PICO 4
Ultra and XRoboToolkit. The target task is receiving a cardboard box from a person.

An **x86_64 Linux workstation** receives tracking and runs IK. The robot's **Orin**
runs motor/hand interfaces and RealSense/DDS recording.

Arms, trigger-controlled hands, and suspended leg following have been demonstrated.
Retargeting remains experimental, including some awkward shoulder/elbow poses.
The recorder keeps `training_ready: false`; no completed dataset or trained policy
is included.

## Setup

Follow [SETUP](docs/SETUP.md) to install pinned dependencies, configure your SSH
alias and device paths, and deploy to Orin. Local configuration belongs in
`config/local.env`; generated files and recordings belong in ignored `work/`.

## Run

Use separate terminals for PC Service, A, and B:

```bash
# PC Service
bash outputs/pico_native.sh service

# Terminal A: native tracking bridge
bash outputs/pico_native.sh bridge --seconds 3600

# Terminal B: shadow test, without motor commands
bash outputs/pico_native.sh preview \
  --arm-link shadow --profile responsive --arm-transport udp \
  --hands inspire --ready-pose current --elbow-hint body \
  --no-viewer --seconds 300
```

Connect XRoboToolkit on the headset and enable Send before using B. A must be the
only native SDK reader. See [OPERATIONS](docs/OPERATIONS.md) for grip/trigger
controls, live startup, optional suspended legs, and shutdown.

For camera/state recording, deferred downloads, quality checks, and outcome
labels, follow [DATASET](docs/DATASET.md).

Source is in `outputs/`, setup tools in `scripts/`, and offline tests in `tests/`.
Third-party revisions are pinned in `dependencies.lock.json`. Licensing is pending;
see [NOTICE.md](NOTICE.md).
