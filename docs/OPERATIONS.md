# Operations

Run from the repository root after [setup](SETUP.md). The wrapper loads
`config/local.env`. Use one motor-control owner and one hand-serial owner; stop
competing control applications before B. For arm-only control, select the robot's
usual normal holding/control mode and use its established support arrangement.

## Tracking and shadow test

Start PC Service and A in separate terminals:

```bash
bash outputs/pico_native.sh service
```

```bash
bash outputs/pico_native.sh bridge --seconds 3600
```

In PICO XRoboToolkit, connect to the workstation, enable Head/Controller and Send.
For elbow guidance or legs, pair/calibrate both ankle trackers and select Full
Body mode. Confirm source and body timestamps advance.

Start B:

```bash
bash outputs/pico_native.sh preview \
  --arm-link shadow --profile responsive --arm-transport udp \
  --hands inspire --ready-pose current --elbow-hint body \
  --no-viewer --seconds 300
```

Release both side grips and both index triggers once after connection:

| Control | Behavior |
| --- | --- |
| Hold both side grips | Follow arm movement |
| Release either side grip | Clutch/hold arms |
| Squeeze an index trigger | Curl the matching hand |
| Release an index trigger | Open the matching hand |

Triggers operate independently of the arm clutch. Each trigger couples four
fingers and thumb bend; thumb opposition stays at its measured starting value.
This is not individual-finger imitation or grasp-force control.

Check normal reaches, return motion, source gaps, IK residuals, and persistent
limits in shadow mode. End shadow B, then rerun the same command with
`--arm-link live` to enable physical control. Keep `--ready-pose current` and the
configured commissioning limits. Release triggers before ending B if hands should
finish open; input loss retains the last grasp.

## Optional suspended legs

Add `--legs suspended` to the shadow command first. This mode replaces the
whole-body controller and does not implement balance or walking. Both feet must
stay clear of the floor and support hardware after the holding pose extends them.

With grips released, lift/lower each foot once to verify both inputs. Return both
feet down, hold both grips, and test small alternating lifts. Repeat in live mode
after checking the shadow targets and physical clearance. Larger movements can
reach the current leg envelope; the body input is an estimated skeleton.

After B and its workers exit, restore normal controller ownership:

```bash
bash outputs/pico_native.sh restore
```

Ownership restoration does not select a posture. While the robot remains
supported, use the handheld to select its holding posture before lowering it.

## Stream and network checks

A is the only native SDK reader. To use a probe, stop B and A, run the probe alone,
then restart A followed by B. Restart B whenever A starts a new accepted session.
Keep the headset worn, XRoboToolkit foregrounded, and Send active during tests.

Control bootstrap uses IPv4 SSH and UDP; the configured alias needs IPv4
connectivity. The PICO stays on Wi-Fi. A wired workstation connection to the
access point's network removes one wireless hop, while an Orin Wi-Fi link remains.
Check Wi-Fi power saving after reboots if stalls return.

With control, recording, and downloads idle:

```bash
bash scripts/network_check.sh --samples 200
```

This measures SSH echo round trips, not physical motion latency. During operation,
inspect source gaps, PC processing, command transport, robot feedback age, and
physical response separately. Repeat measurements with recording active.
