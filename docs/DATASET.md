# Raw episode recording

The recorder captures camera and robot DDS observations on Orin and attaches
existing A/B diagnostics. It does not start teleoperation. The current format is
`g1-raw-episode-v1`.

## Record, fetch, and label

First check that both hands and the complete handover area are visible:

```bash
bash outputs/pico_native.sh record --kind commissioning --seconds 30
```

Commissioning captures download and inspect automatically. RGB is the default;
add `--depth` for lossless native Z16 depth. Capture duration is 1–300 seconds.

With A and a qualified B running, record a rehearsal in another terminal:

```bash
bash outputs/pico_native.sh record --kind rehearsal --seconds 30 \
  --task "Receive an empty cardboard box from a human"
```

Begin the task after `episode_ready`. Rehearsals defer bulk downloads to avoid
competing with control traffic. After B exits, use the printed episode directory:

```bash
bash outputs/pico_native.sh episode fetch EPISODE_DIRECTORY
bash outputs/pico_native.sh episode inspect EPISODE_DIRECTORY
bash outputs/pico_native.sh episode label EPISODE_DIRECTORY \
  --outcome success --notes "Both hands supported the box; human released it."
```

Outcomes are `success`, `failure`, `aborted`, or `commissioning`. If download
fails, preserve the local manifest and remote episode, then retry `fetch`.

## Files

| Artifact | Contents |
| --- | --- |
| `manifest.json`, `remote_manifest.json` | Task/outcome, configuration provenance, hashes, clocks, worker status |
| `rgb.mp4`, `camera_frames.jsonl` | RGB and its exact video-index/frame-number/timestamp mapping |
| `camera_metadata.json`, `depth/*.npy` | Intrinsics, serial, optional native depth, scale and extrinsics |
| `sample_start.jpg`, `sample_middle.jpg`, `sample_end.jpg` | Quick framing checks |
| `state.jsonl`, `commands.jsonl` | Measured joints/diagnostics and observed command publications |
| `pc_logs/` | Existing PICO, arm, and hand diagnostics overlapping capture |
| `robot_model.urdf`, `control_configuration.json` | Available model and control configuration |
| `frame_index.jsonl` | Associations with preceding state, commands, and hand measurements |
| `camera_result.json`, `state_result.json`, `quality.json` | Completeness counters and review requirements |

Depth is unaligned Z16: meters = stored value × depth scale. Use the recorded
intrinsics/extrinsics when aligning it. Keep requested targets, published commands,
and measured motion separate during export.

## Timing and quality

Camera/DDS receipt and individual hand-read completion use Orin monotonic time.
PICO polling and B diagnostics use workstation monotonic time. Before/after clock
exchanges retain offset bounds and uncertainty. Original device timestamps remain
available; receipt-time alignment does not establish exposure-time synchronization.

The frame index uses preceding observations, with age limits of 50 ms for state,
100 ms for command publications, and 120 ms for each hand measurement. MP4's
nominal frame rate is not the acquisition clock; use `camera_frames.jsonl`.

Quality checks flag queue overflow, frame gaps, timing discontinuities, worker
errors, and decoded-video/index count mismatches. DDS uses latest-state history,
and A/B diagnostics can drop snapshots, so this is not a complete servo-action
history. Command publication does not prove execution.

`capture_complete: true` means the implemented capture checks passed. The report
keeps `training_ready: false`; an outcome label does not override that status.
Before scaling collection, review one full rehearsal for visibility, reach/grasp
accuracy, timing, and latency under recording load. Use the stationary-base arm
workflow for the box task; suspended-leg tests remain commissioning data.

Keep recordings and local configuration out of Git. Dataset export, resampling,
splits, model training, and any release of recordings are separate steps.
