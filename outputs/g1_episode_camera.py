"""Recording-only RealSense worker; hardware imports are lazy, no robot commands."""
import argparse
from dataclasses import dataclass
from datetime import datetime, timezone
import json
import math
import os
from pathlib import Path
import queue
import signal
import subprocess
import threading
import time


def write_json(path, value):
    temporary = path.with_name(path.name + '.tmp')
    with temporary.open('w', encoding='utf-8') as stream:
        json.dump(value, stream, indent=2, allow_nan=False)
        stream.write('\n'); stream.flush(); os.fsync(stream.fileno())
    temporary.replace(path)


class FrameStats:
    def __init__(self):
        self.count = 0
        self.first = self.last = self.number = self.timestamp = self.domain = None
        self.missing = self.nonincreasing = self.timestamp_regressions = 0
        self.domain_changes = self.max_gap_ns = 0
    def observe(self, frame):
        receipt = frame['host_receipt_monotonic_ns']
        number, stamp, domain = (frame[k] for k in ('frame_number', 'device_timestamp_ms', 'timestamp_domain'))
        if not math.isfinite(stamp): raise ValueError('Nonfinite camera device timestamp')
        if self.count:
            self.max_gap_ns = max(self.max_gap_ns, receipt - self.last)
            self.missing += max(0, number - self.number - 1)
            self.nonincreasing += int(number <= self.number)
            self.timestamp_regressions += int(domain == self.domain and stamp <= self.timestamp)
            self.domain_changes += int(domain != self.domain)
        else: self.first = receipt
        self.count += 1
        self.last, self.number, self.timestamp, self.domain = receipt, number, stamp, domain
    def summary(self):
        duration = (self.last - self.first) / 1e9 if self.count > 1 else 0
        return dict(frames_received=self.count, first_receipt_mono_ns=self.first,last_receipt_mono_ns=self.last,
                    observed_receipt_fps=(self.count - 1) / duration if duration > 0 else None,
                    max_receipt_gap_ms=self.max_gap_ns / 1e6,frame_number_gaps=self.missing,
                    nonincreasing_frame_numbers=self.nonincreasing,nonincreasing_device_timestamps=self.timestamp_regressions,
                    timestamp_domain_changes=self.domain_changes)


def verify_video(path, expected_frames, runner=subprocess.run):
    proc = runner(['ffprobe', '-v', 'error', '-select_streams', 'v:0', '-count_frames',
                   '-show_entries', 'stream=nb_read_frames,width,height', '-of', 'json', str(path)],
                  capture_output=True, text=True, timeout=60, check=True)
    if proc.stderr.strip(): raise RuntimeError('Video decode errors: ' + proc.stderr.strip()[:400])
    streams = json.loads(proc.stdout)['streams']
    if len(streams) != 1: raise RuntimeError('Expected one RGB video stream')
    stream = streams[0]; decoded = int(stream['nb_read_frames'])
    if decoded != expected_frames or (stream['width'], stream['height']) != (1280, 720):
        raise RuntimeError(f'RGB verification failed: decoded={decoded}, indexed={expected_frames}')
    return dict(decoded_frames=decoded, width=stream['width'], height=stream['height'])


def quality_issues(result):
    issues = list(result.get('errors', []))
    if result['frames_written'] < 2: issues.append('Fewer than two RGB frames written')
    if result['queue_drops']: issues.append('Camera writer queue overflowed')
    if result['capture_timeouts']: issues.append('Camera acquisition timed out')
    for name in ('rgb_capture', 'depth_capture'):
        stream = result.get(name)
        if stream and any(stream[k] for k in ('frame_number_gaps', 'nonincreasing_frame_numbers','nonincreasing_device_timestamps', 'timestamp_domain_changes')):
            issues.append(name + ' has missing, repeated, or discontinuous source timing')
    if not result.get('video_verification'): issues.append('Decoded RGB frame count was not verified')
    return issues


@dataclass
class CapturedFrame:
    rgb: object
    depth: object
    row: dict


class CameraWriter:
    def __init__(self, output, seconds, with_depth, cv2, np, queue_size=8):
        self.output, self.seconds, self.with_depth = output, seconds, with_depth
        self.cv2, self.np = cv2, np
        self.queue = queue.Queue(maxsize=queue_size)
        self.capture_done = threading.Event(); self.ready = threading.Event(); self.errors = []
        self.written = self.dropped = 0; self.first = self.last = None; self.max_encode_ms = 0
        self.thread = threading.Thread(target=self._run, name='camera-writer', daemon=True)
    def submit(self, frame):
        try: self.queue.put_nowait(frame); return True
        except queue.Full: self.dropped += 1; return False
    def _image(self, name, image):
        if not self.cv2.imwrite(str(self.output / name), image): raise RuntimeError('Failed to save ' + name)
    def _run(self):
        video = None
        try:
            gst = ('appsrc ! video/x-raw,format=BGR,width=1280,height=720,framerate=30/1 '
                   '! videoconvert ! video/x-raw,format=BGRx ! nvvidconv '
                   '! video/x-raw(memory:NVMM),format=NV12 '
                   '! nvv4l2h264enc bitrate=8000000 insert-sps-pps=true iframeinterval=30 '
                   '! h264parse ! mp4mux ! filesink location=' + json.dumps(str(self.output / 'rgb.mp4')))
            video = self.cv2.VideoWriter(gst, self.cv2.CAP_GSTREAMER, 0, 30, (1280, 720))
            if not video.isOpened(): raise RuntimeError('NVIDIA H.264 writer could not open')
            if self.with_depth: (self.output / 'depth').mkdir()
            with (self.output / 'camera_frames.jsonl').open('x', encoding='utf-8') as index:
                self.ready.set(); final = None; middle_saved = False
                while not self.capture_done.is_set() or not self.queue.empty():
                    try: frame = self.queue.get(timeout=.1)
                    except queue.Empty: continue
                    received = frame.row['host_receipt_monotonic_ns']
                    if self.first is None: self.first = received
                    row = dict(frame.row, video_index=self.written)
                    start = time.monotonic_ns(); video.write(frame.rgb)
                    row['encode_call_ms'] = (time.monotonic_ns() - start) / 1e6
                    self.max_encode_ms = max(self.max_encode_ms, row['encode_call_ms'])
                    if frame.depth is not None:
                        relative = f'depth/{self.written:06d}.npy'
                        with (self.output / relative).open('xb') as depth_file: self.np.save(depth_file, frame.depth, allow_pickle=False)
                        row['depth_file'] = relative
                    if self.written == 0: self._image('sample_start.jpg', frame.rgb)
                    if not middle_saved and (received - self.first) / 1e9 >= self.seconds / 2:
                        self._image('sample_middle.jpg', frame.rgb); middle_saved = True
                    final = frame.rgb; index.write(json.dumps(row, allow_nan=False) + '\n')
                    self.written += 1; self.last = received
                    if self.written % 30 == 0: index.flush()
                if final is not None:
                    self._image('sample_end.jpg', final)
                    if not middle_saved: self._image('sample_middle.jpg', final)
                index.flush(); os.fsync(index.fileno())
        except Exception as exc: self.errors.append(f'{type(exc).__name__}: {exc}')
        finally:
            if video is not None:
                try: video.release()
                except Exception as exc: self.errors.append(f'Encoder finalization failed: {exc}')
            self.ready.set()


def intrinsics(profile):
    value = profile.as_video_stream_profile().get_intrinsics()
    return dict(width=value.width, height=value.height, fx=value.fx, fy=value.fy,
                ppx=value.ppx, ppy=value.ppy, distortion=str(value.model), coefficients=list(value.coeffs))


def frame_metadata(frame, mono_ns, wall_ns):
    return dict(frame_number=frame.get_frame_number(), device_timestamp_ms=frame.get_timestamp(),
                timestamp_domain=str(frame.get_frame_timestamp_domain()),
                host_receipt_monotonic_ns=mono_ns, host_receipt_wall_ns=wall_ns)


def record(output, seconds, with_depth=False):
    if not output.is_dir(): raise ValueError('--output must be an existing episode directory')
    if any((output / name).exists() for name in ('camera_result.json', 'camera_ready.json', 'camera_frames.jsonl', 'rgb.mp4', 'depth', 'camera_metadata.json', 'sample_start.jpg', 'sample_middle.jpg', 'sample_end.jpg')):
        raise ValueError('Refusing to overwrite camera artifacts in this episode')
    lock = output / '.camera.lock'
    with lock.open('x') as stream: stream.write(str(os.getpid()))
    result = dict(schema=1, status='starting', complete=False, errors=[],seconds_requested=seconds,
                  depth_enabled=with_depth, frames_written=0,queue_drops=0, capture_timeouts=0,
                  first_receipt_mono_ns=None,last_receipt_mono_ns=None, video_verification=None)
    result['repeated_framesets_ignored']=0
    stop = threading.Event(); old_handlers = {}; writer = pipeline = None; pipeline_started = False
    rgb_stats, depth_stats = FrameStats(), FrameStats()
    try:
        write_json(output / 'camera_result.json', result)
        for signum in (signal.SIGTERM, signal.SIGINT): old_handlers[signum] = signal.signal(signum, lambda *_: stop.set())
        if (output / 'STOP').exists(): result['stop_reason'] = 'STOP before camera startup'; return result
        import cv2
        import numpy as np
        import pyrealsense2 as rs
        devices = [d for d in rs.context().query_devices() if 'D435' in d.get_info(rs.camera_info.name)]
        if len(devices) != 1: raise RuntimeError(f'Expected exactly one D435-family camera, found {len(devices)}')
        device = devices[0]; config = rs.config(); config.enable_device(device.get_info(rs.camera_info.serial_number))
        config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
        if with_depth: config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        pipeline = rs.pipeline(); profile = pipeline.start(config); pipeline_started = True
        color_profile = profile.get_stream(rs.stream.color)
        metadata = dict(schema=1, started_utc=datetime.now(timezone.utc).isoformat(),device=device.get_info(rs.camera_info.name),
                        serial_number=device.get_info(rs.camera_info.serial_number),firmware=device.get_info(rs.camera_info.firmware_version),
                        rgb=dict(format='BGR8', requested_fps=30, intrinsics=intrinsics(color_profile)),depth=None,
                        encoder='NVIDIA H.264', queue_capacity_frames=8,
                        timing_note='Host receipt sampled immediately after frameset arrival, before copying/encoding. Preserve original device timestamp domains. No exposure-time clock mapping is established.',
                        video_note='Nominal constant30fps MP4; camera_frames.jsonl maps decoded video_index to actual receipt times.',
                        depth_note='Optional native Z16 arrays are lossless, unaligned; use stored intrinsics, extrinsics and scale.',
                        sample_note='sample_middle is near requested half-duration; an early stop falls back to last frame.')
        if with_depth:
            depth_profile = profile.get_stream(rs.stream.depth); extrinsics = depth_profile.get_extrinsics_to(color_profile)
            metadata['depth'] = dict(format='Z16', requested_fps=30, intrinsics=intrinsics(depth_profile),
                                     scale_m_per_unit=device.first_depth_sensor().get_depth_scale(),
                                     extrinsics_to_rgb=dict(rotation=list(extrinsics.rotation),translation_m=list(extrinsics.translation)))
        write_json(output / 'camera_metadata.json', metadata)
        for _ in range(30):
            if stop.is_set() or (output / 'STOP').exists(): result['stop_reason'] = 'stop during camera warmup'; return result
            pipeline.wait_for_frames(1000)
        writer = CameraWriter(output, seconds, with_depth, cv2, np); writer.thread.start()
        if not writer.ready.wait(15) or writer.errors: raise RuntimeError('Camera writer startup failed: ' + repr(writer.errors))
        start = time.monotonic()
        ready = dict(schema=1, pid=os.getpid(), ready_monotonic_ns=time.monotonic_ns(),ready_wall_ns=time.time_ns(),
                     seconds=seconds, depth_enabled=with_depth,serial_number=metadata['serial_number'])
        write_json(output / 'camera_ready.json', ready); result['status'] = 'recording'; write_json(output / 'camera_result.json', result)
        print(json.dumps(dict(type='camera_ready', **ready)), flush=True)
        while time.monotonic() - start < seconds:
            if stop.is_set() or (output / 'STOP').exists(): result['stop_reason'] = 'signal' if stop.is_set() else 'STOP file'; break
            if writer.errors or not writer.thread.is_alive(): raise RuntimeError('Camera writer stopped during capture: ' + repr(writer.errors))
            try: frameset = pipeline.wait_for_frames(500)
            except RuntimeError as exc:
                result['capture_timeouts'] += 1; raise RuntimeError('Camera acquisition failed: ' + str(exc)) from exc
            mono_ns, wall_ns = time.monotonic_ns(), time.time_ns()
            rgb = frameset.get_color_frame(); depth = frameset.get_depth_frame() if with_depth else None
            if not rgb or (with_depth and not depth): raise RuntimeError('Frameset is missing a required RGB/depth frame')
            if rgb_stats.count and rgb.get_frame_number()==rgb_stats.number and (not with_depth or depth.get_frame_number()==depth_stats.number):
                result['repeated_framesets_ignored']+=1
                continue
            row = frame_metadata(rgb, mono_ns, wall_ns); row['capture_index'] = rgb_stats.count; rgb_stats.observe(row)
            rgb_data = np.asanyarray(rgb.get_data()).copy()
            if rgb_data.shape != (720, 1280, 3) or rgb_data.dtype != np.uint8: raise RuntimeError('Unexpected RGB shape or dtype')
            depth_data = None
            if with_depth:
                row['depth'] = frame_metadata(depth, mono_ns, wall_ns); depth_stats.observe(row['depth'])
                depth_data = np.asanyarray(depth.get_data()).copy()
                if depth_data.shape != (480, 640) or depth_data.dtype != np.uint16: raise RuntimeError('Unexpected depth shape or dtype')
            writer.submit(CapturedFrame(rgb_data, depth_data, row))
        result.setdefault('stop_reason', 'duration reached')
    except Exception as exc: result['errors'].append(f'{type(exc).__name__}: {exc}')
    finally:
        if pipeline_started:
            try: pipeline.stop()
            except Exception as exc: result['errors'].append('Camera stop failed: ' + str(exc))
        if writer:
            writer.capture_done.set(); writer.thread.join(timeout=30)
            if writer.thread.is_alive(): result['errors'].append('Camera writer failed to finish within30seconds')
            result['errors'].extend(writer.errors)
            result.update(frames_written=writer.written, queue_drops=writer.dropped,first_receipt_mono_ns=writer.first,
                          last_receipt_mono_ns=writer.last,max_encode_call_ms=writer.max_encode_ms)
            if not writer.thread.is_alive() and writer.written:
                try: result['video_verification'] = verify_video(output / 'rgb.mp4', writer.written)
                except Exception as exc: result['errors'].append('Video validation failed: ' + str(exc))
        result['rgb_capture'] = rgb_stats.summary(); result['depth_capture'] = depth_stats.summary() if with_depth else None
        result['quality_issues'] = quality_issues(result); result['complete'] = not result['quality_issues']
        result['status'] = 'complete' if result['complete'] else 'incomplete'; result['finished_wall_ns'] = time.time_ns()
        try: write_json(output / 'camera_result.json', result)
        finally:
            for signum, handler in old_handlers.items(): signal.signal(signum, handler)
            lock.unlink(missing_ok=True)
        print(json.dumps(dict(type='camera_result', **result)), flush=True)
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True, help='Existing, fresh episode directory')
    parser.add_argument('--seconds', type=float, default=60); parser.add_argument('--depth', action='store_true'); args = parser.parse_args()
    if not math.isfinite(args.seconds) or not 1 <= args.seconds <= 300: parser.error('Use1<=seconds<=300')
    result = record(args.output, args.seconds, args.depth)
    return 0 if result['complete'] else 1


if __name__ == '__main__': raise SystemExit(main())
