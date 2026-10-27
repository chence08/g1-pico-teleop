"""Authenticated <=1200-byte latest-state transport. No sockets or hardware."""
import hashlib
import hmac
import json
import math
import struct
import zlib

MAX_DATAGRAM = 1200
_HEADER = struct.Struct('!4sBBH16sQ')
_TAG_SIZE = 16
_MAGIC, _VERSION = b'G1DG', 2
_COMMAND, _STATE = 1, 2
_MAX_TEXT = 4096
STATIC_FIELDS = (
    'initial_q', 'operating_lower_rad', 'operating_upper_rad', 'joint_range_rad',
    'profile', 'speed_limit_rad_s', 'acceleration_limit_rad_s2',
    'tracking_lead_limit_rad', 'gravity_comp', 'gravity_calibration',
    'control_scope','leg_initial_q','leg_lower_rad','leg_upper_rad',
)
_ARRAYS = (
    ('q', 14), ('motor_q29', 29), ('q_command', 14), ('q_target', 14),
    ('dq_command', 14), ('dq', 14), ('tau_measured', 14), ('tau_command', 14),
    ('workspace_proposed_clearance_m', 2),
    ('leg_command_q',12),('leg_target_q',12),('leg_command_dq',12),
)
_FLOATS = (
    'state_age_s', 'mode_last_success_age_s', 'weight', 'server_monotonic_s',
    'loop_max_gap_s', 'state_capture_received_s', 'state_delivery_delay_s',
    'pc_sampled_s', 'pc_ready_s', 'pc_sent_s', 'ik_ms',
    'robot_input_received_s', 'robot_accepted_s', 'pc_input_to_send_ms',
    'pc_ready_to_send_ms', 'robot_first_published_s', 'robot_receive_to_publish_ms',
    'robot_challenge_age_s', 'robot_input_age_upper_s',
    'body_input_age_s',
)
_SIGNED = ('fsm', 'mode_machine', 'mode_query_code')
_UNSIGNED = (
    'accepted', 'rejected', 'link_pings', 'state_sequence', 'robot_tick',
    'state_source_timestamp_ns', 'state_ipc_drops', 'command_id', 'source_sequence',
    'body_sequence',
)
_BOOLS = (
    'mode_query_ok', 'velocity_reference', 'takeover_active', 'live', 'fault',
    'walking_client_connected',
)
_BITS = (('tracking_limited', 14), ('workspace_blocked', 2), ('gravity_saturated', 14),('leg_tracking_limited',12))
_TEXT = ('challenge', 'status', 'fault_reasons')
_SPECS = (
    [(n, 'array', k, str(k)+'f') for n, k in _ARRAYS]
    + [(n, 'float', 1, 'd') for n in _FLOATS]
    + [(n, 'int', 1, 'q') for n in _SIGNED]
    + [(n, 'uint', 1, 'Q') for n in _UNSIGNED]
    + [(n, 'bool', 1, 'B') for n in _BOOLS]
    + [(n, 'bits', k, 'H') for n, k in _BITS]
)
_FIXED = struct.Struct('!QQ' + ''.join(s[3] for s in _SPECS))
_FIELDS = {s[0] for s in _SPECS} | set(_TEXT)
_VALID_MASK = (1 << len(_SPECS)) - 1


def _key(key):
    if not isinstance(key, bytes) or len(key) != 32:
        raise ValueError('session key must be exactly 32 bytes')
    return key


def _session(key):
    return hmac.new(key, b'G1DG session v1', hashlib.sha256).digest()[:16]


def _seal(kind, payload, key, seq):
    _key(key)
    if type(seq) is not int or not 0 <= seq < 2**64:
        raise ValueError('invalid transport sequence')
    if len(payload) > MAX_DATAGRAM - _HEADER.size - _TAG_SIZE:
        raise ValueError('datagram exceeds 1200 bytes')
    head = _HEADER.pack(_MAGIC, _VERSION, kind, len(payload), _session(key), seq)
    body = head + payload
    return body + hmac.new(key, body, hashlib.sha256).digest()[:_TAG_SIZE]


def _open(packet, key, kind, last_seq):
    _key(key)
    if not isinstance(packet, bytes) or not _HEADER.size+_TAG_SIZE <= len(packet) <= MAX_DATAGRAM:
        raise ValueError('invalid datagram size')
    magic, version, actual_kind, length, session, seq = _HEADER.unpack_from(packet)
    if (magic, version, actual_kind) != (_MAGIC, _VERSION, kind):
        raise ValueError('wrong protocol, version, or packet kind')
    if length != len(packet) - _HEADER.size - _TAG_SIZE or session != _session(key):
        raise ValueError('wrong length or session')
    expected = hmac.new(key, packet[:-_TAG_SIZE], hashlib.sha256).digest()[:_TAG_SIZE]
    if not hmac.compare_digest(packet[-_TAG_SIZE:], expected):
        raise ValueError('authentication failed')
    if last_seq is not None and seq <= last_seq:
        raise ValueError('replayed or reordered datagram')
    return seq, packet[_HEADER.size:-_TAG_SIZE]


def _reject_constant(value):
    raise ValueError('nonfinite JSON constant')


def _json_loads(payload):
    return json.loads(payload.decode('utf-8'), parse_constant=_reject_constant)


def split_state(row):
    return ({k: row[k] for k in STATIC_FIELDS if k in row},
            {k: row[k] for k in _FIELDS if k in row})


def _finite(value):
    if type(value) not in (int, float) or not math.isfinite(value):
        raise ValueError('invalid finite real value')
    return value


def _text_check(text):
    if not isinstance(text, dict) or set(text) - set(_TEXT):
        raise ValueError('invalid state text')
    if 'challenge' in text:
        value = text['challenge']
        if not isinstance(value, str) or len(value) != 24:
            raise ValueError('invalid robot challenge')
        if len(bytes.fromhex(value)) != 12:
            raise ValueError('invalid robot challenge')
    if 'status' in text and not isinstance(text['status'], str):
        raise ValueError('invalid status')
    if 'fault_reasons' in text and (not isinstance(text['fault_reasons'], list)
            or not all(isinstance(s, str) for s in text['fault_reasons'])):
        raise ValueError('invalid fault reasons')
    return text


def encode_state(row, key, seq):
    if row.get('state_sequence') != seq:
        raise ValueError('state sequence mismatch')
    present = null = 0
    values = []
    for i, (name, kind, count, fmt) in enumerate(_SPECS):
        value = row.get(name)
        if name in row:
            present |= 1 << i
            if value is None:
                null |= 1 << i
        if value is None:
            values.extend([0] * (count if kind == 'array' else 1))
        elif kind == 'array':
            if not isinstance(value, (list, tuple)) or len(value) != count:
                raise ValueError('invalid ' + name)
            values.extend(_finite(v) for v in value)
        elif kind == 'float':
            values.append(_finite(value))
        elif kind in ('int', 'uint'):
            if type(value) is not int:
                raise ValueError('invalid integer ' + name)
            values.append(value)
        elif kind == 'bool':
            if type(value) is not bool:
                raise ValueError('invalid boolean ' + name)
            values.append(int(value))
        else:
            if (not isinstance(value, (list, tuple)) or len(value) != count
                    or not all(type(v) is bool for v in value)):
                raise ValueError('invalid bitset ' + name)
            values.append(sum(int(v) << j for j, v in enumerate(value)))
    text = _text_check({k: row[k] for k in _TEXT if k in row})
    raw = json.dumps(text, separators=(',', ':'), allow_nan=False).encode()
    if len(raw) > _MAX_TEXT:
        raise ValueError('state text exceeds bound')
    try:
        payload = _FIXED.pack(present, null, *values) + zlib.compress(raw, 1)
    except (struct.error, OverflowError) as exc:
        raise ValueError('state value out of wire range') from exc
    return _seal(_STATE, payload, key, seq)


def decode_state(packet, key, metadata=None, last_seq=None):
    seq, payload = _open(packet, key, _STATE, last_seq)
    if len(payload) <= _FIXED.size:
        raise ValueError('truncated state')
    values = _FIXED.unpack_from(payload)
    present, null = values[:2]
    if (present | null) & ~_VALID_MASK or null & ~present:
        raise ValueError('invalid state presence masks')
    row = {k: metadata[k] for k in STATIC_FIELDS if metadata is not None and k in metadata}
    pos = 2
    for i, (name, kind, count, fmt) in enumerate(_SPECS):
        width = count if kind == 'array' else 1
        value = values[pos:pos+width]
        pos += width
        if not present & (1 << i):
            continue
        if null & (1 << i):
            row[name] = None
        elif kind == 'array':
            row[name] = [_finite(float(v)) for v in value]
        elif kind == 'float':
            row[name] = _finite(value[0])
        elif kind == 'bool':
            if value[0] not in (0, 1):
                raise ValueError('invalid boolean')
            row[name] = bool(value[0])
        elif kind == 'bits':
            if value[0] >= 1 << count:
                raise ValueError('invalid bitset')
            row[name] = [bool(value[0] & (1 << j)) for j in range(count)]
        else:
            row[name] = value[0]
    try:
        decoder = zlib.decompressobj()
        raw = decoder.decompress(payload[_FIXED.size:], _MAX_TEXT+1)
        if (len(raw) > _MAX_TEXT or decoder.unconsumed_tail or decoder.unused_data
                or not decoder.eof):
            raise ValueError('invalid or oversized state text')
        row.update(_text_check(_json_loads(raw)))
    except (zlib.error, UnicodeError, json.JSONDecodeError) as exc:
        raise ValueError('invalid state text') from exc
    if row.get('state_sequence') != seq:
        raise ValueError('state sequence mismatch')
    row['type'] = 'arm_state'
    return seq, row


def encode_command(row, key, seq):
    if not isinstance(row, dict) or type(row.get('seq')) is not int or row['seq'] != seq:
        raise ValueError('command sequence mismatch')
    try:
        payload = json.dumps(row, separators=(',', ':'), allow_nan=False).encode()
    except (ValueError, TypeError) as exc:
        raise ValueError('invalid command JSON') from exc
    return _seal(_COMMAND, payload, key, seq)


def decode_command(packet, key, last_seq=None):
    seq, payload = _open(packet, key, _COMMAND, last_seq)
    try:
        row = _json_loads(payload)
    except (UnicodeError, json.JSONDecodeError) as exc:
        raise ValueError('invalid command JSON') from exc
    if not isinstance(row, dict) or type(row.get('seq')) is not int or row['seq'] != seq:
        raise ValueError('command sequence mismatch')
    return seq, row
