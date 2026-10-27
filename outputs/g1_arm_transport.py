"""Ephemeral, authenticated latest-only arm datagrams bootstrapped over SSH.

No SDK or motor access. The SSH process lifetime and robot-side watchdog remain
independent of packet delivery. Invalid/replayed datagrams never refresh input.
"""
import ipaddress
import json
import os
import select
import socket
import threading
import time

from g1_arm_datagram import (split_state, encode_state, decode_state,
                             encode_command, decode_command, MAX_DATAGRAM as MAX_PACKET)


def datagram_socket():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 8192)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 8192)
    sock.setblocking(False)
    return sock


class ServerDatagrams:
    def __init__(self, output):
        # Pin the interface and peer IP to the already authenticated SSH link.
        connection = os.environ.get('SSH_CONNECTION', '').split()
        if len(connection) != 4:
            raise RuntimeError('UDP arm transport requires an SSH bootstrap')
        self.peer_ip = str(ipaddress.IPv4Address(connection[0]))
        self.host = str(ipaddress.IPv4Address(connection[2]))
        self.socket = datagram_socket()
        self.socket.bind((self.host, 0))
        self.output = output
        self.key = os.urandom(32)
        self.peer = None
        self.bootstrapped = False
        self.last_seq = -1
        self.discarded = 0
        self.send_drops = 0

    def read(self, incoming, ending):
        while not ending.is_set():
            try:
                if not select.select([self.socket], [], [], .05)[0]:
                    continue
                # Bounded drain: newer sequence wins, regardless of arrival order.
                for _ in range(64):
                    try:
                        data, peer = self.socket.recvfrom(MAX_PACKET + 1)
                    except BlockingIOError:
                        break
                    received = time.monotonic()
                    if peer[0] != self.peer_ip or (self.peer is not None and peer != self.peer):
                        self.discarded += 1
                        continue
                    try:
                        seq, row = decode_command(data, self.key, last_seq=self.last_seq)
                    except (ValueError, TypeError, OverflowError):
                        self.discarded += 1
                        continue
                    self.last_seq = seq
                    self.peer = peer
                    incoming.append((row, received))
            except (OSError, ValueError):
                return  # Existing robot input expiry handles socket failure.

    def send(self, row):
        if not self.bootstrapped:
            metadata, _ = split_state(row)
            # This line is private SSH protocol, never a console diagnostic.
            self.output.write(json.dumps(dict(type='arm_udp_bootstrap', protocol=1,
                host=self.host, port=self.socket.getsockname()[1],
                key=self.key.hex(), metadata=metadata)) + '\n')
            self.output.flush()
            self.bootstrapped = True
        if self.peer is not None:
            data = encode_state(row, self.key, row['state_sequence'])
            try:
                self.socket.sendto(data, self.peer)
            except (BlockingIOError, OSError):
                self.send_drops += 1

    def close(self):
        self.socket.close()


class ClientDatagrams:
    def __init__(self, bootstrap):
        if bootstrap.get('protocol') != 1:
            raise ValueError('Unsupported arm datagram protocol')
        host = str(ipaddress.IPv4Address(bootstrap['host']))
        port = bootstrap['port']
        if type(port) is not int or not 1 <= port <= 65535:
            raise ValueError('Invalid arm datagram port')
        self.key = bytes.fromhex(bootstrap['key'])
        if len(self.key) != 32 or not isinstance(bootstrap['metadata'], dict):
            raise ValueError('Invalid arm datagram bootstrap')
        self.metadata = bootstrap['metadata']
        self.socket = datagram_socket()
        self.socket.connect((host, port))
        self.last_seq = -1
        self.discarded = 0
        self.send_drops = 0
        self.missing = 0
        self.last_packet_bytes = 0

    def read(self, callback, ending):
        while not ending.is_set():
            try:
                if not select.select([self.socket], [], [], .05)[0]:
                    continue
                latest = None
                for _ in range(64):
                    try:
                        data = self.socket.recv(MAX_PACKET + 1)
                    except BlockingIOError:
                        break
                    received = time.monotonic()
                    try:
                        seq, row = decode_state(data, self.key, self.metadata,
                                                last_seq=self.last_seq)
                    except (ValueError, TypeError, OverflowError):
                        self.discarded += 1
                        continue
                    if self.last_seq >= 0:
                        self.missing += max(0, seq - self.last_seq - 1)
                    self.last_seq = seq
                    self.last_packet_bytes = len(data)
                    row.update(arm_transport='udp', transport_missing=self.missing,
                               transport_discarded=self.discarded,
                               transport_packet_bytes=len(data))
                    latest = row, received
                if latest is not None:
                    callback(*latest)
            except (OSError, ValueError):
                return  # PC feedback expiry handles socket failure.

    def send(self, row):
        data = encode_command(row, self.key, row['seq'])
        try:
            self.socket.send(data)
        except (BlockingIOError, OSError):
            self.send_drops += 1

    def close(self):
        self.socket.close()
