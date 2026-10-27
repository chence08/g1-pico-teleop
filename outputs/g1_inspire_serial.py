"""Small DFX serial adapter using Unitree's published Inspire register protocol.

Constructing/opening never commands a position. probe() only reads angles.
Exclusive locks prevent this adapter sharing a port with another instance.
"""
import argparse
import fcntl
import json
import math
import os
import select
import struct
import termios
import time

from g1_config import LEFT_PORT, RIGHT_PORT

def frame(opcode, address, data, device_id=1):
    body=bytes([device_id,len(data)+3,opcode])+struct.pack('<H',address)+data
    return b'\xeb\x90'+body+bytes([sum(body)&255])


def parse_frame(data, opcode, address, length, device_id=1):
    if (len(data)!=length+8 or data[:2]!=b'\x90\xeb' or data[2]!=device_id
            or data[3]!=length+3 or data[4]!=opcode
            or data[5:7]!=struct.pack('<H',address)
            or sum(data[2:-1])&255!=data[-1]):
        raise ValueError('Invalid Inspire reply framing/address/checksum')
    return data[7:-1]


class InspireSerial:
    def __init__(self, port):
        self.port=port
        self.fd=os.open(port,os.O_RDWR|os.O_NOCTTY|os.O_NONBLOCK)
        try:
            fcntl.flock(self.fd,fcntl.LOCK_EX|fcntl.LOCK_NB)
            # Prevent other ordinary serial opens after this owner starts.
            fcntl.ioctl(self.fd,termios.TIOCEXCL)
            attrs=termios.tcgetattr(self.fd)
            attrs[0]=attrs[1]=attrs[3]=0
            attrs[2]=termios.CS8|termios.CREAD|termios.CLOCAL
            attrs[4]=attrs[5]=termios.B115200
            attrs[6][termios.VMIN]=0;attrs[6][termios.VTIME]=0
            termios.tcsetattr(self.fd,termios.TCSANOW,attrs)
            termios.tcflush(self.fd,termios.TCIOFLUSH)
        except BaseException:
            os.close(self.fd);raise

    def close(self):
        if self.fd is not None:
            try: fcntl.ioctl(self.fd,termios.TIOCNXCL)
            finally: os.close(self.fd);self.fd=None

    def transaction(self, opcode, address, data, reply_length, timeout=.12):
        termios.tcflush(self.fd,termios.TCIFLUSH)
        packet=frame(opcode,address,data)
        if os.write(self.fd,packet)!=len(packet):
            raise OSError('Partial Inspire request')
        deadline=time.monotonic()+timeout;buffer=b''
        while time.monotonic()<deadline:
            if select.select([self.fd],[],[],max(0,deadline-time.monotonic()))[0]:
                buffer+=os.read(self.fd,128)
                start=buffer.find(b'\x90\xeb')
                if start>=0:buffer=buffer[start:]
                if len(buffer)>=reply_length+8:
                    return parse_frame(buffer[:reply_length+8],opcode,address,reply_length)
        raise TimeoutError(f'No valid Inspire reply from {self.port}')

    def read_angles(self):
        data=self.transaction(0x11,0x060A,b'\x0c',12)
        values=struct.unpack('<6H',data)
        if any(x>1000 for x in values):raise ValueError('Inspire angle outside 0..1000')
        return [x/1000 for x in values]

    def write_angles(self, q):
        if len(q)!=6 or any(type(x) not in (int,float) or not math.isfinite(x) or not 0<=x<=1 for x in q):
            raise ValueError('Expected six finite normalized angles')
        data=struct.pack('<6H',*[round(x*1000) for x in q])
        ack=self.transaction(0x12,0x05CE,data,1)
        if ack!=b'\x01':raise RuntimeError(f'Inspire rejected position write: {ack.hex()}')


def probe(ports):
    result=[]
    for port in ports:
        hand=None
        try:
            hand=InspireSerial(port)
            samples=[hand.read_angles() for _ in range(3)]
            result.append(dict(port=port,read_only=True,angles=samples))
        except Exception as exc:
            result.append(dict(port=port,read_only=True,error=str(exc)))
        finally:
            if hand is not None:hand.close()
    return result


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--ports',nargs='+',default=[LEFT_PORT,RIGHT_PORT])
    args=p.parse_args()
    print(json.dumps(probe(args.ports),indent=2))
