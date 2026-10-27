"""Read-only command observation in a separate process; no motor publisher."""
import os,select,socket,struct,subprocess,sys,threading,time
from collections import deque

PACKET=struct.Struct('!BQdI')  # kind, source timestamp / IPC drops, worker receipt, CRC


def command_capture(interface,fd):
    from unitree_sdk2py.core.channel import ChannelFactoryInitialize
    from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_
    from cyclonedds.domain import DomainParticipant
    from cyclonedds.topic import Topic
    from cyclonedds.sub import DataReader
    from cyclonedds.qos import Qos,Policy
    from cyclonedds.internal import InvalidSample
    ChannelFactoryInitialize(0,interface)
    participant=DomainParticipant(0)
    # Preserve batches of competing writer events; avoid a latest-only history
    # letting our own command hide an interleaved foreign writer.
    readers=[DataReader(participant,Topic(participant,name,LowCmd_),
               Qos(Policy.History.KeepLast(128),Policy.Reliability.BestEffort))
             for name in ('rt/lowcmd','rt/arm_sdk')]
    transport=socket.socket(fileno=fd);transport.setblocking(False)
    ending=threading.Event();drops=0;next_heartbeat=0.
    def parent():sys.stdin.read(1);ending.set()
    threading.Thread(target=parent,daemon=True).start()
    def send(kind,source,received,crc):
        nonlocal drops
        try:transport.send(PACKET.pack(kind,source,received,crc))
        except BlockingIOError:drops+=1
    try:
        while not ending.is_set():
            for kind,reader in enumerate(readers,1):
                samples=reader.take(128);received=time.monotonic()
                if len(samples)==128:drops+=1  # Saturation invalidates ownership confidence.
                for msg in samples:
                    if isinstance(msg,InvalidSample) or not msg.sample_info.valid_data:continue
                    send(kind,int(msg.sample_info.source_timestamp),received,int(msg.crc))
            now=time.monotonic()
            if now>=next_heartbeat:send(0,drops,now,0);next_heartbeat=now+.02
            ending.wait(.001)
    finally:transport.close()


class CommandMonitor:
    def __init__(self,interface,on_command):
        parent,child=socket.socketpair(socket.AF_UNIX,socket.SOCK_DGRAM)
        parent.setsockopt(socket.SOL_SOCKET,socket.SO_RCVBUF,65536)
        child.setsockopt(socket.SOL_SOCKET,socket.SO_SNDBUF,65536)
        parent.setblocking(False);self.socket=parent;self.ending=threading.Event()
        self.heartbeat=0.;self.drops=0;self.delivery_age=0.;self.errors=deque(maxlen=3)
        try:
            self.process=subprocess.Popen([sys.executable,'-u',os.path.abspath(__file__),interface,str(child.fileno())],
                stdin=subprocess.PIPE,stdout=sys.stderr,pass_fds=(child.fileno(),))
        except Exception:parent.close();child.close();raise
        child.close()
        self.reader=threading.Thread(target=self._read,args=(on_command,),daemon=True);self.reader.start()

    def _read(self,callback):
        while not self.ending.is_set() and self.process.poll() is None:
            try:
                if not select.select([self.socket],[],[],.05)[0]:continue
                for _ in range(256):
                    try:packet=self.socket.recv(PACKET.size+1)
                    except BlockingIOError:break
                    kind,source,received,crc=PACKET.unpack(packet)
                    self.delivery_age=max(0.,time.monotonic()-received)
                    if self.delivery_age>.15:raise ValueError('Command observation delivery delayed')
                    if kind==0:self.heartbeat=received;self.drops=source
                    elif kind in (1,2):callback(kind,crc,received)
                    else:raise ValueError('Unknown command observation kind')
            except (OSError,ValueError,struct.error) as exc:self.errors.append(str(exc));return

    def healthy(self):
        return bool(self.process.poll() is None and not self.errors and not self.drops and
                    0<=time.monotonic()-self.heartbeat<.2)

    def Close(self):
        self.ending.set();self.process.stdin.close()
        try:self.process.wait(timeout=2)
        except subprocess.TimeoutExpired:self.process.terminate();self.process.wait(timeout=2)
        self.reader.join(timeout=.3);self.socket.close()


if __name__=='__main__':command_capture(sys.argv[1],int(sys.argv[2]))
