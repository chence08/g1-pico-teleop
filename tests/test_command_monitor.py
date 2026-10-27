"""Command-event IPC contract; no DDS or hardware."""
from collections import deque
from pathlib import Path
import socket,sys,threading,time,types,unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'outputs'))
from g1_command_monitor import CommandMonitor,PACKET


class CommandMonitorTests(unittest.TestCase):
    def run_packets(self,packets):
        parent,child=socket.socketpair(socket.AF_UNIX,socket.SOCK_DGRAM);parent.setblocking(False)
        monitor=CommandMonitor.__new__(CommandMonitor);monitor.socket=parent;monitor.ending=threading.Event()
        monitor.process=types.SimpleNamespace(poll=lambda:None);monitor.errors=deque(maxlen=3)
        monitor.heartbeat=0.;monitor.drops=0;monitor.delivery_age=0.;events=[]
        for p in packets:child.send(p)
        thread=threading.Thread(target=monitor._read,args=(lambda *args:events.append(args),),daemon=True);thread.start()
        deadline=time.monotonic()+.2
        while len(events)+int(monitor.heartbeat>0)<len(packets) and not monitor.errors and time.monotonic()<deadline:time.sleep(.001)
        monitor.ending.set();thread.join(timeout=.2);parent.close();child.close()
        return monitor,events

    def test_preserves_interleaved_foreign_and_own_events(self):
        now=time.monotonic()
        packets=[PACKET.pack(1,i,now,crc) for i,crc in enumerate([11,999,11,12],1)]
        packets.append(PACKET.pack(0,0,now,0))
        monitor,events=self.run_packets(packets)
        self.assertEqual([event[1] for event in events],[11,999,11,12])
        self.assertTrue(monitor.healthy());self.assertFalse(monitor.errors)

    def test_delayed_ipc_cannot_refresh_ownership(self):
        monitor,events=self.run_packets([PACKET.pack(1,1,time.monotonic()-1,99)])
        self.assertFalse(events);self.assertTrue(monitor.errors);self.assertFalse(monitor.healthy())

    def test_dropped_observations_invalidate_ownership(self):
        monitor,events=self.run_packets([PACKET.pack(0,1,time.monotonic(),0)])
        self.assertEqual(monitor.drops,1);self.assertFalse(monitor.healthy())

if __name__=='__main__':unittest.main()
