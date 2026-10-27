"""Prevent a second native SDK watcher from replacing terminal A's stream."""
import fcntl,json,os
from pathlib import Path

LOCK_PATH=Path(os.environ.get('XDG_RUNTIME_DIR', str(Path.home()/'.cache')))/'g1-pico-teleop/sdk-60061.lock'


def connected_sdk_sockets(proc_root=Path('/proc/net'),port=60061):
    """Count client endpoints, including readers started before this guard.

    PC Service is configured on loopback port60061. Each SDK client maintains
    a TCP connection there. No new SDK connection is opened by this check.
    """
    count=0;read=False
    for name in ('tcp','tcp6'):
        path=proc_root/name
        try:lines=path.read_text().splitlines()[1:];read=True
        except FileNotFoundError:continue
        except OSError as exc:raise RuntimeError('Cannot inspect existing SDK connections: '+str(exc)) from exc
        for line in lines:
            fields=line.split()
            try:
                if fields[3]=='01' and int(fields[2].rsplit(':',1)[1],16)==port:count+=1
            except (IndexError,ValueError):continue
    if not read:raise RuntimeError('Cannot inspect SDK ownership: no TCP socket tables')
    return count


class NativeSdkLease:
    def __init__(self,label,lock_path=LOCK_PATH,connection_check=connected_sdk_sockets):
        self.label=label;self.path=Path(lock_path);self.check=connection_check;self.file=None

    def acquire(self):
        if self.file is not None:raise RuntimeError('SDK lease already acquired')
        self.path.parent.mkdir(parents=True,exist_ok=True)
        stream=self.path.open('a+',encoding='utf-8')
        try:
            try:fcntl.flock(stream.fileno(),fcntl.LOCK_EX|fcntl.LOCK_NB)
            except BlockingIOError:
                stream.seek(0);owner=stream.read().strip()
                raise RuntimeError('Native SDK reader already active '+owner+
                    '. Keep terminal A as the sole SDK reader; inspect its relay/logs, or stop it before running a probe.') from None
            if self.check():
                raise RuntimeError('PC Service already has an SDK client connected. A second watcher would disconnect it. '
                    'Keep terminal A running and use its relay/logs; stop the existing SDK client before a raw probe.')
            stream.seek(0);stream.truncate();json.dump(dict(label=self.label,pid=os.getpid()),stream);stream.flush()
            self.file=stream
        except BaseException:stream.close();raise
        return self

    def release(self):
        if self.file is not None:
            stream,self.file=self.file,None
            fcntl.flock(stream.fileno(),fcntl.LOCK_UN);stream.close()

    def __enter__(self):return self.acquire()
    def __exit__(self,*_):self.release()
