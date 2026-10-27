"""Independent MotionSwitcher RPC worker; no motor messages or automatic mode selection."""
import json,math,os,subprocess,sys,threading,time
from g1_config import DDS_INTERFACE
MAX_MODE_AGE_S=1.

def restore_ai(client,wait=time.sleep):
    """Explicit ownership recovery, never a posture or walking command."""
    code,mode=client.CheckMode()
    if code!=0 or not isinstance(mode,dict):raise RuntimeError('Cannot confirm current controller ownership')
    if mode.get('name')=='ai':return {'restored':False,'mode':mode}
    if mode.get('name')!='':raise RuntimeError('Another controller is active; leaving it unchanged')
    # A busy/timeout acknowledgement can precede a successful transition.
    # Issue once and confirm the resulting state rather than repeating it.
    result=client.SelectMode('ai')
    for _ in range(12):
        wait(.5);code,mode=client.CheckMode()
        if code==0 and isinstance(mode,dict):
            if mode.get('name')=='ai':return {'restored':True,'select_result':result,'mode':mode}
            if mode.get('name')!='':raise RuntimeError('A different controller became active')
    raise RuntimeError('AI restoration is not confirmed; inspect mode before retrying')

def restore_ai_cli(interface=DDS_INTERFACE):
    from pathlib import Path
    for path in Path('/proc').glob('[0-9]*/cmdline'):
        try:args=path.read_bytes().decode(errors='replace').split('\0')
        except OSError:continue
        if any(Path(arg).name in ('g1_suspended_body.py','g1_arm_link.py') for arg in args):
            raise RuntimeError('Stop terminal B and wait for its worker to exit before restoring normal control')
    from unitree_sdk2py.core.channel import ChannelFactoryInitialize
    from unitree_sdk2py.comm.motion_switcher.motion_switcher_client import MotionSwitcherClient
    ChannelFactoryInitialize(0,interface)
    client=MotionSwitcherClient();client.SetTimeout(1.);client.Init()
    print(json.dumps(restore_ai(client)),flush=True)

def valid_mode_row(row):
    if not isinstance(row,dict) or type(row.get('code')) is not int:return False
    if row.get('name') is not None and not isinstance(row['name'],str):return False
    if row['code']==0 and not isinstance(row.get('name'),str):return False
    for key in ('receipt_s','last_success_s'):
        value=row.get(key)
        if type(value) not in (int,float) or not math.isfinite(value) or value<0:return False
    if row['last_success_s']>row['receipt_s'] or type(row.get('release_requested')) is not bool:return False
    result=row.get('release_result')
    if result is not None:
        if not isinstance(result,dict) or type(result.get('code')) is not int:return False
        stamp=result.get('receipt_s')
        if type(stamp) not in (int,float) or not math.isfinite(stamp) or stamp<0:return False
    return True

def classify_mode(row,now=None):
    if not valid_mode_row(row) or row.get('worker_alive') is not True:return 'fault'
    now=time.monotonic() if now is None else now
    if not math.isfinite(now):return 'fault'
    if not(0<=now-row['receipt_s']<MAX_MODE_AGE_S and 0<=now-row['last_success_s']<MAX_MODE_AGE_S):return 'fault'
    if row['code']==0:return 'released' if row['name']=='' else 'active'
    return 'wait' if row['code']==3104 else 'fault'

def motion_mode_server(interface):
    output=sys.stdout;sys.stdout=sys.stderr
    from unitree_sdk2py.core.channel import ChannelFactoryInitialize
    from unitree_sdk2py.comm.motion_switcher.motion_switcher_client import MotionSwitcherClient
    ChannelFactoryInitialize(0,interface)
    client=MotionSwitcherClient();client.SetTimeout(.4);client.Init()
    ending,requested=threading.Event(),threading.Event()
    def watch_parent():
        try:
            while True:
                command=sys.stdin.buffer.read(1)
                if not command:break
                if command==b'R':requested.set()
        finally:ending.set()
    threading.Thread(target=watch_parent,daemon=True).start()
    attempted,release_result,last_success=False,None,0.
    while not ending.is_set():
        if requested.is_set() and not attempted:
            attempted=True
            try:
                code,_=client.ReleaseMode()
                if type(code) is not int:raise ValueError('Noninteger release result')
                release_result={'code':code,'receipt_s':time.monotonic()}
            except Exception as exc:release_result={'code':-1,'receipt_s':time.monotonic(),'error':str(exc)}
        if ending.is_set():break
        name,error=None,None
        try:
            code,mode=client.CheckMode()
            if type(code) is not int:raise ValueError('Noninteger mode result')
            if code==0:
                if not isinstance(mode,dict) or not isinstance(mode.get('name'),str):raise ValueError('Missing mode name')
                name=mode['name']
        except Exception as exc:code,error=-1,str(exc)
        received=time.monotonic()
        if code==0:last_success=received
        row=dict(code=code,name=name,receipt_s=received,last_success_s=last_success,
                 release_requested=requested.is_set(),release_result=release_result)
        if error is not None:row['error']=error
        output.write(json.dumps(row,allow_nan=False)+'\n');output.flush();ending.wait(.25)

class MotionModeMonitor:
    def __init__(self,interface):
        self.result=None;self._release_sent=self._closed=False;self._lock=threading.Lock()
        self.process=subprocess.Popen([sys.executable,'-u',os.path.abspath(__file__),interface],
            # Raw FileIO.readline() reads one byte per syscall and repeatedly
            # reacquires the GIL. Buffered stdout prevents mode rows queuing
            # behind the motor loop. stdin still uses nonblocking os.write.
            stdin=subprocess.PIPE,stdout=subprocess.PIPE,bufsize=-1)
        os.set_blocking(self.process.stdin.fileno(),False)
        self.reader=threading.Thread(target=self._read,daemon=True);self.reader.start()
    def _read(self):
        for line in self.process.stdout:
            try:
                row=json.loads(line)
                if valid_mode_row(row):
                    with self._lock:
                        if self.result is None or row['receipt_s']>self.result['receipt_s']:self.result=row
            except (ValueError,TypeError):pass
    def snapshot(self,timeout=0):
        if not math.isfinite(timeout) or timeout<0:raise ValueError('Invalid timeout')
        deadline=time.monotonic()+timeout
        while True:
            with self._lock:
                row=None if self.result is None else dict(self.result);release_sent=self._release_sent
            alive=not self._closed and self.process.poll() is None;now=time.monotonic()
            if row is not None or not alive or now>=deadline:break
            time.sleep(min(.01,max(0.,deadline-now)))
        if row is None:row=dict(code=-1,name=None,receipt_s=0.,last_success_s=0.,release_requested=False,release_result=None)
        row.update(worker_alive=alive,release_sent=release_sent,receipt_age_s=max(0.,now-row['receipt_s']),
                   last_success_age_s=max(0.,now-row['last_success_s']))
        row['status']=classify_mode(row,now);row['confirmed_released']=row['status']=='released'
        return row
    def request_release(self):
        with self._lock:
            if self._closed or self.process.poll() is not None:raise RuntimeError('Motion worker unavailable')
            if self._release_sent:return False
            try:
                if os.write(self.process.stdin.fileno(),b'R')!=1:raise RuntimeError('Release request not delivered')
            except OSError as exc:raise RuntimeError('Release request not delivered') from exc
            self._release_sent=True;return True
    def close(self):
        if self._closed:return
        self._closed=True;self.process.stdin.close()
        try:self.process.wait(timeout=2)
        except subprocess.TimeoutExpired:
            self.process.terminate()
            try:self.process.wait(timeout=2)
            except subprocess.TimeoutExpired:self.process.kill();self.process.wait(timeout=2)
        self.reader.join(timeout=.3);self.process.stdout.close()

if __name__=='__main__':
    if sys.argv[1:] == ['--restore-ai']:restore_ai_cli()
    else:motion_mode_server(sys.argv[1])
