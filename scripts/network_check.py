"""Measure application RTT over one SSH session; no robot SDK, files or commands.

Run with control/recording/downloads stopped for an idle baseline. This measures
SSH transport and process scheduling, not controller-to-motor latency.
"""
import argparse
import json
from pathlib import Path
import select
import statistics
import subprocess
import sys
import time
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'outputs'))
from g1_config import ssh
WORKER='import sys\nfor line in sys.stdin:\n print(line.strip(), flush=True)\n'

def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--samples',type=int,default=200);p.add_argument('--interval',type=float,default=.05);args=p.parse_args()
    if not 2<=args.samples<=10000 or not .01<=args.interval<=10: p.error('samples 2..10000, interval .01..10')
    proc=subprocess.Popen(ssh(['python3','-u','-c',WORKER]),stdin=subprocess.PIPE,stdout=subprocess.PIPE,text=True,bufsize=1)
    samples=[]
    try:
        for i in range(args.samples+1):
            start=time.monotonic_ns();proc.stdin.write(str(i)+'\n');proc.stdin.flush()
            if not select.select([proc.stdout],[],[],15)[0]:raise TimeoutError('SSH echo timed out')
            if proc.stdout.readline().strip()!=str(i):raise RuntimeError('SSH echo exited or mismatched')
            if i: samples.append((time.monotonic_ns()-start)/1e6) # Exclude startup handshake.
            time.sleep(args.interval)
    finally:
        proc.stdin.close()
        try:proc.wait(timeout=5)
        except subprocess.TimeoutExpired:proc.terminate();proc.wait(timeout=5)
    ordered=sorted(samples)
    print(json.dumps(dict(samples=len(samples),median_ms=statistics.median(samples),p95_ms=ordered[int(.95*(len(ordered)-1))],max_ms=max(samples),over100ms=sum(v>100 for v in samples),metric='SSH application echo; not motion latency'),indent=2))
if __name__=='__main__':main()
