"""Stream episode files over SSH with literal paths and bounded transfers."""
import io
from pathlib import Path, PurePosixPath
import shutil
import subprocess
import tarfile
import threading
from g1_config import ssh

UPLOAD = '''import pathlib,sys,tarfile
root=pathlib.Path(sys.argv[1]);root.mkdir(parents=True,exist_ok=True)
with tarfile.open(fileobj=sys.stdin.buffer,mode='r|') as archive:
 for member in archive:
  name=pathlib.PurePosixPath(member.name)
  if len(name.parts)!=1 or not member.isfile() or name.is_absolute(): raise ValueError('Expected plain source filenames')
  target=root/member.name
  if target.is_symlink(): raise ValueError('Refusing symlink destination')
  target.write_bytes(archive.extractfile(member).read())
'''

def upload_files(host, paths, destination, timeout=30):
    buffer=io.BytesIO()
    with tarfile.open(fileobj=buffer,mode='w') as archive:
        for path in map(Path,paths):
            data=path.read_bytes();member=tarfile.TarInfo(path.name);member.size=len(data)
            archive.addfile(member,io.BytesIO(data))
    subprocess.run(ssh(['python3','-c',UPLOAD,destination],host=host),input=buffer.getvalue(),check=True,timeout=timeout)

def extract_stream(archive, output):
    output=Path(output).resolve();output.mkdir(parents=True,exist_ok=True)
    for member in archive:
        relative=PurePosixPath(member.name)
        if relative.is_absolute() or '..' in relative.parts or not (member.isfile() or member.isdir()):
            raise ValueError('Unexpected episode archive member: '+member.name)
        target=output/str(relative)
        if not target.resolve().is_relative_to(output) or target.is_symlink():
            raise ValueError('Episode path escapes output directory')
        if member.isdir():target.mkdir(parents=True,exist_ok=True);continue
        target.parent.mkdir(parents=True,exist_ok=True)
        temporary=target.with_name(target.name+'.fetching')
        if temporary.is_symlink():raise ValueError('Unexpected temporary symlink')
        with archive.extractfile(member) as src, temporary.open('wb') as dst:shutil.copyfileobj(src,dst,1024*1024)
        temporary.replace(target)

def download_tree(host, remote, output, timeout=600):
    proc=subprocess.Popen(ssh(['tar','-C',remote,'-cf','-','.'],host=host),stdout=subprocess.PIPE)
    expired=threading.Event()
    def stop():
        expired.set();proc.kill()
    timer=threading.Timer(timeout,stop);timer.daemon=True;timer.start()
    try:
        with tarfile.open(fileobj=proc.stdout,mode='r|') as archive:extract_stream(archive,output)
        result=proc.wait(timeout=5)
        if expired.is_set():raise TimeoutError('Episode download exceeded time limit; retry fetch')
        if result:raise subprocess.CalledProcessError(result,proc.args)
    finally:
        timer.cancel();proc.stdout.close()
        if proc.poll() is None:proc.kill();proc.wait(timeout=5)
