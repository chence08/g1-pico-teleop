import io
from pathlib import Path
import sys
import tarfile
import tempfile
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'outputs'))
from g1_files import extract_stream

def archive(name='folder with spaces/frame.txt',kind=tarfile.REGTYPE):
    stream=io.BytesIO()
    with tarfile.open(fileobj=stream,mode='w') as out:
        item=tarfile.TarInfo(name);item.type=kind;item.size=3 if kind==tarfile.REGTYPE else 0
        out.addfile(item,io.BytesIO(b'xyz') if item.size else None)
    stream.seek(0)
    return tarfile.open(fileobj=stream,mode='r|')
class FileTransferTests(unittest.TestCase):
    def test_nested_spaces_preserved(self):
        with tempfile.TemporaryDirectory() as temp,archive() as source:
            extract_stream(source,temp)
            self.assertEqual((Path(temp)/'folder with spaces/frame.txt').read_bytes(),b'xyz')
    def test_traversal_and_links_rejected(self):
        for name,kind in [('../escape',tarfile.REGTYPE),('/absolute',tarfile.REGTYPE),('link',tarfile.SYMTYPE)]:
            with tempfile.TemporaryDirectory() as temp,archive(name,kind) as source:
                with self.assertRaises(ValueError):extract_stream(source,temp)
    def test_existing_output_symlink_rejected(self):
        with tempfile.TemporaryDirectory() as temp,tempfile.TemporaryDirectory() as outside,archive('escape/frame.txt') as source:
            (Path(temp)/'escape').symlink_to(outside,target_is_directory=True)
            with self.assertRaises(ValueError):extract_stream(source,temp)
if __name__=='__main__':unittest.main()
