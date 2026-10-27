import json,sys,tempfile,unittest
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'outputs'))
from pico_sdk_lease import NativeSdkLease,connected_sdk_sockets


class SdkLeaseTests(unittest.TestCase):
    def test_exclusive_reader_and_release(self):
        with tempfile.TemporaryDirectory() as folder:
            path=Path(folder)/'lease'
            first=NativeSdkLease('bridge',path,lambda:0).acquire()
            try:
                with self.assertRaisesRegex(RuntimeError,'sole SDK reader'):
                    NativeSdkLease('probe',path,lambda:0).acquire()
                self.assertEqual(json.loads(path.read_text())['label'],'bridge')
            finally:first.release()
            with NativeSdkLease('next bridge',path,lambda:0):pass

    def test_existing_unguarded_reader_cannot_be_replaced(self):
        with tempfile.TemporaryDirectory() as folder:
            path=Path(folder)/'lease'
            with self.assertRaisesRegex(RuntimeError,'would disconnect'):
                NativeSdkLease('probe',path,lambda:1).acquire()
            with NativeSdkLease('bridge after old reader stops',path,lambda:0):pass

    def test_client_endpoints_only_ipv4_and_ipv6(self):
        with tempfile.TemporaryDirectory() as folder:
            root=Path(folder);port=f'{60061:04X}'
            (root/'tcp').write_text('header\n 0: 0100007F:ABCD 0100007F:'+port+' 01\n'+
                ' 1: 0100007F:'+port+' 0100007F:ABCD 01\n'+
                ' 2: 0100007F:1234 0100007F:'+port+' 06\n')
            (root/'tcp6').write_text('header\n 0: 0000:ABCD 0000:'+port+' 01\n')
            self.assertEqual(connected_sdk_sockets(root),2)

    def test_unknown_ownership_fails_closed(self):
        with tempfile.TemporaryDirectory() as folder:
            with self.assertRaisesRegex(RuntimeError,'no TCP socket tables'):
                connected_sdk_sockets(Path(folder))

if __name__=='__main__':unittest.main()
