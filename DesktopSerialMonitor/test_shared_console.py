"""Offline tests only: no real serial ports are opened."""
import base64
import json
import sys
import threading
import time
import unittest
from pathlib import Path
from unittest.mock import patch
from types import SimpleNamespace
from urllib.request import Request, urlopen
from urllib.error import HTTPError

sys.path.insert(0,str(Path(__file__).with_name("assets")))
from console_server import SharedFeed, handler_for, ThreadingHTTPServer


class FakeSerial:
    instances=[]
    payload=b"LASER DIR=LEFT D=475 C=100 STATUS=VALID\nraw\x00\xff\n"
    def __init__(self,**kwargs):
        self.settings=kwargs
        self.is_open=False
        self.sent=[]
        self.data=[self.payload]
        self.instances.append(self)
    def open(self): self.is_open=True
    def read(self,size):
        if self.data: return self.data.pop(0)
        time.sleep(.005)
        return b""
    def write(self,data): self.sent.append(data); return len(data)
    def close(self): self.is_open=False
    def cancel_read(self): pass


class SharedConsoleTests(unittest.TestCase):
    def test_one_serial_read_feeds_both_views_and_send_uses_same_handle(self):
        FakeSerial.instances=[]
        feed=SharedFeed()
        thread=threading.Thread(target=feed.run,daemon=True)
        with patch("console_server.serial.Serial",FakeSerial),patch("console_server.list_ports.comports",return_value=[SimpleNamespace(device="COM9")]):
            thread.start()
            try:
                feed.configure(dict(port="COM9",baud=115200,bytesize=8,parity="N",stopbits=1))
                deadline=time.monotonic()+2
                while feed.rx==0 and time.monotonic()<deadline: time.sleep(.01)
                self.assertEqual(len(FakeSerial.instances),1)
                self.assertEqual(base64.b64decode(feed.console(0)["data"]),FakeSerial.payload)
                self.assertEqual(feed.snapshot()["channels"]["2"]["mm"],475)
                result=feed.send(base64.b64encode(b"test\x00\xff").decode())
                self.assertEqual(result["written"],6)
                self.assertEqual(FakeSerial.instances[0].sent,[b"test\x00\xff"])
                feed.disconnect()
                self.assertFalse(FakeSerial.instances[0].is_open)
                self.assertRaises(ValueError,feed.send,"dGVzdA==")
            finally:
                feed.stop();thread.join(1)
        self.assertFalse(thread.is_alive())

    def test_ring_buffer_cursor_and_binary_integrity(self):
        feed=SharedFeed()
        data=bytes(range(256))*2400
        feed.record(data)
        cursor=0;combined=bytearray()
        while True:
            packet=feed.console(cursor)
            combined.extend(base64.b64decode(packet["data"]))
            cursor=packet["next"]
            if cursor==packet["rx"]:break
        self.assertEqual(combined,data[-524288:])
        self.assertTrue(feed.console(0)["dropped"])
        self.assertEqual(feed.console(feed.rx)["data"],"")

    def test_configuration_validation_does_not_open_port(self):
        feed=SharedFeed()
        with patch("console_server.serial.Serial") as factory:
            for config in (dict(port="COM9",baud=3),dict(port="bad"),dict(port="COM9",bytesize=9),dict(port="COM9",parity="X")):
                with self.assertRaises(ValueError):feed.configure(config)
            factory.assert_not_called()

    def test_http_private_send_is_not_exposed_to_web_or_cloud(self):
        feed=SharedFeed()
        server=ThreadingHTTPServer(("127.0.0.1",0),handler_for(feed,"test-desktop-key"))
        thread=threading.Thread(target=server.serve_forever,daemon=True);thread.start()
        url="http://127.0.0.1:%d"%server.server_port
        def request(path,body=None,key=None,origin=None):
            headers={"Content-Type":"application/json"}
            if key:headers["X-Desktop-Key"]=key
            if origin:headers["Origin"]=origin
            with urlopen(Request(url+path,data=None if body is None else json.dumps(body).encode(),headers=headers),timeout=2) as response:return json.load(response)
        try:
            self.assertFalse(request("/api/laser")["serial_enabled"])
            for path,body in (("/api/console",None),("/api/console/send",dict(data="dGVzdA==")),("/api/console/connect",dict(port="COM9"))):
                with self.assertRaises(HTTPError) as error:request(path,body)
                self.assertEqual(error.exception.code,403)
            with self.assertRaises(HTTPError) as error:request("/api/console/send",dict(data="dGVzdA=="),"test-desktop-key","https://example.org")
            self.assertEqual(error.exception.code,403)
            self.assertEqual(request("/api/console",key="test-desktop-key")["rx"],0)
            with self.assertRaises(HTTPError) as error:request("/api/console/send",dict(data="dGVzdA=="),"test-desktop-key")
            self.assertEqual(error.exception.code,400)
            self.assertNotIn("test-desktop-key",json.dumps(request("/api/laser")))
        finally:
            server.shutdown();server.server_close();thread.join(1)

if __name__=="__main__":unittest.main(verbosity=2)
