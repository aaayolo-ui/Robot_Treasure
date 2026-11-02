"""Threaded serial lifecycle proof with fake handles only; never opens hardware."""
import threading,time,unittest
from types import SimpleNamespace
from unittest.mock import patch
from laser_live_server import LaserFeed

class SwitchTests(unittest.TestCase):
    def wait_for(self, pred):
        limit=time.monotonic()+2
        while not pred() and time.monotonic()<limit:time.sleep(.005)
        self.assertTrue(pred())

    def test_default_and_select_do_not_open_serial(self):
        feed=LaserFeed('COM9',115200)
        with patch('laser_live_server.serial.Serial') as ctor, patch('laser_live_server.list_ports.comports',return_value=[SimpleNamespace(device='COM13')]):
            thread=threading.Thread(target=feed.run,daemon=True);thread.start()
            try:
                feed.select_port('COM13');time.sleep(.04)
                ctor.assert_not_called();self.assertFalse(feed.snapshot()['serial_enabled'])
            finally:feed.stop();thread.join(1)
            self.assertFalse(thread.is_alive())

    def test_disconnect_releases_handle_and_cancels_reconnect(self):
        readers=[]
        class Reader:
            def __init__(self,**kwargs):self.is_open=False;self.event=threading.Event();readers.append(self)
            def open(self):self.is_open=True
            def read(self,n):self.event.wait(.2);return b''
            def cancel_read(self):self.event.set()
            def close(self):self.is_open=False;self.event.set()
        feed=LaserFeed('COM9',115200)
        with patch('laser_live_server.serial.Serial',side_effect=Reader),patch('laser_live_server.list_ports.comports',return_value=[SimpleNamespace(device='COM9')]):
            thread=threading.Thread(target=feed.run,daemon=True);thread.start()
            try:
                feed.connect_port('COM9');self.wait_for(lambda:feed.connected)
                feed.disconnect()
                self.assertFalse(readers[0].is_open,'disconnect response must release the COM handle')
                time.sleep(.05);self.assertEqual(len(readers),1)
                self.assertFalse(feed.snapshot()['connected']);self.assertFalse(feed.snapshot()['serial_enabled'])
            finally:feed.stop();thread.join(1)
            self.assertFalse(thread.is_alive())

    def test_missing_port_does_not_enable_reader(self):
        feed=LaserFeed('COM9',115200)
        with patch('laser_live_server.list_ports.comports',return_value=[]):
            with self.assertRaises(ValueError):feed.connect_port('COM9')
        self.assertFalse(feed.serial_enabled)

if __name__=='__main__':unittest.main()
