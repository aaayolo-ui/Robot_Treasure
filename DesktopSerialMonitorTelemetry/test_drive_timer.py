"""Offline tests; never opens a serial port."""
import sys
import unittest
from pathlib import Path
sys.path.insert(0, str(Path(__file__).with_name('assets')))
from drive_timer import DriveTimer
from laser_live_server import LaserFeed

def frame(tick, event, seq, run=1):
    payload='MT3,'+','.join(map(str,[run,seq,tick,event]+[0]*16))
    return payload+'*%04X'%LaserFeed._crc16(payload)

class TimerTests(unittest.TestCase):
    def test_start_stop_restart(self):
        t=DriveTimer()
        t.observe(frame(1000,2,1))
        t.observe(frame(1500,5,2))
        t.observe(frame(2500,2,1,2))
        t.observe(frame(3500,13,2,2))
        self.assertEqual(t.snapshot(),dict(elapsed_ms=2500,running=False,reason='已停车'))
        t.observe(frame(5000,2,1,3))
        self.assertEqual(t.elapsed,0)
        self.assertTrue(t.running)

    def test_fault_end_and_bad_frames(self):
        t=DriveTimer()
        t.observe(frame(100,2,1))
        t.observe(frame(1000,14,2))
        t.observe(frame(5000,13,3)+'0')
        self.assertTrue(t.running)
        t.fault()
        t.observe(frame(1050,13,3))
        self.assertEqual(t.elapsed,950)
        self.assertEqual(t.reason,'故障停车')
        t.observe(frame(900,14,2))
        self.assertEqual(t.elapsed,950)

    def test_rollover_and_reboot(self):
        t=DriveTimer()
        t.observe(frame(0xfffffff0,2,1))
        t.observe(frame(20,14,2))
        self.assertEqual(t.elapsed,36)
        t.observe(frame(5,1,1,2))
        self.assertFalse(t.running)
        self.assertEqual(t.reason,'设备时间重置')

if __name__=='__main__': unittest.main()
