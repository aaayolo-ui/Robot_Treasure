"""Offline KEY2 round checks. No real serial or vehicle commands."""
import sys
import unittest
from pathlib import Path
sys.path.insert(0, str(Path(__file__).with_name('assets')))
from round_timer import RoundTimer
from console_server import SharedFeed
from test_auto_trials import frame, ready


class Clock:
    def __init__(self): self.now = 0.0
    def __call__(self): return self.now
    def advance(self, seconds): self.now += seconds


class RoundTimerTests(unittest.TestCase):
    def feed(self):
        c = Clock(); f = SharedFeed(); f.round_timer = RoundTimer(c)
        return c, f

    def test_boot_and_waiting_do_not_start(self):
        c, f = self.feed()
        f.parse_line('BOOT FW=UARTCH4_16U STAGE=LASER_CONFIG')
        f.parse_line('BOOT_STATUS FW=UARTCH4_16U T=30000 STATE=READY')
        f.parse_line(ready(f)); c.advance(200)
        self.assertIsNone(f.round_timer.snapshot()['elapsed_ms'])

    def test_first_start_excludes_boot_wait(self):
        c, f = self.feed(); c.advance(50)
        f.parse_line(frame(f, 1, 1, 50000, 2))
        self.assertEqual(f.round_timer.snapshot()['elapsed_ms'], 0)
        c.advance(2)
        self.assertEqual(f.round_timer.snapshot()['elapsed_ms'], 2000)

    def test_stops_faults_replans_and_view_reset_do_not_pause(self):
        c, f = self.feed(); f.parse_line(ready(f))
        f.parse_line(frame(f, 1, 1, 10000, 2)); c.advance(20)
        f.parse_line(frame(f, 1, 2, 30000, 13))
        f.parse_line('ROUTE_FAULT=GRAY_BOTH_NO_LINE FROM=FRONT_WAIT')
        c.advance(30); f.parse_line(ready(f, 38, 37))
        f.parse_line(frame(f, 2, 1, 60000, 2, 38, 37))
        f.reset_position_view(); f.disconnect(); c.advance(130)
        self.assertEqual(f.round_timer.snapshot()['elapsed_ms'], 180000)
        self.assertTrue(f.round_timer.snapshot()['reached_180'])

    def test_threshold_keeps_accumulating(self):
        c = Clock(); t = RoundTimer(c); t.start(); c.advance(179.999)
        self.assertFalse(t.snapshot()['reached_180']); c.advance(.001)
        self.assertTrue(t.snapshot()['reached_180']); c.advance(20)
        self.assertEqual(t.snapshot()['elapsed_ms'], 200000)

    def test_reset_waits_for_next_key2(self):
        c, f = self.feed(); f.parse_line(frame(f, 1, 1, 100, 2)); c.advance(220)
        f.parse_line('BOOT FW=UARTCH4_16U STAGE=LASER_CONFIG'); c.advance(10)
        self.assertIsNone(f.round_timer.snapshot()['elapsed_ms'])
        f.parse_line(frame(f, 1, 1, 10000, 2))
        self.assertEqual(f.round_timer.snapshot()['elapsed_ms'], 0)

    def test_bad_crc_and_rejected_start_do_not_start(self):
        c, f = self.feed(); f.parse_line(frame(f, 1, 1, 10000, 2)+'F')
        f.parse_line('START_REJECT STATE=FAULT REASON=GRAY')
        self.assertIsNone(f.round_timer.snapshot()['elapsed_ms'])

    def test_late_connection_does_not_invent_time(self):
        c, f = self.feed(); f.parse_line(frame(f, 1, 15, 125000, 14)); c.advance(10)
        self.assertIsNone(f.round_timer.snapshot()['elapsed_ms'])

    def test_duplicate_start_and_tick_rollover(self):
        c, f = self.feed(); f.parse_line(frame(f, 1, 1, 0xfffffff0, 2)); c.advance(5)
        f.parse_line(frame(f, 1, 1, 0xfffffff0, 2)); f.parse_line(frame(f, 1, 2, 20, 14))
        self.assertEqual(f.round_timer.snapshot()['elapsed_ms'], 5000)

if __name__ == '__main__': unittest.main(verbosity=2)
