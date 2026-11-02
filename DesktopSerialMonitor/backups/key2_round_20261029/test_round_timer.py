"""Offline RESET-to-RESET timer checks; no real serial or vehicle commands."""
import sys
import unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).with_name('assets')))
from round_timer import RoundTimer
from console_server import SharedFeed
from test_auto_trials import frame,ready


class Clock:
    def __init__(self):self.now=0.0
    def __call__(self):return self.now
    def advance(self,seconds):self.now+=seconds


class RoundTimerTests(unittest.TestCase):
    def test_opening_software_does_not_start_a_round(self):
        clock=Clock();timer=RoundTimer(clock);clock.advance(200)
        self.assertIsNone(timer.snapshot()['elapsed_ms'])
        self.assertFalse(timer.snapshot()['reached_180'])

    def test_stops_faults_replans_and_view_reset_do_not_zero_round(self):
        clock=Clock();feed=SharedFeed();feed.round_timer=RoundTimer(clock)
        feed.parse_line('BOOT FW=UARTCH4_16U_MONITOR13 STAGE=LASER_CONFIG')
        clock.advance(10);feed.parse_line(frame(feed,1,1,10000,2))
        clock.advance(20);feed.parse_line(frame(feed,1,2,30000,13))
        feed.parse_line('ROUTE_FAULT=GRAY_BOTH_NO_LINE FROM=FRONT_WAIT')
        clock.advance(30);feed.parse_line(ready(feed));feed.parse_line(frame(feed,2,1,60000,2))
        self.assertEqual(feed.round_timer.snapshot()['elapsed_ms'],60000)
        feed.reset_position_view();feed.disconnect();clock.advance(120)
        state=feed.round_timer.snapshot()
        self.assertEqual(state['elapsed_ms'],180000)
        self.assertTrue(state['running'])
        self.assertTrue(state['reached_180'])

    def test_threshold_is_180_seconds_and_does_not_stop_clock(self):
        clock=Clock();timer=RoundTimer(clock);timer.reset()
        clock.advance(179.999);self.assertFalse(timer.snapshot()['reached_180'])
        clock.advance(.001);self.assertTrue(timer.snapshot()['reached_180'])
        clock.advance(20);self.assertEqual(timer.snapshot()['elapsed_ms'],200000)
        self.assertEqual(timer.snapshot()['remaining_ms'],0)

    def test_next_reset_starts_a_new_full_round(self):
        clock=Clock();timer=RoundTimer(clock);timer.reset();clock.advance(220)
        timer.reset()
        self.assertEqual(timer.snapshot()['elapsed_ms'],0)
        self.assertFalse(timer.snapshot()['reached_180'])
        clock.advance(3);self.assertEqual(timer.snapshot()['elapsed_ms'],3000)

    def test_late_serial_attachment_uses_mcu_uptime(self):
        clock=Clock();timer=RoundTimer(clock);timer.observe_tick(125000)
        self.assertEqual(timer.snapshot()['elapsed_ms'],125000)
        clock.advance(55);self.assertTrue(timer.snapshot()['reached_180'])

    def test_status_time_recovers_reset_without_a_start_record(self):
        clock=Clock();timer=RoundTimer(clock)
        timer.observe_status('BOOT_STATUS FW=UARTCH4_16U T=3500 STATE=FAULT')
        clock.advance(10);self.assertEqual(timer.snapshot()['elapsed_ms'],13500)
        timer.observe_status('BOOT_STATUS FW=UARTCH4_16U T=3300 STATE=FAULT')
        self.assertEqual(timer.snapshot()['elapsed_ms'],13500)

    def test_missed_boot_text_and_tick_wrap(self):
        clock=Clock();feed=SharedFeed();feed.round_timer=RoundTimer(clock)
        feed.parse_line(frame(feed,1,1,100000,2));clock.advance(5)
        feed.parse_line(frame(feed,1,2,105000,13));feed.parse_line(ready(feed))
        feed.parse_line(frame(feed,1,1,10,2))
        self.assertEqual(feed.round_timer.snapshot()['elapsed_ms'],10)
        timer=RoundTimer(clock);timer.observe_tick(0xfffffff0);clock.advance(.036)
        timer.observe_tick(20)
        self.assertEqual(timer.snapshot()['elapsed_ms'],0xfffffff0+36)

if __name__=='__main__':unittest.main(verbosity=2)
