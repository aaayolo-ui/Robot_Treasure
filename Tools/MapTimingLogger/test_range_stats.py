"""Calibration telemetry parsing; no real serial connections."""
import unittest
from types import SimpleNamespace
from unittest.mock import patch
from laser_live_server import LaserFeed


class RangeStatisticsTests(unittest.TestCase):
    def setUp(self):
        guard = patch('laser_live_server.serial.Serial', side_effect=AssertionError('No serial IO'))
        guard.start()
        self.addCleanup(guard.stop)
        self.feed = LaserFeed('TEST', 115200)

    def stats(self, tail=' FRAMES=50 WINDOW_MS=1000 HZ_X10=500', count=40):
        self.feed.parse_line(f'RANGE DIR=FRONT N={count} MEAN_MM=905 MIN_MM=900 MAX_MM=910 REJECT=10 FW=104 REF_MM=0'+tail)

    def test_actual_rate_and_statistics_survive_fast_distance_updates(self):
        self.stats()
        self.feed.parse_line('LASER DIR=FRONT D=906 C=100 STATUS=RAW AGE=10 STATE=RANGE_TEST FAULT=NONE')
        data = self.feed.snapshot()
        stats = data['range_stats']['1']
        self.assertEqual((stats['hz'], stats['frames'], stats['window_ms']), (50,50,1000))
        self.assertEqual((stats['mean_mm'], stats['min_mm'], stats['max_mm']), (905,900,910))
        self.assertEqual(data['channels']['1']['mm'], 906)
        stats['mean_mm']=1
        self.assertEqual(self.feed.snapshot()['range_stats']['1']['mean_mm'],905)

    def test_invalid_measurements_still_have_frame_rate_but_no_mean(self):
        self.stats(count=0)
        stats=self.feed.snapshot()['range_stats']['1']
        self.assertEqual(stats['hz'],50)
        self.assertIsNone(stats['mean_mm'])
        self.assertIsNone(stats['min_mm'])

    def test_legacy_firmware_does_not_invent_frame_rate(self):
        self.stats(tail='')
        stats=self.feed.snapshot()['range_stats']['1']
        self.assertIsNone(stats['hz'])
        self.assertIsNone(stats['window_ms'])
        self.assertEqual(stats['mean_mm'],905)
        self.stats(tail=' FRAMES=0 WINDOW_MS=0 HZ_X10=0')
        self.assertIsNone(self.feed.snapshot()['range_stats']['1']['hz'])

    def test_serial_crlf_is_accepted(self):
        self.feed.parse_line('RANGE DIR=RIGHT N=50 MEAN_MM=900 MIN_MM=899 MAX_MM=901 REJECT=0 FW=104 REF_MM=0 FRAMES=50 WINDOW_MS=1000 HZ_X10=500\r')
        self.assertEqual(self.feed.snapshot()['range_stats']['3']['hz'],50)

    def test_restart_and_port_switch_clear_statistics(self):
        self.stats()
        self.feed.parse_line('BOOT FW=LASER_RANGE_CAL_2 MOTORS=OFF')
        self.assertEqual(self.feed.snapshot()['range_stats'],{})
        self.stats()
        with patch('laser_live_server.list_ports.comports',return_value=[SimpleNamespace(device='COM99')]):
            self.feed.select_port('COM99')
        self.assertEqual(self.feed.snapshot()['range_stats'],{})


if __name__=='__main__':
    unittest.main()
