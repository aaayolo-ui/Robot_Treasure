"""Real raw-pair parser checks; hardware is never opened."""
import sys
import unittest
from pathlib import Path
sys.path.insert(0, str(Path(__file__).with_name('assets')))
from console_server import SharedFeed


class LiveGrayTests(unittest.TestCase):
    def test_both_boards_keep_all_twelve_bits_including_zero(self):
        feed = SharedFeed()
        feed.parse_line('GRAY_LIVE T=250 FRONT=801 REAR=000')
        self.assertEqual(feed.gray['front']['mask'], 0x801)
        self.assertEqual(feed.gray['rear']['mask'], 0)
        self.assertFalse(feed.gray['front']['filtered'])
        self.assertEqual(feed.gray['rear']['source'], 'GRAY_LIVE')
        self.assertEqual(feed.gray['rear']['mcu_ms'], 250)

    def test_existing_recovery_pair_and_single_leading_board(self):
        feed = SharedFeed()
        feed.parse_line('MAP_GRAY T=50 N=38 MM=12 EXPECT=1 RAW=FFF,001 YAW_ERR=0 LR=1,1 BRAKE=0')
        self.assertEqual(feed.gray['front']['mask'], 0xfff)
        self.assertEqual(feed.gray['rear']['mask'], 1)
        old = feed.gray['rear'].copy()
        feed.parse_line('TURN_EXIT_SCAN N=38 MM=40 RAW=800 MODE=1')
        self.assertEqual(feed.gray['rear'], old)

    def test_invalid_and_oversized_masks_are_not_truncated(self):
        feed = SharedFeed()
        for line in ('GRAY_LIVE T=1 FRONT=1000 REAR=000',
                     'GRAY_LIVE T=1 FRONT=GGG REAR=000',
                     'MAP_GRAY T=1 RAW=000,FFFF YAW_ERR=0'):
            feed.parse_line(line)
        self.assertEqual(feed.gray, {'front': None, 'rear': None})

    def test_reboot_clears_old_lights(self):
        feed = SharedFeed()
        feed.parse_line('GRAY_LIVE T=1 FRONT=FFF REAR=FFF')
        feed.parse_line('BOOT FW=UARTCH4_16U STAGE=LASER_CONFIG')
        self.assertEqual(feed.gray, {'front': None, 'rear': None})

if __name__ == '__main__': unittest.main()
