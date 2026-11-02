"""Offline telemetry ambiguity must not become a false map fix."""
import sys,unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).with_name('assets')))
from console_server import SharedFeed

def mt(feed,event,sequence,enc=0):
    values=[1,sequence,1000+sequence*20,event,0,1601,0,4,6,
            feed.map_signature if event==2 else 0,0,1,0,sequence,enc,-enc,enc,-enc,0,1]
    payload='MT3,'+','.join(map(str,values))
    return payload+'*%04X'%feed._crc16(payload)

class IdentityMonitorTests(unittest.TestCase):
    def test_uncertainty_freezes_map_but_keeps_laser_and_yaw(self):
        f=SharedFeed();f.parse_line(mt(f,2,1));cursor=f.track_cursor
        f.parse_line('LOC_UNCERTAIN EXPECT=N006 CAND=6,7,0,0 COUNT=2 RAW=07F')
        f.parse_line('LASER DIR=LEFT D=506 C=100 STATUS=OPPONENT STATE=ESCAPING FAULT=NONE')
        f.parse_line(mt(f,14,2,4000))
        self.assertEqual(f.snapshot()['localization_uncertain']['candidates'],[6,7])
        self.assertEqual(f.track_cursor,cursor)
        self.assertEqual(f.snapshot()['channels']['2']['mm'],506)
        self.assertEqual(f.snapshot()['gyro']['frames'],2)
        f.parse_line('LOC NODE=N007 X_MM=2400 Y_MM=0 BODY=3 TRAVEL=1 MODE=1 VALID=1 PHASE=0 YAW_ERR=0 YAW_AGE=1 CW_SIGN=-1 FIXES=32 STATUS=RELOCALIZED')
        self.assertIsNone(f.snapshot()['localization_uncertain'])
        self.assertEqual(f.snapshot()['position']['x_mm'],2400)

if __name__=='__main__':unittest.main()
