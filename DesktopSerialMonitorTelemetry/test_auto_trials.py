"""Offline trial boundaries and automatic role recognition; no serial access."""
import sys
import unittest
from pathlib import Path
sys.path.insert(0, str(Path(__file__).with_name("assets")))
from console_server import SharedFeed


def frame(feed, run, seq, tick, event, start=54, end=53):
    values=[run,seq,tick,event,0,1601,0,start,end,
            feed.map_signature if event==2 else 0,0,1,0,seq,0,0,0,0,0,0]
    payload="MT3,"+",".join(map(str,values))
    return payload+"*%04X"%feed._crc16(payload)


def ready(feed, start=54, first=53, heading="W"):
    return "MAP_READY R001 START=N%03d HEADING=%s FIRST=N%03d STEPS=5 SIGNATURE=%d"%(start,heading,first,feed.map_signature)


class AutoTrialTests(unittest.TestCase):
    def test_new_test_clears_old_path_and_reanchors_start(self):
        f=SharedFeed();f.parse_line(ready(f))
        f.parse_line(frame(f,1,1,100,2));f.parse_line(frame(f,1,2,300,15))
        f.parse_line(frame(f,1,3,400,13))
        old_cursor=f.track_cursor;generation=f.track_generation
        f.parse_line(frame(f,2,1,900,2))
        self.assertGreater(f.track_generation,generation)
        self.assertGreater(f.track_cursor,old_cursor)
        self.assertEqual(len(f.track),1)
        self.assertEqual((f.latest_position['x_mm'],f.latest_position['y_mm']),f.nodes[54])
        self.assertEqual(f.latest_position['run'],2)
        self.assertEqual(f.drive_timer.elapsed,0)
        self.assertTrue(f.drive_timer.running)

    def test_replanning_does_not_erase_current_trial(self):
        f=SharedFeed();f.parse_line(ready(f));f.parse_line(frame(f,1,1,100,2))
        generation=f.track_generation;first_id=f.track[0]['id']
        f.parse_line(ready(f,38,37))
        f.parse_line(frame(f,2,1,200,2,38,37))
        self.assertEqual(f.track_generation,generation)
        self.assertEqual(f.trial_origin,54)
        self.assertEqual(f.track[0]['id'],first_id)

    def test_resume_from_hide_is_not_a_new_test_at_origin(self):
        f=SharedFeed();f.parse_line(ready(f));f.parse_line(frame(f,1,1,100,2))
        f.parse_line(frame(f,1,2,300,13));generation=f.track_generation
        f.parse_line(ready(f,38,37));f.parse_line(frame(f,2,1,400,2,38,37))
        self.assertEqual(f.track_generation,generation)
        self.assertEqual(f.trial_origin,54)

    def test_reboot_clears_history_without_inventing_a_fix(self):
        f=SharedFeed();f.parse_line(frame(f,1,1,100,2));generation=f.track_generation
        f.parse_line('BOOT FW=UARTCH4_16U_MONITOR13 STAGE=LASER_CONFIG')
        self.assertGreater(f.track_generation,generation)
        self.assertEqual(f.track,[])
        self.assertIsNone(f.latest_position)
        self.assertIsNone(f.vehicle_role)
        f.parse_line(ready(f))
        self.assertEqual(f.latest_position['source'],'MAP_READY')

    def test_explicit_project_role_wins_over_comparison_debug_names(self):
        f=SharedFeed()
        f.parse_line('BOOT FW=UARTCH4_16U_POLICE_COMPARE1 STAGE=LASER_CONFIG')
        self.assertIsNone(f.vehicle_role)
        f.parse_line('Thief16U_PoliceTurnCompare_Test FW=UARTCH4_16U_POLICE_COMPARE1')
        f.parse_line('POLICE_CAPTURE_JUNCTION=N039')
        self.assertEqual(f.vehicle_role,'thief')
        f.parse_line(frame(f,1,1,100,2))
        f.parse_line('Police17A_UARTCh4_LinePursuit_Test FW=UARTCH4_POLICE17A_RUN8')
        self.assertEqual(f.vehicle_role,'police')
        self.assertEqual(f.track,[])
        self.assertIsNone(f.latest_position)

    def test_known_origin_and_late_attachment_fallbacks(self):
        f=SharedFeed();f.parse_line('POLICE_PLAN MODE=SEARCH FROM=N002')
        self.assertEqual(f.vehicle_role,'police')
        f.parse_line(ready(f));self.assertEqual(f.vehicle_role,'police')
        g=SharedFeed();g.parse_line(ready(g));self.assertEqual(g.vehicle_role,'thief')
        g.parse_line('POLICE_PLAN MODE=SEARCH FROM=N054')
        self.assertEqual(g.vehicle_role,'police')  # Explicit traffic beats origin fallback.
        h=SharedFeed();first=next(n for edge in h.edges if 81 in edge for n in edge if n!=81)
        h.parse_line(ready(h,81,first,'N'));self.assertEqual(h.vehicle_role,'police')

    def test_bad_crc_cannot_reset_a_trial(self):
        f=SharedFeed();f.parse_line(frame(f,1,1,100,2));f.parse_line(frame(f,1,2,300,13))
        generation=f.track_generation;cursor=f.track_cursor
        f.parse_line(frame(f,2,1,400,2)+'F')
        self.assertEqual(f.track_generation,generation)
        self.assertEqual(f.track_cursor,cursor)

    def test_reboot_without_boot_text_resets_but_tick_rollover_does_not(self):
        f=SharedFeed();f.parse_line(frame(f,1,1,100,2));f.parse_line(frame(f,1,2,300,15))
        generation=f.track_generation
        f.parse_line(frame(f,1,1,10,2))
        self.assertGreater(f.track_generation,generation)
        self.assertEqual(len(f.track),1)
        self.assertEqual(f.latest_position['mcu_ms'],10)
        g=SharedFeed();g.parse_line(frame(g,1,1,0xfffffff0,2))
        generation=g.track_generation;g.parse_line(frame(g,1,2,20,14))
        self.assertEqual(g.track_generation,generation)

if __name__=='__main__':unittest.main(verbosity=2)
