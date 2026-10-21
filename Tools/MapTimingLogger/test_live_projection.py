"""MCU event timeline tests; no serial port or motor access."""

import unittest
from unittest.mock import patch

from live_projection import RoadProjection


class RoadProjectionTests(unittest.TestCase):
    def setUp(self):
        self.nodes = {1: (0, 0), 2: (1000, 0), 3: (1000, 1000)}
        self.edges = {frozenset((1, 2)): 1000, frozenset((2, 3)): 1000}
        self.projection = RoadProjection(self.nodes, self.edges, 500, 250)
        self.sequence = 0

    def event(self, event, source=0, progress=0, received=None, **changes):
        self.sequence += 1
        record = dict(run=1, sequence=self.sequence, mcu_ms=source, event=event,
                      from_node=1, to_node=2, progress_mm=progress, heading=90,
                      drive_mode=0, lost=0,
                      received_ms=source + 10000 if received is None else received)
        record.update(changes)
        self.projection.on_timing(**record)

    def test_elapsed_host_time_projects_between_samples_without_moving_anchor(self):
        self.event(16, source=100)
        self.event(14, source=200, progress=50)
        self.event(14, source=300, progress=100)
        view = self.projection.view(now_ms=10400)
        self.assertEqual(view["x_mm"], 150)
        self.assertEqual(view["anchor_x_mm"], 100)
        self.assertTrue(view["predicted"])
        self.assertEqual(view["speed_source"], "samples")
        self.assertEqual(self.projection.view(now_ms=10300)["x_mm"], 100)

    def test_default_clock_is_monotonic_and_can_be_injected(self):
        with patch("live_projection.time.monotonic", return_value=10):
            self.projection.on_timing(1, 1, 100, 16, 1, 2, 0, 90, 0, 0)
        with patch("live_projection.time.monotonic", return_value=10.2):
            self.assertEqual(self.projection.view()["x_mm"], 100)

    def test_reverse_uses_reverse_speed_and_keeps_body_heading(self):
        self.event(16, drive_mode=1, from_node=2, to_node=1, heading=90)
        view = self.projection.view(now_ms=10400)
        self.assertEqual(view["x_mm"], 900)
        self.assertEqual(view["heading"], 90)
        self.assertEqual(view["speed_mm_s"], 250)
        self.assertEqual(view["speed_source"], "configured_reverse")

    def test_pause_and_arrival_never_continue_on_old_road(self):
        self.event(16)
        self.event(14, 100, 50)
        self.event(5, 150, 75)
        self.assertEqual(self.projection.view(10500)["x_mm"], 75)
        self.assertEqual(self.projection.view(10500)["mode"], "paused")
        self.event(6, 200, 500)
        self.event(7, 300, 700)
        self.assertEqual(self.projection.view(10600)["x_mm"], 75)
        self.event(14, 400, 100)
        self.assertEqual(self.projection.view(10450)["mode"], "predicting")
        self.event(15, 500, 800)
        self.assertEqual(self.projection.view(11000)["x_mm"], 1000)
        self.assertEqual(self.projection.view(11000)["mode"], "node_confirmed")
        self.event(8, 600, 200)
        self.event(14, 700, 300)
        self.assertEqual(self.projection.view(10800)["x_mm"], 1000)
        self.event(16, 800, 0, from_node=2, to_node=3, heading=180)
        self.assertEqual(self.projection.view(10900)["x_mm"], 1000)
        self.assertGreater(self.projection.view(10900)["y_mm"], 0)

    def test_reverse_ready_on_old_leg_does_not_resume_prediction(self):
        self.event(16)
        self.event(18, 100)
        self.event(19, 200, drive_mode=1)
        self.event(14, 300, 50, drive_mode=1)
        self.assertEqual(self.projection.view(10400)["mode"], "paused")
        self.assertFalse(self.projection.view(10400)["predicted"])

    def test_serial_queue_age_is_not_host_interval_speed(self):
        self.event(16, 0, received=10000)
        self.event(14, 100, 50, received=10350)
        self.event(14, 200, 100, received=10400)
        view = self.projection.view(10400)
        self.assertEqual(view["speed_mm_s"], 500)
        self.assertEqual(view["telemetry_age_ms"], 200)
        self.assertEqual(view["x_mm"], 200)

    def test_latest_source_age_tracks_queued_pause_events_not_road_anchor(self):
        self.assertIsNone(self.projection.source_age_ms(10000))
        self.event(16, 0, received=10000)
        self.event(14, 100, 50, received=10100)
        self.event(5, 200, 75, received=10500)
        # A newer gyro/rotation event arrives in the same delayed event batch.
        # The road anchor remains at source=200; latest telemetry is source=300.
        self.event(6, 300, 75, received=10600)
        self.assertEqual(self.projection.source_age_ms(10600), 300)
        self.assertEqual(self.projection.view(10600)["telemetry_age_ms"], 400)
        self.assertEqual(self.projection.source_age_ms(10650), 350)
        self.event(11, 400, 75, received=10650)
        self.assertEqual(self.projection.source_age_ms(10650), 250)
        self.projection.reset()
        self.assertIsNone(self.projection.source_age_ms(11000))

    def test_missing_sequence_requires_two_consecutive_samples(self):
        self.event(16)
        self.event(14, 100, 50)
        self.sequence += 1
        self.event(14, 200, 100, lost=1)
        self.assertEqual(self.projection.view(10300)["mode"], "awaiting_samples")
        self.assertEqual(self.projection.view(10300)["x_mm"], 100)
        self.event(8, 220, 110, lost=1)
        self.event(14, 300, 150, lost=1)
        self.assertEqual(self.projection.view(10350)["mode"], "awaiting_samples")
        self.event(14, 400, 200, lost=1)
        self.assertEqual(self.projection.view(10450)["mode"], "predicting")
        self.assertEqual(self.projection.view(10450)["x_mm"], 225)

    def test_source_gap_does_not_learn_a_slow_speed_and_requires_recovery(self):
        self.event(16)
        self.event(14, 100, 50)
        self.event(14, 2000, 60)
        self.assertEqual(self.projection.view(12000)["speed_mm_s"], 500)
        self.assertEqual(self.projection.view(12000)["mode"], "awaiting_samples")
        self.event(14, 2100, 110)
        self.assertEqual(self.projection.view(12150)["mode"], "predicting")

    def test_freezes_after_700ms_and_disconnect_does_not_move_forever(self):
        self.event(16)
        first = self.projection.view(10701)
        second = self.projection.view(14000)
        self.assertEqual(first["mode"], "stale")
        self.assertEqual(first["x_mm"], 350)
        self.assertEqual(second["x_mm"], 350)
        self.assertEqual(self.projection.view(10100, connected=False)["mode"], "stale")

    def test_prediction_stops_before_node_but_anchor_past_margin_does_not_reverse(self):
        self.event(16)
        self.event(14, 100, 950)
        view = self.projection.view(10300)
        self.assertEqual(view["mode"], "waiting_node")
        self.assertEqual(view["x_mm"], 965)
        self.event(14, 200, 980)
        self.assertEqual(self.projection.view(10600)["x_mm"], 980)
        self.event(15, 300, 990)
        self.assertEqual(self.projection.view(11000)["x_mm"], 1000)

    def test_short_road_keeps_eight_percent_margin(self):
        projection = RoadProjection({1: (0, 0), 2: (100, 0)},
                                    {frozenset((1, 2)): 100}, 500, 250)
        projection.on_timing(1, 1, 0, 16, 1, 2, 0, 90, 0, 0, received_ms=0)
        self.assertEqual(projection.view(500)["x_mm"], 92)

    def test_start_does_not_restart_old_road_or_reset_clock(self):
        self.event(16)
        self.event(14, 100, 50)
        self.event(2, 200, run=2, sequence=1)
        self.assertEqual(self.projection.view(10500)["mode"], "paused")
        self.assertEqual(self.projection.view(10500)["x_mm"], 50)
        self.event(14, 300, 100, run=2, sequence=2)
        self.assertEqual(self.projection.view(10600)["x_mm"], 50)
        self.event(16, 400, run=2, sequence=3)
        self.assertEqual(self.projection.view(10500)["x_mm"], 50)

    def test_32bit_source_wrap_is_not_a_reboot(self):
        self.event(16, source=0xFFFFFFC0, received=10000)
        self.event(14, source=36, progress=50, received=10100)
        self.event(14, source=136, progress=100, received=10200)
        self.assertEqual(self.projection.view(10300)["x_mm"], 150)
        self.assertEqual(self.projection.view(10300)["speed_mm_s"], 500)

    def test_duplicate_records_are_ignored(self):
        self.event(16)
        self.event(14, 100, 50)
        self.event(14, 100, 800, sequence=2)
        self.assertEqual(self.projection.view(10100)["x_mm"], 50)

    def test_junction_speed_subtracts_known_stopped_time(self):
        self.event(16)
        self.event(5, 1000, 250)
        self.event(14, 1500, 250)
        self.event(15, 2500, 1000)
        # 1000 mm in 2000 ms moving; the 500 ms stop is excluded.
        self.assertEqual(self.projection.view(12500)["speed_mm_s"], 500)
        self.assertEqual(self.projection.view(12500)["speed_source"], "junction_timing")

    def test_junction_speed_is_not_learned_across_missing_records(self):
        self.event(16)
        self.event(14, 100, 50)
        self.sequence += 1
        self.event(15, 1000, 1000, lost=1)
        self.assertEqual(self.projection.view(11000)["speed_source"], "configured_forward")

    def test_end_and_explicit_stop_hold_until_new_leg(self):
        self.event(16)
        self.event(13, 100, 50)
        self.event(14, 200, 100)
        self.assertEqual(self.projection.view(10300)["x_mm"], 50)
        self.projection.stop("fault")
        self.assertEqual(self.projection.view(10400)["mode"], "paused")
        self.projection.reset()
        self.assertIsNone(self.projection.view(10400))
        self.event(16, source=0, received=20000, sequence=1)
        self.assertEqual(self.projection.view(20100)["x_mm"], 50)


if __name__ == "__main__":
    unittest.main()
