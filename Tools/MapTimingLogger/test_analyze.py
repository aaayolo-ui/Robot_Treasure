import unittest

from analyze import analyze, crc16


MAP = {"calibrated": False, "segments": [
    {"id": "S001", "start": "N001", "end": "N002", "length_m": 0.6},
    {"id": "S003", "start": "N002", "end": "N003", "length_m": 0.6}]}


def line(seq, time, event, step=0, arg=0, yaw=0, frames=None, lost=0, age=0):
    payload = (f"MT1,1,{seq},{time},{event},{step},{arg},{yaw},1,{age},"
               f"{seq if frames is None else frames},0,0,0,0,{lost}").encode()
    return payload + f"*{crc16(payload):04X}\r\n".encode()


def good_run():
    return b"".join([line(1, 100, 2), line(2, 1000, 3, 1),
                     line(3, 1000, 4, 1, 1), line(4, 1200, 5, 1, 1),
                     line(5, 1300, 6, 1, 1), line(6, 1600, 7, 1, yaw=5000),
                     line(7, 1800, 8, 1, yaw=9000), line(8, 1900, 9, 1, 3, yaw=9000),
                     line(9, 2100, 13, 1, 1, yaw=9000)])


class AnalyzeTests(unittest.TestCase):
    def assignment(self):
        return {"session": "r1.1", "start_sequence": 2, "end_sequence": 8,
                "segments": [{"id": "S001", "from": "N001", "to": "N002"}],
                "measured_interval_m": 0.54, "distance_calibrated": True,
                "route_verified": True, "boundary_note": "measured reference interval"}

    def test_crc_reference(self):
        self.assertEqual(crc16(b"123456789"), 0x29B1)

    def test_turn_uses_fifty_degree_rotation_not_ninety(self):
        turn = analyze(good_run(), MAP)["sessions"][0]["turns"][0]
        self.assertTrue(turn["valid"])
        self.assertAlmostEqual(turn["rotation_s"], 0.3)
        self.assertEqual(turn["rotation_angle_deg"], 50)
        self.assertAlmostEqual(turn["mean_rotation_deg_s"], 50 / 0.3)
        self.assertAlmostEqual(turn["full_flow_s"], 0.9)
        self.assertTrue(turn["quarter_turn_angle_check"])

    def test_calibrated_speed_uses_interval_distance(self):
        interval = analyze(good_run(), MAP, {"intervals": [self.assignment()]})["intervals"][0]
        self.assertTrue(interval["valid"])
        self.assertAlmostEqual(interval["measured_speed_m_s"], 0.6)
        self.assertAlmostEqual(interval["nominal_map_length_m"], 0.6)

    def test_nominal_map_does_not_become_measured_speed(self):
        assignment = self.assignment()
        assignment["distance_calibrated"] = False
        interval = analyze(good_run(), MAP, {"intervals": [assignment]})["intervals"][0]
        self.assertFalse(interval["valid"])
        self.assertIsNone(interval["measured_speed_m_s"])

    def test_host_noise_and_queued_deliveries_do_not_change_times(self):
        raw = b"ordinary debug\r\n" + good_run().replace(b"\r\n", b"\r\nHOST_RX=999999\r\n")
        self.assertEqual(analyze(raw, MAP)["sessions"][0]["turns"],
                         analyze(good_run(), MAP)["sessions"][0]["turns"])

    def test_wraparound(self):
        raw = line(1, 2**32 - 100, 2) + line(2, 100, 13, arg=1)
        session = analyze(raw, MAP)["sessions"][0]
        self.assertTrue(session["valid"])
        self.assertEqual(session["records"][-1]["elapsed_ms"], 200)

    def test_corruption_blocks_speed(self):
        raw = good_run().replace(b",1000,3,", b",1001,3,")
        report = analyze(raw, MAP, {"intervals": [self.assignment()]})
        self.assertEqual(len(report["corrupt_lines"]), 1)
        self.assertFalse(report["sessions"][0]["valid"])
        self.assertIsNone(report["intervals"][0]["measured_speed_m_s"])

    def test_sequence_gap_and_firmware_overflow(self):
        for raw in [line(1, 100, 2) + line(3, 300, 13, arg=1),
                    line(1, 100, 2) + line(2, 300, 13, arg=1, lost=1)]:
            self.assertFalse(analyze(raw, MAP)["sessions"][0]["valid"])

    def test_aborted_and_partial_capture(self):
        for raw in [line(1, 100, 2), line(1, 100, 2) + line(2, 300, 13, arg=2)]:
            self.assertFalse(analyze(raw, MAP)["sessions"][0]["valid"])

    def test_reboot_run_number_can_repeat(self):
        sessions = analyze(good_run() + good_run(), MAP)["sessions"]
        self.assertEqual([s["id"] for s in sessions], ["r1.1", "r1.2"])
        self.assertTrue(all(s["valid"] for s in sessions))

    def test_incorrect_map_assignment(self):
        assignment = self.assignment()
        assignment["segments"][0]["to"] = "N003"
        interval = analyze(good_run(), MAP, {"intervals": [assignment]})["intervals"][0]
        self.assertFalse(interval["valid"])
        self.assertIsNone(interval["measured_speed_m_s"])

    def test_disconnected_route(self):
        assignment = self.assignment()
        assignment["segments"].append({"id": "S003", "from": "N003", "to": "N002"})
        report = analyze(good_run(), MAP, {"intervals": [assignment]})
        self.assertIn("disconnected_segment_chain", report["intervals"][0]["issues"])

    def test_stale_yaw_keeps_duration_but_not_angular_speed(self):
        raw = good_run().replace(line(6, 1600, 7, 1, yaw=5000),
                                 line(6, 1600, 7, 1, yaw=5000, age=151))
        turn = analyze(raw, MAP)["sessions"][0]["turns"][0]
        self.assertAlmostEqual(turn["rotation_s"], 0.3)
        self.assertIsNone(turn["mean_rotation_deg_s"])

    def test_reversed_source_clock_invalidates(self):
        raw = line(1, 1000, 2) + line(2, 900, 13, arg=1)
        self.assertFalse(analyze(raw, MAP)["sessions"][0]["valid"])

    def test_zero_duration_and_infinite_distance_are_rejected(self):
        assignment = self.assignment()
        assignment["measured_interval_m"] = float("inf")
        self.assertFalse(analyze(good_run(), MAP, {"intervals": [assignment]})["intervals"][0]["valid"])
        assignment = self.assignment()
        assignment["end_sequence"] = 3
        interval = analyze(good_run(), MAP, {"intervals": [assignment]})["intervals"][0]
        self.assertIn("nonpositive_duration", interval["issues"])
        self.assertIsNone(interval["measured_speed_m_s"])


if __name__ == "__main__":
    unittest.main()
