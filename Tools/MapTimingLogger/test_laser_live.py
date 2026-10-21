"""Offline integration checks for the real map and firmware MT3 wire layout."""

import binascii
import unittest
from unittest.mock import patch
from types import SimpleNamespace

from laser_live_server import COUNTS_PER_REV, WHEEL_CIRCUMFERENCE_MM, LaserFeed


class LaserLiveIntegrationTests(unittest.TestCase):
    def setUp(self):
        # Constructing/parsing a LaserFeed must never open an actual port.
        self.serial_guard = patch("laser_live_server.serial.Serial",
                                  side_effect=AssertionError("No serial access in tests"))
        self.serial_guard.start()
        self.addCleanup(self.serial_guard.stop)
        self.host_ms = 10000.0
        for target, replacement in (
            ("laser_live_server.time.monotonic", lambda: self.host_ms / 1000.0),
            ("laser_live_server.time.time", lambda: 1700000000 + self.host_ms / 1000.0),
        ):
            clock = patch(target, new=replacement)
            clock.start()
            self.addCleanup(clock.stop)
        self.feed = LaserFeed("TEST_ONLY_NO_PORT", 115200)
        self.feed.connected = True
        self.sequence = 0

    def timing(self, event, source=0, progress=0, argument=0, drive_mode=0,
               yaw=-1234, yaw_valid=1, yaw_age=8, yaw_frames=42):
        """Use firmware's 20 integer fields; do not reuse the server CRC code."""
        self.sequence += 1
        counts = round(progress * COUNTS_PER_REV / WHEEL_CIRCUMFERENCE_MM)
        if drive_mode:
            counts = -counts
        # A/B/C/D physical order and logical signs match NewFrontDriveMap.
        encoders = [-counts, counts, -counts, counts]
        values = [1, self.sequence, source, event, 0, 1600, 0, 54, 53,
                  argument, yaw, yaw_valid, yaw_age, yaw_frames,
                  *encoders, 0, drive_mode]
        payload = "MT3," + ",".join(str(value) for value in values)
        crc = binascii.crc_hqx(payload.encode("ascii"), 0xFFFF)
        return f"{payload}*{crc:04X}"

    def send(self, event, source=0, progress=0, **values):
        self.host_ms = 10000.0 + source
        self.feed.parse_line(self.timing(event, source, progress, **values))

    def begin(self, drive_mode=0):
        self.send(2, argument=self.feed.map_signature, drive_mode=drive_mode)
        self.send(16, drive_mode=drive_mode)

    def test_real_map_start_leg_samples_project_without_adding_forecast_to_track(self):
        self.assertEqual(self.feed.nodes[54], (3300, 1500))
        self.assertEqual(self.feed.nodes[53], (3000, 1500))
        self.assertEqual(self.feed.edges[frozenset((54, 53))], 300)
        self.begin()
        self.send(14, 100, 50)
        self.send(14, 200, 100)
        observed = self.feed.snapshot()
        self.assertTrue(observed["map_match"])
        self.assertAlmostEqual(observed["position"]["x_mm"], 3200, delta=0.2)
        self.assertEqual(observed["position"]["y_mm"], 1500)
        self.host_ms += 100
        predicted = self.feed.snapshot()
        self.assertAlmostEqual(predicted["projection"]["x_mm"], 3150, delta=0.3)
        self.assertAlmostEqual(predicted["projection"]["anchor_x_mm"], 3200, delta=0.2)
        self.assertTrue(predicted["projection"]["predicted"])
        self.assertEqual(predicted["projection"]["heading"], 270)
        self.assertEqual(predicted["track_cursor"], observed["track_cursor"])
        self.assertEqual(predicted["position"], observed["position"])

    def test_port_switch_clears_old_telemetry_and_requires_existing_port(self):
        self.feed.parse_line("LASER DIR=RIGHT D=300 C=90 STATUS=VALID")
        self.begin()
        original_generation = self.feed.snapshot()["track_generation"]
        with patch("laser_live_server.list_ports.comports", return_value=[SimpleNamespace(device="COM9"), SimpleNamespace(device="COM13")]):
            self.feed.select_port("com13")
            data = self.feed.snapshot()
            self.assertEqual(data["port"], "COM13")
            self.assertFalse(data["connected"])
            self.assertEqual(data["valid_frames"], 0)
            self.assertIsNone(data["channels"]["3"]["mm"])
            self.assertIsNone(data["position"])
            self.assertEqual(data["track_generation"], original_generation + 1)
            with self.assertRaises(ValueError):
                self.feed.select_port("COM99")
            self.assertEqual(self.feed.snapshot()["port"], "COM13")

    def test_reader_discards_inflight_old_port_data_and_closes_before_switch(self):
        feed = self.feed
        readers = []

        class Finished(Exception):
            pass

        class Reader:
            def __init__(self, **_kwargs):
                self.is_open = False
                self.reads = 0
                readers.append(self)

            def open(self):
                if len(readers) > 1:
                    self_test.assertFalse(readers[0].is_open)
                self_test.assertFalse(self.dtr)
                self_test.assertFalse(self.rts)
                self.is_open = True

            def close(self):
                self.is_open = False

            def read(self, _size):
                self.reads += 1
                if len(readers) == 1:
                    feed.select_port("COM13")
                    return b"LASER DIR=RIGHT D=300 C=90 STATUS=VALID\n"
                if self.reads == 1:
                    return b"LASER DIR=LEFT D=400 C=90 STATUS=VALID\n"
                raise Finished()

        self_test = self
        with patch("laser_live_server.list_ports.comports", return_value=[SimpleNamespace(device="COM13")]), patch("laser_live_server.serial.Serial", side_effect=Reader):
            with self.assertRaises(Finished):
                feed.run()
        self.assertEqual(len(readers), 2)
        self.assertTrue(all(not reader.is_open for reader in readers))
        self.assertEqual(feed.snapshot()["valid_frames"], 1)
        self.assertIsNone(feed.snapshot()["channels"]["3"]["mm"])
        self.assertEqual(feed.snapshot()["channels"]["2"]["mm"], 400)

    def test_gray_arrival_is_physical_front_in_forward_travel(self):
        self.begin()
        self.send(14, 100, 50)
        self.assertEqual(self.feed.snapshot()["gray"], {"front": None, "rear": None})
        self.send(15, 500, 300, argument=0xC03)
        data = self.feed.snapshot()
        self.assertEqual(data["gray"]["front"]["mask"], 0xC03)
        self.assertEqual(data["gray"]["front"]["node"], 53)
        self.assertEqual(data["gray"]["front"]["source"], "NODE_ARRIVE")
        self.assertTrue(data["gray"]["front"]["filtered"])
        self.assertIsNone(data["gray"]["rear"])
        self.assertEqual(data["position"]["x_mm"], 3000)
        self.assertEqual(data["projection"]["mode"], "node_confirmed")

    def test_gray_arrival_is_physical_rear_in_reverse_travel(self):
        self.begin(drive_mode=1)
        self.send(14, 100, 50, drive_mode=1)
        self.assertAlmostEqual(self.feed.snapshot()["position"]["x_mm"], 3250, delta=0.2)
        self.send(15, 500, 300, argument=0x081, drive_mode=1)
        data = self.feed.snapshot()
        self.assertIsNone(data["gray"]["front"])
        self.assertEqual(data["gray"]["rear"]["mask"], 0x081)
        self.assertEqual(data["gray"]["rear"]["node"], 53)
        self.assertEqual(data["position"]["heading"], 90)

    def test_yaw_wire_indices_and_sensor_age_are_preserved(self):
        self.begin()
        self.send(14, 100, 50, yaw=-12345, yaw_valid=0, yaw_age=1200, yaw_frames=123456)
        self.host_ms += 250
        gyro = self.feed.snapshot()["gyro"]
        self.assertEqual(gyro["yaw_deg"], -123.45)
        self.assertFalse(gyro["valid"])
        self.assertEqual(gyro["sensor_age_ms"], 1200)
        self.assertEqual(gyro["data_age_ms"], 1450)
        self.assertEqual(gyro["frames"], 123456)
        self.assertEqual(gyro["mcu_ms"], 100)
        self.send(14, 500, 100, yaw=9001, yaw_valid=1, yaw_age=3)
        self.assertTrue(self.feed.snapshot()["gyro"]["valid"])
        self.assertEqual(self.feed.snapshot()["gyro"]["yaw_deg"], 90.01)

    def test_map_ready_resets_path_projection_gray_and_gyro(self):
        self.begin()
        self.send(15, 500, 300, argument=0xC00)
        generation = self.feed.snapshot()["track_generation"]
        self.feed.parse_line(
            "MAP_READY R1600 START=N054 HEADING=W FIRST=N053 STEPS=15 "
            f"SIGNATURE={self.feed.map_signature}")
        data = self.feed.snapshot()
        self.assertIsNone(data["gyro"])
        self.assertEqual(data["gray"], {"front": None, "rear": None})
        self.assertIsNone(data["projection"])
        self.assertEqual(data["track_generation"], generation + 1)
        self.assertEqual(data["position"]["source"], "MAP_READY")
        self.assertEqual((data["position"]["x_mm"], data["position"]["y_mm"]), (3300, 1500))
        self.assertEqual(data["position"]["heading"], 270)
        self.assertEqual(len(self.feed.path_since(0)["points"]), 1)

    def test_manual_start_clears_trail_but_preserves_sensor_and_fault_data(self):
        self.begin()
        self.send(15, 500, 300, argument=0xC00)
        self.feed.parse_line("LASER DIR=FRONT D=400 C=90 STATUS=VALID")
        self.feed.parse_line("ROUTE_FAULT=LOC_GRAY_MISMATCH FROM=FOLLOW")
        before = self.feed.snapshot()
        reset = self.feed.reset_position_view()
        self.assertEqual(reset["position"]["source"], "VIEW_RESET")
        self.assertEqual((reset["position"]["x_mm"], reset["position"]["y_mm"], reset["position"]["heading"]), (3300, 1500, 270))
        self.assertIsNone(reset["projection"])
        self.assertGreater(reset["track_cursor"], before["track_cursor"])
        self.assertEqual(reset["track_generation"], before["track_generation"] + 1)
        for key in ("channels", "gray", "gyro", "vehicle_state", "fault_reason", "raw_log_path", "connected"):
            self.assertEqual(reset[key], before[key], key)
        self.assertEqual(len(self.feed.path_since(before["track_cursor"])["points"]), 1)
        self.send(14, 600, 200)
        self.feed.parse_line("LOC NODE=N053 X_MM=3000 Y_MM=1500 BODY=3 VALID=1")
        self.assertEqual(self.feed.snapshot()["position"], reset["position"])
        self.send(16, 700)
        self.assertEqual(self.feed.snapshot()["position"]["source"], "MT3")

    def test_manual_start_uses_latest_firmware_route_start(self):
        self.feed.parse_line(
            "MAP_READY R001 START=N081 HEADING=N FIRST=N066 STEPS=210 "
            f"SIGNATURE={self.feed.map_signature}")
        reset = self.feed.reset_position_view()
        self.assertEqual((reset["position"]["x_mm"], reset["position"]["y_mm"]), self.feed.nodes[81])
        self.assertEqual(reset["position"]["heading"], 0)

    def test_buffered_tw_pin_dump_does_not_replace_current_gray(self):
        self.begin()
        self.send(15, 500, 300, argument=0x003)
        before = self.feed.snapshot()
        self.feed.parse_line(
            "TW_PIN T=400 EV=1 N=20 HIGH_C=0 HIGH_D=0 CHECK_C=0 BAD_C=0 "
            "CHECK_D=0 BAD_D=0 PHASE=0 IN=0 GRAY=FFF,FFF POS=999 HEAD=999 AGE=1")
        after = self.feed.snapshot()
        self.assertEqual(after["gray"], before["gray"])
        self.assertEqual(after["gyro"], before["gyro"])
        self.assertEqual(after["position"], before["position"])

    def test_uartch4_probe_hex_does_not_crash_or_replace_laser_data(self):
        self.feed.parse_line(
            "UARTCH4_TRY I=13 BAUD=115200 CH=1 CRLF=0 TX=1 ACK=0 "
            "RX=536 ERR=0 MASK=0 MS=300 HEX=00B100642BAA056700B000642A"
        )
        self.feed.parse_line("LASER DIR=RIGHT D=300 C=100 STATUS=VALID")
        self.assertEqual(self.feed.snapshot()["channels"]["3"]["mm"], 300)
        self.assertEqual(self.feed.snapshot()["valid_frames"], 1)

    def test_wrong_crc_does_not_update_position_gyro_gray_or_sequence(self):
        self.begin()
        self.send(14, 100, 50)
        before = self.feed.snapshot()
        line = self.timing(15, 200, 300, argument=0xFFF, yaw=17000)
        payload, crc = line.split("*")
        corrupt = f"{payload}*{int(crc, 16) ^ 0xFFFF:04X}"
        self.feed.parse_line(corrupt)
        after = self.feed.snapshot()
        for key in ("position", "gyro", "gray", "projection", "track_cursor"):
            self.assertEqual(after[key], before[key], key)
        self.assertEqual(self.feed.last_sequence, self.sequence - 1)
        # The discarded sequence must still be accepted when sent correctly.
        self.feed.parse_line(line)
        self.assertEqual(self.feed.snapshot()["gray"]["front"]["mask"], 0xFFF)

    def test_road_transition_events_after_arrival_do_not_rewind_to_start(self):
        self.begin()
        self.send(15, 500, 300, argument=0xC00)
        self.send(3, 500)
        self.send(5, 500)
        self.send(6, 600)
        self.send(7, 900)
        self.send(14, 1000, 80)
        data = self.feed.snapshot()
        self.assertEqual(data["position"]["x_mm"], 3000)
        self.assertEqual(data["projection"]["x_mm"], 3000)
        self.assertFalse(data["projection"]["predicted"])


if __name__ == "__main__":
    unittest.main()
