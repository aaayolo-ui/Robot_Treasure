"""Serve four laser readings and map positions from the existing UART stream.

The serial connection is receive-only. The server listens on loopback only.
"""

from __future__ import annotations

import argparse
import json
import math
import re
import threading
import time
import webbrowser
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, urlsplit

import serial
from serial.tools import list_ports

from live_projection import RoadProjection


HEX_LINE = re.compile(r"HEX=([0-9A-Fa-f ]+)")
STRATEGY_LASER = re.compile(
    r"LASER DIR=(FRONT|LEFT|RIGHT|REAR) D=(\d+) C=(\d+) STATUS=([A-Z_]+)"
)
STRATEGY_STATE = re.compile(r"\bSTATE=([A-Z_]+) FAULT=([A-Z0-9_]+)")
BOOT_STATE = re.compile(r"^BOOT_STATUS .*\bSTATE=([A-Z_]+).*\bREASON=([A-Z0-9_]+)")
ROUTE_FAULT = re.compile(r"^ROUTE_FAULT=([A-Z0-9_]+) FROM=([A-Z_]+)")
LOC_DIAG = re.compile(
    r"^LOC NODE=N(?P<node>\d+).*? BODY=(?P<body>\d+).*? VALID=(?P<valid>[01]) "
    r"PHASE=(?P<phase>\d+) YAW_ERR=(?P<yaw_error>-?\d+) "
    r"YAW_AGE=(?P<yaw_age>\d+) CW_SIGN=(?P<sign>-?\d+)"
)
LOC_LINE = re.compile(
    r"LOC NODE=N(?P<node>\d+) X_MM=(?P<x>-?\d+) Y_MM=(?P<y>-?\d+) "
    r"BODY=(?P<body>\d+).*? VALID=(?P<valid>[01])"
)
MAP_READY_LINE = re.compile(
    r"^MAP_READY R(?P<run>\d+) START=N(?P<start>\d+) HEADING=(?P<heading>[NESW]) "
    r"FIRST=N(?P<first>\d+) STEPS=\d+ SIGNATURE=(?P<signature>\d+)$"
)
PAGE = Path(__file__).with_name("laser_live.html")
MAP_FILE = Path(__file__).with_name("full_map_route.json")
RAW_LOG_DIR = Path(__file__).with_name("logs")
COUNTS_PER_REV = 1560
WHEEL_CIRCUMFERENCE_MM = 204.204
TRACK_LIMIT = 5000
MOTION_PROFILE = PAGE.parent.parent / "TreasurePursuitSimulator" / "config.json"


def motion_speeds() -> tuple[float, float]:
    """Nominal-distance measurements are a prior, not an absolute position fix."""
    try:
        config = json.loads(MOTION_PROFILE.read_text(encoding="utf-8"))
        forward = float(config["thief_speed_m_s"]) * 1000
        reverse = forward * float(config["measured_motion"]["reverse_speed_ratio"])
        if all(math.isfinite(value) and 50 <= value <= 3000 for value in (forward, reverse)):
            return forward, reverse
    except (OSError, ValueError, KeyError, TypeError):
        pass
    return 674.157, 678.733


class LaserFeed:
    def __init__(self, port: str, baud: int) -> None:
        source_map = json.loads(MAP_FILE.read_text(encoding="utf-8"))
        self.map_signature = source_map["map_route_signature"]
        self.nodes = {
            int(node["id"][1:]): tuple(round(value * 1000) for value in node["xy_m"])
            for node in source_map["nodes"]
        }
        self.edges = {
            frozenset((int(segment["start"][1:]), int(segment["end"][1:]))):
                round(segment["length_m"] * 1000)
            for segment in source_map["segments"]
        }
        self.projection = RoadProjection(self.nodes, self.edges, *motion_speeds())
        self.map_view = {
            "signature": self.map_signature,
            "calibrated": source_map["calibrated"],
            "nodes": [{"id": node["id"], "xy_m": node["xy_m"]} for node in source_map["nodes"]],
            "segments": [{"id": segment["id"], "start": segment["start"], "end": segment["end"]}
                         for segment in source_map["segments"]],
        }
        self.port = port
        self.baud = baud
        RAW_LOG_DIR.mkdir(exist_ok=True)
        self.raw_log_path = RAW_LOG_DIR / f"laser_live_{int(time.time() * 1000)}.log"
        self.lock = threading.RLock()
        self.serial_generation = 0
        self.serial_changed = threading.Event()
        self.connected = False
        self.error = ""
        self.last_receive_ms = 0
        self.valid_frames = 0
        self.vehicle_state = None
        self.fault_reason = None
        self.status_time_ms = 0
        self.fault_from = None
        self.localization_diagnostic = None
        self.gyro = None
        self.gyro_received_ms = 0
        self.gray = {"front": None, "rear": None}
        self.channels = {
            str(ch): {"mm": None, "confidence": None, "time_ms": 0, "status": None}
            for ch in range(1, 5)
        }
        self.frame_bytes = bytearray()
        self.map_match = None
        self.latest_position = None
        self.track = []
        self.track_cursor = 0
        self.track_generation = 0
        self.leg = None
        self.last_run = None
        self.last_sequence = 0
        # Current thief firmware starts at N054 facing N053 (west).
        # A valid MAP_READY replaces this fallback with its reported route start.
        self.initial_pose = (54, 270)
        self.position_reset_pending = False

    def reset_position_view(self) -> dict:
        """Reset the dashboard only; no bytes are sent to the vehicle."""
        with self.lock:
            node, heading = self.initial_pose
            self.track.clear()
            self.latest_position = None
            self.leg = None
            self.track_generation += 1
            self.projection.reset()
            self.position_reset_pending = True
            self._add_position(*self.nodes[node], heading, "VIEW_RESET", None, None, None)
            return self.snapshot()

    def select_port(self, port: str) -> None:
        available = {item.device.upper(): item.device for item in list_ports.comports()}
        selected = available.get(port.upper())
        if selected is None:
            raise ValueError("串口不存在或已拔出")
        with self.lock:
            if selected == self.port:
                return
            self.port = selected
            self.serial_generation += 1
            self.connected = False
            self.error = "正在切换串口"
            self.last_receive_ms = 0
            self.valid_frames = 0
            self.vehicle_state = None
            self.fault_reason = None
            self.fault_from = None
            self.status_time_ms = 0
            self.localization_diagnostic = None
            self.gyro = None
            self.gray = {"front": None, "rear": None}
            self.channels = {
                str(ch): {"mm": None, "confidence": None, "time_ms": 0, "status": None}
                for ch in range(1, 5)
            }
            self.frame_bytes.clear()
            self.map_match = None
            self.latest_position = None
            self.track.clear()
            self.track_cursor = 0
            self.track_generation += 1
            self.leg = None
            self.last_run = None
            self.last_sequence = 0
            self.projection.reset()
            self.initial_pose = (54, 270)
            self.position_reset_pending = False
            self.raw_log_path = RAW_LOG_DIR / f"laser_live_{int(time.time() * 1000)}.log"
            self.serial_changed.set()

    def snapshot(self) -> dict:
        with self.lock:
            gyro = self.gyro.copy() if self.gyro else None
            if gyro:
                receipt_age = max(0, time.monotonic() * 1000 - self.gyro_received_ms)
                source_age = self.projection.source_age_ms()
                gyro["data_age_ms"] = round(max(receipt_age, source_age or 0)
                                             + gyro["sensor_age_ms"])
            projection = self.projection.view(connected=self.connected)
            if self.latest_position and self.latest_position["source"] in ("LOC", "MAP_READY", "VIEW_RESET"):
                projection = None
            return {
                "port": self.port,
                "baud": self.baud,
                "raw_log_path": str(self.raw_log_path),
                "connected": self.connected,
                "error": self.error,
                "last_receive_ms": self.last_receive_ms,
                "valid_frames": self.valid_frames,
                "vehicle_state": self.vehicle_state,
                "fault_reason": self.fault_reason,
                "status_time_ms": self.status_time_ms,
                "fault_from": self.fault_from,
                "localization_diagnostic": self.localization_diagnostic.copy()
                    if self.localization_diagnostic else None,
                "gyro": gyro,
                "gray": {side: value.copy() if value else None for side, value in self.gray.items()},
                "projection": projection,
                "channels": {ch: values.copy() for ch, values in self.channels.items()},
                "position": self.latest_position.copy() if self.latest_position else None,
                "map_match": self.map_match,
                "track_cursor": self.track_cursor,
                "track_generation": self.track_generation,
                "server_time_ms": int(time.time() * 1000),
            }

    def path_since(self, cursor: int) -> dict:
        with self.lock:
            return {
                "cursor": self.track_cursor,
                "generation": self.track_generation,
                "truncated": bool(self.track and cursor < self.track[0]["id"] - 1),
                "points": [point.copy() for point in self.track if point["id"] > cursor],
            }

    @staticmethod
    def _crc16(text: str) -> int:
        crc = 0xFFFF
        for value in text.encode("ascii"):
            crc ^= value << 8
            for _ in range(8):
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
        return crc

    @staticmethod
    def _forward_counts(encoders: list[int]) -> float:
        # Same logical signs and wheel order as NewFrontDriveMap / MapRoutePosition.
        return (-encoders[0] + encoders[1] - encoders[2] + encoders[3]) / 4.0

    def _add_position(self, x_mm: float, y_mm: float, heading: float,
                      source: str, run: int | None, sequence: int | None,
                      mcu_ms: int | None, break_before: bool = False) -> None:
        previous = self.latest_position
        if previous is not None:
            gap = math.hypot(x_mm - previous["x_mm"], y_mm - previous["y_mm"])
            if gap > 300:
                break_before = True
            if (gap < 3 and source == previous["source"] and
                    int(time.time() * 1000) - previous["time_ms"] < 1000):
                return
        self.track_cursor += 1
        point = {
            "id": self.track_cursor, "x_mm": round(x_mm, 1), "y_mm": round(y_mm, 1),
            "heading": round(heading % 360, 1), "source": source,
            "run": run, "sequence": sequence, "mcu_ms": mcu_ms,
            "time_ms": int(time.time() * 1000), "break_before": break_before,
        }
        self.latest_position = point
        self.track.append(point)
        if len(self.track) > TRACK_LIMIT:
            del self.track[:len(self.track) - TRACK_LIMIT]

    def _parse_timing(self, line: str) -> None:
        try:
            payload, checksum = line.strip().split("*", 1)
            if self._crc16(payload) != int(checksum, 16):
                return
            parts = payload.split(",")
            if len(parts) != 21 or parts[0] != "MT3":
                return
            values = [int(value) for value in parts[1:]]
        except (ValueError, UnicodeError):
            return
        run, sequence, mcu_ms, event = values[:4]
        from_node, to_node = values[7:9]
        argument, encoders, drive_mode = values[9], values[14:18], values[19]
        if from_node not in self.nodes or to_node not in self.nodes:
            return
        edge_length = self.edges.get(frozenset((from_node, to_node)))
        if edge_length is None:
            return
        start, end = self.nodes[from_node], self.nodes[to_node]
        travel_heading = math.degrees(math.atan2(end[0] - start[0], start[1] - end[1]))
        body_heading = travel_heading + (180 if drive_mode else 0)
        counts = self._forward_counts(encoders)
        with self.lock:
            reboot = run == self.last_run and sequence <= self.last_sequence and sequence == 1
            if reboot:
                self.leg = None
                self.map_match = None
                self.projection.reset()
            elif run == self.last_run and sequence <= self.last_sequence:
                return
            self.last_run, self.last_sequence = run, sequence
            self.gyro_received_ms = time.monotonic() * 1000
            self.gyro = {
                "yaw_deg": values[10] / 100,
                "valid": bool(values[11]),
                "sensor_age_ms": values[12],
                "frames": values[13],
                "mcu_ms": mcu_ms,
                "time_ms": int(time.time() * 1000),
            }
            if self.position_reset_pending:
                if event not in (2, 16):
                    return
                self.position_reset_pending = False
            progress = None
            if event == 2:  # MAP_TIMING_START: argument is the map signature.
                self.map_match = argument == self.map_signature
                self.leg = None
                if not self.map_match:
                    self.projection.stop("map_mismatch")
                    return
                self._add_position(*start, body_heading, "MT3", run, sequence, mcu_ms, reboot)
            if self.map_match is False:
                return
            if event == 16:  # LEG_BEGIN provides the encoder baseline for this road.
                self.leg = (from_node, to_node, counts, drive_mode)
                progress = 0.0
                self._add_position(*start, body_heading, "MT3", run, sequence, mcu_ms, reboot)
            elif event == 15:  # A gray-confirmed node arrival.
                progress = float(edge_length)
                self._add_position(*end, body_heading, "MT3", run, sequence, mcu_ms)
                if 0 <= argument <= 0xFFF:
                    side = "rear" if drive_mode else "front"
                    self.gray[side] = {
                        "mask": argument, "time_ms": int(time.time() * 1000),
                        "mcu_ms": mcu_ms, "node": to_node,
                        "source": "NODE_ARRIVE", "filtered": True,
                    }
                self.leg = None
            elif self.leg and self.leg[:2] == (from_node, to_node):
                _, _, baseline, leg_mode = self.leg
                delta = (counts - baseline) * (-1 if leg_mode else 1)
                progress = max(0.0, min(edge_length, delta * WHEEL_CIRCUMFERENCE_MM / COUNTS_PER_REV))
                fraction = progress / edge_length
                x_mm = start[0] + (end[0] - start[0]) * fraction
                y_mm = start[1] + (end[1] - start[1]) * fraction
                self._add_position(x_mm, y_mm, body_heading, "MT3", run, sequence, mcu_ms)
            self.projection.on_timing(run, sequence, mcu_ms, event, from_node, to_node,
                                      0.0 if progress is None else progress,
                                      body_heading, drive_mode, values[18],
                                      received_ms=self.gyro_received_ms)

    def _parse_location(self, line: str) -> None:
        match = LOC_LINE.search(line)
        if not match or match.group("valid") != "1":
            return
        x_mm, y_mm = int(match.group("x")), int(match.group("y"))
        if not (0 <= x_mm <= 3300 and 0 <= y_mm <= 2700):
            return
        node = int(match.group("node"))
        if node not in self.nodes:
            return
        # LOC is emitted only while stationary; reject a pose far from its confirmed node.
        if math.hypot(x_mm - self.nodes[node][0], y_mm - self.nodes[node][1]) > 160:
            return
        with self.lock:
            if self.map_match is False or self.position_reset_pending:
                return
            self._add_position(x_mm, y_mm, int(match.group("body")) * 90,
                               "LOC", self.last_run, None, None)
            self.projection.stop("stationary")

    def _parse_map_ready(self, line: str) -> None:
        match = MAP_READY_LINE.match(line.strip())
        if not match:
            return
        start = int(match.group("start"))
        first = int(match.group("first"))
        signature_matches = int(match.group("signature")) == self.map_signature
        with self.lock:
            self.track_generation += 1
            self.projection.reset()
            self.gray = {"front": None, "rear": None}
            self.gyro = None
            self.localization_diagnostic = None
            self.track.clear()
            self.latest_position = None
            self.leg = None
            self.last_run = None
            self.last_sequence = 0
            self.map_match = signature_matches
            self.position_reset_pending = False
            if (signature_matches and start in self.nodes and first in self.nodes
                    and frozenset((start, first)) in self.edges):
                heading = "NESW".index(match.group("heading")) * 90
                self.initial_pose = (start, heading)
                self._add_position(*self.nodes[start], heading, "MAP_READY",
                                   int(match.group("run")), None, None)

    def parse_line(self, line: str) -> None:
        loc_diag = LOC_DIAG.search(line)
        if loc_diag:
            with self.lock:
                self.localization_diagnostic = {
                    "node": int(loc_diag.group("node")),
                    "body": int(loc_diag.group("body")),
                    "valid": loc_diag.group("valid") == "1",
                    "phase": int(loc_diag.group("phase")),
                    "yaw_error_deg": int(loc_diag.group("yaw_error")) / 100,
                    "yaw_age_ms": int(loc_diag.group("yaw_age")),
                    "yaw_cw_sign": int(loc_diag.group("sign")),
                    "time_ms": int(time.time() * 1000),
                }
        boot = BOOT_STATE.search(line)
        strategy = STRATEGY_STATE.search(line)
        fault = ROUTE_FAULT.search(line)
        if boot or strategy or fault:
            with self.lock:
                self.status_time_ms = int(time.time() * 1000)
                if fault:
                    self.vehicle_state = "FAULT"
                    self.fault_reason = fault.group(1)
                    self.fault_from = fault.group(2)
                else:
                    self.vehicle_state = (boot or strategy).group(1)
                    self.fault_reason = (boot or strategy).group(2)
                    if self.vehicle_state != "FAULT":
                        self.fault_from = None
                if self.vehicle_state in ("FAULT", "READY", "HIDE", "HIDDEN", "DONE", "STOPPED"):
                    self.projection.stop("vehicle_stopped")
        if line.startswith("MAP_READY "):
            self._parse_map_ready(line)
            return
        if line.startswith("MT3,"):
            self._parse_timing(line)
            return
        if line.startswith("LOC NODE="):
            self._parse_location(line)
            return
        strategy_laser = STRATEGY_LASER.search(line)
        if strategy_laser:
            channel = {"FRONT": 1, "LEFT": 2, "RIGHT": 3, "REAR": 4}[strategy_laser.group(1)]
            distance, confidence = int(strategy_laser.group(2)), int(strategy_laser.group(3))
            with self.lock:
                self.channels[str(channel)] = {
                    "mm": distance if distance else None,
                    "confidence": confidence,
                    "time_ms": int(time.time() * 1000),
                    "status": strategy_laser.group(4),
                }
                self.valid_frames += 1
            return
        # UART-CH4 startup probing prints long, contiguous HEX diagnostics.
        # They are not live laser frames and must never terminate the reader.
        if line.startswith("UARTCH4_TRY "):
            return
        match = HEX_LINE.search(line)
        if not match:
            return
        try:
            self.frame_bytes.extend(bytes.fromhex(match.group(1)))
        except ValueError:
            return
        index = 0
        while index <= len(self.frame_bytes) - 10:
            data = self.frame_bytes
            ch = data[index]
            if (
                49 <= ch <= 52
                and data[index + 1] == 35
                and data[index + 2 : index + 4] == b"\xaa\x05"
                and data[index + 4] in (0x67, 0x68)
            ):
                frame = data[index + 2 : index + 10]
                if sum(frame[:7]) & 255 == frame[7]:
                    with self.lock:
                        self.channels[str(ch - 48)] = {
                            "mm": frame[4] | (frame[5] << 8),
                            "confidence": frame[6],
                            "time_ms": int(time.time() * 1000),
                            "status": None,
                        }
                        self.valid_frames += 1
                    index += 10
                    continue
            index += 1
        if index:
            del self.frame_bytes[:index]
        if len(self.frame_bytes) > 4096:
            del self.frame_bytes[:-9]

    def run(self) -> None:
        while True:
            with self.lock:
                active_port = self.port
                generation = self.serial_generation
                self.serial_changed.clear()
            port_reader = serial.Serial(port=None, baudrate=self.baud, timeout=0.25)
            port_reader.port = active_port
            port_reader.dtr = False
            port_reader.rts = False
            try:
                port_reader.open()
                with self.lock:
                    if self.serial_generation == generation:
                        self.connected = True
                        self.error = ""
                text_buffer = ""
                with (RAW_LOG_DIR / f"laser_live_{int(time.time() * 1000)}.log").open("ab", buffering=0) as raw_log:
                    with self.lock:
                        if self.serial_generation == generation:
                            self.raw_log_path = Path(raw_log.name)
                    while True:
                        with self.lock:
                            if self.serial_generation != generation:
                                break
                        chunk = port_reader.read(4096)
                        if not chunk:
                            continue
                        with self.lock:
                            if self.serial_generation != generation:
                                break
                            raw_log.write(chunk)
                            self.last_receive_ms = int(time.time() * 1000)
                            text_buffer += chunk.decode("ascii", errors="ignore")
                            lines = text_buffer.split("\n")
                            text_buffer = lines.pop()
                            for line in lines:
                                self.parse_line(line)
                            if len(text_buffer) > 16384:
                                text_buffer = text_buffer[-4096:]
            except (OSError, serial.SerialException) as exc:
                with self.lock:
                    if self.serial_generation == generation:
                        self.connected = False
                        self.error = str(exc)
                self.serial_changed.wait(2)
            finally:
                if port_reader.is_open:
                    port_reader.close()
                with self.lock:
                    if self.serial_generation == generation:
                        self.connected = False


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default="COM9")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--http-port", type=int, default=8766)
    parser.add_argument("--open-browser", action="store_true",
                        help="Open the local dashboard after its HTTP port is ready")
    args = parser.parse_args()
    feed = LaserFeed(args.port, args.baud)
    threading.Thread(target=feed.run, daemon=True).start()

    class Handler(BaseHTTPRequestHandler):
        def do_GET(self) -> None:
            request = urlsplit(self.path)
            path = request.path
            if path == "/api/laser":
                body = json.dumps(feed.snapshot(), ensure_ascii=False).encode("utf-8")
                content_type = "application/json; charset=utf-8"
            elif path == "/api/ports":
                ports = sorted((item.device for item in list_ports.comports()), key=str.upper)
                body = json.dumps({"ports": ports, "selected": feed.snapshot()["port"]}).encode("utf-8")
                content_type = "application/json; charset=utf-8"
            elif path == "/api/map":
                body = json.dumps(feed.map_view, ensure_ascii=False).encode("utf-8")
                content_type = "application/json; charset=utf-8"
            elif path == "/api/path":
                try:
                    cursor = max(0, int(parse_qs(request.query).get("after", ["0"])[0]))
                except ValueError:
                    self.send_error(400, "Invalid path cursor")
                    return
                body = json.dumps(feed.path_since(cursor), ensure_ascii=False).encode("utf-8")
                content_type = "application/json; charset=utf-8"
            elif path in ("/", "/laser_live.html"):
                body = PAGE.read_bytes()
                content_type = "text/html; charset=utf-8"
            else:
                self.send_error(404)
                return
            self.send_response(200)
            self.send_header("Content-Type", content_type)
            self.send_header("Cache-Control", "no-store")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def do_POST(self) -> None:
            path = urlsplit(self.path).path
            if path not in ("/api/serial", "/api/reset-position"):
                self.send_error(404)
                return
            try:
                if self.headers.get_content_type() != "application/json":
                    raise ValueError("需要 JSON 请求")
                size = int(self.headers.get("Content-Length", "0"))
                if not 0 < size <= 256:
                    raise ValueError("请求长度无效")
                payload = json.loads(self.rfile.read(size))
                if not isinstance(payload, dict):
                    raise ValueError("请求格式无效")
                if path == "/api/reset-position":
                    status, data = 200, feed.reset_position_view()
                else:
                    port = payload["port"]
                    if not isinstance(port, str) or not re.fullmatch(r"COM\d{1,3}", port, re.I):
                        raise ValueError("串口名称无效")
                    feed.select_port(port)
                    status, data = 200, {"port": feed.snapshot()["port"]}
            except (ValueError, KeyError, json.JSONDecodeError) as exc:
                status, data = 400, {"error": str(exc)}
            body = json.dumps(data, ensure_ascii=False).encode("utf-8")
            self.send_response(status)
            self.send_header("Content-Type", "application/json; charset=utf-8")
            self.send_header("Cache-Control", "no-store")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def log_message(self, _format: str, *_args: object) -> None:
            pass

    server = ThreadingHTTPServer(("127.0.0.1", args.http_port), Handler)
    print(f"Laser dashboard: http://127.0.0.1:{args.http_port}/laser_live.html", flush=True)
    print(f"Receive-only serial: {args.port} at {args.baud} 8N1", flush=True)
    if args.open_browser:
        webbrowser.open(f"http://127.0.0.1:{args.http_port}/laser_live.html", new=2)
    try:
        server.serve_forever()
    finally:
        server.server_close()


if __name__ == "__main__":
    main()
