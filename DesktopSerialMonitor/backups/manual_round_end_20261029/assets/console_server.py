"""Single serial owner shared by the desktop terminal and existing web monitor."""
import base64
from collections import deque
import json
from pathlib import Path
import re
import secrets
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlsplit, parse_qs
import serial
from serial.tools import list_ports
from laser_live_server import LaserFeed, PAGE, RAW_LOG_DIR, MAP_READY_LINE
from drive_timer import DriveTimer
from round_timer import RoundTimer


class SharedFeed(LaserFeed):
    def __init__(self, port="COM9", baud=115200):
        super().__init__(port, baud)
        self.stream = secrets.token_hex(12)
        self.raw = bytearray()
        self.rx = self.tx = 0
        self.settings = dict(bytesize=8, parity="N", stopbits=1)
        self.drive_timer = DriveTimer()
        self.round_timer = RoundTimer()
        self.vehicle_role = None
        self.role_priority = 0
        self.trial_origin = None

    def _clear_trial(self):
        """Clear display history only; never send anything to the vehicle."""
        self.track.clear()
        self.track_generation += 1
        self.latest_position = None
        self.leg = None
        self.last_run = None
        self.last_sequence = 0
        self.map_match = None
        self.position_reset_pending = False
        self.localization_uncertain = None
        self.projection.reset()
        self.gray = {"front": None, "rear": None}
        self.gyro = None
        self.localization_diagnostic = None

    @staticmethod
    def _identify_role(line):
        # Project identity wins over copied algorithm/debug names (for example,
        # Thief16U_PoliceTurnCompare is still a thief firmware).
        if re.match(r"^Thief\d", line, re.I):
            return "thief", 3
        if re.match(r"^Police\d", line, re.I):
            return "police", 3
        if re.match(r"^BOOT(?:_STATUS)? .*\bFW=(?:UARTCH4_)?POLICE\d", line):
            return "police", 3
        if line.startswith("THIEF_"):
            return "thief", 2
        if line.startswith("POLICE_"):
            return "police", 2
        return None, 0

    def _parse_map_ready(self, line):
        match = MAP_READY_LINE.match(line)
        if not match:
            return
        start = int(match["start"])
        signature_ok = int(match["signature"]) == self.map_signature
        if signature_ok and (self.drive_timer.running or
                (self.trial_origin is not None and start != self.trial_origin)):
            # A preview for an interception/replan is not a new physical test.
            self.map_match = True
            return
        super()._parse_map_ready(line)
        if self.latest_position and self.latest_position["source"] == "MAP_READY":
            self.trial_origin = start
            self.drive_timer = DriveTimer()
            if self.vehicle_role is None:
                self.vehicle_role = {54: "thief", 81: "police"}.get(start)
                self.role_priority = 1 if self.vehicle_role else 0

    def parse_line(self, line):
        line = line.strip()
        with self.lock:
            if line.startswith("BOOT FW="):
                self._clear_trial()
                self.trial_origin = None
                self.vehicle_role = None
                self.role_priority = 0
                self.drive_timer = DriveTimer()
                self.round_timer.reset()
            role, priority = self._identify_role(line)
            if role and priority >= self.role_priority:
                if self.vehicle_role and role != self.vehicle_role:
                    self._clear_trial()
                    self.trial_origin = None
                    self.drive_timer = DriveTimer()
                    self.round_timer.reset()
                self.vehicle_role = role
                self.role_priority = priority
            was_running = self.drive_timer.running
            previous_tick = self.drive_timer.last[2] if self.drive_timer.last else None
            values = self.drive_timer.observe(line)
            clock_restarted = bool(values and previous_tick is not None and
                values[2] < previous_tick and previous_tick-values[2] < 0x80000000)
            if clock_restarted:
                self.round_timer.reset()
                self._clear_trial()
                self.trial_origin = None
                self.vehicle_role = None
                self.role_priority = 0
                was_running = False
            new_trial = bool(values and values[3] == 2 and not was_running and
                values[9] == self.map_signature and
                frozenset(values[7:9]) in self.edges and
                (self.trial_origin is None or values[7] == self.trial_origin))
            if new_trial:
                self.round_timer.start()
                if not clock_restarted:
                    self._clear_trial()
                self.trial_origin = values[7]
                if self.vehicle_role is None:
                    self.vehicle_role = {54: "thief", 81: "police"}.get(values[7])
                    self.role_priority = 1 if self.vehicle_role else 0
            if line.startswith("ROUTE_FAULT="):
                self.drive_timer.fault()
            super().parse_line(line)
            if new_trial and self.latest_position:
                self.initial_pose = (values[7], self.latest_position["heading"])

    def select_port(self, port):
        previous = self.port
        super().select_port(port)
        if self.port != previous:
            with self.lock:
                self.vehicle_role = None
                self.role_priority = 0
                self.trial_origin = None
                self.drive_timer = DriveTimer()
                self.round_timer = RoundTimer()

    def configure(self, data):
        port = str(data.get("port", ""))
        if not re.fullmatch(r"COM\d{1,3}", port, re.I):
            raise ValueError("串口名称无效")
        baud = int(data.get("baud", 115200))
        bits = int(data.get("bytesize", 8))
        parity = data.get("parity", "N")
        stop = float(data.get("stopbits", 1))
        if baud not in (1200,2400,4800,9600,19200,38400,57600,115200,230400,460800,921600):
            raise ValueError("波特率无效")
        if bits not in (5,6,7,8) or parity not in ("N","E","O") or stop not in (1,1.5,2):
            raise ValueError("串口配置无效")
        self.disconnect()
        with self.lock:
            self.baud = baud
            self.settings = dict(bytesize=bits, parity=parity, stopbits=stop)
        self.connect_port(port)

    def snapshot(self):
        with self.lock:
            state = super().snapshot()
            state.update(console_version=1, rx_bytes=self.rx, tx_bytes=self.tx,
                         console_stream=self.stream, serial_settings=self.settings.copy(),
                         drive_timer=self.drive_timer.snapshot(),
                         round_timer=self.round_timer.snapshot(), round_timer_version=2,
                         vehicle_role=self.vehicle_role, trial_tracking_version=1)
        return state

    def record(self, chunk):
        self.raw.extend(chunk)
        self.rx += len(chunk)
        if len(self.raw) > 524288:
            del self.raw[:-524288]

    def console(self, offset):
        with self.lock:
            start = self.rx - len(self.raw)
            begin = max(start, min(offset, self.rx))
            chunk = self.raw[begin-start:begin-start+65536]
            return dict(stream=self.stream, start=begin, next=begin+len(chunk),
                        rx=self.rx, tx=self.tx, dropped=offset < start,
                        data=base64.b64encode(chunk).decode("ascii"))

    def send(self, encoded):
        try:
            data = base64.b64decode(encoded, validate=True)
        except (ValueError, TypeError) as exc:
            raise ValueError("发送数据格式无效") from exc
        if not 0 < len(data) <= 65536:
            raise ValueError("每次发送需为 1 至 65536 字节")
        with self.lock:
            reader = self.serial_reader
            if not self.serial_enabled or not self.connected or reader is None or not reader.is_open:
                raise ValueError("串口尚未连接")
            # No automatic retry: a partially completed command must never be sent twice.
            count = reader.write(data)
            self.tx += count
            if count != len(data):
                raise ValueError("仅发送了部分数据，请检查串口；未自动重发")
            return dict(written=count, tx=self.tx)

    def run(self):
        while not self.stopping:
            with self.lock:
                self.serial_changed.clear()
                enabled, port, generation = self.serial_enabled, self.port, self.serial_generation
                config = self.settings.copy()
                baud = self.baud
            if not enabled:
                self.serial_changed.wait()
                continue
            reader = serial.Serial(port=None, baudrate=baud, timeout=.10, write_timeout=.5, **config)
            reader.port, reader.dtr, reader.rts = port, False, False
            try:
                with self.lock:
                    if self.serial_generation != generation or not self.serial_enabled:
                        continue
                    reader.open()
                    self.serial_reader = reader
                    self.connected, self.error = True, ""
                text = ""
                with (RAW_LOG_DIR / ("serial_%d.log" % (time.time()*1000))).open("ab", buffering=0) as log:
                    with self.lock:
                        self.raw_log_path = Path(log.name)
                    while True:
                        with self.lock:
                            if self.serial_generation != generation:
                                break
                        chunk = reader.read(4096)
                        if not chunk:
                            continue
                        with self.lock:
                            if self.serial_generation != generation:
                                break
                            log.write(chunk)
                            self.record(chunk)
                            self.last_receive_ms = int(time.time()*1000)
                            text += chunk.decode("ascii", errors="ignore")
                            lines = text.split("\n")
                            text = lines.pop()[-16384:]
                            for line in lines:
                                self.parse_line(line)
            except (OSError,serial.SerialException) as exc:
                with self.lock:
                    if self.serial_generation == generation:
                        self.connected, self.error = False, str(exc)
                self.serial_changed.wait(2)
            finally:
                if reader.is_open:
                    reader.close()
                with self.lock:
                    if self.serial_reader is reader:
                        self.serial_reader = None
                    if self.serial_generation == generation:
                        self.connected = False


def handler_for(feed, token):
    class Handler(BaseHTTPRequestHandler):
        def reply(self, data, status=200):
            body = json.dumps(data, ensure_ascii=False).encode("utf-8")
            self.send_response(status)
            self.send_header("Content-Type", "application/json; charset=utf-8")
            self.send_header("Cache-Control", "no-store")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def private(self):
            return secrets.compare_digest(self.headers.get("X-Desktop-Key", ""), token)

        def do_GET(self):
            request = urlsplit(self.path)
            path = request.path
            if path == "/embedded.css":
                body = Path(__file__).with_name("embedded.css").read_bytes()
                self.send_response(200)
                self.send_header("Content-Type","text/css; charset=utf-8")
                self.send_header("Content-Length",str(len(body)))
                self.end_headers()
                return self.wfile.write(body)
            if path == "/api/console":
                if not self.private():
                    return self.reply(dict(error="仅桌面工具可访问原始串口"),403)
                try:
                    return self.reply(feed.console(max(0,int(parse_qs(request.query).get("after",[0])[0]))))
                except ValueError:
                    return self.reply(dict(error="游标无效"),400)
            if path == "/api/laser":
                return self.reply(feed.snapshot())
            if path == "/api/ports":
                return self.reply(dict(ports=sorted(p.device for p in list_ports.comports()),selected=feed.port))
            if path == "/api/map":
                return self.reply(feed.map_view)
            if path == "/api/path":
                try:
                    return self.reply(feed.path_since(max(0,int(parse_qs(request.query).get("after",[0])[0]))))
                except ValueError:
                    return self.reply(dict(error="游标无效"),400)
            if path in ("/","/laser_live.html"):
                body = PAGE.read_bytes()
                self.send_response(200)
                self.send_header("Content-Type","text/html; charset=utf-8")
                self.send_header("Content-Length",str(len(body)))
                self.end_headers()
                self.wfile.write(body)
            else:
                self.reply(dict(error="Not found"),404)

        def do_POST(self):
            path = urlsplit(self.path).path
            if path not in ("/api/console/connect","/api/console/send","/api/serial","/api/reset-position"):
                return self.reply(dict(error="Not found"),404)
            if path.startswith("/api/console/") and not self.private():
                return self.reply(dict(error="仅桌面工具可配置和发送"),403)
            # Public monitor bridge never forwards these private terminal endpoints.
            origin = self.headers.get("Origin", "")
            if origin and origin != "http://127.0.0.1:8766":
                return self.reply(dict(error="请求来源无效"),403)
            try:
                size = int(self.headers.get("Content-Length",0))
                if self.headers.get_content_type() != "application/json" or not 0 < size <= 100000:
                    raise ValueError("请求格式或长度无效")
                data = json.loads(self.rfile.read(size))
                if not isinstance(data,dict):
                    raise ValueError("请求格式无效")
                if path == "/api/console/connect":
                    feed.configure(data)
                    return self.reply(feed.snapshot())
                if path == "/api/console/send":
                    return self.reply(feed.send(data.get("data")))
                if path == "/api/reset-position":
                    return self.reply(feed.reset_position_view())
                action = data.get("action")
                if action == "disconnect":
                    feed.disconnect()
                elif action in ("connect","select"):
                    port = data.get("port","")
                    if not re.fullmatch(r"COM\d{1,3}",port,re.I):
                        raise ValueError("串口名称无效")
                    (feed.connect_port if action=="connect" else feed.select_port)(port)
                else:
                    raise ValueError("串口操作无效")
                self.reply(feed.snapshot())
            except (ValueError,TypeError,KeyError,OSError,serial.SerialException) as exc:
                self.reply(dict(error=str(exc)),400)

        def log_message(self,*args):
            pass
    return Handler


def main():
    token = secrets.token_urlsafe(32)
    key = Path(__file__).with_name("console.key")
    feed = SharedFeed()
    server = ThreadingHTTPServer(("127.0.0.1",8766),handler_for(feed,token))
    key.write_text(token,encoding="ascii")
    threading.Thread(target=feed.run,daemon=True).start()
    try:
        server.serve_forever()
    finally:
        feed.stop()
        server.server_close()

if __name__ == "__main__":
    main()
