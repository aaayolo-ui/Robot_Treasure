"""Route elapsed time from checksum-verified MCU start/end records."""
class DriveTimer:
    def __init__(self):
        self.started = None
        self.elapsed = 0
        self.running = False
        self.last = None
        self.reason = "等待出发"

    def observe(self, line):
        if not line.startswith("MT3,"):
            return
        try:
            from laser_live_server import LaserFeed
            payload, checksum = line.strip().split("*", 1)
            if LaserFeed._crc16(payload) != int(checksum, 16):
                return
            parts = payload.split(",")
            if len(parts) != 21:
                return
            values = [int(value) for value in parts[1:]]
            run, seq, tick, event = values[:4]
        except (ValueError, TypeError):
            return
        # Ignore duplicate/out-of-order records, except a new route sequence.
        if self.last and run == self.last[0] and seq <= self.last[1] and seq != 1:
            return
        if self.last and tick < self.last[2] and self.last[2] - tick < 0x80000000:
            self.running = False
            self.started = None
            self.reason = "设备时间重置"
        self.last = (run, seq, tick)
        if event == 2 and not self.running:
            self.started = tick
            self.elapsed = 0
            self.running = True
            self.reason = "行驶中"
        if self.running or (event == 13 and self.started is not None and self.reason == "故障停车"):
            self.elapsed = (tick - self.started) & 0xffffffff
            if event == 13:
                self.running = False
                if self.reason != "故障停车":
                    self.reason = "已停车"
        return values

    def fault(self):
        if self.running:
            self.running = False
            self.reason = "故障停车"

    def snapshot(self):
        return dict(elapsed_ms=self.elapsed, running=self.running, reason=self.reason)
