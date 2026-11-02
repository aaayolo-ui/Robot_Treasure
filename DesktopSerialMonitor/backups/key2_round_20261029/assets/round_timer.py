"""RESET-to-RESET wall-clock timer; vehicle stops never pause this clock."""
import re
import time


class RoundTimer:
    LIMIT_MS = 180000

    def __init__(self, clock=time.monotonic):
        self.clock = clock
        self.anchor = None
        self.synced = False
        self.source = None
        self.last_mcu_tick = None

    def reset(self):
        self.anchor = self.clock()
        self.synced = False
        self.source = "reset_signal"
        self.last_mcu_tick = None

    def observe_tick(self, tick, restarted=False, sequence=None):
        if not 0 <= tick <= 0xffffffff:
            return
        if sequence is not None:
            if (sequence == 1 and self.last_mcu_tick is not None and
                    tick < self.last_mcu_tick and self.last_mcu_tick-tick < 0x80000000):
                restarted = True
            self.last_mcu_tick = tick
        if restarted:
            self.anchor = None
            self.synced = False
        if not self.synced:
            anchor = self.clock() - tick / 1000
            self.anchor = anchor if self.anchor is None else min(self.anchor, anchor)
            self.synced = True
            self.source = "mcu_uptime"

    def observe_status(self, line):
        match = re.match(r"^BOOT_STATUS\b.*\bT=(\d+)\b", line)
        if match:
            self.observe_tick(int(match[1]))

    def snapshot(self):
        elapsed = None if self.anchor is None else max(0, round((self.clock()-self.anchor)*1000))
        return dict(elapsed_ms=elapsed, running=self.anchor is not None,
                    reached_180=elapsed is not None and elapsed >= self.LIMIT_MS,
                    remaining_ms=None if elapsed is None else max(0,self.LIMIT_MS-elapsed),
                    source=self.source, mode="reset_round")
