"""Whole round begins with the first accepted KEY2 route, not MCU power-on."""
import time


class RoundTimer:
    LIMIT_MS = 180000

    def __init__(self, clock=time.monotonic):
        self.clock = clock
        self.anchor = None
        self.source = None

    def reset(self):
        # RESET arms the next round; waiting for KEY2 does not consume time.
        self.anchor = None
        self.source = None

    def start(self):
        if self.anchor is None:
            self.anchor = self.clock()
            self.source = "first_accepted_start"

    def snapshot(self):
        elapsed = None if self.anchor is None else max(0, round((self.clock()-self.anchor)*1000))
        return dict(elapsed_ms=elapsed, running=self.anchor is not None,
                    reached_180=elapsed is not None and elapsed >= self.LIMIT_MS,
                    remaining_ms=None if elapsed is None else max(0,self.LIMIT_MS-elapsed),
                    source=self.source, mode="key2_round")
