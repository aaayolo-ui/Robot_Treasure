"""Short, road-constrained display prediction from MCU timing records.

All movement estimates use the MCU clock. Host time is used only to determine
how old an observation is. The minimum observed host/source offset removes
relative serial queuing delay; the absolute delay of the first packet is unknown.
This class never adds predicted points to a recorded vehicle trajectory.
"""

from __future__ import annotations

import math
import time


class RoadProjection:
    HORIZON_MS = 700.0
    SOURCE_GAP_MS = 1000.0
    PAUSE_EVENTS = frozenset((4, 5, 6, 7, 10, 11, 12, 17, 18, 19))

    def __init__(self, nodes: dict, edges: dict, forward_mm_s: float,
                 reverse_mm_s: float) -> None:
        self.nodes = nodes
        self.edges = edges
        self._configured = {0: float(forward_mm_s), 1: float(reverse_mm_s)}
        if any(not math.isfinite(speed) or speed <= 0
               for speed in self._configured.values()):
            raise ValueError("Configured road speeds must be positive finite mm/s")
        self.reset()

    @staticmethod
    def _now_ms() -> float:
        return time.monotonic() * 1000.0

    def reset(self) -> None:
        """Forget a reboot's clock, active road and learned movement speeds."""
        self._speeds = self._configured.copy()
        self._speed_sources = {0: "configured_forward", 1: "configured_reverse"}
        self._leg = None
        self._run = None
        self._sequence = None
        self._lost = None
        self._source_raw = None
        self._source_ms = None
        self._offset_ms = None
        self._needs_samples = False
        self._sample_streak = 0
        self._previous_event = None
        self._await_leg = True
        self._state = "paused"
        self._stop_reason = "reset"
        self._sample_anchor = None
        self._movement_start = None
        self._movement_ms = 0.0
        self._junction_learning = False

    def stop(self, reason: str = "paused") -> None:
        """Stop predicting until a new LEG_BEGIN; never invent a new road."""
        self._state = "paused"
        self._stop_reason = reason
        self._await_leg = True
        self._sample_anchor = None
        self._movement_start = None
        self._junction_learning = False

    def source_age_ms(self, now_ms: float | None = None) -> float | None:
        """Age of the latest accepted source event, including relative queuing.

        This follows all timing events, including stationary/rotation records;
        it deliberately does not use the older road-position anchor.
        """
        if self._source_ms is None or self._offset_ms is None:
            return None
        now = self._now_ms() if now_ms is None else float(now_ms)
        return round(max(0.0, now - (self._source_ms + self._offset_ms)), 1)

    def _time(self, raw: int, received_ms: float) -> tuple[float, bool] | None:
        raw &= 0xFFFFFFFF
        if self._source_raw is None:
            source = float(raw)
            gap = False
        else:
            delta = (raw - self._source_raw) & 0xFFFFFFFF
            # A backwards source timestamp is neither a valid elapsed duration
            # nor a 32-bit rollover. The caller resets us on a firmware reboot.
            if delta >= 0x80000000:
                return None
            source = self._source_ms + delta
            gap = delta > self.SOURCE_GAP_MS
        self._source_raw = raw
        self._source_ms = source
        offset = received_ms - source
        if self._offset_ms is None or offset < self._offset_ms:
            self._offset_ms = offset
        return source, gap

    def _learn(self, speed: float, mode: int, source: str) -> None:
        # Reject stationary jitter and implausible spikes caused by encoder
        # resets. These are display estimates rather than a control command.
        if not math.isfinite(speed) or not 5.0 <= speed <= 3000.0:
            return
        old_source = self._speed_sources[mode]
        weight = 1.0 if old_source.startswith("configured") else (
            0.35 if source == "samples" else 0.2)
        self._speeds[mode] += weight * (speed - self._speeds[mode])
        self._speed_sources[mode] = source

    def _pause_movement(self, source_ms: float) -> None:
        if self._movement_start is not None:
            self._movement_ms += max(0.0, source_ms - self._movement_start)
        self._movement_start = None
        self._sample_anchor = None

    def _anchor(self, progress: float, heading: float, source: float,
                received: float) -> None:
        self._leg["progress"] = max(0.0, min(self._leg["length"], progress))
        self._leg["heading"] = heading % 360.0
        self._leg["source_ms"] = source
        self._leg["received_ms"] = received

    def on_timing(self, run: int, sequence: int, mcu_ms: int, event: int,
                  from_node: int, to_node: int, progress_mm: float,
                  heading: float, drive_mode: int, lost: int,
                  received_ms: float | None = None) -> None:
        received = self._now_ms() if received_ms is None else float(received_ms)
        if not all(math.isfinite(value) for value in
                   (received, float(progress_mm), float(heading))):
            return
        # A run is a route execution, not necessarily a reboot. Do not use it
        # to restart the host/source clock or to erase a displayed trajectory.
        same_run = run == self._run
        if same_run and self._sequence is not None and sequence <= self._sequence:
            return
        stamp = self._time(int(mcu_ms), received)
        if stamp is None:
            self._state = "awaiting_samples"
            self._needs_samples = True
            self._sample_streak = 0
            self._sample_anchor = None
            self._junction_learning = False
            return
        source, long_gap = stamp
        missing = bool(same_run and self._sequence is not None and
                       (sequence != self._sequence + 1 or lost > self._lost))
        self._run, self._sequence, self._lost = run, sequence, lost
        if missing or (long_gap and event not in (2, 16)):
            self._needs_samples = True
            self._sample_streak = 0
            self._sample_anchor = None
            self._junction_learning = False
            if self._state not in ("node_confirmed", "paused"):
                self._state = "awaiting_samples"
        if event != 14:
            self._sample_streak = 0
        previous_event = self._previous_event
        self._previous_event = event

        if event == 2:
            self.stop("awaiting_leg")
            return
        mode = 1 if drive_mode else 0
        edge = self.edges.get(frozenset((from_node, to_node)))
        valid_road = (from_node in self.nodes and to_node in self.nodes and
                      edge is not None and math.isfinite(float(edge)) and edge > 0)
        if event == 16:
            if not valid_road:
                self.stop("unknown_road")
                return
            self._leg = {"from": from_node, "to": to_node, "length": float(edge),
                         "drive_mode": mode, "heading": heading % 360.0,
                         "progress": 0.0, "source_ms": source,
                         "received_ms": received}
            self._anchor(progress_mm, heading, source, received)
            self._await_leg = False
            self._state = "awaiting_samples" if self._needs_samples else "predicting"
            self._sample_anchor = None
            self._movement_start = source
            self._movement_ms = 0.0
            self._junction_learning = not (missing or self._needs_samples)
            return
        if self._leg is None:
            return
        same_leg = (from_node == self._leg["from"] and to_node == self._leg["to"])
        if event == 13:
            # END also covers an emergency stop between nodes. Its encoder
            # snapshot is usable only on the still-active translation road.
            if same_leg and not self._await_leg and self._state == "predicting":
                self._anchor(progress_mm, heading, source, received)
            self.stop("route_end")
            return
        if self._await_leg or not same_leg:
            return
        if event == 15:
            self._pause_movement(source)
            if self._junction_learning and self._movement_ms >= 200.0:
                self._learn(self._leg["length"] * 1000.0 / self._movement_ms,
                            self._leg["drive_mode"], "junction_timing")
            self._anchor(self._leg["length"], heading, source, received)
            self._state = "node_confirmed"
            self._await_leg = True
            return
        if event in self.PAUSE_EVENTS:
            # BRAKE/APPROACH/FRONT_WAIT still have a meaningful translating
            # encoder snapshot. Rotation records can have the old road tags.
            if event in (4, 5, 12) and self._state in ("predicting", "awaiting_samples"):
                self._anchor(progress_mm, heading, source, received)
            self._pause_movement(source)
            self._state = "paused"
            return
        if event != 14:
            return
        # Mode changes on an old road are the reverse-alignment phase, not a
        # confirmation that the next road has begun.
        if mode != self._leg["drive_mode"]:
            self._pause_movement(source)
            self._state = "paused"
            return
        progress = max(0.0, min(self._leg["length"], float(progress_mm)))
        if self._needs_samples:
            self._sample_streak = (self._sample_streak + 1 if
                                   previous_event == 14 and not missing and not long_gap else 1)
        if self._sample_anchor is not None and not missing and not long_gap:
            previous_source, previous_progress = self._sample_anchor
            duration = source - previous_source
            if 0 < duration <= self.SOURCE_GAP_MS:
                self._learn((progress - previous_progress) * 1000.0 / duration,
                            mode, "samples")
        self._sample_anchor = (source, progress)
        self._anchor(progress, heading, source, received)
        if self._movement_start is None:
            self._movement_start = source
        if self._needs_samples and self._sample_streak < 2:
            self._state = "awaiting_samples"
        else:
            self._needs_samples = False
            self._state = "predicting"

    def view(self, now_ms: float | None = None, connected: bool = True) -> dict | None:
        if self._leg is None:
            return None
        now = self._now_ms() if now_ms is None else float(now_ms)
        leg = self._leg
        anchor = leg["progress"]
        source_host_time = leg["source_ms"] + self._offset_ms
        age = max(0.0, now - source_host_time)
        speed = self._speeds[leg["drive_mode"]]
        progress = anchor
        state = self._state
        if state == "predicting":
            # A prediction may approach the current endpoint, but may never
            # choose or enter its outgoing road before a gray confirmation.
            cap = leg["length"] - min(35.0, leg["length"] * 0.08)
            progress = max(anchor, min(cap, anchor + speed * min(age, self.HORIZON_MS) / 1000.0))
            if not connected or age > self.HORIZON_MS:
                state = "stale"
            elif progress >= cap:
                state = "waiting_node"
        elif not connected and state not in ("node_confirmed", "paused"):
            state = "stale"
        start, end = self.nodes[leg["from"]], self.nodes[leg["to"]]

        def point(distance: float) -> tuple[float, float]:
            ratio = distance / leg["length"]
            return (start[0] + (end[0] - start[0]) * ratio,
                    start[1] + (end[1] - start[1]) * ratio)

        x, y = point(progress)
        anchor_x, anchor_y = point(anchor)
        predicted = progress > anchor + 0.01
        uncertainty = 15.0 if self._state == "node_confirmed" else (
            35.0 + (progress - anchor) * 0.2 +
            speed * min(age, self.HORIZON_MS) / 1000.0 * 0.15)
        if self._needs_samples:
            uncertainty += 50.0
        return {"x_mm": round(x, 1), "y_mm": round(y, 1),
                "heading": round(leg["heading"], 1),
                "anchor_x_mm": round(anchor_x, 1), "anchor_y_mm": round(anchor_y, 1),
                "mode": state, "predicted": predicted,
                "speed_mm_s": round(speed, 1),
                "speed_source": self._speed_sources[leg["drive_mode"]],
                "telemetry_age_ms": round(age, 1),
                "from_node": leg["from"], "to_node": leg["to"],
                "uncertainty_mm": round(uncertainty, 1)}
