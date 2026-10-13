"""Read MT1 source timestamps. No hardware access and no inferred global pose."""

import argparse
import binascii
import json
import math
from collections import defaultdict
from pathlib import Path

ROOT = Path(__file__).resolve().parent
MAP = ROOT.parent / "TreasurePursuitSimulator" / "map_annotations.json"
EVENTS = {1: "BOOT", 2: "START", 3: "JUNCTION", 4: "APPROACH", 5: "BRAKE",
          6: "ROTATE", 7: "SPIN_END", 8: "FOLLOW", 9: "LINE_STABLE",
          10: "GYRO_WAIT", 11: "GYRO_RECOVER", 12: "FRONT_WAIT", 13: "END", 14: "SAMPLE"}


def crc16(payload):
    return binascii.crc_hqx(payload, 0xFFFF)


def decode_line(line):
    marker = line.find(b"MT1,")
    if marker < 0:
        return None
    payload, checksum = line[marker:].strip().rsplit(b"*", 1)
    if len(checksum) != 4 or crc16(payload) != int(checksum, 16):
        raise ValueError("CRC mismatch or incomplete measurement line")
    values = payload.decode("ascii").split(",")
    if len(values) != 16:
        raise ValueError("wrong field count")
    nums = [int(value) for value in values[1:]]
    run, sequence, tick, event, junction, arg, yaw, valid, age, frames, *remaining = nums
    if event not in EVENTS or valid not in (0, 1):
        raise ValueError("unknown event or invalid yaw flag")
    if not (0 <= tick < 2 ** 32 and run >= 0 and sequence > 0 and age >= 0):
        raise ValueError("invalid source time or sequence")
    return dict(run=run, sequence=sequence, source_ms=tick, event=EVENTS[event],
                junction=junction, argument=arg, yaw_deg=yaw / 100,
                yaw_valid=bool(valid), yaw_age_ms=age, yaw_frames=frames,
                encoder_counts=remaining[:4], lost=remaining[4])


def yaw_delta(start, end):
    return (end["yaw_deg"] - start["yaw_deg"] + 180) % 360 - 180


def fresh_yaw(record):
    return record["yaw_valid"] and record["yaw_age_ms"] <= 150


def _turns(records):
    groups = defaultdict(list)
    for record in records:
        if record["junction"] > 0:
            groups[record["junction"]].append(record)
    output = []
    phases = ("JUNCTION", "APPROACH", "BRAKE", "ROTATE", "SPIN_END", "FOLLOW", "LINE_STABLE")
    for junction, samples in sorted(groups.items()):
        by_event = defaultdict(list)
        for sample in samples:
            by_event[sample["event"]].append(sample)
        issues = []
        if any(len(by_event[event]) != 1 for event in phases):
            issues.append("incomplete_or_repeated_phase")
        if any(by_event[event] for event in ("FRONT_WAIT", "GYRO_WAIT", "GYRO_RECOVER")):
            issues.append("sensor_recovery")
        item = {"junction_index": junction, "valid": False, "issues": issues}
        if "incomplete_or_repeated_phase" not in issues:
            events = [by_event[event][0] for event in phases]
            if any(a["sequence"] >= b["sequence"] or a["elapsed_ms"] > b["elapsed_ms"]
                   for a, b in zip(events, events[1:])):
                issues.append("phase_order")
            first, approach, brake, rotate, spin, follow, stable = events
            if spin["argument"] != 0:
                issues.append("gray_backup_turn_finish")
            spin_time = (spin["elapsed_ms"] - rotate["elapsed_ms"]) / 1000
            angle = yaw_delta(rotate, spin) if fresh_yaw(rotate) and fresh_yaw(spin) else None
            total_angle = yaw_delta(rotate, stable) if fresh_yaw(rotate) and fresh_yaw(stable) else None
            if spin["yaw_frames"] <= rotate["yaw_frames"]:
                angle = None
            item.update(direction="left" if rotate["argument"] == -1 else "right",
                        detection_sequence=first["sequence"], stable_sequence=stable["sequence"],
                        approach_s=(brake["elapsed_ms"] - approach["elapsed_ms"]) / 1000,
                        brake_and_gyro_wait_s=(rotate["elapsed_ms"] - brake["elapsed_ms"]) / 1000,
                        rotation_s=spin_time,
                        exit_to_first_follow_s=(follow["elapsed_ms"] - spin["elapsed_ms"]) / 1000,
                        full_flow_s=(stable["elapsed_ms"] - first["elapsed_ms"]) / 1000,
                        rotation_angle_deg=angle, full_angle_deg=total_angle,
                        mean_rotation_deg_s=abs(angle) / spin_time if angle is not None and spin_time > 0 else None,
                        quarter_turn_angle_check=total_angle is not None and abs(abs(total_angle) - 90) <= 10,
                        note="LINE_STABLE = three valid front-gray control samples; not physical pose proof.")
            item["valid"] = not issues
        output.append(item)
    return output


def analyze(raw, annotations, assignments=None):
    bad = []
    decoded = []
    for number, line in enumerate(raw.splitlines(), 1):
        try:
            record = decode_line(line)
        except (ValueError, UnicodeError, IndexError) as error:
            bad.append({"line": number, "error": str(error)})
            continue
        if record:
            decoded.append(record)
    sessions = []
    occurrences = defaultdict(int)
    current = None
    for record in decoded:
        if record["run"] == 0:
            continue
        if record["event"] == "START" or current is None or record["run"] != current["run"]:
            occurrences[record["run"]] += 1
            current = {"id": f'r{record["run"]}.{occurrences[record["run"]]}',
                       "run": record["run"], "records": [], "issues": []}
            sessions.append(current)
        records = current["records"]
        if records:
            previous = records[-1]
            if record["sequence"] != previous["sequence"] + 1:
                current["issues"].append("sequence_gap_or_duplicate")
            delta = (record["source_ms"] - previous["source_ms"]) & 0xFFFFFFFF
            if delta > 3_600_000:
                current["issues"].append("source_clock_reset_or_unreasonable_interval")
            record["elapsed_ms"] = previous["elapsed_ms"] + delta
        else:
            record["elapsed_ms"] = 0
            if record["event"] != "START" or record["sequence"] != 1:
                current["issues"].append("start_missing")
        if record["lost"] != 0:
            current["issues"].append("firmware_queue_loss")
        records.append(record)
    for session in sessions:
        records = session["records"]
        if records[-1]["event"] != "END":
            session["issues"].append("end_missing")
        elif records[-1]["argument"] != 1:
            session["issues"].append("abnormal_stop")
        if bad:
            session["issues"].append("capture_contains_corrupt_measurement")
        session["issues"] = sorted(set(session["issues"]))
        session["valid"] = not session["issues"]
        session["turns"] = _turns(records)
        for turn in session["turns"]:
            turn["valid"] = turn["valid"] and session["valid"]
    segment_by_id = {s["id"]: s for s in annotations["segments"]}
    intervals = []
    for assignment in (assignments or {}).get("intervals", []):
        item = dict(assignment)
        issues = []
        session = next((s for s in sessions if s["id"] == assignment.get("session")), None)
        records = {r["sequence"]: r for r in session["records"]} if session else {}
        start = records.get(assignment.get("start_sequence"))
        end = records.get(assignment.get("end_sequence"))
        if not session or not session["valid"]:
            issues.append("invalid_or_missing_session")
        if start is None or end is None or start["sequence"] >= end["sequence"]:
            issues.append("invalid_interval_endpoints")
        chain = assignment.get("segments", [])
        if not chain:
            issues.append("segment_assignment_missing")
        previous_to = None
        nominal_length = 0.0
        for entry in chain:
            segment = segment_by_id.get(entry.get("id"))
            if not segment or {entry.get("from"), entry.get("to")} != {segment["start"], segment["end"]}:
                issues.append("segment_endpoints_mismatch")
                continue
            if previous_to is not None and previous_to != entry["from"]:
                issues.append("disconnected_segment_chain")
            previous_to = entry["to"]
            nominal_length += segment["length_m"]
        duration = ((end["elapsed_ms"] - start["elapsed_ms"]) / 1000) if start and end else None
        if duration is not None and duration <= 0:
            issues.append("nonpositive_duration")
        distance = assignment.get("measured_interval_m")
        if not isinstance(distance, (int, float)) or isinstance(distance, bool) or not math.isfinite(distance) or distance <= 0:
            distance = None
        measured = (distance is not None and assignment.get("distance_calibrated") is True
                    and assignment.get("route_verified") is True and bool(assignment.get("boundary_note")))
        if not measured:
            issues.append("distance_or_route_not_verified")
        if start and end and session:
            middle = [r for r in session["records"] if start["sequence"] <= r["sequence"] <= end["sequence"]]
            if any(r["event"] in ("FRONT_WAIT", "GYRO_WAIT", "GYRO_RECOVER") for r in middle):
                issues.append("sensor_recovery_in_interval")
        item.update(valid=not issues, issues=sorted(set(issues)),
                    elapsed_s=duration, nominal_map_length_m=nominal_length,
                    measured_speed_m_s=distance / duration if not issues else None,
                    note="Measured distance must match these timing boundaries; map length is a separate nominal reference.")
        intervals.append(item)
    return {"schema_version": 1, "raw_measurement_records": len(decoded), "corrupt_lines": bad,
            "sessions": sessions, "intervals": intervals,
            "map_calibrated": annotations["calibrated"],
            "coverage_note": "No global localization is inferred. Assign and verify routes before claiming road coverage.",
            "measurement_note": "Source tick resolution is 1 ms; physical accuracy depends on detection, control and calibration."}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("log", type=Path)
    parser.add_argument("--map", type=Path, default=MAP)
    parser.add_argument("--assignments", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    assignments = json.loads(args.assignments.read_text(encoding="utf-8-sig")) if args.assignments else None
    report = analyze(args.log.read_bytes(), json.loads(args.map.read_text(encoding="utf-8-sig")), assignments)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    print(f'{len(report["sessions"])} sessions; {len(report["corrupt_lines"])} bad lines; output: {args.output}')


if __name__ == "__main__":
    main()
