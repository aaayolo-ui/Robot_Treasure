"""Compare 16L ARM C with validated frozen 16K runs under identical physics.

No hardware is accessed. Historical 16K reports are read-only. Default coverage
is seeds 10/11/12 at police speed multipliers 1.0/1.1/1.2, with both sensors at
600 mm. Non-default seed/factor combinations rerun only missing baseline rows.
"""
import argparse
import copy
import hashlib
import json
import math
import statistics
from pathlib import Path


DEFAULT_SEEDS = (10, 11, 12)
DEFAULT_FACTORS = (1.0, 1.1, 1.2)
RANGE_DEFINE = "THIEF_LASER_MAX_MM=600U"


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def csv_values(value, convert):
    result = tuple(convert(part.strip()) for part in value.split(",") if part.strip())
    if not result or len(result) != len(set(result)):
        raise argparse.ArgumentTypeError("Provide a non-empty list without duplicates")
    return result


def contact_kind(stages, contact):
    """Classify contact by motion stage; do not infer an unrecorded rear impact."""
    if not contact:
        return "no_contact"
    if any(stage in ("pivot", "align") for stage in stages):
        return "turn_contact"
    if "brake" in stages:
        return "before_turn_brake_contact"
    if "switch" in stages:
        return "direction_switch_contact"
    if any(stage in ("forward", "reverse_move") for stage in stages):
        return "translation_contact"
    return "hold_contact"


def frozen_baseline(here, repo, cfg, model):
    comparison_path = here.with_name("16k") / "heldout_comparison.json"
    report_path = here.with_name("16k") / "report.json"
    prior = json.loads(comparison_path.read_text(encoding="utf-8"))
    old_report = json.loads(report_path.read_text(encoding="utf-8"))
    if old_report["config"] != cfg or prior["body_model"] != model:
        raise ValueError("Frozen 16K config/body model differs from the current comparison")
    hashes = prior["source_hashes"]["16K"]
    if hashes != old_report["source_hashes"]:
        raise ValueError("16K heldout and replay source manifests disagree")
    if not hashes:
        raise ValueError("Frozen 16K source manifest is empty")
    for name, expected in hashes.items():
        if digest(repo / name) != expected:
            raise ValueError("Frozen 16K C/header source changed: " + name)
    rows = [copy.deepcopy(row) for row in prior["runs"] if row["variant"] == "16K"]
    expected_keys = {(factor, seed) for factor in DEFAULT_FACTORS for seed in DEFAULT_SEEDS}
    keys = [(row["factor"], row["seed"]) for row in rows]
    if len(rows) != 9 or len(set(keys)) != 9 or set(keys) != expected_keys:
        raise ValueError("Frozen 16K heldout coverage is not the expected nine unique runs")
    for row in rows:
        goals, count = row["goals"], row["arrived_hides"]
        if not 0 <= count <= len(goals):
            raise ValueError("Invalid stored hide arrival count in a baseline row")
        # 16K records actual arrival events; preserve them rather than infer
        # arrival from requested goals or discard existing evidence.
        if row.get("arrived_hide_nodes") is not None:
            if len(row["arrived_hide_nodes"]) != count:
                raise ValueError("Stored actual hide arrival sequence/count mismatch")
            row["arrived_n062"] = row["arrived_hide_nodes"].count("N062")
        else:
            row["arrived_hide_nodes"] = None
            row["arrived_hide_events"] = None
            row["arrival_source"] = "unavailable_in_frozen_baseline"
            row["arrived_n062"] = None
        row["planned_n062"] = goals.count("N062")
        row["collision_kind"] = contact_kind(row["end_stages"], row["winner"] == "police")
        row["collision_kind_source"] = "stored_terminal_stages_and_same_body_referee"
        row["collision_details_available"] = row.get("collision") is not None
        row["measurement_source"] = "validated_frozen_16K_heldout"
    return rows, hashes, {
        "comparison": str(comparison_path.relative_to(repo)).replace("\\", "/"),
        "comparison_sha256": digest(comparison_path),
        "report": str(report_path.relative_to(repo)).replace("\\", "/"),
        "report_sha256": digest(report_path),
        "validated_source_files": len(hashes),
        "config_equal": True,
        "body_model_equal": True,
        "arrival_limit": "Preserve recorded 16K actual arrival nodes/events. Requested goals are counted separately from completed arrivals.",
        "collision_limit": "Contact kind is classified by terminal motion stage; recorded collision data is preserved where available.",
    }


def run_row(name, factor, seed, track, cfg, elf, model, game_class, node_ids):
    config = copy.deepcopy(cfg)
    config["police_speed_m_s"] *= factor
    game = game_class(track, config, elf, model, seed=seed)
    result = game.run()
    stages = result["frames"][-1][35:37]
    arrivals = []
    for event in game.events:
        if event.get("event") != "firmware_hide":
            continue
        node = event["node"]
        if node not in node_ids:
            raise ValueError("Unknown actual hide node: " + repr(node))
        arrivals.append({"t": event["t"], "node": node_ids[node], "original_node": node})
    goals = [event.get("goal") for event in game.planner_events]
    collision = result.get("collision")
    return {
        "variant": name, "factor": factor, "seed": seed,
        "duration_s": result["duration_s"], "winner": result["winner"],
        "end_reason": result["end_reason"], "end_stages": stages,
        "plan_failures": sum(event["kind"] == "fault" for event in game.planner_events),
        "goals": goals,
        "max_model_steps": max((event.get("plan", {}).get("model_steps", 0)
                                for event in game.planner_events), default=0),
        "max_candidates": max((event.get("plan", {}).get("candidates", 0)
                               for event in game.planner_events), default=0),
        "arrived_hides": len(arrivals),
        "arrived_hide_nodes": [event["node"] for event in arrivals],
        "arrived_hide_events": arrivals,
        "arrival_source": "actual_firmware_hide_events",
        "arrived_n062": sum(event["node"] == "N062" for event in arrivals),
        "planned_n062": goals.count("N062"),
        "planned_n026": goals.count("N026"),
        "terminal_finalist_calls": sum(event.get("plan", {}).get("terminal_candidates", 0)
                                      for event in game.planner_events),
        "qualified_terminal_plans": sum(event.get("plan", {}).get("terminal_clear_models", 0) == 6
                                         for event in game.planner_events),
        "emergency_plans": sum(bool(event.get("plan", {}).get("emergency_hide", False))
                               for event in game.planner_events),
        "collision_kind": contact_kind(stages, bool(collision)),
        "collision_kind_source": "actual_body_contact_and_terminal_motion_stages",
        "collision": collision,
        "collision_details_available": True,
        "measurement_source": "new_ARM_C_simulation",
    }


def summarize(runs, factors):
    rows = []
    for name in ("16K", "16L"):
        for factor in factors:
            selected = [row for row in runs if row["variant"] == name and row["factor"] == factor]
            if not selected:
                continue
            planned_goals = sum(goal is not None for row in selected for goal in row["goals"])
            planned_n062 = sum(row["planned_n062"] for row in selected)
            rows.append({
                "variant": name, "factor": factor, "runs": len(selected),
                "mean_s": statistics.mean(row["duration_s"] for row in selected),
                "min_s": min(row["duration_s"] for row in selected),
                "max_s": max(row["duration_s"] for row in selected),
                "thief_wins": sum(row["winner"] == "thief" for row in selected),
                "police_wins": sum(row["winner"] == "police" for row in selected),
                "plan_failures": sum(row["plan_failures"] for row in selected),
                "planned_goals": planned_goals,
                "planned_n062": planned_n062,
                "planned_n062_fraction": planned_n062 / planned_goals if planned_goals else None,
                "arrived_n062": (sum(row["arrived_n062"] for row in selected)
                                 if all(row["arrived_n062"] is not None for row in selected) else None),
                "collision_kinds": {kind: sum(row["collision_kind"] == kind for row in selected)
                                    for kind in sorted({row["collision_kind"] for row in selected})},
            })
    paired, deltas = [], []
    for new in runs:
        if new["variant"] != "16L":
            continue
        old = next(row for row in runs if row["variant"] == "16K" and
                   row["factor"] == new["factor"] and row["seed"] == new["seed"])
        delta = new["duration_s"] - old["duration_s"]
        paired.append({"factor": new["factor"], "seed": new["seed"],
                       "baseline_duration_s": old["duration_s"], "selected_duration_s": new["duration_s"],
                       "duration_delta_s": delta,
                       "baseline_winner": old["winner"], "selected_winner": new["winner"],
                       "baseline_planned_n062": old["planned_n062"],
                       "selected_planned_n062": new["planned_n062"],
                       "baseline_arrived_n062": old["arrived_n062"],
                       "selected_arrived_n062": new["arrived_n062"]})
    for factor in factors:
        pairs = [row for row in paired if row["factor"] == factor]
        if not pairs:
            continue
        old = next(row for row in rows if row["variant"] == "16K" and row["factor"] == factor)
        new = next(row for row in rows if row["variant"] == "16L" and row["factor"] == factor)
        deltas.append({
            "factor": factor, "paired_runs": len(pairs),
            "baseline_mean_s": old["mean_s"], "selected_mean_s": new["mean_s"],
            "mean_duration_delta_s": new["mean_s"] - old["mean_s"],
            "baseline_min_s": old["min_s"], "selected_min_s": new["min_s"],
            "min_duration_delta_s": new["min_s"] - old["min_s"],
            "baseline_thief_wins": old["thief_wins"], "selected_thief_wins": new["thief_wins"],
            "thief_win_delta": new["thief_wins"] - old["thief_wins"],
            "baseline_plan_failures": old["plan_failures"], "selected_plan_failures": new["plan_failures"],
            "baseline_planned_n062": old["planned_n062"], "selected_planned_n062": new["planned_n062"],
            "planned_n062_delta": new["planned_n062"] - old["planned_n062"],
            "baseline_planned_n062_fraction": old["planned_n062_fraction"],
            "selected_planned_n062_fraction": new["planned_n062_fraction"],
            "baseline_arrived_n062": old["arrived_n062"], "selected_arrived_n062": new["arrived_n062"],
            "longer_survival_runs": sum(row["duration_delta_s"] > 1e-8 for row in pairs),
            "shorter_survival_runs": sum(row["duration_delta_s"] < -1e-8 for row in pairs),
            "unchanged_duration_runs": sum(abs(row["duration_delta_s"]) <= 1e-8 for row in pairs),
        })
    return rows, paired, deltas


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--seeds", type=lambda value: csv_values(value, int), default=DEFAULT_SEEDS)
    parser.add_argument("--factors", type=lambda value: csv_values(value, float), default=DEFAULT_FACTORS)
    parser.add_argument("--rerun-baseline", action="store_true",
                        help="Rerun baseline rather than reuse its verified saved measurements")
    args = parser.parse_args()
    if any(not math.isfinite(factor) or factor <= 0 for factor in args.factors):
        parser.error("Police speed factors must be finite positive values")

    from simulate_16l import physics, MultiExitGame, build, FIRMWARE, HERE, base
    repo = HERE.parents[3]
    cfg, model = physics.load_config()
    # Refuse an unacknowledged range change before accepting frozen results.
    if cfg["sensor_range_m"] != 0.6 or cfg["police_sensor_range_m"] != 0.6:
        raise ValueError("Heldout comparison requires unchanged 600 mm sensor ranges for both cars")
    frozen, old_hashes, provenance = frozen_baseline(HERE, repo, cfg, model)
    requested = {(factor, seed) for factor in args.factors for seed in args.seeds}
    runs = [] if args.rerun_baseline else [row for row in frozen if (row["factor"], row["seed"]) in requested]
    hashes = {"16K": old_hashes}
    track = base.TrackMap()
    node_ids = {node["original_node"]: node["id"] for node in physics.ANNOTATIONS["nodes"]}
    metadata = {
        "seeds": list(args.seeds), "factors": list(args.factors), "config": cfg,
        "body_model": model, "comparison_laser_range_mm": 600,
        "selected_build_defines": [RANGE_DEFINE], "baseline_validation": provenance,
        "scope": "Same finite sensors, police, measured motion and continuous body referee. ARM C planner simulation with ideal localization; no hardware evidence.",
        "duration_note": "Longer duration is survival time under this referee, not a win unless winner is thief. Differences are limited to these seeds/factors.",
    }
    missing = requested - {(row["factor"], row["seed"]) for row in runs}
    if missing:
        baseline_firmware = repo / "STM32/Keil_Project/Thief16K_RefugeAccess_FourLaser_Test"
        baseline_elf, hashes["16K"] = build((RANGE_DEFINE,), "16K_heldout", baseline_firmware)
        for factor in args.factors:
            for seed in args.seeds:
                if (factor, seed) not in missing:
                    continue
                row = run_row("16K", factor, seed, track, cfg, baseline_elf, model, MultiExitGame, node_ids)
                runs.append(row)
                print("16K", factor, seed, round(row["duration_s"], 6), row["winner"], flush=True)
                (HERE / "heldout_progress.json").write_text(json.dumps(
                    dict(metadata, runs=runs, source_hashes=hashes, complete=False),
                    ensure_ascii=False, indent=2) + "\n", encoding="utf-8")

    elf, hashes["16L"] = build((RANGE_DEFINE,), "16L_heldout", FIRMWARE)
    for factor in args.factors:
        for seed in args.seeds:
            row = run_row("16L", factor, seed, track, cfg, elf, model, MultiExitGame, node_ids)
            runs.append(row)
            print("16L", factor, seed, round(row["duration_s"], 6), row["winner"],
                  "hides", row["arrived_hide_nodes"], "contact", row["collision_kind"], flush=True)
            (HERE / "heldout_progress.json").write_text(json.dumps(
                dict(metadata, runs=runs, source_hashes=hashes, complete=False),
                ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    summary, paired, deltas = summarize(runs, args.factors)
    report = dict(metadata, summary=summary, paired_runs=paired, per_factor_delta=deltas,
                  runs=runs, source_hashes=hashes, complete=True)
    (HERE / "heldout_comparison.json").write_text(json.dumps(report, ensure_ascii=False, indent=2)
                                                + "\n", encoding="utf-8")
    (HERE / "heldout_progress.json").write_text(json.dumps(report, ensure_ascii=False, indent=2)
                                              + "\n", encoding="utf-8")
    print(json.dumps(deltas, ensure_ascii=False), flush=True)


if __name__ == "__main__":
    main()
