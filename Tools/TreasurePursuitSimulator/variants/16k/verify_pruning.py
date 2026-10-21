"""16K actual ARM C exhaustive/pruned parity; offline, no hardware access."""
import hashlib
import json
from arm_adapter import build, Planner, HERE, FIRMWARE

common = ("THIEF_LASER_MAX_MM=600U",)
reference_elf, reference_hashes = build(common + ("THIEF_ROLLOUT_PRUNE=0",), "no_prune")
selected_elf, selected_hashes = build(common, "prune_check")
assert reference_hashes == selected_hashes, "Parity binaries must use identical sources"
reference, selected = Planner(reference_elf), Planner(selected_elf)
for planner in (reference, selected):
    planner.initial()
report = json.loads((HERE / "report.json").read_text(encoding="utf-8"))
nodes = {int(node["id"][1:]): node for node in report["annotations"]["nodes"]}
offsets = (0, 3, 1, 2)
directions = {"N": 0, "E": 1, "S": 2, "W": 3}
rows = []
tick = 30000
for own, body in ((11, 1), (12, 3), (38, 0), (38, 1), (62, 1)):
    for neighbor in nodes[own]["connections"]:
        heading = directions[neighbor["direction"]]
        for evidence in (False, True):
            channel = offsets.index((heading - body) % 4)
            distance = min(600, round(neighbor["length_m"] * 1000))
            plans = []
            for planner in (reference, selected):
                planner.clear_gate()
                plan = None
                if evidence:
                    for sample in range(4):
                        planner.observe(own, body, tick + sample * 20,
                                        [2000] * 4, [25] * 4, 15, 0)
                for sample in range(5):
                    distances = [2000] * 4
                    distances[channel] = distance
                    result = planner.observe(own, body, tick + 80 + sample * 20,
                                             distances, [25] * 4, 15, 0)
                    if result["plan"]:
                        plan = result["plan"]
                assert plan is not None, (own, body, heading, evidence)
                plans.append(plan)
            ignored = {"rollouts", "model_steps"}
            complete = [{key: value for key, value in plan.items() if key not in ignored}
                        for plan in plans]
            assert complete[0] == complete[1], (own, body, heading, evidence, plans)
            rows.append(dict(node=own, body=body, threat=heading, evidence=evidence,
                             full_models=plans[0]["rollouts"], pruned_models=plans[1]["rollouts"],
                             full_model_steps=plans[0]["model_steps"],
                             pruned_model_steps=plans[1]["model_steps"],
                             goal=plans[1]["goal"], decision=complete[1]))
            tick += 1000
repo = HERE.parents[3]
assert all(hashlib.sha256((repo / name).read_bytes()).hexdigest() == value
           for name, value in selected_hashes.items()), "A compared source changed during parity testing"
payload = {
    "date": "2026-10-14", "firmware": FIRMWARE.name, "cases": len(rows),
    "all_decisions_identical": True, "ignored_work_counters": sorted(ignored),
    "comparison_laser_range_mm": 600,
    "reference_defines": list(common + ("THIEF_ROLLOUT_PRUNE=0",)),
    "selected_defines": list(common), "source_hashes": selected_hashes,
    "source_unchanged_during_run": True,
    "binaries": {"reference_sha256": hashlib.sha256(reference_elf.read_bytes()).hexdigest(),
                 "selected_sha256": hashlib.sha256(selected_elf.read_bytes()).hexdigest()},
    "scope": "Actual ARMCC C algorithms in Cortex-M3 CPU emulator; exhaustive/pruned decision parity at annotated threat directions, with and without endpoint-entry evidence. No hardware or measured planner latency claim.",
    "rows": rows,
}
(HERE / "pruning_result.json").write_text(json.dumps(payload, ensure_ascii=False, indent=2)
                                        + "\n", encoding="utf-8")
print(len(rows), "exhaustive/pruned decisions identical", flush=True)
