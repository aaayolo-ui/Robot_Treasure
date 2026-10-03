"""Compress the existing track into localization landmarks, without changing it."""
from __future__ import annotations

import copy

TYPE_LABELS = {"corner": "直角拐弯", "t_junction": "T 型路口", "cross": "十字路口",
               "endpoint": "死路端点", "police_spawn": "警方出生点",
               "thief_spawn": "小偷出生点", "treasure": "宝藏点"}


def landmark_type(track, original):
    neighbors = list(track.graph[original])
    degree = len(neighbors)
    if degree == 1:
        return "endpoint"
    if degree == 3:
        return "t_junction"
    if degree == 4:
        return "cross"
    if degree == 2:
        a, b = (track.xy[n] for n in neighbors)
        if a[0] != b[0] and a[1] != b[1]:
            return "corner"
    return None


def direction(a, b):
    if a[0] == b[0]:
        return "S" if b[1] > a[1] else "N"
    if a[1] == b[1]:
        return "E" if b[0] > a[0] else "W"
    raise ValueError("A localization segment must be straight")


def build_annotations(track, previous=None):
    """Stable IDs survive dictionary order, regeneration, and later additions.

    The exported identity registry reserves old IDs, even if a future map
    removes a landmark. Ordinary grid subdivisions remain segment offsets.
    """
    registry = copy.deepcopy((previous or {}).get("identity_registry", {"nodes": {}, "segments": {}}))
    order = lambda n: (track.xy[n][1], track.xy[n][0])
    roles = {track.resolve(alias): kind for alias, kind in
             (("P", "police_spawn"), ("B", "thief_spawn"), ("T", "treasure"))}
    landmarks = sorted((n for n in track.graph if landmark_type(track, n) or n in roles), key=order)

    def assign(section, key, prefix):
        ids = registry[section]
        if key not in ids:
            maximum = max((int(value[1:]) for value in ids.values()), default=0)
            ids[key] = f"{prefix}{maximum + 1:03d}"
        return ids[key]

    ids = {n: assign("nodes", n, "N") for n in landmarks}
    nodes = []
    for n in landmarks:
        geometry = landmark_type(track, n)
        kinds = ([geometry] if geometry else []) + ([roles[n]] if n in roles else [])
        nodes.append({"id": ids[n], "original_node": n, "xy_grid": list(track.xy[n]),
                      "xy_m": [round(v * track.spacing, 6) for v in track.xy[n]],
                      "types": kinds, "type": roles.get(n, geometry),
                      "geometry_type": geometry or "straight_special",
                      "labels": [TYPE_LABELS[k] for k in kinds], "degree": len(track.graph[n]),
                      "connections": [], "is_track_landmark": geometry is not None})
    by_id = {n["id"]: n for n in nodes}
    visited, segments = set(), []
    for source in landmarks:
        for neighbor in sorted(track.graph[source], key=order):
            if frozenset((source, neighbor)) in visited:
                continue
            path, ticks = [source, neighbor], track.graph[source][neighbor]
            visited.add(frozenset((source, neighbor)))
            while path[-1] not in ids:
                following = [n for n in track.graph[path[-1]] if n != path[-2]]
                if len(following) != 1:
                    raise ValueError("Unlabelled branch or endpoint")
                nxt = following[0]
                ticks += track.graph[path[-1]][nxt]
                visited.add(frozenset((path[-1], nxt)))
                path.append(nxt)
            target = path[-1]
            if order(target) < order(source):
                path.reverse()
            start, end = path[0], path[-1]
            key = f"{start}|{end}"
            sid = assign("segments", key, "S")
            length = round(ticks * track.spacing / 2, 6)
            forward = direction(track.xy[start], track.xy[end])
            reverse = direction(track.xy[end], track.xy[start])
            offsets, distance = [], 0.0
            for i, original in enumerate(path):
                if i:
                    distance += track.graph[path[i - 1]][original] * track.spacing / 2
                offsets.append({"original_node": original, "distance_from_start_m": round(distance, 6)})
            segment = {"id": sid, "start": ids[start], "end": ids[end], "length_m": length,
                       "length_half_cell_ticks": ticks, "direction_from_start": forward,
                       "direction_from_end": reverse, "original_nodes": path,
                       "original_offsets": offsets, "calibrated": False}
            segments.append(segment)
            for a, b, bearing in ((start, end, forward), (end, start, reverse)):
                by_id[ids[a]]["connections"].append({"segment": sid, "neighbor": ids[b],
                                                     "direction": bearing, "length_m": length})
    expected = sum(len(edges) for edges in track.graph.values()) // 2
    if len(visited) != expected:
        raise ValueError("Annotation export did not cover every original edge")
    mapping = {n: {"xy_grid": list(track.xy[n]), "node_id": ids.get(n), "segments": []}
               for n in sorted(track.graph, key=order)}
    for segment in segments:
        for item in segment["original_offsets"]:
            offset = item["distance_from_start_m"]
            mapping[item["original_node"]]["segments"].append({"segment_id": segment["id"],
                "distance_from_start_m": offset, "distance_from_end_m": round(segment["length_m"] - offset, 6)})
    for n in nodes:
        n["connections"].sort(key=lambda c: (c["direction"], c["neighbor"]))
    return {"schema_version": 1, "coordinate_system": "x向右、y向下；grid为地图格坐标，m按格距推算",
            "grid_spacing_m": track.spacing, "calibrated": False,
            "length_note": "沿用原地图0.3米格距推算，尚未实地标定",
            "localization_note": "标注表示地图几何。灰度识别特征尚需实测；相同形状路口不能单独确定全局位置。宝藏点是任务标记，不是新增灰度路口。",
            "type_labels": TYPE_LABELS, "nodes": nodes,
            "segments": sorted(segments, key=lambda s: s["id"]),
            "original_node_mapping": mapping, "identity_registry": registry}
