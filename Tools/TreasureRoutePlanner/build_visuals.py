"""Render diagrams from the graph; Pillow is only needed for PNG export."""
import html
import json
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont

from planner import ROOT, TrackMap

WIDTH, HEIGHT = 1150, 1110
OX, OY, CELL = 90, 240, 82
COLORS = {"track": "#38414a", "police": "#2563b0", "thief": "#cf3545",
          "large": "#008578", "central": "#bb7100", "muted": "#5c6570"}
FONT_PATH = "C:/Windows/Fonts/msyh.ttc"


def font(size):
    return ImageFont.truetype(FONT_PATH, size)


def point(xy):
    return OX + xy[0] * CELL, OY + xy[1] * CELL


def dashed(draw, a, b, fill, width=8, on=14, off=9):
    length = abs(b[0] - a[0]) + abs(b[1] - a[1])
    if not length:
        return
    for offset in range(0, round(length), on + off):
        end = min(offset + on, length)
        draw.line([(a[0] + (b[0] - a[0]) * offset / length,
                    a[1] + (b[1] - a[1]) * offset / length),
                   (a[0] + (b[0] - a[0]) * end / length,
                    a[1] + (b[1] - a[1]) * end / length)], fill=fill, width=width)


def base(track, title, legends):
    image = Image.new("RGB", (WIDTH, HEIGHT), "white")
    draw = ImageDraw.Draw(image)
    draw.text((56, 26), title, fill="#1e2832", font=font(38))
    draw.text((56, 86), "黑线为行驶路线  ·  网格间距按 0.30 m 推算", fill=COLORS["muted"], font=font(22))
    for i, (text, color, style) in enumerate(legends):
        x = 56 + i * 545
        if style == "dash":
            dashed(draw, (x, 149), (x + 54, 149), color)
        else:
            draw.line([(x, 149), (x + 54, 149)], fill=color, width=8)
        draw.text((x + 68, 130), text, fill="#1e2832", font=font(22))
    for x in range(12):
        draw.text((OX + x * CELL, OY - 40), str(x), fill=COLORS["muted"], font=font(19), anchor="mm")
    for y in range(10):
        draw.text((OX - 38, OY + y * CELL), str(y), fill=COLORS["muted"], font=font(19), anchor="mm")
    for u, adjacent in track.graph.items():
        for v in adjacent:
            if u < v:
                draw.line([point(track.xy[u]), point(track.xy[v])], fill=COLORS["track"], width=5)
    for x, y, orientation in track.data["endpoint_caps"]:
        px, py = point((x, y))
        delta = 16
        a, b = ((px - delta, py), (px + delta, py)) if orientation == "horizontal" else ((px, py - delta), (px, py + delta))
        draw.line([a, b], fill=COLORS["track"], width=5)
    for u, adjacent in track.graph.items():
        if len(adjacent) > 2:
            px, py = point(track.xy[u])
            draw.ellipse((px - 4, py - 4, px + 4, py + 4), fill=COLORS["track"])
    draw.text((56, HEIGHT - 76), "坐标：左上为 (0,0)，x 向右，y 向下；端帽表示路线终点。", fill=COLORS["muted"], font=font(20))
    draw.text((56, HEIGHT - 42), "按截图描图及离线计算；米制长度与实车行驶时间需现场复核。", fill=COLORS["muted"], font=font(20))
    return image, draw


def overlay(draw, route, color, style="solid", arrows=True):
    for leg in route["legs"]:
        a, b = point(parse_xy(leg["from"])), point(parse_xy(leg["to"]))
        if style == "dash":
            dashed(draw, a, b, color)
        else:
            draw.line([a, b], fill=color, width=8)
        if arrows and leg["length_m"] >= .3:
            mx, my = (a[0] + b[0]) / 2, (a[1] + b[1]) / 2
            ux, uy = (0 if a[0] == b[0] else 1 if b[0] > a[0] else -1,
                      0 if a[1] == b[1] else 1 if b[1] > a[1] else -1)
            draw.polygon([(mx + ux * 13, my + uy * 13),
                          (mx - ux * 8 - uy * 9, my - uy * 8 + ux * 9),
                          (mx - ux * 8 + uy * 9, my - uy * 8 - ux * 9)], fill=color)


def parse_xy(name):
    return tuple(float(x) for x in name[1:-1].split(","))


def specials(draw, track):
    for alias in ("P", "B", "T"):
        px, py = point(track.xy[track.special[alias]])
        color = COLORS["police"] if alias == "P" else COLORS["thief"] if alias == "B" else "#936900"
        if alias == "T":
            draw.polygon([(px, py - 12), (px + 12, py), (px, py + 12), (px - 12, py)], fill="#e4bf38", outline=color)
            draw.text((px, py - 72), "T 宝藏中心", fill="#1e2832", font=font(23), anchor="mm")
        else:
            draw.ellipse((px - 10, py - 10, px + 10, py + 10), fill=color, outline="white", width=2)
            label = "P 警方起点" if alias == "P" else "B 小偷起点"
            draw.text((px + 19, py + 16), label, fill=color, font=font(22))


def vector_graph(track):
    # A separate editable SVG contains every elementary graph node.
    parts = ['<svg xmlns="http://www.w3.org/2000/svg" width="1150" height="1110" viewBox="0 0 1150 1110">',
             '<rect width="1150" height="1110" fill="white"/>',
             '<g font-family="Microsoft YaHei, sans-serif" fill="#202a35">',
             '<text x="56" y="65" font-size="34">密室夺宝 节点拓扑图</text>',
             '<text x="56" y="110" font-size="20">空心点为中间采样点，实心点为岔路；悬停可查看坐标与度数。</text>']
    for u, adjacent in track.graph.items():
        for v in adjacent:
            if u < v:
                a, b = point(track.xy[u]), point(track.xy[v])
                parts.append(f'<line x1="{a[0]}" y1="{a[1]}" x2="{b[0]}" y2="{b[1]}" stroke="#38414a" stroke-width="4"/>')
    for u, xy in track.xy.items():
        px, py = point(xy)
        deg = len(track.graph[u])
        fill = "#38414a" if deg >= 3 else "white"
        parts.append(f'<circle cx="{px}" cy="{py}" r="5" fill="{fill}" stroke="#38414a" stroke-width="2"><title>{html.escape(u)} degree {deg}</title></circle>')
    for alias, node in track.special.items():
        px, py = point(track.xy[node])
        parts.append(f'<circle cx="{px}" cy="{py}" r="10" fill="#e4bf38" stroke="#38414a"/><text x="{px+15}" y="{py-15}" font-size="22">{alias}</text>')
    for x in range(12):
        parts.append(f'<text x="{OX+x*CELL}" y="{OY-30}" text-anchor="middle" font-size="18">{x}</text>')
    for y in range(10):
        parts.append(f'<text x="{OX-35}" y="{OY+y*CELL+6}" text-anchor="middle" font-size="18">{y}</text>')
    stats = track.structure()
    parts.append(f'<text x="56" y="1070" font-size="20">{stats["node_count"]} 个采样节点 · {stats["edge_count"]} 条无向边 · {stats["cycle_rank"]} 个独立环路 · 米制长度按网格推算</text></g></svg>')
    (ROOT / "map_graph.svg").write_text("\n".join(parts), encoding="utf-8")


def audit_source_geometry(track):
    """Compare all neighboring grid links with the supplied screenshot."""
    source = Image.open(ROOT / "reference_map.png").convert("RGB")
    xs, ys = track.data["image_x_px"], track.data["image_y_px"]
    expected = set()
    for y, start, end in track.data["horizontal_segments"]:
        expected.update(("H", x, y) for x in range(start, end))
    for x, start, end in track.data["vertical_segments"]:
        expected.update(("V", x, y) for y in range(start, end))
    detected = set()
    support = {}
    candidates = [("H", x, y) for x in range(11) for y in range(10)]
    candidates += [("V", x, y) for x in range(12) for y in range(9)]
    for axis, x, y in candidates:
        horizontal = axis == "H"
        a = xs[x], ys[y]
        b = (xs[x + 1], ys[y]) if horizontal else (xs[x], ys[y + 1])
        hits = 0
        for i in range(31):
            fraction = .1 + .8 * i / 30
            px, py = round(a[0] + (b[0] - a[0]) * fraction), round(a[1] + (b[1] - a[1]) * fraction)
            # The screenshot has up to ~5 px line-alignment distortion.
            hits += any(max(source.getpixel((px if horizontal else px + k,
                                             py + k if horizontal else py))) < 90
                        for k in range(-7, 8))
        ratio = hits / 31
        support[axis, x, y] = ratio
        if ratio >= .9:
            detected.add((axis, x, y))
    missed = sorted(detected - expected)
    unsupported = sorted(expected - detected)
    if missed or unsupported:
        raise ValueError(f"Map-image mismatch: omitted={missed}, unsupported={unsupported}")
    print(f"Screenshot audit: {len(candidates)} possible links checked; "
          f"{len(expected)} modeled links match; min support={min(support[x] for x in expected):.3f}")


def main():
    track = TrackMap()
    audit_source_geometry(track)
    result = json.loads((ROOT / "results.json").read_text(encoding="utf-8"))
    image, draw = base(track, "密室夺宝 取宝路线", [
        ("警方 5.25 m  /  7 次转弯", COLORS["police"], "solid"),
        ("小偷 3.15 m  /  4 次转弯", COLORS["thief"], "dash")])
    overlay(draw, result["routes"]["police"]["distance"], COLORS["police"])
    overlay(draw, result["scenarios"]["thief_upper_route"], COLORS["thief"], "dash")
    specials(draw, track)
    image.save(ROOT / "route_paths.png")
    image, draw = base(track, "密室夺宝 取宝后的候选回路", [
        ("左侧大回路 9.00 m", COLORS["large"], "solid"),
        ("中央窄回路 3.60 m", COLORS["central"], "dash")])
    overlay(draw, result["cycles"]["left_large"], COLORS["large"])
    overlay(draw, result["cycles"]["central"], COLORS["central"], "dash")
    specials(draw, track)
    image.save(ROOT / "escape_cycles.png")
    vector_graph(track)
    print("Rendered route_paths.png, escape_cycles.png, map_graph.svg")


if __name__ == "__main__":
    main()
