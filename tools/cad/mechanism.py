#!/usr/bin/env python3
"""从 STEP 推机构：把「共轴的孔」合成一根销，再算销与销之间的中心距，画 2D 简图。

    python3 tools/cad/mechanism.py "<x.stp>" --out /tmp/cadout

判据（不用眼睛）：
- 两根轴线**垂距 ≤ 1 mm** 且**夹角 ≤ 5°** → 同一根销穿过这两个零件
- 销与销的距离 = 轴线的公垂线段长度（中心距）

输出 mechanism.md（销表 + 销距）+ mechanism.png（正对销轴方向看过去的 2D 图）。
"""

from __future__ import annotations

import argparse
import itertools
import math
import os
import sys

from PIL import Image, ImageDraw, ImageFont

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import hinges as H  # noqa: E402
import step_render as R  # noqa: E402

TOL_LINE = 1.0
TOL_ANGLE = 5.0
COLORS = {1: (255, 96, 96), 2: (110, 170, 255), 3: (126, 226, 158), 4: (255, 190, 62),
          5: (208, 124, 255), 6: (96, 230, 230), 7: (255, 140, 190)}


def angle_between(d1, d2):
    dot = sum(d1[k] * d2[k] for k in range(3))
    return math.degrees(math.acos(min(1.0, abs(dot))))


def build_pins(per_solid):
    """把跨零件的共轴孔合成销。"""
    entries = []
    for s, hs in per_solid.items():
        for k, h in enumerate(hs, 1):
            entries.append({"solid": s, "idx": k, "tag": f"{s}-{k}",
                            "p": h["point_mm"], "d": h["dir"], "r": h["r_mm"]})
    used = set()
    pins = []
    for i, a in enumerate(entries):
        if a["tag"] in used:
            continue
        group = [a]
        used.add(a["tag"])
        for b in entries[i + 1:]:
            if b["tag"] in used or b["solid"] == a["solid"]:
                continue
            if angle_between(a["d"], b["d"]) > TOL_ANGLE:
                continue
            if H.line_dist(a["p"], b["p"], a["d"]) > TOL_LINE:
                continue
            group.append(b)
            used.add(b["tag"])
        pins.append(group)
    return pins


def mid_point(pin):
    ps = [e["p"] for e in pin]
    return tuple(sum(p[k] for p in ps) / len(ps) for k in range(3))


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("step")
    ap.add_argument("--out", default="/tmp/cadout")
    ap.add_argument("--size", type=int, default=1100)
    ap.add_argument("--min-r", type=float, default=5.0, help="只显示半径 ≥ 这个值的销（过滤小螺孔）")
    args = ap.parse_args()
    os.makedirs(args.out, exist_ok=True)

    ents = R.parse(args.step)
    roots = sorted((r for r in ents if ents[r][0] == "MANIFOLD_SOLID_BREP"), key=int)
    per = {i: H.merge(H.collect(ents, rid)) for i, rid in enumerate(roots, 1)}
    pins = build_pins(per)
    # 过滤：保留「跨零件」或「半径够大」的销，小螺孔不进表也不画
    pins = [p for p in pins
            if max(e["r"] for e in p) >= args.min_r
            or len({e["solid"] for e in p}) > 1]

    lines = ["# 机构：销表", "", "## 销（共轴的孔合成）", "",
             "| 销 | 涉及的孔 | 半径 mm | 位置 mm |", "|---|---|---|---|"]
    for i, pin in enumerate(pins, 1):
        tags = " / ".join(e["tag"] for e in pin)
        rs = "/".join(f'{e["r"]:.1f}' for e in pin)
        mp = mid_point(pin)
        lines.append(f"| P{i} | {tags} | {rs} | ({mp[0]:.1f}, {mp[1]:.1f}, {mp[2]:.1f}) |")

    lines += ["", "## 销距（中心距，mm）", "", "| 销对 | 距离 |", "|---|---|"]
    dists = []
    for i, j in itertools.combinations(range(len(pins)), 2):
        a, b = pins[i], pins[j]
        d = H.line_dist(mid_point(a), mid_point(b), a[0]["d"])
        dists.append((d, i + 1, j + 1))
    for d, i, j in sorted(dists):
        if d < 400:
            lines.append(f"| P{i} ↔ P{j} | {d:.2f} |")
    with open(os.path.join(args.out, "mechanism.md"), "w", encoding="utf-8") as f:
        f.write("\n".join(lines))
    print("\n".join(lines))

    # 2D 图：沿销轴方向看
    d0 = pins[0][0]["d"]
    # 构造与 d0 垂直的两个基向量
    tmp = (0.0, 0.0, 1.0) if abs(d0[2]) < 0.9 else (1.0, 0.0, 0.0)
    u = [d0[1] * tmp[2] - d0[2] * tmp[1], d0[2] * tmp[0] - d0[0] * tmp[2], d0[0] * tmp[1] - d0[1] * tmp[0]]
    n = math.sqrt(sum(c * c for c in u)) or 1
    u = [c / n for c in u]
    v = [d0[1] * u[2] - d0[2] * u[1], d0[2] * u[0] - d0[0] * u[2], d0[0] * u[1] - d0[1] * u[0]]

    pts = []
    for i, pin in enumerate(pins, 1):
        mp = mid_point(pin)
        pts.append((sum(mp[k] * u[k] for k in range(3)), sum(mp[k] * v[k] for k in range(3)),
                    max(e["r"] for e in pin), i, pin))
    us = [p[0] for p in pts]
    vs = [p[1] for p in pts]
    span = max(max(us) - min(us), max(vs) - min(vs), 1)
    W = args.size
    pad = 110
    s = (W - 2 * pad) / span
    img = Image.new("RGB", (W, W), (14, 17, 21))
    d = ImageDraw.Draw(img)
    try:
        font = ImageFont.load_default(size=20)
    except TypeError:
        font = ImageFont.load_default()

    def m(x, y):
        return (pad + (x - min(us)) * s, W - pad - (y - min(vs)) * s)

    # 只在「同一个零件上的两个销」之间连线：这才是刚体构件，不是最近邻
    solid_pins = {}
    for idx, (_x, _y, _r, _i, pin) in enumerate(pts):
        for e in pin:
            solid_pins.setdefault(e["solid"], set()).add(idx)
    edges = set()
    for _s, idxs in solid_pins.items():
        for i, j in itertools.combinations(sorted(idxs), 2):
            edges.add((i, j))
    for i, j in sorted(edges):
        dline = math.hypot(pts[i][0] - pts[j][0], pts[i][1] - pts[j][1])
        col = COLORS.get(pts[i][4][0]["solid"], (70, 80, 96))
        d.line([m(pts[i][0], pts[i][1]), m(pts[j][0], pts[j][1])],
               fill=tuple(int(c * 0.55) for c in col), width=2)
        if dline > 25:
            mx, my = m((pts[i][0] + pts[j][0]) / 2, (pts[i][1] + pts[j][1]) / 2)
            d.text((mx + 4, my + 2), f"{dline:.1f}", fill=(150, 160, 175), font=font)

    for x, y, r, idx, pin in pts:
        cx, cy = m(x, y)
        rr = max(5.0, r * s * 0.5)
        col = COLORS.get(pin[0]["solid"], (210, 210, 210))
        d.ellipse([cx - rr, cy - rr, cx + rr, cy + rr], outline=col, width=3)
        d.text((cx + rr + 4, cy - 10), f"P{idx}", fill=col, font=font)

    ly = 16
    for i, pin in enumerate(pins, 1):
        col = COLORS.get(pin[0]["solid"], (210, 210, 210))
        d.rectangle([16, ly, 30, ly + 11], fill=col)
        d.text((38, ly - 5), f"P{i} = " + "/".join(e["tag"] for e in pin),
               fill=(190, 200, 214), font=font)
        ly += 24
    path = os.path.join(args.out, "mechanism.png")
    img.save(path)
    print("→", path)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
