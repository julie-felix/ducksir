#!/usr/bin/env python3
"""从 STEP 提「铰点表」：把圆柱面聚成铰链轴线，再跨零件配对。

    python3 tools/cad/hinges.py "<x.stp>" --out /tmp/cadout

为什么这么做：并联机构的 `f` 只需要几个数——每根铰链的轴线位置、方向、销径，
以及各铰链之间的距离。这些在 STEP 里就是 `CYLINDRICAL_SURFACE` 的
（轴线点, 方向, 半径），不需要 CAD 内核。

输出：
  <out>/hinges.md    铰链表 + 跨零件共轴配对 + 铰距矩阵
  <out>/hinges.json  同样的内容，给后续写 f 用
"""

from __future__ import annotations

import argparse
import itertools
import json
import math
import os
import re
import sys

ENTITY_RE = re.compile(r"#(\d+)\s*=\s*([A-Z0-9_]+)\s*\((.*)\)\s*$", re.S)

TOL_POINT_MM = 1.0     # 轴线点合并容差
TOL_ANGLE_DEG = 2.0    # 轴线方向合并容差
TOL_LINE_MM = 2.0      # 共轴判定：点到直线距离


def parse(path):
    text = open(path, encoding="utf-8", errors="ignore").read()
    body = text.split("DATA;", 1)[1] if "DATA;" in text else text
    stmts, buf = [], []
    for line in body.splitlines():
        if line.strip().startswith("ENDSEC"):
            break
        buf.append(line)
        if line.rstrip().endswith(";"):
            stmts.append(" ".join(buf))
            buf = []
    ents = {}
    for stmt in stmts:
        m = ENTITY_RE.match(stmt.strip().rstrip(";"))
        if not m:
            continue
        ents[m.group(1)] = (m.group(2), m.group(3),
                            [int(x) for x in re.findall(r"#(\d+)", m.group(3))])
    return ents


def nums(s):
    return [float(x) for x in re.findall(r"[-+]?\d*\.\d+(?:E[-+]?\d+)?|[-+]?\d+", s)]


def point(ents, rid):
    ent = ents.get(str(rid))
    if not ent or ent[0] != "CARTESIAN_POINT":
        return None
    v = nums(ent[1])
    return tuple(v[-3:]) if len(v) >= 3 else None


def direction(ents, rid):
    ent = ents.get(str(rid))
    if not ent or ent[0] != "DIRECTION":
        return None
    v = nums(ent[1])
    if len(v) < 3:
        return None
    n = math.sqrt(sum(c * c for c in v[-3:])) or 1.0
    return tuple(c / n for c in v[-3:])


def axis(ents, rid):
    ent = ents.get(str(rid))
    if not ent or ent[0] != "AXIS2_PLACEMENT_3D" or len(ent[2]) < 2:
        return None
    p = point(ents, ent[2][0])
    d = direction(ents, ent[2][1])
    return (p, d) if p and d else None


def collect(ents, root):
    """返回该实体引用的所有圆柱面 (点, 方向, 半径)。"""
    seen, cyls = set(), []
    stack = [str(root)]
    while stack:
        cur = stack.pop()
        if cur in seen:
            continue
        seen.add(cur)
        ent = ents.get(cur)
        if not ent:
            continue
        kind, args, refs = ent
        if kind == "CYLINDRICAL_SURFACE" and refs:
            ax = axis(ents, refs[-1])
            rad = nums(args)
            if ax and rad:
                cyls.append((ax[0], ax[1], rad[-1]))
        stack.extend(str(r) for r in refs)
    return cyls


def line_dist(p, q, d):
    w = [p[i] - q[i] for i in range(3)]
    proj = sum(w[i] * d[i] for i in range(3))
    perp = [w[i] - proj * d[i] for i in range(3)]
    return math.sqrt(sum(c * c for c in perp))


def same_axis(a, b):
    (pa, da, _), (pb, db, _) = a, b
    dot = sum(da[i] * db[i] for i in range(3))
    ang = math.degrees(math.acos(min(1.0, abs(dot))))
    if ang > TOL_ANGLE_DEG:
        return False
    return line_dist(pa, pb, da) <= TOL_LINE_MM


def merge(cyls):
    """把同一条轴线上的圆柱面合成一个铰链。"""
    groups = []
    for c in cyls:
        for g in groups:
            if same_axis(c, g[0]):
                g.append(c)
                break
        else:
            groups.append([c])
    out = []
    for g in groups:
        p, d, _r = g[0]
        r = max(x[2] for x in g)
        while len(d) < 3:
            d = tuple(d) + (0.0,)
        out.append({"point_mm": [round(v, 3) for v in p],
                    "dir": [round(v, 4) for v in d],
                    "r_mm": round(r, 2),
                    "faces": len(g)})
    out.sort(key=lambda h: (-h["r_mm"], h["point_mm"][2]))
    return out


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("step")
    ap.add_argument("--out", default="/tmp/cadout")
    args = ap.parse_args()
    os.makedirs(args.out, exist_ok=True)
    ents = parse(args.step)
    roots = sorted((r for r in ents if ents[r][0] == "MANIFOLD_SOLID_BREP"), key=int)

    per_solid = {}
    for i, rid in enumerate(roots, 1):
        cyls = collect(ents, rid)
        per_solid[i] = merge(cyls)

    # 跨零件配对：不同实体上共轴的铰
    pairs = []
    for a, b in itertools.combinations(sorted(per_solid), 2):
        for ha in per_solid[a]:
            for hb in per_solid[b]:
                pa = (ha["point_mm"], ha["dir"], ha["r_mm"])
                pb = (hb["point_mm"], hb["dir"], hb["r_mm"])
                if same_axis(pa, pb):
                    pairs.append({"solid_a": a, "solid_b": b,
                                  "r_a": ha["r_mm"], "r_b": hb["r_mm"],
                                  "dist_mm": round(line_dist(pa[0], pb[0], pa[1]), 2),
                                  "axis_point_mm": ha["point_mm"], "dir": ha["dir"]})

    # 铰距矩阵（每实体内部，取半径最大的几个）
    lines = ["# 铰点表（从 STEP 自动提取）", "",
             f"文件 `{os.path.basename(args.step)}` · 实体 {len(roots)} 个 · 单位 mm", ""]
    for i in sorted(per_solid):
        hs = per_solid[i]
        lines.append(f"## 实体 {i}：{len(hs)} 条轴线")
        lines.append("")
        lines.append("| # | 轴点 (mm) | 方向 | 半径 mm | 圆柱面数 |")
        lines.append("|---|---|---|---|---|")
        for k, h in enumerate(hs[:8], 1):
            lines.append(f"| {k} | {h['point_mm']} | {h['dir']} | {h['r_mm']} | {h['faces']} |")
        if len(hs) > 8:
            lines.append(f"| … | 还有 {len(hs) - 8} 条 | | | |")
        lines.append("")
        big = hs[:5]
        if len(big) >= 2:
            lines.append("**主要轴线间距（中心距，mm）**")
            lines.append("")
            for x, y in itertools.combinations(range(len(big)), 2):
                d = line_dist(big[x]["point_mm"], big[y]["point_mm"], big[x]["dir"])
                par = math.degrees(math.acos(min(1.0, abs(
                    sum(big[x]["dir"][k] * big[y]["dir"][k] for k in range(3))))))
                lines.append(f"- #{x + 1} ↔ #{y + 1}: {d:.2f} mm（夹角 {par:.1f}°）")
            lines.append("")

    lines.append("## 跨零件共轴（同一根销穿过两个零件）")
    lines.append("")
    if pairs:
        lines.append("| 实体 A | 实体 B | r_A mm | r_B mm | 插入深度 mm | 轴点 mm |")
        lines.append("|---|---|---|---|---|---|")
        for p in pairs:
            lines.append(f"| {p['solid_a']} | {p['solid_b']} | {p['r_a']} | {p['r_b']} | "
                         f"{p['dist_mm']} | {p['axis_point_mm']} |")
    else:
        lines.append("（没有跨零件共轴——可能零件之间靠销连接，而销在实体 2/3 里）")
    lines.append("")

    with open(os.path.join(args.out, "hinges.json"), "w", encoding="utf-8") as f:
        json.dump({"solids": per_solid, "cross": pairs}, f, indent=1, ensure_ascii=False)
    text = "\n".join(lines)
    with open(os.path.join(args.out, "hinges.md"), "w", encoding="utf-8") as f:
        f.write(text)
    print(text)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
