#!/usr/bin/env python3
"""从 STEP 里"认零件"：按实体分组、量包围盒、清点圆柱面（铰点）、渲染三视图。

    python3 tools/cad/step_analyze.py "<x.stp>" --out /tmp/cadout

为什么不用 CAD 内核：本机没有 FreeCAD/OpenCascade。STEP 是文本，B-rep 的
`CYLINDRICAL_SURFACE` 本身带着**轴线位置 + 方向 + 半径**，这恰好就是铰链的信息，
所以「连杆 = 两个平行圆柱孔、中心距 L、半径 r」可以直接从文本里读出来。

输出：
  <out>/solid-<n>-<n>.png   每个实体的三视图（XY / XZ / YZ 散点）
  <out>/all-iso.png         全部实体的等轴测散点（按实体育色）
  <out>/report.md           清单：尺寸、柱面清点、平行孔对（中心距）
"""

from __future__ import annotations

import argparse
import collections
import math
import os
import re

from PIL import Image, ImageDraw

ENTITY_RE = re.compile(r"#(\d+)\s*=\s*([A-Z0-9_]+)\s*\((.*)\)\s*$", re.S)
PALETTE = [(230, 90, 90), (90, 160, 230), (110, 200, 120), (240, 180, 70),
           (190, 120, 230), (90, 210, 210), (240, 130, 180)]


def parse(path):
    text = open(path, encoding="utf-8", errors="ignore").read()
    body = text.split("DATA;", 1)[-1]
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
        ents[m.group(1)] = (m.group(2), m.group(3), [int(x) for x in re.findall(r"#(\d+)", m.group(3))])
    return ents


def nums(s):
    return [float(x) for x in re.findall(r"[-+]?\d*\.\d+(?:E[-+]?\d+)?|[-+]?\d+", s)]


def point_of(ents, rid):
    ent = ents.get(str(rid))
    if not ent or ent[0] != "CARTESIAN_POINT":
        return None
    v = nums(ent[1])
    return tuple(v[-3:]) if len(v) >= 3 else None


def dir_of(ents, rid):
    ent = ents.get(str(rid))
    if not ent or ent[0] != "DIRECTION":
        return None
    v = nums(ent[1])
    return tuple(v[-3:]) if len(v) >= 3 else None


def axis_of(ents, rid):
    """AXIS2_PLACEMENT_3D(#point,#dir,#ref) → (location, direction)。"""
    ent = ents.get(str(rid))
    if not ent or ent[0] != "AXIS2_PLACEMENT_3D" or len(ent[2]) < 2:
        return None
    loc = point_of(ents, ent[2][0])
    d = dir_of(ents, ent[2][1])
    if not loc or not d:
        return None
    n = math.sqrt(sum(c * c for c in d)) or 1.0
    return loc, tuple(c / n for c in d)


def collect(ents, root, kinds):
    seen, out = set(), []
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
        if kind == "CARTESIAN_POINT":
            v = nums(args)
            if len(v) >= 3:
                out.append(("pt", tuple(v[-3:])))
            continue
        if kind in kinds:
            out.append((kind, cur, args))
        stack.extend(str(r) for r in refs)
    return out


def parallel_pair(cyls, tol_deg=3.0, min_r=4.0):
    """找平行且不同轴的圆柱面对：连杆的两个铰孔。"""
    best = None
    for i in range(len(cyls)):
        for j in range(i + 1, len(cyls)):
            (l1, d1, r1), (l2, d2, r2) = cyls[i], cyls[j]
            dot = abs(sum(a * b for a, b in zip(d1, d2)))
            ang = math.degrees(math.acos(min(1.0, dot)))
            if ang > tol_deg or min(r1, r2) < min_r:
                continue
            # 两轴距离（点到直线的垂距）
            w = [l2[k] - l1[k] for k in range(3)]
            proj = sum(w[k] * d1[k] for k in range(3))
            perp = [w[k] - proj * d1[k] for k in range(3)]
            dist = math.sqrt(sum(c * c for c in perp))
            if dist < 1.0:
                continue
            cand = (dist, r1, r2, ang)
            if best is None or dist > best[0]:  # 中心距越大越像连杆主孔
                best = cand
    return best


def render(points, path, size=360, pad=18):
    img = Image.new("RGB", (size * 3, size), (16, 19, 23))
    d = ImageDraw.Draw(img)
    if not points:
        img.save(path)
        return
    views = [("XY 俯视", 0, 1), ("XZ 正视", 0, 2), ("YZ 侧视", 1, 2)]
    for vi, (_label, ax, ay) in enumerate(views):
        xs = [p[ax] for p in points]
        ys = [p[ay] for p in points]
        x0, x1, y0, y1 = min(xs), max(xs), min(ys), max(ys)
        sx = (size - 2 * pad) / max(x1 - x0, 1e-6)
        sy = (size - 2 * pad) / max(y1 - y0, 1e-6)
        s = min(sx, sy)
        ox = vi * size + pad + (size - 2 * pad - (x1 - x0) * s) / 2
        oy = pad + (size - 2 * pad - (y1 - y0) * s) / 2
        for p in points:
            u = ox + (p[ax] - x0) * s
            v = size - (oy + (p[ay] - y0) * s)
            d.point((u, v), fill=(200, 214, 230))
    img.save(path)


def render_iso(groups, path, size=900, pad=30):
    img = Image.new("RGB", (size, size), (16, 19, 23))
    d = ImageDraw.Draw(img)
    ca, sa = math.cos(math.radians(30)), math.sin(math.radians(30))
    pts2 = []
    for gi, pts in enumerate(groups):
        for p in pts:
            u = (p[0] - p[1]) * ca
            v = p[2] - (p[0] + p[1]) * sa
            pts2.append((u, v, gi))
    if not pts2:
        img.save(path)
        return
    us = [p[0] for p in pts2]
    vs = [p[1] for p in pts2]
    x0, x1, y0, y1 = min(us), max(us), min(vs), max(vs)
    s = (size - 2 * pad) / max(x1 - x0, y1 - y0, 1e-6)
    for u, v, gi in pts2:
        d.point((pad + (u - x0) * s, size - pad - (v - y0) * s), fill=PALETTE[gi % len(PALETTE)])
    img.save(path)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("step")
    ap.add_argument("--out", default="/tmp/cadout")
    args = ap.parse_args()
    os.makedirs(args.out, exist_ok=True)
    ents = parse(args.step)
    roots = sorted((rid for rid in ents if ents[rid][0] == "MANIFOLD_SOLID_BREP"), key=int)

    lines = ["# STEP 实体清单", "",
             f"文件 `{os.path.basename(args.step)}` · 实体 {len(roots)} 个", ""]
    all_pts = []
    for i, rid in enumerate(roots, 1):
        name = ents[rid][1].split(",")[0].strip().strip("'")[:24]
        items = collect(ents, rid, {"CYLINDRICAL_SURFACE", "PLANE", "CONICAL_SURFACE"})
        pts = [v for kind, *rest in items if kind == "pt" for v in [rest[0]]]
        cyl_info = []
        for item in items:
            if item[0] != "CYLINDRICAL_SURFACE":
                continue
            args_ = item[2]
            refs = [int(x) for x in re.findall(r"#(\d+)", args_)]
            rad = nums(args_)
            if not refs:
                continue
            ax = axis_of(ents, refs[-1])
            if ax and rad:
                cyl_info.append((ax[0], ax[1], rad[-1]))
        planes = sum(1 for it in items if it[0] == "PLANE")
        cones = sum(1 for it in items if it[0] == "CONICAL_SURFACE")
        if pts:
            xs = [p[0] for p in pts]
            ys = [p[1] for p in pts]
            zs = [p[2] for p in pts]
            dims = (max(xs) - min(xs), max(ys) - min(ys), max(zs) - min(zs))
            ctr = (sum(xs) / len(xs), sum(ys) / len(ys), sum(zs) / len(zs))
        else:
            dims, ctr = (0, 0, 0), (0, 0, 0)
        all_pts.append(pts)
        render(pts, os.path.join(args.out, f"solid-{i}.png"))

        pair = parallel_pair(cyl_info)
        big = sorted(cyl_info, key=lambda c: -c[2])[:4]
        lines += [f"## 实体 {i} · `{name}`",
                  f"- 包围盒 {dims[0]:.1f} × {dims[1]:.1f} × {dims[2]:.1f} mm（中心 {ctr[0]:.0f},{ctr[1]:.0f},{ctr[2]:.0f}）",
                  f"- 顶点 {len(pts)} · 平面 {planes} · 圆柱面 {len(cyl_info)} · 锥面 {cones}"]
        if big:
            lines.append("- 最大圆柱：" + "；".join(
                f"r={c[2]:.1f}mm @({c[0][0]:.0f},{c[0][1]:.0f},{c[0][2]:.0f}) 轴({c[1][0]:.2f},{c[1][1]:.2f},{c[1][2]:.2f})"
                for c in big))
        if pair:
            lines.append(f"- **平行孔对**：中心距 {pair[0]:.2f} mm，半径 {pair[1]:.1f}/{pair[2]:.1f} mm，夹角 {pair[3]:.1f}°"
                         " → 连杆候选")
        lines.append(f"- 三视图：`solid-{i}.png`")
        lines.append("")

    render_iso(all_pts, os.path.join(args.out, "all-iso.png"))
    lines += ["## 全实体等轴测", "", "`all-iso.png`（按实体育色，颜色顺序 = 实体序号）", ""]
    with open(os.path.join(args.out, "report.md"), "w", encoding="utf-8") as f:
        f.write("\n".join(lines))
    print("\n".join(lines))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
