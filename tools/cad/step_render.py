#!/usr/bin/env python3
"""STEP → 三视图 + 等轴测（线框 + 平面填充），不装 CAD 内核也能看零件长什么样。

    python3 tools/cad/step_render.py "<x.stp>" --out /tmp/cadout

棱线来自 EDGE_CURVE 的两个 VERTEX_POINT；面片来自 PLANE 面的外壳环
（ADVANCED_FACE → FACE_OUTER_BOUND → EDGE_LOOP → ORIENTED_EDGE → EDGE_CURVE）。
圆柱/圆锥面不填充，它们的边界圆会以棱线形式出现——认零件够用。
"""

from __future__ import annotations

import argparse
import math
import os
import re

from PIL import Image, ImageDraw

ENTITY_RE = re.compile(r"#(\d+)\s*=\s*([A-Z0-9_]+)\s*\((.*)\)\s*$", re.S)
PALETTE = [(235, 110, 110), (110, 170, 235), (120, 205, 130), (240, 185, 80),
           (200, 130, 235), (100, 215, 215), (240, 140, 185)]


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


def vertex(ents, rid):
    ent = ents.get(str(rid))
    if not ent or ent[0] != "VERTEX_POINT" or not ent[2]:
        return None
    return point(ents, ent[2][0])


def solid_parts(ents, root):
    seen = set()
    edges, face_ids = [], []
    stack = [str(root)]
    while stack:
        cur = stack.pop()
        if cur in seen:
            continue
        seen.add(cur)
        ent = ents.get(cur)
        if not ent:
            continue
        kind, _args, refs = ent
        if kind == "EDGE_CURVE" and len(refs) >= 2:
            a, b = vertex(ents, refs[0]), vertex(ents, refs[1])
            if a and b:
                edges.append((a, b))
            continue
        if kind == "ADVANCED_FACE":
            face_ids.append(cur)
        stack.extend(str(r) for r in refs)

    loops = []
    for fid in face_ids:
        refs = ents[fid][2]
        if not refs:
            continue
        surface = ents.get(str(refs[-1]))
        if not surface or surface[0] != "PLANE":
            continue
        for br in refs[:-1]:
            bound = ents.get(str(br))
            if not bound or bound[0] != "FACE_OUTER_BOUND" or not bound[2]:
                continue
            loop = ents.get(str(bound[2][0]))
            if not loop or loop[0] != "EDGE_LOOP":
                continue
            pts = []
            for oe in loop[2]:
                oent = ents.get(str(oe))
                if not oent or not oent[2]:
                    continue
                ec = ents.get(str(oent[2][-1]))
                if not ec or ec[0] != "EDGE_CURVE" or len(ec[2]) < 2:
                    continue
                p = vertex(ents, ec[2][0])
                if p:
                    pts.append(p)
            if len(pts) >= 3:
                loops.append(pts)
    return edges, loops


def project(p, view):
    if view == "iso":
        ca, sa = math.cos(math.radians(30)), math.sin(math.radians(30))
        return ((p[0] - p[1]) * ca, p[2] - (p[0] + p[1]) * sa)
    if view == "xy":
        return (p[0], p[1])
    if view == "xz":
        return (p[0], p[2])
    return (p[1], p[2])


def render_view(size, edges, loops, view, box, color):
    img = Image.new("RGB", (size, size), (16, 19, 23))
    d = ImageDraw.Draw(img)
    x0, y0, x1, y1 = box
    s = (size - 40) / max(x1 - x0, y1 - y0, 1e-6)

    def m(p):
        u, v = project(p, view)
        return (20 + (u - x0) * s, size - 20 - (v - y0) * s)

    for loop in loops:
        d.polygon([m(p) for p in loop], fill=(46, 55, 66))
    for a, b in edges:
        d.line([m(a), m(b)], fill=color, width=1)
    return img


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("step")
    ap.add_argument("--out", default="/tmp/cadout")
    ap.add_argument("--size", type=int, default=430)
    args = ap.parse_args()
    os.makedirs(args.out, exist_ok=True)
    ents = parse(args.step)
    roots = sorted((r for r in ents if ents[r][0] == "MANIFOLD_SOLID_BREP"), key=int)
    size = args.size
    print(f"{len(roots)} 个实体 → {args.out}")
    for i, rid in enumerate(roots, 1):
        edges, loops = solid_parts(ents, rid)
        pts = [p for e in edges for p in e] + [p for loop in loops for p in loop]
        if not pts:
            print(f"  实体 {i}: 无线框（可能全是曲面）")
            continue
        xs = [p[0] for p in pts]
        ys = [p[1] for p in pts]
        zs = [p[2] for p in pts]
        diag = max(max(xs) - min(xs), max(ys) - min(ys), max(zs) - min(zs))
        cx, cy, cz = (max(xs) + min(xs)) / 2, (max(ys) + min(ys)) / 2, (max(zs) + min(zs)) / 2
        h = diag / 2
        boxes = {"xy": (cx - h, cy - h, cx + h, cy + h),
                 "xz": (cx - h, cz - h, cx + h, cz + h),
                 "yz": (cy - h, cz - h, cy + h, cz + h),
                 "iso": (cx - h, cz - h, cx + h, cz + h)}
        color = PALETTE[(i - 1) % len(PALETTE)]
        sheet = Image.new("RGB", (size * 2, size * 2), (16, 19, 23))
        for vi, view in enumerate(("xy", "xz", "yz", "iso")):
            sub = render_view(size, edges, loops, view, boxes[view], color)
            sheet.paste(sub, ((vi % 2) * size, (vi // 2) * size))
            ImageDraw.Draw(sheet).text(
                ((vi % 2) * size + 10, (vi // 2) * size + 8), view.upper(), fill=(150, 160, 175))
        path = os.path.join(args.out, f"wire-{i}.png")
        sheet.save(path)
        print(f"  实体 {i}: {len(edges)} 棱线 / {len(loops)} 平面环 → {path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
