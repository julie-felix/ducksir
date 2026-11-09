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


def solid_parts(ents, root, all_faces=False):
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
        if not all_faces and (not surface or surface[0] != "PLANE"):
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


def newell(loop):
    nx = ny = nz = 0.0
    n = len(loop)
    for i in range(n):
        x0, y0, z0 = loop[i]
        x1, y1, z1 = loop[(i + 1) % n]
        nx += (y0 - y1) * (z0 + z1)
        ny += (z0 - z1) * (x0 + x1)
        nz += (x0 - x1) * (y0 + y1)
    norm = math.sqrt(nx * nx + ny * ny + nz * nz) or 1.0
    return nx / norm, ny / norm, nz / norm


VIEW_DIR = {"xy": (0, 0, -1), "xz": (0, -1, 0), "yz": (-1, 0, 0),
            "iso": (-0.577, -0.577, -0.577)}


def render_assembly(size, solids, view, shade=True):
    """把多个实体画在一张图上：面按深度排序填充（画家算法），棱线最后画。"""
    faces, lines = [], []
    vd = VIEW_DIR.get(view, (-0.577, -0.577, -0.577))
    for idx, (edges, loops) in enumerate(solids):
        base = PALETTE[idx % len(PALETTE)]
        for loop in loops:
            n = newell(loop)
            if sum(n[k] * vd[k] for k in range(3)) >= 0:   # 背面剔除
                continue
            depth = sum(sum(p[k] for k in range(3)) for p in loop) / (3 * len(loop))
            faces.append((depth, loop, base))
        for a, b in edges:
            lines.append((a, b, base))
    pts = [p for _e, l in solids for loop in l for p in loop] + \
          [p for edges, _l in solids for e in edges for p in e]
    if not pts:
        return None
    us = [project(p, view) for p in pts]
    xs = [u[0] for u in us]
    ys = [u[1] for u in us]
    x0, x1, y0, y1 = min(xs), max(xs), min(ys), max(ys)
    margin = 24
    scale = (size - 2 * margin) / max(x1 - x0, y1 - y0, 1e-6)
    img = Image.new("RGB", (size, size), (14, 17, 21))
    d = ImageDraw.Draw(img)

    def m(p):
        u, v = project(p, view)
        return (margin + (u - x0) * scale, size - margin - (v - y0) * scale)

    if shade:
        faces.sort(key=lambda t: t[0])
        for depth, loop, base in faces:
            w = 0.45 + 0.55 * ((depth - min(f[0] for f in faces)) /
                               max(max(f[0] for f in faces) - min(f[0] for f in faces), 1e-6))
            col = tuple(min(255, int(c * (0.35 + 0.65 * w))) for c in base)
            d.polygon([m(p) for p in loop], fill=col)
    for a, b, base in lines:
        d.line([m(a), m(b)], fill=tuple(min(255, int(c * 1.25)) for c in base), width=1)
    return img


def rasterize(solids, view, W=900, H=900):
    """真·z-buffer 软件光栅化：面三角化后逐像素比较深度，画出实体感。"""
    from array import array

    # 投影函数：返回 (u, v, depth)
    def proj(p):
        if view == "iso":
            ca, sa = math.cos(math.radians(30)), math.sin(math.radians(30))
            u = (p[0] - p[1]) * ca
            v = p[2] - (p[0] + p[1]) * sa
            d = -(p[0] + p[1] + p[2]) * 0.577
            return u, v, d
        if view == "xy":
            return p[0], p[1], -p[2]
        if view == "xz":
            return p[0], p[2], -p[1]
        return p[1], p[2], -p[0]

    tris = []  # (p0,p1,p2,color)
    allp = []
    for idx, (_edges, loops) in enumerate(solids):
        base = PALETTE[idx % len(PALETTE)]
        for loop in loops:
            for i in range(1, len(loop) - 1):
                n = newell([loop[0], loop[i], loop[i + 1]])
                tris.append((loop[0], loop[i], loop[i + 1], base, n))
            allp.extend(loop)
    if not allp:
        return None
    us = [proj(p) for p in allp]
    xs = [u[0] for u in us]
    ys = [u[1] for u in us]
    x0, x1, y0, y1 = min(xs), max(xs), min(ys), max(ys)
    scale = (min(W, H) - 40) / max(x1 - x0, y1 - y0, 1e-6)
    ox = (W - (x1 - x0) * scale) / 2
    oy = (H - (y1 - y0) * scale) / 2

    def to_px(p):
        u, v, d = proj(p)
        return (ox + (u - x0) * scale, H - (oy + (v - y0) * scale), d)

    zbuf = array("f", [-1e18]) * (W * H)
    buf = bytearray(W * H * 3)
    for i in range(W * H):
        buf[i * 3] = 14
        buf[i * 3 + 1] = 17
        buf[i * 3 + 2] = 21

    light = (-0.4, -0.6, 0.7)
    for p0, p1, p2, base, n in tris:
        (u0, v0, z0), (u1, v1, z1), (u2, v2, z2) = to_px(p0), to_px(p1), to_px(p2)
        den = (v1 - v2) * (u0 - u2) + (u2 - u1) * (v0 - v2)
        if abs(den) < 1e-9:
            continue
        minx, maxx = max(0, int(min(u0, u1, u2))), min(W - 1, int(max(u0, u1, u2)) + 1)
        miny, maxy = max(0, int(min(v0, v1, v2))), min(H - 1, int(max(v0, v1, v2)) + 1)
        if minx > maxx or miny > maxy:
            continue
        shade = 0.35 + 0.65 * abs(sum(n[k] * light[k] for k in range(3)))
        col = bytes(max(0, min(255, int(c * shade))) for c in base)
        for y in range(miny, maxy + 1):
            base_i = y * W
            for x in range(minx, maxx + 1):
                w0 = ((v1 - v2) * (x - u2) + (u2 - u1) * (y - v2)) / den
                if w0 < 0:
                    continue
                w1 = ((v2 - v0) * (x - u2) + (u0 - u2) * (y - v2)) / den
                if w1 < 0 or w0 + w1 > 1:
                    continue
                w2 = 1 - w0 - w1
                z = w0 * z0 + w1 * z1 + w2 * z2
                idx = base_i + x
                if z > zbuf[idx]:
                    zbuf[idx] = z
                    j = idx * 3
                    buf[j:j + 3] = col
    return Image.frombytes("RGB", (W, H), bytes(buf))


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("step")
    ap.add_argument("--out", default="/tmp/cadout")
    ap.add_argument("--size", type=int, default=430)
    ap.add_argument("--assembly", action="store_true", help="把全部实体画在一张图上")
    ap.add_argument("--only", default=None, help="只画这些实体序号，逗号分隔，如 1,4,5,6,2,3")
    ap.add_argument("--raster", action="store_true", help="用 z-buffer 光栅化（实体感；慢）")
    args = ap.parse_args()
    os.makedirs(args.out, exist_ok=True)
    ents = parse(args.step)
    roots = sorted((r for r in ents if ents[r][0] == "MANIFOLD_SOLID_BREP"), key=int)

    if args.assembly and args.raster:
        keep = None
        if args.only:
            keep = {int(x) for x in args.only.split(",") if x.strip()}
        picked = [(i, rid) for i, rid in enumerate(roots, 1) if keep is None or i in keep]
        solids = [solid_parts(ents, rid, all_faces=True) for _i, rid in picked]
        suffix = f"-only{args.only.replace(',', '')}" if args.only else ""
        for view in ("iso", "xz", "xy"):
            img = rasterize(solids, view, args.size, args.size)
            if img is None:
                continue
            path = os.path.join(args.out, f"raster{suffix}-{view}.png")
            img.save(path)
            print(f"  光栅 {view}: {len(solids)} 个实体 → {path}")
        return 0

    if args.assembly:
        keep = None
        if args.only:
            keep = {int(x) for x in args.only.split(",") if x.strip()}
        picked = [(i, rid) for i, rid in enumerate(roots, 1) if keep is None or i in keep]
        solids = [solid_parts(ents, rid) for _i, rid in picked]
        suffix = f"-only{args.only.replace(',', '')}" if args.only else ""
        for view in ("iso", "xz", "xy", "yz"):
            img = render_assembly(args.size, solids, view)
            if img is None:
                continue
            path = os.path.join(args.out, f"assembly{suffix}-{view}.png")
            img.save(path)
            print(f"  组装 {view}: {len(solids)} 个实体 → {path}")
        return 0

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
