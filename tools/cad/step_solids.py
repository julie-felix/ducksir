#!/usr/bin/env python3
"""STEP(AP214) 里按实体（MANIFOLD_SOLID_BREP）分组量尺寸。

    python3 tools/cad/step_solids.py "<path/to/x.stp>"

只做一件事：把每个实体引用的 CARTESIAN_POINT 收全，给包围盒、尺寸、点数、名字。
不给装配语义（宇树那份文件本来就没有），所以输出要人确认哪根是连杆。

输出 mm（STEP 里常是 mm，脚本会把 units 行读出来打印，但不做换算）。
"""

from __future__ import annotations

import collections
import re
import sys

ENTITY_RE = re.compile(r"#(\d+)\s*=\s*([A-Z0-9_]+)\s*\((.*)\)\s*$", re.S)


def parse(path: str):
    text = open(path, encoding="utf-8", errors="ignore").read()
    body = text.split("DATA;", 1)[-1]
    stmts = []
    buf = []
    for line in body.splitlines():
        if line.strip().startswith("ENDSEC"):
            break
        buf.append(line)
        if line.rstrip().endswith(";"):
            stmts.append(" ".join(buf))
            buf = []
    ents = {}
    for stmt in stmts:
        stmt = stmt.strip().rstrip(";")
        m = ENTITY_RE.match(stmt)
        if not m:
            continue
        rid, kind, args = m.group(1), m.group(2), m.group(3)
        refs = [int(x) for x in re.findall(r"#(\d+)", args)]
        ents[rid] = (kind, args, refs)
    return text, ents


def coords(args: str):
    nums = re.findall(r"[-+]?\d*\.\d+(?:E[-+]?\d+)?|[-+]?\d+", args)
    if len(nums) < 3:
        return None
    return float(nums[-3]), float(nums[-2]), float(nums[-1])


def collect_points(root_id: str, ents: dict) -> list:
    seen, pts = set(), []
    stack = [root_id]
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
            c = coords(args)
            if c:
                pts.append(c)
            continue
        stack.extend(str(r) for r in refs)
    return pts


def main() -> int:
    path = sys.argv[1]
    text, ents = parse(path)
    unit = "mm" if "SI_UNIT(.MILLI.,.METRE.)" in text else "见文件"
    schema = re.search(r"FILE_SCHEMA\s*\(\s*\(\s*'([^']+)'", text)
    print(f"{path}\n量纲 {unit} · schema {schema.group(1) if schema else '?'} · 实体总数 {len(ents)}")

    solids = [(rid, ents[rid]) for rid in ents if ents[rid][0] == "MANIFOLD_SOLID_BREP"]
    print(f"MANIFOLD_SOLID_BREP: {len(solids)}\n")
    print(f"{'#':>2} {'name':<22} {'X (mm)':<20} {'Y (mm)':<20} {'Z (mm)':<20} {'size dx,dy,dz':<26} pts")
    for i, (rid, (_kind, args, _refs)) in enumerate(sorted(solids, key=lambda kv: int(kv[0])), 1):
        name = args.split(",")[0].strip().strip("'")[:22]
        pts = collect_points(rid, ents)
        if not pts:
            print(f"{i:>2} {name:<22} (no points)")
            continue
        xs = [p[0] for p in pts]
        ys = [p[1] for p in pts]
        zs = [p[2] for p in pts]
        fmt = lambda v: f"{v[0]:.1f}..{v[1]:.1f}"  # noqa: E731
        size = f"{max(xs)-min(xs):.1f},{max(ys)-min(ys):.1f},{max(zs)-min(zs):.1f}"
        print(f"{i:>2} {name:<22} {fmt((min(xs),max(xs))):<20} {fmt((min(ys),max(ys))):<20} "
              f"{fmt((min(zs),max(zs))):<20} {size:<26} {len(pts)}")

    colors = sorted({m for m in re.findall(r"COLOUR_RGB\s*\(\s*'[^']*'\s*,\s*([^)]*)\)", text)})
    print(f"\nCOLOUR_RGB 共 {len(colors)} 种：")
    for c in colors:
        print("  ", c.strip())
    print("\n注意：颜色到实体的对应要解 STYLED_ITEM 链，本脚本没做；先按包围盒判定。")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
