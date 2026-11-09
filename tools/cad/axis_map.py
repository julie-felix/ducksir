#!/usr/bin/env python3
"""把铰链轴线画到 CAD 渲染图上并编号，供人确认「哪条是电机轴 / 连杆销 / 平台轴」。

    python3 tools/cad/axis_map.py "<x.stp>" --out /tmp/cadout

输出 axes-iso.png / axes-xz.png：实体按灰阶渲染，轴线画成亮色线段，
标签 `<实体>-<序号>`（例如 `1-3`），序号对应 hinges.md 里同名的那一行。
"""

from __future__ import annotations

import argparse
import json
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import hinges as H  # noqa: E402
import step_render as R  # noqa: E402

SOLID_COLORS = [(255, 90, 90), (90, 170, 255), (120, 230, 140), (255, 190, 60),
                (210, 120, 255), (90, 230, 230), (255, 140, 190)]


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("step")
    ap.add_argument("--out", default="/tmp/cadout")
    ap.add_argument("--per-solid", type=int, default=3, help="每个实体画前几条轴（按半径排序）")
    ap.add_argument("--solids", default="1,4,5,6", help="只标这些实体")
    ap.add_argument("--size", type=int, default=1050)
    args = ap.parse_args()
    os.makedirs(args.out, exist_ok=True)

    ents = R.parse(args.step)
    roots = sorted((r for r in ents if ents[r][0] == "MANIFOLD_SOLID_BREP"), key=int)
    keep = {int(x) for x in args.solids.split(",") if x.strip()}
    picked = [(i, rid) for i, rid in enumerate(roots, 1) if i in keep]

    solids = [R.solid_parts(ents, rid, all_faces=True) for _i, rid in picked]
    boxes = {}
    for idx, (i, _rid) in enumerate(picked):
        edges, loops = solids[idx]
        pts = [p for loop in loops for p in loop] + [p for pair in edges for p in pair]
        if pts:
            xs = [p[0] for p in pts]
            ys = [p[1] for p in pts]
            zs = [p[2] for p in pts]
            boxes[i] = (min(xs), min(ys), min(zs), max(xs), max(ys), max(zs))

    per_solid = {i: H.merge(H.collect(ents, rid)) for i, rid in picked}
    for view in ("iso", "xz", "xy"):
        ann = []
        for idx, (i, _rid) in enumerate(picked):
            color = SOLID_COLORS[(i - 1) % len(SOLID_COLORS)]
            box = boxes.get(i)
            if not box:
                continue
            diag = math.dist(box[:3], box[3:]) or 100.0
            for k, h in enumerate(per_solid[i][: args.per_solid], 1):
                p = h["point_mm"]
                d = h["dir"]
                half = diag * 0.42
                p0 = tuple(p[j] - d[j] * half for j in range(3))
                p1 = tuple(p[j] + d[j] * half for j in range(3))
                ann.append((p0, p1, color, f"{i}-{k}"))
        img = R.rasterize(solids, view, args.size, args.size, annotations=ann)
        if img is None:
            continue
        path = os.path.join(args.out, f"axes-{view}.png")
        img.save(path)
        print(f"  {view}: {len(ann)} 条轴线 → {path}")

    lines = ["# 轴号对照", ""]
    for _idx, (i, _rid) in enumerate(picked):
        lines.append(f"## 实体 {i}（图中标签前缀 `{i}-`）")
        lines.append("")
        lines.append("| 标签 | 轴点 (mm) | 方向 | 半径 mm |")
        lines.append("|---|---|---|---|")
        for k, h in enumerate(per_solid[i][: args.per_solid], 1):
            lines.append(f"| `{i}-{k}` | {h['point_mm']} | {h['dir']} | {h['r_mm']} |")
        lines.append("")
    with open(os.path.join(args.out, "axes.md"), "w", encoding="utf-8") as f:
        f.write("\n".join(lines))
    with open(os.path.join(args.out, "axes.json"), "w", encoding="utf-8") as f:
        json.dump(per_solid, f, indent=1, ensure_ascii=False)
    print("\n".join(lines))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
