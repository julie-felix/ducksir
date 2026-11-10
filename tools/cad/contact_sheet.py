#!/usr/bin/env python3
"""看清「哪根轴在哪」的两张图。

    python3 tools/cad/contact_sheet.py "<x.stp>" --out /tmp/cadout

1. `sheet-parts.png`：每个机构零件单独一张（1 连杆块 / 4 平台 / 5、6 电机），
   灰底 + 它自己的轴线，标签放大——先认「零件」，再认「轴」。
2. `sheet-axes.png`：整机灰色幽灵底图，只画候选的 7 条轴，线拉长、标签放大。
"""

from __future__ import annotations

import argparse
import math
import os
import sys

from PIL import Image, ImageDraw, ImageFont

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import hinges as H  # noqa: E402
import step_render as R  # noqa: E402

COLORS = {1: (255, 96, 96), 2: (110, 170, 255), 3: (126, 226, 158), 4: (255, 190, 62),
          5: (208, 124, 255), 6: (96, 230, 230), 7: (255, 140, 190)}
# PIL 自带字体没有中文，标题用 ASCII，避免方框
TITLES = {1: "solid 1 - link block?", 4: "solid 4 - platform / foot?",
          5: "solid 5 - motor?", 6: "solid 6 - motor or bracket?"}


def axis_segment(h, half):
    p, d = h["point_mm"], h["dir"]
    p0 = tuple(p[i] - d[i] * half for i in range(3))
    p1 = tuple(p[i] + d[i] * half for i in range(3))
    return p0, p1


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("step")
    ap.add_argument("--out", default="/tmp/cadout")
    ap.add_argument("--size", type=int, default=560)
    args = ap.parse_args()
    os.makedirs(args.out, exist_ok=True)
    ents = R.parse(args.step)
    roots = sorted((r for r in ents if ents[r][0] == "MANIFOLD_SOLID_BREP"), key=int)
    picks = [1, 4, 5, 6]

    panels = []
    for i in picks:
        rid = roots[i - 1]
        solids = [R.solid_parts(ents, rid, all_faces=True)]
        pts = [p for loop in solids[0][1] for p in loop] + [p for e in solids[0][0] for p in e]
        xs = [p[0] for p in pts]
        ys = [p[1] for p in pts]
        zs = [p[2] for p in pts]
        diag = math.dist((min(xs), min(ys), min(zs)), (max(xs), max(ys), max(zs))) or 100.0
        hs = H.merge(H.collect(ents, rid))[:3]
        ann = []
        for k, h in enumerate(hs, 1):
            p0, p1 = axis_segment(h, diag * 0.42)
            ann.append((p0, p1, COLORS[i], f"{i}-{k}"))
        img = R.rasterize(solids, "iso", args.size, args.size, annotations=ann,
                          base_override=(120, 124, 132), label_size=30)
        img = img.crop((int(args.size * 0.06), int(args.size * 0.06),
                        int(args.size * 0.94), int(args.size * 0.94)))
        panels.append((TITLES.get(i, f"实体 {i}"), img))

    pw, ph = panels[0][1].size
    sheet = Image.new("RGB", (pw * 2, (ph + 34) * 2), (10, 12, 16))
    d = ImageDraw.Draw(sheet)
    try:
        font = ImageFont.load_default(size=22)
    except TypeError:
        font = ImageFont.load_default()
    for idx, (title, img) in enumerate(panels):
        x = (idx % 2) * pw
        y = (idx // 2) * (ph + 34)
        sheet.paste(img, (x, y + 34))
        d.text((x + 12, y + 6), title, fill=(220, 228, 238), font=font)
    p1 = os.path.join(args.out, "sheet-parts.png")
    sheet.save(p1)
    print("→", p1)

    # 整机幽灵图：只画候选轴
    solids = [R.solid_parts(ents, rid, all_faces=True) for rid in roots]
    allp = [p for s in solids for loop in s[1] for p in loop]
    xs = [p[0] for p in allp]
    ys = [p[1] for p in allp]
    zs = [p[2] for p in allp]
    diag = math.dist((min(xs), min(ys), min(zs)), (max(xs), max(ys), max(zs))) or 300.0
    wanted = {1: [1, 2, 3], 4: [1, 2], 5: [1], 6: [1]}
    ann = []
    for si, ks in wanted.items():
        hs = H.merge(H.collect(ents, roots[si - 1]))
        for k in ks:
            if k - 1 >= len(hs):
                continue
            p0, p1 = axis_segment(hs[k - 1], diag * 0.30)
            ann.append((p0, p1, COLORS[si], f"{si}-{k}"))
    img = R.rasterize(solids, "iso", int(args.size * 1.7), int(args.size * 1.7),
                      annotations=ann, base_override=(96, 100, 108), label_size=30)
    p2 = os.path.join(args.out, "sheet-axes.png")
    img.save(p2)
    print("→", p2)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
