#!/usr/bin/env python3
"""机构简图：把铰点投影成 2D，画「点 = 铰、连线 = 可能构件、标注 = 中心距」。

    python3 tools/cad/schematic.py "<x.stp>" --out /tmp/cadout

3D 渲染里标签必然乱；机构简图没有这个问题——它只保留判断机构需要的三件事：
铰在哪（投影后平面坐标）、孔多大（半径）、彼此多远（中心距）。
投影平面按「所有轴线的主方向」自动选：主方向 ≈ Y 就投到 X–Z 平面。
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

COLORS = {1: (255, 96, 96), 4: (255, 190, 62), 5: (208, 124, 255), 6: (96, 230, 230),
          2: (110, 170, 255), 3: (126, 226, 158), 7: (255, 140, 190)}


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("step")
    ap.add_argument("--out", default="/tmp/cadout")
    ap.add_argument("--size", type=int, default=1200)
    ap.add_argument("--solids", default="1,2,3,4,5,6,7")
    ap.add_argument("--per-solid", type=int, default=4)
    ap.add_argument("--link-dist", type=float, default=70.0, help="小于这个距离才画连线(mm)")
    args = ap.parse_args()
    os.makedirs(args.out, exist_ok=True)

    ents = R.parse(args.step)
    roots = sorted((r for r in ents if ents[r][0] == "MANIFOLD_SOLID_BREP"), key=int)
    keep = [int(x) for x in args.solids.split(",") if x.strip()]

    axes = []
    for i in keep:
        hs = H.merge(H.collect(ents, roots[i - 1]))[: args.per_solid]
        for k, h in enumerate(hs, 1):
            axes.append({"tag": f"{i}-{k}", "solid": i, "p": h["point_mm"],
                         "dir": h["dir"], "r": h["r_mm"]})

    # 主方向：出现最多的轴向 → 投影平面 (u, v) 取与之垂直的两个坐标轴
    dirs = [a["dir"] for a in axes]
    ax = [sum(abs(d[i]) for d in dirs) for i in range(3)]
    normal = ax.index(max(ax))          # 轴线主要沿哪个坐标轴
    uv = [i for i in range(3) if i != normal]

    pts = [(a["p"][uv[0]], a["p"][uv[1]], a) for a in axes]
    us = [p[0] for p in pts]
    vs = [p[1] for p in pts]
    u0, u1, v0, v1 = min(us), max(us), min(vs), max(vs)
    W = H_ = args.size
    pad = 80
    s = (W - 2 * pad) / max(u1 - u0, v1 - v0, 1)
    img = Image.new("RGB", (W, H_), (14, 17, 21))
    d = ImageDraw.Draw(img)
    try:
        font = ImageFont.load_default(size=22)
        small = ImageFont.load_default(size=17)
    except TypeError:
        font = small = ImageFont.load_default()

    def m(u, v):
        return (pad + (u - u0) * s, H_ - pad - (v - v0) * s)

    # 先画连线（按半径大的铰两两配对，距离近的才连）
    for a in pts:
        for b in pts:
            if a[2]["tag"] >= b[2]["tag"]:
                continue
            dist = math.hypot(a[0] - b[0], a[1] - b[1])
            if dist > args.link_dist or dist < 5:
                continue
            d.line([m(a[0], a[1]), m(b[0], b[1])], fill=(60, 70, 82), width=2)
            mx, my = m((a[0] + b[0]) / 2, (a[1] + b[1]) / 2)
            d.text((mx + 3, my + 3), f"{dist:.1f}", fill=(120, 132, 148), font=small)

    # 再画铰（圆 = 孔，半径按比例）+ 标签
    for u, v, a in pts:
        x, y = m(u, v)
        rr = max(4.0, a["r"] * s * 0.5)
        col = COLORS.get(a["solid"], (200, 200, 200))
        d.ellipse([x - rr, y - rr, x + rr, y + rr], outline=col, width=3)
        d.text((x + rr + 4, y - 10), f'{a["tag"]}', fill=col, font=font)

    # 图例 + 投影说明
    d.text((16, 12), f"projection: {chr(85 + uv[0])}-{chr(85 + uv[1])}  (axis normal = {'XYZ'[normal]})",
           fill=(150, 160, 175), font=small)
    ly = 40
    for i in keep:
        d.rectangle([16, ly, 30, ly + 12], fill=COLORS.get(i, (200, 200, 200)))
        d.text((38, ly - 4), f"solid {i}", fill=(180, 190, 205), font=small)
        ly += 22
    d.text((16, ly + 8), f"circles = hinges (radius scaled) · numbers = centre distance mm",
           fill=(120, 132, 148), font=small)

    path = os.path.join(args.out, "schematic.png")
    img.save(path)
    print(f"→ {path}  （{len(pts)} 个铰，投影到 {'XYZ'[uv[0]]}-{'XYZ'[uv[1]]}）")
    for u, v, a in sorted(pts, key=lambda t: t[2]["tag"]):
        print(f'   {a["tag"]:>4}  r={a["r"]:5.1f}  ({u:8.2f}, {v:8.2f})')
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
