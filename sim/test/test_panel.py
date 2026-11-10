#!/usr/bin/env python3
"""面板 schema 守卫：8 个键必须在、必须有限、必须和面板页的标签表对得上。

    PYTHONPATH=$HOME/.local/share/ducksir/pylibs python3 sim/test/test_panel.py
"""

from __future__ import annotations

import json
import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
sys.path.insert(0, os.path.join(ROOT, "sim", "host"))

import panel as P  # noqa: E402

fails = 0


def check(label, cond, detail=""):
    global fails
    print(f"  {'PASS' if cond else 'FAIL'}  {label}{'  ' + str(detail) if detail else ''}")
    if not cond:
        fails += 1


print("== schema")
p = P.build_panel(control_hz=50, physics_hz=200, seconds=10, wall=2.0,
                  step_times=[0.001, 0.03, 0.002], base_height_min=0.11,
                  upright_final=0.99, distance=1.2)
check("8 个键齐全", set(p) == set(P.PANEL_KEYS), f"缺 {set(P.PANEL_KEYS) - set(p)}")
check("check_panel 通过", P.check_panel(p) == [])
check("超期步数算出 1（0.03s > 1/50）", p["deadline_miss"] == 1, f"实际 {p['deadline_miss']}")
check("p95 取到 30 ms", p["control_p95_ms"] == 30.0, f"实际 {p['control_p95_ms']}")
check("实时倍率 5×", p["realtime_factor"] == 5.0, f"实际 {p['realtime_factor']}")

print("== 缺键 / 非数要报错")
bad = dict(p)
bad.pop("distance")
check("缺键能检出", P.check_panel(bad) != [], P.check_panel(bad)[:1])
bad2 = dict(p)
bad2["upright_final"] = float("nan")
check("NaN 能检出", P.check_panel(bad2) != [], P.check_panel(bad2)[:1])

print("== 页面标签表与 schema 对齐")
meta_path = os.path.join(ROOT, "client/viewer/panel-meta.mjs")
src = open(meta_path, encoding="utf-8").read()
keys = set(re.findall(r"^\s{2}([a-z0-9_]+): \{", src, re.M))
check("panel-meta.mjs 的键 = PANEL_KEYS", keys == set(P.PANEL_KEYS),
      f"页面多 {keys - set(P.PANEL_KEYS)}，页面少 {set(P.PANEL_KEYS) - keys}")

print("== 样例报告（若存在）")
sample = os.path.join(ROOT, "client/viewer/data/live/report.json")
if os.path.exists(sample):
    rep = json.load(open(sample, encoding="utf-8"))
    check("样例报告带 panel 且合法", P.check_panel(rep.get("panel", {})) == [],
          str(P.check_panel(rep.get("panel", {}))[:1]))
else:
    print("  SKIP  没找到样例报告（跑 bin/ducksir run --report ...）")

print()
print("全部通过" if fails == 0 else f"失败 {fails} 项")
sys.exit(0 if fails == 0 else 1)
