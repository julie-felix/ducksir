#!/usr/bin/env python3
"""builder 与底层 `ir.Plan` 必须产出**同一个 blob**。

    PYTHONPATH=$HOME/.local/share/ducksir/pylibs python3 sim/test/test_graph_equiv.py

这是 Q16 的验收条件：高层只是写法，不是另一套语义。若这里红了，说明 builder 的折叠顺序、
参数填充、clip 处理里有一处和 `run_policy.build_plan()` 不同——那种差异在运行时只会表现为
「行为微妙不同」，所以必须在这里钉死。
"""

from __future__ import annotations

import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
sys.path.insert(0, os.path.join(ROOT, "sim", "plan"))
sys.path.insert(0, os.path.join(ROOT, "sim", "host"))
sys.path.insert(0, os.path.join(ROOT, "tools", "robot"))

import yaml

import entity as E
import graph as G
import ir
import run_policy as RP


def main() -> int:
    with open(os.path.join(ROOT, "build/preset/duck-hd1910.preset.yaml"), encoding="utf-8") as f:
        preset = yaml.safe_load(f)
    with open(os.path.join(ROOT, "contract/policy/duck-walk.manifest.yaml"), encoding="utf-8") as f:
        man = yaml.safe_load(f)
    doc = E.read_yaml(os.path.join(ROOT, preset["generated"]["from"]))
    driven = [j["name"] for j in doc["joints"] if j.get("actuated")]
    acts = doc["actuators"]
    lo = [float(a["ctrlrange"][0]) for a in acts]
    hi = [float(a["ctrlrange"][1]) for a in acts]

    old = ir.compile_blob(RP.build_plan(man, preset, len(driven), lo, hi))
    new = G.Graph.standard(man, preset, lo, hi).build()
    old_info, new_info = ir.read_blob(old), ir.read_blob(new)

    ok = True
    same = old == new
    print(f"  {'PASS' if same else 'FAIL'}  blob 完全相同（{len(old)} B，{len(old_info['nodes'])} 节点）")
    ok &= same
    if not same:
        print(f"        旧 {ir.sha256_of_blob(old)[:16]} / 新 {ir.sha256_of_blob(new)[:16]}")
        print(f"        维度 {old_info['input_dim']}→{old_info['output_dim']} vs "
              f"{new_info['input_dim']}→{new_info['output_dim']}")
        for i, (a, b) in enumerate(zip(old_info["nodes"], new_info["nodes"])):
            if a != b:
                print(f"        首个不同节点 #{i}：{a} vs {b}")
                break

    # 另：缺步骤要报错，而不是产出半张图
    for label, fn in [("没 observe 就 policy", lambda: G.Graph(man, preset).policy("x")),
                      ("没 policy 就 action", lambda: G.Graph(man, preset).observe_all().action())]:
        try:
            fn()
            print(f"  FAIL  {label} 应该报错")
            ok = False
        except G.GraphError as e:
            print(f"  PASS  {label} → GraphError：{str(e)[:40]}")

    print()
    print("全部通过" if ok else "失败")
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
