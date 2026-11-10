#!/usr/bin/env python3
"""导出「一个可运行的策略包」：plan blob + 模型 + **契约**（contract.json）。

    PYTHONPATH=$HOME/.local/share/ducksir/pylibs python3 sim/host/export_artifacts.py \
        --manifest contract/policy/duck-walk.manifest.yaml --out client/viewer/data/live

为什么要有这个脚本（2026-11-10 的决定）：**契约是产物，不是消费端的常量**。
以前观测偏移、`fsm` 输出的 `[state, mode, command(13), clear]` 这些形状写在浏览器 JS 里，
改 Python 生成新 IR 之后端侧用旧偏移读新 blob——不报错，只是行为诡异。
现在编译时把这些形状写进 `contract.json`，端侧按名字取槽位、加载时校验维度与 digest。

产物（都不入库，除 contract.json）：
- `pre.blob` / `post.blob`：推理边界两段（ORT Web 是 async，核是 sync）
- `fsm.blob`：状态机（推理边界之前，整块在核里跑）
- `<policy>.onnx`：各策略权重
- `contract.json`：**唯一真源**——plan 形状 + 具名槽位 + 策略清单 + 观测配方 + 频率 + 机器人元数据
"""

from __future__ import annotations

import argparse
import json
import os
import shutil
import sys

import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
sys.path.insert(0, os.path.join(ROOT, "sim", "plan"))
sys.path.insert(0, os.path.join(ROOT, "tools", "robot"))
sys.path.insert(0, HERE)

import contract  # noqa: E402
import entity as E  # noqa: E402
import export_browser_fsm as FSM  # noqa: E402
import ir  # noqa: E402
import run_policy as RP  # noqa: E402

# 策略清单：mode 与 botlab `_policy_mode()` 一致
POLICIES = [
    ("walking", 0), ("sitstand", 1), ("kick-left", 2), ("kick-right", 3),
    ("roll", 4), ("ground-pick", 5), ("stand", 6),
]

# 观测配方：槽位 + 从哪取（消费端照这个取数，不再写死偏移）
OBS_RECIPE = [
    {"name": "imu_ang_vel", "slot": [0, 3], "source": "sensor:imu_ang_vel"},
    {"name": "projected_gravity", "slot": [3, 3], "source": "gravity:body:trunk_base"},
    {"name": "joint_pos", "slot": [6, 14], "source": "qpos:joints", "minus": "default"},
    {"name": "joint_vel", "slot": [20, 14], "source": "qvel:joints"},
    {"name": "last_action", "slot": [34, 14], "source": "last_action"},
    {"name": "command", "slot": [48, 13], "source": "fsm_slot:command"},
]


def build_pre(man: dict) -> tuple[bytes, dict]:
    total = sum(o["len"] for o in man["obs"])
    p = ir.Plan(input_dim=total, output_dim=total, name="pre")
    sl, off = [], 0
    for o in man["obs"]:
        sl.append(p.obs(off, o["len"]))
        off += o["len"]
    level = list(sl)
    while len(level) > 4:
        nxt = []
        for i in range(0, len(level), 4):
            chunk = level[i:i + 4]
            nxt.append(chunk[0] if len(chunk) == 1 else p.concat(*chunk))
        level = nxt
    p.out_action(level[0] if len(level) == 1 else p.concat(*level))
    return ir.compile_blob(p.to_json()), {"in:obs": (0, total)}


def build_post(preset: dict, n: int, lo, hi) -> tuple[bytes, dict]:
    p = ir.Plan(input_dim=n, output_dim=n, name="post")
    raw = p.obs(0, n)
    s = p.affine(raw, preset["actionScale"], preset["defaultJointPositions"])
    p.out_action(p.clip(s, lo, hi))
    return ir.compile_blob(p.to_json()), {"in:action": (0, n), "out:ctrl": (0, n)}


def build_fsm() -> tuple[bytes, dict]:
    doc = FSM.build()
    blob = ir.compile_blob(doc)
    slots = {
        "in:policy_command": (0, 1), "in:gravity_z": (1, 1),
        "in:vx": (2, 1), "in:wz": (3, 1),
        "out:state": (0, 1), "out:mode": (1, 1),
        "out:command": (2, 13), "out:clear_last_action": (15, 1),
    }
    return blob, slots


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--manifest", default="contract/policy/duck-walk.manifest.yaml")
    ap.add_argument("--preset", default="build/preset/duck-hd1910.preset.yaml")
    ap.add_argument("--out", default="client/viewer/data/live")
    args = ap.parse_args()

    with open(os.path.join(ROOT, args.preset), encoding="utf-8") as f:
        preset = yaml.safe_load(f)
    with open(os.path.join(ROOT, args.manifest), encoding="utf-8") as f:
        man = yaml.safe_load(f)
    doc = E.read_yaml(os.path.join(ROOT, preset["generated"]["from"]))
    driven = [j["name"] for j in doc["joints"] if j.get("actuated")]
    n = len(driven)
    RP.check_manifest(man, preset, n)
    actuators = doc.get("actuators") or []
    lo = [float(a["ctrlrange"][0]) for a in actuators]
    hi = [float(a["ctrlrange"][1]) for a in actuators]

    out = os.path.join(ROOT, args.out)
    os.makedirs(out, exist_ok=True)

    pre_blob, pre_slots = build_pre(man)
    post_blob, post_slots = build_post(preset, n, lo, hi)
    fsm_blob, fsm_slots = build_fsm()
    for name, blob in (("pre", pre_blob), ("post", post_blob), ("fsm", fsm_blob)):
        with open(os.path.join(out, f"{name}.blob"), "wb") as f:
            f.write(blob)

    policies = []
    for fname, mode in POLICIES:
        src = os.path.join(ROOT, "build/models", f"{fname}.onnx")
        if not os.path.exists(src):
            print(f"  跳过 {fname}（缺 build/models/{fname}.onnx）")
            continue
        shutil.copyfile(src, os.path.join(out, f"{fname}.onnx"))
        policies.append({"key": fname, "mode": mode, "file": f"{fname}.onnx"})

    doc_contract = contract.build_contract(
        plans={
            "pre": contract.plan_entry("pre", pre_blob, pre_slots),
            "post": contract.plan_entry("post", post_blob, post_slots),
            "fsm": contract.plan_entry("fsm", fsm_blob, fsm_slots),
        },
        policies=policies,
        obs=OBS_RECIPE,
        rates={"control_hz": 50, "physics_hz": 200, "inference_decimation": 1},
        states={"count": len(FSM.STATE_NAMES) if hasattr(FSM, "STATE_NAMES") else 14,
                "initial": 0},
        robot={
            "id": doc["robot"]["id"],
            "joints": driven,
            "default_q": preset["defaultJointPositions"],
            "initial_qpos": preset["initialQpos"],
            "action_scale": preset["actionScale"],
            "ctrlrange": [[lo[i], hi[i]] for i in range(n)],
            "commands": {"forward_max": 0.25, "back_max": -0.2, "yaw_max": 1.0},
        },
    )
    with open(os.path.join(out, "contract.json"), "w", encoding="utf-8") as f:
        json.dump(doc_contract, f, ensure_ascii=False, indent=2)

    print(f"导出 → {args.out}")
    for name, p in doc_contract["plans"].items():
        print(f"  {name:<5} {p['nodes']:>3} 节点 {p['input_dim']:>3} → {p['output_dim']:>3}"
              f"｜{p['bytes']} B｜digest {p['digest'][:12]}")
    print(f"  策略 {len(policies)} 个｜契约 contract.json")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
