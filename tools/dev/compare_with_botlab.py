#!/usr/bin/env python3
"""把我们和 botlab 的 run 逐段对拍，输出一张对齐报告。

两边的 trace 都是「每控制步一行 JSONL」：

    # 我们这边
    python3 sim/host/run_bundle.py --bundle build/release/g1-wbc \\
        --mjcf build/release/g1-wbc/robot.xml --steps 120 --trace /tmp/ours.jsonl

    # botlab 那边（他们自己的运行时；见 tools/dev/botlab_record_native.py 的用法）
    DUCK_CAP=/tmp/theirs.jsonl ... python3 -c "import patch_trace; from botlab._cli import main; ..."

    python3 tools/dev/compare_with_botlab.py /tmp/ours.jsonl /tmp/theirs.jsonl

比对四件事（由粗到细）：

1. **动作参考 67 维**：直接用同一帧动作 + 同一 base 状态复算，和他们的 obs 前 67 维比；
   这一项对齐说明「参考公式」一致（当前 **4.7e-07**）。
2. **策略**：把**他们录的 439 维 obs** 喂给我们的 ONNX，比动作输出；
   这一项对齐说明「观测布局 + 模型调用」一致（当前 **9.4e-06**）。
3. **执行器**：在他们记录的状态上按我们的力矩公式算，和他们写进 ctrl 的值比（当前 ~0.5 N·m）。
4. **物理轨迹**：用他们的动作**开环重放**，比 qpos（当前早期 0.1 rad，120 步后 ~0.9 rad——
   还差在求解器/子步细节，未定位）。
"""
import argparse
import json
import os
import sys

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(ROOT, "sim", "plan"))
sys.path.insert(0, os.path.join(ROOT, "sim", "host"))


def load_trace(path):
    return [json.loads(line) for line in open(path, encoding="utf-8") if line.strip()]


def seg_report(ours, theirs, step, label=""):
    o = np.asarray(ours["obs"], dtype=float)
    t = np.asarray(theirs["in"]["obs"], dtype=float)
    segs = [("参考 67", 0, 67), ("角速度", 67, 70), ("重力", 70, 73), ("q", 73, 102),
            ("qd", 102, 131), ("last", 131, 160), ("历史 1", 160, 253),
            ("历史 2", 253, 346), ("历史 3(最新)", 346, 439)]
    print(f"  第 {step} 步 obs 分段最大差{label}:")
    for name, a, b in segs:
        if b <= len(o) and b <= len(t):
            print(f"    {name:12s} {np.abs(o[a:b] - t[a:b]).max():.4f}")


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("ours")
    ap.add_argument("theirs")
    ap.add_argument("--bundle", default="build/release/g1-wbc",
                    help="bundle 目录（比策略与执行器时用它的模型/preset）")
    ap.add_argument("--motion", default=None, help="动作目录（默认 bundle/motion）")
    args = ap.parse_args()

    ours = load_trace(args.ours)
    theirs = load_trace(args.theirs)
    print(f"我们的步数 {len(ours)}｜他们的步数 {len(theirs)}\n")

    # 1) 参考段：用同一帧动作复算
    import motion as M
    import wbc_obs as W
    motion_dir = args.motion or os.path.join(ROOT, args.bundle, "motion")
    meta = json.load(open(os.path.join(motion_dir, "metadata.json"), encoding="utf-8"))
    names = meta["body_names"]
    jp = M.load_motion(motion_dir, "joint_pos")["data"]
    jv = M.load_motion(motion_dir, "joint_vel")["data"]
    bp = M.load_motion(motion_dir, "body_pos_w", indices=names)["data"].reshape(-1, len(names), 3)
    bq = M.load_motion(motion_dir, "body_quat_w", indices=names)["data"].reshape(-1, len(names), 4)
    print("1) 动作参考 67 维（idx=步+1、回看 5 帧、botlab 引擎公式）")
    worst = 0.0
    for k in (3, 10, 30, 60, 100):
        if k + 1 >= len(theirs) or not theirs[k].get("qpos"):
            continue
        idx, last = k + 1, max(1, k + 1 - 5)
        mine = W.reference_block(joint_pos=jp[idx], joint_vel=jv[idx],
                                 anchor_pos=bp[idx][0], anchor_quat=bq[idx][0],
                                 last_anchor_pos=bp[last][0], last_anchor_quat=bq[last][0])
        d = float(np.abs(mine - np.asarray(theirs[k]["in"]["obs"][:67])).max())
        worst = max(worst, d)
        print(f"    第 {k:3d} 步  max|Δ| = {d:.2e}")
    print(f"  → 参考段最大差 {worst:.2e}\n")

    # 2) 策略：他们的 obs 喂我们的模型
    import onnxruntime as ort
    contract = json.load(open(os.path.join(ROOT, args.bundle, "contract.json"), encoding="utf-8"))
    model_path = os.path.join(ROOT, args.bundle, contract["policies"][0]["file"])
    sess = ort.InferenceSession(model_path, providers=["CPUExecutionProvider"])
    name = sess.get_inputs()[0].name
    diffs = []
    for k in range(1, len(theirs)):
        r = theirs[k]
        obs = np.asarray(r["in"]["obs"], dtype=np.float32).reshape(1, -1)
        act = np.asarray(sess.run(None, {name: obs})[0]).reshape(-1)
        diffs.append(float(np.abs(act - np.asarray(r["out"]["actions"])).max()))
    print("2) 策略：用他们录的 obs 喂我们的模型")
    if diffs:
        print(f"    {len(diffs)} 步，最大差 {max(diffs):.2e}，平均 {float(np.mean(diffs)):.2e}\n")

    # 3) 执行器：他们的状态 + 我们的力矩公式 vs 他们写进 ctrl 的值
    import yaml
    preset = yaml.safe_load(open(os.path.join(ROOT, args.bundle, "preset.yaml"), encoding="utf-8"))
    MAP = preset["motorJointOrder"]
    REF = np.asarray(preset["referenceJointPositions"])
    print("3) 执行器：同一状态下我们的力矩 vs 他们的 ctrl")
    for k in (5, 20, 60):
        if k >= len(theirs) or not theirs[k].get("qpos") or not theirs[k].get("ctrl_prev"):
            continue
        r = theirs[k]
        q = np.asarray(r["qpos"][7:36])
        qv = np.asarray(r["qvel"][6:35])
        act = np.asarray(theirs[k - 1]["out"]["actions"])
        target = REF.copy()
        for i in range(29):
            target[MAP[i]] = REF[MAP[i]] + 0.25 * act[i]
        tau = np.clip(np.asarray(preset["kp"]) * (target - q) - np.asarray(preset["kd"]) * qv,
                      -np.asarray(preset["effortLimit"]), np.asarray(preset["effortLimit"]))
        their = np.asarray(r["ctrl_prev"])
        print(f"    第 {k:3d} 步  max|Δ| = {np.abs(tau - their).max():.2f} N·m"
              f"｜相关 {np.corrcoef(tau, their)[0, 1]:.3f}")
    print()

    # 4) 总览：obs / 动作 / qpos（注意两边记录时刻略有相位差）
    print("4) 逐步总览（obs439 / action / qpos）")
    print("    步 |  obs439  |   action  |   qpos   | 他们 z | 我们 z")
    for k in (1, 5, 20, 60, 100, min(len(ours), len(theirs)) - 1):
        if k >= len(ours) or k >= len(theirs) or not theirs[k].get("qpos"):
            continue
        o, t = ours[k], theirs[k]
        d_obs = float(np.abs(np.asarray(o["obs"]) - np.asarray(t["in"]["obs"])).max())
        d_act = float(np.abs(np.asarray(o["action"]) - np.asarray(t["out"]["actions"])).max())
        d_q = float(np.abs(np.asarray(o["qpos"]) - np.asarray(t["qpos"])).max())
        print(f"    {k:3d} | {d_obs:8.4f} | {d_act:9.4f} | {d_q:8.4f} | "
              f"{t['qpos'][2]:.3f} | {o['qpos'][2]:.3f}")
    if len(ours) > 2:
        seg_report(ours[min(20, len(ours) - 1)], theirs[min(20, len(theirs) - 1)], min(20, len(ours) - 1))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
