#!/usr/bin/env python3
"""导出 MuJoCo 的 (θA, θB) → (roll, pitch) 查表，给查看器做并排对比。

    PYTHONPATH=/tmp/pylibs python3 tools/parallel/export_mujoco_grid.py \
        contract/robot/rigs/ankle-rig-g1.yaml /tmp/rig_g1_nomesh.xml --grid 41 -o ...

rig 的 XML 里执行器是 **position 伺服**（讲者把 motor 改成了 position），
所以直接把目标角写进 ctrl、跑几百步等它稳住，读 roll/pitch 就行；不需要自己写 PD。
"""

from __future__ import annotations

import argparse
import json
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, "..", "robot"))

import solver as S  # noqa: E402
import entity as E  # noqa: E402


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("entity")
    ap.add_argument("xml")
    ap.add_argument("-o", "--out", required=True)
    ap.add_argument("--grid", type=int, default=41)
    ap.add_argument("--span", type=float, default=0.5)
    ap.add_argument("--steps", type=int, default=600)
    args = ap.parse_args()

    import mujoco

    doc = E.read_yaml(args.entity)
    sol = S.ParallelAnkle.from_entity(doc)
    m = mujoco.MjModel.from_xml_path(args.xml)
    d = mujoco.MjData(m)
    jid = lambda n: mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, n)  # noqa: E731
    adr = {n: m.jnt_qposadr[jid(n)] for n in [sol.roll_joint, sol.pitch_joint] + sol.motors}
    act_of = {mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_JOINT, m.actuator_trnid[a][0]): a
              for a in range(m.nu)}
    ctrl = [act_of[j] for j in sol.motors]        # 执行器顺序 ≠ 关节顺序，必须按名字对

    n = args.grid
    ths = np.linspace(-args.span, args.span, n)
    table, worst = [], 0.0
    for t1 in ths:
        row = []
        for t2 in ths:
            d.qpos[:] = 0
            d.qvel[:] = 0
            mujoco.mj_forward(m, d)
            d.ctrl[ctrl[0]] = t1
            d.ctrl[ctrl[1]] = t2
            for _ in range(args.steps):
                mujoco.mj_step(m, d)
            roll, pitch = d.qpos[adr[sol.roll_joint]], d.qpos[adr[sol.pitch_joint]]
            row.append([round(float(roll), 6), round(float(pitch), 6)])
            r, p, ok, _ = sol.fw(np.array([t1, t2]))
            if ok:
                worst = max(worst, abs(roll - r), abs(pitch - p))
        table.append(row)
        print(f"  {len(table)}/{n}", end="\r")

    out = {"entity": doc["robot"]["id"], "span": args.span, "n": n,
           "motors": sol.motors, "roll_joint": sol.roll_joint, "pitch_joint": sol.pitch_joint,
           "grid": table}
    os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
    with open(args.out, "w", encoding="utf-8") as f:
        json.dump(out, f)
    print(f"\n→ {args.out}  {n}×{n} · MuJoCo vs 我们 最大差 {worst*1e3:.3f} mrad")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
