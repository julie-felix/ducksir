#!/usr/bin/env python3
"""一次性推导工具：把 rig 的连杆几何算成「踝 pitch 系」里的参数。

**产物已经写进** `contract/robot/specs/g1-parallel.yaml` 的 `mechanisms[].coupling.params`，
查看器现在直接读实体，不再读这里导出的 JSON。保留这个脚本是为了说明那组数字怎么来的、以及能重算。

导出「并联踝叠加层」数据：几何（挂在 G1 踝 pitch link 坐标系里）+ MuJoCo 查表。

    PYTHONPATH=/tmp/pylibs python3 tools/parallel/export_overlay.py \
        contract/robot/types/ankle-rig-g1.yaml /tmp/rig_g1_nomesh.xml \
        -o client/viewer/data/parallel-g1-overlay.json --grid 41

坐标系约定：rig 的 `pitch_link` 原点 = 踝的 pitch 关节中心（rig 里它在 z=0.15）。
所有点都减去这个原点，于是文件里的坐标可以直接加进 G1 的 `left_ankle_pitch_link` 局部坐标系。
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
    ap.add_argument("--span", type=float, default=0.6, help="电机角扫的范围 ±span（rad）")
    args = ap.parse_args()

    import mujoco

    doc = E.read_yaml(args.entity)
    sol = S.ParallelAnkle.from_entity(doc)

    # 踝 pitch link 的世界原点 = pitch 关节位置
    pj = sol.joints[sol.pitch_joint]
    origin = sol._pose(pj["child"], {})[:3, 3].copy()
    rj = sol.joints[sol.roll_joint]
    roll_origin = sol._pose(rj["child"], {})[:3, 3].copy()

    def rel(p):
        return [round(float(x), 6) for x in (np.asarray(p, float) - origin)]

    out = {
        "rig": doc["robot"]["id"],
        "frame": "ankle_pitch",                # 叠加到 ankle_pitch 关节（不是 body），原点=关节中心
        "frame_origin_world": [round(float(x), 5) for x in origin],
        "chains": [{
            "A": rel(ch["pivot"]),
            "axis": [round(float(x), 6) for x in ch["axis"]],
            "v": [round(float(x), 6) for x in ch["v"]],     # 销相对转轴的点（随 θ 绕轴转）
            "rod": round(float(ch["rod"]), 6),
        } for ch in sol.chains],
        "platform": {
            "pitch_axis": [round(float(x), 6) for x in pj["axis"]["dir"]],
            "pitch_pivot": rel(origin),
            "roll_axis": [round(float(x), 6) for x in rj["axis"]["dir"]],
            "roll_pivot": rel(roll_origin),
            "c0": [rel(ch["c0"]) for ch in sol.chains],
        },
    }

    # MuJoCo 查表：把两个电机角直接写进 qpos，让它把约束解稳，读 pitch/roll
    cwd = os.getcwd()
    os.chdir(os.path.dirname(os.path.abspath(args.xml)))
    try:
        m = mujoco.MjModel.from_xml_path(os.path.basename(args.xml))
    finally:
        os.chdir(cwd)
    d = mujoco.MjData(m)
    jid = lambda n: mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, n)  # noqa: E731
    adr = {n: m.jnt_qposadr[jid(n)] for n in [sol.roll_joint, sol.pitch_joint] + sol.motors}

    # 查表：用 PD 把两个电机「控」到目标角，等它稳下来再读 pitch/roll。
    # 直接把电机角写进 qpos 会违反动力学（等于瞬移），MuJoCo 会自己滑到别的装配支去。
    n = args.grid
    ths = np.linspace(-args.span, args.span, n)
    kp, kd = 40.0, 2.0
    motor_dof = [m.jnt_dofadr[jid(t)] for t in sol.motors]
    table = []
    for t1 in ths:
        row = []
        for t2 in ths:
            d.qpos[:] = 0
            d.qvel[:] = 0
            mujoco.mj_forward(m, d)
            tail = []
            for k in range(1500):
                for i, target in enumerate((t1, t2)):
                    err = target - d.qpos[adr[sol.motors[i]]]
                    d.ctrl[i] = kp * err - kd * d.qvel[motor_dof[i]]
                mujoco.mj_step(m, d)
                if k > 1200:
                    tail.append((d.qpos[adr[sol.roll_joint]], d.qpos[adr[sol.pitch_joint]]))
            arr = np.array(tail)
            row.append([round(float(arr[:, 0].mean()), 6), round(float(arr[:, 1].mean()), 6)])
        table.append(row)
        if (len(table) % 10) == 0:
            print(f"  grid {len(table)}/{n}")
    out["grid"] = {"span": args.span, "n": n, "mj_roll_pitch": table}

    os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
    with open(args.out, "w", encoding="utf-8") as f:
        json.dump(out, f, ensure_ascii=False)
    print(f"→ {args.out}  ({len(out['chains'])} 条链, 查表 {n}×{n})")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
