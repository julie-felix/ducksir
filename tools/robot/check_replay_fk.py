#!/usr/bin/env python3
"""校验「查看器 FK」与 MuJoCo 真实位姿一致 —— 回放看到的到底是不是物理里那个姿态。

    PYTHONPATH=$HOME/.local/share/ducksir/pylibs \
    python3 tools/robot/check_replay_fk.py \
        client/viewer/data/duck-hd1910.json \
        client/viewer/data/replay-duck-pd.json \
        build/mjcf/duck-hd1910.xml

为什么需要这个：回放只有「自由基座位姿 + 驱动关节角」，几何全靠页面自己的 FK。
如果 FK 或根节点的换算错一点，画面上就会「看着怪」——而肉眼判断不了是渲染错还是模型本来躺成那样。
这里用同一帧的 qpos 让 MuJoCo 算一遍 `xpos`，和页面那套 FK 逐个 body 比，用毫米说话。

判据：最大偏差 ≤ 0.1 mm。三帧（首/中/末）都过才 PASS。

已知边界：viewer 的 joints[] 只有 axis、没有关节原点，所以要求关节原点在 body 原点
（本仓导出的 MJCF 满足：`joint pos` 只在 axis.origin 非零时才写，鸭子/G1 都是零）。
没有 mujoco 就跳过（exit 0）。
"""

from __future__ import annotations

import argparse
import json
import sys

TOL_MM = 0.1


def qmat(qwxyz):
    import numpy as np

    w, x, y, z = qwxyz
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def axis_rot(axis, ang):
    import numpy as np

    a = np.array(axis, dtype=float)
    a = a / (np.linalg.norm(a) or 1.0)
    K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + np.sin(ang) * K + (1 - np.cos(ang)) * (K @ K)


def fk(data: dict, frame: dict, joint_names: list[str]) -> dict:
    """复刻 client/viewer/app.js 的 FK：根节点吃 qpos，子节点 = 父 × (xyz, quat) × 关节旋转。"""
    import numpy as np

    jby_child = {j["child"]: j for j in data["joints"]}
    names = {j["name"]: j for j in data["joints"]}
    root = data["root"]
    pos = {root: np.array(frame["pos"], dtype=float)}
    rot = {root: qmat(frame["quat"])}

    def walk(name: str) -> None:
        for link in data["links"]:
            if (link.get("parent") or "world") != name:
                continue
            Rp, pp = rot[name], pos[name]
            Rj = np.eye(3)
            j = jby_child.get(link["name"])
            if j is not None:
                ang = frame["q"][joint_names.index(names[j["name"]]["name"])]
                Rj = axis_rot(j["axis"], ang)
            pos[link["name"]] = pp + Rp @ np.array(link["xyz"], dtype=float)
            rot[link["name"]] = Rp @ qmat(link["quat"]) @ Rj
            walk(link["name"])

    walk(root)
    return pos


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("viewer_json")
    ap.add_argument("trajectory")
    ap.add_argument("mjcf")
    args = ap.parse_args()

    try:
        import mujoco
        import numpy as np
    except ImportError:
        print("跳过：没装 mujoco / numpy（见 docs/env.md）")
        return 0

    data = json.load(open(args.viewer_json, encoding="utf-8"))
    traj = json.load(open(args.trajectory, encoding="utf-8"))
    frames = traj["frames"]
    names = traj["meta"]["joints"]
    m = mujoco.MjModel.from_xml_path(args.mjcf)
    d = mujoco.MjData(m)

    picks = [0, len(frames) // 2, len(frames) - 1]
    worst = 0.0
    for i in picks:
        fr = frames[i]
        d.qpos[:] = 0.0
        d.qpos[0:3] = fr["pos"]
        d.qpos[3:7] = fr["quat"]
        for k, nm in enumerate(names):
            jid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, nm)
            if jid < 0:
                print(f"FAIL 轨迹里有 MJCF 没有的关节：{nm}")
                return 1
            d.qpos[int(m.jnt_qposadr[jid])] = fr["q"][k]
        mujoco.mj_forward(m, d)

        P = fk(data, fr, names)
        errs = []
        for bi in range(m.nbody):
            bn = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, bi)
            if bn in P:
                errs.append(float(np.linalg.norm(np.array(d.xpos[bi]) - P[bn])))
        e = max(errs) if errs else 0.0
        worst = max(worst, e)
        print(f"t={fr['t']:6.2f}s  比对 {len(errs)} 个 body，最大偏差 {e * 1000:.3f} mm")

    ok = worst * 1000 <= TOL_MM
    print(f"\n{'PASS' if ok else 'FAIL'}  最大偏差 {worst * 1000:.3f} mm（阈 {TOL_MM} mm）")
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
