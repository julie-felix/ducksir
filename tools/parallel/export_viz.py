#!/usr/bin/env python3
"""导出「解算 vs MuJoCo」的逐帧数据，给 client/viewer/parallel.html 播动画。

    PYTHONPATH=/tmp/pylibs python3 tools/parallel/export_viz.py \
        contract/robot/types/ankle-rig-tg10.yaml /tmp/rig_tg10_nomesh.xml \
        -o client/viewer/data/parallel-tg10.json

每帧给两边各自算一套几何：
- ours：我们用闭式逆解/正解算出来的 B(推杆销) 与 C(平台点)
- mj  ：MuJoCo 实际报告出来的 B(电机座锚点世界坐标) 与 C(平台 link 原点)
两边杆长之差就是软约束的缝，动画里能看见。
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


def _pose_of(M):
    return {"R": [round(float(x), 5) for x in M[:3, :3].reshape(9)],
            "p": [round(float(x), 5) for x in M[:3, 3]]}


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("entity")
    ap.add_argument("xml")
    ap.add_argument("-o", "--out", required=True)
    ap.add_argument("--frames", type=int, default=400)
    ap.add_argument("--amp", type=float, default=0.35)
    ap.add_argument("--freq", type=float, default=0.5)
    ap.add_argument("--dt", type=float, default=1e-3)
    args = ap.parse_args()

    import mujoco

    doc = E.read_yaml(args.entity)
    sol = S.ParallelAnkle.from_entity(doc)

    cwd = os.getcwd()
    os.chdir(os.path.dirname(os.path.abspath(args.xml)))
    try:
        m = mujoco.MjModel.from_xml_path(os.path.basename(args.xml))
    finally:
        os.chdir(cwd)
    d = mujoco.MjData(m)

    jid = lambda n: mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, n)  # noqa: E731
    qadr = {n: m.jnt_qposadr[jid(n)] for n in [sol.roll_joint, sol.pitch_joint] + sol.motors}
    platform_link = sol.joints[sol.roll_joint]["child"]
    platform_bid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, platform_link)
    body = {lp["body1"]: mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, lp["body1"])
            for lp in doc["loops"] if lp["kind"] == "connect"}
    body2 = {lp["body2"]: mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, lp["body2"])
             for lp in doc["loops"] if lp["kind"] == "connect"}
    anchors = [lp["anchor"] for lp in doc["loops"] if lp["kind"] == "connect"]

    every = max(1, int(round(1.0 / (args.freq * args.dt) / args.frames)))
    samples = []
    step = 0
    while len(samples) < args.frames:
        u = args.amp * np.sin(2 * np.pi * args.freq * step * args.dt)
        d.ctrl[0] = u
        d.ctrl[1] = -0.6 * u
        mujoco.mj_step(m, d)
        step += 1
        if step % every:
            continue
        th = np.array([d.qpos[qadr[n]] for n in sol.motors])
        roll_mj, pitch_mj = d.qpos[qadr[sol.roll_joint]], d.qpos[qadr[sol.pitch_joint]]
        r, p, ok, _ = sol.fw(th, max_iter=80)
        c_ours = sol.platform_point(r, p) if ok else [np.zeros(3), np.zeros(3)]
        b_ours = [sol.pin_point(i, th[i]) for i in range(2)]
        b_mj, c_mj = [], []
        for k, lp in enumerate([x for x in doc["loops"] if x["kind"] == "connect"]):
            bid = body[lp["body1"]]
            p1 = d.xpos[bid] + d.xmat[bid].reshape(3, 3) @ np.array(anchors[k], float)
            b_mj.append(p1.tolist())
            c_mj.append(d.xpos[body2[lp["body2"]]].tolist())
        samples.append({
            "t": round(step * args.dt, 4),
            "th": [round(float(x), 6) for x in th],
            "ours": {"roll": round(float(r), 6), "pitch": round(float(p), 6), "ok": bool(ok),
                     "pose": _pose_of(sol._pose(platform_link, {sol.roll_joint: r, sol.pitch_joint: p})),
                     "B": [list(map(float, np.round(b, 5))) for b in b_ours],
                     "C": [list(map(float, np.round(c, 5))) for c in c_ours],
                     "rod_err": [round(float(sol.rod_error(th[i], i, r, p)), 9) for i in range(2)]},
            "mj": {"roll": round(float(roll_mj), 6), "pitch": round(float(pitch_mj), 6),
                   "pose": {"R": [round(float(x), 5) for x in d.xmat[platform_bid].reshape(9)],
                            "p": [round(float(x), 5) for x in d.xpos[platform_bid]]},
                   "B": [list(map(float, np.round(b, 5))) for b in b_mj],
                   "C": [list(map(float, np.round(c, 5))) for c in c_mj],
                   "rod_err": [round(float(sol.rod_error(th[i], i, roll_mj, pitch_mj)), 9) for i in range(2)]},
        })

    out = {
        "robot": doc["robot"]["id"],
        "xml": os.path.basename(args.xml),
        "chains": [{
            "A": list(map(float, np.round(c["pivot"], 5))),
            "axis": list(map(float, np.round(c["axis"], 5))),
            "crank": round(float(np.linalg.norm(c["v"])), 5),
            "rod": round(float(c["rod"]), 5),
            "c0": list(map(float, np.round(c["c0"], 5))),
        } for c in sol.chains],
        "samples": samples,
    }
    os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
    with open(args.out, "w", encoding="utf-8") as f:
        json.dump(out, f, ensure_ascii=False)
    err = max(max(abs(s["ours"]["roll"] - s["mj"]["roll"]), abs(s["ours"]["pitch"] - s["mj"]["pitch"]))
              for s in samples if s["ours"]["ok"])
    gap = max(max(abs(e) for e in s["mj"]["rod_err"]) for s in samples)
    print(f"{out['robot']}: {len(samples)} 帧 → {args.out}")
    print(f"  roll/pitch 最大差 {err:.3e} rad · MuJoCo 杆长违反最大 {gap*1e6:.1f} µm")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
