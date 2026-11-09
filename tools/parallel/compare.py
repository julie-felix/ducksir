#!/usr/bin/env python3
"""把解算器和 MuJoCo 对拍：给两个电机同一条正弦，比 pitch/roll 曲线。

    PYTHONPATH=/tmp/pylibs python3 tools/parallel/compare.py \
        contract/robot/types/ankle-rig-tg10.yaml \
        refs/parallel-ankle/5_tiangong/urdf_TG10-00.xml

做法（与笔记里的对拍同构）：
1. MuJoCo 里按 ctrl 给两个电机正弦激励，跑 N 步，记 qpos 和等式约束残差 efc_pos；
2. 从 qpos 取电机角，喂给我们自己的 fw，得到 pitch/roll；
3. 和 MuJoCo 的 pitch/roll 逐点比，出 max/RMS 误差。

判据（docs/plans/2026-11-09-parallel-mechanism.md Q9）：
- pitch/roll 最大误差 ≤ 1e-4 rad
- 杆长守恒 ≤ 1e-9 m（纯代数，压不到浮点级就是推导错）
- 雅可比 vs 中心差分 ≤ 1e-6
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


def run_mujoco(xml, steps, amp, freq, dt, sol=None):
    import mujoco
    cwd = os.getcwd()
    os.chdir(os.path.dirname(os.path.abspath(xml)))  # meshdir 是相对路径，必须切过去
    try:
        m = mujoco.MjModel.from_xml_path(os.path.basename(xml))
    finally:
        os.chdir(cwd)
    d = mujoco.MjData(m)
    # 找电机执行器（按名字里的 motor）
    acts = [(mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_ACTUATOR, i) or "", i) for i in range(m.nu)]
    motors = [i for n, i in acts if "motor" in n]
    if len(motors) != 2:
        raise SystemExit(f"期望 2 个电机执行器，实际 {acts}")
    rows = []
    gaps = []
    for k in range(steps):
        u = amp * np.sin(2 * np.pi * freq * k * dt)
        d.ctrl[motors[0]] = u
        d.ctrl[motors[1]] = -0.5 * u
        mujoco.mj_step(m, d)
        row = d.qpos.copy()
        rows.append(row)
        if sol is not None:
            ap = lambda n: row[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, n) * 0 + m.jnt_qposadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, n)]]  # noqa: E731
            th = np.array([ap(n) for n in sol.motors])
            roll = ap(sol.roll_joint); pitch = ap(sol.pitch_joint)
            gaps.append(max(abs(sol.rod_error(th[i], i, roll, pitch)) for i in range(2)))
    return m, np.array(rows), (max(gaps) if gaps else 0.0)


def joint_qpos(m, name):
    import mujoco
    jid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, name)
    if jid < 0:
        raise SystemExit(f"XML 里没有 joint {name}")
    return m.jnt_qposadr[jid]


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("entity")
    ap.add_argument("xml")
    ap.add_argument("--steps", type=int, default=10000)
    ap.add_argument("--amp", type=float, default=0.3)
    ap.add_argument("--freq", type=float, default=0.6)
    ap.add_argument("--dt", type=float, default=1e-3)
    ap.add_argument("--out", default=None)
    args = ap.parse_args()

    doc = E.read_yaml(args.entity)
    m_sol = S.ParallelAnkle.from_entity(doc)
    doc0 = E.read_yaml(args.entity)
    m, qpos, gap = run_mujoco(args.xml, args.steps, args.amp, args.freq, args.dt,
                              sol=S.ParallelAnkle.from_entity(doc0))
    zipped = len(qpos) > 0
    print(f"MuJoCo: {args.steps} 步 · 关节 {m.nq} · 执行器 {m.nu} · 等式约束 {m.neq}")
    if zipped:
        pass
    print(f"  在 MuJoCo 真值处量杆长：最大违反 {gap*1e6:.2f} µm（软约束；这是它与「刚性」的差）")

    ap_pitch = joint_qpos(m, m_sol.pitch_joint)
    ap_roll = joint_qpos(m, m_sol.roll_joint)
    ap_m1 = joint_qpos(m, m_sol.motors[0])
    ap_m2 = joint_qpos(m, m_sol.motors[1])
    print(f"  qpos 索引: pitch={ap_pitch} roll={ap_roll} motor0={ap_m1} motor1={ap_m2}")

    err_p, err_r, fails = [], [], 0
    hint = None
    for row in qpos[::10]:
        th = np.array([row[ap_m1], row[ap_m2]])
        r, p, ok, _it = m_sol.fw(th, tol=1e-10, max_iter=50)
        if not ok:
            fails += 1
            continue
        err_r.append(abs(r - row[ap_roll]))
        err_p.append(abs(p - row[ap_pitch]))
        hint = th
    err_r, err_p = np.array(err_r), np.array(err_p)
    worst = max(err_r.max(), err_p.max())
    print(f"  采样 {len(err_r)} 点 · pitch err max {err_p.max():.3e} / rms {np.sqrt((err_p**2).mean()):.3e} rad")
    print(f"                roll  err max {err_r.max():.3e} / rms {np.sqrt((err_r**2).mean()):.3e} rad")
    print(f"  牛顿不收敛 {fails} 次")

    # 三项自检（与 Q9 一致）
    ok_rod = S.selfcheck(doc, verbose=False)
    print(f"  自检（杆长守恒 + 雅可比）: {'PASS' if ok_rod else 'FAIL'}")

    verdict = (worst <= 1e-4) and ok_rod and fails == 0
    print(f"总判定: {'PASS' if verdict else 'FAIL'}  (pitch/roll 最大误差 {worst:.3e} rad，阈值 1e-4)")

    if args.out:
        with open(args.out, "w", encoding="utf-8") as f:
            json.dump({"entity": args.entity, "xml": args.xml, "steps": args.steps,
                       "worst_rad": float(worst), "pitch_max": float(err_p.max()),
                       "roll_max": float(err_r.max()), "fails": int(fails),
                       "constraint_gap_mm": float(gap * 1e3), "verdict": bool(verdict)}, f, indent=1)
        print(f"报告写入 {args.out}")
    return 0 if verdict else 1


if __name__ == "__main__":
    raise SystemExit(main())
