#!/usr/bin/env python3
"""两份 MJCF 的结构对比：我们的导出 vs 上游原版。

    PYTHONPATH=$HOME/.local/share/ducksir/pylibs \
    python3 tools/robot/compare_mjcf.py build/mjcf/duck-hd1910.xml \
        third_party/fanhao-microduck-replica/robot_walk.xml

看四件事，每件都给「哪边多/少/差多少」：
1. 规模：body / joint / qpos / dof / actuator / 总质量；
2. 名字集合：body、joint、actuator 各自的差集（顺序也算——ctrl 序错位是最贵的错）；
3. 数值：同名关节的轴（点积）与限位（rad 差）、同名 body 的质量差；
4. 结论行：能不能对上，对不上差在哪。

退出码：0 = 结构一致（数值差都在容差内）；1 = 有实质差异，看输出。
容差：质量 1e-9 kg、限位 1e-9 rad、轴点积 ≥ 1-1e-9。
"""

from __future__ import annotations

import argparse
import sys

MASSTOL = 1e-9
RANGETOL = 1e-9
AXISTOL = 1e-9


def load(path: str):
    import mujoco

    m = mujoco.MjModel.from_xml_path(path)
    out = {"m": m, "path": path}
    out["bodies"] = [mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, i) or f"<{i}>" for i in range(m.nbody)]
    out["joints"] = [mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_JOINT, i) or f"<{i}>" for i in range(m.njnt)]
    out["acts"] = [mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_ACTUATOR, i) or f"<{i}>" for i in range(m.nu)]
    return out


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("ours")
    ap.add_argument("theirs")
    args = ap.parse_args()
    try:
        import mujoco
    except ImportError:
        print("跳过：没装 mujoco（见 docs/env.md）")
        return 0

    a, b = load(args.ours), load(args.theirs)
    ma, mb = a["m"], b["m"]
    print(f"ours   {a['path']}")
    print(f"theirs {b['path']}\n")

    rows = [
        ("body", ma.nbody, mb.nbody),
        ("joint", ma.njnt, mb.njnt),
        ("qpos", ma.nq, mb.nq),
        ("dof", ma.nv, mb.nv),
        ("actuator", ma.nu, mb.nu),
    ]
    bad = 0
    print(f"{'项':<10}{'ours':>8}{'theirs':>8}  {'':<4}")
    for name, x, y in rows:
        ok = x == y
        bad += 0 if ok else 1
        print(f"{name:<10}{x:>8}{y:>8}  {'OK' if ok else 'DIFF'}")

    mass_a, mass_b = float(ma.body_mass.sum()), float(mb.body_mass.sum())
    mass_ok = abs(mass_a - mass_b) < MASSTOL
    bad += 0 if mass_ok else 1
    print(f"{'mass(kg)':<10}{mass_a:>8.5f}{mass_b:>8.5f}  {'OK' if mass_ok else 'DIFF'}")

    def diff(label, xs, ys):
        nonlocal bad
        only_a, only_b = sorted(set(xs) - set(ys)), sorted(set(ys) - set(xs))
        same_order = xs == ys
        if only_a or only_b or not same_order:
            bad += 1
        print(f"\n{label}: {'一致' if not (only_a or only_b or not same_order) else ''}")
        if only_a:
            print(f"  只在 ours: {only_a[:8]}{' …' if len(only_a) > 8 else ''}")
        if only_b:
            print(f"  只在 theirs: {only_b[:8]}{' …' if len(only_b) > 8 else ''}")
        if not same_order and not (only_a or only_b):
            for i, (x, y) in enumerate(zip(xs, ys)):
                if x != y:
                    print(f"  顺序首次不同 @{i}: ours={x} theirs={y}")
                    break

    diff("body 名单", a["bodies"], b["bodies"])
    diff("joint 名单", a["joints"], b["joints"])
    diff("actuator 顺序", a["acts"], b["acts"])

    # 同名关节：轴与限位
    diffs = []
    for name in set(a["joints"]) & set(b["joints"]):
        ja = mujoco.mj_name2id(ma, mujoco.mjtObj.mjOBJ_JOINT, name)
        jb = mujoco.mj_name2id(mb, mujoco.mjtObj.mjOBJ_JOINT, name)
        if ja < 0 or jb < 0:
            continue
        axa, axb = ma.jnt_axis[ja], mb.jnt_axis[jb]
        dot = float((axa * axb).sum())
        if abs(dot - 1.0) > AXISTOL:
            diffs.append(f"{name}: 轴不一致 dot={dot:.6f}")
        ra, rb = ma.jnt_range[ja], mb.jnt_range[jb]
        if ma.jnt_limited[ja] and mb.jnt_limited[jb]:
            if max(abs(ra[0] - rb[0]), abs(ra[1] - rb[1])) > RANGETOL:
                diffs.append(f"{name}: 限位差 ({ra[0]:+.6f},{ra[1]:+.6f}) vs ({rb[0]:+.6f},{rb[1]:+.6f})")
    print(f"\n关节轴/限位: {'全部一致' if not diffs else ''}")
    for d in diffs[:10]:
        print(f"  {d}")
    bad += 1 if diffs else 0

    # 同名 body：质量
    mdiffs = []
    for name in set(a["bodies"]) & set(b["bodies"]):
        ba = mujoco.mj_name2id(ma, mujoco.mjtObj.mjOBJ_BODY, name)
        bb = mujoco.mj_name2id(mb, mujoco.mjtObj.mjOBJ_BODY, name)
        if ba < 0 or bb < 0:
            continue
        d = abs(float(ma.body_mass[ba]) - float(mb.body_mass[bb]))
        if d > MASSTOL:
            mdiffs.append(f"{name}: 质量差 {d:.3e} kg")
    print(f"body 质量: {'全部一致' if not mdiffs else ''}")
    for d in mdiffs[:10]:
        print(f"  {d}")
    bad += 1 if mdiffs else 0

    print(f"\n结论：{'结构一致' if not bad else f'{bad} 处差异'}")
    return 0 if not bad else 1


if __name__ == "__main__":
    raise SystemExit(main())
