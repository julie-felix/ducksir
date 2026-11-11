#!/usr/bin/env python3
"""G1（botlab 模板里的 29DOF）能不能在本机 MuJoCo 里站着跑起来。

这不是策略测试，只验三件事：
1. 导出的 MJCF 能被 MuJoCo 加载（含那些 1e-9 kg 的占位 body）；
2. preset 的 initialQpos 真的把机器人放在地上而不是埋在 / 顶在别处；
3. 保持默认关节角跑 1 秒，数值不发散。

真正的运控要等 G1 的观测配方 + 策略接进来。
"""
import os
import sys

import numpy as np
import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
MJCF = os.path.join(ROOT, "build/mjcf/g1-29dof-botlab.xml")
PRESET = os.path.join(ROOT, "build/preset/g1-29dof-botlab.preset.yaml")

fails = []


def ok(name, cond, detail=""):
    print(("  PASS  " if cond else "  FAIL  ") + name + (f"  {detail}" if detail else ""))
    if not cond:
        fails.append(name)


if not (os.path.exists(MJCF) and os.path.exists(PRESET)):
    print("  跳过：没导出 g1-29dof-botlab 的 MJCF/preset（见 docs/notes/g1-botlab.md）")
    print("PASS（跳过）")
    raise SystemExit(0)

import mujoco  # noqa: E402

pre = yaml.safe_load(open(PRESET, encoding="utf-8"))
spec = mujoco.MjSpec.from_file(MJCF)
plane = spec.worldbody.add_geom()
plane.type = mujoco.mjtGeom.mjGEOM_PLANE
plane.size = [6.0, 6.0, 0.1]
plane.condim = 3
m = spec.compile()
d = mujoco.MjData(m)
m.opt.timestep = 1.0 / 200.0

ok("MJCF 能加载", m.nu == 29 and m.nq == 36, f"nu={m.nu} nq={m.nq}")
ok("总质量合理（G1 ≈ 33 kg）", 25 < float(np.sum(m.body_mass)) < 45,
   f"{float(np.sum(m.body_mass)):.1f} kg")

d.qpos[:] = pre["initialQpos"]
mujoco.mj_forward(m, d)
z0 = float(d.qpos[2])
ok("初始基座高度合理（0.5–1.2 m）", 0.5 < z0 < 1.2, f"z={z0:.3f}")

zs = []
for _ in range(200):
    d.ctrl[:] = pre["defaultJointPositions"]
    mujoco.mj_step(m, d)
    zs.append(float(d.qpos[2]))

ok("跑 1 秒不发散（有限值、没飞天）", bool(np.isfinite(d.qpos).all()) and max(zs) < z0 + 0.5,
   f"z: {z0:.3f} → {zs[-1]:.3f}（最低 {min(zs):.3f}）")
ok("关节角保持在合理范围（|q| < π）", float(np.abs(d.qpos[7:]).max()) < np.pi + 0.1,
   f"max|q|={float(np.abs(d.qpos[7:]).max()):.2f}")

print()
print("说明：这里只是「能加载、能跑、不发散」。保持默认角不等于站稳——G1 要靠策略平衡，")
print("下一步是 G1 的观测配方 + 策略（WBC / Instinct），那才是真正的运控验证。")
if fails:
    print(f"失败 {len(fails)} 项：" + "、".join(fails))
    raise SystemExit(1)
print("PASS：G1 在本机 MuJoCo 里能加载并稳定模拟（不是策略测试）")
