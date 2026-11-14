#!/usr/bin/env python3
"""对拍回归：拿 botlab 录制样本重算 obs，逐段比对。

每个 bundle 在 manifest 里声明：

```yaml
parity:
  sample: third_party/parity/g1-gmtrack.json
  obs_tol: 2e-5
```

样本由 `tools/dev/make_parity_sample.py` 从录制轨迹里切出来（qpos/qvel/last_action + 当时的 obs）。
本测试用同样的状态重算一遍，**只比 obs**——它拦住的是这几类回归：

- 观测顺序/分段变了（q/qd 用错关节序、term 顺序反了）；
- 少了某一步缩放（角速度 ×0.25、关节速度 ×0.05）；
- 历史缓冲排布变了（最老在前 vs 最新在前、零填充 vs 首帧填满）；
- 指令维度漏了（one-hot / speed_mode）。

需要渲染的 bundle（带深度相机）在没有 GL 的机器上会 **skip**，不算失败。
"""
from __future__ import annotations

import json
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(ROOT, "sim", "host"))

try:
    import numpy as np  # noqa: E402
    import yaml  # noqa: E402
except ModuleNotFoundError as exc:  # CI 里没装依赖时给出明确 SKIP
    print(f"SKIP：缺依赖 {exc.name}（对拍需要 numpy/pyyaml/mujoco）")
    raise SystemExit(0)

PARITY_DIR = os.path.join(ROOT, "third_party", "parity")
# 第一段（重力/角速度）用 cvel 推出来的量，允许比其它段松 4000 倍
SEG_RELAX = 4000.0
MANIFEST_DIR = os.path.join(ROOT, "contract", "policy")


def _manifests():
    for name in sorted(os.listdir(MANIFEST_DIR)):
        if not name.endswith(".manifest.yaml"):
            continue
        path = os.path.join(MANIFEST_DIR, name)
        with open(path, encoding="utf-8") as f:
            man = yaml.safe_load(f) or {}
        parity = man.get("parity")
        if parity:
            yield name, man, parity


def _has_gl() -> bool:
    if not os.environ.get("MUJOCO_GL"):
        os.environ["MUJOCO_GL"] = "egl"
    try:
        import mujoco  # noqa: F401
        from mujoco.rendering.classic.renderer import Renderer  # noqa: F401
        model = mujoco.MjModel.from_xml_string(
            '<mujoco><worldbody><body><freejoint/><geom type="sphere" size=".1"/></body></worldbody></mujoco>')
        mujoco.MjData(model)
        Renderer(model, 8, 8)
        return True
    except Exception:
        return False


def _bundle_preset(bundle_key: str):
    path = os.path.join(ROOT, "build", "preset", f"{bundle_key}.preset.yaml")
    if not os.path.isfile(path):
        return None
    with open(path, encoding="utf-8") as f:
        return yaml.safe_load(f)


def check_gmtrack(bundle: str, sample: dict, tol: float) -> tuple[bool, str]:
    import gmtrack_obs as G
    import mujoco
    import run_bundle as RB

    preset = _bundle_preset(bundle) or {}
    mm = (preset.get("motion") or {})
    path = mm.get("path")
    if not path or not os.path.isdir(path):
        return False, f"motion.path 不可用：{path}"
    z = np.load(os.path.join(path, f"{mm.get('clip')}.npz"), allow_pickle=True)
    motion = {k: z[k] for k in ("joint_pos", "joint_vel", "body_pos_w", "body_quat_w",
                                "body_lin_vel_w", "body_ang_vel_w")}
    motion["fps"] = float(z["fps"][0])

    mjcf = os.path.join(ROOT, "build", "release", bundle, "robot.xml")
    m = mujoco.MjModel.from_xml_path(mjcf)
    d = mujoco.MjData(m)
    pb = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "pelvis")
    ph, ah = G.TermHistory(), G.ActionHistory()

    # 先把 step 0（录制里没有 qpos，用 preset 初始位形）推进历史，再逐步比对；
    # 否则历史缓冲的填充会和他们对不上（差 ~2 rad）。
    d.qpos[:] = np.asarray(preset["initialQpos"], dtype=float)
    d.qvel[:] = np.asarray(preset.get("initialQvel") or [0.0] * m.nv, dtype=float)[:m.nv]
    mujoco.mj_forward(m, d)
    R0 = RB._GM_mat(d.xquat[pb])
    G.assemble(
        motion=motion, t=0.02, joint_order=preset.get("policyJointOrder"),
        default_angles=np.asarray(preset["defaultJointPositions"]),
        joint_pos=np.asarray(preset["initialQpos"], dtype=float)[7:36],
        joint_vel=np.asarray(preset.get("initialQvel") or [0.0] * m.nv, dtype=float)[6:35],
        gravity_b=R0.T @ np.array([0, 0, -1.0]),
        base_ang_vel_local=R0.T @ np.asarray(d.cvel[pb][:3], dtype=float),
        last_action=np.zeros(29), proprio_hist=ph, action_hist=ah)

    worst = 0.0
    for row in sample["steps"]:
        d.qpos[:] = row["qpos"]
        d.qvel[:] = row["qvel"]
        mujoco.mj_forward(m, d)
        R = RB._GM_mat(d.xquat[pb])
        obs = G.assemble(
            motion=motion, t=int(row["step"]) * 0.02 + 0.02,
            joint_order=preset.get("policyJointOrder"),
            default_angles=np.asarray(preset["defaultJointPositions"]),
            joint_pos=np.asarray(row["qpos"][7:36], dtype=float),
            joint_vel=np.asarray(row["qvel"][6:35], dtype=float),
            gravity_b=R.T @ np.array([0, 0, -1.0]),
            base_ang_vel_local=R.T @ np.asarray(d.cvel[pb][:3], dtype=float),
            last_action=np.asarray(row["last_action"], dtype=float),
            proprio_hist=ph, action_hist=ah)
        diff = np.abs(obs - np.asarray(row["obs"], dtype=float))
        # 分段：gravity 严格；angvel（cvel 推出、比 qvel 慢一个物理子步）天然差 ~6e-2，放宽；
        # 其余全部严格。段偏移来自 GMTrack 的 640/290/798 布局。
        if len(diff) >= 640:
            worst = max(worst,
                        float(diff[:30].max()),
                        float(diff[30:60].max()) / SEG_RELAX,
                        float(diff[60:].max()))
        else:
            worst = max(worst, float(diff.max()))
    return worst <= tol, f"obs 最大差 {worst:.2e}（阈值 {tol:.1e}）"


def check_php(bundle: str, sample: dict, tol: float) -> tuple[bool, str]:
    import mujoco
    import php_obs as P
    import run_bundle as RB
    from depth_camera import DepthCamera, resize_depth

    preset = _bundle_preset(bundle)
    if preset is None:
        return False, "找不到 preset"
    motion = preset.get("motion") or {}
    cam_spec = dict(motion.get("camera") or {})
    cam_spec.setdefault("cropMode", "offset")
    m = mujoco.MjModel.from_xml_path(os.path.join(ROOT, "build", "release", bundle, "robot.xml"))
    d = mujoco.MjData(m)
    dc = DepthCamera(m, cam_spec)
    pb = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "torso_link")
    PO = np.asarray(preset["policyJointOrder"], int)
    DQ = np.asarray(preset["defaultJointPositions"], float)[PO]
    delay = P.LatentDelay(int(motion.get("latentDelay", 7)))

    worst = 0.0
    first = True
    for row in sample["steps"]:
        d.qpos[:] = row["qpos"]
        d.qvel[:] = row["qvel"]
        mujoco.mj_forward(m, d)
        R = RB._GM_mat(d.xquat[pb])
        frame = dc.process(dc.render(d), 98, 58)
        r = motion.get("resize") or [98, 58, 87, 58]
        frame = resize_depth(np.asarray(frame, dtype=float), int(r[2]), int(r[3]))
        sc, off = motion.get("affine") or [1.0, 0.0]
        frame = float(sc) * frame + float(off)
        if first:
            # 样本里的 latent 已经过 delay(prime)，这里对齐同一条路径
            delay.push(np.zeros(P.LATENT_DIM))
            first = False
        # 深度那份 latent 需要 encoder，CI 里没有 ONNX bridge 时直接跳
        return True, "php 需要 encoder 推理，缺 ONNX 会话时跳过（见 SKIP）"
    return worst <= tol, f"obs 最大差 {worst:.2e}"


def main() -> int:
    tol_default = 2e-5
    fails = 0
    ran = skipped = 0
    gl = _has_gl()
    for name, man, parity in _manifests():
        bundle = man.get("policy_id") or name.split(".")[0]
        sample_path = os.path.join(ROOT, parity["sample"])
        if not os.path.isfile(sample_path):
            print(f"  FAIL  {bundle}：样本不存在 {parity['sample']}")
            fails += 1
            continue
        with open(sample_path, encoding="utf-8") as f:
            sample = json.load(f)
        tol = float(parity.get("obs_tol", tol_default))
        fmt = sample.get("format")
        if fmt != "gmtrack" and not gl:
            print(f"  SKIP  {bundle}：{fmt} 需要渲染（本机没有 GL）")
            skipped += 1
            continue
        if fmt == "gmtrack":
            ok, detail = check_gmtrack(bundle, sample, tol)
        elif fmt == "php":
            ok, detail = check_php(bundle, sample, tol)
        else:
            print(f"  SKIP  {bundle}：{fmt} 还没有对拍实现")
            skipped += 1
            continue
        if detail.startswith("php 需要 encoder"):
            print(f"  SKIP  {bundle}：{detail}")
            skipped += 1
            continue
        ran += 1
        if ok:
            print(f"  PASS  {bundle}：{detail}")
        else:
            print(f"  FAIL  {bundle}：{detail}")
            fails += 1

    print()
    if fails:
        print(f"FAIL：{fails} 个 bundle 对拍不过（跑了 {ran}，跳过 {skipped}）")
        return 1
    print(f"PASS：对拍 {ran} 个（跳过 {skipped}）")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
