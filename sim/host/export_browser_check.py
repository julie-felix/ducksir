#!/usr/bin/env python3
"""导出「浏览器自算」用的素材：plan blob + obs 序列 + 宿主算出的 action。

    PYTHONPATH=$HOME/.local/share/ducksir/pylibs python3 sim/host/export_browser_check.py \
        --manifest contract/policy/duck-walk.manifest.yaml --steps 60 --vx 0.5

产物落 `client/viewer/data/live/`：
- `plan.blob`   编译好的图（浏览器用 wasm 核加载它）
- `obs.json`    每一步的 61 维观测（宿主在同一次运行里采的）
- `actions.json` 宿主用 C 核 + ORT 算出的 action（浏览器要复现它）
- `model.onnx`  策略权重（`*.onnx` 已 gitignore，不入库）

浏览器页 `client/viewer/live.html` 会把这三样喂给 wasm 核 + ORT Web，和 `actions.json` 逐位比。
"""

from __future__ import annotations

import argparse
import ctypes
import json
import os
import shutil
import sys

import numpy as np
import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
sys.path.insert(0, os.path.join(ROOT, "sim", "plan"))
sys.path.insert(0, os.path.join(ROOT, "tools", "robot"))
sys.path.insert(0, HERE)

import entity as E  # noqa: E402
import ir  # noqa: E402
import run_policy as RP  # noqa: E402
from run_policy import Info, load_libs, quat_to_mat  # noqa: E402


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--mjcf", default="build/mjcf/duck-hd1910-all.xml")
    ap.add_argument("--preset", default="build/preset/duck-hd1910.preset.yaml")
    ap.add_argument("--manifest", default="contract/policy/duck-walk.manifest.yaml")
    ap.add_argument("--steps", type=int, default=60)
    ap.add_argument("--vx", type=float, default=0.5)
    ap.add_argument("--out", default="client/viewer/data/live")
    args = ap.parse_args()

    import mujoco

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
    # 切成两段：推理在中间由浏览器异步做（ORT Web 只有 async API，核是同步的）
    pre = ir.Plan(input_dim=sum(o["len"] for o in man["obs"]), output_dim=sum(o["len"] for o in man["obs"]),
                  name="pre")
    sl, off = [], 0
    for o in man["obs"]:
        sl.append(pre.obs(off, o["len"]))
        off += o["len"]
    level = list(sl)
    while len(level) > 4:
        nxt = []
        for i in range(0, len(level), 4):
            chunk = level[i:i + 4]
            nxt.append(chunk[0] if len(chunk) == 1 else pre.concat(*chunk))
        level = nxt
    pre.out_action(level[0] if len(level) == 1 else pre.concat(*level))
    pre_blob = ir.compile_blob(pre.to_json())

    post = ir.Plan(input_dim=n, output_dim=n, name="post")
    raw = post.obs(0, n)
    ctrl_sig = post.affine(raw, preset["actionScale"], preset["defaultJointPositions"])
    post.out_action(post.clip(ctrl_sig, lo, hi))
    post_blob = ir.compile_blob(post.to_json())

    # 宿主自己的参照运行用**完整图**（61→14）；pre/post 两份只给浏览器
    plan = RP.build_plan(man, preset, n, lo, hi)
    blob = ir.compile_blob(plan)

    core, ort = load_libs()
    err = ctypes.create_string_buffer(512)
    model_path = os.path.join(ROOT, man["model"]["file"])
    be = ort.ds_ort_create(model_path.encode(), err, 512)
    if not be:
        raise SystemExit("ORT 后端创建失败：" + err.value.decode())
    handle = core.ds_load(blob, len(blob), err, 512)
    if not handle:
        raise SystemExit("加载 plan 失败：" + err.value.decode())
    info = Info()
    core.ds_info_of(handle, ctypes.byref(info))

    spec = mujoco.MjSpec.from_file(os.path.join(ROOT, args.mjcf))
    g = spec.worldbody.add_geom()
    g.type = mujoco.mjtGeom.mjGEOM_PLANE
    g.size = [4.0, 4.0, 0.1]
    g.condim = 3
    m = spec.compile()
    d = mujoco.MjData(m)
    m.opt.timestep = 1.0 / 200.0
    d.qpos[:] = preset["initialQpos"]
    mujoco.mj_forward(m, d)

    qoff = [int(m.jnt_qposadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, nm)]) for nm in driven]
    doff = [int(m.jnt_dofadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, nm)]) for nm in driven]
    sadr = int(m.sensor_adr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SENSOR, "imu_ang_vel")])
    default_q = np.array(preset["defaultJointPositions"])

    obs_list, act_list = [], []
    act = np.zeros(n, dtype=np.float32)
    for _ in range(args.steps):
        R = quat_to_mat(d.qpos[3:7])
        obs = np.zeros(61, dtype=np.float32)
        obs[0:3] = d.sensordata[sadr:sadr + 3]
        obs[3:6] = R.T @ np.array([0.0, 0.0, -1.0])
        obs[6:20] = np.array([d.qpos[i] for i in qoff]) - default_q
        obs[20:34] = np.array([d.qvel[i] for i in doff])
        obs[34:48] = act
        obs[48] = args.vx
        bi = (ctypes.c_float * 61)(*obs.tolist())
        bo = (ctypes.c_float * n)()
        rc = core.ds_step(handle, bi, bo, be)
        if rc != 0:
            raise SystemExit(f"ds_step 返回 {rc}")
        act = np.array(bo[:], dtype=np.float32)
        obs_list.append([float(x) for x in obs])
        act_list.append([float(x) for x in act])
        d.ctrl[:] = act
        for _ in range(4):
            mujoco.mj_step(m, d)

    out = os.path.join(ROOT, args.out)
    os.makedirs(out, exist_ok=True)
    with open(os.path.join(out, "pre.blob"), "wb") as f:
        f.write(pre_blob)
    with open(os.path.join(out, "post.blob"), "wb") as f:
        f.write(post_blob)
    with open(os.path.join(out, "obs.json"), "w", encoding="utf-8") as f:
        json.dump(obs_list, f)
    with open(os.path.join(out, "actions.json"), "w", encoding="utf-8") as f:
        json.dump(act_list, f)
    shutil.copyfile(model_path, os.path.join(out, "model.onnx"))
    with open(os.path.join(out, "meta.json"), "w", encoding="utf-8") as f:
        json.dump({"policy_id": man["policy_id"], "steps": args.steps, "vx": args.vx,
                   "pre_digest": ir.sha256_of_blob(pre_blob),
                   "post_digest": ir.sha256_of_blob(post_blob),
                   "input_dim": info.input_dim, "output_dim": info.output_dim,
                   "nodes": info.node_count,
                   "model_input": man["model"]["input"]["name"],
                   "model_output": man["model"]["output"]["name"]}, f, ensure_ascii=False, indent=2)

    print(f"导出 {args.steps} 步 → {args.out}")
    print(f"  pre {len(pre_blob)} B / post {len(post_blob)} B｜{info.node_count} 节点｜{info.input_dim}→{info.output_dim}")
    print(f"  模型 {os.path.getsize(model_path)/1024:.0f} KB")

    core.ds_free(handle)
    ort.ds_ort_destroy(be)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
