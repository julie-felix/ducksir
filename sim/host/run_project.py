#!/usr/bin/env python3
"""跑一个项目脚本：`ducksir run project.py` 的实现。

    python3 sim/host/run_project.py --project /tmp/my_project.py --seconds 20
    ... --no-browser --report build/report/proj.json

生命周期归它（Q1′/Q8 的决定）：
- `--seconds` 与 `--steps` 双上限，先到先停；
- Ctrl-C **先把报告和轨迹落盘**、报告记 `interrupted: true`、退出码 130；
- 默认起显示服务并打开浏览器（Q5′=C），`--no-browser` 关掉。

观测装配走契约（`contract.json` 的配方），控制图用 `Graph.standard`（与 `ir.Plan` 逐字节等价）。
"""

from __future__ import annotations

import argparse
import ctypes
import importlib.util
import json
import os
import signal
import sys
import time
import webbrowser

import numpy as np
import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
sys.path.insert(0, os.path.join(ROOT, "sim", "plan"))
sys.path.insert(0, os.path.join(ROOT, "sim", "host"))
sys.path.insert(0, os.path.join(ROOT, "tools", "robot"))
sys.path.insert(0, os.path.join(ROOT, "sim", "sdk"))

import contract_obs as CO  # noqa: E402
import display as DSP  # noqa: E402
import entity as E  # noqa: E402
import graph as G  # noqa: E402
import ir  # noqa: E402
import panel as PANEL  # noqa: E402
from run_policy import Info, load_libs, sha256  # noqa: E402


def load_project(path: str) -> dict:
    """import 用户脚本，取 `project` 变量（也接受 `PROJECT` 或直接返回 dict 的函数）。"""
    spec = importlib.util.spec_from_file_location("ducksir_project", path)
    if spec is None or spec.loader is None:
        raise SystemExit(f"读不了项目脚本：{path}")
    mod = importlib.util.module_from_spec(spec)
    sys.modules["ducksir_project"] = mod
    spec.loader.exec_module(mod)
    obj = getattr(mod, "project", None) or getattr(mod, "PROJECT", None)
    if obj is None:
        raise SystemExit(f"{path} 里没有 `project = Project(...)`")
    cfg = obj.deliver() if hasattr(obj, "deliver") else obj.describe() if hasattr(obj, "describe") else obj
    if not isinstance(cfg, dict):
        raise SystemExit(f"{path} 的 project 不是 Project 也不是 dict")
    for key in ("robot", "manifest", "policies"):
        if key not in cfg:
            raise SystemExit(f"项目配置缺 `{key}`")
    return cfg


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--project", required=True, help="项目脚本路径")
    ap.add_argument("--seconds", type=float, help="跑多久（覆盖脚本里的）")
    ap.add_argument("--steps", type=int, help="跑多少控制步（与 --seconds 先到先停）")
    ap.add_argument("--contract", default="client/viewer/data/live/contract.json")
    ap.add_argument("--mjcf", default="build/mjcf/duck-hd1910-all.xml")
    ap.add_argument("--preset", default="build/preset/duck-hd1910.preset.yaml")
    ap.add_argument("--report")
    ap.add_argument("--trajectory")
    ap.add_argument("--port", type=int, default=8765)
    ap.add_argument("--no-browser", action="store_true", help="不起显示服务也不开浏览器")
    ap.add_argument("--no-open", action="store_true", help="起显示服务但不自动开浏览器")
    ap.add_argument("--page", choices=("studio", "follow"), default="studio",
                    help="打开哪个页面（默认 studio：文件树 + 3D + 参数/指标）")
    ap.add_argument("--pace", choices=("auto", "realtime", "max"), default="auto",
                    help="节拍：auto=开显示时按实时、否则全速（默认）")
    args = ap.parse_args()

    import mujoco

    try:
        cfg = load_project(os.path.join(ROOT, args.project) if not os.path.isabs(args.project) else args.project)
    except SystemExit:
        raise
    except Exception as e:  # noqa: BLE001
        raise SystemExit(f"项目脚本有问题：{e}")
    with open(os.path.join(ROOT, args.preset), encoding="utf-8") as f:
        preset = yaml.safe_load(f)
    with open(os.path.join(ROOT, cfg["manifest"]), encoding="utf-8") as f:
        man = yaml.safe_load(f)
    doc = E.read_yaml(os.path.join(ROOT, preset["generated"]["from"]))
    driven = [j["name"] for j in doc["joints"] if j.get("actuated")]
    n = len(driven)
    acts = doc["actuators"]
    lo = [float(a["ctrlrange"][0]) for a in acts]
    hi = [float(a["ctrlrange"][1]) for a in acts]

    con = CO.load_contract(os.path.join(ROOT, args.contract))
    control_hz = float(cfg["rates"]["control_hz"])
    physics_hz = float(cfg["rates"]["physics_hz"])
    decim = int(round(physics_hz / control_hz))
    seconds = args.seconds if args.seconds is not None else (cfg.get("seconds") or 20.0)
    steps = int(seconds * control_hz)
    if args.steps is not None:
        steps = min(steps, args.steps)

    if len(cfg["policies"]) != 1:
        raise SystemExit("目前一次只能跑一个策略；多策略切换要等状态机接入 run_project")
    pol = cfg["policies"][0]
    model_path = os.path.join(ROOT, "build/models", f"{pol['key']}.onnx")
    if not os.path.exists(model_path):
        raise SystemExit(f"缺模型 {model_path}")

    # 图：用 builder（与 ir.Plan 逐字节等价），端点仍编译成 blob 交给 C 核
    g = (G.Graph(man, preset, name=man.get("policy_id", "project"))
         .observe_all().policy(pol["key"], mode=int(pol.get("mode", 0)))
         .action(clip="ctrlrange", ctrl_lo=lo, ctrl_hi=hi))
    blob = g.build()

    core, ort = load_libs()
    err = ctypes.create_string_buffer(512)
    be = ort.ds_ort_create(model_path.encode(), err, 512)
    if not be:
        raise SystemExit("ORT 后端创建失败：" + err.value.decode())
    handle = core.ds_load(blob, len(blob), err, 512)
    if not handle:
        raise SystemExit("加载 plan 失败：" + err.value.decode())
    info = Info()
    core.ds_info_of(handle, ctypes.byref(info))

    spec = mujoco.MjSpec.from_file(os.path.join(ROOT, args.mjcf))
    geom = spec.worldbody.add_geom()
    geom.type = mujoco.mjtGeom.mjGEOM_PLANE
    geom.size = [6.0, 6.0, 0.1]
    geom.condim = 3
    m = spec.compile()
    d = mujoco.MjData(m)
    m.opt.timestep = 1.0 / physics_hz
    d.qpos[:] = preset["initialQpos"]
    mujoco.mj_forward(m, d)

    qpos_off = [int(m.jnt_qposadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, nm)]) for nm in driven]
    dof_off = [int(m.jnt_dofadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, nm)]) for nm in driven]

    def sensor_adr(name: str) -> int:
        sid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SENSOR, name)
        if sid < 0:
            raise SystemExit(f"MJCF 里没有传感器 {name}")
        return int(m.sensor_adr[sid])

    def quat_of(body: str):
        bid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, body)
        if bid < 0:
            raise SystemExit(f"MJCF 里没有 body {body}")
        return d.xquat[bid]

    view = {"data": d, "qpos_off": qpos_off, "dof_off": dof_off,
            "sensor_adr": sensor_adr, "quat_of": quat_of}

    body_names = [mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, i) or f"body{i}" for i in range(m.nbody)]
    scene = {"model": os.path.basename(args.mjcf), "bodies": body_names,
             "control_hz": control_hz, "physics_hz": physics_hz,
             "policy": pol["key"], "project": os.path.basename(args.project)}

    # studio 的文件树：项目脚本所在目录里的源码/配置（只读展示）
    project_abs = args.project if os.path.isabs(args.project) else os.path.join(ROOT, args.project)
    project_dir = os.path.dirname(project_abs) or ROOT
    allow_ext = (".py", ".yaml", ".yml", ".md", ".json", ".txt")
    proj_files = []
    for name in sorted(os.listdir(project_dir)):
        full = os.path.join(project_dir, name)
        if os.path.isfile(full) and name.endswith(allow_ext):
            proj_files.append(name)
    # 再挂上仓库里跟这个项目有关的两份声明（相对路径，只读）
    for rel in (cfg["manifest"], "contract/robot/postures/duck-hd1910.yaml"):
        if os.path.exists(os.path.join(ROOT, rel)):
            proj_files.append(rel)

    disp = None
    if not args.no_browser:
        disp = DSP.Display(port=args.port, scene=scene,
                           files_root=project_dir, files=proj_files,
                           command=dict(cfg["command"])).start()
        if not DSP.wait_ready(f"http://127.0.0.1:{args.port}"):
            print("  (显示服务没起来，继续无头跑)")
            disp.stop()
            disp = None
        else:
            follow_url = f"http://127.0.0.1:{args.port}/{args.page}"
            if not args.no_open:
                webbrowser.open(follow_url)
            print(f"  显示：{follow_url}（SSE {disp.url}）")

    cmd13 = np.zeros(13, dtype=np.float32)
    cmd13[0] = float(cfg["command"].get("forward", 0.0))
    cmd13[2] = float(cfg["command"].get("yaw", 0.0))

    act = np.zeros(n, dtype=np.float32)
    heights, tilts, step_times, frames = [], [], [], []
    x0, y0 = float(d.qpos[0]), float(d.qpos[1])
    # 节拍：给浏览器看就必须按实时走，否则 170× 的模糊帧没人看得清
    pace_realtime = args.pace == "realtime" or (args.pace == "auto" and not args.no_browser)
    deadline = 0.0
    interrupted = False
    dead = False
    t0 = time.time()
    deadline = t0
    t_sim = 0.0
    completed = 0

    def handle_sigint(signum, frame):
        nonlocal interrupted
        interrupted = True

    old_handler = signal.signal(signal.SIGINT, handle_sigint)

    try:
        for k in range(steps):
            if interrupted:
                break
            t_step = time.perf_counter()
            if disp is not None:
                live = disp.get_command()
                cmd13[0] = float(live.get("forward", cmd13[0]))
                cmd13[2] = float(live.get("yaw", cmd13[2]))
            obs = CO.make_obs(con, view, last_action=act, command=cmd13)
            bi = (ctypes.c_float * len(obs))(*obs.tolist())
            bo = (ctypes.c_float * n)()
            rc = core.ds_step(handle, bi, bo, be)
            if rc != 0:
                raise SystemExit(f"ds_step 返回 {rc}")
            act = np.array(bo[:], dtype=np.float32)
            d.ctrl[:] = act
            for _ in range(decim):
                mujoco.mj_step(m, d)
            if not np.isfinite(d.qpos).all():
                dead = True
                break
            t_sim += 1.0 / control_hz
            step_times.append(time.perf_counter() - t_step)
            heights.append(float(d.qpos[2]))
            tilts.append(float(1 - 2 * (d.qpos[4] ** 2 + d.qpos[5] ** 2)))
            completed = k + 1
            if disp is not None:
                disp.set_stats({
                    "base_height": round(float(d.qpos[2]), 4),
                    "upright": round(float(1 - 2 * (d.qpos[4] ** 2 + d.qpos[5] ** 2)), 4),
                    "rtf": round(completed / max(1e-6, time.time() - t0), 2),
                    "forward": round(float(cmd13[0]), 3),
                    "yaw": round(float(cmd13[2]), 3),
                    "step": completed,
                })
                disp.push(t_sim, d.xpos.reshape(-1), d.xquat.reshape(-1))
            if pace_realtime:
                deadline += 1.0 / control_hz
                slack = deadline - time.time()
                if slack > 0:
                    time.sleep(slack)
            if args.trajectory:
                frames.append({"t": round(k / control_hz, 4),
                               "pos": [round(float(x), 6) for x in d.qpos[0:3]],
                               "quat": [round(float(x), 6) for x in d.qpos[3:7]],
                               "q": [round(float(d.qpos[i]), 6) for i in qpos_off]})
    finally:
        signal.signal(signal.SIGINT, old_handler)
        if disp is not None:
            disp.stop()
        wall = time.time() - t0

    report = {
        "report_kind": "ducksir-project-run",
        "project": os.path.basename(args.project),
        "policy_id": pol["key"],
        "policy_manifest_digest": sha256(os.path.join(ROOT, cfg["manifest"])),
        "plan_digest": ir.sha256_of_blob(blob),
        "nodes": info.node_count,
        "steps": steps,
        "completed_steps": completed,
        "interrupted": bool(interrupted),
        "wall_seconds": round(wall, 3),
        "verdict": {
            "finite": not dead,
            "no_sink": bool(heights and min(heights) > -0.05),
            "ran_full": completed >= steps and not interrupted,
            "pass": (not dead) and bool(heights) and min(heights) > -0.05,
        },
        "panel": PANEL.build_panel(
            control_hz=control_hz, physics_hz=physics_hz,
            seconds=completed / control_hz, wall=wall, step_times=step_times,
            base_height_min=min(heights) if heights else 0.0,
            upright_final=tilts[-1] if tilts else 0.0,
            distance=float(np.hypot(d.qpos[0] - x0, d.qpos[1] - y0)),
        ),
    }
    print(f"项目 {report['project']}｜策略 {pol['key']}｜{completed}/{steps} 步"
          f"（{'中断' if interrupted else '跑完'}）")
    print(f"  基座 终 {heights[-1]:+.3f} 最低 {min(heights):+.3f} m｜直立 {tilts[-1]:+.3f}"
          if heights else "  没采到帧")
    print(f"  {report['panel']['realtime_factor']}× 实时｜p95 {report['panel']['control_p95_ms']} ms"
          f"｜超期 {report['panel']['deadline_miss']} 步")

    if args.report:
        out = os.path.join(ROOT, args.report)
        os.makedirs(os.path.dirname(out), exist_ok=True)
        with open(out, "w", encoding="utf-8") as f:
            json.dump(report, f, ensure_ascii=False, indent=2)
        print(f"  报告 → {args.report}")
    if args.trajectory:
        out = os.path.join(ROOT, args.trajectory)
        os.makedirs(os.path.dirname(out), exist_ok=True)
        with open(out, "w", encoding="utf-8") as f:
            json.dump({"meta": {"robot": doc["robot"]["id"], "joints": driven,
                                "control_hz": control_hz, "physics_hz": physics_hz,
                                "policy_id": pol["key"]}, "frames": frames}, f, ensure_ascii=False)
        print(f"  轨迹 → {args.trajectory}（{len(frames)} 帧）")

    core.ds_free(handle)
    return 130 if interrupted else (0 if report["verdict"]["pass"] else 1)


if __name__ == "__main__":
    raise SystemExit(main())
