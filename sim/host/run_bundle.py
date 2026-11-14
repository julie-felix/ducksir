#!/usr/bin/env python3
"""跑一个 **bundle**（编译产物目录）——运行时不碰 Python 声明，只吃产物。

    python3 sim/host/run_bundle.py --bundle build/release/test --seconds 20 --forward 0.25
    python3 sim/host/run_bundle.py --bundle bundle/ --fsm --pulses 3:1,8:1 --report out.json

bundle 里有什么（`export_artifacts.py` 产出）：
- `contract.json`：计划的形状 + 具名槽位 + 策略清单 + 观测配方 + 频率 + 机器人元数据
- `pre.blob` / `post.blob` / `fsm.blob`
- `*.onnx`：各策略权重

执行顺序（与浏览器那条链**逐项相同**）：
1. `fsm.blob`（若有）：吃 [脉冲, 重力z, vx, wz] → 吐 [state, mode, command, clear]
2. 观测按契约配方装配（`contract_obs`，与 `play-core.mjs` 同一套 source 语义）
3. `pre.blob`（若有）→ 模型（按 mode 选会话）→ `post.blob` → ctrl
4. 物理 4 子步，循环

这条脚本存在的意义：把「宿主跑」从「跑 Python 声明」降级为「跑产物」，
于是宿主与浏览器真的只是两个 target，而不是两套实现。
"""

from __future__ import annotations

import argparse
import ctypes
import json
import threading
import time as _time_mod
import os
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
sys.path.insert(0, os.path.join(ROOT, "sim", "plan"))
sys.path.insert(0, os.path.join(ROOT, "sim", "host"))

import contract as C  # noqa: E402
import contract_obs as CO  # noqa: E402
import panel as PANEL  # noqa: E402
import ir  # noqa: E402
from run_policy import Info, load_libs  # noqa: E402


class Plan:
    """一个已加载的 plan：句柄 + 契约里的形状与槽位。"""

    def __init__(self, core, name: str, blob: bytes, entry: dict, err):
        self.name = name
        self.entry = entry
        self.handle = core.ds_load(blob, len(blob), err, 512)
        if not self.handle:
            raise SystemExit(f"加载 {name} 失败：" + err.value.decode())
        info = Info()
        core.ds_info_of(self.handle, ctypes.byref(info))
        if info.input_dim != entry["input_dim"] or info.output_dim != entry["output_dim"]:
            raise SystemExit(
                f"{name} 维度不符：blob {info.input_dim}→{info.output_dim}，"
                f"契约 {entry['input_dim']}→{entry['output_dim']}")

    def slot(self, name: str) -> tuple[int, int]:
        s = (self.entry.get("slots") or {}).get(name)
        if not s:
            raise SystemExit(f"plan {self.name} 没有槽位 {name}")
        return int(s[0]), int(s[1])


def _with_gains(preset: dict, n: int) -> dict:
    """把 preset 里可能缺的 kp/kd/effortLimit/actionScale/default 按 n 补齐。"""
    out = dict(preset)
    for key, default in (("kp", 0.0), ("kd", 0.0), ("effortLimit", 0.0),
                         ("actionScale", 1.0), ("defaultJointPositions", 0.0)):
        v = out.get(key) or []
        if len(v) != n:
            out[key] = [default] * n
    return out


def run_plan(core, plan: Plan, values) -> np.ndarray:
    n_in = plan.entry["input_dim"]
    n_out = plan.entry["output_dim"]
    bi = (ctypes.c_float * n_in)(*[float(v) for v in values])
    bo = (ctypes.c_float * n_out)()
    rc = core.ds_step(plan.handle, bi, bo, ctypes.create_string_buffer(512))
    if rc != 0:
        raise SystemExit(f"ds_step({plan.name}) 返回 {rc}")
    return np.array(bo[:], dtype=np.float32)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--bundle", required=True, help="bundle 目录（含 contract.json 与 blob/模型）")
    ap.add_argument("--mjcf", help="物理模型（默认 bundle/duck.xml，没有就用 build/mjcf/duck-hd1910-all.xml）")
    ap.add_argument("--seconds", type=float, default=20.0)
    ap.add_argument("--steps", type=int)
    ap.add_argument("--pace", action="store_true",
                    help="按实时速度跑（直播用；否则宿主会几十倍速跑完）")
    ap.add_argument("--control", action="store_true",
                    help="从 stdin 读 JSONL 指令（{\"forward\":0.3,\"yaw\":0,\"paused\":true}），直播时用")
    ap.add_argument("--stdio", action="store_true",
                    help="把场景和每帧位姿按 JSONL 打到 stdout（插件宿主用它转 SSE）")
    ap.add_argument("--stream", type=int, metavar="PORT",
                    help="把每帧位姿推给浏览器（0 = 随机端口）：ws://127.0.0.1:PORT/")
    ap.add_argument("--forward", type=float, default=0.0)
    ap.add_argument("--yaw", type=float, default=0.0)
    ap.add_argument("--fsm", action="store_true", help="用 fsm.blob 驱动（否则单策略常值指令）")
    ap.add_argument("--pulses", default="", help="FSM 指令脉冲，格式 秒:值,秒:值")
    ap.add_argument("--report")
    ap.add_argument("--trajectory")
    ap.add_argument("--trace", help="逐步落 JSONL（obs/action/ctrl/qpos），用于和 botlab 对拍")
    args = ap.parse_args()

    import mujoco
    import onnxruntime as ort_py   # 模型在 Python 侧跑（与浏览器 ORT Web 对位）

    bdir = os.path.join(ROOT, args.bundle) if not os.path.isabs(args.bundle) else args.bundle
    con = C.load(os.path.join(bdir, "contract.json"))
    # preset 决定执行器语义：位置（MuJoCo 算 PD）还是扭矩（我们在控制步算 PD）
    preset_path = os.path.join(bdir, "preset.yaml")
    preset = {}
    if os.path.exists(preset_path):
        import yaml as _yaml
        preset = _yaml.safe_load(open(preset_path, encoding="utf-8")) or {}
    torque_mode = str(preset.get("actuatorMode") or "").lower() == "torque"
    # 动作参考（WBC / AMP）：契约里有 robot.motion 才启用；播放头 + 参考目标角
    # 只有 WBC(3)/AMP(4) 用动作参考；loco(2)/mjamp(5) 不读参考段，也不该把目标基准换掉
    use_reference = any(int(p.get("mode", 0)) in (3, 4) for p in (con.get("policies") or []))
    _motion_all = (con.get("robot") or {}).get("motion")
    # UniSuperMimic：配方是单条目 mimic:uni，不走 reference，但要读动作
    _is_uni = bool(_motion_all) and _motion_all.get("format") in ("unisupermimic", "instinct")
    motion_cfg = _motion_all if (use_reference or _is_uni) else None
    ref_angles = (con.get("robot") or {}).get("reference_joint_positions")
    motion_arrays = motion_meta = playhead = None
    uni_obs_mod = uni_motion = uni_hist = None
    inst_obs_mod = inst_motion = inst_hist = inst_cfg = inst_cam = None
    if motion_cfg and motion_cfg.get("format") == "instinct":
        import numpy as _np
        import instinct_obs as _IN
        import depth_camera as _DC
        _DC_DepthCamera = _DC.DepthCamera
        _mp = motion_cfg["path"]
        if not os.path.isabs(_mp):
            _mp = os.path.join(bdir, _mp)
        _clip = motion_cfg["clip"]
        z = _np.load(os.path.join(_mp, f"{_clip}.npz"), allow_pickle=True)
        inst_motion = {k: z[k] for k in ("joint_pos", "joint_vel", "body_pos_w", "body_quat_w")}
        inst_motion["fps"] = float(z["fps"][0])
        _dof = [str(x) for x in z["dof_names"]]
        inst_cfg = dict(motion_cfg["config"])
        inst_cfg["joint_order"] = [_dof.index(n) for n in inst_cfg["joint_names"]]
        inst_cfg["torso_index"] = [str(b) for b in z["body_names"]].index("torso_link")
        inst_obs_mod = _IN
        inst_hist = _IN.TermHistory()
        # 模型要到后面才建（这里只解析动作），相机延迟到循环里创建
    elif motion_cfg and motion_cfg.get("format") == "unisupermimic":
        import numpy as _np
        import unisupermimic_obs as _U
        _mp = motion_cfg["path"]
        if not os.path.isabs(_mp):
            _mp = os.path.join(bdir, _mp)      # bundle 内相对路径（导出时写 motion/）
        z = _np.load(os.path.join(_mp, f"{motion_cfg['clip']}.npz"), allow_pickle=True)
        uni_motion = {k: z[k] for k in ("joint_pos", "joint_vel", "body_pos_w", "body_quat_w",
                                        "body_lin_vel_w", "body_ang_vel_w")}
        uni_motion["fps"] = float(z["fps"][0])
        uni_obs_mod = _U
        uni_hist = _U.History()
    elif motion_cfg and motion_cfg.get("path"):
        import sys as _sys
        _sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "plan"))
        import motion as _M
        import wbc_obs as _W
        base = motion_cfg["path"]
        if not os.path.isabs(base):
            base = os.path.join(bdir, base)   # bundle 里的相对路径（导出时写成 motion/）
        meta_path = os.path.join(base, "metadata.json")
        names = (json.load(open(meta_path, encoding="utf-8")).get("body_names")
                 if os.path.exists(meta_path) else None)
        motion_arrays = {
            "joint_pos": _M.load_motion(base, "joint_pos")["data"],
            "joint_vel": _M.load_motion(base, "joint_vel")["data"],
            "body_pos": _M.load_motion(base, "body_pos_w", indices=names)["data"].reshape(-1, len(names), 3),
            "body_quat": _M.load_motion(base, "body_quat_w", indices=names)["data"].reshape(-1, len(names), 4),
        }
        motion_meta = motion_cfg
        playhead = _W.Playhead(motion_arrays["joint_pos"].shape[0],
                               start=0, frame_interval=int(motion_cfg.get("frame_interval", 5)))
        playhead.pause(True)   # 进 WBC 之前参考停在起点（botlab: active_mode=3）

    mjcf = args.mjcf or (os.path.join(bdir, "duck.xml") if os.path.exists(os.path.join(bdir, "duck.xml"))
                         else "build/mjcf/duck-hd1910-all.xml")
    mjcf = mjcf if os.path.isabs(mjcf) else os.path.join(ROOT, mjcf)

    core, _ort_c = load_libs()   # 核用 native .so；模型走 onnxruntime Python 绑定
    err = ctypes.create_string_buffer(512)
    plans = {}
    for name, entry in con["plans"].items():
        path = os.path.join(bdir, entry.get("file") or f"{name}.blob")
        with open(path, "rb") as f:
            plans[name] = Plan(core, name, f.read(), entry, err)

    policies = con["policies"]
    if not policies:
        raise SystemExit("契约里没有策略")
    mode_to_key = {int(p["mode"]): p["key"] for p in policies}
    sessions = {}
    rnn_state = {}      # 多输入模型（loco 的 h/c）跨步保持
    for p in policies:
        path = os.path.join(bdir, p["file"])
        if not os.path.exists(path):
            raise SystemExit(f"缺模型 {path}")
        sessions[p["key"]] = ort_py.InferenceSession(path, providers=["CPUExecutionProvider"])

    robot = con["robot"]
    rates = con["rates"]
    control_hz = float(rates["control_hz"])
    physics_hz = float(rates["physics_hz"])
    decim = int(round(physics_hz / control_hz))
    steps = int(args.seconds * control_hz)
    if args.steps:
        steps = min(steps, args.steps)

    spec = mujoco.MjSpec.from_file(mjcf)
    geom = spec.worldbody.add_geom()
    geom.type = mujoco.mjtGeom.mjGEOM_PLANE
    geom.size = [6.0, 6.0, 0.1]
    geom.condim = 3
    m = spec.compile()
    d = mujoco.MjData(m)
    m.opt.timestep = 1.0 / physics_hz
    d.qpos[:] = robot["initial_qpos"]
    mujoco.mj_forward(m, d)

    stream = None
    stdio = bool(args.stdio)
    _HS = None
    if stdio or args.stream is not None:
        import host_stream as _HS
    if stdio:
        sys.stdout.write(_HS.build_scene_b64(m, mujoco) + "\n")
        sys.stdout.flush()
        print("  直播（stdio）：场景已发", file=sys.stderr)
    if args.stream is not None:
        import host_stream as _HS
        stream = _HS.HostStream(args.stream)
        _scene, _blob = _HS.build_scene(m, mujoco)
        stream.set_scene(json.dumps(_scene, separators=(",", ":")), _blob)
        stream.send(json.dumps(_scene, separators=(",", ":")), _blob)
        print(f"  直播 ws://127.0.0.1:{stream.port}/（{len(_scene['geoms'])} 个视觉 geom）")

    driven = robot["joints"]
    n = len(driven)
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

    def body_of(name: str):
        """body 位姿/速度 + 接触数——给 contract_obs 的 body:*/contact:* 用。

        语义对齐 botlab 的 Robot.body_position/body_quaternion/body_linear_velocity/
        body_angular_velocity/contact：位置四元数取世界系，速度取 cvel（前 3 角速度、后 3 线速度）。
        """
        bid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, name)
        if bid < 0:
            raise SystemExit(f"MJCF 里没有 body {name}")
        lo = int(m.body_dofadr[bid]) if m.body_dofnum[bid] else 0
        contacts = 0
        for c in range(d.ncon):
            con = d.contact[c]
            b1 = int(m.geom_bodyid[con.geom1])
            b2 = int(m.geom_bodyid[con.geom2])
            if bid in (b1, b2):
                contacts += 1
        return {"xpos": d.xpos[bid], "xquat": d.xquat[bid],
                "cvel": d.cvel[bid], "contacts": contacts, "dofadr": lo}

    view = {"data": d, "qpos_off": qpos_off, "dof_off": dof_off,
            "sensor_adr": sensor_adr, "quat_of": quat_of, "body": body_of}

    pulses = {}
    for part in (args.pulses.split(",") if args.pulses else []):
        sec, val = part.split(":")
        pulses[round(float(sec) * control_hz)] = float(val)

    fsm = plans.get("fsm") if args.fsm else None
    if args.fsm and fsm is None:
        raise SystemExit("这个 bundle 里没有 fsm.blob")

    last_action = np.zeros(n, dtype=np.float32)
    cmd = np.zeros(13, dtype=np.float32)
    cmd[0], cmd[2] = args.forward, args.yaw
    act = np.zeros(n, dtype=np.float32)
    heights, tilts, step_times, frames = [], [], [], []
    x0, y0 = float(d.qpos[0]), float(d.qpos[1])
    path_len, speed, prev_xy, speed_hist = 0.0, 0.0, (x0, y0), []
    states, pulse = [], 0.0
    t0 = time.time()
    dead = False
    trace_fh = open(args.trace, "w", encoding="utf-8") if args.trace else None
    mode_key = policies[0]["key"]

    # 运行期指令（直播用）：一次性读 stdin，主循环取最新的
    control = {"forward": float(args.forward), "yaw": float(args.yaw), "paused": False}
    if args.control:
        def _read_control():
            for raw in sys.stdin:
                raw = raw.strip()
                if not raw:
                    continue
                try:
                    upd = json.loads(raw)
                except json.JSONDecodeError:
                    continue
                for key in ("forward", "yaw", "paused"):
                    if key in upd:
                        control[key] = upd[key]
                cmd[0], cmd[2] = float(control["forward"]), float(control["yaw"])
        threading.Thread(target=_read_control, daemon=True).start()

    _t_start = _time_mod.time() if args.pace else 0.0
    sim_k = 0            # 真正步进过的步数（暂停时不动，直播时间轴才不会继续走）
    k = 0                # 循环计数（含暂停期间的轮询）
    while sim_k < steps:
        k = sim_k
        if args.control:
            if control.get("paused"):
                # 暂停：不步进、时间轴不动，但继续推当前位姿（否则画面冻住看不出是暂停）
                if stdio:
                    try:
                        sys.stdout.write(_HS.pose_message(sim_k, sim_k / control_hz, d.xpos, d.xquat) + "\n")
                        sys.stdout.flush()
                    except BrokenPipeError:
                        stdio = False
                _time_mod.sleep(0.02)
                continue
        if k in pulses:
            pulse = pulses[k]
        t_step = time.perf_counter()

        if fsm is not None:
            buf = np.zeros(fsm.entry["input_dim"], dtype=np.float32)
            for slot_name, val in (("in:policy_command", pulse), ("in:gravity_z", -(1 - 2 * (d.qpos[4] ** 2 + d.qpos[5] ** 2))),
                                   ("in:vx", cmd[0]), ("in:wz", cmd[2])):
                off, _ = fsm.slot(slot_name)
                buf[off] = val
            out = run_plan(core, fsm, buf)
            pulse = 0.0
            off_s, _ = fsm.slot("out:state")
            off_m, _ = fsm.slot("out:mode")
            off_c, len_c = fsm.slot("out:command")
            off_clr, _ = fsm.slot("out:clear_last_action")
            states.append(int(out[off_s]))
            mode_key = mode_to_key.get(int(out[off_m]), mode_key)
            cmd = out[off_c:off_c + len_c].copy()
            if out[off_clr]:
                last_action = np.zeros(n, dtype=np.float32)

        motion_ref = None
        in_active_mode = False
        if playhead is not None:
            import wbc_obs as _W
            active = int(motion_meta.get("active_mode", 3))
            in_active_mode = (fsm is None) or (int(out[off_m]) == active)
            # 起步先让参考停几帧（机器人从初始位形settle），再到点开播——
            # 直接开播会让策略第 0 帧就吃到参考速度尖峰（实测会倒）。
            settle = int(os.environ.get("DUCK_WBC_SETTLE", "10"))
            if in_active_mode and playhead.paused and k >= settle:
                playhead.pause(False)
            if not in_active_mode:
                playhead.pause(True)
            idx, last = playhead.current()
            motion_ref = _W.reference_block(
                joint_pos=motion_arrays["joint_pos"][idx],
                joint_vel=motion_arrays["joint_vel"][idx],
                anchor_pos=motion_arrays["body_pos"][idx][0],
                anchor_quat=motion_arrays["body_quat"][idx][0],
                last_anchor_pos=motion_arrays["body_pos"][last][0],
                last_anchor_quat=motion_arrays["body_quat"][last][0],
                base_quat=d.qpos[3:7], pause=playhead.paused)
        mimic_obs = None
        if inst_obs_mod is not None:
            if inst_cam is None:
                inst_cam = _DC_DepthCamera(m, motion_cfg["camera"])
            _bid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "torso_link")
            _w, _x, _y, _z = d.xquat[_bid]
            _R = np.array([
                [1 - 2 * (_y * _y + _z * _z), 2 * (_x * _y - _z * _w), 2 * (_x * _z + _y * _w)],
                [2 * (_x * _y + _z * _w), 1 - 2 * (_x * _x + _z * _z), 2 * (_y * _z - _x * _w)],
                [2 * (_x * _z - _y * _w), 2 * (_y * _z + _x * _w), 1 - 2 * (_x * _x + _y * _y)]])
            _q = np.array([d.qpos[i] for i in qpos_off], dtype=np.float64)
            _qd = np.array([d.qvel[i] for i in dof_off], dtype=np.float64)
            _block, _terms = inst_obs_mod.build_terms(
                motion=inst_motion, t=k * 0.02, duration=float(inst_cfg["duration"]),
                joint_order=inst_cfg["joint_order"],
                config_defaults=np.asarray(inst_cfg["default_angles"]),
                robot_joint_pos=_q - np.asarray(preset["defaultJointPositions"]),
                robot_joint_vel=_qd, gravity_b=_R.T @ np.array([0, 0, -1.0]),
                base_ang_vel=d.qvel[3:6], last_action=last_action,
                torso_index=inst_cfg["torso_index"], robot_torso_quat=d.xquat[_bid].copy())
            _hist990 = inst_hist.push(_terms)
            _depth = inst_cam.process(inst_cam.render(d)).reshape(-1)
            mimic_obs = np.concatenate([_block, _depth, _hist990]).astype(np.float32)
        if uni_obs_mod is not None:
            q_now = np.array([d.qpos[i] for i in qpos_off], dtype=np.float64)
            qd_now = np.array([d.qvel[i] for i in dof_off], dtype=np.float64)
            w, xq, yq, zq = d.qpos[3:7]
            Rm = np.array([
                [1 - 2 * (yq * yq + zq * zq), 2 * (xq * yq - zq * w), 2 * (xq * zq + yq * w)],
                [2 * (xq * yq + zq * w), 1 - 2 * (xq * xq + zq * zq), 2 * (yq * zq - xq * w)],
                [2 * (xq * zq - yq * w), 2 * (yq * zq + xq * w), 1 - 2 * (xq * xq + yq * yq)]])
            per_frame = uni_obs_mod.build_obs(
                motion=uni_motion, joint_pos=q_now, joint_vel=qd_now,
                base_ang_vel=d.qvel[3:6], gravity_b=Rm.T @ np.array([0, 0, -1.0]),
                last_action=last_action, qpos_quat=d.qpos[3:7],
                default_angles=np.asarray(preset["defaultJointPositions"]),
                t=(k - 1) * float(motion_cfg.get("dt", 0.02)),
                joint_order=preset.get("policyJointOrder"))
            hist990 = uni_hist.push(per_frame[:99])
            mimic_obs = np.concatenate([hist990, per_frame[99:]]).astype(np.float32)
        obs = CO.make_obs(con, view, last_action=last_action, command=cmd,
                          motion_ref=motion_ref, mimic_obs=mimic_obs)
        obs_raw = obs.copy()
        if "pre" in plans:
            obs = run_plan(core, plans["pre"], obs)
        sess = sessions[mode_key]
        # 多输入模型（loco 的 h_in/c_in）要跨步保持递归状态；其余输入喂 0 初始化
        in_names = [i.name for i in sess.get_inputs()]
        feed = {in_names[0]: obs.reshape(1, -1).astype(np.float32)}
        if len(in_names) > 1:
            st = rnn_state.setdefault(mode_key, {})
            for name, meta_in in zip(in_names[1:], sess.get_inputs()[1:]):
                if name not in st:
                    shape = [d if isinstance(d, int) else 1 for d in meta_in.shape]
                    st[name] = np.zeros(shape, dtype=np.float32)
                feed[name] = st[name]
        out_names = [o.name for o in sess.get_outputs()]
        results = sess.run(None, feed)
        raw = np.asarray(results[0]).reshape(-1).astype(np.float32)
        if len(out_names) > 1:
            # 递归状态按**位置**回填：out[1] → in[1]（h_out → h_in），
            # 按名字会错位（h_out 存进去、下一步却读 h_in → 永远吃全零）。
            st = rnn_state.setdefault(mode_key, {})
            for in_name, value in zip(in_names[1:], results[1:]):
                st[in_name] = np.asarray(value, dtype=np.float32)
        # botlab 的控制器对动作做 ±100 截断（`policy_bank(...).clip(-100, 100)`），
        # 少了这一步，扭矩模式会把超界的动作直接放大成力矩。
        raw = np.clip(raw, -100.0, 100.0)
        last_action = raw
        if "post" in plans:
            act = run_plan(core, plans["post"], raw)
        else:
            act = raw
        if playhead is not None and in_active_mode:
            playhead.advance()
        if playhead is not None and ref_angles is not None and in_active_mode:
            # WBC 的目标基准是 referenceJointPositions（不是 default），
            # post 已按 default 算好，这里补上差值即可（缩放不变）
            act = np.asarray(act, dtype=np.float64) + (np.asarray(ref_angles, np.float64)
                                                       - np.asarray(preset.get("defaultJointPositions"), np.float64))
        if trace_fh is not None:
            trace_fh.write(json.dumps({
                "step": sim_k, "t": sim_k / control_hz,
                "obs": [round(float(x), 6) for x in obs],
                "action": [round(float(x), 6) for x in raw],
                "ctrl": [round(float(x), 6) for x in act],
                "qpos": [round(float(x), 6) for x in d.qpos],
                "qvel": [round(float(x), 6) for x in d.qvel],
            }) + "\n")
            trace_fh.flush()
        if torque_mode:
            # **每个物理子步都重算 PD**（不是每个控制步算一次）。
            # 实测：控制步算一次只能撑 20 步，子步重算能撑 104 步——
            # 这也正是 MuJoCo 位置执行器的语义（它每个子步按 kp/kv 算力矩）。
            import actuators as _A
            gains = _with_gains(preset, len(driven))
            # 注意：`act` 已经是 **目标角**（post 里做过 default + scale×action），
            # 不能再走 ctrl_from_action（那会把 action 当原始动作再缩放一次）。
            target_deg = np.asarray(act, dtype=np.float64)
            for _ in range(decim):
                q_now = np.array([d.qpos[i] for i in qpos_off], dtype=np.float64)
                qd_now = np.array([d.qvel[i] for i in dof_off], dtype=np.float64)
                tau = _A.torque_from_target(target_deg, q_now, qd_now,
                                            kp=gains["kp"], kd=gains["kd"],
                                            effort_limit=gains["effortLimit"] or None)
                d.ctrl[:] = tau
                mujoco.mj_step(m, d)
        else:
            d.ctrl[:] = act
            for _ in range(decim):
                mujoco.mj_step(m, d)
        if (stream is not None or stdio) and k % max(1, int(round(control_hz / 30))) == 0:
            _msg = _HS.pose_message(sim_k, sim_k / control_hz, d.xpos, d.xquat)
            if stream is not None:
                stream.send(_msg)
            if stdio:
                try:
                    sys.stdout.write(_msg + "\n")
                    sys.stdout.flush()
                except BrokenPipeError:
                    stdio = False       # 页面关了就别再写，也别打一堆 traceback
        if args.pace:
            _due = _t_start + (k + 1) / control_hz
            _slack = _due - _time_mod.time()
            if _slack > 0:
                _time_mod.sleep(min(_slack, 0.5))
        # 参考动作放完就结束这一轮：botlab 那边是 end_after_steps 直接停；
        # 我们如果继续跑，策略失去参考会在一两秒内倒（Instinct case_08 约 10.45 s）。
        if inst_cfg is not None:
            _dur = float(inst_cfg.get("duration") or 0.0)
            if _dur and sim_k * float(motion_cfg.get("dt", 0.02)) >= _dur:
                print(f"  参考动作结束（{_dur:.2f} s），本轮到此为止")
                break
        sim_k += 1
        if not np.isfinite(d.qpos).all():
            dead = True
            break

        step_times.append(time.perf_counter() - t_step)
        heights.append(float(d.qpos[2]))
        tilts.append(float(1 - 2 * (d.qpos[4] ** 2 + d.qpos[5] ** 2)))
        cur_xy = (float(d.qpos[0]), float(d.qpos[1]))
        seg = float(np.hypot(cur_xy[0] - prev_xy[0], cur_xy[1] - prev_xy[1]))
        path_len += seg
        speed_hist.append(seg * control_hz)
        if len(speed_hist) > 10:
            speed_hist.pop(0)
        speed = sum(speed_hist) / len(speed_hist)
        prev_xy = cur_xy
        if args.trajectory:
            frames.append({"t": round(k / control_hz, 4),
                           "pos": [round(float(x), 6) for x in d.qpos[0:3]],
                           "quat": [round(float(x), 6) for x in d.qpos[3:7]],
                           "q": [round(float(d.qpos[i]), 6) for i in qpos_off]})

    wall = time.time() - t0
    completed = len(heights)
    report = {
        "report_kind": "ducksir-bundle-run",
        "bundle": os.path.basename(bdir),
        "plans": {k2: {"nodes": v.entry["nodes"], "digest": v.entry["digest"][:16]} for k2, v in plans.items()},
        "policy": mode_key if fsm is not None else policies[0]["key"],
        "steps": steps, "completed_steps": completed, "dead": dead,
        "state_transitions": [f"{i / control_hz:.2f}s→{s}" for i, s in
                              enumerate(states) if i and s != states[i - 1]][:12],
        "verdict": {"finite": not dead,
                    "no_sink": bool(heights and min(heights) > -0.05),
                    "ran_full": completed >= steps,
                    "pass": (not dead) and bool(heights) and min(heights) > -0.05},
        "panel": PANEL.build_panel(
            control_hz=control_hz, physics_hz=physics_hz,
            seconds=completed / control_hz, wall=wall, step_times=step_times,
            base_height_min=min(heights) if heights else 0.0,
            upright_final=tilts[-1] if tilts else 0.0,
            distance=float(np.hypot(d.qpos[0] - x0, d.qpos[1] - y0))),
    }
    print(f"bundle {report['bundle']}｜{completed}/{steps} 步｜{report['panel']['realtime_factor']}× 实时"
          f"｜p95 {report['panel']['control_p95_ms']} ms")
    if states:
        print("  状态迁移：" + ("，".join(report["state_transitions"]) or "（无）"))
    print(f"  基座 终 {heights[-1]:+.3f} 最低 {min(heights):+.3f} m｜直立 {tilts[-1]:+.3f}"
          if heights else "  没采到帧")
    print(f"  指令 forward {cmd[0]:+.2f} → 实际速度 {speed:.3f} m/s｜路程 {path_len:.2f} m")

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
            json.dump({"meta": {"joints": driven, "control_hz": control_hz,
                                "physics_hz": physics_hz, "policy": report["policy"]},
                       "frames": frames}, f, ensure_ascii=False)
        print(f"  轨迹 → {args.trajectory}（{len(frames)} 帧）")

    for p in plans.values():
        core.ds_free(p.handle)
    if stream is not None or stdio:
        # 跑完了也别立刻断：页面还在看最后一帧。Ctrl-C 或超时再关。
        import time as _time
        hold = float(os.environ.get("DUCK_STREAM_HOLD", "180"))
        print(f"  直播保持中 {hold:.0f}s（关掉页面或 Ctrl-C 即可结束）")
        try:
            _time.sleep(hold)
        except KeyboardInterrupt:
            pass

    return 0 if report["verdict"]["pass"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
