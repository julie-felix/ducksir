#!/usr/bin/env python3
"""导出「一个可运行的策略包」：plan blob + 模型 + **契约**（contract.json）。

    PYTHONPATH=$HOME/.local/share/ducksir/pylibs python3 sim/host/export_artifacts.py \
        --manifest contract/policy/duck-walk.manifest.yaml --out client/viewer/data/live

为什么要有这个脚本（2026-11-10 的决定）：**契约是产物，不是消费端的常量**。
以前观测偏移、`fsm` 输出的 `[state, mode, command(13), clear]` 这些形状写在浏览器 JS 里，
改 Python 生成新 IR 之后端侧用旧偏移读新 blob——不报错，只是行为诡异。
现在编译时把这些形状写进 `contract.json`，端侧按名字取槽位、加载时校验维度与 digest。

产物（都不入库，除 contract.json）：
- `pre.blob` / `post.blob`：推理边界两段（ORT Web 是 async，核是 sync）
- `fsm.blob`：状态机（推理边界之前，整块在核里跑）
- `<policy>.onnx`：各策略权重
- `contract.json`：**唯一真源**——plan 形状 + 具名槽位 + 策略清单 + 观测配方 + 频率 + 机器人元数据
"""

from __future__ import annotations

import argparse
import json
import os
import shutil
import sys

import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
sys.path.insert(0, os.path.join(ROOT, "sim", "plan"))
sys.path.insert(0, os.path.join(ROOT, "tools", "robot"))
sys.path.insert(0, HERE)

import contract  # noqa: E402
import entity as E  # noqa: E402
import export_browser_fsm as FSM  # noqa: E402
import ir  # noqa: E402
import run_policy as RP  # noqa: E402

# 策略清单：mode 与 botlab `_policy_mode()` 一致
POLICIES = [
    ("walking", 0), ("sitstand", 1), ("kick-left", 2), ("kick-right", 3),
    ("roll", 4), ("ground-pick", 5), ("stand", 6),
]

# 观测配方：**从 manifest 的 obs 段派生**（不手写偏移，manifest 才是声明源）。
# 每种 slot 名映射到「端侧从哪取」；偏移由累加得出。
SOURCE_BY_SLOT = {
    "imu_ang_vel": lambda o: "sensor:imu_ang_vel",
    "projected_gravity": lambda o: f"gravity:body:{o.get('target', 'trunk_base')}",
    "joint_pos": lambda o: "qpos:joints",
    "joint_vel": lambda o: "qvel:joints",
    "last_action": lambda o: "last_action",
    "command": lambda o: "fsm_slot:command",
    # 以下与 botlab 的 Robot.body_*/contact/base_height 对位：
    #   manifest 里写 {slot: "body_pos", target: "trunk", len: 3}
    "body_pos": lambda o: f"body:pos:{o.get('target', 'trunk')}",
    "body_quat": lambda o: f"body:quat:{o.get('target', 'trunk')}",
    "body_vel": lambda o: f"body:vel:{o.get('target', 'trunk')}",
    "body_angvel": lambda o: f"body:angvel:{o.get('target', 'trunk')}",
    "contact": lambda o: f"contact:{o.get('target', 'trunk')}",
    "base_height": lambda o: "base:z",
    "motion_ref": lambda o: "motion_ref",
    # 速度指令 3 维（vx, vy, yaw_rate）——loco / AMP / MJAMP 的观测里有它
    "velocity_command": lambda o: "command:velocity",
    "mimic_uni": lambda o: "mimic:uni",
    "mimic_instinct": lambda o: "mimic:instinct",
    "mimic_gmtrack": lambda o: "mimic:gmtrack",
    "mimic_parkour": lambda o: "mimic:parkour",
}


def obs_recipe(man: dict) -> list[dict]:
    """把 manifest 的 obs 段翻成端侧配方：`[{name, slot:[off,len], source, minus?}]`。"""
    out, off = [], 0
    for o in man["obs"]:
        slot = o["slot"]
        if slot not in SOURCE_BY_SLOT:
            raise SystemExit(f"obs 段里有不认识的 slot：{slot}")
        item = {"name": slot, "slot": [off, o["len"]], "source": SOURCE_BY_SLOT[slot](o)}
        if o.get("minus"):
            item["minus"] = o["minus"]
        if o.get("order"):
            item["order"] = o["order"]
        if o.get("frame"):
            item["frame"] = o["frame"]
        out.append(item)
        off += o["len"]
    return out


def build_pre(man: dict) -> tuple[bytes, dict]:
    """观测 → 模型输入。

    默认只做拼接。若 manifest 声明了 `obs_history`（WBC 那种 4 帧历史），则：

        ref   = obs[offset_before]            （长度 offset）
        state = obs[offset, offset+length)    （长度 length）
        hist  = history(state, frames)        （frames × length）
        out   = concat(ref, hist)             （offset + frames*length）

    例（G1 WBC）：ref 67 + state 93×4 = 439。
    """
    total = sum(o["len"] for o in man["obs"])
    hist_cfg = man.get("obs_history")
    # 历史展开会改变输出维度：out = offset + frames × length
    out_dim = total
    if hist_cfg:
        out_dim = int(hist_cfg["offset"]) + int(hist_cfg["frames"]) * int(hist_cfg["length"])
    p = ir.Plan(input_dim=total, output_dim=out_dim, name="pre")
    if hist_cfg:
        h_off = int(hist_cfg["offset"])
        h_len = int(hist_cfg["length"])
        h_frames = int(hist_cfg["frames"])
        ref = p.obs(0, h_off) if h_off else None
        state = p.obs(h_off, h_len)
        hist = p.history(state, frames=h_frames)
        out = hist if ref is None else p.concat(ref, hist)
        p.out_action(out)
        return ir.compile_blob(p.to_json()), {"in:obs": (0, total),
                                              "out:model_in": (0, h_off + h_frames * h_len)}

    sl, off = [], 0
    for o in man["obs"]:
        sl.append(p.obs(off, o["len"]))
        off += o["len"]
    level = list(sl)
    while len(level) > 4:
        nxt = []
        for i in range(0, len(level), 4):
            chunk = level[i:i + 4]
            nxt.append(chunk[0] if len(chunk) == 1 else p.concat(*chunk))
        level = nxt
    p.out_action(level[0] if len(level) == 1 else p.concat(*level))
    return ir.compile_blob(p.to_json()), {"in:obs": (0, total)}


def build_post(preset: dict, n: int, lo, hi) -> tuple[bytes, dict]:
    """动作 → ctrl（扭矩模式下是目标角）。

    策略输出是**策略序**，而 default / scale / ctrlrange 都是**电机序**。
    两者不同时（G1 的 dof_mapping 就不是恒等），必须先 reorder 再 affine——
    否则默认角与缩放会张冠李戴，看起来能跑但姿态全错。
    """
    p = ir.Plan(input_dim=n, output_dim=n, name="post")
    raw = p.obs(0, n)
    action = raw
    # 策略序 → 电机序：**motorJointOrder** 才是那份置换（botlab JSON 里 policyJointOrder 是恒等，
    # 真实映射来自参考实现的 dof_mapping）。用错的话动作会张冠李戴。
    order = preset.get("motorJointOrder") or preset.get("policyJointOrder")
    if order and list(order) != list(range(n)):
        # `reorder` 是 gather（out[i] = x[perm[i]]），而这里要的是 scatter
        # （out[motor_order[i]] = action[i]），所以传**逆置换**。
        inv = [0] * n
        for i, m in enumerate(order):
            inv[int(m)] = i
        action = p.reorder(raw, inv)
    s = p.affine(action, preset["actionScale"], preset["defaultJointPositions"])
    p.out_action(p.clip(s, lo, hi))
    return ir.compile_blob(p.to_json()), {"in:action": (0, n), "out:ctrl": (0, n)}


def build_fsm() -> tuple[bytes, dict]:
    doc = FSM.build()
    blob = ir.compile_blob(doc)
    slots = {
        "in:policy_command": (0, 1), "in:gravity_z": (1, 1),
        "in:vx": (2, 1), "in:wz": (3, 1),
        "out:state": (0, 1), "out:mode": (1, 1),
        "out:command": (2, 13), "out:clear_last_action": (15, 1),
    }
    return blob, slots


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--manifest", default="contract/policy/duck-walk.manifest.yaml")
    ap.add_argument("--preset", default="build/preset/duck-hd1910.preset.yaml")
    ap.add_argument("--out", default="client/viewer/data/live")
    ap.add_argument("--entity", default="contract/robot/types/g1-29dof-torque.yaml",
                    help="实体 yaml（preset 的 generated.from 不是实体时用它）")
    ap.add_argument("--mjcf", help="物理模型（给了就把 robot.xml + 网格一起打进 bundle，浏览器要用）")
    args = ap.parse_args()

    with open(os.path.join(ROOT, args.preset), encoding="utf-8") as f:
        preset = yaml.safe_load(f)
    with open(os.path.join(ROOT, args.manifest), encoding="utf-8") as f:
        man = yaml.safe_load(f)
    # preset 有两个来源：① 从实体导出的（generated.from = 实体 yaml）
    # ② 从 botlab preset 转的（generated.from = botlab 的 yaml，这时要看 --entity）
    entity_path = preset["generated"]["from"]
    if not entity_path.endswith((".yaml", ".yml")) or "/contract/robot/types/" not in entity_path:
        entity_path = args.entity
    doc = E.read_yaml(os.path.join(ROOT, entity_path))
    driven = [j["name"] for j in doc["joints"] if j.get("actuated")]
    n = len(driven)
    RP.check_manifest(man, preset, n)
    actuators = doc.get("actuators") or []
    # 位置执行器（G1 botlab 模型）没有 ctrlrange，只有 forcerange；缺省就用 ±100 当动作夹紧
    lo = [float(a["ctrlrange"][0]) if a.get("ctrlrange") else -100.0 for a in actuators]
    hi = [float(a["ctrlrange"][1]) if a.get("ctrlrange") else 100.0 for a in actuators]

    out = os.path.join(ROOT, args.out)
    os.makedirs(out, exist_ok=True)

    pre_blob, pre_slots = build_pre(man)
    # action.order=motor 的策略（mjamp）输出已经是电机序，**不能**再做 reorder
    post_preset = preset
    if (man.get("action") or {}).get("order") == "motor":
        post_preset = dict(preset, motorJointOrder=list(range(n)))
    post_blob, post_slots = build_post(post_preset, n, lo, hi)
    fsm_blob, fsm_slots = build_fsm()
    for name, blob in (("pre", pre_blob), ("post", post_blob), ("fsm", fsm_blob)):
        with open(os.path.join(out, f"{name}.blob"), "wb") as f:
            f.write(blob)
    # preset 也要进 bundle：运行期靠它判断执行器模式（位置/扭矩）与 PD 增益
    shutil.copyfile(os.path.join(ROOT, args.preset), os.path.join(out, "preset.yaml"))

    # 动作参考（WBC/AMP）连同素材一起进 bundle：浏览器和宿主都按 `motion/` 相对路径读
    motion_cfg = preset.get("motion")
    if motion_cfg and motion_cfg.get("path"):
        src = motion_cfg["path"]
        if not os.path.isabs(src):
            src = os.path.join(ROOT, src)
        dst = os.path.join(out, "motion")
        if os.path.isdir(src):
            if os.path.isdir(dst):
                shutil.rmtree(dst)
            if motion_cfg.get("format") == "parkour":
                # 两段推理：编码器模型也要进 bundle
                enc = motion_cfg.get("encoder")
                if enc and os.path.isfile(enc):
                    shutil.copyfile(enc, os.path.join(bundle_dir, "parkour_enc.onnx"))
            elif motion_cfg.get("format") == "gmtrack":
                os.makedirs(dst, exist_ok=True)
                clip = motion_cfg.get("clip")
                shutil.copyfile(os.path.join(src, f"{clip}.npz"), os.path.join(dst, f"{clip}.npz"))
            elif motion_cfg.get("format") == "instinct":
                os.makedirs(dst, exist_ok=True)
                clip = motion_cfg.get("clip")
                shutil.copyfile(os.path.join(src, f"{clip}.npz"), os.path.join(dst, f"{clip}.npz"))
            elif motion_cfg.get("format") == "unisupermimic":
                # 只拷当前 clip 的 npz，别把整个动作库搬进 bundle
                os.makedirs(dst, exist_ok=True)
                clip = motion_cfg.get("clip")
                shutil.copyfile(os.path.join(src, f"{clip}.npz"), os.path.join(dst, f"{clip}.npz"))
            else:
                shutil.copytree(src, dst)
            motion_cfg = dict(motion_cfg, path="motion/")
            preset["motion"] = motion_cfg        # 契约与 bundle 内的 preset 都用相对路径
            print(f"  动作参考 {os.path.basename(src)} → motion/")

    # 策略清单：manifest 里声明了 `policies`（多模式）就用它，否则用鸭子那 7 个动作模型
    spec = man.get("policies") or [{"key": k, "mode": m, "file": f"build/models/{k}.onnx"}
                                   for k, m in POLICIES]
    policies = []
    for item in spec:
        src = os.path.join(ROOT, item["file"])
        if not os.path.exists(src):
            print(f"  跳过 {item['key']}（缺 {item['file']}）")
            continue
        dst_name = item.get("as") or os.path.basename(item["file"])
        shutil.copyfile(src, os.path.join(out, dst_name))
        policies.append({"key": item["key"], "mode": int(item["mode"]), "file": dst_name})
    if not policies:
        raise SystemExit("一个策略都没导出（manifest.policies / build/models 都缺）")

    doc_contract = contract.build_contract(
        plans={
            "pre": contract.plan_entry("pre", pre_blob, pre_slots),
            "post": contract.plan_entry("post", post_blob, post_slots),
            "fsm": contract.plan_entry("fsm", fsm_blob, fsm_slots),
        },
        policies=policies,
        obs=obs_recipe(man),
        rates={"control_hz": 50, "physics_hz": 200, "inference_decimation": 1},
        states={"count": len(FSM.STATE_NAMES) if hasattr(FSM, "STATE_NAMES") else 14,
                "initial": 0},
        robot={
            "id": doc["robot"]["id"],
            "joints": driven,
            "default_q": preset["defaultJointPositions"],
            "initial_qpos": preset["initialQpos"],
            # 有些上游 preset 带非零初始速度（GMTrack 的 spawn 就有一份），丢了会让第一步动力学偏
            "initial_qvel": preset.get("initialQvel"),
            "action_scale": preset["actionScale"],
            "ctrlrange": [[lo[i], hi[i]] for i in range(n)],
            # 速度指令范围：preset 显式给 None 表示「这个策略没有速度指令」；
            # 没写这个键才退回鸭子那组（MicroDuck 的旧默认）
            "commands": (preset["commands"] if "commands" in preset
                         else {"forward_max": 0.25, "back_max": -0.2, "yaw_max": 1.0}),
            # 执行器语义：position = MuJoCo 自己算 PD；torque = 消费者每个物理子步按 kp/kd 算
            "actuator_mode": preset.get("actuatorMode", "position"),
            "kp": preset.get("kp") or [],
            "kd": preset.get("kd") or [],
            "effort_limit": preset.get("effortLimit") or [],
            "motor_order": preset.get("motorJointOrder") or list(range(n)),
            # 动作参考（WBC / AMP 才用）：运行期按这些参数滚动播放头，算 67 维参考
            "motion": preset.get("motion") or None,
            "reference_joint_positions": preset.get("referenceJointPositions") or None,
        },
    )
    with open(os.path.join(out, "contract.json"), "w", encoding="utf-8") as f:
        json.dump(doc_contract, f, ensure_ascii=False, indent=2)

    print(f"导出 → {args.out}")
    for name, p in doc_contract["plans"].items():
        print(f"  {name:<5} {p['nodes']:>3} 节点 {p['input_dim']:>3} → {p['output_dim']:>3}"
              f"｜{p['bytes']} B｜digest {p['digest'][:12]}")
    # 物理模型 + 网格进 bundle：浏览器侧 MjVFS 需要（meshdir="."），宿主可以直接用同一份
    if args.mjcf:
        import re
        src = os.path.join(ROOT, args.mjcf)
        xml = open(src, encoding="utf-8").read()
        mesh_dir = re.search(r'meshdir="([^"]+)"', xml)
        if not mesh_dir:
            # meshdir 常写在被 include 的文件里（Parkour 的 scene 就是一层壳）
            for _inc in re.findall(r'<include file="([^"]+)"', xml):
                _p = os.path.join(os.path.dirname(src), _inc)
                if os.path.isfile(_p):
                    mesh_dir = re.search(r'meshdir="([^"]+)"', open(_p, encoding="utf-8").read())
                    if mesh_dir:
                        break
        assets_dir = os.path.join(os.path.dirname(src), mesh_dir.group(1)) if mesh_dir else os.path.dirname(src)
        xml = re.sub(r'meshdir="[^"]+"', 'meshdir="."', xml)
        # 相对上级目录的资源（Instinct 的 ../../terrain/*.obj，深度相机会看到它）一起搬进 bundle
        up_dirs = set()
        _done = set()
        for rel in re.findall(r'file="((?:\.\./)+[^"]+)"', xml):
            m2 = re.match(r'((?:\.\./)+)([^/]+)/', rel)
            if not m2:
                continue
            ups, folder = m2.group(1), m2.group(2)
            base = os.path.dirname(src)
            for _ in range(ups.count("..")):
                base = os.path.dirname(base)
            src_dir = os.path.join(base, folder)
            if not os.path.isdir(src_dir) or (src_dir, folder) in _done:
                continue
            _done.add((src_dir, folder))
            target = os.path.join(out, folder)
            if os.path.isdir(target):
                shutil.rmtree(target)
            shutil.copytree(src_dir, target)
            up_dirs.add(ups)
            print(f"  额外资源 {folder}/（被 MJCF 相对引用）")
        for ups in up_dirs:
            xml = xml.replace(ups, "")
        # 程序化楼梯：几何完全照 botlab worker 的生成器（mjw.js 的 hn()）：
        #   m += stepDepth; half_z = stepHeight*(e+1); z = half_z/2
        #   size = [stepDepth, stepWidth, half_z]（MJCF 是半长！）
        _scene = (man.get("scene") or {}) if isinstance(man, dict) else {}
        _stairs = _scene.get("stairs")
        if _stairs:
            import math as _math
            _pos = _stairs.get("position", [0, 0, 0]); _yaw = float(_stairs.get("yaw", 0) or 0)
            _d = float(_stairs["stepDepth"]); _h = float(_stairs["stepHeight"])
            _w = float(_stairs["stepWidth"]); _n = int(_stairs["stepCount"])
            _rows, _m = [], 0.0
            for _i in range(_n):
                _m += _d
                _hz = _h * (_i + 1)
                _c, _s = _math.cos(_yaw), _math.sin(_yaw)
                _cx = _pos[0] + _m * _c; _cy = _pos[1] + _m * _s
                _rows.append(
                    f'    <geom name="stair_{_i+1}" type="box" pos="{_cx:.6f} {_cy:.6f} {_pos[2] + _hz/2:.6f}"'
                    f' size="{_d} {_w} {_hz:.6f}" rgba="0.68 0.62 0.52 1"'
                    f' friction="1.5 0.005 0.0001" group="0" contype="1" conaffinity="1"/>')
            xml = xml.replace("</mujoco>", "  <worldbody>\n" + "\n".join(_rows) + "\n  </worldbody>\n</mujoco>")
            print(f"  楼梯 {_n} 级（照 worker 生成器）")
        # 上游 MJCF 没写 <actuator>（Parkour 的机器人就是这种，自动执行器由 preset 生成）：
        # 按契约的驱动关节顺序补一组 motor，ctrlrange 取 preset 的 effortLimit。
        if "<actuator" not in xml:
            _ent = E.read_yaml(os.path.join(ROOT, args.entity)) if getattr(args, "entity", None) else {}
            acts = doc_contract.get("actuators") or _ent.get("actuators") or []
            # 契约里执行器的关节名与力矩上限
            rows = []
            for i, a in enumerate(acts):
                jn = a.get("joint")
                lim = a.get("ctrlrange")
                if not jn:
                    continue
                lim = float(lim[1]) if lim else 100.0
                lim = abs(lim)
                if preset.get("actuatorMode") == "position":
                    kp = float(preset["kp"][i]); kd = float(preset["kd"][i])
                    rows.append(f'    <position joint="{jn}" kp="{kp}" kv="{kd}" '
                                f'ctrlrange="-3.14 3.14" forcerange="-{lim} {lim}"/>')
                else:
                    rows.append(f'    <motor joint="{jn}" ctrlrange="-{lim} {lim}" gear="1"/>')
            if rows:
                block = "  <actuator>\n" + "\n".join(rows) + "\n  </actuator>\n"
                xml = xml.replace("</mujoco>", block + "</mujoco>")
                print(f"  补执行器 {len(rows)} 个（motor）")
        with open(os.path.join(out, "robot.xml"), "w", encoding="utf-8") as f:
            f.write(xml)
        # 顶层 XML 可能只写了 <include file="..."/>（Parkour 的 scene 就是这样）：
        # 把被 include 的文件也复制进 bundle，并把它的网格一起收进来
        inc_meshes = []
        for inc in re.findall(r'<include file="([^"]+)"', xml):
            inc_src = os.path.join(os.path.dirname(src), inc)
            if not os.path.isfile(inc_src):
                continue
            inc_txt = open(inc_src, encoding="utf-8").read()
            # 被 include 的文件也要把 meshdir 改成 "."（网格都放在 bundle 根）
            inc_txt = re.sub(r'meshdir="[^"]+"', 'meshdir="."', inc_txt)
            with open(os.path.join(out, inc), "w", encoding="utf-8") as f:
                f.write(inc_txt)
            inc_meshes += re.findall(r'file="([^"]+\.(?:stl|STL|obj|OBJ))"', inc_txt)
        meshes = sorted(set(re.findall(r'file="([^"]+\.(?:stl|STL|obj|OBJ))"', xml)) | set(inc_meshes))
        mesh_out = out          # 网格直接放 bundle 根：meshdir="." 对宿主和浏览器都成立
        copied = 0
        for name in meshes:
            path = os.path.join(assets_dir, name)
            if os.path.exists(path):
                shutil.copyfile(path, os.path.join(mesh_out, name))
                copied += 1
        with open(os.path.join(out, "meshes.json"), "w", encoding="utf-8") as f:
            # 只列 bundle 根下的网格：子目录里的资源（如 terrain/*.obj）是给物理/深度用的，
            # 列进 meshes.json 会让浏览器去下载几百个大文件（实测 557 个 → 3 分钟还没跑起来）
            json.dump([n for n in meshes if "/" not in n and os.path.exists(os.path.join(mesh_out, n))], f)
        print(f"  物理模型 robot.xml + {copied} 个网格（bundle 根）")

    print(f"  策略 {len(policies)} 个｜契约 contract.json")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
