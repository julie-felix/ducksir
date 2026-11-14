#!/usr/bin/env python3
"""宿主侧的观测装配器——和浏览器 `play-core.mjs` 的 `makeObs()` 同一套配方。

    import contract_obs
    c = contract_obs.load_contract("client/viewer/data/live/contract.json")
    obs = contract_obs.make_obs(c, view, last_action=act)
    off, length = contract_obs.slot(c, "fsm", "out:command")

以前宿主脚本把 `obs[0..3]`、`obs[6..20]` 这些偏移写死在代码里，浏览器也是各写一份——
2026-11-10 的教训是：改 Python 生成新 IR 之后，两边都各自漂。现在两边都读同一份契约。

配方条目 `{name, slot, source}` 支持：
`sensor:<名>`、`gravity:body:<名>`、`qpos:joints`、`qvel:joints`、`last_action`、`fsm_slot:<槽位名>`、
`motion_ref`（动作参考 67 维：关节位置/速度 + 锚点 6D，由调用方按播放头算好传进来）。
"""

from __future__ import annotations

import json
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "plan"))

import contract as C  # noqa: E402


def load_contract(path: str) -> dict:
    return C.load(path)


def slot(doc: dict, plan: str, name: str) -> tuple[int, int]:
    """取某个 plan 的具名槽位（名字形如 `out:command`）。"""
    p = (doc.get("plans") or {}).get(plan)
    if not p:
        raise SystemExit(f"契约里没有 plan {plan}")
    s = (p.get("slots") or {}).get(name)
    if not s:
        raise SystemExit(f"plan {plan} 没有槽位 {name}")
    return int(s[0]), int(s[1])


def obs_dim(doc: dict) -> int:
    return max(off + length for off, length in
               ((o["slot"][0], o["slot"][1]) for o in doc["obs"]))


def quat_to_mat(q):
    w, x, y, z = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


BODY_SOURCES = ("body:pos:", "body:quat:", "body:vel:", "body:angvel:")


def _body(name: str, view: dict):
    return view["body"](name) if callable(view.get("body")) else None


def make_obs(doc: dict, view: dict, *, last_action=None, command=None, vx: float = 0.0,
             motion_ref=None, mimic_obs=None) -> np.ndarray:
    """按契约配方组 61 维观测。

    `view` 需要提供：`data`（MuJoCo 数据）、`qpos_off`/`dof_off`（驱动序地址）、
    `sensor_adr`（按传感器名索引的函数）、`quat_of`（按 body 名取四元数）。
    """
    robot = doc["robot"]
    data = view["data"]
    obs = np.zeros(obs_dim(doc), dtype=np.float32)
    for item in doc["obs"]:
        off, length = item["slot"]
        src = item.get("source", "")
        if src.startswith("sensor:"):
            adr = view["sensor_adr"](src.split(":", 1)[1])
            obs[off:off + length] = data.sensordata[adr:adr + length]
        elif src.startswith("gravity:body:"):
            q = view["quat_of"](src.split(":", 2)[2])
            obs[off:off + 3] = quat_to_mat(q).T @ np.array([0.0, 0.0, -1.0])
        elif src == "qpos:joints":
            q = np.array([data.qpos[i] for i in view["qpos_off"]])
            minus = item.get("minus")
            if minus == "default":
                q = q - np.array(robot["default_q"])
            elif minus == "reference":
                # WBC/AMP：策略看到的关节量是「相对参考站姿」（referenceJointPositions）
                ref = robot.get("reference_joint_positions")
                if not ref:
                    raise SystemExit("配方用了 minus=reference，但契约里没有 reference_joint_positions")
                q = q - np.array(ref)
            if item.get("order") == "policy":
                # 策略训练时用的是策略序（G1 的 dof_mapping 不是恒等），按 motor_order 取
                q = np.array([q[int(m)] if int(m) < len(q) else 0.0
                              for m in (robot.get("motor_order") or range(len(q)))])
            obs[off:off + length] = q
        elif src == "qvel:joints":
            v = np.array([data.qvel[i] for i in view["dof_off"]])
            if item.get("order") == "policy":
                v = np.array([v[int(m)] if int(m) < len(v) else 0.0
                              for m in (robot.get("motor_order") or range(len(v)))])
            obs[off:off + length] = v
        elif src == "last_action":
            if last_action is not None:
                obs[off:off + length] = last_action
        elif src == "mimic:gmtrack":
            if mimic_obs is None:
                raise SystemExit("契约里有 mimic:gmtrack 槽位，但调用方没给 mimic_obs 数组")
            arr = np.asarray(mimic_obs, dtype=np.float32)
            if arr.shape != (length,):
                raise SystemExit(f"mimic_obs 长度 {arr.shape} != 槽位 {length}")
            obs[off:off + length] = arr
        elif src == "mimic:instinct":
            if mimic_obs is None:
                raise SystemExit("契约里有 mimic:instinct 槽位，但调用方没给 mimic_obs 数组")
            arr = np.asarray(mimic_obs, dtype=np.float32)
            if arr.shape != (length,):
                raise SystemExit(f"mimic_obs 长度 {arr.shape} != 槽位 {length}")
            obs[off:off + length] = arr
        elif src == "mimic:uni":
            # UniSuperMimic 的 1720 维观测（99×历史 + 10×未来目标）由运行器用
            # sim/host/unisupermimic_obs.py 算好传进来
            if mimic_obs is None:
                raise SystemExit("契约里有 mimic:uni 槽位，但调用方没给 mimic_obs 数组")
            arr = np.asarray(mimic_obs, dtype=np.float32)
            if arr.shape != (length,):
                raise SystemExit(f"mimic_obs 长度 {arr.shape} != 槽位 {length}")
            obs[off:off + length] = arr
        elif src == "command:velocity":
            c = command if command is not None else np.zeros(13)
            obs[off:off + 3] = np.asarray(c[:3], dtype=np.float32)
        elif src == "motion_ref":
            # 动作参考（WBC/AMP）：67 维，由 run_bundle 用 wbc_obs 按播放头算好传进来
            if motion_ref is None:
                raise SystemExit("契约里有 motion_ref 槽位，但调用方没给 motion_ref 数组")
            ref = np.asarray(motion_ref, dtype=np.float32)
            if ref.shape != (length,):
                raise SystemExit(f"motion_ref 长度 {ref.shape} != 槽位 {length}")
            obs[off:off + length] = ref
        elif src.startswith("body:"):
            # botlab 的 Robot.body_*：body 位姿/速度，全部世界系
            parts = src.split(":")
            kind, name = parts[1], parts[2]
            b = view["body"](name)
            if kind == "pos":
                obs[off:off + 3] = b["xpos"]
            elif kind == "quat":
                obs[off:off + 4] = b["xquat"]
            elif kind == "vel":
                obs[off:off + 3] = b["cvel"][3:6]
            elif kind == "angvel":
                # botlab 的 Robot.body_angular_velocity(body, frame="local") 是**机体系**角速度；
                # MuJoCo 的 cvel 是世界系，所以这里乘 R^T 转过去。
                w = np.asarray(b["cvel"][0:3], dtype=float)
                mode = item.get("frame", "local")
                obs[off:off + 3] = (quat_to_mat(b["xquat"]).T @ w) if mode == "local" else w
            else:
                raise SystemExit(f"不认识的 body 观测：{src}")
        elif src.startswith("contact:"):
            # contact:<body>：该 body 有任一接触 → 1，否则 0
            b = view["body"](src.split(":", 1)[1])
            obs[off:off + 1] = 1.0 if b["contacts"] > 0 else 0.0
        elif src == "base:z":
            obs[off:off + 1] = float(data.qpos[2])
        elif src.startswith("fsm_slot:"):
            if command is None:
                raise SystemExit(f"观测配方引用 {src}，但调用方没给 command")
            obs[off:off + length] = command
        else:
            raise SystemExit(f"观测配方里有不认识的 source：{src}")
    return obs
