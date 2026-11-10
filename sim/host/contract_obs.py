#!/usr/bin/env python3
"""宿主侧的观测装配器——和浏览器 `play-core.mjs` 的 `makeObs()` 同一套配方。

    import contract_obs
    c = contract_obs.load_contract("client/viewer/data/live/contract.json")
    obs = contract_obs.make_obs(c, view, last_action=act)
    off, length = contract_obs.slot(c, "fsm", "out:command")

以前宿主脚本把 `obs[0..3]`、`obs[6..20]` 这些偏移写死在代码里，浏览器也是各写一份——
2026-11-10 的教训是：改 Python 生成新 IR 之后，两边都各自漂。现在两边都读同一份契约。

配方条目 `{name, slot, source}` 支持：
`sensor:<名>`、`gravity:body:<名>`、`qpos:joints`、`qvel:joints`、`last_action`、`fsm_slot:<槽位名>`。
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


def make_obs(doc: dict, view: dict, *, last_action=None, command=None, vx: float = 0.0) -> np.ndarray:
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
            if item.get("minus") == "default":
                q = q - np.array(robot["default_q"])
            obs[off:off + length] = q
        elif src == "qvel:joints":
            obs[off:off + length] = np.array([data.qvel[i] for i in view["dof_off"]])
        elif src == "last_action":
            if last_action is not None:
                obs[off:off + length] = last_action
        elif src.startswith("fsm_slot:"):
            if command is None:
                raise SystemExit(f"观测配方引用 {src}，但调用方没给 command")
            obs[off:off + length] = command
        else:
            raise SystemExit(f"观测配方里有不认识的 source：{src}")
    return obs
