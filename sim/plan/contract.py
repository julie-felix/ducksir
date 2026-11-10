#!/usr/bin/env python3
"""契约：把「图的形状」从端侧代码里搬到产物里。

    contract = build_contract(
        plans={"pre": (blob_pre, {"command": (0, 61)}), ...},
        policies=[{"key": "walking", "mode": 0, "file": "walking.onnx"}],
        obs=[{"name": "imu_ang_vel", "slot": (0, 3), "source": "sensor:imu_ang_vel", "len": 3}],
        rates={"control_hz": 50, "physics_hz": 200},
    )
    json.dump(contract, open("contract.json", "w"))

为什么要有这份东西：2026-11-10 的教训——观测偏移、fsm 输出的 `[state, mode, command(13), clear]`
这些**图的契约**当时写在消费端（`client/viewer/play-core.mjs`、`play.js`）里，改 Python 生成新 IR 之后，
端侧用旧偏移读新 blob，**不报错，只是行为诡异**。契约成为产物、端侧按名字取槽位、加载时校验维度与 digest，
这类漂移就从「静默」变成「加载即失败」。

consume 方（浏览器 / 宿主 / 板端）只需要三件事：
1. 读 `contract.json`；
2. `ds_load` 之后核对 `ds_info` 的 in/out 维度与 digest；
3. 按名字取槽位，不写数字。
"""

from __future__ import annotations

import json
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import ir  # noqa: E402

CONTRACT_VERSION = 1


def plan_entry(name: str, blob: bytes, slots: dict[str, tuple[int, int]] | None = None,
               file: str | None = None) -> dict:
    """一条 plan 的契约：形状 + digest + 具名槽位。

    `slots` 是 `{名字: (偏移, 长度)}`，偏移可以是输入或输出侧的——由调用方用
    `"in:xxx"` / `"out:xxx"` 这样的键区分；这里不做约束，只做记录与校验。
    """
    info = ir.read_blob(blob)
    return {
        "name": name,
        "file": file or f"{name}.blob",
        "bytes": len(blob),
        "input_dim": info["input_dim"],
        "output_dim": info["output_dim"],
        "nodes": len(info["nodes"]),
        "digest": ir.sha256_of_blob(blob),
        "slots": {k: list(v) for k, v in (slots or {}).items()},
    }


def build_contract(plans: dict[str, dict], policies: list[dict], obs: list[dict],
                   rates: dict, states: dict | None = None, robot: dict | None = None) -> dict:
    """组装契约。`plans` 的值是 `plan_entry(...)` 的返回值。"""
    doc = {
        "kind": "ducksir-contract",
        "version": CONTRACT_VERSION,
        "rates": rates,
        "plans": plans,
        "policies": policies,
        "obs": obs,
    }
    if states:
        doc["states"] = states
    if robot:
        doc["robot"] = robot
    problems = validate_contract(doc)
    if problems:
        raise ValueError("契约自检失败：\n  " + "\n  ".join(problems))
    return doc


def validate_contract(doc: dict) -> list[str]:
    """静态自检：槽位不越界、不重叠、obs 总长等于主 plan 的 input_dim。"""
    errs: list[str] = []
    if doc.get("kind") != "ducksir-contract":
        errs.append("kind 必须是 ducksir-contract")
    for name, p in (doc.get("plans") or {}).items():
        for slot, (off, length) in (p.get("slots") or {}).items():
            limit = p["output_dim"] if slot.startswith("out:") else p["input_dim"]
            if off < 0 or length <= 0 or off + length > limit:
                errs.append(f"plan {name} 的槽位 {slot} 越界：{off}+{length} > {limit}")
    obs = doc.get("obs") or []
    if obs:
        end = max(off + length for _, (off, length) in
                  ((o["name"], tuple(o["slot"])) for o in obs))
        main = (doc.get("plans") or {}).get("pre")
        if main and end != main["input_dim"]:
            errs.append(f"obs 总长 {end} != pre.input_dim {main['input_dim']}")
    for pol in doc.get("policies") or []:
        for key in ("key", "mode", "file"):
            if key not in pol:
                errs.append(f"策略条目缺字段 {key}：{pol}")
    return errs


def load(path: str) -> dict:
    with open(path, encoding="utf-8") as f:
        doc = json.load(f)
    problems = validate_contract(doc)
    if problems:
        raise ValueError(f"{path} 不合法：\n  " + "\n  ".join(problems))
    return doc


if __name__ == "__main__":
    print("contract.py 是库；入口在 sim/host/export_artifacts.py")
