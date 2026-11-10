#!/usr/bin/env python3
"""canonical → botlab 形状的 `<robot>.preset.yaml`（13 个字段）。

    PYTHONPATH=$HOME/.local/share/ducksir/pylibs \
    python3 tools/robot/export_preset.py contract/robot/types/duck-hd1910.yaml \
        --mjcf build/mjcf/duck-hd1910.xml -o build/preset/duck-hd1910.preset.yaml

为什么是这 13 个字段：botlab 的 preset 代码实际只读这些（见 `docs/plans/2026-11-10-botlab-replica.md` D13）。
我们的 canonical 是唯一真源，这个文件是**派生物**，不手改、入库要带 generated 标记。

字段来源：
- `actionDimension` / `kp` / `kd` / `effortLimit` / `*JointOrder` ← canonical 的 `actuators`
- `jointQposOffset` / `jointQvelOffset` ← 导出 MJCF 的真实布局（有 mujoco 时直接读，否则按 free=7/6 + 每关节 1 算）
- `defaultJointPositions` / `initialQpos` / `actionScale` / `imuExtrinsic` ← **目前没有上游依据**，
  一律取「零位 + 单位量」并写进 `assumptions`，等训练侧标定/调参后再改。宁可显式假设，不编造厂商数据。
"""

from __future__ import annotations

import argparse
import hashlib
import os
import sys

import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import entity as E  # noqa: E402

FREE_QPOS = 7
FREE_QVEL = 6


def sha256(path: str) -> str:
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def driven_joints(doc: dict) -> list[dict]:
    """驱动关节，按 canonical 里的声明顺序（这就是 motorJointOrder 的基准序）。"""
    return [j for j in doc["joints"] if j.get("actuated")]


def offsets_from_mjcf(path: str, names: list[str]):
    import mujoco

    m = mujoco.MjModel.from_xml_path(path)
    qpos, qvel = [], []
    for n in names:
        jid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, n)
        if jid < 0:
            raise SystemExit(f"MJCF 里没有关节 {n}")
        qpos.append(int(m.jnt_qposadr[jid]))
        qvel.append(int(m.jnt_dofadr[jid]))
    return qpos, qvel, m


def offsets_computed(names: list[str], floating: bool):
    qpos = [FREE_QPOS if floating else 0]
    qvel = [FREE_QVEL if floating else 0]
    return qpos, qvel


def build(entity_path: str, mjcf_path: str | None) -> dict:
    doc = E.read_yaml(entity_path)
    joints = driven_joints(doc)
    acts = doc.get("actuators") or []
    names = [j["name"] for j in joints]

    if acts:
        act_joints = [a["joint"] for a in acts]
        if sorted(act_joints) != sorted(names):
            missing = sorted(set(names) - set(act_joints))
            extra = sorted(set(act_joints) - set(names))
            raise SystemExit(f"actuators 与驱动关节不是双射：缺 {missing} 多 {extra}")

    floating = doc["frames"].get("world_attachment") == "floating"
    if mjcf_path and os.path.exists(mjcf_path):
        try:
            qpos, qvel, _ = offsets_from_mjcf(mjcf_path, names)
            layout_src = f"mjcf:{os.path.basename(mjcf_path)}"
        except ImportError:
            qpos, qvel = offsets_computed(names, floating)
            layout_src = "computed(no mujoco)"
    else:
        qpos, qvel = offsets_computed(names, floating)
        layout_src = "computed(no mjcf)"

    # ctrl 顺序 = actuators 的声明顺序；motorJointOrder 给的是「第 i 个电机对应第几个驱动关节」
    index_of = {n: i for i, n in enumerate(names)}
    order = [index_of[a["joint"]] for a in acts] if acts else list(range(len(names)))

    forker = [abs(a["forcerange"][0]) if a.get("forcerange") else None for a in acts]
    preset = {
        "kind": "botlab-preset",
        "generated": {
            "by": "tools/robot/export_preset.py",
            "from": os.path.relpath(entity_path),
            "entity_sha256": sha256(entity_path),
            "mjcf": os.path.relpath(mjcf_path) if mjcf_path else None,
            "mjcf_sha256": sha256(mjcf_path) if mjcf_path and os.path.exists(mjcf_path) else None,
            "layout": layout_src,
        },
        "actionDimension": len(names),
        "jointQposOffset": qpos,
        "jointQvelOffset": qvel,
        "actuatorJointOrder": order,
        "motorJointOrder": order,
        "defaultJointPositions": [0.0] * len(names),
        "initialQpos": [0.0] * (7 if floating else 0) + [0.0] * len(names),
        "kp": [a.get("kp") for a in acts],
        "kd": [a.get("kv") for a in acts],
        "effortLimit": forker,
        "actionScale": [1.0] * len(names),
        "imuExtrinsic": [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0],
        "autoActuators": False,
        "assumptions": {
            "defaultJointPositions": "全 0：上游 MJCF 没有 keyframe/home，训练侧也未给初始位形",
            "initialQpos": "基座位姿取原点 + 单位四元数；实际站立高度由标定/训练侧给",
            "actionScale": "全 1：策略输出直接当关节增量，缩放由训练侧确定",
            "imuExtrinsic": "单位变换：IMU 安装位置未标定",
        },
    }
    if not acts:
        preset["assumptions"]["kp/kd/effortLimit"] = "canonical 里没有 actuators（URDF 不带执行器参数，见 importer W002）"

    warnings = []
    if any(v is None for v in preset["kp"]) or any(v is None for v in preset["kd"]):
        warnings.append("kp/kd 有 None：训练侧未定，填之前不能跑 PD 保持")
    if any(v is None for v in preset["effortLimit"]):
        warnings.append("effortLimit 有 None：forcerange 缺失")
    return {"preset": preset, "warnings": warnings}


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("entity")
    ap.add_argument("-o", "--output", required=True)
    ap.add_argument("--mjcf", help="导出的 MJCF（用来取真实的 qpos/qvel 偏移）")
    args = ap.parse_args()

    out = build(args.entity, args.mjcf)
    os.makedirs(os.path.dirname(os.path.abspath(args.output)), exist_ok=True)
    with open(args.output, "w", encoding="utf-8") as f:
        f.write(f"# 自动生成，不要手改：{args.entity} → preset\n")
        yaml.safe_dump(out["preset"], f, allow_unicode=True, sort_keys=False)
    print(f"写入 {args.output}（actionDimension={out['preset']['actionDimension']}）")
    for w in out["warnings"]:
        print(f"  W: {w}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
