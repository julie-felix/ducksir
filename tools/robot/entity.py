"""实体（数字描述）的公共工具。

只依赖标准库 + PyYAML：本机没有 pip，也不要为了几个换算装一堆包。

约定（见 docs/plans/2026-11-09-robot-entity.md）：
- 长度 m、质量 kg、角度 rad、时间 s、力 N、力矩 N*m、惯量 kg*m^2
- 四元数一律 wxyz（MuJoCo 的顺序），欧拉角一律 rpy（绕固定轴 X→Y→Z）
- 每个数值都要能在 provenance 里找到「凭什么算数」
"""

from __future__ import annotations

import copy
import datetime
import hashlib
import math
import os
from typing import Any

import yaml

SCHEMA_ID = "ducksir/robot/v0"

SI_UNITS = {
    "length": "m",
    "mass": "kg",
    "angle": "rad",
    "time": "s",
    "force": "N",
    "torque": "N*m",
    "inertia": "kg*m^2",
    "angular_velocity": "rad/s",
    "velocity": "m/s",
    "stiffness": "N*m/rad",
    "damping": "N*m*s/rad",
}
ALLOWED_UNITS = set(SI_UNITS.values())

# 谁压谁：measured > identified > spec > cad > mjcf > guessed
SOURCE_RANK = {
    "measured": 6,
    "identified": 5,
    "spec": 4,
    "cad": 3,
    "mjcf": 2,
    "urdf": 2,   # 与 mjcf 同级：都是模型文件，不是原始测量
    "guessed": 1,
}
SOURCES = tuple(SOURCE_RANK)

# warnings 的 code 有意稳定，报告里按 code 汇总。
CODES = {
    "W001": "源文件里就没有这个字段（格式本身装不下）",
    "W002": "字段在源文件的另一个文件里（不在被导入的文件里）",
    "W003": "两个源对同一个值说法不同",
    "W004": "mesh 未入库，只有引用",
    "W005": "源里有多套建模变体，只导入了一套",
    "W006": "闭合/并联机构被降级表达",
}


def sha256_file(path: str) -> str:
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def today() -> str:
    return datetime.datetime.now(datetime.timezone.utc).strftime("%Y-%m-%d")


def warn(warnings: list, code: str, path: str, message: str, **extra) -> None:
    entry = {"code": code, "path": path, "message": message}
    entry.update(extra)
    warnings.append(entry)


def frames(root: str, world_attachment: str = "floating") -> dict:
    return {
        "root": root,
        "world_attachment": world_attachment,
        "axes": {"x": "forward", "y": "left", "z": "up"},
        "units": dict(SI_UNITS),
        "quat_order": "wxyz",
        "euler_order": "rpy",
        "angular_velocity_frame": "body",
    }


def prov(path: str, value: Any, unit: str, source: str, ref: str | None = None,
         date: str | None = None, note: str | None = None) -> dict:
    if source not in SOURCE_RANK:
        raise ValueError(f"未知 source: {source}")
    unit = SI_UNITS.get(unit, unit)  # 允许传维度名（mass）或符号（kg）
    if unit not in ALLOWED_UNITS:
        raise ValueError(f"单位不在 SI 白名单: {unit}")
    entry = {
        "path": path,
        "value": copy.deepcopy(value),
        "unit": unit,
        "source": source,
        "date": date or today(),
    }
    if ref:
        entry["ref"] = ref
    if note:
        entry["note"] = note
    return entry


def make_entity(*, entity_id: str, name: str, type_id: str, source_ref: str,
                license_id: str, generator: dict, root: str, world_attachment: str) -> dict:
    return {
        "schema": SCHEMA_ID,
        "robot": {
            "id": entity_id,
            "name": name,
            "type": type_id,
            "source_ref": source_ref,
            "license": license_id,
            "generator": generator,
            "created_at": today(),
        },
        "frames": frames(root, world_attachment),
        "links": [],
        "joints": [],
        "actuators": [],
        "mechanisms": [],
        "sensors": [],
        "bus": {"note": "未定：SDK 的统一 interface 还没拿到，字段等那份规范"},
        "presets": {},
        "provenance": [],
        "warnings": [],
    }


def attach_meshes(doc: dict, glb_manifest: str) -> None:
    """把视觉 mesh 换成本地 GLB，并补 sha256（碰撞仍指向上游原件）。"""
    import json
    with open(glb_manifest, encoding="utf-8") as f:
        manifest = json.load(f)
    assets_dir = doc["robot"].get("assets_dir") or "assets"
    hit = 0
    for link in doc["links"]:
        for item in link["visual"]:
            name = item.get("mesh_name")
            if name and name in manifest:
                item["mesh"] = os.path.join(assets_dir, name + ".glb")
                item["sha256"] = manifest[name]["sha256"]
                item["bytes"] = manifest[name]["bytes"]
                item["vendored"] = True
                hit += 1
    doc["robot"]["mesh_manifest"] = glb_manifest
    doc["summary"]["visual_meshes_vendored"] = hit


def write_yaml(path: str, doc: dict) -> None:
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w", encoding="utf-8") as f:
        yaml.safe_dump(doc, f, allow_unicode=True, sort_keys=False,
                       default_flow_style=False, width=110)


def read_yaml(path: str) -> dict:
    with open(path, encoding="utf-8") as f:
        return yaml.safe_load(f)


# ---------- 姿态换算（只用标准库） ----------

def rpy_to_quat(r: float, p: float, y: float) -> list[float]:
    """固定轴 X→Y→Z（URDF 的 rpy）→ wxyz 四元数。"""
    cr, sr = math.cos(r / 2), math.sin(r / 2)
    cp, sp = math.cos(p / 2), math.sin(p / 2)
    cy, sy = math.cos(y / 2), math.sin(y / 2)
    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    return [round(w, 12), round(x, 12), round(y, 12), round(z, 12)]


def quat_to_rpy(q: list[float]) -> list[float]:
    w, x, y, z = q
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    r = math.atan2(sinr_cosp, cosr_cosp)
    sinp = 2 * (w * y - z * x)
    p = math.copysign(math.pi / 2, sinp) if abs(sinp) >= 1 else math.asin(sinp)
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yw = math.atan2(siny_cosp, cosy_cosp)
    return [round(r, 12), round(p, 12), round(yw, 12)]


def _mat_mul(A, B):
    return [[sum(A[i][k] * B[k][j] for k in range(3)) for j in range(3)] for i in range(3)]


def _rot(rpy) -> list[list[float]]:
    r, p, y = rpy
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    Rx = [[1, 0, 0], [0, cr, -sr], [0, sr, cr]]
    Ry = [[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]]
    Rz = [[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]]
    return _mat_mul(Rz, _mat_mul(Ry, Rx))


def quat_to_mat(q) -> list[list[float]]:
    w, x, y, z = q
    n = math.sqrt(w * w + x * x + y * y + z * z) or 1.0
    w, x, y, z = w / n, x / n, y / n, z / n
    return [[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]]


def rotate_inertia_quat(diag, quat) -> dict:
    """主轴惯量 diaginertia + 姿态 quat → link 系张量。"""
    R = quat_to_mat(quat)
    I = [[diag[0], 0, 0], [0, diag[1], 0], [0, 0, diag[2]]]
    Rt = [[R[j][i] for j in range(3)] for i in range(3)]
    out = _mat_mul(R, _mat_mul(I, Rt))
    return {"ixx": round(out[0][0], 15), "ixy": round(out[0][1], 15), "ixz": round(out[0][2], 15),
            "iyy": round(out[1][1], 15), "iyz": round(out[1][2], 15), "izz": round(out[2][2], 15)}


def rotate_inertia(comp: dict, rpy) -> dict:
    """把惯量张量从 <inertial><origin rpy> 的坐标系转到 link 系：I_link = R · I · Rᵀ。

    URDF 允许 inertial origin 带旋转，而惯量是写在那个旋转后的坐标系里的；
    不转这一步，带 90° 安装角的 link 惯量主轴就是错的（G1 的 waist_roll 等）。
    """
    I = [[comp.get("ixx", 0.0), comp.get("ixy", 0.0), comp.get("ixz", 0.0)],
         [comp.get("ixy", 0.0), comp.get("iyy", 0.0), comp.get("iyz", 0.0)],
         [comp.get("ixz", 0.0), comp.get("iyz", 0.0), comp.get("izz", 0.0)]]
    if not any(rpy):
        return comp
    R = _rot(rpy)
    Rt = [[R[j][i] for j in range(3)] for i in range(3)]
    out = _mat_mul(R, _mat_mul(I, Rt))
    return {"ixx": round(out[0][0], 15), "ixy": round(out[0][1], 15), "ixz": round(out[0][2], 15),
            "iyy": round(out[1][1], 15), "iyz": round(out[1][2], 15), "izz": round(out[2][2], 15)}


def vec3(text: str | None, default=(0.0, 0.0, 0.0)) -> list[float]:
    if not text:
        return list(default)
    return [round(float(v), 12) for v in text.split()]


def numbers(text: str) -> list[float]:
    return [float(v) for v in text.split()]
