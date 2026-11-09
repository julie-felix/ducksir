#!/usr/bin/env python3
"""两份实体比对（同一台机器的两条路）。

    # 鸭子：原 MJCF → 实体 → 导出 MJCF → 再导入 → 比
    python3 tools/robot/compare_entities.py A.yaml B.yaml

    # G1：URDF 一条路 vs MJCF 一条路
    python3 tools/robot/compare_entities.py types/g1-29dof.yaml /tmp/g1-from-mjcf.yaml

比的是「物理上不可协商的东西」：质量、质心、惯量、拓扑、轴、限位、增益。
不比：命名风格、mesh 路径、class 展开方式、被动件的组织方式——那些是表达差异。
"""

from __future__ import annotations

import argparse
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import entity as E  # noqa: E402

TOL = {"mass": 1e-6, "len": 1e-6, "inertia": 1e-3, "gain": 1e-6, "limit": 1e-5}


def _fro_rel(a: dict, b: dict) -> float:
    keys = ("ixx", "ixy", "ixz", "iyy", "iyz", "izz")
    num = sum((a.get(k, 0.0) - b.get(k, 0.0)) ** 2 for k in keys) ** 0.5
    den = max(sum(a.get(k, 0.0) ** 2 for k in keys) ** 0.5,
              sum(b.get(k, 0.0) ** 2 for k in keys) ** 0.5, 1e-12)
    return num / den


def close(a, b, tol) -> bool:
    if a is None or b is None:
        return a == b
    if isinstance(a, dict) and isinstance(b, dict) and "ixx" in a and "ixx" in b:
        return _fro_rel(a, b) <= tol
    if isinstance(a, (int, float)) and isinstance(b, (int, float)):
        return abs(a - b) <= tol * max(1.0, abs(a), abs(b))
    if isinstance(a, list) and isinstance(b, list) and len(a) == len(b):
        return all(close(x, y, tol) for x, y in zip(a, b))
    if isinstance(a, dict) and isinstance(b, dict):
        return a.keys() == b.keys() and all(close(a[k], b[k], tol) for k in a)
    return a == b


def diff(label: str, a, b, tol, out: list) -> None:
    if not close(a, b, tol):
        out.append(f"{label}: {a!r} vs {b!r}")


def compare(A: dict, B: dict, *, allow_fixed_missing: bool = False) -> list[str]:
    out: list[str] = []
    for key in ("root", "world_attachment"):
        diff(f"frames.{key}", A["frames"].get(key), B["frames"].get(key), TOL["len"], out)

    la, lb = {l["name"]: l for l in A["links"]}, {l["name"]: l for l in B["links"]}
    out.append(f"#links 只有 A: {sorted(set(la) - set(lb))}；只有 B: {sorted(set(lb) - set(la))}")
    for name in sorted(set(la) & set(lb)):
        a, b = la[name], lb[name]
        diff(f"links[{name}].mass", a["mass"], b["mass"], TOL["mass"], out)
        diff(f"links[{name}].com", a["com"], b["com"], TOL["len"], out)
        diff(f"links[{name}].inertia", a["inertia"], b["inertia"], TOL["inertia"], out)

    ja, jb = {j["name"]: j for j in A["joints"]}, {j["name"]: j for j in B["joints"]}
    only_a, only_b = set(ja) - set(jb), set(jb) - set(ja)
    if allow_fixed_missing:
        only_a = {n for n in only_a if (ja[n].get("type") != "fixed") and not n.startswith("floating")}
        only_b = {n for n in only_b if (jb[n].get("type") != "fixed") and not n.startswith("floating")}
    out.append(f"#joints 只有 A: {sorted(only_a)}；只有 B: {sorted(only_b)}")
    for name in sorted(set(ja) & set(jb)):
        a, b = ja[name], jb[name]
        for k in ("parent", "child", "type"):
            diff(f"joints[{name}].{k}", a.get(k), b.get(k), TOL["len"], out)
        diff(f"joints[{name}].axis.dir", a["axis"]["dir"], b["axis"]["dir"], TOL["len"], out)
        for side in ("lower", "upper"):
            av = (a["limit"].get("mech") or {}).get(side)
            bv = (b["limit"].get("mech") or {}).get(side)
            diff(f"joints[{name}].limit.{side}", av, bv, TOL["limit"], out)

    aa, ab = {x["name"]: x for x in A["actuators"]}, {x["name"]: x for x in B["actuators"]}
    out.append(f"#actuators 只有 A: {sorted(set(aa) - set(ab))}；只有 B: {sorted(set(ab) - set(aa))}")
    for name in sorted(set(aa) & set(ab)):
        for k in ("kp", "kv", "forcerange", "ctrlrange", "damping", "armature"):
            diff(f"actuators[{name}].{k}", aa[name].get(k), ab[name].get(k), TOL["gain"], out)
    return out


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("a")
    ap.add_argument("b")
    ap.add_argument("--allow-fixed-missing", action="store_true")
    args = ap.parse_args()
    A, B = E.read_yaml(args.a), E.read_yaml(args.b)
    lines = compare(A, B, allow_fixed_missing=args.allow_fixed_missing)
    hard = [x for x in lines if not x.startswith("#")]
    print(f"== {args.a}\n   vs {args.b}")
    for line in lines:
        print("  " + line)
    print(f"\n不一致 {len(hard)} 项")
    return 0 if not hard else 1


if __name__ == "__main__":
    raise SystemExit(main())
