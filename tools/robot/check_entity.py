#!/usr/bin/env python3
"""实体的读时检查（validator 只做这五条，别的不做）。

    python3 tools/robot/check_entity.py contract/robot/types/*.yaml

C1 树      ：一个根、parent 指得到、无环、关节两端都在
C2 名字    ：link/joint/actuator 各自唯一；actuator.joint 必须是一个 actuated 关节
C3 单位    ：frames.units 必须是 SI 白名单全集；provenance 每条 unit 在白名单里
C4 出处    ：同一 path 两个不同值 → 失败（除非显式 override: true）
C5 preset  ：preset 不得改变 actuated 关节集合

退出码：0 全过；1 有 FAIL。
"""

from __future__ import annotations

import argparse
import glob
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import entity as E  # noqa: E402


def resolve(doc: dict, path: str):
    """把 'links[knee].mass' / 'actuators[*].kt' / 'summary.total_mass' 解析成 [(具体path, 值)]。"""
    head, _, rest = path.partition(".")
    if "[" in head:
        name, _, key = head.partition("[")
        key = key.rstrip("]")
    else:
        name, key = head, None
    node = doc.get(name)
    if node is None:
        return []
    if isinstance(node, list):
        items = node if key == "*" else [x for x in node if isinstance(x, dict) and x.get("name") == key]
        label_of = lambda it: f"{name}[{it.get('name')}]"  # noqa: E731
    else:
        items = [node]
        label_of = lambda it: name  # noqa: E731
    out: list[tuple[str, object]] = []
    for it in items:
        value = it
        for k in (rest.split(".") if rest else []):
            value = value.get(k) if isinstance(value, dict) else None
        out.append((f"{label_of(it)}.{rest}" if rest else label_of(it), value))
    return out


def check(doc: dict) -> list[tuple[str, bool, str]]:
    results: list[tuple[str, bool, str]] = []
    links = {l["name"]: l for l in doc.get("links", [])}
    joints = doc.get("joints", [])
    acts = doc.get("actuators", [])

    # C1 树
    problems = []
    roots = [n for n, l in links.items() if not l.get("parent") or l["parent"] == "world"]
    if len(roots) != 1:
        problems.append(f"根不唯一: {roots}")
    for n, l in links.items():
        p = l.get("parent")
        if p and p != "world" and p not in links:
            problems.append(f"{n} 的 parent {p} 不存在")
    for j in joints:
        for side in ("parent", "child"):
            if j.get(side) and j[side] != "world" and j[side] not in links:
                problems.append(f"关节 {j['name']} 的 {side} {j[side]} 不存在")
    # 环检测（走 link.parent）
    for start in links:
        seen, cur = set(), start
        while cur and cur != "world" and cur in links:
            if cur in seen:
                problems.append(f"父子成环，起点 {start}")
                break
            seen.add(cur)
            cur = links[cur].get("parent")
    results.append(("C1", not problems, "; ".join(problems) or f"{len(links)} link / {len(joints)} joint / 根 {roots[0]}"))

    # C2 名字
    problems = []
    for label, names in (("link", [l["name"] for l in doc.get("links", [])]),
                         ("joint", [j["name"] for j in joints]),
                         ("actuator", [a["name"] for a in acts])):
        dup = {n for n in names if names.count(n) > 1}
        if dup:
            problems.append(f"{label} 重名: {sorted(dup)}")
    actuated = {j["name"] for j in joints if j.get("actuated")}
    for a in acts:
        if a.get("joint") not in actuated:
            problems.append(f"执行器 {a['name']} 指向的 {a.get('joint')} 不是 actuated 关节")
    results.append(("C2", not problems, "; ".join(problems) or f"actuated {len(actuated)} / actuator {len(acts)}"))

    # C3 单位
    problems = []
    units = doc.get("frames", {}).get("units", {})
    for k, v in E.SI_UNITS.items():
        if units.get(k) != v:
            problems.append(f"frames.units.{k} 应为 {v}，实际 {units.get(k)}")
    bad = [p["unit"] for p in doc.get("provenance", []) if p.get("unit") not in E.ALLOWED_UNITS]
    if bad:
        problems.append(f"provenance 里有非 SI 单位: {sorted(set(bad))}")
    results.append(("C3", not problems, "; ".join(problems) or "全 SI"))

    # C4 provenance 冲突
    problems, seen = [], {}
    for p in doc.get("provenance", []):
        for concrete, value in resolve(doc, p["path"]) or [(p["path"], p.get("value"))]:
            key = concrete
            if key in seen and seen[key][0] != value and not p.get("override"):
                problems.append(f"{key}: {seen[key][0]!r}({seen[key][1]}) vs {value!r}({p['source']})")
            seen.setdefault(key, (value, p["source"]))
    results.append(("C4", not problems, "; ".join(problems) or f"{len(seen)} 条出处无冲突"))

    # C5 preset
    problems = []
    for name, preset in (doc.get("presets") or {}).items():
        if "actuated" in preset and set(preset["actuated"]) != actuated:
            problems.append(f"preset {name} 改变了 actuated 集合")
    results.append(("C5", not problems, "; ".join(problems) or f"{len(doc.get('presets') or {})} 个 preset 不动驱动集"))

    # C6 并联机构
    problems, infos = [], []
    seen_mech = set()
    for m in doc.get("mechanisms") or []:
        mid = m.get("id")
        if mid in seen_mech:
            problems.append(f"mechanism 重名: {mid}")
        seen_mech.add(mid)
        charts = m.get("charts") or {}
        pr_joints = (charts.get("pr") or {}).get("joints") or []
        for name in pr_joints:
            if name not in {j["name"] for j in joints}:
                problems.append(f"{mid}: PR 关节 {name} 不存在")
            elif name not in actuated:
                problems.append(f"{mid}: PR 关节 {name} 不是 actuated")
        motors = (charts.get("ab") or {}).get("motors") or []
        letters = [x.get("ab") for x in motors]
        if sorted(letters) != ["A", "B"]:
            problems.append(f"{mid}: AB 电机必须恰好是 A 和 B，实际 {letters}")
        idx = [x.get("index") for x in motors]
        if len(set(idx)) != len(idx):
            problems.append(f"{mid}: AB 电机 index 重复 {idx}")
        coupling = (m.get("coupling") or {}).get("kind")
        if coupling == "unresolved":
            infos.append(f"{mid}: coupling 未解")
        elif coupling not in ("analytic", "table"):
            problems.append(f"{mid}: coupling.kind 非法 {coupling!r}")
    detail = "; ".join(problems) if problems else (
        f"{len(seen_mech)} 个并联机构" + (f"（{len(infos)} 个 coupling 未解：{'、'.join(infos)}）" if infos else ""))
    results.append(("C6", not problems, detail))

    return results


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("entities", nargs="+")
    args = ap.parse_args()
    paths = []
    for pattern in args.entities:
        paths.extend(sorted(glob.glob(pattern)) or [pattern])
    failed = 0
    for path in paths:
        doc = E.read_yaml(path)
        print(f"\n== {path}")
        print(f"   {doc['robot']['id']} · {doc['summary']} · warnings={len(doc.get('warnings', []))}")
        for code, ok, detail in check(doc):
            print(f"   {'PASS' if ok else 'FAIL'} {code}  {detail}")
            failed += 0 if ok else 1
    print()
    print("全部通过" if failed == 0 else f"失败 {failed} 项")
    return 0 if failed == 0 else 1


if __name__ == "__main__":
    raise SystemExit(main())
