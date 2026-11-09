#!/usr/bin/env python3
"""URDF → 实体（数字描述）。

    python3 tools/robot/importer_urdf.py <robot.urdf> -o contract/robot/types/g1-29dof.yaml \
        --entity-id g1-29dof --type-id g1-29dof --license BSD-3-Clause --source-ref <url>

URDF 的限制就是这一版的重点：
- 没有执行器参数（减速比、Kt、力矩常数）——`actuators[]` 会空着，并给出 W002 指路
- 没有并联/闭环；遇到就记 W006 并降级
- fixed joint 保留为 joint（拓扑是真的），导出 MJCF 时再焊死
- 根 link 没有 world 时，world_attachment 记为 floating 并给 W002（G1 的 world 是注释掉的）
"""

from __future__ import annotations

import argparse
import os
import sys
import xml.etree.ElementTree as ET

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import entity as E  # noqa: E402


def _origin(el: ET.Element | None) -> dict:
    if el is None:
        return {"xyz": [0.0, 0.0, 0.0], "quat": [1.0, 0.0, 0.0, 0.0], "rpy": [0.0, 0.0, 0.0]}
    xyz = E.vec3(el.get("xyz"))
    rpy = E.vec3(el.get("rpy"))
    return {"xyz": xyz, "rpy": rpy, "quat": E.rpy_to_quat(*rpy)}


def import_urdf(path: str, *, entity_id: str, type_id: str, license_id: str,
                source_ref: str, assets_dir: str) -> dict:
    root = ET.parse(path).getroot()
    doc = E.make_entity(
        entity_id=entity_id, name=root.get("name") or entity_id, type_id=type_id,
        source_ref=source_ref, license_id=license_id,
        generator={"name": "importer_urdf", "version": "0.1.0",
                   "input": os.path.relpath(path), "sha256": E.sha256_file(path)},
        root="", world_attachment="floating",
    )
    doc["robot"]["assets_dir"] = assets_dir
    warnings, prov = doc["warnings"], doc["provenance"]

    materials: dict[str, list] = {}
    for m in root.findall("material"):
        color = m.find("color")
        materials[m.get("name")] = E.numbers(color.get("rgba")) if color is not None else []

    def add_prov(p, v, unit, source="urdf", ref=None, note=None):
        prov.append(E.prov(p, v, unit, source, ref=ref, note=note))

    links: dict[str, dict] = {}
    for link_el in root.findall("link"):
        name = link_el.get("name")
        link = {"name": name, "parent": None, "transform": None,
                "mass": None, "com": None, "inertia": None, "visual": [], "collision": []}
        inertial = link_el.find("inertial")
        if inertial is not None:
            mass_value = inertial.find("mass")
            link["mass"] = round(float(mass_value.get("value")), 12) if mass_value is not None else None
            link["com"] = _origin(inertial.find("origin"))["xyz"]
            inertia = inertial.find("inertia")
            if inertia is not None:
                g = lambda k: float(inertia.get(k) or 0.0)  # noqa: E731
                raw_inertia = {"ixx": g("ixx"), "ixy": g("ixy"), "ixz": g("ixz"),
                               "iyy": g("iyy"), "iyz": g("iyz"), "izz": g("izz")}
                inertial_origin = _origin(inertial.find("origin"))
                link["inertia"] = E.rotate_inertia(raw_inertia, inertial_origin["rpy"])
                if any(inertial_origin["rpy"]):
                    add_prov(f"links[{name}].inertia.rotated", list(inertial_origin["rpy"]), "angle",
                             ref=os.path.basename(path),
                             note="inertial origin 带 rpy，惯量已转到 link 系（I_link = R·I·Rᵀ）")
            add_prov(f"links[{name}].mass", link["mass"], "mass", ref=os.path.basename(path))
            add_prov(f"links[{name}].com", link["com"], "length", ref=os.path.basename(path))
            add_prov(f"links[{name}].inertia", link["inertia"], "inertia", ref=os.path.basename(path))
        for kind in ("visual", "collision"):
            for el in link_el.findall(kind):
                geom = el.find("geometry")
                mesh = geom.find("mesh") if geom is not None else None
                mat = el.find("material")
                src = os.path.basename(mesh.get("filename")) if mesh is not None else None
                link[kind].append({
                    # mesh_name 一律取 stem（不带扩展名）：URDF 写 `x.STL`、MJCF 写 `x`，统一成一份
                    "mesh": os.path.join(assets_dir, os.path.splitext(src)[0] + ".glb") if src else None,
                    "mesh_name": os.path.splitext(src)[0] if src else None,
                    "source_file": src,
                    "origin": _origin(el.find("origin")),
                    "material": (materials.get(mat.get("name")) if mat is not None else None) or [],
                    "vendored": False,
                })
        links[name] = link
        doc["links"].append(link)

    # 关节
    child_of: dict[str, str] = {}
    for j in root.findall("joint"):
        jname, jtype = j.get("name"), j.get("type")
        parent = j.find("parent").get("link")
        child = j.find("child").get("link")
        child_of[child] = jname
        origin = _origin(j.find("origin"))
        axis_el = j.find("axis")
        limit = j.find("limit")
        mech = None
        if limit is not None and limit.get("lower") is not None and limit.get("upper") is not None:
            mech = {"lower": float(limit.get("lower")), "upper": float(limit.get("upper"))}
        entity_type = {"revolute": "revolute", "continuous": "revolute", "prismatic": "prismatic",
                       "fixed": "fixed", "floating": "free", "planar": "planar"}.get(jtype, jtype)
        joint = {
            "name": jname, "type": entity_type, "parent": parent, "child": child,
            "origin": {"xyz": origin["xyz"], "rpy": origin["rpy"], "quat": origin["quat"]},
            "axis": {"origin": [0.0, 0.0, 0.0],
                     "dir": E.numbers(axis_el.get("xyz")) if axis_el is not None else [1.0, 0.0, 0.0]},
            "limit": {"mech": mech,
                      "effort": float(limit.get("effort")) if limit is not None and limit.get("effort") else None,
                      "velocity": float(limit.get("velocity")) if limit is not None and limit.get("velocity") else None},
            "actuated": entity_type in ("revolute", "prismatic"),
        }
        doc["joints"].append(joint)
        if mech is not None:
            add_prov(f"joints[{jname}].limit.mech", mech, "angle", ref=os.path.basename(path))
        add_prov(f"joints[{jname}].axis.dir", joint["axis"]["dir"], "length", ref=os.path.basename(path))
        if joint["limit"]["effort"] is not None:
            add_prov(f"joints[{jname}].limit.effort", joint["limit"]["effort"], "torque",
                     ref=os.path.basename(path))

    # 拓扑：child link 的 transform 来自指向它的关节 origin
    for j in doc["joints"]:
        link = links[j["child"]]
        link["parent"] = j["parent"]
        link["transform"] = {"xyz": j["origin"]["xyz"], "quat": j["origin"]["quat"]}
    roots = [l["name"] for l in doc["links"] if l["parent"] is None]
    if len(roots) != 1:
        E.warn(warnings, "W001", "frames.root", f"找到 {len(roots)} 个根 link: {roots}；只接受 1 个")
    doc["frames"]["root"] = roots[0] if roots else ""
    for l in doc["links"]:
        if l["transform"] is None:
            l["transform"] = {"xyz": [0.0, 0.0, 0.0], "quat": [1.0, 0.0, 0.0, 0.0]}

    # URDF 的结构性缺口
    if not any(j["actuated"] for j in doc["joints"]):
        E.warn(warnings, "W001", "joints", "没有任何 actuated 关节")
    E.warn(warnings, "W002", "actuators[]",
           "URDF 不含执行器参数（减速比/Kt/力矩常数）；G1 的减速比在 README 表、effort/velocity 在 <limit> 与 motor_info.md")
    E.warn(warnings, "W002", "frames.world_attachment",
           "源 URDF 的 world+floating_base_joint 是注释掉的（原文 uncomment when convert to mujoco）；这里记 floating，导出 MJCF 时补 world")
    fixed = sum(1 for j in doc["joints"] if j["type"] == "fixed")
    if fixed:
        E.warn(warnings, "W006", "joints[*]",
               f"{fixed} 个 fixed joint 保留成关节（拓扑是真的）；导出 MJCF 时会被焊成同一个 body")
    missing = sorted({v["mesh"] for l in doc["links"] for v in l["visual"] if v["mesh"]})
    if missing:
        E.warn(warnings, "W004", "links[*].visual[*].mesh",
               f"{len(missing)} 个 mesh 只有引用、没有入库", meshes=missing[:8])

    doc["summary"] = {
        "links": len(doc["links"]),
        "joints": len(doc["joints"]),
        "actuated_joints": sum(1 for j in doc["joints"] if j["actuated"]),
        "actuators": len(doc["actuators"]),
        "total_mass": round(sum(l["mass"] or 0.0 for l in doc["links"]), 9),
    }
    add_prov("summary.total_mass", doc["summary"]["total_mass"], "mass", ref=os.path.basename(path),
             note=f"{len(doc['links'])} 个 link 求和")
    return doc


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("input")
    ap.add_argument("-o", "--output", required=True)
    ap.add_argument("--entity-id", required=True)
    ap.add_argument("--type-id", required=True)
    ap.add_argument("--license", default="unknown")
    ap.add_argument("--source-ref", default="")
    ap.add_argument("--assets-dir", default="assets/g1")
    ap.add_argument("--mesh-manifest", default=None)
    ap.add_argument("--mechanisms", default=None)
    args = ap.parse_args()
    doc = import_urdf(args.input, entity_id=args.entity_id, type_id=args.type_id,
                      license_id=args.license, source_ref=args.source_ref,
                      assets_dir=args.assets_dir)
    if args.mesh_manifest:
        E.attach_meshes(doc, args.mesh_manifest)
    if args.mechanisms:
        spec = E.read_yaml(args.mechanisms)
        doc["mechanisms"] = spec.get("mechanisms") or []
        doc["presets"].update(spec.get("presets") or {})
        doc["robot"]["mechanisms_spec"] = args.mechanisms
        E.warn(doc["warnings"], "W007", "mechanisms[]",
               f"并联机构的坐标与耦合来自手写 spec（{args.mechanisms}），不是 URDF 导入产物；"
               "URDF 只有 PR 串联等效，实物是并联")
    E.write_yaml(args.output, doc)
    print(f"写入 {args.output}: {doc['summary']}；warnings={len(doc['warnings'])}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
