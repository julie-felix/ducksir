#!/usr/bin/env python3
"""实体 → 最小 MJCF（只做 body 树 / joint / inertial / geom / 执行器）。

用途只有一个：证明导入是对的。不是给训练用的导出端（那个以后再说）。

    python3 tools/robot/export_mjcf.py contract/robot/types/duck-hd1910.yaml -o /tmp/duck.xml
"""

from __future__ import annotations

import argparse
import os
import sys
import xml.etree.ElementTree as ET

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import entity as E  # noqa: E402


def _f(x: float) -> str:
    return repr(round(float(x), 12))


def _vec(xs) -> str:
    return " ".join(_f(x) for x in xs)


def apply_collision_variant(doc: dict, variant_path: str) -> dict:
    """用变体文件（`contract/robot/variants/<id>.all.yaml`）里的碰撞体覆盖本体的碰撞体。

    只换 collision，visual/关节/质量一概不动——变体是「同一个机器人的另一种碰撞集」，
    不是另一台机器人（`robot.variant_of` 指着本体）。
    """
    alt = E.read_yaml(variant_path)
    if alt["robot"].get("variant_of") != doc["robot"]["id"]:
        raise SystemExit(f"{variant_path}: variant_of={alt['robot'].get('variant_of')} 与 {doc['robot']['id']} 不符")
    by_name = {l["name"]: l for l in alt["links"]}
    for link in doc["links"]:
        other = by_name.get(link["name"])
        if other is not None:
            link["collision"] = other.get("collision") or []
    doc["robot"]["collision_set"] = {
        "name": alt["robot"].get("variant", {}).get("value", "variant"),
        "source": os.path.basename(variant_path),
        "source_file": alt["robot"].get("variant", {}).get("source_file"),
    }
    return doc


def export(doc: dict, mesh_dir: str | None = None, mesh_ext: str | None = None) -> ET.ElementTree:
    root = ET.Element("mujoco", model=doc["robot"]["id"])
    assets_dir = mesh_dir or doc["robot"].get("assets_dir") or "assets"
    ET.SubElement(root, "compiler", angle="radian", meshdir=assets_dir, autolimits="true")

    def _mesh_file(mesh_path: str) -> str:
        base = os.path.basename(mesh_path)
        if mesh_ext:
            base = os.path.splitext(base)[0] + "." + mesh_ext.lstrip(".")
        return base

    links = {l["name"]: l for l in doc["links"]}
    children: dict[str, list] = {}
    for l in doc["links"]:
        children.setdefault(l.get("parent") or "world", []).append(l)

    # asset：mesh + material（按 rgba 去重）
    asset = ET.SubElement(root, "asset")
    seen_mesh, materials = set(), []
    for l in doc["links"]:
        for item in l["visual"] + l["collision"]:
            name = item.get("mesh_name")
            if not name or name in seen_mesh:
                continue
            seen_mesh.add(name)
            ET.SubElement(asset, "mesh", name=name, file=_mesh_file(item["mesh"]))
            rgba = item.get("material") or []
            if rgba and rgba not in materials:
                materials.append(rgba)
                ET.SubElement(asset, "material", name=f"mat{len(materials) - 1}", rgba=_vec(rgba))

    worldbody = ET.SubElement(root, "worldbody")
    joint_by_child = {j["child"]: j for j in doc["joints"]}

    def build(link: dict, parent_el: ET.Element) -> None:
        body = ET.SubElement(parent_el, "body", name=link["name"])
        # 根 link 的 parent 可能写成 null，也可能写成 "world"；两种都算根（否则少一个 freejoint）
        if (link.get("parent") or "world") == "world":
            body.set("pos", _vec(link["transform"]["xyz"]))
            if doc["frames"].get("world_attachment") == "floating":
                ET.SubElement(body, "freejoint", name=doc["frames"].get("freejoint") or f"{link['name']}_freejoint")
        else:
            body.set("pos", _vec(link["transform"]["xyz"]))
            body.set("quat", _vec(link["transform"]["quat"]))

        if link.get("mass") is not None:
            inertia = link.get("inertia") or {}
            ET.SubElement(body, "inertial",
                          pos=_vec(link.get("com") or [0, 0, 0]),
                          mass=_f(link["mass"]),
                          fullinertia=" ".join(_f(inertia.get(k, 0.0)) for k in
                                               ("ixx", "iyy", "izz", "ixy", "ixz", "iyz")))

        joint = joint_by_child.get(link["name"])
        if joint and joint["type"] in ("revolute", "prismatic"):
            attrs = {"name": joint["name"], "type": "hinge" if joint["type"] == "revolute" else "slide",
                     "axis": _vec(joint["axis"]["dir"])}
            if joint["axis"].get("origin") and any(joint["axis"]["origin"]):
                attrs["pos"] = _vec(joint["axis"]["origin"])
            mech = (joint.get("limit") or {}).get("mech")
            if mech:
                attrs["range"] = f"{_f(mech['lower'])} {_f(mech['upper'])}"
            props = joint.get("props") or {}
            for k in ("damping", "frictionloss", "armature"):
                if props.get(k) is not None:
                    attrs[k] = _f(props[k])
            ET.SubElement(body, "joint", **attrs)
        # type=fixed 的关节不导出：MuJoCo 里父 body 直接就是焊死的

        for item in link["visual"]:
            if not item.get("mesh_name"):
                continue
            a = {"type": "mesh", "mesh": item["mesh_name"], "contype": "0", "conaffinity": "0",
                 "group": "2", "pos": _vec(item["origin"]["xyz"]), "quat": _vec(item["origin"]["quat"])}
            if item.get("material"):
                a["material"] = f"mat{materials.index(item['material'])}"
            ET.SubElement(body, "geom", **a)
        for item in link["collision"]:
            if item.get("mesh_name"):
                ET.SubElement(body, "geom", type="mesh", mesh=item["mesh_name"], group="3",
                              pos=_vec(item["origin"]["xyz"]), quat=_vec(item["origin"]["quat"]))
            elif item.get("type"):
                a = {"type": item["type"], "group": "3"}
                if item.get("size"):
                    a["size"] = _vec(item["size"])
                ET.SubElement(body, "geom", **a)

        for child in children.get(link["name"], []):
            build(child, body)

    roots = [l for l in doc["links"] if (l.get("parent") or "world") == "world"]
    if len(roots) != 1:
        raise SystemExit(f"根不唯一: {[r['name'] for r in roots]}")
    build(roots[0], worldbody)

    actuators = doc.get("actuators") or []
    if actuators:
        act = ET.SubElement(root, "actuator")
        for a in actuators:
            attrs = {"name": a["name"], "joint": a["joint"]}
            if a.get("kp") is not None:
                attrs["kp"] = _f(a["kp"])
            if a.get("kv") is not None:
                attrs["kv"] = _f(a["kv"])
            if a.get("forcerange"):
                attrs["forcerange"] = _vec(a["forcerange"])
            if a.get("ctrlrange"):
                attrs["ctrlrange"] = _vec(a["ctrlrange"])
            ET.SubElement(act, "position", **attrs)
    return ET.ElementTree(root)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("entity")
    ap.add_argument("-o", "--output", required=True)
    ap.add_argument("--mesh-dir", help="MJCF 的 meshdir（默认取实体里的 assets_dir）")
    ap.add_argument("--mesh-ext", help="强制 mesh 扩展名，例如 stl（MuJoCo 不认 glb）")
    ap.add_argument("--collision-set", default="default", choices=["default", "all"],
                    help="碰撞集：default=本体自带（walk 子集）；all=用 variants/<id>.all.yaml")
    ap.add_argument("--collision-variant", help="显式指定变体文件（默认按 <id>.all.yaml 约定找）")
    args = ap.parse_args()
    doc = E.read_yaml(args.entity)
    if args.collision_set == "all":
        vp = args.collision_variant or os.path.join(
            os.path.dirname(os.path.abspath(args.entity)), "..", "variants",
            f"{doc['robot']['id']}.all.yaml")
        if not os.path.exists(vp):
            raise SystemExit(f"找不到碰撞变体：{vp}")
        doc = apply_collision_variant(doc, os.path.normpath(vp))
    tree = export(doc, args.mesh_dir, args.mesh_ext)
    ET.indent(tree, space="  ")
    os.makedirs(os.path.dirname(os.path.abspath(args.output)), exist_ok=True)
    tree.write(args.output, encoding="utf-8", xml_declaration=True)
    cs = (doc.get("robot") or {}).get("collision_set")
    print(f"写入 {args.output}" + (f"（碰撞集 {cs['name']} ← {cs['source']}）" if cs else "（碰撞集 default）"))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
