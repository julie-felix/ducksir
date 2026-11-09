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


def export(doc: dict) -> ET.ElementTree:
    root = ET.Element("mujoco", model=doc["robot"]["id"])
    assets_dir = doc["robot"].get("assets_dir") or "assets"
    ET.SubElement(root, "compiler", angle="radian", meshdir=assets_dir, autolimits="true")

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
            ET.SubElement(asset, "mesh", name=name, file=os.path.basename(item["mesh"]))
            rgba = item.get("material") or []
            if rgba and rgba not in materials:
                materials.append(rgba)
                ET.SubElement(asset, "material", name=f"mat{len(materials) - 1}", rgba=_vec(rgba))

    worldbody = ET.SubElement(root, "worldbody")
    joint_by_child = {j["child"]: j for j in doc["joints"]}

    def build(link: dict, parent_el: ET.Element) -> None:
        body = ET.SubElement(parent_el, "body", name=link["name"])
        if link.get("parent") == "world":
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
    args = ap.parse_args()
    doc = E.read_yaml(args.entity)
    tree = export(doc)
    ET.indent(tree, space="  ")
    os.makedirs(os.path.dirname(os.path.abspath(args.output)), exist_ok=True)
    tree.write(args.output, encoding="utf-8", xml_declaration=True)
    print(f"写入 {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
