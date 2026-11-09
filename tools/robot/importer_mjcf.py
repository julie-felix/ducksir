#!/usr/bin/env python3
"""MJCF → 实体（数字描述）。

只解析 XML，不加载 MuJoCo：本机没有 pip，而这一版要的也只是「结构 + 惯性 + 执行器」。

    python3 tools/robot/importer_mjcf.py <robot.xml> -o contract/robot/types/duck-hd1910.yaml \
        --entity-id duck-hd1910 --type-id duck-hd1910 --license Apache-2.0 \
        --source-ref <upstream url> --preset walk

能处理：default 类继承、body 树、hinge/free 关节、inertial（fullinertia/diaginertia）、
geom（visual/collision 按 contype/conaffinity 判定）、position/motor/general 执行器、sensor、material。
不处理：include（先自己展开）、tendon、plugin、keyframe（这些不影响数字描述本体）。
"""

from __future__ import annotations

import argparse
import os
import sys
import xml.etree.ElementTree as ET

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import entity as E  # noqa: E402

GEOM_VISUAL_GROUPS = {"visual"}


# ---------- default 类继承 ----------

def collect_classes(root: ET.Element) -> dict:
    raw: dict[str, dict] = {"main": {}}
    parent: dict[str, str | None] = {"main": None}

    def visit(node: ET.Element, parent_name: str, cls_name: str, merge_main: bool) -> None:
        if not merge_main:
            parent[cls_name] = parent_name
            raw.setdefault(cls_name, {})
        for el in node:
            if el.tag == "default":
                continue
            raw.setdefault(cls_name, {}).setdefault(el.tag, {}).update(el.attrib)
        for child in node.findall("default"):
            visit(child, cls_name, child.get("class") or cls_name, False)

    for d in root.findall("default"):
        cls = d.get("class")
        if cls:
            visit(d, "main", cls, False)
        else:
            visit(d, "main", "main", True)

    cum: dict[str, dict] = {}

    def resolve(name: str, stack: tuple = ()) -> dict:
        if name in cum:
            return cum[name]
        if name in stack:
            raise ValueError(f"default 继承成环: {name}")
        base = resolve(parent[name], stack + (name,)) if parent.get(name) else {}
        merged = {tag: dict(vals) for tag, vals in base.items()}
        for tag, vals in raw.get(name, {}).items():
            merged.setdefault(tag, {}).update(vals)
        cum[name] = merged
        return merged

    for name in list(raw):
        resolve(name)
    return cum


def merged(cls: str, tag: str, classes: dict, el: ET.Element | None = None) -> dict:
    out = dict(classes.get(cls, {}).get(tag, {}))
    if el is not None:
        out.update(el.attrib)
    return out


# ---------- 主体 ----------

def import_mjcf(path: str, *, entity_id: str, type_id: str, license_id: str,
                source_ref: str, preset: str | None, assets_dir: str,
                bam_json: str | None, bam_ref: str | None) -> dict:
    tree = ET.parse(path)
    root = tree.getroot()
    classes = collect_classes(root)

    compiler = root.find("compiler")
    angle = (compiler.get("angle") if compiler is not None else None) or "radian"
    if angle != "radian":
        raise SystemExit("这一版只支持 angle=radian；角度换算还没写")
    meshdir = (compiler.get("meshdir") if compiler is not None else None) or ""

    # mesh 名 → 文件
    mesh_files: dict[str, str] = {}
    materials: dict[str, dict] = {}
    asset = root.find("asset")
    if asset is not None:
        for m in asset.findall("mesh"):
            f = m.get("file")
            name = m.get("name") or os.path.splitext(os.path.basename(f))[0]
            mesh_files[name] = os.path.join(meshdir, f) if meshdir else f
        for mt in asset.findall("material"):
            materials[mt.get("name")] = {
                "rgba": E.numbers(mt.get("rgba")) if mt.get("rgba") else None,
                "texture": mt.get("texture"),
            }

    doc = E.make_entity(
        entity_id=entity_id, name=root.get("model") or entity_id, type_id=type_id,
        source_ref=source_ref, license_id=license_id,
        generator={"name": "importer_mjcf", "version": "0.1.0",
                   "input": os.path.relpath(path), "sha256": E.sha256_file(path)},
        root="", world_attachment="floating",
    )
    doc["robot"]["assets_dir"] = assets_dir
    warnings = doc["warnings"]
    prov = doc["provenance"]
    id_to_link: dict[str, dict] = {}
    id_to_joint: dict[str, dict] = {}

    def add_prov(path_, value, unit, source, ref=None, note=None):
        prov.append(E.prov(path_, value, unit, source, ref=ref, note=note))

    # 1) worldbody
    worldbody = root.find("worldbody")
    if worldbody is None:
        raise SystemExit("没有 <worldbody>")

    def walk_body(body: ET.Element, parent_name: str, parent_childclass: str) -> None:
        name = body.get("name")
        childclass = body.get("childclass") or parent_childclass
        xyz = E.vec3(body.get("pos"))
        quat = E.numbers(body.get("quat")) if body.get("quat") else [1.0, 0.0, 0.0, 0.0]
        link = {
            "name": name,
            "parent": parent_name,
            "transform": {"xyz": xyz, "quat": [round(v, 12) for v in quat]},
            "mass": None,
            "com": None,
            "inertia": None,
            "visual": [],
            "collision": [],
        }

        inertial = body.find("inertial")
        if inertial is not None:
            mass = float(merged(childclass, "inertial", classes, inertial).get("mass", inertial.get("mass", 0.0)))
            com = E.vec3(inertial.get("pos"))
            link["mass"] = round(mass, 12)
            link["com"] = com
            if inertial.get("fullinertia"):
                ixx, iyy, izz, ixy, ixz, iyz = E.numbers(inertial.get("fullinertia"))
                link["inertia"] = {"ixx": ixx, "ixy": ixy, "ixz": ixz,
                                   "iyy": iyy, "iyz": iyz, "izz": izz}
            elif inertial.get("diaginertia"):
                diag = E.numbers(inertial.get("diaginertia"))
                q = E.numbers(inertial.get("quat")) if inertial.get("quat") else [1.0, 0.0, 0.0, 0.0]
                link["inertia"] = E.rotate_inertia_quat(diag, q)
                add_prov(f"links[{name}].inertia.principal", diag, "inertia", "mjcf",
                         ref=os.path.basename(path), note="源是 diaginertia + quat，已转到 link 系")
            add_prov(f"links[{name}].mass", link["mass"], "mass", "mjcf", ref=os.path.basename(path))
            add_prov(f"links[{name}].com", com, "length", "mjcf", ref=os.path.basename(path))
            add_prov(f"links[{name}].inertia", link["inertia"], "inertia", "mjcf", ref=os.path.basename(path))
        else:
            E.warn(warnings, "W001", f"links[{name}]", "body 没有 <inertial>，mass/com/inertia 为空")

        for geom in body.findall("geom"):
            if geom.get("type") not in (None, "mesh"):
                link["collision"].append({"type": geom.get("type"),
                                          "size": E.numbers(geom.get("size")) if geom.get("size") else None,
                                          "origin": {"xyz": E.vec3(geom.get("pos")),
                                                     "quat": E.numbers(geom.get("quat")) if geom.get("quat") else [1, 0, 0, 0]}})
                continue
            cls = geom.get("class") or childclass
            attrs = merged(cls, "geom", classes, None)
            contype = int(attrs.get("contype", geom.get("contype", 1)))
            conaffinity = int(attrs.get("conaffinity", geom.get("conaffinity", 1)))
            mesh_name = geom.get("mesh")
            item = {
                "mesh": os.path.join(assets_dir, os.path.basename(mesh_files.get(mesh_name, "")))
                if mesh_name else None,
                "mesh_name": mesh_name,
                "origin": {"xyz": E.vec3(geom.get("pos")),
                           "quat": E.numbers(geom.get("quat")) if geom.get("quat") else [1.0, 0.0, 0.0, 0.0]},
                "material": list((materials.get(geom.get("material")) or {}).get("rgba") or []),
                "class": geom.get("class"),
                "vendored": False,
            }
            if contype == 0 and conaffinity == 0:
                link["visual"].append(item)
            else:
                item["self_collision"] = contype == 2
                link["collision"].append(item)

        id_to_link[name] = link
        doc["links"].append(link)
        if parent_name == "world":
            doc["frames"]["root"] = name

        # 关节
        free = body.find("freejoint")
        if free is not None:
            doc["frames"]["freejoint"] = free.get("name")
            E.warn(warnings, "W002", f"links[{name}]",
                   "根 body 有 freejoint；实体里记在 frames.world_attachment=floating + frames.freejoint，不重复成一条 joint")
        for j in body.findall("joint"):
            jname = j.get("name")
            cls = j.get("class") or childclass
            attrs = merged(cls, "joint", classes, j)
            jtype = {"hinge": "revolute", "slide": "prismatic"}.get(j.get("type", "hinge"), j.get("type"))
            lo_up = None
            if j.get("range"):
                r = E.numbers(j.get("range"))
                lo_up = {"lower": r[0], "upper": r[1]}
            else:
                E.warn(warnings, "W001", f"joints[{jname}].limit",
                       "关节没有 range（autolimits 下即无硬限位）")
            joint = {
                "name": jname,
                "type": jtype,
                "parent": parent_name,
                "child": name,
                "axis": {"origin": E.vec3(j.get("pos")),
                         "dir": E.numbers(j.get("axis")) if j.get("axis") else [0.0, 0.0, 1.0]},
                "limit": {"mech": lo_up},
                "actuated": False,
                "class": j.get("class"),
            }
            id_to_joint[jname] = joint
            doc["joints"].append(joint)
            add_prov(f"joints[{jname}].limit.mech", lo_up, "angle", "mjcf", ref=os.path.basename(path))
            add_prov(f"joints[{jname}].axis.dir", joint["axis"]["dir"], "length", "mjcf", ref=os.path.basename(path))
            for k in ("damping", "frictionloss", "armature"):
                if attrs.get(k) is not None:
                    joint.setdefault("props", {})[k] = float(attrs[k])

        for site in body.findall("site"):
            doc["sensors"].append({"name": site.get("name"), "kind": "site", "body": name,
                                   "pos": E.vec3(site.get("pos")),
                                   "quat": E.numbers(site.get("quat")) if site.get("quat") else [1, 0, 0, 0]})

        for child_body in body.findall("body"):
            walk_body(child_body, name, childclass)

    for child_body in worldbody.findall("body"):
        walk_body(child_body, "world", "main")

    # 2) 执行器
    act = root.find("actuator")
    if act is not None:
        for el in act:
            cls = el.get("class") or "main"
            attrs = merged(cls, "position" if el.tag == "position" else el.tag, classes, el)
            jname = el.get("joint")
            item = {
                "name": el.get("name") or jname,
                "joint": jname,
                "kind": {"position": "position", "motor": "motor", "general": "general"}.get(el.tag, el.tag),
                "model": "unknown",
                "kp": float(attrs["kp"]) if attrs.get("kp") is not None else None,
                "kv": float(attrs["kv"]) if attrs.get("kv") is not None else None,
                "forcerange": E.numbers(attrs["forcerange"]) if attrs.get("forcerange") else None,
                "ctrlrange": E.numbers(attrs["ctrlrange"]) if attrs.get("ctrlrange") else None,
                "damping": id_to_joint.get(jname, {}).get("props", {}).get("damping"),
                "frictionloss": id_to_joint.get(jname, {}).get("props", {}).get("frictionloss"),
                "armature": id_to_joint.get(jname, {}).get("props", {}).get("armature"),
                "class": el.get("class"),
            }
            doc["actuators"].append(item)
            if jname in id_to_joint:
                id_to_joint[jname]["actuated"] = True
                id_to_joint[jname]["actuator"] = item["name"]
            add_prov(f"actuators[{item['name']}].kp", item["kp"], "stiffness", "mjcf", ref=os.path.basename(path))
            if item["forcerange"]:
                add_prov(f"actuators[{item['name']}].forcerange", item["forcerange"], "torque", "mjcf",
                         ref=os.path.basename(path))


    # 3) BAM 参数（若给了）
    if bam_json:
        import json
        with open(bam_json, encoding="utf-8") as f:
            bam = json.load(f)
        for a in doc["actuators"]:
            a["kt"] = bam.get("kt")
            a["resistance"] = bam.get("R")
            a["command_delay"] = bam.get("command_delay")
            a["params_ref"] = bam_ref or os.path.basename(bam_json)
        add_prov("actuators[*].kt", bam.get("kt"), "torque", "identified", ref=bam_ref or bam_json,
                 note="BAM M6 对 sts3215 的辨识结果，不是本机实测")
        add_prov("actuators[*].resistance", bam.get("R"), "length", "identified", ref=bam_ref or bam_json)

    # 4) sensor（site 已在 walk_body 里收）
    sensor = root.find("sensor")
    if sensor is not None:
        for el in sensor:
            doc["sensors"].append({"name": el.get("name"), "kind": el.tag,
                                   "site": el.get("site"), "body": el.get("body"),
                                   "noise": float(el.get("noise")) if el.get("noise") else None})
            add_prov(f"sensors[{el.get('name')}].noise", float(el.get("noise") or 0.0), "length",
                     "mjcf", ref=os.path.basename(path))

    missing = sorted({v["mesh"] for l in doc["links"] for v in l["visual"] if v["mesh"]})
    if missing:
        E.warn(warnings, "W004", "links[*].visual[*].mesh",
               f"{len(missing)} 个 mesh 只有引用、没有入库；需要时可从上游按 sha256 拉一次",
               meshes=missing[:8], more=max(0, len(missing) - 8))

    # 5) 变体
    base = os.path.basename(path)
    preset_name = preset or os.path.splitext(base)[0]
    collision = "allcollisions" if "allcollisions" in base else "walk"
    doc["presets"][preset_name] = {
        "source_file": base,
        "collision": collision,
        "rollers": "rollers" in base,
        "backlash": "backlash" in base,
    }
    E.warn(warnings, "W005", "presets",
           "fanhao 仓库有 6 个 MJCF 变体（collision×rollers×backlash），本实体只按导入的那一份生成 preset；"
           "其余 5 个按需再加")

    doc["summary"] = {
        "links": len(doc["links"]),
        "joints": len(doc["joints"]),
        "actuated_joints": sum(1 for j in doc["joints"] if j["actuated"]),
        "actuators": len(doc["actuators"]),
        "total_mass": round(sum(l["mass"] or 0.0 for l in doc["links"]), 9),
    }
    add_prov("summary.total_mass", doc["summary"]["total_mass"], "mass", "mjcf",
             ref=os.path.basename(path), note="15 个刚体求和")
    return doc


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("input")
    ap.add_argument("-o", "--output", required=True)
    ap.add_argument("--entity-id", required=True)
    ap.add_argument("--type-id", required=True)
    ap.add_argument("--license", default="unknown")
    ap.add_argument("--source-ref", default="")
    ap.add_argument("--assets-dir", default="assets/microduck")
    ap.add_argument("--preset", default=None)
    ap.add_argument("--bam-json", default=None)
    ap.add_argument("--bam-ref", default=None)
    args = ap.parse_args()
    doc = import_mjcf(args.input, entity_id=args.entity_id, type_id=args.type_id,
                      license_id=args.license, source_ref=args.source_ref,
                      preset=args.preset, assets_dir=args.assets_dir,
                      bam_json=args.bam_json, bam_ref=args.bam_ref)
    E.write_yaml(args.output, doc)
    print(f"写入 {args.output}: {doc['summary']}；warnings={len(doc['warnings'])}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
