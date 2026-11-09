#!/usr/bin/env python3
"""把实体压成给浏览器用的 JSON（客户端只读这个，不解析 YAML）。

    python3 tools/robot/export_viewer.py contract/robot/types/*.yaml

产物：`client/viewer/data/index.json` + 每台机器一个 `<id>.json`。
mesh 路径改写成相对查看器的路径，其它（transform / 轴向 / 限位 / 颜色）原样带过去。
"""

from __future__ import annotations

import argparse
import glob
import json
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import entity as E  # noqa: E402

OUT = os.path.join("client", "viewer", "data")
ASSET_PREFIX = "../../"  # 从 client/viewer/ 回到仓库根


def slim(doc: dict) -> dict:
    links = []
    for l in doc["links"]:
        geoms = []
        for g in l.get("visual", []):
            if not g.get("mesh"):
                continue
            geoms.append({
                "mesh": ASSET_PREFIX + g["mesh"],
                "xyz": g["origin"]["xyz"],
                "quat": g["origin"]["quat"],
                "color": g.get("material") or [0.75, 0.75, 0.75, 1.0],
            })
        links.append({
            "name": l["name"],
            "parent": l.get("parent"),
            "xyz": l["transform"]["xyz"],
            "quat": l["transform"]["quat"],
            "mass": l.get("mass"),
            "geoms": geoms,
        })
    joints = []
    for j in doc["joints"]:
        if j.get("type") not in ("revolute", "prismatic"):
            continue
        mech = (j.get("limit") or {}).get("mech") or {}
        joints.append({
            "name": j["name"],
            "parent": j["parent"],
            "child": j["child"],
            "type": j["type"],
            "axis": j["axis"]["dir"],
            "lower": mech.get("lower"),
            "upper": mech.get("upper"),
            "actuated": bool(j.get("actuated")),
        })
    return {
        "id": doc["robot"]["id"],
        "name": doc["robot"]["name"],
        "root": doc["frames"]["root"],
        "world_attachment": doc["frames"].get("world_attachment"),
        "links": links,
        "joints": joints,
        "summary": doc.get("summary"),
        "warnings": doc.get("warnings", []),
    }


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("entities", nargs="+")
    ap.add_argument("--out", default=OUT)
    args = ap.parse_args()
    paths = []
    for pattern in args.entities:
        paths.extend(sorted(glob.glob(pattern)) or [pattern])
    os.makedirs(args.out, exist_ok=True)
    index = []
    for path in paths:
        doc = E.read_yaml(path)
        data = slim(doc)
        file_name = data["id"] + ".json"
        with open(os.path.join(args.out, file_name), "w", encoding="utf-8") as f:
            json.dump(data, f, ensure_ascii=False, separators=(",", ":"))
        index.append({"id": data["id"], "name": data["name"], "file": file_name,
                      "links": len(data["links"]), "joints": len(data["joints"])})
        print(f"  {data['id']}: {len(data['links'])} link / {len(data['joints'])} joint → {file_name}")
    with open(os.path.join(args.out, "index.json"), "w", encoding="utf-8") as f:
        json.dump({"robots": index}, f, ensure_ascii=False, indent=1)
    print(f"{len(index)} 台机器 → {args.out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
