#!/usr/bin/env python3
"""STL → GLB（纯标准库，本机没有 assimp / blender）。

    python3 tools/robot/stl2glb.py third_party/unitree-g1/meshes assets/g1

做四件事：
1. 解析二进制 / ASCII STL；
2. 顶点焊接（0.1 mm 网格），把「一个三角形三个顶点」压成共享顶点；
3. 按 40° 折痕角切法线：同一位置上的面如果夹角小就平均（表面平滑），超过就拆开（棱角保住）——
   CAD 模型直接用平滑法线会糊，直接用三角形汤又白占体积；
4. 写 GLB（glTF 2.0 二进制，POSITION + NORMAL + indices）。
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
import struct
import sys

CREASE_COS = math.cos(math.radians(40.0))
QUANT = 1e-4


def read_stl(path: str) -> list[tuple[tuple[float, float, float], ...]]:
    with open(path, "rb") as f:
        data = f.read()
    if len(data) >= 84:
        n = struct.unpack("<I", data[80:84])[0]
        if 84 + n * 50 == len(data):
            tris = []
            for i in range(n):
                off = 84 + i * 50 + 12  # 跳过面法线，自己算
                v = struct.unpack_from("<9f", data, off)
                tris.append(((v[0], v[1], v[2]), (v[3], v[4], v[5]), (v[6], v[7], v[8])))
            return tris
    # ASCII
    tris, cur = [], []
    for line in data.decode("utf-8", "ignore").splitlines():
        parts = line.split()
        if len(parts) == 4 and parts[0] == "vertex":
            cur.append((float(parts[1]), float(parts[2]), float(parts[3])))
            if len(cur) == 3:
                tris.append(tuple(cur))
                cur = []
    return tris


def build(tris):
    welded: dict[tuple, int] = {}
    verts: list[tuple[float, float, float]] = []
    faces: list[tuple[int, int, int]] = []
    for tri in tris:
        idx = []
        for p in tri:
            key = (round(p[0] / QUANT), round(p[1] / QUANT), round(p[2] / QUANT))
            i = welded.get(key)
            if i is None:
                i = len(verts)
                welded[key] = i
                verts.append(p)
            idx.append(i)
        if idx[0] == idx[1] or idx[1] == idx[2] or idx[0] == idx[2]:
            continue
        faces.append((idx[0], idx[1], idx[2]))

    face_n = []
    for a, b, c in faces:
        ax, ay, az = verts[a]
        bx, by, bz = verts[b]
        cx, cy, cz = verts[c]
        ux, uy, uz = bx - ax, by - ay, bz - az
        vx, vy, vz = cx - ax, cy - ay, cz - az
        nx, ny, nz = uy * vz - uz * vy, uz * vx - ux * vz, ux * vy - uy * vx
        norm = math.sqrt(nx * nx + ny * ny + nz * nz) or 1.0
        face_n.append((nx / norm, ny / norm, nz / norm))

    clusters: dict[int, list] = {}          # vert -> [[sumx,sumy,sumz,count], ...]
    corner_cluster: dict[tuple, int] = {}   # (face, corner) -> cluster id
    for f, (a, b, c) in enumerate(faces):
        n = face_n[f]
        for corner, v in enumerate((a, b, c)):
            group = clusters.setdefault(v, [])
            found = None
            for cid, acc in enumerate(group):
                sx, sy, sz, cnt = acc
                ln = math.sqrt(sx * sx + sy * sy + sz * sz) or 1.0
                if (sx / ln) * n[0] + (sy / ln) * n[1] + (sz / ln) * n[2] >= CREASE_COS:
                    found = cid
                    acc[0] += n[0]
                    acc[1] += n[1]
                    acc[2] += n[2]
                    acc[3] += 1
                    break
            if found is None:
                group.append([n[0], n[1], n[2], 1])
                found = len(group) - 1
            corner_cluster[(f, corner)] = found

    out_pos: list[float] = []
    out_nrm: list[float] = []
    out_idx: list[int] = []
    render_of: dict[tuple, int] = {}
    for f, vs in enumerate(faces):
        for corner, v in enumerate(vs):
            cid = corner_cluster[(f, corner)]
            key = (v, cid)
            r = render_of.get(key)
            if r is None:
                sx, sy, sz, _ = clusters[v][cid]
                ln = math.sqrt(sx * sx + sy * sy + sz * sz) or 1.0
                r = len(out_pos) // 3
                render_of[key] = r
                out_pos.extend(verts[v])
                out_nrm.extend((sx / ln, sy / ln, sz / ln))
            out_idx.append(r)
    return out_pos, out_nrm, out_idx, len(faces)


def write_glb(path: str, name: str, pos, nrm, idx) -> None:
    pos_bin = struct.pack(f"<{len(pos)}f", *pos)
    nrm_bin = struct.pack(f"<{len(nrm)}f", *nrm)
    idx_bin = struct.pack(f"<{len(idx)}I", *idx)
    while len(pos_bin) % 4:
        pos_bin += b"\0"
    while len(nrm_bin) % 4:
        nrm_bin += b"\0"
    while len(idx_bin) % 4:
        idx_bin += b"\0"
    blob = pos_bin + nrm_bin + idx_bin
    nverts = len(pos) // 3
    js = {
        "asset": {"version": "2.0", "generator": "ducksir stl2glb"},
        "scene": 0,
        "scenes": [{"nodes": [0]}],
        "nodes": [{"mesh": 0, "name": name}],
        "meshes": [{"name": name, "primitives": [{"attributes": {"POSITION": 0, "NORMAL": 1},
                                                   "indices": 2, "mode": 4}]}],
        "buffers": [{"byteLength": len(blob)}],
        "bufferViews": [
            {"buffer": 0, "byteOffset": 0, "byteLength": len(pos_bin), "target": 34962},
            {"buffer": 0, "byteOffset": len(pos_bin), "byteLength": len(nrm_bin), "target": 34962},
            {"buffer": 0, "byteOffset": len(pos_bin) + len(nrm_bin), "byteLength": len(idx_bin),
             "target": 34963},
        ],
        "accessors": [
            {"bufferView": 0, "componentType": 5126, "count": nverts, "type": "VEC3",
             "min": [min(pos[0::3]), min(pos[1::3]), min(pos[2::3])],
             "max": [max(pos[0::3]), max(pos[1::3]), max(pos[2::3])]},
            {"bufferView": 1, "componentType": 5126, "count": nverts, "type": "VEC3"},
            {"bufferView": 2, "componentType": 5125, "count": len(idx), "type": "SCALAR"},
        ],
    }
    js_bin = json.dumps(js, separators=(",", ":")).encode("utf-8")
    while len(js_bin) % 4:
        js_bin += b" "
    total = 12 + 8 + len(js_bin) + 8 + len(blob)
    with open(path, "wb") as f:
        f.write(struct.pack("<III", 0x46546C67, 2, total))
        f.write(struct.pack("<II", len(js_bin), 0x4E4F534A))
        f.write(js_bin)
        f.write(struct.pack("<II", len(blob), 0x004E4942))
        f.write(blob)


def convert(src: str, dst: str) -> dict:
    tris = read_stl(src)
    pos, nrm, idx, nfaces = build(tris)
    write_glb(dst, os.path.splitext(os.path.basename(dst))[0], pos, nrm, idx)
    with open(dst, "rb") as f:
        digest = hashlib.sha256(f.read()).hexdigest()
    return {"glb": dst, "sha256": digest, "bytes": os.path.getsize(dst),
            "triangles": nfaces, "vertices": len(pos) // 3,
            "source_bytes": os.path.getsize(src)}


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("stage_dir")
    ap.add_argument("out_dir")
    ap.add_argument("--manifest", default=None)
    ap.add_argument("--ext", default=None)
    args = ap.parse_args()
    exts = {args.ext} if args.ext else {"stl", "STL"}
    os.makedirs(args.out_dir, exist_ok=True)
    files = sorted(f for f in os.listdir(args.stage_dir) if f.rsplit(".", 1)[-1] in exts)
    out = {}
    for f in files:
        src = os.path.join(args.stage_dir, f)
        name = os.path.splitext(f)[0]
        dst = os.path.join(args.out_dir, name + ".glb")
        out[name] = convert(src, dst)
        print(f"  {name:38s} {out[name]['triangles']:7d} tri  "
              f"{out[name]['source_bytes']/1e6:6.2f} → {out[name]['bytes']/1e6:5.2f} MB")
    manifest = args.manifest or os.path.join(args.out_dir, "manifest.json")
    with open(manifest, "w", encoding="utf-8") as fh:
        json.dump(out, fh, indent=1, ensure_ascii=False, sort_keys=True)
    src_total = sum(v["source_bytes"] for v in out.values()) / 1e6
    dst_total = sum(v["bytes"] for v in out.values()) / 1e6
    print(f"{len(out)} 个 mesh：{src_total:.1f} MB → {dst_total:.1f} MB → {manifest}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
