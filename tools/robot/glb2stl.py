#!/usr/bin/env python3
"""GLB → STL（纯标准库）。MuJoCo 不认 glb，本仓的 mesh 却是给查看器用的 glb。

    python3 tools/robot/glb2stl.py assets/tienkung build/meshes/tienkung

只做一件事：把 glTF 2.0 的三角面取出来写成二进制 STL。节点变换会应用（平移/旋转/缩放），
法线不写（MuJoCo 自己按面算）。多 primitive、多 mesh、多节点都拼一起。

为什么不用 trimesh：本机没有 pip，零依赖是硬约束；stl2glb.py 已经是反方向，这套约定一致。
"""

from __future__ import annotations

import argparse
import json
import math
import os
import struct
import sys

JSON_CHUNK = 0x4E4F534A
BIN_CHUNK = 0x004E4942
GLB_MAGIC = 0x46546C67

COMP_FMT = {5120: "b", 5121: "B", 5122: "h", 5123: "H", 5125: "I", 5126: "f"}
COMP_SIZE = {5120: 1, 5121: 1, 5122: 2, 5123: 2, 5125: 4, 5126: 4}
NUM_COMP = {"SCALAR": 1, "VEC2": 2, "VEC3": 3, "VEC4": 4, "MAT4": 16}


def read_glb(path: str) -> tuple[dict, bytes]:
    with open(path, "rb") as f:
        data = f.read()
    magic, version, length = struct.unpack_from("<III", data, 0)
    if magic != GLB_MAGIC:
        raise SystemExit(f"{path}: 不是 GLB（magic={magic:#x}）")
    if version != 2:
        raise SystemExit(f"{path}: 只支持 glTF 2.0，得到 {version}")
    off, js, blob = 12, None, b""
    while off < min(length, len(data)):
        clen, ctype = struct.unpack_from("<II", data, off)
        chunk = data[off + 8: off + 8 + clen]
        if ctype == JSON_CHUNK:
            js = json.loads(chunk.decode("utf-8"))
        elif ctype == BIN_CHUNK:
            blob = chunk
        off += 8 + clen + ((4 - clen % 4) % 4 if clen % 4 else 0)
    if js is None:
        raise SystemExit(f"{path}: 没有 JSON chunk")
    return js, blob


def accessor(js: dict, blob: bytes, index: int):
    acc = js["accessors"][index]
    n = NUM_COMP[acc["type"]]
    fmt = COMP_FMT[acc["componentType"]]
    size = COMP_SIZE[acc["componentType"]]
    count = acc["count"]
    bv = js["bufferViews"][acc["bufferView"]]
    base = bv.get("byteOffset", 0) + acc.get("byteOffset", 0)
    stride = bv.get("byteStride") or (n * size)
    out = []
    for i in range(count):
        out.append(struct.unpack_from("<" + fmt * n, blob, base + i * stride))
    return out


def _mat_mul(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(4)) for j in range(4)] for i in range(4)]


def _quat_matrix(q):
    x, y, z, w = q
    return [
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w), 0.0],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w), 0.0],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y), 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ]


def node_matrix(node: dict):
    if "matrix" in node:
        m = node["matrix"]  # 列主序
        return [[m[c * 4 + r] for c in range(4)] for r in range(4)]
    t = node.get("translation") or [0, 0, 0]
    r = node.get("rotation") or [0, 0, 0, 1]
    s = node.get("scale") or [1, 1, 1]
    T = [[1, 0, 0, t[0]], [0, 1, 0, t[1]], [0, 0, 1, t[2]], [0, 0, 0, 1]]
    R = _quat_matrix(r)
    S = [[s[0], 0, 0, 0], [0, s[1], 0, 0], [0, 0, s[2], 0], [0, 0, 0, 1]]
    return _mat_mul(_mat_mul(T, R), S)


def triangulate(js: dict, blob: bytes, mesh_index: int, xf) -> list:
    tris = []
    for prim in js["meshes"][mesh_index]["primitives"]:
        mode = prim.get("mode", 4)
        if mode != 4:
            continue  # 只要三角形
        pos = accessor(js, blob, prim["attributes"]["POSITION"])
        idx = [i for (i,) in accessor(js, blob, prim["indices"])] if "indices" in prim else None
        if idx is None:
            idx = list(range(len(pos)))
        pts = []
        for p in pos:
            v = [xf[i][0] * p[0] + xf[i][1] * p[1] + xf[i][2] * p[2] + xf[i][3] for i in range(3)]
            pts.append(tuple(v))
        for k in range(0, len(idx) - 2, 3):
            tris.append((pts[idx[k]], pts[idx[k + 1]], pts[idx[k + 2]]))
    return tris


def walk(js: dict, blob: bytes, node_index: int, parent, out: list) -> None:
    node = js["nodes"][node_index]
    xf = _mat_mul(parent, node_matrix(node))
    if "mesh" in node:
        out.extend(triangulate(js, blob, node["mesh"], xf))
    for child in node.get("children", []):
        walk(js, blob, child, xf, out)


def decimate(tris, max_faces: int):
    """顶点聚类简化：格子在包围盒对角线里的边长从 1/64 起，逐次放大到面数达标。

    只为了「MuJoCo 能加载」——STL 解码器硬上限 20 万面/文件，而导出给查看器的 glb 里
    有 66 万面的整块腰。碰撞与视觉都不需要那个精度，所以按格子把顶点并掉。
    """
    if len(tris) <= max_faces:
        return tris
    xs = [p[0] for t in tris for p in t]
    ys = [p[1] for t in tris for p in t]
    zs = [p[2] for t in tris for p in t]
    diag = math.sqrt((max(xs) - min(xs)) ** 2 + (max(ys) - min(ys)) ** 2 + (max(zs) - min(zs)) ** 2) or 1.0
    for div in (64, 32, 16, 8, 4, 2, 1):
        h = diag / div
        out, seen = [], set()
        for a, b, c in tris:
            key = []
            for v in (a, b, c):
                key.append((round(v[0] / h), round(v[1] / h), round(v[2] / h)))
            if len(set(key)) != 3:
                continue
            flat = tuple(key)
            if flat in seen:
                continue
            seen.add(flat)
            out.append(tuple((k[0] * h, k[1] * h, k[2] * h) for k in key))
        if len(out) <= max_faces:
            return out
    return out


def normals(tris):
    out = []
    for a, b, c in tris:
        u = [b[i] - a[i] for i in range(3)]
        v = [c[i] - a[i] for i in range(3)]
        n = (u[1] * v[2] - u[2] * v[1], u[2] * v[0] - u[0] * v[2], u[0] * v[1] - u[1] * v[0])
        ln = math.sqrt(sum(x * x for x in n)) or 1.0
        out.append(tuple(x / ln for x in n))
    return out


def write_stl(path: str, tris) -> None:
    nrm = normals(tris)
    with open(path, "wb") as f:
        f.write(b"\0" * 80)
        f.write(struct.pack("<I", len(tris)))
        for n, (a, b, c) in zip(nrm, tris):
            f.write(struct.pack("<3f", *n))
            f.write(struct.pack("<9f", *(a + b + c)))
            f.write(b"\0\0")


def convert(src: str, dst: str, max_faces: int = 0) -> dict:
    js, blob = read_glb(src)
    scene = js.get("scenes", [{}])[js.get("scene", 0)]
    tris = []
    identity = [[1 if i == j else 0 for j in range(4)] for i in range(4)]
    for root in scene.get("nodes", []):
        walk(js, blob, root, identity, tris)
    if not tris:
        raise SystemExit(f"{src}: 没有三角面")
    raw = len(tris)
    if max_faces:
        tris = decimate(tris, max_faces)
    os.makedirs(os.path.dirname(os.path.abspath(dst)), exist_ok=True)
    write_stl(dst, tris)
    return {"triangles": len(tris), "raw": raw}


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("src", help="GLB 文件或目录")
    ap.add_argument("dst", help="输出目录")
    ap.add_argument("-q", "--quiet", action="store_true")
    ap.add_argument("--max-faces", type=int, default=150000,
                    help="每个 mesh 的面数上限（MuJoCo 的 STL 解码器硬上限 20 万），0=不简化")
    args = ap.parse_args()

    if os.path.isdir(args.src):
        files = sorted(f for f in os.listdir(args.src) if f.lower().endswith(".glb"))
        total = 0
        for f in files:
            out = os.path.join(args.dst, os.path.splitext(f)[0] + ".stl")
            info = convert(os.path.join(args.src, f), out, args.max_faces)
            total += info["triangles"]
            if not args.quiet:
                print(f"{f} → {os.path.basename(out)} ({info['triangles']}/{info['raw']} 面)")
        print(f"共 {len(files)} 个 mesh / {total} 个三角面 → {args.dst}")
        return 0
    info = convert(args.src, args.dst, args.max_faces)
    print(f"{info['triangles']} 个三角面 → {args.dst}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
