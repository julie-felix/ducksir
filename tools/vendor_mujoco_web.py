#!/usr/bin/env python3
"""把 MuJoCo 的 JS/WASM 绑定和浏览器用的 mesh 抓到本地 vendor（都不进 git）。

    python3 tools/vendor_mujoco_web.py            # @mujoco/mujoco 单线程构建 + 鸭子 STL

产物：
- `client/viewer/vendor/mujoco/{mujoco.js,mujoco.wasm}`（约 11 MB，来自 npm 包）
- `client/viewer/assets/mjcf/*.stl`（38 个，来自 third_party，约 21 MB）

为什么要拷 STL：MuJoCo 只吃 STL/OBJ，仓库里的 `assets/microduck/*.glb` 是给 three.js 用的；
浏览器里两边都要，所以物理这份单独放 `assets/mjcf/`，viewer 的 GLB 不动。

网络：registry.npmjs.org 直连通常比代理快一个数量级（2026-11-10 实测 521 KB/s vs 51 KB/s）。
"""

from __future__ import annotations

import argparse
import io
import json
import os
import shutil
import tarfile
import urllib.request

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
OUT_PKG = os.path.join(ROOT, "client/viewer/vendor/mujoco")
OUT_MESH = os.path.join(ROOT, "client/viewer/assets/mjcf")
STL_SRC = os.path.join(ROOT, "third_party/fanhao-microduck-replica/assets")
TAKE = ("package/mujoco.js", "package/mujoco.wasm")


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--version", default="3.17.0",
                    help="必须与宿主 Python 的 mujoco 版本一致——物理版本是配方的一部分（4.0.0 会让软 PD 步态失稳）")
    ap.add_argument("--proxy", help="例如 http://127.0.0.1:7890；缺省直连")
    args = ap.parse_args()

    url = f"https://registry.npmjs.org/@mujoco/mujoco/-/mujoco-{args.version}.tgz"
    opener = urllib.request.build_opener(
        urllib.request.ProxyHandler({"http": args.proxy, "https": args.proxy})) if args.proxy \
        else urllib.request.build_opener(urllib.request.ProxyHandler({}))
    print(f"下载 {url} …")
    with opener.open(url, timeout=180) as r:
        data = r.read()
    print(f"  {len(data) / 1e6:.1f} MB")

    os.makedirs(OUT_PKG, exist_ok=True)
    with tarfile.open(fileobj=io.BytesIO(data)) as tf:
        names = {m.name: m for m in tf.getmembers()}
        for want in TAKE:
            m = names.get(want)
            if not m:
                raise SystemExit(f"tgz 里没有 {want}")
            blob = tf.extractfile(m).read()
            dst = os.path.join(OUT_PKG, os.path.basename(want))
            with open(dst, "wb") as f:
                f.write(blob)
            print(f"  → {dst}（{len(blob) / 1e6:.1f} MB）")

    os.makedirs(OUT_MESH, exist_ok=True)
    n = 0
    for f in sorted(os.listdir(STL_SRC)):
        if f.lower().endswith(".stl"):
            shutil.copyfile(os.path.join(STL_SRC, f), os.path.join(OUT_MESH, f))
            n += 1
    print(f"  → {OUT_MESH}（{n} 个 STL）")

    with open(os.path.join(OUT_PKG, "manifest.json"), "w", encoding="utf-8") as f:
        json.dump({"package": "@mujoco/mujoco", "version": args.version, "files": list(TAKE)},
                  f, ensure_ascii=False, indent=2)
    print("好了")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
