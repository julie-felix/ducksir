#!/usr/bin/env python3
"""把 onnxruntime-web 的运行时抓进本地 vendor（只取两个文件，不进 git）。

    python3 tools/vendor_ort_web.py            # 默认版本见下
    python3 tools/vendor_ort_web.py --version 1.31.1

为什么要这一步：viewer 是「静态页、不跑 npm」的（three.js 也是 vendor 进来的）。
ORT Web 的 dist 里我们只用两样东西：`ort.min.mjs` 和 `ort-wasm-simd-threaded.wasm`（约 15 MB），
所以直接下 npm 的 tgz 解开取文件，落 `client/viewer/vendor/ort/`，并写一份 manifest 记版本与 sha256。

网络提示：registry.npmjs.org **直连**通常比代理快一个数量级（2026-11-10 实测 521 KB/s vs 51 KB/s）。
本脚本默认直连，加 `--proxy http://127.0.0.1:7890` 才走代理。
"""

from __future__ import annotations

import argparse
import hashlib
import io
import json
import os
import tarfile
import urllib.request

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
OUT = os.path.join(ROOT, "client/viewer/vendor/ort")
# 只取 wasm-only 那一套：入口 + 胶水 .mjs + .wasm。
# 不要用 ort.min.mjs —— 那是含 WebGPU/WebNN 的默认构建，会去加载 jsep 那套（29 MB）的胶水。
TAKE = (
    "dist/ort.wasm.min.mjs",
    "dist/ort-wasm-simd-threaded.mjs",
    "dist/ort-wasm-simd-threaded.wasm",
)


def sha256(b: bytes) -> str:
    return hashlib.sha256(b).hexdigest()


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--version", default="1.31.1")
    ap.add_argument("--proxy", help="例如 http://127.0.0.1:7890；缺省直连")
    args = ap.parse_args()

    url = f"https://registry.npmjs.org/onnxruntime-web/-/onnxruntime-web-{args.version}.tgz"
    opener = urllib.request.build_opener(
        urllib.request.ProxyHandler({"http": args.proxy, "https": args.proxy})) if args.proxy \
        else urllib.request.build_opener(urllib.request.ProxyHandler({}))
    print(f"下载 {url} …")
    with opener.open(url, timeout=180) as r:
        data = r.read()
    print(f"  {len(data) / 1e6:.1f} MB")

    os.makedirs(OUT, exist_ok=True)
    manifest = {"package": "onnxruntime-web", "version": args.version, "files": {}}
    with tarfile.open(fileobj=io.BytesIO(data)) as tf:
        names = {n: m for n, m in ((m.name, m) for m in tf.getmembers())}
        for want in TAKE:
            member = names.get("package/" + want)
            if not member:
                raise SystemExit(f"tgz 里没有 {want}")
            blob = tf.extractfile(member).read()
            dst = os.path.join(OUT, os.path.basename(want))
            with open(dst, "wb") as f:
                f.write(blob)
            manifest["files"][os.path.basename(want)] = {"bytes": len(blob), "sha256": sha256(blob)}
            print(f"  → {dst}（{len(blob) / 1024 / 1024:.1f} MB）")
    with open(os.path.join(OUT, "manifest.json"), "w", encoding="utf-8") as f:
        json.dump(manifest, f, ensure_ascii=False, indent=2)
    print(f"写好了 {OUT}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
