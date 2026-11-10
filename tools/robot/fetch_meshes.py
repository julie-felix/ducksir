#!/usr/bin/env python3
"""把实体引用的 mesh 从上游拉下来（只拉引用的，不拉整个目录）。

    python3 tools/robot/fetch_meshes.py contract/robot/types/g1-29dof.yaml \
      --base-url https://raw.githubusercontent.com/unitreerobotics/unitree_ros/master/robots/g1_description/meshes \
      --ext STL --stage third_party/unitree-g1/meshes

产物：`<stage>/<name>.<ext>` + `<stage>/manifest.json`（name → sha256/size）。
已存在则跳过并重算 sha256，所以可以反复跑。

G1 的扩展名是大写 `.STL`（Linux 大小写敏感），鸭子是小写 `.stl`，都用 --ext 指定。
"""

from __future__ import annotations

import argparse
import concurrent.futures as cf
import hashlib
import json
import os
import sys
import urllib.error
import urllib.request

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import entity as E  # noqa: E402


def fetch_one(name: str, ext: str, base_url: str, stage: str, timeout: float) -> dict:
    os.makedirs(stage, exist_ok=True)
    path = os.path.join(stage, f"{name}.{ext}")
    url = f"{base_url.rstrip('/')}/{name}.{ext}"
    if not os.path.exists(path) or os.path.getsize(path) == 0:
        # 显式不用环境代理：github raw 直连更快，走代理反而常挂
        opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))
        try:
            # 必须带非 python 的 UA：raw.githubusercontent 对 Python-urllib 直接 404（不是 403）
            req = urllib.request.Request(url, headers={"User-Agent": "ducksir-fetch/0.1"})
            with opener.open(req, timeout=timeout) as resp, open(path, "wb") as f:
                f.write(resp.read())
        except (urllib.error.URLError, TimeoutError) as exc:
            return {"name": name, "ok": False, "error": str(exc), "url": url}
    return {"name": name, "ok": True, "file": os.path.relpath(path),
            "bytes": os.path.getsize(path), "sha256": E.sha256_file(path), "url": url}


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("entity")
    ap.add_argument("--base-url", required=True)
    ap.add_argument("--ext", default="stl")
    ap.add_argument("--stage", required=True)
    ap.add_argument("--workers", type=int, default=6)
    ap.add_argument("--timeout", type=float, default=45.0)
    args = ap.parse_args()

    doc = E.read_yaml(args.entity)
    names = sorted({v["mesh_name"] for l in doc["links"] for v in l["visual"] + l["collision"]
                    if v.get("mesh_name")})
    print(f"{len(names)} 个 mesh，从 {args.base_url} 拉")
    results = []
    with cf.ThreadPoolExecutor(max_workers=args.workers) as pool:
        futures = [pool.submit(fetch_one, n, args.ext, args.base_url, args.stage, args.timeout) for n in names]
        for fut in cf.as_completed(futures):
            results.append(fut.result())

    results.sort(key=lambda r: r["name"])
    ok = [r for r in results if r["ok"]]
    bad = [r for r in results if not r["ok"]]
    manifest = {r["name"]: {k: r[k] for k in ("file", "bytes", "sha256", "url")} for r in ok}
    out = os.path.join(args.stage, "manifest.json")
    with open(out, "w", encoding="utf-8") as f:
        json.dump(manifest, f, indent=1, ensure_ascii=False, sort_keys=True)
    total = sum(r["bytes"] for r in ok) / 1e6
    print(f"成功 {len(ok)} / 失败 {len(bad)}，共 {total:.1f} MB → {out}")
    for r in bad:
        print("  FAIL", r["name"], r["error"][:80])
    return 0 if not bad else 1


if __name__ == "__main__":
    raise SystemExit(main())
