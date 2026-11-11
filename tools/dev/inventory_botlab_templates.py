#!/usr/bin/env python3
"""把下载下来的 botlab 模板做一份机器可读的清点：文件树、ONNX 输入输出、配置、动作数据。

用法：
    python3 tools/dev/inventory_botlab_templates.py /tmp/bt-all > /tmp/inventory.json

产物是 JSON（供写分析笔记时引用），不写进仓库。
"""
from __future__ import annotations

import json
import os
import pathlib
import sys

MESH_SUFFIX = {".stl", ".STL", ".obj", ".png", ".jpg", ".jpeg", ".mtl", ".dae", ".glb", ".bin"}


def human(n: float) -> str:
    for unit in ("B", "KB", "MB", "GB"):
        if n < 1024:
            return f"{n:.1f}{unit}"
        n /= 1024
    return f"{n:.1f}TB"


def walk(root: pathlib.Path) -> dict:
    files = []
    for path in sorted(root.rglob("*")):
        if path.is_file():
            rel = path.relative_to(root).as_posix()
            files.append({"path": rel, "size": path.stat().st_size})
    return {"files": files, "total": sum(f["size"] for f in files)}


def onnx_io(path: pathlib.Path) -> dict | None:
    try:
        import onnxruntime as ort
    except Exception:
        return None
    try:
        sess = ort.InferenceSession(str(path), providers=["CPUExecutionProvider"])
        return {
            "inputs": [{"name": i.name, "shape": [str(d) for d in i.shape]} for i in sess.get_inputs()],
            "outputs": [{"name": o.name, "shape": [str(d) for d in o.shape]} for o in sess.get_outputs()],
        }
    except Exception as e:  # 大模型缺外部数据等
        return {"error": str(e)[:160]}


def read_text(path: pathlib.Path, limit: int = 20000) -> str | None:
    try:
        return path.read_text(encoding="utf-8")[:limit]
    except Exception:
        return None


def motion_meta(root: pathlib.Path) -> list[dict]:
    out = []
    for meta in sorted(root.rglob("metadata.json")):
        try:
            out.append({"path": meta.relative_to(root).as_posix(), "meta": json.loads(meta.read_text())})
        except Exception:
            pass
    return out


def main() -> int:
    base = pathlib.Path(sys.argv[1] if len(sys.argv) > 1 else "/tmp/bt-all")
    inv = {}
    for proj in sorted(p for p in base.iterdir() if p.is_dir()):
        tree = walk(proj)
        files = [f["path"] for f in tree["files"]]
        models = [f for f in files if f.endswith(".onnx")]
        inv[proj.name] = {
            "file_count": len(files),
            "total_bytes": tree["total"],
            "total_human": human(tree["total"]),
            "meshes": sum(1 for f in files if pathlib.Path(f).suffix in MESH_SUFFIX),
            "onnx": {m: onnx_io(proj / m) for m in models},
            "controller_py": read_text(proj / "controller.py"),
            "main_py": read_text(proj / "main.py"),
            "configs": {f: read_text(proj / f, 4000) for f in files
                        if f.startswith("config/") and f.endswith((".yaml", ".yml"))},
            "presets": [f for f in files if f.endswith(".preset.yaml")],
            "motion_meta": motion_meta(proj),
            "project_json": None,
            "tree": files,
        }
        pj = proj / "botlab.project.json"
        if pj.exists():
            try:
                inv[proj.name]["project_json"] = json.loads(pj.read_text())
            except Exception:
                pass
    json.dump(inv, sys.stdout, ensure_ascii=False, indent=1)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
