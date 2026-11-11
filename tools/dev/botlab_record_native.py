#!/usr/bin/env python3
"""让 **botlab 自己的运行时**跑一遍模板，并把每次推理的 feeds/输出录下来，作为 sim2sim 的真值。

为什么这样做：光看源码只能猜 obs 的缩放/顺序；录一份真值，就能逐点对拍。

准备（一次）：

    # botlab 的 wheel 解到某目录，再用 PYTHONPATH 挂进来
    unzip botlab_python-<ver>-py3-none-any.whl -d /tmp/botlab-whl
    # 它缺两个纯 python 依赖，直接从 PyPI 下 wheel 解到 pylibs
    #   rfc8785（canonical json）、websockets（本地运行时要）

跑：

    DUCK_CAP=/tmp/botlab-cap.jsonl PYTHONPATH=/tmp/botlab-whl:$HOME/.local/share/ducksir/pylibs \\
      BOTLAB_STEPS=150 python3 tools/dev/botlab_record_native.py <项目目录>

产物：每行一条 JSON（`in` / `out` / `in_len` / `out_len`）。

已知：录 WBC 模板时要把 controller 里的 `templates/g1-wbc/...` 前缀去掉（模板在站点上跑，
路径是站点相对的），robot 用 `assets/robot/*.xml` 这种项目相对路径。
"""
import json
import os
import sys

import onnxruntime as ort

OUT = os.environ.get("DUCK_CAP", "/tmp/botlab-cap.jsonl")
_orig = ort.InferenceSession.run
_state = {"n": 0, "fh": open(OUT, "w", encoding="utf-8")}


def run(self, output_names, input_feed, run_options=None):
    res = _orig(self, output_names, input_feed, run_options)
    try:
        rec = {
            "step": _state["n"],
            "in": {k: [round(float(x), 6) for x in v.reshape(-1)[:4096]] for k, v in input_feed.items()},
            "in_len": {k: int(v.size) for k, v in input_feed.items()},
            "out": {k: [round(float(x), 6) for x in v.reshape(-1)[:64]]
                    for k, v in zip(output_names, res)},
            "out_len": {k: int(v.size) for k, v in zip(output_names, res)},
        }
        _state["fh"].write(json.dumps(rec) + "\n")
        _state["fh"].flush()
        _state["n"] += 1
    except Exception as e:  # 录失败不能影响他们跑
        _state["fh"].write(json.dumps({"error": str(e)[:200]}) + "\n")
    return res


ort.InferenceSession.run = run

if __name__ == "__main__":
    from botlab._cli import main

    project = sys.argv[1] if len(sys.argv) > 1 else "."
    sys.argv = ["botlab", "run", "--headless", project]
    try:
        main()
    except SystemExit as e:
        print("exit", e.code)
