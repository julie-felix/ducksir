#!/usr/bin/env python3
"""UI 单源检查：键位与指令区间**只能来自契约**，不能写在页面代码里。

背景：A/D 到底是「转向」还是「横移」，我在三天里改错过三次。每次都是把模板
`config/io.yaml` 的键位临时抄进页面，抄错一处就全错。正确的唯一来源是
`contract.json` 的 `robot.ui.bindings` / `robot.ui.*.min|max`。

规则（命中即失败）：

R1 `client/viewer/host.js` 里用字面量字母直接决定语义
   （`held.has('a')`、`key === 'd'`）——必须走 `uiSpec.bindings`。
R2 `client/viewer/host.html` 的滑杆 `min` 是负常量——负区间只能来自契约。
R3 `client/viewer/*.js` 里出现 botlab 的键名（`KeyW` / `KeyA` …）——模板键位应进 preset。

豁免：同一行末尾写 `// ui-single-source: allow <理由>`。
"""
from __future__ import annotations

import os
import re
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
VIEWER = os.path.join(ROOT, "client", "viewer")
ALLOW = "ui-single-source: allow"

RULES = [
    ("R1", os.path.join(VIEWER, "host.js"),
     re.compile(r"""(held\.has\(\s*['"][wasdqe]['"]\s*\)|key\s*===\s*['"][wasdqe]['"])"""),
     "键位语义必须来自 contract.ui.bindings，不能写字面量"),
    ("R2", os.path.join(VIEWER, "host.html"),
     re.compile(r"""<input[^>]*id="(fwd|yaw|lat)"[^>]*min="-\d"""),
     "滑杆负区间必须来自契约（找不到契约时才用 0 起步）"),
    ("R3", VIEWER,
     re.compile(r"""['"]Key[WASDQE]['"]"""),
     "botlab 键名属于 preset/契约，不该出现在页面代码里"),
]


def _iter_files(target: str):
    if os.path.isfile(target):
        yield target
        return
    for name in sorted(os.listdir(target)):
        if name.endswith((".js", ".html")):
            yield os.path.join(target, name)


def main() -> int:
    fails = []
    for rule, target, pattern, reason in RULES:
        if not os.path.exists(target):
            continue
        for path in _iter_files(target):
            with open(path, encoding="utf-8") as f:
                for lineno, line in enumerate(f, 1):
                    if ALLOW in line:
                        continue
                    if pattern.search(line):
                        fails.append((rule, os.path.relpath(path, ROOT), lineno,
                                      line.strip()[:100], reason))
    if fails:
        print("UI 单源检查失败：")
        for rule, path, lineno, snippet, reason in fails:
            print(f"  ✗ [{rule}] {path}:{lineno}  {reason}")
            print(f"        {snippet}")
        print("\n只在契约里改键位/区间；页面按 ui.bindings 与 ui.*.min/max 驱动。")
        return 1
    print("✓ UI 单源检查通过：键位与区间都来自契约")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
