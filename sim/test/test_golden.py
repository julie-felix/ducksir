#!/usr/bin/env python3
"""golden 快照比对：改完核跑这条，确认行为**没变**。

    PYTHONPATH=$HOME/.local/share/ducksir/pylibs python3 sim/test/test_golden.py
    ... --update-golden    # 有意改图/改核之后，重新记录基准（会写文件！）

和 `test_core_vs_oracle.py` 的分工：那条证明「算得对」（对着 numpy 参考实现），
这条证明「没变」（对着上一次的记录）。两条都要过。
"""

from __future__ import annotations

import argparse
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
sys.path.insert(0, os.path.join(ROOT, "sim", "plan"))
sys.path.insert(0, os.path.join(ROOT, "sim", "host"))

import golden  # noqa: E402

FIXTURE = os.path.join(ROOT, "contract/plan/fixtures/release-golden.json")
RELEASE = os.path.join(ROOT, "build/release/test")


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--update-golden", action="store_true", help="重新记录基准（会写文件）")
    ap.add_argument("--release", default=RELEASE, help="从哪个 release 目录取 plan")
    args = ap.parse_args()

    if not os.path.exists(os.path.join(args.release, "contract.json")):
        print(f"缺 {args.release}/contract.json —— 先跑 `bin/ducksir build --out {args.release}`")
        return 2

    cases = golden.cases_from_release(args.release)
    if args.update_golden:
        golden.record(cases, FIXTURE, note="2026-11-10 首版：pre/post/fsm 的 C 核输出")
        print(f"已更新快照 → {os.path.relpath(FIXTURE, ROOT)}")
        return 0

    problems = golden.check(cases, FIXTURE)
    if problems:
        for p in problems:
            print("  FAIL  " + p)
        print()
        print(f"快照对不上（{len(problems)} 条）。若是**有意**改图，跑 --update-golden 并说明原因。")
        return 1
    print(f"  PASS  {len(cases)} 个 plan 与快照逐位相同：" + ", ".join(f"{n}({len(f)} 帧)" for n, _, f in cases))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
