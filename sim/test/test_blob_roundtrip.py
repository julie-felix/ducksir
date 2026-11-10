#!/usr/bin/env python3
"""blob 往返自检（纯标准库，CI 里跑这个；numpy 那套对拍在本地跑）。

    python3 sim/test/test_blob_roundtrip.py

验四件事：
1. `ir.compile_blob` 造出来的 blob 能被 `ir.read_blob` 原样读回（节点数/维度/参数/摘要一致）；
2. digest 是覆盖 payload 的：改一个字节就读不回来；
3. 版本/ABI 不对时拒绝；
4. 缺 `out_action`、输入不是前序输出这类结构错，编译期就挡住。
"""

from __future__ import annotations

import os
import struct
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(os.path.dirname(HERE), "plan"))

import ir  # noqa: E402


def build_sample() -> dict:
    p = ir.Plan(input_dim=6, output_dim=6, name="roundtrip")
    a = p.obs(0, 6)
    b = p.affine(a, [2.0], [0.5])
    c = p.clip(b, [-1.0], [1.0])
    p.out_action(c)
    return p.to_json()


def main() -> int:
    oks = []

    doc = build_sample()
    blob = ir.compile_blob(doc)
    back = ir.read_blob(blob)
    same = (
        back["input_dim"] == doc["input_dim"]
        and back["output_dim"] == doc["output_dim"]
        and back["arena_len"] == doc["arena_len"]
        and len(back["nodes"]) == len(doc["nodes"])
        and len(back["params"]) == len(doc["params"])
    )
    oks.append(same)
    print(f"  往返读回              {'PASS' if same else 'FAIL'}  "
          f"{len(blob)} B / {len(back['nodes'])} 节点 / digest {back['digest']:#x}")

    tampered = bytearray(blob)
    tampered[-1] ^= 0xFF
    try:
        ir.read_blob(bytes(tampered))
        oks.append(False)
        print("  改一字节要读不回来      FAIL（竟然读回来了）")
    except ValueError as e:
        oks.append("digest" in str(e))
        print(f"  改一字节要读不回来      PASS（{e}）")

    bad_ver = bytearray(blob)
    struct.pack_into("<H", bad_ver, 4, 99)
    try:
        ir.read_blob(bytes(bad_ver))
        oks.append(False)
        print("  版本不对要拒绝          FAIL")
    except ValueError as e:
        oks.append(True)
        print(f"  版本不对要拒绝          PASS（{e}）")

    p = ir.Plan(input_dim=3, output_dim=3)
    a = p.obs(0, 3)
    p.affine(a, [1.0], [0.0])
    no_sink = {"kind": "ducksim-plan", "version": 1, "abi": 1, "name": "x",
               "input_dim": 3, "output_dim": 3, "arena_len": p._arena,
               "nodes": p.nodes, "params": p.params}
    try:
        ir.compile_blob(no_sink)
        oks.append(False)
        print("  缺 out_action 要失败     FAIL")
    except ValueError:
        oks.append(True)
        print("  缺 out_action 要失败     PASS")

    # 输入不接前序输出
    p2 = ir.Plan(input_dim=4, output_dim=2)
    a = p2.obs(0, 2)
    p2.out_action(a)
    bad = p2.to_json()
    bad["nodes"][1]["inputs"][0]["off"] = 99
    try:
        ir.compile_blob(bad)
        oks.append(False)
        print("  输入悬空要失败          FAIL")
    except ValueError:
        oks.append(True)
        print("  输入悬空要失败          PASS")

    print(f"\n{'全部通过' if all(oks) else '有失败'}（{sum(oks)}/{len(oks)}）")
    return 0 if all(oks) else 1


if __name__ == "__main__":
    raise SystemExit(main())
