#!/usr/bin/env python3
"""计算图：Python 声明 → plan JSON（源码）→ blob（产物）。

    from ir import Plan
    p = Plan(input_dim=20, output_dim=14)
    q = p.obs(7, 14)                       # obs 缓冲里的一段
    t = p.affine(q, scale=[1]*14, offset=[0]*14)
    a = p.clip(t, [-0.5]*14, [0.5]*14)
    p.out_action(a)
    p.to_blob()                            # 给 C++ 核加载

两副形态（计划 D11）：
- **JSON** 是可读的源码：能 diff、能 review、能给 AI 读；
- **blob** 是产物：定长记录 + 固定偏移，C 核零解析直接跑。

版本与摘要：blob 头里有 `version` / `abi` / `required`（节点类型 + 最低版本）/ `digest`（FNV-1a 64，
非加密用途，只防「包被改过/传错版本」）。老核遇到不认识的节点会干净拒绝（`ds_load` 返回 0），不是跑一半。

W4 的 8 个节点：OBS / CONCAT / AFFINE / CLIP / REORDER / HISTORY / MODEL / OUT_ACTION。
语义在 `contract/plan/README.md`，numpy 参考实现在 `sim/plan/oracle.py`，两边逐点对拍。
"""

from __future__ import annotations

import hashlib
import json
import struct

MAGIC = b"DSIM"
VERSION = 1
ABI = 1
HEADER_SIZE = 48
NODE_SIZE = 32
MAX_INPUTS = 4

# 节点类型：名字 ↔ id ↔ 该节点从哪个版本开始有
NODE_TYPES = {
    "obs": (1, 1),
    "concat": (2, 1),
    "affine": (3, 1),
    "clip": (4, 1),
    "reorder": (5, 1),
    "history": (6, 1),
    "model": (7, 1),
    "out_action": (8, 1),
}
TYPE_BY_ID = {tid: name for name, (tid, _) in NODE_TYPES.items()}
NODE_VERSION = {name: ver for name, (_, ver) in NODE_TYPES.items()}

FLAG_PASS_THROUGH = 1 << 0   # 仅测试用：model 节点不调后端，直接拷贝输入
FLAG_NATIVE_ONLY = 1 << 1    # 逃生口：出部署产物时必须拒绝（W6 起用）


def fnv1a64(data: bytes, seed: int = 0xCBF29CE484222325) -> int:
    h = seed
    for b in data:
        h ^= b
        h = (h * 0x100000001B3) & 0xFFFFFFFFFFFFFFFF
    return h


class NodeRef:
    """一个节点的输出，供下游引用。off/len 是 arena 里的 float 下标与长度。"""

    __slots__ = ("node", "off", "length")

    def __init__(self, node: int, off: int, length: int):
        self.node = node
        self.off = off
        self.length = length

    def __repr__(self) -> str:
        return f"<node#{self.node} @{self.off}+{self.length}>"


class Plan:
    def __init__(self, input_dim: int, output_dim: int, name: str = "plan"):
        self.name = name
        self.input_dim = input_dim
        self.output_dim = output_dim
        self.nodes: list[dict] = []
        self.params: list[float] = []
        self._arena = 0

    # ---------- 内部 ----------
    def _alloc(self, length: int) -> int:
        if length <= 0:
            raise ValueError("节点输出长度必须 > 0")
        off = self._arena
        self._arena += length
        return off

    def _param(self, values) -> tuple[int, int]:
        vals = [float(v) for v in values]
        off = len(self.params)
        self.params.extend(vals)
        return off, len(vals)

    def _add(self, type_name: str, inputs: list[NodeRef], out_len: int,
             param: list[float], flags: int = 0) -> NodeRef:
        if len(inputs) > MAX_INPUTS:
            raise ValueError(f"{type_name}: 最多 {MAX_INPUTS} 个输入")
        if len(param) > 0xFFFF:
            raise ValueError("param 太长")
        off = self._alloc(out_len)
        p_off, p_len = self._param(param)
        idx = len(self.nodes)
        self.nodes.append({
            "type": NODE_TYPES[type_name][0],
            "name": type_name,
            "flags": flags,
            "inputs": [{"off": n.off, "len": n.length} for n in inputs],
            "out": {"off": off, "len": out_len},
            "param": {"off": p_off, "len": p_len},
        })
        return NodeRef(idx, off, out_len)

    # ---------- 8 个节点 ----------
    def obs(self, src_off: int, length: int) -> NodeRef:
        """从 obs 缓冲里搬一段出来（语义由上层 manifest 决定，核只做搬运）。"""
        if src_off < 0 or src_off + length > self.input_dim:
            raise ValueError(f"obs 越界：{src_off}+{length} > {self.input_dim}")
        return self._add("obs", [], length, [float(src_off), float(length)])

    def concat(self, *refs: NodeRef) -> NodeRef:
        if not refs:
            raise ValueError("concat 至少要一个输入")
        return self._add("concat", list(refs), sum(r.length for r in refs), [])

    def affine(self, ref: NodeRef, scale, offset) -> NodeRef:
        s, o = _fit(scale, ref.length, "scale"), _fit(offset, ref.length, "offset")
        return self._add("affine", [ref], ref.length, list(s) + list(o))

    def clip(self, ref: NodeRef, lo, hi) -> NodeRef:
        l, h = _fit(lo, ref.length, "lo"), _fit(hi, ref.length, "hi")
        if any(a > b for a, b in zip(l, h)):
            raise ValueError("clip: lo > hi")
        return self._add("clip", [ref], ref.length, list(l) + list(h))

    def reorder(self, ref: NodeRef, perm) -> NodeRef:
        idx = [int(i) for i in perm]
        if len(idx) != ref.length or sorted(idx) != list(range(ref.length)):
            raise ValueError("reorder: perm 必须是 0..n-1 的排列")
        return self._add("reorder", [ref], ref.length, [float(i) for i in idx])

    def history(self, ref: NodeRef, frames: int) -> NodeRef:
        """环形缓冲：输出 = 最近 frames 帧按 by-frame 铺开（最老在前）。"""
        if frames < 1:
            raise ValueError("history: frames 至少 1")
        return self._add("history", [ref], ref.length * frames, [float(frames)])

    def model(self, ref: NodeRef, out_len: int, model_id: int = 0,
              pass_through: bool = False) -> NodeRef:
        flags = FLAG_PASS_THROUGH if pass_through else 0
        if pass_through and out_len != ref.length:
            raise ValueError("pass_through 要求 out_len == 输入长度")
        return self._add("model", [ref], out_len, [float(model_id)], flags)

    def out_action(self, ref: NodeRef) -> NodeRef:
        if any(n["name"] == "out_action" for n in self.nodes):
            raise ValueError("一个 plan 只能有一个 out_action")
        if ref.length != self.output_dim:
            raise ValueError(f"out_action 长度 {ref.length} != output_dim {self.output_dim}")
        return self._add("out_action", [ref], ref.length, [])

    # ---------- 导出 ----------
    def to_json(self) -> dict:
        if not any(n["name"] == "out_action" for n in self.nodes):
            raise ValueError("plan 缺 out_action")
        return {
            "kind": "ducksim-plan",
            "version": VERSION,
            "abi": ABI,
            "name": self.name,
            "input_dim": self.input_dim,
            "output_dim": self.output_dim,
            "arena_len": self._arena,
            "nodes": self.nodes,
            "params": self.params,
        }

    def to_blob(self) -> bytes:
        return compile_blob(self.to_json())


def _fit(values, n: int, what: str) -> list[float]:
    vals = [float(v) for v in values]
    if len(vals) == 1:
        return vals * n
    if len(vals) != n:
        raise ValueError(f"{what} 长度 {len(vals)} != 节点长度 {n}")
    return vals


def validate(doc: dict) -> list[str]:
    """结构校验：C 核加载时做同一套（顺序、边界、唯一 sink）。这里先在人这一侧挡住。"""
    errs: list[str] = []
    if doc.get("kind") != "ducksim-plan":
        errs.append("kind 不是 ducksim-plan")
    if doc.get("version") != VERSION:
        errs.append(f"version {doc.get('version')} != {VERSION}")
    nodes = doc.get("nodes") or []
    if not nodes:
        errs.append("没有节点")
        return errs
    producers: list[tuple[int, int, int]] = []   # (node, off, len)
    sinks = 0
    for i, nd in enumerate(nodes):
        name = nd.get("name")
        if name not in NODE_TYPES:
            errs.append(f"#{i} 未知节点 {name}")
            continue
        if len(nd["inputs"]) > MAX_INPUTS:
            errs.append(f"#{i} 输入超过 {MAX_INPUTS}")
        for k, inp in enumerate(nd["inputs"]):
            if not any(p[1] == inp["off"] and p[2] == inp["len"] for p in producers):
                errs.append(f"#{i} 第 {k} 个输入 @{inp['off']}+{inp['len']} 不是任何前序节点的输出")
        out = nd["out"]
        if out["off"] + out["len"] > doc["arena_len"]:
            errs.append(f"#{i} 输出越界")
        producers.append((i, out["off"], out["len"]))
        if name == "out_action":
            sinks += 1
            if out["len"] != doc["output_dim"]:
                errs.append(f"out_action 长度 {out['len']} != output_dim")
    if sinks != 1:
        errs.append(f"out_action 数量 {sinks} != 1")
    return errs


def compile_blob(doc: dict) -> bytes:
    errs = validate(doc)
    if errs:
        raise ValueError("plan 校验失败：\n  " + "\n  ".join(errs))

    nodes = doc["nodes"]
    params = doc["params"]
    required = sorted({(NODE_TYPES[n["name"]][0], NODE_VERSION[n["name"]]) for n in nodes})

    body = bytearray()
    for tid, ver in required:
        body += struct.pack("<HH", tid, ver)
    for nd in nodes:
        ins = nd["inputs"] + [{"off": 0, "len": 0}] * (MAX_INPUTS - len(nd["inputs"]))
        rec = struct.pack(
            "<HBB" + "HH" * MAX_INPUTS + "HHHH",
            nd["type"], len(nd["inputs"]), nd["flags"],
            *[v for i in ins for v in (i["off"], i["len"])],
            nd["out"]["off"], nd["out"]["len"], nd["param"]["off"], nd["param"]["len"],
        )
        body += rec + b"\0" * (NODE_SIZE - len(rec))
    body += struct.pack(f"<{len(params)}f", *params)

    digest = fnv1a64(bytes(body))
    head = bytearray(HEADER_SIZE)
    struct.pack_into("<4s", head, 0, MAGIC)
    struct.pack_into("<H", head, 4, VERSION)
    struct.pack_into("<H", head, 6, ABI)
    struct.pack_into("<H", head, 8, len(nodes))
    struct.pack_into("<H", head, 10, len(required))
    struct.pack_into("<H", head, 12, doc["input_dim"])
    struct.pack_into("<H", head, 14, doc["output_dim"])
    struct.pack_into("<I", head, 16, doc["arena_len"])
    struct.pack_into("<I", head, 20, len(params))
    struct.pack_into("<I", head, 24, 0)          # flags
    struct.pack_into("<I", head, 28, 0)          # reserved0
    struct.pack_into("<I", head, 32, 0)          # reserved1
    struct.pack_into("<I", head, 36, 0)          # reserved2
    struct.pack_into("<Q", head, 40, digest)
    return bytes(head) + bytes(body)


def read_blob(blob: bytes) -> dict:
    """反向解析（Python 侧自检 + 给测试用）。C 核里有等价实现。"""
    if len(blob) < HEADER_SIZE:
        raise ValueError("blob 太短")
    magic = blob[0:4]
    version, abi, node_count, req_count, input_dim, output_dim = struct.unpack_from("<6H", blob, 4)
    arena_len, param_len, flags = struct.unpack_from("<3I", blob, 16)
    digest = struct.unpack_from("<Q", blob, 40)[0]
    if magic != MAGIC:
        raise ValueError(f"magic 不对：{magic!r}")
    if version != VERSION or abi != ABI:
        raise ValueError(f"版本不匹配：version={version} abi={abi}")
    pos = HEADER_SIZE
    body = blob[pos:]
    if fnv1a64(body) != digest:
        raise ValueError("digest 不匹配（包被改过或版本串了）")
    required = [struct.unpack_from("<HH", body, i * 4) for i in range(req_count)]
    pos = req_count * 4
    nodes = []
    for i in range(node_count):
        rec = struct.unpack_from("<HBB" + "HH" * MAX_INPUTS + "HHHH", body, pos + i * NODE_SIZE)
        tid, n_in, nflags = rec[0], rec[1], rec[2]
        pairs = rec[3:3 + MAX_INPUTS * 2]
        out_off, out_len, p_off, p_len = rec[3 + MAX_INPUTS * 2:]
        nodes.append({
            "type": tid, "name": TYPE_BY_ID.get(tid, f"<{tid}>"), "flags": nflags,
            "inputs": [{"off": pairs[2 * k], "len": pairs[2 * k + 1]} for k in range(n_in)],
            "out": {"off": out_off, "len": out_len},
            "param": {"off": p_off, "len": p_len},
        })
    pos += node_count * NODE_SIZE
    params = list(struct.unpack_from(f"<{param_len}f", body, pos)) if param_len else []
    return {
        "version": version, "abi": abi, "input_dim": input_dim, "output_dim": output_dim,
        "arena_len": arena_len, "required": required, "nodes": nodes, "params": params,
        "digest": digest,
    }


def sha256_of_blob(blob: bytes) -> str:
    return hashlib.sha256(blob).hexdigest()


def save(doc: dict, json_path: str, blob_path: str | None = None) -> str:
    with open(json_path, "w", encoding="utf-8") as f:
        json.dump(doc, f, ensure_ascii=False, indent=2)
    if blob_path:
        blob = compile_blob(doc)
        with open(blob_path, "wb") as f:
            f.write(blob)
        return sha256_of_blob(blob)
    return ""
