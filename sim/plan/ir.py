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
    "constant": (9, 1),
    "select": (10, 1),
    "where": (11, 1),
    "phase_clock": (12, 1),
    "add": (13, 1),
    "state_machine": (14, 1),
    "compare": (15, 1),
    "motion": (16, 1),
    "delay": (17, 1),
    "model_rnn": (18, 1),
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
        """按索引重排 / 取样（gather）。

        允许子集：`reorder(x, [0])` 就是取第 0 维（切片的实现方式），`[3,1,2]` 是重排，
        `range(n)` 的排列是恒等。索引必须落在输入长度内。
        """
        idx = [int(i) for i in perm]
        if not idx:
            raise ValueError("reorder: perm 不能为空")
        if any(i < 0 or i >= ref.length for i in idx):
            raise ValueError(f"reorder: 索引越界（输入长 {ref.length}）")
        return self._add("reorder", [ref], len(idx), [float(i) for i in idx])

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

    def constant(self, values) -> NodeRef:
        """常值向量：botlab 的 `constant(0, size=10)`。"""
        vals = [float(v) for v in values]
        return self._add("constant", [], len(vals), vals)

    def select(self, mode: NodeRef, choices: dict, passive: int = -1,
               default_width: int | None = None) -> NodeRef:
        """按 mode 选一路：`choices={mode值: ref}`，最多 3 路（核里 concat/select 都受 4 输入限制，
        更多路要自己折）。mode 不匹配时输出全零（passive=-1）或指定那一路（passive=下标）。"""
        if len(choices) > 3:
            raise ValueError("select 一次最多 3 路，更多请先折叠")
        refs = list(choices.values())
        width = refs[0].length if refs else (default_width or mode.length)
        for r in refs:
            if r.length != width:
                raise ValueError("select 的几路宽度必须一致")
        if passive is not None and passive >= 0 and passive >= len(refs):
            raise ValueError("passive 下标越界")
        params = [float(k) for k in choices.keys()]
        params.append(float(width))
        params.append(float(-1 if passive is None else passive))
        return self._add("select", [mode] + refs, width, params)

    def where(self, cond: NodeRef, a: NodeRef, b: NodeRef) -> NodeRef:
        """cond 为真取 a，否则取 b（两边都会算，不做短路）。"""
        if cond.length != 1:
            raise ValueError("where 的条件必须是 1 维")
        if a.length != b.length:
            raise ValueError("where 的两路宽度必须一致")
        return self._add("where", [cond, a, b], a.length, [])

    def phase_clock(self, steps: NodeRef, period_steps: int) -> NodeRef:
        """把步数折成 [0,1) 的相位：botlab 的 `phase_clock(machine.steps, period_steps=...)`。"""
        if steps.length != 1:
            raise ValueError("phase_clock 的输入必须是 1 维步数")
        if period_steps < 1:
            raise ValueError("period_steps 至少 1")
        return self._add("phase_clock", [steps], 1, [float(period_steps)])

    def add(self, *refs: NodeRef) -> NodeRef:
        """逐元素相加（最多 4 路）。多策略分块选择后用它对消——没命中的那几路输出全零。"""
        if not refs:
            raise ValueError("add 至少要一个输入")
        width = refs[0].length
        for r in refs:
            if r.length != width:
                raise ValueError("add 各路宽度必须一致")
        return self._add("add", list(refs), width, [])

    def state_machine(self, command: NodeRef, initial: int, transitions: list,
                      conditions: list | None = None) -> NodeRef:
        """状态机（botlab 的 `state_machine`）。输出 2 维：[state, steps]。

        `transitions` 每项：`{from?, command?, after_steps?, when?, to, reset_steps?}`
        - `from` 缺省 = 任意状态；`command` 是脉冲（值相等即触发，优先级最高）；
        - `after_steps` 在当前状态待够多少步；`when` 是第几个 condition 信号为真（0/1）；
        - 同一行多个条件要同时满足；同级按声明顺序取首个命中；
        - `reset_steps` 缺省 True（切换后步数归零）。
        """
        if command.length != 1:
            raise ValueError("state_machine: command 必须 1 维")
        conds = conditions or []
        if len(conds) > 3:
            raise ValueError("state_machine: 最多 3 个条件信号（核里输入上限 4 = command + 3）")
        for c in conds:
            if c.length != 1:
                raise ValueError("state_machine: 条件信号必须 1 维")
        params = [float(len(transitions)), float(initial), float(len(conds))]
        for t in transitions:
            kind, value, ci = 0, 0.0, -1
            if t.get("command") is not None:
                kind, value = 0, float(t["command"])
            elif t.get("after_steps") is not None:
                kind, value = 1, float(t["after_steps"])
            elif t.get("when") is not None:
                kind, value, ci = 2, 0.0, int(t["when"])
            else:
                raise ValueError("transition 至少要给 command / after_steps / when 之一")
            params += [float(t.get("from", -1)), float(t["to"]), float(kind), value, float(ci),
                       float(1 if t.get("reset_steps", True) else 0)]
        return self._add("state_machine", [command] + conds, 2, params)

    def motion(self, time: NodeRef, frames, dim: int, fps: float, loop: bool = False) -> NodeRef:
        """动作参考采样（botlab 的 `Motion.sample` / `motion_reference` 的引擎侧等价物）。

        - 输入 `time` 是 1 维秒数（通常 `steps / control_hz`，可用 `scale`+`add` 拼出暂停/回绕）；
        - `frames` 是 (n_frames, dim) 的 float 数据，逐帧行主序；
        - 按 `t*fps` 取相邻两帧线性插值；`loop` 为真时环形回绕，否则夹在两端；
        - 输出 dim 维。

        为什么做成节点而不是在宿主里算：动作参考是**跨端**的输入（浏览器与宿主必须一致），
        放进图里才能被同一份 blob 两端复现。
        """
        # 不用 numpy：这个模块要保持「零第三方依赖」，CI 里没有 numpy 也要能 import
        rows = frames.tolist() if hasattr(frames, "tolist") else list(frames)
        rows = [[float(x) for x in row] for row in rows]
        if not rows or any(len(row) != dim for row in rows):
            shapes = sorted({len(row) for row in rows})
            raise ValueError(f"motion 数据每行宽度必须都是 {dim}（实际 {shapes}）")
        if fps <= 0:
            raise ValueError("motion fps 必须为正")
        if time.length != 1:
            raise ValueError("motion 的时间输入必须是 1 维")
        params = [float(len(rows)), float(dim), float(fps), 1.0 if loop else 0.0]
        params += [x for row in rows for x in row]
        return self._add("motion", [time], dim, params)

    def model_rnn(self, ref: NodeRef, action_len: int, *, h_len: int = 0, c_len: int = 0,
                  model_id: int = 0, flags: int = 0) -> NodeRef:
        """带状态槽的模型（LSTM/GRU）：h/c 存在 plan 里，跨步保持，`ds_reset` 清零。

        约定（核与后端双方都要遵守）：

            in  = [obs, h, c]          （obs 是本节点输入，h/c 是上一步的状态）
            out = [action, h_new, c_new]

        后端只认「一个输入张量、一个输出张量」，所以这台机器把拼接/拆分放在核里做；
        宿主/wasm 侧只要按同一约定包一层 onnxruntime 会话（多输入模型 → 拼成一个）。
        这让同一个 h_new 在下一步自然成为 h，不需要在同一时间步内成环。
        """
        if action_len < 1:
            raise ValueError("model_rnn 的 action_len 至少 1")
        if h_len < 0 or c_len < 0:
            raise ValueError("model_rnn 的状态维度不能为负")
        return self._add("model_rnn", [ref], action_len,
                         [float(model_id), float(h_len), float(c_len)], flags=flags)

    def delay(self, ref: NodeRef, frames: int = 1, fill=None) -> NodeRef:
        """N 步延迟（botlab 的 `delay`）：输出 `frames` 步之前的值，头 `frames` 步输出 `fill`。

        为什么需要它：递归模型（LSTM/GRU）要把它上一步的输出 `h_new/c_new` 接回下一步的输入。
        图是 DAG，不能在一个时间步内成环，所以用「延迟一步」把环拆开：

            concat(obs, h_prev, c_prev) → model → [action, h_new, c_new]
                                                     │
                                       delay(h_new) ─┘ → 下一步的 h_prev

        `ds_reset` 会把延迟线重新填成 `fill`，因此「从第 0 步重放」是确定的。
        """
        if frames < 1:
            raise ValueError("delay 的 frames 至少 1")
        fill_vals = fill if fill is not None else [0.0] * ref.length
        fill_vals = list(fill_vals)
        if len(fill_vals) != ref.length:
            raise ValueError(f"delay 的 fill 长度 {len(fill_vals)} != 输入宽度 {ref.length}")
        return self._add("delay", [ref], ref.length, [float(frames)] + [float(v) for v in fill_vals])

    def compare(self, a: NodeRef, op: str, b: NodeRef) -> NodeRef:
        """比较两路 1 维信号，输出 0/1。op: gt / gte / lt / lte / eq。"""
        ops = {"gt": 0, "gte": 1, "lt": 2, "lte": 3, "eq": 4}
        if op not in ops:
            raise ValueError(f"compare: 未知 op {op}")
        if a.length != 1 or b.length != 1:
            raise ValueError("compare: 两路都必须是 1 维")
        return self._add("compare", [a, b], 1, [float(ops[op])])

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
