#!/usr/bin/env python3
"""高层构图 API：说明意图，别手动算偏移。

    from graph import Graph
    g = Graph(policy_manifest=man, preset=preset)
    g.observe_all()                       # 按 manifest 的 obs 段接线（顺序/偏移自动）
    g.policy("walking", mode=0)           # 调模型
    g.action(scale="preset", clip="ctrlrange")
    blob = g.build()                      # 编译成 blob，核里跑的东西和 ir.Plan 手搓的一样

为什么有这一层（Q16 的决定）：`ir.Plan` 是「汇编级」——`p.obs(48, 13)` 这种偏移写错了不报错、
只是行为诡异。而 Q9=B 之后「图」是唯一的扩展出口，写起来像汇编就没人想扩展它。

**两层是等价的**：`Graph.standard()` 产出的节点序列与 `run_policy.build_plan()` 逐节点相同，
`test_graph_equiv.py` 直接比 blob 摘要。`ir.Plan` 保留不动，作为底座与调试入口。
"""

from __future__ import annotations

import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import ir  # noqa: E402


class GraphError(ValueError):
    """构图错误（比 io 错误更早暴露，信息更具体）。"""


class Graph:
    """声明一张策略图。

    `policy_manifest`（dict）给观测布局与动作映射；`preset`（dict）给动作偏置与限位。
    也可以不传 manifest，用 `observe(...)` 手工点名槽位——但那样就失去了「自动算偏移」的意义。
    """

    def __init__(self, policy_manifest: dict | None = None, preset: dict | None = None,
                 name: str = "graph", expose_raw_action: bool = False):
        self.manifest = policy_manifest or {}
        self.preset = preset or {}
        self.name = name
        self._obs_refs: list[tuple[str, ir.NodeRef]] = []
        self._model_ref: ir.NodeRef | None = None
        self._action_ref: ir.NodeRef | None = None
        self._built = False
        self.expose_raw_action = expose_raw_action
        total = sum(o["len"] for o in self.manifest.get("obs") or [])
        self._out_len = int(self.preset.get("actionDimension")
                            or len(self.preset.get("defaultJointPositions") or []) or 14)
        self._plan = ir.Plan(input_dim=total or 61, output_dim=self._out_len, name=name)

    # ── 观测 ────────────────────────────────────────────────
    def observe_all(self) -> "Graph":
        """按 manifest 的 obs 段顺序接线（这是默认做法）。"""
        obs = self.manifest.get("obs") or []
        if not obs:
            raise GraphError("没有 manifest 的 obs 段；手工请用 observe(...)")
        p = self._plan
        off = 0
        for o in obs:
            ref = p.obs(off, o["len"])
            self._obs_refs.append((o["slot"], ref))
            off += o["len"]
        return self

    def observe(self, *slots: str) -> "Graph":
        """点名要哪些槽位（顺序 = manifest 里的顺序，不是参数顺序）。"""
        obs = self.manifest.get("obs") or []
        by_slot = {}
        off = 0
        for o in obs:
            by_slot[o["slot"]] = (off, o["len"])
            off += o["len"]
        for name in slots:
            if name not in by_slot:
                raise GraphError(f"manifest 里没有观测槽 {name}（有：{', '.join(by_slot)}）")
            off_, length = by_slot[name]
            self._obs_refs.append((name, self._plan.obs(off_, length)))
        return self

    # ── 拼接 / 模型 ─────────────────────────────────────────
    def _concat_all(self, refs: list[ir.NodeRef]) -> ir.NodeRef:
        """核里 concat 最多 4 输入，层层折叠（与 run_policy.build_plan 的折叠规则一致）。"""
        p = self._plan
        if len(refs) == 1:
            return refs[0]
        level = list(refs)
        while len(level) > 4:
            nxt = []
            for i in range(0, len(level), 4):
                chunk = level[i:i + 4]
                nxt.append(chunk[0] if len(chunk) == 1 else p.concat(*chunk))
            level = nxt
        return p.concat(*level)

    def concat(self) -> ir.NodeRef:
        """把已声明的观测按声明顺序拼起来。"""
        if not self._obs_refs:
            raise GraphError("还没 observe(...)")
        return self._concat_all([r for _, r in self._obs_refs])

    def policy(self, key: str | None = None, *, mode: int = 0, out_len: int | None = None) -> "Graph":
        """调模型。`out_len` 默认取预设的动作维度。"""
        n = out_len or int(self.preset.get("actionDimension")
                           or len(self.preset.get("defaultJointPositions") or []) or 14)
        h = self.manifest.get("history") or {}
        frames = int(h.get("length", 0))
        x = self.concat()
        if frames > 0:
            if h.get("layout") != "by-frame":
                raise GraphError(f"history.layout 只支持 by-frame（现在 {h.get('layout')}）")
            x = self._plan.history(x, frames)
        self._model_ref = self._plan.model(x, out_len=n, model_id=mode)
        self._action_ref = self._model_ref
        self._out_len = n
        return self

    # ── 动作映射 ────────────────────────────────────────────
    def motion(self, time, frames, *, fps: float, loop: bool = False) -> "Graph":
        """加一路动作参考采样（`ir.Plan.motion`）：时间→当前帧，线性插值。"""
        import numpy as _np
        arr = _np.asarray(frames, dtype=_np.float32)
        self._action_ref = self._plan.motion(time, arr, arr.shape[1], fps, loop)
        return self

    def action(self, *, scale="preset", clip="ctrlrange", ctrl_lo=None, ctrl_hi=None) -> "Graph":
        """`ctrl = 站姿 + 缩放 × 动作`，再按 ctrlrange 夹。"""
        if self._action_ref is None:
            raise GraphError("还没 policy(...)")
        preset = self.preset
        scale_vals = preset["actionScale"] if scale == "preset" else scale
        offset = preset["defaultJointPositions"]
        if not offset:
            raise GraphError("preset 里没有 defaultJointPositions")
        s = self._plan.affine(self._action_ref, scale_vals, offset)
        if clip == "ctrlrange":
            lo, hi = ctrl_lo, ctrl_hi
            if lo is None or hi is None:
                raise GraphError("clip='ctrlrange' 需要 ctrl_lo/ctrl_hi（从 canonical 的 actuators 来）")
        elif clip == "none":
            lo = hi = None
        else:
            raise GraphError(f"不认识的 clip：{clip}")
        out = self._plan.clip(s, lo, hi) if lo is not None else s
        if self.expose_raw_action:
            # 观测里的 last_action 必须是**策略原始输出**（affine 之前）。
            # 把 ctrl 当 last_action 喂回去，策略看到的是加了站姿偏置的假动作，
            # 表现就是「指令给到上限也不走」（2026-11-11 用 20 s 路程 0.01 m 抓到的）。
            self._plan.output_dim = self._out_len * 2
            self._plan.out_action(self._plan.concat(out, self._action_ref))
        else:
            self._plan.out_action(out)
        self._built = True
        return self

    # ── 产物 ────────────────────────────────────────────────
    def to_json(self) -> dict:
        if not self._built:
            raise GraphError("图还没完成：至少要 observe → policy → action")
        return self._plan.to_json()

    def build(self) -> bytes:
        return ir.compile_blob(self.to_json())

    def digest(self) -> str:
        return ir.sha256_of_blob(self.build())

    # ── 便捷：标准单策略图（等价于 run_policy.build_plan）────
    @classmethod
    def standard(cls, policy_manifest: dict, preset: dict, ctrl_lo, ctrl_hi,
                 mode: int = 0, expose_raw_action: bool = False) -> "Graph":
        return (cls(policy_manifest, preset, name=policy_manifest.get("policy_id", "graph"),
                    expose_raw_action=expose_raw_action)
                .observe_all()
                .policy("standard", mode=mode)
                .action(clip="ctrlrange", ctrl_lo=ctrl_lo, ctrl_hi=ctrl_hi))


if __name__ == "__main__":
    print(__doc__)
