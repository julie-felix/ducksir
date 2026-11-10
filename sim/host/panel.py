#!/usr/bin/env python3
"""面板指标 schema：把「这次跑得怎么样」收敛成固定 8 个键。

    from panel import PANEL_KEYS, build_panel
    panel = build_panel(metrics=..., verdict=..., steps=..., seconds=..., step_times=...)

为什么要有 schema（2026-11-10 的决定，照 botlab `sdk/_panels.py` 的思路）：
显示什么**是声明的一部分**，而且这些量要在运行时真的存在。否则报告字段各写各的，
面板那边只能靠猜，出问题时也没法说清「哪一项判断依据不在了」。

8 个键（少一个都算不合格）：

| 键 | 含义 |
|---|---|
| `control_hz` / `physics_hz` | 实际配置的控制/物理频率 |
| `realtime_factor` | 仿真时长 ÷ 墙钟时长 |
| `control_p95_ms` | 控制步（推理 + 物理）耗时 95 分位 |
| `deadline_miss` | 超过 1/control_hz 的控制步个数 |
| `base_height_min` | 基座最低高度（判「陷地」） |
| `upright_final` | 末态直立度（−1 = 完全倒立，+1 = 完全正立） |
| `distance` | 水平位移（米） |
"""

from __future__ import annotations

import math

PANEL_KEYS = (
    "control_hz",
    "physics_hz",
    "realtime_factor",
    "control_p95_ms",
    "deadline_miss",
    "base_height_min",
    "upright_final",
    "distance",
)

PANEL_LABELS = {
    "control_hz": ("控制频率", "Hz"),
    "physics_hz": ("物理频率", "Hz"),
    "realtime_factor": ("实时倍率", "×"),
    "control_p95_ms": ("控制步 p95", "ms"),
    "deadline_miss": ("超期步数", "步"),
    "base_height_min": ("基座最低", "m"),
    "upright_final": ("末态直立度", "—"),
    "distance": ("水平位移", "m"),
}


def _p95(values) -> float:
    if not values:
        return 0.0
    xs = sorted(values)
    idx = min(len(xs) - 1, int(math.ceil(0.95 * len(xs))) - 1)
    return float(xs[max(idx, 0)])


def build_panel(*, control_hz: float, physics_hz: float, seconds: float, wall: float,
                step_times=None, base_height_min: float = 0.0, upright_final: float = 0.0,
                distance: float = 0.0) -> dict:
    """按 schema 组面板数据。`step_times` 是每个控制步的墙钟秒数（可选）。"""
    rtf = (seconds / wall) if wall > 0 else 0.0
    times = list(step_times or [])
    deadline = (1.0 / control_hz) if control_hz else 0.0
    panel = {
        "control_hz": float(control_hz),
        "physics_hz": float(physics_hz),
        "realtime_factor": round(rtf, 3),
        "control_p95_ms": round(_p95([t * 1000 for t in times]), 3),
        "deadline_miss": int(sum(1 for t in times if deadline and t > deadline)),
        "base_height_min": round(float(base_height_min), 4),
        "upright_final": round(float(upright_final), 4),
        "distance": round(float(distance), 4),
    }
    missing = [k for k in PANEL_KEYS if k not in panel]
    if missing:
        raise ValueError("面板缺键：" + ", ".join(missing))
    return panel


def check_panel(panel: dict) -> list[str]:
    """校验一个面板 dict 是否满足 schema（缺键 / 非数 / 类型不对）。"""
    errs = []
    for key in PANEL_KEYS:
        if key not in panel:
            errs.append(f"缺键 {key}")
            continue
        v = panel[key]
        if not isinstance(v, (int, float)) or isinstance(v, bool):
            errs.append(f"{key} 不是数字：{v!r}")
        elif not math.isfinite(float(v)):
            errs.append(f"{key} 不是有限数：{v!r}")
    return errs
