#!/usr/bin/env python3
"""薄门面：把「声明文件 + 脚本」包成几个名字，够写十行脚本就行。

    from ducksir import Duck
    d = Duck()                       # duck-hd1910
    print(d.joints(), d.default_pose()[:3])
    r = d.run(seconds=5, vx=0.25)    # 跑单策略闭环，返回报告 dict
    print(r["panel"]["realtime_factor"])

**不做**完整对象模型（`Simulation` 的 scene/motion/sensor 装配那套）。理由：厚门面只有在
多机器人 + 多场景 + 动作重定向都落地之后才有意义；现在这层只为了「新人打开就知道怎么跑」，
以及让 UI 上那段示例代码是真的能跑的代码（不是伪代码）。

对应的完整命令见 `docs/env.md` §7.5 / `bin/ducksir`。
"""

from __future__ import annotations

import json
import os
import subprocess
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))

__all__ = ["Duck"]


class Duck:
    """一台 MicroDuck：知道自己的声明在哪、能跑起来、能读报告。"""

    def __init__(self, robot_id: str = "duck-hd1910"):
        self.robot_id = robot_id
        self.canonical = os.path.join(ROOT, f"contract/robot/types/{robot_id}.yaml")
        self.preset = os.path.join(ROOT, f"build/preset/{robot_id}.preset.yaml")
        self.mjcf = os.path.join(ROOT, f"build/mjcf/{robot_id}-all.xml")
        self.manifest = os.path.join(ROOT, "contract/policy/duck-walk.manifest.yaml")
        self.contract = os.path.join(ROOT, "client/viewer/data/live/contract.json")

    # ── 只读查询：直接读声明，不参与仿真 ─────────────────────────
    def _preset(self) -> dict:
        import yaml
        with open(self.preset, encoding="utf-8") as f:
            return yaml.safe_load(f)

    def joints(self) -> list[str]:
        """驱动关节名（顺序 = 动作/控制顺序）。"""
        return list(self._preset()["jointOrder"]) if "jointOrder" in self._preset() else \
            self._joints_from_contract()

    def _joints_from_contract(self) -> list[str]:
        with open(self.contract, encoding="utf-8") as f:
            return json.load(f)["robot"]["joints"]

    def default_pose(self) -> list[float]:
        """站姿（动作的偏置，`ctrl = 站姿 + 动作`）。"""
        return list(self._preset()["defaultJointPositions"])

    def contract_doc(self) -> dict:
        with open(self.contract, encoding="utf-8") as f:
            return json.load(f)

    # ── 跑起来 ──────────────────────────────────────────────────
    def run(self, *, seconds: float = 5.0, vx: float | None = 0.25,
            fsm: bool = False, pulses: str = "3:1,8:1",
            report_json: str | None = None) -> dict:
        """跑一遍闭环，返回报告 dict（含 `panel` 八项）。"""
        if not os.path.exists(self.mjcf):
            raise FileNotFoundError(f"缺 {self.mjcf}：先跑 `bin/ducksir build` 或 tools/robot/export_mjcf.py")
        tmp = None
        if report_json is None:
            tmp = tempfile.NamedTemporaryFile(suffix=".json", delete=False)
            report_json = tmp.name
            tmp.close()
        script = "sim/host/run_fsm.py" if fsm else "sim/host/run_policy.py"
        argv = [sys.executable, os.path.join(ROOT, script),
                "--mjcf", self.mjcf, "--preset", self.preset, "--seconds", str(seconds),
                "--report", os.path.relpath(report_json, ROOT)]
        if fsm:
            argv += ["--pulses", pulses]
        elif vx is not None:
            argv += ["--manifest", self.manifest, "--vx", str(vx)]
        else:
            argv += ["--manifest", self.manifest]
        subprocess.run(argv, cwd=ROOT, check=False, stdout=subprocess.DEVNULL)
        with open(report_json, encoding="utf-8") as f:
            doc = json.load(f)
        if tmp is not None:
            os.unlink(report_json)
        return doc


if __name__ == "__main__":
    d = Duck()
    print(f"{d.robot_id}：{len(d.joints())} 个驱动关节，站姿前 3 项 {[round(x, 3) for x in d.default_pose()[:3]]}")
    r = d.run(seconds=2, vx=0.25)
    print("panel：", json.dumps(r.get("panel", {}), ensure_ascii=False))
