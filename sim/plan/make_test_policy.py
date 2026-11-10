#!/usr/bin/env python3
"""造一份「测试用」的鸭子策略 ONNX（合成，不是训练出来的）。

    PYTHONPATH=$HOME/.local/share/ducksir/pylibs \
    python3 sim/plan/make_test_policy.py -o build/models/duck-hold.onnx

为什么要有它：W5 要验「真 ONNX 在核里跑通、在 MuJoCo 里跑 30 s」，而仓里还没有训练出来的策略。
这份合成策略干的事是 **把关节往零位拉**：

    action = clip(W · obs + b, -0.4, 0.4)，其中只对 joint_pos 取负增益

它证明不了运控好不好（鸭子照样会倒，因为自由基座的平衡要真策略），但它证明**整条链是活的**：
obs 组装 → ONNX 推理 → action 后处理 → PD → MuJoCo。换成真策略只是换一个 .onnx。

观测布局（34 维，和 sim/host/run_policy.py 里的切片一致）：
    [0:3]   机体角速度（body frame）
    [3:6]   投影重力
    [6:20]  关节角（14）
    [20:34] 关节角速度（14）
"""

from __future__ import annotations

import argparse
import os

import numpy as np
import onnx
from onnx import TensorProto, helper, numpy_helper

OBS_ANG_VEL = (0, 3)
OBS_GRAVITY = (3, 6)
OBS_JOINT_POS = (6, 20)
OBS_JOINT_VEL = (20, 34)
OBS_DIM = 34
N_JOINTS = 14


def build(gain: float = 1.0, mode: str = "hold", value: float = 0.0) -> onnx.ModelProto:
    """mode=hold: action = -gain*q（把关节往零位拉）；mode=constant: action = value（常值目标）。"""
    w = np.zeros((OBS_DIM, N_JOINTS), dtype=np.float32)
    b = np.zeros((N_JOINTS,), dtype=np.float32)
    if mode == "hold":
        w[OBS_JOINT_POS[0]:OBS_JOINT_POS[1], :] = -gain * np.eye(N_JOINTS, dtype=np.float32)
    elif mode == "constant":
        b[:] = value
    else:
        raise SystemExit(f"未知 mode: {mode}")

    nodes = [
        helper.make_node("MatMul", ["obs", "W"], ["mul"]),
        helper.make_node("Add", ["mul", "b"], ["action"]),
    ]
    graph = helper.make_graph(
        nodes,
        "duck_hold",
        [helper.make_tensor_value_info("obs", TensorProto.FLOAT, [1, OBS_DIM])],
        [helper.make_tensor_value_info("action", TensorProto.FLOAT, [1, N_JOINTS])],
        initializer=[numpy_helper.from_array(w, "W"), numpy_helper.from_array(b, "b")],
    )
    model = helper.make_model(graph, producer_name="ducksir-test-policy")
    model.opset_import[0].version = 17
    model.ir_version = 9
    onnx.checker.check_model(model)
    return model


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("-o", "--output", required=True)
    ap.add_argument("--gain", type=float, default=1.0)
    ap.add_argument("--mode", default="hold", choices=["hold", "constant"])
    ap.add_argument("--value", type=float, default=0.0, help="mode=constant 时的目标（rad）")
    args = ap.parse_args()
    os.makedirs(os.path.dirname(os.path.abspath(args.output)), exist_ok=True)
    model = build(args.gain, args.mode, args.value)
    onnx.save(model, args.output)
    print(f"写入 {args.output}（obs {OBS_DIM} → action {N_JOINTS}，mode={args.mode}）")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
