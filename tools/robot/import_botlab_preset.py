#!/usr/bin/env python3
"""把 botlab 模板自带的 `<x>.preset.yaml` 转成我们的 preset（含执行器模式）。

为什么要这一步：botlab 的 preset 不只是「默认角」，它还带 **kp/kd/effortLimit/actionScale**——
G1 那种扭矩模型里，PD 是**引擎/宿主**按这三个数组算的，不在 MJCF 里。
这些值在 canonical 实体里没有（URDF/MJCF 都不带），只能从 preset 搬。

    python3 tools/robot/import_botlab_preset.py <botlab-preset.yaml> \\
        --mjcf build/mjcf/g1-29dof-torque.xml -o build/preset/g1-29dof-torque.preset.yaml

产物字段与 `export_preset.py` 一致，另加：

    actuatorMode: torque | position   # torque 时宿主/浏览器按 PD 算 ctrl
"""
import argparse
import hashlib
import pathlib

import yaml


def sha256(path: pathlib.Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("botlab_preset")
    ap.add_argument("--mjcf", required=True)
    ap.add_argument("-o", "--output", required=True)
    ap.add_argument("--control-hz", type=float, default=50.0)
    ap.add_argument("--physics-hz", type=float, default=200.0)
    args = ap.parse_args()

    src_path = pathlib.Path(args.botlab_preset)
    src = yaml.safe_load(src_path.read_text(encoding="utf-8"))
    mjcf = pathlib.Path(args.mjcf)
    xml = mjcf.read_text(encoding="utf-8")
    # <motor> = 扭矩（PD 在宿主/引擎）；<position kp> = 位置（MuJoCo 自己算 PD）
    motor = "<motor" in xml
    n_act = xml.count("<motor") if motor else xml.count("<position")

    def arr(key, default=None):
        v = src.get(key)
        if v is None:
            return default
        out = []
        for x in v:
            if isinstance(x, bool):
                out.append(1.0 if x else 0.0)
            else:
                out.append(float(x))
        return out

    def ints(key):
        v = src.get(key)
        return [int(x) for x in v] if v else list(range(n_act))

    joints = arr("defaultJointPositions") or [0.0] * n_act
    preset = {
        "kind": "botlab-preset",
        "generated": {
            "by": "tools/robot/import_botlab_preset.py",
            "from": str(args.botlab_preset),
            "source_sha256": sha256(src_path),
            "mjcf": str(mjcf),
            "mjcf_sha256": sha256(mjcf),
        },
        "actuatorMode": "torque" if motor else "position",
        "controlHz": args.control_hz,
        "physicsHz": args.physics_hz,
        "actionDimension": n_act,
        "jointQposOffset": [7],
        "jointQvelOffset": [6],
        "actuatorJointOrder": list(range(n_act)),
        "motorJointOrder": ints("motorJointOrder"),
        "policyJointOrder": ints("policyJointOrder"),
        "defaultJointPositions": joints,
        "kp": arr("kp") or [0.0] * n_act,
        "kd": arr("kd") or [0.0] * n_act,
        "effortLimit": arr("effortLimit") or [0.0] * n_act,
        "actionScale": arr("actionScale") or [1.0] * n_act,
        "initialQpos": arr("initialQpos"),
        "assumptions": {
            "actuatorMode": ("扭矩：PD 由宿主按 kp/kd 在每个控制步算，ctrl 是力矩" if motor
                             else "位置：MuJoCo 自己做 PD"),
            "kp/kd/effortLimit": "来自 botlab 模板的 preset（其策略训练时用的那组）",
        },
    }
    out = pathlib.Path(args.output)
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(
        "# 由 tools/robot/import_botlab_preset.py 生成，不要手改\n"
        + yaml.safe_dump(preset, allow_unicode=True, sort_keys=False),
        encoding="utf-8")
    print(f"写入 {args.output}（actuatorMode={'torque' if motor else 'position'}，{n_act} 个执行器）")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
