---
name: ducksir-parity
description: 把 botlab 模板的观测/动作对齐到我们 runner 时用。给出录制、切样本、跑对拍、读差值的固定流程，以及四类必然踩的坑。
---

# 对拍（obs / 动作与 botlab 对齐）

目标：**用数字证明**我们的 obs 与 botlab 一致，而不是"看着差不多"。

## 步骤

1. **录制**（在模板副本里，不要改 third_party）：
   ```bash
   cp -r third_party/botlab-templates/unpacked/<Tpl> /tmp/<name>-run
   # 写 patch_rec.py：hook onnxruntime.InferenceSession.run + mujoco.mj_step
   PYTHONPATH=/tmp/botlab-py/src:$HOME/.local/share/ducksir/pylibs MUJOCO_GL=egl \
     BOTLAB_STEPS=40 python3 -u -c "import patch_rec; from botlab._cli import main; \
     import sys; sys.argv=['botlab','run','--headless','/tmp/<name>-run']; main()"
   ```
2. **切样本**：
   ```bash
   python3 tools/dev/make_parity_sample.py --bundle <bundle> --format <gmtrack|parkour|php> \
     --trace /tmp/<name>.jsonl --obs-dim <dim> --out third_party/parity/<bundle>.json
   ```
3. **manifest 声明**：
   ```yaml
   parity: {sample: third_party/parity/<bundle>.json, obs_tol: 2e-5}
   ```
4. **跑**：`python3 sim/test/test_parity.py`（CI 里也跑）

## 四类必踩的坑

| 坑 | 症状 | 正解 |
|---|---|---|
| timestep 没设 | replay 误差 0.12 | 设 `m.opt.timestep = 1/physics_hz`（0.005） |
| q/qd 关节序 | q 段差 0.3，q 差 0 但 qd 差 1.3 | 看模板：GMTrack 用 **qpos 序**；Parkour 用 **policy 序** |
| 历史排布 | qd/action 段差一个大数 | 最老在前 + 首帧填满（GMTrack）vs 零填充；别通用化 |
| action 语义 | action 段差 0.3 | 样本里存的是**这一步 obs 用的动作**（= 上一步算出的） |

## 验收

```
PASS  <bundle>：obs 最大差 1.5e-05（阈值 2.0e-05）
```

angvel 段允许 6e-2（cvel 比 qvel 慢一个物理子步），其余段严格。
