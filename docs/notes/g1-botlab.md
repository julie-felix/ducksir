# G1（botlab 模板的 29DOF）接入笔记

> 目标：把 botlab 的 G1 模板（WBC / Instinct / PHP / UniSuper 都用这台机器人）接到我们的链路上。
> 数据来源：`third_party/botlab-templates/*.zip`（`tools/dev/grab_botlab_templates.mjs` 下载）。

## 1. 已有 vs 新增

| 东西 | 状态 |
|---|---|
| `contract/robot/types/g1-29dof.yaml` | **原本就有**（从 Unitree 官方 URDF 导入，29 驱动关节） |
| `assets/g1/*.glb` | 原本就有（视觉用） |
| `contract/robot/types/g1-29dof-botlab.yaml` | **本轮新增**：从 botlab 模板的 `bare.xml` 导入 |
| `third_party/g1-botlab/assets/*.STL` | 本轮落地（83 个 STL，42 MB，gitignore） |
| `contract/robot/postures/g1-29dof.yaml` | **本轮新增**：站姿 + 整段 `initial_qpos`（来自 botlab 的 `robot.preset.yaml`） |
| `build/mjcf/g1-29dof-botlab.xml` | 本轮导出（physics-only，STL） |
| `build/preset/g1-29dof-botlab.preset.yaml` | 本轮导出（kp/kd/默认角/初始位姿/actionScale 都是上游的值） |

**为什么再导一份 type**：URDF 那条链没有执行器参数（importer 会给 W002 警告），
`kp/kd` 是空的；而 botlab 的 MJCF 里 29 个 `<position>` 执行器**自带 kp/kv**，
和它们策略训练时用的是同一组增益。所以 G1 的策略走 `g1-29dof-botlab` 这个类型。

## 2. 踩到的两个坑（都修在导出器里）

1. **零惯量占位 body**：上游有 `mass="1e-09" diaginertia="0 0 0"` 的 body（`LL_FOOT`/`RL_FOOT` 这种）。
   我们导出器原来一律写 `fullinertia`，MuJoCo 对 fullinertia 要做特征值检查 → **直接报错**。
   已修：纯对角惯量写 `diaginertia`（保持上游的 `0 0 0` 原样）。
2. **位形里的整段 `initial_qpos`**：上游给的是「基座位姿 7 + 关节角 29」一整条。
   导出器原来只会用 `base + joints` 拼，于是初始基座 z=0、机器人埋在地里。
   已修：位形里给了 `initial_qpos` 就用它（长度校验 7+n）。

## 3. 现在能到什么程度

```
python3 sim/test/test_g1_physics.py
  PASS  MJCF 能加载  nu=29 nq=36
  PASS  总质量合理（G1 ≈ 33 kg）  33.3 kg
  PASS  初始基座高度合理（0.5–1.2 m）  z=0.942
  PASS  跑 1 秒不发散（有限值、没飞天）
  PASS  关节角保持在合理范围（|q| < π）
```

**注意**：保持默认关节角 ≠ 站稳。实测 1 秒内基座从 0.942 掉到 0.18 m——它是真的倒了，
因为 G1 要靠策略平衡。这只是「模型能加载、初值合理、数值不发散」的冒烟。

## 4. 下一步（真正运控）

按难度排序，G1 的策略有三档：

| 档 | 策略 | 观测 | 难点 |
|---|---|---|---|
| 简单 | `loco_0731`（WBC 的 loco） | 96 维 + LSTM 256 | 观测配方（角速度/重力/指令/关节/上一步动作） |
| 中 | `dance12_0207_1`（WBC 的舞） | 439 维 = 384 + 55（动作参考分量） | 要接 `Motion` |
| 难 | Instinct / PHP / UniSuper | 1990 / 140+深度 / 1720 | 深度相机、99D 历史、四元数信号族 |

第一档不需要 `motion`、不需要深度、也不需要四元数族——只有「历史 + 状态」，
这正好是这一轮刚补上的 `history`（已有）+ `model_rnn`（新）。所以**先接 loco_0731**。
