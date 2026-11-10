# 策略清单（manifest v1）

策略产物 = **ONNX 权重** + **这份 manifest**。权重回答「算出来是什么」，manifest 回答「吃进去的是什么、吐出来的怎么落到关节上」。

为什么必须有它（计划 D12）：obs 顺序、缩放、历史布局、动作映射这些事，只要有一处两边不一致，策略就会「看着在动但不对」，而且不报错——botlab 自己也承认 `history` 的 `by-frame` / `by-term` 选错不报错、`joint_position` 不会自动减参考位姿。所以宁可多一份文件，也要让「语义」有唯一出处。

## 字段

```yaml
kind: ducksir-policy-manifest
version: 1
policy_id: duck-hold

model:
  file: build/models/duck-hold.onnx      # 相对仓库根；*.onnx 不入库
  input:  {name: obs,    shape: [1, 34]}
  output: {name: action, shape: [1, 14]}

# obs 按这里声明的顺序拼成一维向量；每项给槽名、长度、缩放
obs:
  - {slot: base_ang_vel,      len: 3,  frame: body, scale: 1.0}
  - {slot: projected_gravity, len: 3,  scale: 1.0}
  - {slot: joint_pos,         len: 14, source: qpos, scale: 1.0}
  - {slot: joint_vel,         len: 14, source: qvel, scale: 1.0}

history: {length: 0, layout: by-frame}   # length=0 表示不挂历史节点；>0 时按 by-frame 铺开

action:
  mapping: default_plus_scale            # ctrl = defaultJointPositions + actionScale * action
  order: motor                           # action 第 i 维 → motorJointOrder[i]（preset 里的置换）
  clip: ctrlrange                        # 用本体 actuators[].ctrlrange 夹，不自己编数字

rates: {control_hz: 50, physics_hz: 200, inference_decimation: 1}
```

## 一致性检查（`sim/host/run_policy.py` 会做，不通过直接退出）

1. `sum(obs[].len)` == `model.input.shape[-1]`；
2. `len(驱动关节)` == `model.output.shape[-1]` == preset `actionDimension`；
3. `rates.physics_hz % rates.control_hz == 0`，且 `inference_decimation` ≥ 1 且能整除；
4. `action.order == motor` 时，`motorJointOrder` 必须是 0..n-1 的置换（C8 已间接保证双射）；
5. manifest 与它在 `model.file` 里读到的 IO 名/形状逐字一致。

**这些检查都是「红就停」**：宁可跑不起来，也不要跑一份语义不明的策略。

## 现在有什么

| 文件 | 说明 |
|---|---|
| `duck-hold.manifest.yaml` | 合成测试策略（把关节往零位拉），用来打通链路；**不是训练产物** |
