# 节点对照表：botlab 56 个节点 vs ducksir 集成情况

> 日期：2026-11-10 ｜ 状态：待确认（随 `docs/plans/2026-11-10-botlab-replica.md` D15 生效）
> 事实来源：`/tmp/botlab-research/botlab-python` 的 `src/botlab/_host.py:34 SUPPORTED_NODE_TYPES`（56 项，与 `_runtime/_processors/__init__.py:11 build_registry()` 的键数一致）。
> 规矩：**每一行都要有结论**，「未集成」必须写理由；以后每次对齐都改这张表，不在别处另开一份。

图例：✅ 已集成 ｜ ⇄ 我们的等价节点替代 ｜ ⏳ 排进 v0.1 ｜ ❌ 不集成

统计：✅ 25 ｜ ⇄ 2 ｜ ⏳ 2 ｜ ❌ 27 —— 我们的 27 个节点对齐到它 25 个（`affine` 并进 `proc-math`、`moving_average` 并进 `proc-signal-filter`、`take` 用 `proc-demux` 的等价形态）。

## 观测（15）

| botlab | 我们 | 说明 |
|---|---|---|
| `obs-qpos` | ❌ | 暴露 MuJoCo 原始 qpos 索引 = 把物理布局泄进图；我们用按驱动序的 `joint_pos` |
| `obs-qvel` | ❌ | 同上 |
| `obs-imu` | ✅ | `imu` |
| `obs-constant` | ⇄ | `affine` 的常量输入覆盖 |
| `obs-base-height` | ✅ | `base_height` |
| `obs-base-lin-vel` | ❌ | v0 不出：真机线速度噪声大，训练侧多用 gravity + ang_vel |
| `obs-base-ang-vel` | ✅ | `base_ang_vel` |
| `obs-projected-gravity` | ✅ | `projected_gravity` |
| `obs-commands` | ✅ | `commands` |
| `obs-dof-pos` | ✅ | `joint_pos` |
| `obs-dof-vel` | ✅ | `joint_vel` |
| `obs-actions` | ✅ | `last_action` |
| `obs-camera` | ❌ | v0 无视觉（深度要 GPU/EGL，本机没有） |
| `obs-mujoco-query` | ❌ | 逃生口，直接泄漏仿真器内部；端侧无对应物 |
| `obs-height-scan` | ❌ | v0 无地形 |

## 处理（34）

| botlab | 我们 | 说明 |
|---|---|---|
| `proc-math` | ✅ | `math`（含标量四则/三角）；`affine` 也归并到这里 |
| `proc-history` | ✅ | `history`（`layout` 必填，`by-frame`/`by-term`） |
| `proc-mux` | ✅ | `concat` |
| `proc-demux` | ⇄ | `take` 只做拆分场景，不做通用 demux |
| `proc-clamp` | ✅ | `clip` |
| `proc-reorder` | ✅ | `reorder` |
| `proc-resize` | ❌ | 静默改语义的典型（插值/广播规则不可见），故意不做 |
| `proc-reroute` | ❌ | 图里直连即可，不需要中间节点 |
| `proc-pd` | ❌ | PD 交给执行侧（MuJoCo position actuator / 舵机内置），核里再做就是双重 PD |
| `proc-motion-reference` | ❌ | 等重定向管线落地再做 |
| `proc-motion-sample` | ❌ | 同上 |
| `proc-policy-input` | ✅ | `policy_input`（按 manifest 组装某策略的 obs） |
| `proc-onehot-index` | ❌ | `mode_select` 的整型输入已覆盖 |
| `proc-action-hook` | ❌ | 逃生口；端侧不可用，允许即等于三端不一致 |
| `proc-delay` | ✅ | `delay` |
| `proc-rule-index` | ❌ | 规则表选索引，v0 用 `state_machine` + `mode_select` |
| `proc-one-hot` | ❌ | v0 不需要 |
| `proc-compare` | ⏳ | v0.1 补：`state_machine` 的 `when` 要真值输入 |
| `proc-logic` | ⏳ | v0.1 补：同上（`& | ~`） |
| `proc-where` | ❌ | 两边都求值（不短路），用起来全是坑 |
| `proc-state-machine` | ✅ | `state_machine` |
| `proc-phase-clock` | ✅ | `phase_clock`（喂 `state_machine.steps`） |
| `proc-clock` | ❌ | `phase_clock` 够用 |
| `proc-spatial-math` | ❌ | v0 无 3D 场景 |
| `proc-signal-math` | ✅ | `math` |
| `proc-signal-noise` | ✅ | `noise` |
| `proc-signal-filter` | ✅ | `low_pass` 与 `moving_average` 都归这里 |
| `proc-vector-math` | ✅ | `norm` 与 `math` 的分量运算 |
| `proc-fsm` | ❌ | 与 `state-machine` 重复，只做一个 |
| `proc-mode-switch` | ❌ | `mode_select` 覆盖 |
| `proc-select` | ❌ | `mode_select` 覆盖 |
| `proc-mode-select` | ✅ | `mode_select` |
| `proc-signal-send` | ❌ | 单图内无需跨图通信 |
| `proc-signal-receive` | ❌ | 同上 |

## 模型与输出（5）

| botlab | 我们 | 说明 |
|---|---|---|
| `model-onnx` | ✅ | `model_onnx`（单输入、float32、单文件） |
| `out-action` | ✅ | `out_action` |
| `out-mujoco` | ❌ | 核不碰物理（D9） |
| `out-scope` | ❌ | 调试示波器，v0 用 `out_metric` + 曲线面板 |
| `out-logger` | ✅ | `out_metric` |

## 场景（2）

| botlab | 我们 | 说明 |
|---|---|---|
| `scene-robot` | ❌ | v0 单机器人 |
| `scene-terrain` | ❌ | v0 无地形 |

## 附：它的 golden 覆盖度（判断「照抄它」可行性的依据）

`tests/fixtures/processors.json` 里 54 个 case / 41 个节点类型，`pendingTypes` 恰是 15 个：`obs-qpos` `obs-qvel` `obs-imu` `obs-base-height` `obs-base-lin-vel` `obs-base-ang-vel` `obs-projected-gravity` `obs-commands` `obs-dof-pos` `obs-dof-vel` `obs-actions` `obs-camera` `obs-mujoco-query` `obs-height-scan` `model-onnx`。

即：**它对拍的全是纯数据变换，与 MuJoCo/ORT 耦合、最容易两端漂移的那批一个都没覆盖** —— 这也是观测语义必须我们自己定义的原因（见计划 D18）。
