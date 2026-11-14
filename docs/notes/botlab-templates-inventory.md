# botlab 8 个模板项目的全量清点与分析

> 数据来源：2026-11-11 从 `https://botlab.enkeebot.com/` 的 **文件 → 打开示例** 逐个打开，
> 再用 **文件 → 保存项目** 下载成 `.botlab.zip`，解压后逐文件分析。
> 原始包存在 `third_party/botlab-templates/*.zip`（220 MB，已 gitignore；用
> `node tools/dev/grab_botlab_templates.mjs <目录>` 可重下）。
>
> 模型输入输出维度是用 `onnxruntime` 实读的（`tools/dev/inventory_botlab_templates.py`），不是抄文档。

## 0. 总表

| 项目 | 文件数 | 体积 | 机器人 | MJCF | 控制/物理 | 模型数 | 备注 |
|---|---|---|---|---|---|---|---|
| MicroDuck Multi-Policy | 56 | 10.2 MB | MicroDuck（14 自由度） | 项目内自带 | 50/200 | 7 | 我们已经对标的就是这个 |
| Go2W Lab2MJ | 47 | 67.9 MB | Go2W 轮足四足（16 关） | `assets/scenes/scene_parkour.xml` + `go2w.xml` | 50/200 | 8 | VTM 记忆模块，地形场景 |
| G1 WBC Multi-Policy | 20 | 17.6 MB | Unitree G1 29DOF | 共享 `models/g1/…` | 50/200 | 4 | 4 个策略 + 1 段 motion |
| Unitree G1 GMTrack Stage-II | 44 | 27.3 MB | G1 29DOF | `assets/robot/scene.xml` | 50/200 | 1 | 全身动作跟踪 + 1 段 npz |
| G1 Hiking Parkour | 9 | 5.0 MB | G1 29DOF | **共享** `models/g1/instinctmj/…` | 50/200 | 2 | 深度编码器 + actor |
| G1 PHP Parkour | 64 | 32.9 MB | G1 29DOF + 地形 | `assets/scenes/g1_with_terrain.xml` | **50/500** | 2 | 深度 backbone + actor |
| Instinct-SuperMimic | 635 | 56.8 MB | G1 29DOF | `assets/robot/scene.xml` | 50 | 1 | 10 段动作 npz |
| UniSuperMimic | 60 | 105.5 MB | G1 29DOF | `assets/robot/scene.xml` | 50 | 1 | 3 段动作 npz，最大包 |

**共同的骨架**（8 个都一样）：

```
controller.py      声明：project() / Robot() / Model()（+ Sensor/Motion/Scene）/ create_controller(sim)
main.py            5 行：Simulation.from_project(); sim.run(create_controller(sim))
config/runtime.yaml     name + control_hz + physics_hz
config/parameters.yaml  滑杆（label 双语、default/range/step/unit）
config/io.yaml          键位映射（键盘/手柄 → 参数、策略指令）
*.preset.yaml      机器人参数：actionScale / defaultJointPositions / kp / kd / initialQpos
assets/models/*.onnx
assets/robot/…      MJCF + mesh
assets/motions/…    .npz 或 .bin（动作参考）
```

`main.py` 全都是这 5 行，说明**项目里没有「每步 Python」**——控制律全部由 `controller.py` 的声明组合出来。
（这与 `botlab-python` 的 README 一致：浏览器 target 不执行 per-step Python。）

---

## 1. MicroDuck Multi-Policy（我们已对标）

**结构**：`controller.py`(1.0 KB) + `microduck.py`(158 行，项目内库) + `microduck_policy.py`(状态机常量) + `microduck.preset.yaml` + 7 个 onnx + 41 个 STL。

**预设**（`microduck.preset.yaml` 实读）：

```yaml
actionDimension: 14
jointQposOffset: 7      # qpos 前 7 位是浮动基座
jointQvelOffset: 6
policyJointOrder: [0..13]   # 策略顺序 = MJCF 执行器顺序
motorJointOrder:  [0..13]
actionScale: 1
```

**模型**：7 个全部是 `obs[1,61] → actions[1,14]`（walking / sitstand / kick-left / kick-right / roll / ground-pick / stand）。

**状态机**（`microduck_policy.py`）：14 个状态 `WALK(0) … RECOVER_UPRIGHT(13)`，7 个策略槽 `POLICY_WALK(0)…POLICY_STAND(6)`；
迁移表是按 `command`（按键脉冲）和 `after_steps`（步数）两种条件写的，例如
`{from: WALK, command: SIT_TOGGLE, to: SIT_ENTER}`、`{from: SIT_ENTER, after_steps: SIT_ENTER_STEPS, to: SIT}`。

**控制面**：`declare_control(control_profile(**read_config("config/control.yaml")))` —— 一份给配对中间件编译成能力的声明。

> 我们的 `fsm.blob`（14 状态、`out:state/mode/command/clear_last_action`）在结构上就是这条的等价物；
> 差别是他们把状态机写在**项目内 Python 库**里，我们编进 blob。

---

## 2. Go2W Lab2MJ（四足 + 记忆模块）

**模型（8 个，实读 dims）**：

| 文件 | 输入 | 输出 |
|---|---|---|
| `motion_tracking/policy.onnx` | `obs[1,187]` | `actions[1,16]` |
| `vtm/student.onnx` | `obs[batch,5449]` | `actions[batch,16]` |
| `vtm_*_sru/student_encoder.onnx` | `obs[batch,629]` | `encoded_obs[batch,181]` |
| `vtm_*_sru/student_memory.onnx` | `encoded_obs[181]` + `hidden_state[1,batch,512]` (+`cell_state` for LSTM) | `latent`, `next_hidden_state`, `next_cell_state` |
| `vtm_*_sru/student_actor.onnx` | `latent[batch,512]` | `actions[batch,16]` |

**这是「一个策略 = 三个模型」的形态**：encoder → memory（带递归状态）→ actor。
我们的图里 `history` 节点能做定长历史，但**没有「模型输出回灌成下一帧输入」的递归状态**（`h_in/c_out`）——
这是 G1-WBC 的 `loco_0731.onnx`（`h_in/c_in` 256）和这里共有的第二种缺口。

**配置**：`policy_command` 用 `Digit1/2/3` 切换 motion tracking / VTM / VTM LSTM（`policy_bank` + `mode_switch`）。
**场景**：`scene_parkour.xml`（地形），另有 `depth-camera.preset.yaml`。
**控制器**：`PolicyController` + `policy_bank` + `mode_select` + `history(...)`。

---

## 3. G1 WBC Multi-Policy（4 策略 + 动作参考）

| 模型 | 输入 | 输出 |
|---|---|---|
| `amp_0309_1.onnx` | `obs[1,384]` | `actions[1,29]` |
| `Unitree-G1-AMP-Flat_model_30000.onnx` | `obs[batch,384]` | `actions[batch,29]` |
| `dance12_0207_1.onnx` | `obs[1,439]` | `actions[1,29]` |
| `loco_0731.onnx` | `obs[1,96]` + `h_in[1,1,256]` + `c_in[1,1,256]` | `actions[1,29]` + `h_out` + `c_out` |

- 439 = 384 + 55：多出来的是**动作参考**的分量（`motion_reference`）——跳舞策略吃参考动作；
- `loco_0731` 是 LSTM 递归（和 Go2W 的 memory 同型）；
- 动作素材 `assets/motions/dance12/`：8 个 `.bin` + `metadata.json`（`joint_pos/vel`、`body_pos/quat/lin_vel/ang_vel`、`fps`），**不是 npz**。

---

## 4. Unitree G1 GMTrack Stage-II（全身动作跟踪）

- 1 个模型：`stage2_actor.onnx`，`obs[1,1728] → actions[1,29]`；
- 动作：`assets/motions/current.npz`；
- 控制器用 `clock()` + `history()` + `rotate()`：把参考动作的**未来帧**（lookahead）与机器人当前状态一起喂给策略。
- 1728 = 864×2 量级，说明一帧观测里既有「当前状态」也有「参考动作窗口」。

---

## 5. G1 Hiking Parkour（深度条件）

`controller.py` 只有 45 行，是最干净的「感知 → 策略」例子：

```python
state = history([base_angular_velocity(scale=0.25), projected_gravity(),
                 commands(..., waypoint=waypoint_tracking(min_forward_velocity=0.1)),
                 joint_position(default=...), joint_velocity(scale=0.05), last_action()],
                8, layout="by-term")
depth = history(depth_camera.depth(max_age=5), 8, stride=5)
latent = depth_encoder(depth)          # [1,8,18,32] → [1,128]
action = actor(state, latent).clip(-100, 100).action()   # [1,896] → [1,29]
```

- `parkour_depth_encoder.onnx`：`input[1,8,18,32] → output[1,128]`（8 帧历史深度，18×32 分辨率）
- `parkour_actor.onnx`：`input[1,896] → output[1,29]`；896 = 768（历史状态 8×96）+ 128（深度 latent）
- 机器人 MJCF 与场景（`procedural://stairs` + `starter-parkour-stairs` 预设）**不在项目里**，引用站点共享的 `models/g1/…`
- 项目只有 9 个文件——这是「薄项目 + 共享模型树」的极端例子

---

## 6. G1 PHP Parkour（感知跑酷，500 Hz 物理）

| 模型 | 输入 | 输出 |
|---|---|---|
| `php_parkour_depth_backbone.onnx` | `depth_image[1,58,87]` | `depth_latent[1,32]` |
| `php_parkour_actor.onnx` | `obs[1,140]` + `time_step[1,1]` | `actions[1,29]` **+ 4 组参考量**：`joint_pos[29]`、`joint_vel[29]`、`body_pos_w[14,3]`、`body_quat_w[14,4]`、`body_lin_vel_w[14,3]`、`body_ang_vel_w[14,3]` |

- **输出不只有 action**：策略同时吐出参考动作（多输出头）。我们的图目前只允许一个 `out_action`；
- 物理 500 Hz（其余都是 200）——说明 `physics_hz` 是可配的、不绑死 4 倍抽取；
- 场景自带地形 `assets/scenes/g1_with_terrain.xml`。

---

## 7. Instinct-SuperMimic（深度条件 + 10 段动作）

- 1 个模型：`policy.onnx`，`obs[1,1990] → actions[1,29]`；
- 10 段动作 `case_00.npz … case_09.npz`（每段一条地形动作参考）；
- 635 个文件（其中绝大多数是 mesh/贴图）：**共享模型树被完整打进包里**，说明下载包有两种形态（薄 vs 全）；
- 控制器用 `clock` + `history` + `mode_select` + 相对位置/姿态。

---

## 8. UniSuperMimic（最大包，105 MB）

- 1 个模型：`policy.onnx`，`obs[1,1720] → actions[1,29]`；
- 3 段动作：`walk4_subject1.npz`、`jumps1_subject1.npz`、`fallAndGetUp1_subject1.npz`；
- 控制器（docstring 自己写了口径）：**「99D history + 730D goal」**，用了
  `quaternion_inverse/heading`、`rotate`、`relative_position/quaternion`、`rotation_6d`、`clock`、`history`、`mode_select`。

---

## 9. 横向结论：这 8 个项目暴露了我们 5 个缺口

| # | 缺口 | 出现在 | 性质 | 我们现状 |
|---|---|---|---|---|
| 1 | **动作参考 `Motion` / `motion_reference`**（npz/bin 采样、lookahead 窗口） | GMTrack、WBC、Instinct、UniSuper（4/8） | 纯数据 + 图 | ✅ **节点已做**（`motion`，2026-11-11）：npz 形态可用；WBC 的私有 `.bin` 容器**显式不支持** |
| 2 | **递归状态模型**（`h_in/c_in` → `h_out/c_out`，跨帧回灌） | Go2W（memory×2 套）、WBC（loco） | 图能力（有状态） | ❌ 没有；`history` 只是定长窗口 |
| 3 | **四元数信号族**（inverse/heading/relative/rotate/rotation_6d） | UniSuper、Instinct、GMTrack | 纯数学，跨端可复现 | ❌ 只有投影重力 |
| 4 | **深度相机**（`Sensor.depth` + `history(stride=5)`） | Hiking、PHP（2/8，但这 2 个是「感知」标杆） | 渲染/传感器 | ❌ 两端都没有 |
| 5 | **多输出头策略**（action + 参考量） | PHP（1/8） | 图能力 | ❌ 只有单个 `out_action` |

另外两个「形态」层面的观察：

- **薄项目 + 共享模型树**（Hiking 只有 9 个文件）和 **全量打包**（Instinct 635 个文件）并存 ——
  说明他们允许项目引用站点级 `models/…`；我们的 bundle 目前是「全量自包含」，这对分发更简单，但没法做「共享模型只存一份」。
- **配置三分**（`runtime` / `parameters` / `io`）非常值得抄：参数带 `label`（双语）、`range`、`step`、`unit`，
  直接生成 UI 滑杆；`io` 把键位到参数的映射写成声明，我们的工作台是硬编码 `W/S/A/D`。

## 10. 对我们意味着什么（推荐顺序）

1. **先补 `Motion`**：4/8 个项目要用，且它是纯数据 + 图（npz 采样 + lookahead 窗口），符合「能进图才进图」；
2. **再补递归状态模型**：2/8 个项目要用（Go2W 的 VTM、WBC 的 loco）。这一条会动到图的「有状态」语义，
   要比 `history` 更谨慎（需要在 blob 里表达 `h/c` 的读写与重置）；
3. **四元数信号族**：纯数学，工作量小，UniSuper/Instinct 的对齐门槛就是它；
4. **深度相机**：成本最大（两端都要渲染取深度），建议单独一轮；
5. **多输出头**：只有 PHP 一个项目用，可以先不做。

顺带两条马上能做的（低成本、直接提升「像 botlab」）：

- `config/parameters.yaml` 的 `label/range/step/unit` → 我们工作台滑杆从契约生成，而不是写死；
- `config/io.yaml` 的键位映射 → 我们的键位提示也从声明生成。

## 11. 进度（2026-11-11 晚）

`motion` 节点（第 16 种）已落地：

- 语义：输入 1 维秒数 → `t*fps` 取相邻两帧线性插值，`loop` 控制回绕，越界夹住；
- 数据编进 blob 参数表，**不需要运行时文件 IO**，宿主与浏览器吃同一份；
- 实现：`engine/include/ducksim.h`（DS_NODE_MOTION=16）+ `engine/src/core.cpp` + `sim/plan/ir.py` + `sim/plan/oracle.py` + `sim/plan/graph.py`；
- 素材加载：`sim/plan/motion.py`（npz 可用；WBC 的 `.bin` 私有容器**已解码**，见文末补记）；
- 测试：`sim/test/test_motion_node.py`（核 ↔ numpy 参考、整数帧/半帧/回绕/越界/参数校验）、
  `sim/test/test_motion_load.py`（用下载下来的真实 npz 验 417 帧 × 29 维、fps=50）。

还没接线：`contract.json` 里还没有「动作参考」这一槽位，`fsm.blob` 也还没用到这个节点。
真正用起来要等「动作跟踪类」项目（GMTrack / UniSuper）接入。

## 12. 递归状态（2026-11-11 夜）

`model_rnn`（第 18 种节点）与 `delay`（第 17 种）已落地：

| 节点 | 语义 | 用途 |
|---|---|---|
| `delay` | N 步延迟，前 N 步输出 fill | botlab `delay(mode, 1)` 这种信号级用法；只能延迟**前序**节点的值 |
| `model_rnn` | `in=[obs,h,c]` → `out=[action,h_new,c_new]`，h/c 存 plan、跨步保持、`ds_reset` 清零 | LSTM/GRU 策略（Go2W 的 VTM、G1-WBC 的 loco） |

为什么不是「用 `delay` 把模型输出接回去」：图是 DAG，`h_new` 要在同一时间步喂给 `model` 就是环。
把状态放进节点（`model_rnn`）是唯一不破坏 DAG 的做法；`delay` 只能延迟更早的值。

**验证**：`sim/test/test_rnn_node.py` 用假后端实现 `h_new = 0.5h + obs`，
核与 numpy 参考逐点一致、`ds_reset` 重放逐位相同、没后端时明确返回 `DS_ERR_NO_BACKEND`。

**宿主侧多张量已打通**（同日后续提交）：ORT 后端从「只收单输入单输出」扩到 **1..3 进 1..3 出**——
核照旧送一条 `[obs,h,c]`、收一条 `[action,h_new,c_new]`，后端按模型声明的张量切分与合并。
用真实的 `loco_0731.onnx`（96+256+256 → 29+256+256）端到端验过：跑通、状态跨步、连跑 50 步无 NaN、
`ds_reset` 后第一步与最初**逐位相同**（0.00e+00）。

**浏览器侧已补**（同日）：`client/viewer/model-io.mjs`（纯函数：按 metadata 切分打包输入、按声明顺序合并输出）
+ `play.js` 里的多张量分支；拿不到 metadata 时回落到「一进一出」老路径。
wasm 核也重新编过了（`make wasm`，emsdk 在 `~/emsdk`），`test_wasm_parity` 仍是 **0.000e+00**。
还没跑过真实的递归 bundle（我们还没有这样的包），节点与适配层的单测有：
`test_model_io.mjs`、`test_rnn_host_model.py`。

### 补：`.bin` 容器解码了（2026-11-12）

WBC 的 `dance12/` 用私有容器：`NPZ\0` 魔数 + 若干 u32 头 + 行主序 float32。
两件事都是**可验证**地定下来的，不是猜：

1. **头长度**：从 {16,20,24,28,32} 里挑唯一能让 `(文件长 − h)` 被 `4×宽` 整除的那个
   （joint_pos.bin：762608 B、宽 29 → h=24、6574 帧）；
2. **fps**：不猜 `fps.bin` 的字段语义（没有公开规范），改用**物理一致性**反推——
   取候选 fps 让「关节位置差分 × fps ≈ 关节速度」误差最小。
   dance12 实测 **fps = 50.0，误差 0.00e+00**（逐位相等，不是近似）。

验证（`sim/test/test_motion_load.py`）：

```
PASS  bin：读出 joint_pos  6574 帧 × 29
PASS  bin：fps 反推出 50  50.0
PASS  bin：位置求导 = 速度（容器解码正确）  平均误差 0.00e+00
PASS  bin：四元数 4 列且是单位四元数  |q0|=1.0000
PASS  bin：pelvis 高度合理  z=0.780
```

于是 4/8 个模板用到的动作素材（3 个 npz + 1 个 bin）现在都能读了。
