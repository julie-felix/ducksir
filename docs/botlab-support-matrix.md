# botlab 支持哪些「项目」，我们要对齐哪些

> 结论先行：botlab 的「支持」不是一份机器人清单，而是**一套能力面**——
> 任意 MuJoCo 机器人 × 任意 ONNX 策略 × 一组观测源/动作接口/场景/状态机。
> 所以「我们也都支持」要落成一张能力矩阵，而不是逐个机器人去接。

本文的证据来源：本机 `/tmp/botlab-research/botlab-python`（PyPI 包 `botlab-python`，作者 OpenLegged）的源码与文档。
凡是「未确认」的地方都标了「**未确认**」，不编。

---

## 1. botlab 的「项目」是什么

一个项目 = **一个目录**，里面是：

| 文件 | 作用 |
|---|---|
| `controller.py` | 声明：`project()` + `Robot` / `Model` / `Sensor` / `Motion` / `Scene`。**在被 `main.py` 导入时收集** |
| `main.py` | 执行：`Simulation.run(controller)` —— 注册控制器、启动循环 |
| `*.preset.yaml` | 项目本地的资源预设（README 明说：内置具名预设注册表已移除） |
| `models/…` | MJCF / ONNX / 网格；先按项目目录解析，再回落到共享 assets 根（`../..`） |

所以「botlab 支持哪些项目」在它自己的口径里 = **任何能这样声明的机器人项目**，
官方只是额外提供几个**模板**（template）。公开代码里能确证的模板名只有两个：

| 模板 | 证据 | 说明 |
|---|---|---|
| `public/templates/microduck` | `tests/test_native_run.py:201`、`tests/e2e/*` | 双足小鸭（我们正在做的那只） |
| `public/templates/go2w-lab2mj` | `tests/fixtures/README.md:9` | Unitree **Go2** 四足，模型来自 `lab2mj` 导出链 |

> **2026-11-11 更正**：模板清单不是「未确认」，可以直接看到——产品站 `https://botlab.enkeebot.com/` 的
> **文件 → 打开示例** 就是项目列表（9 项：空白 + 8 个模板），而且每个模板的文件按
> `https://botlab.enkeebot.com/templates/<id>/<路径>` 直接可取。逐项见 §6。

---

## 2. botlab 的能力面（按 SDK 的真实方法签名）

### 2.1 机器人模型与动作

`Robot`（`sdk/_robot.py`）提供：

| 类别 | 方法 |
|---|---|
| 动作输出 | `position_pd(target, kp, kd, effort_limit)`、`velocity_pd(target, kp)`、`effort_control(effort)`、`apply_action(action)` |
| 执行器参数 | `action_scale()`、`kp()`、`kd()`、`effort_limit()`、`default_angles()`、`action_dimension()` |
| 关节顺序 | `policy_order()`、`motor_order()`、`default_joint_positions()` |
| 指令 | `commands(gamepad, keyboard, magnitude, parameters, ramp_rate, rest, waypoint)` |

### 2.2 观测源（这是差距最大的一块）

| 观测 | botlab 方法 | 我们（ducksir）现状 |
|---|---|---|
| IMU 角速度/线速度 | `base_angular_velocity()`、`base_linear_velocity()`、`imu()` | ✅ `sensor:imu_ang_vel` |
| 投影重力 | `projected_gravity()` | ✅ `gravity:body:<name>` |
| 关节位置/速度 | `joint_position()`、`joint_velocity()`、`qpos()`、`qvel()` | ✅ `qpos:joints`、`qvel:joints` |
| 上一帧动作 | `last_action()` | ✅ `last_action` |
| 基座高度 | `base_height()` | ✅ `base:z`（观测配方可用） |
| body 位姿 | `body_position()`、`body_quaternion()` | ✅ `body:pos:<body>`、`body:quat:<body>`（2026-11-11） |
| body 线/角速度 | `body_linear_velocity()`、`body_angular_velocity()` | ✅ `body:vel:<body>`、`body:angvel:<body>` |
| 接触 | `contact(targets, kind)` | ⚠️ `contact:<body>` **仅宿主**；浏览器侧待核对 mjContact 偏移 |
| 高度扫描（地形感知） | `height_scan(points, geom_groups, …)` | ❌ |
| 任意 MuJoCo 量 | `mujoco(query, size, scale, start, target)`、`read(query, …)` | ❌（我们故意不开放任意查询，见 §4） |
| 命名传感器 | `sensor_data(name, size)` | ✅ 同 `sensor:` |
| IMU 外参 | `imu_extrinsic()` | ❌ |

### 2.3 相机

`Sensor.rgb()` / `.depth()` / `.depth_frame()` —— 可以拿视觉观测喂策略（headless 下需要 EGL）。
我们：**❌ 完全没有**（浏览器与宿主都没有渲染相机取图）。

### 2.4 动作/轨迹素材

`Motion.source()` / `.sample(time, channel, indices, interpolation, loop, preview)` —— 重定向动作按时采样。
我们：❌ 没有 Motion 概念（只有 `phase_clock` + 关节相位表，够做站/坐/踢，不够做「播放一段重定向动作」）。

### 2.5 场景

`Scene` —— 场景里放物体/多机器人。我们：❌（只加一块地面；多机器人未做）。

### 2.6 控制编排

`project(panels=…)`、`control_profile(robot, state_node, vx, yaw_rate, actions, motion_states, states, experience, feedback, budgets)`、
`action(name, event_value, allowed_states, target_states, capability, idempotent, task)`。
我们：✅ 等价物是 `fsm.blob`（14 状态 + 动作脉冲 + 预算），字段名不同但能力对得上。

### 2.7 运行方式

| botlab | 我们 |
|---|---|
| 浏览器（MuJoCo WASM + ORT Web），`botlab run` 起本地编辑器 | ✅ 已对齐（`play.html?bundle=` + 工作台） |
| `--native`（CPython + 原生 mujoco/onnxruntime） | ✅ `ducksir run bundle/`（`sim/host/run_bundle.py`） |
| `--headless`（无 viewer 的 CI 跑法） | ✅ `--no-browser` + `--report` |
| HIL（硬件在环，`skills/hil-rdk-x5`） | ❌ 未做（我们有 `runtime/` 端侧，但不是 sim 内 HIL） |
| control hosting（把控制托管给浏览器） | ❌ 未做 |

---

## 3. 差距清单与优先级

| 优先级 | 能力 | 为什么 | 工作量（粗估） |
|---|---|---|---|
| ~~P0~~ | ~~观测源补 `body_*`、`base_height`~~ | 已完成（2026-11-11）：宿主 + 浏览器都有，`sim/test/test_obs_sources.py` 钉住语义 | ✅ |
| **P0** | `contact` 的浏览器侧 | 宿主已有；浏览器端要核对 mjContact 的字段偏移（不猜） | 小 |
| **P0** | 通用化到「任意 MJCF 机器人」验收一遍（用 Go2 当样本） | 这是 botlab 模板库的第二个模板；能跑通它才叫「支持项目」 | 中：canonical 导入器 + preset + obs 配方各来一份 |
| **P1** | `height_scan` | 地形/楼梯类策略的前提 | 中 |
| **P1** | `Motion`（重定向动作采样） | 跳舞/表演类项目 | 中 |
| **P2** | `Sensor.rgb/depth`（相机） | 视觉策略；浏览器侧还要相机渲染 | 大 |
| **P2** | `Scene` 多物体/多机器人 | demo 场景 | 中 |
| **P2** | HIL | 上板前的半实物闭环 | 大 |

---

## 4. 一条我们**故意**不跟的地方

botlab 有 `Robot.mujoco(query, …)` / `read(…)`：允许在声明里写任意 MuJoCo 查询。
我们不开这个口子（`docs/plan` 的 Q9=B）：**跨端可复现的东西才进图**。
任意查询在浏览器 wasm 端要复刻同一语义，成本高、易漂移。需要新量时，按三条准入标准扩节点/观测源，而不是开逃生口。

---

## 5. 结论

- 对齐 botlab 的正确做法：**把能力面补齐，而不是去列机器人清单**；
- 现在「能跑别人的项目」的本钱：契约驱动（contract.json）+ 一套 15 节点图 + 浏览器/宿主两个 target + 三路对拍；
- 差的第一块是**观测源**（body/contact/height）和第二块是**第二个机器人模板（Go2）**——这两块做完，
  才能说「botlab 的模板我们也能跑」。


---


---

## 6. botlab 项目清单（文件 → 打开示例，2026-11-11 实测）

在 `https://botlab.enkeebot.com/` 里点 **文件 → 打开示例**，得到 9 项（空白 + 8 个模板）。
每个模板的文件按 `/templates/<id>/<相对路径>` 直接可取（实测全部 200），因此下面这些数据都是从
各模板的 `controller.py` 与 `config/runtime.yaml` 里读出来的，不是猜的。

| # | 示例（菜单文案） | 模板 id | 机器人 | 频率 | 策略/模型 | 关键能力 |
|---|---|---|---|---|---|---|
| 1 | 空白项目 | — | — | — | — | 从零开始 |
| 2 | Unitree G1 GMTrack 跟踪 | `g1-gmtrack` | G1（`assets/robot/scene.xml`） | 50/200 | 1 个 ONNX（`stage2_actor.onnx`）+ 1 段动作 `current.npz` | `Motion` / `motion_reference`、`clock`、`history`、`rotate` |
| 3 | G1 Hiking Parkour 策略 | `g1-hiking-parkour` | G1 29DOF（instinctmj 场景 + 鞋） | 50/200 | 2 个 ONNX（深度编码器 + actor） | `history`、**深度感知** |
| 4 | Instinct-SuperMimic | `g1-instinct-supermimic` | G1（`assets/robot/scene.xml`） | 50 | 1 个 ONNX | `clock`、`history`、`mode_select`、相对位置/姿态 |
| 5 | G1 PHP Parkour 策略 | `g1-php-parkour` | G1（带地形场景） | **50/500** | 2 个 ONNX（深度 backbone + actor） | 感知跑酷、深度 |
| 6 | UniSuperMimic | `g1-unisupermimic` | G1 | 50 | 1 个 ONNX | 99D history + 730D goal、四元数运算、`rotation_6d` |
| 7 | WBC 模板 | `g1-wbc` | G1 29DOF | 50/200 | **4 个 ONNX** + 动作 `dance12` | `policy_bank` + `mode_switch`、`motion_reference` |
| 8 | Go2W Lab2MJ 策略 | `go2w-lab2mj` | Go2W 轮足四足（`scene_parkour.xml`） | 50/200 | **8 个 ONNX**（motion_tracking、vtm、vtm_lstm_sru×3、vtm_gru_sru…） | `PolicyController`、`policy_bank`、`history`、`mode_select/switch`、地形 |
| 9 | MicroDuck 多策略 | `microduck` | MicroDuck | 50/200 | 7 个 ONNX（walking/sitstand/kick×2/roll/ground-pick/stand） | 项目内 `microduck.py` 自管观测 + 状态机（**我们已对标**） |

**机器人只有三类**：MicroDuck（双足小鸭）、Go2W（轮足四足）、Unitree G1（人形 29DOF）。

## 7. 这 8 个项目要用到的 SDK 能力（我们逐项对照）

| 能力 | 出现模板 | 我们 |
|---|---|---|
`Model(...)` 多模型资源 | 除 microduck 外全部 | ⚠️ 有 model 节点，但一份图只能挂一个会话（`mode` 选一个） |
`policy_bank` + `mode_switch` / `mode_select` | go2w、g1-wbc、instinct、unisuper | ✅ 等价（`fsm.blob` 的 `out:mode` + 多会话） |
`Motion` / `motion_reference`（npz 动作采样） | gmtrack、g1-wbc | ❌ 未做 |
`history(n, dim)` | 多数 G1 项目 | ✅ `history` 节点 |
`clock()`（相位/时间输入） | gmtrack、instinct、unisuper | ✅ `phase_clock` |
四元数信号：`quaternion_inverse/heading`、`relative_position/quaternion`、`rotate`、`rotation_6d` | unisuper、instinct、gmtrack | ❌ 未做（只有投影重力） |
`Sensor` 深度/RGB | hiking、php（深度编码器） | ❌ 未做（浏览器与宿主都没有相机） |
`delay` / `where` / `constant` / `concat` | go2w、instinct | ✅ `where`/`constant`/`concat` 有；**`delay` 没有** |
`read_config` / `resource_config` / 项目内 Python 库（`microduck.py`、`go2w_policy.py`） | 全部 | ⚠️ 我们有 manifest+preset，但没有「项目内 Python 模块」这一层 |

**结论**：我们已对标的是 **MicroDuck 模板**（9 个里的第 9 个）。要「都能跑」，缺的顺序是：
① `Motion`（动作采样）→ ② 四元数信号族 → ③ 深度相机 → ④ `delay` → ⑤ 项目内 Python 模块层。
其中 ① 和 ② 是纯图能力，跨端可复现，符合我们「能进图才进图」的判据；③ 是渲染/传感器，成本最大。

## 8. 附：同名但无关的另一个 BotLab

`Davidhidalgo/BotLab` 是一个**积木式教学编辑器**（Scratch 积木、React/Nx、`examples/` 20 个 `.botlab.json`、
机器人 Jumper/MicroDuck/StackChan、界面西班牙语），和我们参考的 `botlab.enkeebot.com`（Python DSL + MuJoCo + ONNX）
**不是同一个产品**，两者都叫 BotLab 但形态不同。它对我们的价值只有两条：
**按距离/到障碍停的闭环动作**、**面向人的积木 UI**。不要把它当成模板来源。
