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

## 5. 找到权威参考了：`ccrpRepo/wbc_fsm`

botlab 的 WBC 预设里写了 provenance：`joint_order` 来自 `ccrpRepo/wbc_fsm include/FSM/State_WBC.h dof_mapping`。
顺着这条线把参考实现读出来了（`include/FSM/State_Loco.h` + `src/FSM/State_Loco.cpp`）：

| 项 | 参考值 |
|---|---|
| obs 顺序 | `[body_ang_vel(3), projected_gravity(3), cmd(3), q−default(29), qd(29), last_action(29)]` = 96 |
| `last_action` | **上一步被 clip 过的原始动作**（策略序） |
| 重力 | `QuatRotateInverse(base_quat, [0,0,-1])`（即 `R^T g`） |
| `dof_mapping` | `[0,6,12, 1,7,13, 2,8,14, 3,9,15,22, 4,10,16,23, 5,11,17,24, 18,25, 19,26, 20,27, 21,28]`（策略序 → 电机序） |
| `action_scale` | **0.25（统一值，不是 preset 里那组逐关节数）** |
| 目标角 | `ctrl[mapping[i]] = action[i] * 0.25 + default[i]` |
| Kp / Kd | `{200,150,150,200,20,20}×2, {200,200,200}, {100,100,50,50,40,40,40}×2` / `{5,5,5,5,2,2}×2, {5,5,5}, {2,…}` |
| 默认角 | `{-0.2,0,0,0.42,-0.23,0}×2, {0,0,0}, {0.35,0.18,0,0.87,0,0,0}×2(左右镜像)` |
| 指令范围 | vx ∈ [−0.6, 0.85]，vy ∈ [−0.4, 0.4]，wyaw ∈ [−1, 1]，`cmd_smoothes = 0.95` |
| h/c | 全零初始化；`enter()` 里先跑 1 次观测再看结果 |

## 6. 现在的状态：能跑，但站不住（未解决）

用上面这套规格在 MuJoCo 里跑 `loco_0731.onnx`（我们的 G1 + 上游 gains）：

```
z 起 0.75 → 0.121（最低 0.069）
直立度 0.019｜300 步（6 s）没报错，但倒了
```

按参考实现逐项对齐后仍然倒。已经排除的：obs 顺序、重力方向、`last_action` 语义（试过 raw/scaled/ctrl 三种）、
关节映射（补上 `dof_mapping` 后）、`action_scale`（0.25 vs preset 逐关节）、Kp/Kd（用参考数组覆盖 MJCF）。

**还没排除的**（按我判断的可能性排序）：

1. **初始位形**：参考实现的 `enter()` 从「当前电机位置」起，并且先把目标设成默认角；
   我们用 preset 的 `initialQpos`（z=0.793）起，两者不一定等价；
2. **力矩限幅**：上游 joint 有 `actuatorfrcrange="-50 50"`，我们的导出没有（`effortLimit` 是 None）；
3. **接触参数**：上游 geom 的 friction/condim 用默认，我们的导出从 URDF 来，可能不同；
4. **`cmd_smoothes` 平滑**：参考里指令要一阶低通，我们是硬写 0。

下一步要做得更省时间：把参考实现的 **obs/action 中间值**拿一份（或让 botlab 的模板在浏览器里跑一段录下来），
逐帧对比就知道差在哪一项——盲扫已经试了 12 组，都不行。

## 7. 录到真值了：我们的接口是对的，倒的是他们的策略本身

自己录的办法（两条路，最后走通第二条）：

1. 反代他们的产品站、在 `SimulationEngine` 里把推理入口 `S()` 包一层 —— 抓到了 24 条记录，
   但 MuJoCo worker 在反代下一直卡在「正在启动」，录到的都是启动前那次（obs 全零）。放弃。
2. **让 botlab 自己的 native 运行时跑模板**，在 `onnxruntime.InferenceSession.run` 上挂钩子录 feeds/输出
   （`tools/dev/botlab_record_native.py`）。WBC 模板 headless 跑 150 步，录到 150 条真值。

### 对拍结果（决定性）

拿他们第 0 步的 96 维 obs 喂给我们自己的推理路径：

```
我方 action[:6] [ 0.158303 -1.249568 -0.312786  0.980323 -0.16619   0.144863]
他们 action[:6] [ 0.158303 -1.249568 -0.312786  0.980323 -0.16619   0.144863]
最大差 4.8e-07 → 相同
```

**obs 布局、顺序、缩放、模型调用全都对**。所以之前「站不住」不是我们的接口问题。

### 但他们的策略在他们自己的运行时里也倒

真值记录的 obs 演化（用他们自己的 G1 场景跑 WBC 模板）：

```
rec  0 ang=[0 0 0]      grav=[0 0 -1]        qmax=0.00
rec 30 ang=[-2.4 -3.6 2.4] grav=[-0.94 0.33 -0.03] qmax=1.22 qdmax=16.7
rec 80 ang=[-2.5 0.7 -1.4] grav=[-0.99 -0.05 0.13] qmax=0.85 qdmax=11.9
```

第 30 步重力就翻到 x 轴（≈90°），说明**它也倒了**。也就是说：
loco 这个策略单独从 preset 给的那个初始位形起跑、没有 WBC 的上下文时，本来就不站。

**结论**：我们不该再调自己的探针。要做 G1 运控，得先确认「哪个策略、从哪个初始状态、在什么模式下」
才真的能站——这一步在 botlab 的语义里就是「WBC 模板整体的一个 mode」，不是单拿 loco 就能跑。

## 8. 又一个结构性发现：G1 的执行器是「扭矩 + 引擎侧 PD」，不是「位置执行器」

站点共享的 G1 模型 `models/g1/g1_29dof.xml` 用的是 **`<motor>` 执行器**：

```xml
<motor name="left_hip_pitch" joint="left_hip_pitch_joint" ctrlrange="-88 88" />
```

而模板里那份（Instinct 用的 `assets/robot/bare.xml`）用的是 **`<position kp= kv=>`**。

这两者语义完全不同：

| | botlab 的 G1（站点共享） | 我们现在的导出 |
|---|---|---|
| MJCF 执行器 | `<motor ctrlrange="±88">`（**扭矩**） | `<position kp kv>`（位置） |
| PD 在哪算 | **引擎算**：`tau = kp*(target−q) − kd*qd`，再用 `effortLimit` 截断 | MuJoCo 内部算 |
| 策略输出怎么用 | `target = default + 0.25*action`，再走 PD | 直接当位置目标 |

我们的 `export_mjcf.py` 只会导 `<position>`，等于在 MuJoCo 里又套了一层 PD。
对鸭子没问题（它的策略就是位置增量），但**对 G1 这类扭矩策略就不等价**：
它的 `effortLimit` 截断、以及 PD 的采样率（每个控制步一次，不是每个物理子步）都没了。

**要做的事**（下一轮）：契约里表达「执行器模式」——`position` 还是 `torque+PD`，
pcrest 时把 `kp/kd/effortLimit` 写进 preset，宿主/浏览器各自按同一套公式算扭矩。
这是「接口对齐了、物理还没对齐」里剩下的那一半。

### 顺带：他们 native 路径本身也不可靠

用站点扭矩模型跑他们的 native 运行时，机器人直接**穿过地面**（z → −42）——共享场景里没有地面。
换成 Instinct 那份带地面的场景，loco 也照样倒（z 0.79 → 0.19）。
所以「在 botlab 里能不能站住」这件事，目前只有**浏览器 target** 能作数；native 路径的结论不能当基准。

## 9. 用 botlab 的真值把执行器链路钉死了

录真值时顺手把 `d.ctrl` 也从 `mujoco.mj_step` 里抓了出来（`tools/dev/botlab_record_native.py` 的同源做法），
于是可以直接对比「他们写进 ctrl 的值」和「我们按公式算的值」：

```
step2:  差(动作直接当电机序) = 34.63   ｜  差(按 dof_mapping) = 0.80
step5:  差(直接)             = 33.35   ｜  差(映射)          = 0.75
step10: 差(直接)             = 58.26   ｜  差(映射)          = 0.58
```

**结论：`tau = kp·(default + 0.25·action − q) − kd·qd`，动作按 `dof_mapping` 从策略序换到电机序，
再按 `effortLimit` 截断**（G1 的 effortLimit 是逐关节的：5 / 25 / 50 / 88 / 139）。
直接当电机序会差 30–58，映射后只差 0.6–0.8（残差来自我们取 q/qd 的时刻与它不同）。

### 已经落地的部分

| 东西 | 状态 |
|---|---|
| `tools/robot/import_botlab_preset.py` | 把 botlab preset（kp/kd/effortLimit/actionScale/默认角/初始位姿）转成我们的 preset，并标 `actuatorMode` |
| `tools/robot/export_mjcf.py` | `kind=motor` 的实体导出 `<motor ctrlrange=…>`（扭矩），否则 `<position kp kv>` |
| `contract/robot/types/g1-29dof-torque.yaml` | 站点共享的扭矩模型（29 `<motor>`） |
| `sim/host/actuators.py` | 两端共用的执行器语义：策略序→电机序、目标角、扭矩公式 + 截断 |
| `sim/host/run_bundle.py` | `preset.actuatorMode == "torque"` 时按 PD 算 ctrl |
| `sim/test/test_actuators.py` | 13 条断言（映射/截断/两种模式/真实 G1 preset） |

### 端到端已经跑通（`tools/dev/g1_bundle_smoke.py`）

```
z 起 0.793 → 0.293（最低 0.293），直立 0.524，共 82 步
PASS：扭矩模型 + 子步 PD + loco（判据：≥60 步；botlab native 自身约 80–120 步）
```

两个关键结论都是实测出来的：

1. **PD 必须每个物理子步重算**。控制步算一次、4 个子步保持不变 → 只能撑 20 步；
   子步重算 → 82 步。这也解释了 MuJoCo 的 `<position>` 为什么更稳：它本来就是子步算的。
2. **策略序 ≠ 电机序必须显式映射**。botlab 的 JSON preset 里 `policyJointOrder/motorJointOrder`
   都是恒等，真正的映射在 `ccrpRepo/wbc_fsm` 的 `dof_mapping` 里。不映射 → 35 步就倒；映射后 82 步。

### 浏览器侧也补上了

- `client/viewer/actuators.mjs`：与宿主 `sim/host/actuators.py` **同一套公式**（策略序→电机序、目标角、扭矩 + 截断）；
- `client/viewer/play.js`：扭矩模式下**每个物理子步**重算 PD（与宿主对齐）；
- `contract.json` 的 `robot` 段新增 `actuator_mode` / `kp` / `kd` / `effort_limit` / `motor_order`，两端都从契约读；
- `sim/test/test_actuators_parity.mjs`：同一组随机输入分别喂给两端实现，**逐元素一致（0.00e+0）**。

### 还差

loco 单独跑本来就站不久（botlab native 自己也是 80–120 步）；要真站住得接需要
`motion_reference` 的 WBC/AMP mode，那是 `Motion` + 多策略的下一个台阶。

## 10. WBC mode 的观测（439）拆开了，并用真值逐段核对

规格来自 `ccrpRepo/wbc_fsm` 的 `State_WBC.{h,cpp}`；数据用 botlab 运行时录的真值
（`/tmp/cap-wbc-mode3.jsonl`，初始 mode=3）逐段对过：

| 段 | 长度 | 内容 | 核对 |
|---|---|---|---|
| 参考·关节位置 | 29 | 动作参考当前帧 `joint_pos` | 与 motion 第 1 帧逐位相同 |
| 参考·关节速度 | 29 | 同帧 `joint_vel` | 逐位相同（max diff 0.000） |
| 参考·锚点位移（机体系） | 3 | 相对上一参考帧 | 首步 `[0,0,0]` ✓ |
| 参考·锚点朝向 6D | 6 | 相对旋转矩阵 `[m00,m01,m10,m11,m20,m21]` | 首步 `[1,0,0,1,0,0]` ✓ |
| 机器人状态 | 93×4 | `[角速度3, 投影重力3, q29, qd29, 上一步动作29]` | 4 帧各一个 `[0,0,-1]`，位置 72/165/258/351 ✓ |

我们自己装配的参考段与真值差 **5.0e-06**（float32 精度内一致）。

实现：`sim/host/wbc_obs.py`（`reference_block` / `robot_state_frame` / `assemble`），
测试：`sim/test/test_wbc_obs.py`。参考帧索引按参考实现夹到 `[1, end]`，暂停时参考速度强制为 0。

### 播放头（已实现，并用真值反推验证）

`wbc_obs.Playhead`：起始帧夹到 1、每控制步推进 `frame_interval`、暂停时 `idx == last_idx` 且参考速度置 0、
越界夹到 `end`。用真值反推：每步 `obs[0:29]` 匹配 motion 第 **1,2,3…** 帧（误差 5e-06），
说明这次录制是**每步推进 1 帧**。

### 锚点规则：解出来了 —— 推 1 帧、回看 5 帧

用「扫描 (idx, last) 找能复现真值 6D 的组合」的办法，前 10 步结果很整齐：

```
step 1 → (2,1)   step 2 → (3,1)   step 3 → (4,1)   step 4 → (5,1)   step 5 → (6,1)
step 6 → (7,2)   step 7 → (8,3)   step 8 → (9,4)   step 9 → (10,5)  step 10 → (11,6)
```

即：**播放头每控制步推进 1 帧，锚点回看 `frame_interval = 5` 帧**（`last = max(1, idx−5)`）。
与参考实现的两行对上：

```cpp
int idx      = _refer_idx + i * _frame_interval;   // i=0 → idx = _refer_idx（每步 +1）
int last_idx = idx - _frame_interval;             // 锚点回看 _frame_interval 帧
```

验证（第 10 步）：

```
推 1 帧 + 回看 5 帧  → max|Δ| = 4.09e-05   ✅
反例：回看 1 帧      → max|Δ| = 7.86e-03   （差 200 倍）
```

**完整 439 维**（参考 67 + 真值状态历史 4×93）与真值差 **4.09e-05**。

### 一处仍未对齐

`step ≥ 20` 之后误差升到 1e-3 ~ 1e-2：我按「每步 +1 帧」推进，但那一跑后期显然不是每步都推
（可能 mode/pause 状态变了，或录制侧与播放侧节拍不同）。前 19 步在 1e-4 以内，后面偏差
来源未定位——记录在案，没有硬凑。

### 还差

- 锚点 `last_refer_anchor_*` 的更新规则（前 5 步对上、第 10 步偏）；
- 439 维 obs 还没进 bundle 的观测配方（`contract.json` 没有「动作参考 + 4 帧历史」组合槽位）；
- `contract_obs` 还没有 `motion_ref` 这个 source。

## 11. WBC 模式在我们自己的链路里跑起来了（12 秒不倒）

`tools/dev/g1_wbc_smoke.py`：439 观测 + 参考动作播放 + 扭矩 PD，复刻 botlab `g1-wbc` 的 mode 3。

```
$ WBC_STEPS=600 WBC_UNPAUSE=10 python3 tools/dev/g1_wbc_smoke.py
模型输入 ['obs']（第一维 439），输出 ['actions']
z 起 0.793 → 0.659（最低 0.649），直立 0.995，共 600 步
跟踪误差：前 20 步均值 0.287 rad，后 20 步均值 0.127 rad
PASS
```

600 控制步 = **12 秒**，基座高度在 0.65–0.79 m 之间，直立度 0.995，跟踪误差从 0.287 rad
降到 0.127 rad（越跟越准）。**这是 G1 第一次在我们自己的核里真正站稳并跟动作。**

### 复刻到的三处语义（都和 loco 不同）

| 项 | loco（mode 2） | WBC（mode 3） |
|---|---|---|
| 观测 | 96 | **439**（参考 67 + 状态 4×93） |
| 目标基准 | `defaultJointPositions` | **`referenceJointPositions`**（preset 里那组，不是默认角） |
| 增益 | `balanceStiffness/Damping` | `robot.kp/kd` |
| 动作参考 | 无 | 每控制步 +1 帧、锚点回看 5 帧、进 mode 前暂停 |

动作缩放仍然是 **0.25 统一**：`target = reference + 0.25 × action[motor_order]`。

### 还没做

- 这是宿主侧（Python + MuJoCo）跑通；**浏览器侧还没接**（要 model_rnn 之外再加「动作参考 + 4 帧历史」的图）；
- 动作参考还没进 `contract.json`（现在是冒烟脚本里直接装配的）；
- mode 切换（loco ↔ WBC）还没接 `state_machine`。

## 12. 把 WBC 接进契约/bundle：管道通了，端到端还没站住

### 已落地（都有单测）

| 改动 | 说明 |
|---|---|
| `contract_obs` 新增 `motion_ref` source | 67 维动作参考由运行期按播放头算好传进来 |
| `contract_obs` 的 `body:angvel` 默认转机体系 | 对位 botlab 的 `body_angular_velocity(frame="local")`（`frame: world` 可关） |
| `export_artifacts.build_pre` 支持 `obs_history` | 状态段在图里用 `history` 节点展开：160 → 439，5 个节点 |
| `export_artifacts` 写 `preset.yaml` 进 bundle | 运行期靠它判断执行器模式与 PD 增益 |
| 契约 `robot` 段加 `motion` / `reference_joint_positions` | 动作参考路径、fps、frame_interval、active_mode、参考目标角 |
| `run_bundle` 支持动作参考 + mode 切换 | 播放头随 fsm 的 mode 起停；WBC 的目标基准换成 `referenceJointPositions` |
| `contract/policy/g1-wbc.manifest.yaml` | 439 观测 + `obs_history` + 策略清单（manifest 驱动，不再写死鸭子那 7 个） |
| `sim/test/test_pre_history.py` | 历史展开的图：结构逐字节一致 + 语义（最老在前/尾部最新）正确 |

`build/release/g1-wbc` 能导出：`pre 5 节点 160 → 439`、`post 5 节点 29 → 29`、`fsm 66 节点`。

### 三个 bug 修完，bundle 端到端站稳了

```
$ python3 sim/host/run_bundle.py --bundle build/release/g1-wbc --mjcf build/mjcf/g1-29dof-torque.xml --steps 600
bundle g1-wbc｜600/600 步｜28.5× 实时
  基座 终 +0.682 最低 +0.645 m｜直立 +0.994
```

同样是同一套观测/模型/PD，之前 20–40 步就倒。逐个定位出来的三个 bug：

| # | bug | 症状 | 修法 |
|---|---|---|---|
| 1 | `actionScale` 用了 botlab JSON 里的**逐关节值**（0.29/0.35/0.44/0.55） | 目标角整体偏大，越调越倒 | 参考实现是**统一 0.25**（`State_Loco/WBC` 里写死），导入时强制 0.25，JSON 原值留档在 `actionScaleFromPresetJson` |
| 2 | `post` 的 `reorder` 传了正置换 | 动作张冠李戴（`t_b[1]=-0.725` vs 正确 `+0.649`） | `reorder` 节点是 **gather**（`out[i]=x[perm[i]]`），要 scatter 必须传**逆置换** |
| 3 | 观测里的关节量用了电机序 | 历史段与真值差 8.4，策略看到错姿态 | 策略吃的是**策略序**（`obs[73:102]` 对应 policy order），动作输出再在 post 里换回电机序 |

定位方法：把「直接装配」和「bundle 路径」在**同一初始状态下第 0 步的 160/439 维 obs 与 target** 打出来逐维比——
差异 0 就说明两条路在起点等价，接着逐帧往后比。第 2 个 bug 就是这一步暴露的（target 差 1.37 rad）。

### 回归

- `sim/test/test_g1_wbc_bundle.py`：300 步、基座 ≥0.55 m、直立 >0.9（新）；
- `test_golden` / `test_pre_history` / `test_project_run` / `test_actuators` 全绿（鸭子不受影响：它的
  preset 没有非恒等 `motorJointOrder`，post 不插 reorder）。

## 13. 浏览器里也能跑了（工作台点开即可）

![浏览器里跑 G1 WBC](../images/viewer-g1-wbc-browser.png)

`client/viewer/play.html?bundle=/build/release/g1-wbc/`：439 观测、动作参考播放、扭矩 PD 全在浏览器里跑，
基座 0.74 m、控制 ~50 Hz、1.0× 实时。工作台的 bundle 列表会自动列出 `build/release/*`，点开即是这一页。

### 这一轮补的四件浏览器侧能力

| 能力 | 位置 | 验证 |
|---|---|---|
| `.bin` 动作容器解析（头长度唯一化） | `client/viewer/motion.mjs` | 与宿主逐值一致（≤1e-6） |
| 播放头（推 1 帧 / 看 5 帧 / 暂停 / 夹取） | `motion.mjs` `Playhead` | 与宿主同语义 |
| 参考段 67 维（锚点 6D + 偏航对齐） | `motion.mjs` `referenceBlock` | 与宿主差 **1.46e-11** |
| `motion_ref` 观测 + `body:angvel` 机体系 | `play-core.mjs` | 三处 bug 修完后站稳 |

跨端一致性测试：`node sim/test/test_motion_parity.mjs`。

### 又抓到三个 bug（都在浏览器侧）

1. **扭矩分支把目标角当原始动作又缩放了一次** → 相当于 `default + 0.25×(default+0.25a)`，机器人 20 步内倒；
2. **每个控制步推进了 8 个物理子步**（扭矩分支自己 4 步 + 公共逻辑又 4 步）→ 时间轴对不上；
3. **观测的关节量没按 `order: policy` 重排** → 策略看到错姿态。

修完：`node tools/dev/browser_bundle_check.mjs` → 基座 0.743 m，PASS。

### 渲染兜底

bundle 里没有手搓的 GLB 索引（那是鸭子专用），所以 `play.js` 新增了
`renderFromModel()`：直接用 MuJoCo 的 `mesh_vert/mesh_face` 建 three.js 几何体，
按 `geom_rgba` 上色，相机按机器人自身包围盒取景（鸭子 ~0.2 m、G1 ~1.3 m 差一个数量级）。

### 还差

- 浏览器侧的 `fsm` 模式切换（现在单策略 bundle 默认常开 WBC；多 mode bundle 还没接）；
- 工作台里如果 bundle 很大（G1 带网格 31 MB），首次加载要几十秒，可以再压网格。
