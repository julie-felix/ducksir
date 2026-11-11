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

### 还没做

- 浏览器侧（`play.js`）的扭矩 PD——现在只支持位置执行器；
- bundle 里还没有「策略序 ≠ 电机序」的表达（`post.blob` 假设同序），G1 要接 `reorder`；
- 扭矩模型 + 地面 + 完整 PD 跑我们的 `run_bundle` 还没端到端验（差这一步）。
