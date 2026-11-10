# 与 botlab 的差异清单

> 日期：2026-11-10 ｜ 状态：已确认
> 对象：botlab（`botlab-python` 的 `src/botlab/` + 模板 `templates/microduck/`）vs 我们（`engine/` + `sim/` + `client/` + `contract/`）
> 与 [`node-parity.md`](node-parity.md) 的分工：那张表只对**图节点**；这张讲**子系统**。
>
> 每条都分三档：**行为等价**（能对拍）｜**形态不等价**（能力有，做法不同）｜**真缺口**（没有）。

## 0. 一句话结论

图的**行为**已经能对上：状态迁移逐帧相同、物理重放 200 步差 0.00000 m、native↔wasm 逐位相同。
差的是 botlab 的**产品外壳**——SDK 门面、统一 CLI、golden 快照、部署/远程控制子系统、重定向与场景。

## 1. SDK 门面（Python API）

**botlab**：`sdk/` 里是给用户的面向对象门面。

| 类 / 函数 | 提供什么 |
|---|---|
| `Robot` | `observe()`、`action_dimension()`、`default_joint_positions()`、`position_pd()`、`apply_action()`、`action_scale()`、`kp()`、`motor_order()`、`policy_order()`、`_calibration()`、`waypoint_tracking()` |
| `Simulation` | `robot()` / `model()` / `motion()` / `sensor()` / `scene()` 装配，`update()`、`from_project()`、`request_reset()`、`reset_to()`、`on_substep()` |
| `Signal` | `clip()`、`at()`、`compare()`、`gt/gte/lt/lte/eq/ne()`、`__getitem__` |
| `Resources` / `Parameters` / `OutputChannel` | 资源、参数、输出通道的声明对象 |
| `policy_bank()` / `policy_inputs()` | 多策略装载与输入绑定 |
| `motion_reference()` | 动作参考（重定向后轨迹）的读取 |
| `project()` / 声明层 | 项目 = 资源 + 状态 + 控制状态码 + 重生绑定 |

**我们**：没有 Python 门面。等价物是三层声明文件 + 一组脚本：

- `contract/robot/**`：canonical（base + overlay）、preset、位形；
- `contract/policy/*.manifest.yaml`：观测槽位、动作映射、频率；
- `sim/host/run_policy.py` / `run_fsm.py`：把上面的声明编成 blob 再跑；
- `tools/robot/*`：导入、导出、校验（C1–C9）。

**差在哪**：能力都在，但都要「知道文件怎么摆」。botlab 用户写的是
`sim = Simulation(); sim.robot("microduck").on_substep(...)`，我们写的是改 YAML + 跑脚本。
对我们的定位（内部编排台）不算致命，但**新人上手成本差一个量级**。

**值不值得补**：值得，但**不必照抄**。最小可用是给 `sim/` 加一个薄门面：
`DuckRobot(canonical=...).observe()` / `.apply(action)` / `Scenario(...).run(seconds)`。

## 2. 图与处理器（56 vs 15）

**botlab**：`_host.py:SUPPORTED_NODE_TYPES` 56 项；实现分 7 组模块
（`_parts` / `_math` / `_temporal` / `_modes` / `_output` / `_model` / 观测），
`build_registry()` 合并成一个注册表，**少一组就报错**（「本地运行时只有在每组都移植完才完整」）。

**我们**：15 种节点（`contract/plan/README.md` 是权威语义），能表达 14 状态 / 7 策略 FSM。

**差在哪**：`node-parity.md` 逐条记了。语义上被覆盖的约 25–33 种，剩下的分两类：

- **主动不等价（有理由，24 条）**：`obs-qpos`/`obs-qvel`（会把物理布局泄进图）、`proc-resize`（静默改语义）、
  `proc-pd`（双重 PD）、`proc-where`（两边都求值）、`out-mujoco`（核不碰物理，D9）、`proc-action-hook` 与
  `obs-mujoco-query`（逃生口，端侧没有对应物）、`proc-signal-send/receive`（单图内不需要）、`scene-*`（单机无地形）、
  `proc-onehot-index`/`proc-rule-index`/`proc-clock`/`proc-fsm`（我们有等价节点，只做一个）、`proc-reroute`、`proc-spatial-math`、`out-scope`。
- **真缺口**：`obs-camera`（视觉）、`obs-height-scan`（地形）、`proc-motion-reference` / `proc-motion-sample`（重定向）、`obs-base-lin-vel`。

## 3. golden 快照（`_runtime/golden_check.py`）

**botlab**：TS 引擎用 `UPDATE_GOLDEN=1` 导出 `tests/fixtures/processors.json`，Python 侧逐条重放：
`dt=0.02`、`time=index*0.02`、`index=step`，帧契约与导出端**逐字段对齐**。换实现（TS→Python）时一键验「行为没漂」。

**我们**：有 numpy oracle 对拍（`test_core_vs_oracle.py`，0.000e+00）、native↔wasm 对拍、宿主↔浏览器全链路对拍。
**没有的是「快照」这层**：快照允许你**先记录后验证**（改实现时不需要重新推导期望值），测试是「每次重新算」。

**差在哪**：我们是「有参考实现的测试」，他们是「无参考实现的回归」。前者更强、后者更耐改。
**值不值得补**：值得，很便宜——把现有 4 份 plan 的输入/输出按帧存成 `fixtures/processors.json`，
再加一个 `--update-golden` 开关。

## 4. 统一 CLI（9 个子命令）

**botlab**：`botlab run`（含 `--headless`、`BOTLAB_STEPS` 限步）、`build`、`validate`、`ui`、`target`、
`deploy`、`agent`、`control`、`docs`。`run` 会先「编译声明 → 建原生 MuJoCo → 装会话 → 跑 `main.py`」，
`main.py` 里的 `Simulation.run(...)` 把控制器注册进会话。

**我们**：一堆平行脚本（`sim/host/run_policy.py`、`run_fsm.py`、`tools/robot/check_entity.py`、`export_mjcf.py`…），
靠 `docs/env.md` 记命令。

**差在哪**：最容易踩的是**没有 `validate`**——现在「声明合法吗、图能编吗、MJCF 能 load 吗」分散在三个脚本里，
而且只能在编译期发现；没有 `build`（从声明产出可分发的 release：blob + 模型 + 报告）。
**值不值得补**：值得。`ducksir validate` / `ducksir build` / `ducksir run` 三条就能覆盖日常 90%。

## 5. 部署子系统（HIL + Control API）

**botlab**：

- `hil/`：`agent.py`（目标侧常驻）、`backends.py`（真机后端）、`devices.py`、`gateway.py`、`protocol.py`
  （**bounded length-framed JSON + float32 tensor 契约**，`MAX_MODEL_BYTES=32MB`、`MAX_FRAME_BYTES=48MB`）、
  `targets.py`、`artifacts.py`、`_onnx.py`。
- `control/`：`ControlClient` + `API_VERSION=1`、`PROTOCOL="botlab-control"`、`API_PREFIX="/control/v1"`，
  含 discovery / receipts（回执）/ serve_service。这是**机器人无关**的远程控制面。

**我们**：0。没有目标侧 agent、没有控制协议、没有回执、没有 aarch64 产物。

**差在哪**：这是「训练→仿真→部署」的最后一段，也是唯一完全空白的一段。而且它不是「编个 .so」那么简单：
设备发现、模型分发（32 MB 上限那种约束）、回执、掉线保护、看门狗，botlab 把每一件都做成了协议。

**值不值得补**：**最高价值**。但建议分两步：先做 `ducksir run --native`（同一份核在本机 aarch64 上跑），
证明核能在板上跑；再做协议层。

## 6. 浏览器运行时

**botlab**：`browser.py` + `_browser_runtime.py`（浏览器侧运行时桥）+ `_ui_sync.py`（服务端→UI 状态同步）
+ `_site.py` + `ui` CLI（管理内置 UI 快照）。浏览器里跑的是**同一张图**，UI 只是另一层。

**我们**：`client/viewer/play.html`——MuJoCo wasm 物理 + wasm 核 + ORT Web + 状态机全本地，
手机可用（操作坞、安全区、自适应像素比）。

**差在哪**：我们**没有 UI 同步通道**（服务端改参数、浏览器要跟着变），也没有内置快照的概念。
能力上「浏览器能跑」是等价的，形态上他们是「同一个引擎的两个 target」，我们是「两条各自实现的链」——
后者会在改语义时悄悄漂移（这次 W8d 的指令映射就是这么漂的）。

**值不值得补**：中等。真正的收益是**把两端收敛到同一份「图 + 声明」**，而不是加同步通道。

## 7. 面板与指标 schema（`_panels.py`）

**botlab**：`_TELEMETRY_KEYS`（`baseSpeed`/`targetSpeed`/`roll`/`pitch`/`yaw`/`torquePeak`/`freqHz`/`realtimeFactor`/`distance`/`trackingError`）、
`_RUNTIME_KEYS`（`controlFrequency`/`physicsFrequency`/`controlP95`/`deadlineMiss`/`fps`/`gpuFrameLoadPct`）、
`_TILE_IDS`（`view:attitude`/`view:actuatorForces`/`status:inferenceProvider`）、`_ARRAY_SOURCES`（`qpos`/`qvel`/`ctrl`/`sensordata`/`refq`/`errq`）。
**在编译期**就把显示元数据校验一遍（独立于控制执行）。

**我们**：`out_metric` 节点 + `--report` 的最小字段（`base_height_min`、`upright_final`、verdict）。
没有 schema、没有校验、没有面板。

**差在哪**：他们把「要显示什么」变成了**声明的一部分**，顺带保证了这些量在运行时真的存在。
**值不值得补**：值得但便宜——把 `--report` 的字段先规范化成上面那组键，够用。

## 8. 动作重定向（`_motion.py`）

**botlab**：`motion_reference(motion, *, mode, command, command_codes, active_mode, start, ...)` 读重定向后的
参考轨迹；对应节点 `proc-motion-reference` / `proc-motion-sample`；`_runtime/_motion.py` 负责热路径。

**我们**：0。Mimic / 动作模仿类策略目前无法接（我们的策略全是「命令驱动」的盲走型）。

**差在哪**：这是**一整类策略**的输入源，不是一个小节点。
**值不值得补**：取决于你要不要做 Mimic。要，就绕不开；不要，就先别做。

## 9. 场景与多机器人（`mujoco_scene.py` / `_compose.py`）

**botlab**：

- `_compose.py`：多机器人 MJCF 合成——展开 include、把 asset 路径改绝对、注入 auto-actuator、
  给每个名字加 `r{N}_` 前缀，再塞进主机器人 XML；作用域解析（每台机的 qpos/qvel/ctrl 切片）对齐 `TS` 的 `mujocoRobotScope`。
- `mujoco_scene.py`：导出模型与世界状态给显示层（字段与 `MujocoParser.ts` 一一对应）。

**我们**：单机器人、单地形；场景靠手写 XML（`--physics-only` 那条路）。
**差在哪**：多机是「合成 + 命名空间 + 作用域」三件事，任何一件偷懒都会在 3 台机以上炸。
**值不值得补**：中等偏低（除非你要做多机协同）。

## 10. 项目模型（`_declaration.py`）

**botlab**：项目 = `resources` + `states` + `control state codes` + `respawn binding`，
`project()` 负责把声明编译成运行时可用的结构，并且**在编译期校验**（本地化文本、计划值、资源来源、声明位置）。

**我们**：有 canonical（base + overlay）、preset、policy manifest，但**没有「项目」这一层**——
没有资源/状态的声明模型，重启策略（respawn）、控制状态码这些概念都没有。

**差在哪**：项目层是把「机器」「策略」「场景」「显示」串起来的胶水；没有它，每加一个 demo 都要写新脚本。
**值不值得补**：中等。可以先只做「项目 = 机器人 + 场景 + 策略清单 + 面板字段」四件。

## 11. 策略库存

**botlab**：9 个 ONNX —— `BEST_alpha_walking`、`BEST_alpha_sitstand`、`BEST_alpha_stand`、`alpha_ground_pick`、
`ball_kick_left`、`ball_kick_right`、`roulade`、`BEST_roller`、`BEST_roller_crouch`。

**我们**：7 个（缺 `BEST_roller`、`BEST_roller_crouch`）。

**差在哪**：缺的两个是「轮滑变体」（驱动 / 蹲滑），它们把同一台鸭子换成另一种运动方式。
**值不值得补**：便宜（拷文件 + 加 manifest 条目），主要成本是验证。

## 12. 发布形态

**botlab**：`pip install botlab`（wheel），带 `py.typed`。
**我们**：monorepo，`PYTHONPATH` 指向 `~/.local/share/ducksir/pylibs`，靠 `docs/env.md` 记。

**差在哪**：不可安装意味着**别人复现要读文档**。如果我们只内部用，这不算问题。

---

## 汇总：按「对你的目标」排

| 序 | 差异 | 档位 | 补的代价 | 不补的后果 |
|---|---|---|---|---|
| 1 | 部署子系统（HIL + Control API + aarch64 产物） | 真缺口 | 大（工具链 + 协议） | 「训练→仿真→部署」缺最后一段 |
| 2 | 统一 CLI（`validate` / `build` / `run`） | 真缺口 | 小 | 日常靠记命令，容易漏校验 |
| 3 | SDK 门面 | 形态不等价 | 中 | 新人上手成本高 |
| 4 | golden 快照 | 真缺口 | 小 | 改实现时只能靠测试重新推导 |
| 5 | 面板/指标 schema | 真缺口 | 小 | 报告字段各写各的 |
| 6 | 动作重定向 | 真缺口 | 大 | 做不了 Mimic 类策略 |
| 7 | 场景/多机 | 真缺口 | 中 | 只能单机单地形 |
| 8 | 项目模型 | 真缺口 | 中 | 每加 demo 写新脚本 |
| 9 | 两个 roller 策略 | 小缺口 | 小 | 少两种运动方式 |
| 10 | 浏览器 UI 同步 | 形态不等价 | 中 | 两端语义可能悄悄漂 |
| 11 | pip 发布 | 形态不等价 | 小 | 复现要读文档 |
