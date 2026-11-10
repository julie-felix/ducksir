# BotLab 复刻计划：编排台 + 可搬上板子的执行核

> 日期：2026-11-10 ｜ 状态：**已确认，当前阶段**（2026-11-10 起）
> 事实来源：`botlab-deep-note-2026-11-10.md`（产品实测）、`/tmp/botlab-research/botlab-python`（源码，HEAD `e201151`）、本仓 `contract/robot` 与 `tools/robot` 现状。
> 口径：「实测」= 今天当场核过；「源码」= 读 botlab-python 得到；「本仓」= ducksir 里已有的东西。

---

## 0. 一句话

**用一份自写的 C++ 执行核 + 一张编译期生成的图，把「多份 ONNX 的编排 → 原生 MuJoCo 里看 → 同一份图搬上板子」这条链打通。botlab 只借契约形状，不借代码。**

它要解决的原始痛点（庆喆原话）：各种训练出来的 ONNX 产物都能很方便地编排使用，先在仿真里看，之后直接打通部署。

---

## 1. 决策表（逐条都有出处，改一条要改这一行）

| # | 决策 | 理由 |
|---|---|---|
| D1 | 目标定位是**编排台**，不是验证闸门 | 痛点不在「报告判读」，在「一堆 ONNX 怎么组合起来用」；报告降为副产品 |
| D2 | 落在 **ducksir**（`sim/` + `engine/`），不做独立 web 应用、不进 DSH 插件 | 插件被 DSH 构建/沙箱绑住；端侧不需要 UI |
| D3 | 1:1 的判据 = **接口契约等价 + 行为等价**；不复刻它的 UI/交互/发布托管 | UI 是无限工程量且零验收价值 |
| D4 | 复刻 L1–L4（声明层 / 图 IR + 编译校验 / 执行器 / 运行时）；L5 只留 `--report` 与静态站点；**HIL 不做** | 它的 HIL 是 model-service、agent 无鉴权无加密，上不了产线 |
| D5 | 「推理」这条线全自研：obs 组装 → 前向 → 后处理 → 关节重排/仿射 | 训练/仿真/部署要共用一份；它的打包库把这段埋进编译产物，外面接不上 |
| D6 | **内部自用、不发布衍生库**；官方库只在 dev 环境当抽查对照，不进生产 | `license = Proprietary`，仓库无 LICENSE、前端闭源（源码核对） |
| D7 | 执行核 = **C++20**，单文件 + arena 分配 + 零全局态，目标 ≤2000 行 | ORT 的 wasm 静态库路径只有 C/C++ 是官方支持的（见 D8） |
| D8 | 核内含推理，但只经三个函数：`create / prepare / run`；native 链 `libonnxruntime.so`，wasm 首选自己编的 `libonnxruntime_webassembly.a`，崩则回落宿主回调 | ORT 官方 FAQ 明确支持 `--build_wasm_static_lib`；但源码构建 1–2 小时 + emsdk 版本冲突，必须留逃生口 |
| D9 | **MuJoCo 不进核**；物理留在宿主（native 用 python 绑定，浏览器用官方 mujoco wasm） | 核的边界就是 `obs→action`；进核就变半仿真器，实时性与可测性一起崩 |
| D10 | 编排 = **Python 声明式 DSL → 编译成图 → 运行期只跑图**（混合，不是纯图也不是纯 Python） | 写得像 Python、跑起来是可校验的确定性产物 |
| D11 | 图的两副形态：**JSON 是源码，blob 是产物**。blob 头带 `magic/version/abi/requiredNodes[]/digest` | JSON 留在能 diff、能 review、能给 AI 读的那侧；blob 只解决零解析与可校验 |
| D12 | obs/action 语义写进 **`contract/policy/manifest`**，每个 ONNX 产物带一份填好的 manifest，图只消费不猜 | 「策略换了图没换」必须是编译期红，而不是静默跑歪 |
| D13 | **canonical 是唯一真源**；`preset.yaml` 由 canonical 生成；`kp/kd/effortLimit` 进 canonical，`actionScale/imuExtrinsic` 留 preset | botlab 实际只读 13 个 preset 字段（源码），其中 8 个我们已有、3 个是真机属性、2 个与策略绑定 |
| D14 | MJCF **从 canonical 导出**（不走 `third_party` 那份直接改）；禁止 exporter 私有字段；schema 改动走 ADR + 版本号递增 | 「有问题就优化现在的系统」——但私有字段会把 base+overlay 的 round-trip 零差异作废 |
| D15 | 节点集 27 个（见 §6 与 `docs/node-parity.md`）；逃生口允许但编译期打 `native-only` 标，出部署产物时拒绝 | 端侧没有对应物，允许就等于三端不一致 |
| D16 | 部署产物 = **整套 plan**（blob 图 + ONNX + preset 快照 + digest），端侧一个解释器加载 | 图本来就是部署单元；`incident_id` 链能直接挂上 |
| D17 | `--report` 出最小 JSON，逐帧另开 `--trace` 走 npz | 默认产物要能进 CI/git 与 incident 链 |
| D18 | 对拍 oracle = **自写 numpy 参考实现**（27 节点，只进测试）为主；官方库抽查为辅 | 接官方库会强迫兼容它的工程形状；numpy 那份本身就是节点语义的可执行规格 |
| D19 | 范围扩版、全情投入、**9 周**（2026-11-10 → 2027-01-11），含 W3 止损点 | 庆喆选 B（范围一起扩）+ 全情投入；浏览器要交互，比只读多一周 |
| D20 | 浏览器端是**可交互控制台**，不是只读查看器：相机拖拽 + 键盘/滑条驾驶（走 `commands`）+ 参数面板实时改 + 策略/模式切换；全部经同一条 WS 通道下发 | 只读台看不出「编排对不对」——编排的价值就在切模式、调参数那一刻；但交互必须走图，否则仿真看到的和端侧跑的是两条路 |

---

## 2. 架构

```
  编排（人 / AI）                  产物                              运行
┌──────────────────┐      ┌────────────────────┐      ┌─────────────────────────┐
│ Python DSL        │      │ plan.json（源码）   │      │ C++ 核（同一份代码）      │
│ project/robot     │─────▶│ plan.blob（产物）   │─────▶│ arena + 图节点 + ORT     │
│ policy.manifest   │ 编译 │ 头：version/abi/    │ 加载 │   ↓ action[14]          │
└──────────────────┘      │ requiredNodes/digest│      └───────────┬─────────────┘
                          └────────────────────┘                  │
       物理与总线在宿主侧（核不碰）                                  │
┌───────────────────────────────────┐                          │
│ native: MuJoCo python 绑定         │◀── obs ──────────────────┘
│ browser: mujoco wasm + 可交互控制台│
│ 端侧: 舵机总线 + 50 Hz 环          │
└───────────────────────────────────┘
```

三个平面（沿用 README 的三平面 + 一条契约）：
- `contract/` 新增 `contract/plan/`（IR schema、blob 头、版本）与 `contract/policy/`（manifest schema）。
- `sim/` 编排 + 编译 + 运行 + 报告；`engine/` 那份 C++ 核（独立 CMake 工程，可单独交叉编译出 `.so` 与 wasm）。
- `client/` 复用现有「机器人」面板与 `client/viewer`（three.js + 自写 FK），不新开插件。

---

## 3. 范围

**做**：canonical→MJCF 导出（补齐为可用导出端）、preset 生成器 + C8 检查、图 IR + 编译校验、C++ 核 + 27 节点、ORT native 推理、blob 打包、`--report`、numpy 对拍、wasm 后端、浏览器可交互控制台（相机 / 键盘滑条 / 参数 / 策略切换）。

**不做（第一版）**：HIL / 板端 agent、发布托管与链接、AI 助手面板、多机器人场景、地形、motion reference、TS 双实现、端侧 aarch64 实跑（留到 v0.2，见 §8）。

**交互的边界**：做到「驾驶 + 调参 + 切模式」为止；**在线改图 / 在线改代码不做**——那是 botlab 编辑器那一层，要做另开一段。

---

## 4. 里程碑（9 周）

| 周 | 交付 | 通过判据 |
|---|---|---|
| W1 11-10~11-16 | canonical→MJCF 导出端 v0（body/joint/actuator/惯性/限位） | MuJoCo `mj_loadXML` 成功，0 warning |
| W2 11-17~11-23 | preset 生成器、C8（canonical 驱动序 ↔ MJCF actuator 序双射）、与 `third_party/fanhao-microduck-replica` 的结构对比脚本 | 结构对比差异逐条有解释；C8 红黄绿可跑 |
| W3 11-24~11-30 | **PD 跟踪 + 物理链路验证**（两跑法，见 §9 W3）+ `--report` 落地 | **止损点**（2026-11-10 改定）：原判据「PD 保持零位站 10 s」已实测不可达，站立并入 W5 有策略时验 |
| W4 12-01~12-07 | blob 格式定版；C++ 核骨架（arena/ABI/loader）+ 8 节点；numpy oracle 起步 | 8 节点与 numpy 逐点误差 ≤1e-6 |
| W5 12-08~12-14 | 27 节点全量 + ORT `create/prepare/run`；一份现成 ONNX 在 native MuJoCo 里跑 30 s | 不炸；action 与 numpy 参考一致；曲线能看 |
| W6 12-15~12-21 | Python DSL 编译器（声明→图）+ manifest 读取 + 图校验器（同源同一套 schema） | 一个真项目从 DSL 编到 blob 一条命令 |
| W7 12-22~12-28 | wasm 构建（`libonnxruntime_webassembly.a`）+ 浏览器查看器（复用 `client/viewer`，WS 推状态） | 浏览器里看到同一份图在动 |
| W8 12-29~01-04 | **交互通道**：键盘/滑条 → `commands`、参数滑杆热更新、策略/模式切换 | 能用键盘把鸭子开起来、拖参数能看曲线变化、能切策略 |
| W9 01-05~01-11 | `--report` 完整字段、`docs/node-parity.md` 收尾、demo | 一份 ONNX + 一张图：主机与浏览器行为一致，且浏览器里可驾驶 |

---

## 5. 契约与产物

**blob 头（草案，W4 定版）**

```
magic u32 'DBLP' | version u16 | abi u16
node_count u32 | edge_count u32 | const_count u32
requiredNodes: u16 n + n*(u16 type_id, u16 min_version)
digest u64（覆盖其后全部字节 + plan.json 的 rfc8785 摘要）
```

**preset 生成**：`tools/robot/export_mjcf.py` 加 `--preset <out.preset.yaml>`，只吐 botlab 实际读的那 13 个字段；`kp/kd/effortLimit` 从 canonical 的 `actuators` 读。

**report（最小字段）**

```json
{"incident_id":"","plan_digest":"","policy_manifest_digest":"","mjcf_digest":"",
 "steps":0,"control_hz":50,"resets":[{"step":0,"reason":""}],"metrics":{},
 "base_pose_samples":[{"step":0,"pos":[0,0,0],"quat":[1,0,0,0]}]}
```

**digest 链**：`plan → policy manifest → MJCF → preset`，四者在 report 里各一行，串起「这一版策略配哪份标定、哪份模型」。

---

## 6. 节点集（27 个）

- **观测 8**：`base_ang_vel` `projected_gravity` `commands` `joint_pos` `joint_vel` `last_action` `base_height` `imu`
- **算术 10**：`concat` `take` `reorder` `affine` `clip` `math` `norm` `low_pass` `moving_average` `noise`
- **时序 3**：`history`（layout 必填）`delay` `phase_clock`
- **编排 6**：`state_machine` `mode_select` `policy_input` `model_onnx` `out_action` `out_metric`

v0.1 补：`compare` / `logic`（`state_machine` 的 `when` 需要真值输入）。
它与 botlab 56 个节点的逐一对照见 [`docs/node-parity.md`](../node-parity.md)。

---

## 7. 风险

| 风险 | 实情 | 应对 |
|---|---|---|
| ORT wasm 静态库 | 要自己编 ORT（1–2 h）+ emsdk 版本对齐 + 三方头文件 | W7 单独留一周；崩了就切 D8 的回调后端，核不改 |
| C++ 审阅量 | 「AI 写 + 你审」，2000 行是硬上限 | 单文件、arena、零全局态；超过 2000 行就说明节点该拆出去 |
| canonical 表达力 | 现有 9 块 schema 是给「描述」用的，未必够「仿真可用」 | W3 止损点就是为它设的；缺字段走 ADR，不许私有字段 |
| MuJoCo 环境 | 现在装在 `/tmp/pylibs`（`PYTHONPATH=/tmp/pylibs python3`），/tmp 会被清 | 固化到 `~/.venv-ducksir` 或仓库 doc 写明重建命令 |
| 本机无 GPU | WebGPU 的 wasm 路径无法在本机验证 | 浏览器端先跑 wasm SIMD 单线程；WebGPU 留到有 GPU 的机器 |
| 许可 | 内部自用可，对外分发不可；接口形状同构 | 不发布衍生库；对外只谈「自研编排台」 |
| 浏览器交互旁路 | 交互最容易图省事直接改物理或直接写关节目标，那样仿真与端侧立刻分叉 | 交互只许改三样：`commands`、参数值、mode |

---

## 8. 未定（要庆喆拍板，别当结论）

1. **端侧 aarch64 何时进来**：D16 定了产物形状，但第一版不实跑板子。硬件路线（kissqy / fanhao375）与舵机（XL330 / HD1910）还没定，`types/duck-hd1910.yaml` 已存在说明倾向 HD1910，但没拍板。
2. **训练侧还没有管线**，第一版 manifest 只能手写或脚本生成；mjlab 真接进来时这份 schema 要不要改。
3. **在线改图/改代码**（botlab 编辑器那一层）要不要做，以及在交互版跑通之后再做还是不做。

---

## 9. 进度记录

### W1（2026-11-10）canonical → MJCF，三台机型都能加载

**交付**

- `tools/robot/export_mjcf.py`：加 `--mesh-dir` / `--mesh-ext`（MuJoCo 不认 glb，要能指到 STL）；修根 link 判断——`parent` 写成 `null` 和写成 `"world"` 都算根，原来只认后者，导致 G1 与天工**少一个 freejoint**。
- `tools/robot/glb2stl.py`（新，零依赖）：glb → 二进制 STL，带顶点聚类简化（STL 解码器上限 20 万面/文件）。
- `tools/robot/mjcf_smoke.py`（新）：三台机型导出 → 加载 → 抓 stderr 数警告 → 比总质量。
- `docs/env.md`（新）：依赖目录 + `PYTHONPATH`、mesh 来源与限制。

**验收**（`python3 tools/robot/mjcf_smoke.py`）

| 机型 | load | body | jnt | act | 质量 (kg) | canonical | warn |
|---|---|---|---|---|---|---|---|
| duck-hd1910 | OK | 16 | 15 | 14 | 0.73724318 | 0.73724318 | 0 |
| g1-29dof | OK | 40 | 30 | 0 | 33.34114202 | 33.34114202 | 0 |
| tienkung-lite | OK | 23 | 21 | 0 | 42.51599707 | 42.51599707 | 0 |

**三个发现**（都是 canonical 或 exporter 的真空洞，不是环境问题）

1. **根 link 的 `parent` 有两种写法**。鸭子写 `"world"`，G1 / 天工写 `null`；exporter 只认前者 → 后两台少了 freejoint（29/20 关节变 30/21）。已修，并提醒：`check_entity.py` 的 C1 只校验「parent 指得到」，没约束写法，这类不一致还能长出来。
2. **天工的 mesh 只有 glb 且是重模**。`waist_link` 66.8 万面，MuJoCo 的 STL 解码器上限 20 万，必须简化；已用顶点聚类降到 2.07 万面。
3. **G1 / 天工的 `actuators` 是空的** → `nu=0`。鸭子有 14 条（含 `kp/kv/forcerange/ctrlrange/armature` 等）。补它们要先把 `kp/kd/effortLimit` 落进 canonical（D13），属于 W2–W3，不是 W1 的活。

**遗留**

- MJCF 产物暂时只写到 `build/mjcf/`（gitignore）。它以后的家（`sim/` 还是 `contract/robot/export/`）等 W4 建 `sim/` 时一起定。
- 环境仍靠 `PYTHONPATH` 约定，没有锁文件。若以后上 CI，需要在 runner 上装 mujoco 才能跑冒烟。

### W2（2026-11-10）preset 生成器 + C8 双射 + 与上游 MJCF 结构对比

**交付**

- `tools/robot/export_preset.py`（新）：canonical → botlab 形状的 `*.preset.yaml`，13 个字段。`jointQposOffset` / `jointQvelOffset` **从导出的 MJCF 实测**（有 mujoco 时读 `jnt_qposadr`/`jnt_dofadr`），其余从 canonical 的 `actuators` 派生；没有上游依据的四个字段写进 `assumptions`。
- `tools/robot/compare_mjcf.py`（新）：两份 MJCF 的结构对比——规模、名字集合与顺序、同名关节的轴与限位、同名 body 的质量。
- `check_entity.py` 加 **C8**：`actuators` 与 actuated 关节必须双射（无重复、无缺、无多），并打印 ctrl 序 ↔ 驱动序的置换。
- `mjcf_smoke.py` 扩展：导出后顺便生成 preset，并核 `actionDimension == nu`、逐条 `actuatorJointOrder` 指向的关节、`jointQposOffset == jnt_qposadr`。状态分三档：`OK` / `BLOCKED`（canonical 没执行器参数）/ `DIFF`（真错）。

**验收**

```
机型             load   body  jnt  act        质量(kg)  preset warn
duck-hd1910    OK       16   15   14    0.73724318      OK    0
g1-29dof       OK       40   30    0   33.34114202 BLOCKED    0
tienkung-lite  OK       23   21    0   42.51599707 BLOCKED    0
3/3 通过
```

鸭子与上游 `third_party/fanhao-microduck-replica/robot_walk.xml` 对比（`compare_mjcf.py`）：

```
body 16 / joint 15 / qpos 21 / dof 20 / actuator 14，质量都是 0.73724318 kg
body 与 joint 名单一致、actuator 顺序一致、关节轴与限位全部一致、每 body 质量一致
结论：结构一致
```

**三个发现**

1. **我们的导出与上游 MJCF 结构完全一致**——说明前一段的导入（`importer_mjcf`）一个字段都没丢；这不是「看起来像」，是逐 body 质量、逐关节轴点积、逐限位比对出来的。
2. **preset 有 4 个字段没有上游依据**：`defaultJointPositions`、`initialQpos`、`actionScale`、`imuExtrinsic`。上游 MJCF 没有 `keyframe`/`home`，标定也没有。所以它们一律以 `assumptions` 显式写在 preset 里（零位 / 原点+单位四元数 / 全 1 / 单位变换），**宁可写清假设，不冒充规格**。
3. **G1 与天工的 `actuators` 为空**，preset 因此是 `BLOCKED`。而且它们的力矩上限只存在于 `third_party/unitree-g1/motor_info.md` 那张 5 行的**模式表**里（`shoulder.{roll,yaw}` 这种写法），映射到 29 个具体关节需要人判读，机器猜不起。

**遗留 / 要拍板**

- G1 与天工**要不要做成可驱动**？现在它们是「load-only」：几何与质量可信，控制参数空着。若要做，得先人判读 `motor_info.md` 的关节归类，再由训练侧给 `kp/kd`——不是写代码能解决的。

### W3（2026-11-10）PD 跟踪 + 物理链路验证

**判据改了**。原判据「PD 保持 `defaultJointPositions` 站 10 秒」在动工前被探针推翻：

| kp | 终点 z | 直立度 | 关节最大偏差 |
|---|---|---|---|
| 0.55（canonical 现值） | -0.009 m | -0.18 | 0.309 rad |
| 5 | -0.010 m | -0.15 | 0.041 rad |
| 20 | -0.011 m | -0.14 | 0.010 rad |
| 120 | -0.011 m | -0.14 | 0.002 rad |

跟踪误差随 kp 单调收敛（actuator 映射没问题），但**终点姿态几乎不动，鸭子照样侧躺**：全零位形不是站姿，腿伸直、质心在支撑面外，而 PD 管不了自由基座的平衡。上游 `config_mjcf_walk.json` / `1910_m6.json` 里也没有任何 default/stand pose 可抄。庆喆 2026-11-10 定：**改成「链路验证 + 物理验证」两跑法**，站立并到 W5「一份真 ONNX 跑 30 s」里验。

**交付**

- `sim/README.md` + `sim/host/pd_hold.py`：宿主仿真的第一块。**物理在宿主（D9）**，C++ 核以后只接 `obs → action` 这一段。
- `--report` 落地（D17 的最小字段）：`mjcf_digest` / `preset_digest` / `entity_digest` / `metrics` / `verdict`。

**验收**

```
链路验证  --gravity off --target step --amp 0.2 --seconds 5
  最大误差 0.1903 rad｜稳态 0.0031 rad（阈 0.005）
  PASS  finite=True track=True contact=True obs=True geom=True

物理验证  --gravity on --floor --target sine --amp 0.2 --seconds 10
  最大误差 0.6204 rad｜稳态 0.3638 rad（只记录，不设阈）
  接触比 0.98｜基座高度 终 -0.009 最低 -0.012 m
  PASS  finite=True track=True contact=True obs=True geom=True
```

**三个发现**

1. **导出端不生成场景**，MJCF 里没有地面 → 第一次跑鸭子直接自由落体到 z=-490。地面现在由宿主用 `MjSpec` 注入（`--floor`）；真正的场景文件还没定，留给 W4。
2. **设备增益偏软**：`kp=0.55`、`kv=0` 在重力下跟踪 0.2 rad 正弦，稳态误差 0.36 rad。链路是通的（关重力后稳态 0.003 rad），所以这不是 bug，是「这条策略要的增益得由训练侧给」——和 D13 的结论一致。
3. **`--report` 的 digest 链现在能闭合**：`entity → mjcf → preset` 三层都有 sha256，report 里各一行。这是 incident 链的起点。

4. **碰撞覆盖不全 —— 这才是「看着陷进地板」的真因**。canonical 的碰撞体是从上游 `robot_walk.xml`（walk 碰撞子集）导入的，
   鸭子 15 个 link 里有 **10 个没有碰撞体**（`jaw_soft`、`neck*`、`upper_leg*` 等）。鸭子被重力放倒、压到头部时，
   `jaw_soft` 的视觉网格比地面低 **181 mm**，而碰撞 geom 只到 -3 mm —— 也就是说物理上它躺在一个「没有头的碰撞壳」上，
   看起来就是头和下巴插进地里。实测换用上游 `robot_allcollisions.xml` 重导，无碰撞体的 link 从 10 降到 7（jaw、hip 有了），
   但 `neck*` 系列上游本来就没有。**这是资产限制，不是导出 bug**；选哪套碰撞集是建模决策（训练用 walk 快、摔倒测试要 all）。
   已加 **C9** 检查把这件事固定报出来。

   **已修（2026-11-10）**：新增 `contract/robot/variants/<id>.all.yaml`（碰撞变体，`robot.variant_of` 指向本体，
   导入自 `robot_allcollisions.xml`）+ 导出端 `--collision-set all`。实测：视觉最低点 **-181.5 mm → -12.5 mm**，
   物理末态基座高度从 -0.009 m 变成 +0.035 m（头/髋有了碰撞体，撑住了）。剩下的 -12.5 mm 是上游本来就没有碰撞体的
   `neck*` / `upper_leg*`，补它们要自己造碰撞壳（试过用视觉 mesh 凸包兜底，反而更差 -18.8 mm，已放弃）。

**遗留**

- **碰撞集选择**（W4 决策）：canonical 要不要同时带 walk / all 两套碰撞体，exports 用 `--collision` 选。现在只有 walk 那套。
- 观测语义还内联在 `pd_hold.py`（8 个槽里的 5 个 + `base_height`），W6 迁到 `contract/policy/manifest` 后才算有契约。
- G1 / 天工仍是 `BLOCKED`（没有执行器参数），W3 只在鸭子上验收。
- 站立/行走：等策略。

### W4（2026-11-10）blob 定版 + C++ 核骨架 + 八个节点 + oracle 对拍

**交付**

- `contract/plan/README.md`：**blob 格式定版**。48 字节头、32 字节节点记录、参数表、`required`（节点类型 + 最低版本）协商、digest（FNV-1a 64，非加密）。加载策略是**整份拒绝**，不是跑一半。
- `sim/plan/ir.py`（新）：Python 声明 → plan JSON（源码）→ blob（产物），编译期做同一套结构校验（拓扑序、唯一 sink、边界、perm 合法性）。
- `sim/plan/oracle.py`（新）：同一张图的 numpy 参考实现——**把 8 个节点的语义写成可执行规格**（D18）。
- `engine/`（新，本仓第一份 C++）：`include/ducksim.h`（C ABI）+ `src/core.cpp`（462 行）+ `Makefile`（没有 cmake，本机只有 g++ 与 make）。
- `sim/test/test_core_vs_oracle.py`：ctypes 加载 `.so`，同一份图跑 40 步逐点比。
- `sim/test/test_blob_roundtrip.py`：纯标准库的 blob 自检，进 CI（CI 不装 numpy）。
- CI 加一步「执行核编译 + blob 自检」，没有 g++ 就只跑自检。

**验收**

```
引擎 ABI 1
  simple   nodes=4   arena=24   step=40  最大偏差 0.000e+00  PASS
  rich     nodes=9   arena=144  step=40  最大偏差 0.000e+00  PASS
拒绝路径：digest 被改 / 缺 out_action / 未知节点类型 全部拒绝
```

**0.000e+00 不是「差不多」**：float32 逐位一致。靠三件事：arena 每步清零、节点按声明顺序执行、编译时 `-ffp-contract=off`。

**四个决定**（写进 `contract/plan/README.md`，以后改要三端重编）

1. **头和记录的布局按偏移写死**，不用 struct 格式串拼。踩过一次：Python 侧 `struct.pack` 的字段数与格式串差一个，编译期才发现；改成 `pack_into` 按偏移写，和 C 侧注释一一对应。
2. **`obs` 只有一个通用搬运节点**（`src_off, len`），不做 `joint_pos` / `projected_gravity` 这种语义节点——语义归 manifest（D12），核只管偏移。加一个观测不用改核。
3. **`model` 节点不推理**，只调 `ds_backend.onnx_run` 回调；`pass-through` flag 仅测试用（出产物前必须清）。
4. **`history` 首步是「前面全零」**，这条两端必须一致，否则策略看到的历史会差一截。

**遗留**

- ORT 绑定（`ds_backend.onnx_run` 接 onnxruntime）是 W5；现在是 pass-through。
- 逃生口 `native-only` flag 已在头文件里留位，编译期打标与拒绝在 W6 的 DSL 编译器里做。
- `sim/plan/` 与 `contract/plan/` 的 JSON schema 还没有机器可读版本（现在靠 `ir.validate` + 文档），要不要落 JSON Schema 待定。

### W5（2026-11-10）ORT 进核 + 一份真 ONNX 跑 30 s

**交付**

- `engine/src/ort_backend.cpp`（新，单独编成 `libducksim_ort.so`）：把 `ds_backend.onnx_run` 接到 onnxruntime 的 C API。核本身仍然零依赖——要推理才链这个库。
- `sim/plan/make_test_policy.py`（新）：造一份**合成**测试策略（34 维 obs → 14 维动作，只把关节往零位拉）。不是训练产物，作用是让整条链有一个真 ONNX 可跑。
- `sim/host/run_policy.py`（新）：策略闭环。obs 组装 → 图 → ONNX → action 后处理全在 C 核，物理在宿主，一次跑 30 s。
- `sim/test/test_ort_parity.py`（新）：端到端对拍——同一张图、同一份 ONNX、同一条 obs，C 核 vs numpy oracle + Python ORT。

**验收**

```
策略闭环 30.0s @ 50/200 Hz · 图 9 节点 · 模型 duck-hold.onnx
  帧数 1500/1500｜墙钟 0.17s（实时率 178.25×）
  基座高度 终 +0.035 最低 +0.035 m｜直立度 终 +0.368
  PASS  finite=True no_sink=True full=True

端到端对拍 20 步（含 ONNX 推理）：最大偏差 0.000e+00  PASS（阈 1e-06）
```

**注意这条 PASS 证明什么、不证明什么**：它证明**链是活的**（obs 组装 → 推理 → 动作映射 → PD → MuJoCo → 报告 → 回放），
不证明运控好不好——合成策略只会把关节往零位拉，鸭子照样会倒（直立度 0.368）。真策略只是换一个 `.onnx`。

**踩到的两个坑**

1. **ORT 的张量要按模型声明的形状建**。第一版摊成 `[34]`，ORT 不会立刻炸，核拿到 `DS_ERR_GRAPH`，到第二步才暴露。
2. **推理必须单线程**。多线程归约会让同一份输入算出不同末位，直接毁掉 1e-6 的对拍判据。

**环境**（写进 `docs/env.md`）

- ORT 的 wheel **不带 C 头文件**，SDK 要从 GitHub release 下 tgz 解到 `~/.local/share/ducksir/ort`；版本与 Python 包对齐 1.31.1。
- Python 侧的 `onnxruntime` / `onnx` / `ml_dtypes` / `protobuf` / `sympy` / `packaging` 都是从 PyPI 下 wheel 直接解开到 `PYTHONPATH` 那套目录（本机没有 pip）。
- ONNX 模型不入库（`.gitignore` 里的 `*.onnx`），但**回放轨迹入库**：`client/viewer/data/replay-duck-policy.json`（335 KB，1500 帧）。

**遗留**

- `policy_manifest_digest` 还是空串——manifest 是 W6 的事（D12）。
- 核里仍只有 8 个节点；计划的「27 节点」里剩下的是 `select` / `state_machine` / `phase_clock` / `delay` / `low_pass` 这类编排与时序节点，按需加（W6 做 DSL 时一起）。
- wasm 端要另走 ORT Web（W7）。

### W5b（2026-11-10）与 botlab 对齐模型 IO / 整个计算图 / 频率

庆喆看回放提的问题：*「机器人看着都没有挣扎，感觉推理不太对，能不能跟 botlab 完全对齐模型输入输出、整个计算图、以及推理频率」*。
先把「是不是推理错」用数字问清楚，再把缺口补上。

**诊断：不是推理错，是这份策略本来就「不出力」**

同一次 30 s 运行里的三个量：

| 量 | 实测 | 说明 |
|---|---|---|
| `\|action\|` 均值 | 0.016 rad | 策略几乎不说话（`action = -q`，而 `q` 一直贴着 0） |
| PD 误差 `\|ctrl - q\|` | 0.032 rad | 太小，推不动任何东西 |
| 实际力矩 | 0.018 Nm | forcerange 是 ±0.96 Nm，**只用了 2%** |

为了排除「action 根本没到关节」这一可能，用一份**常值目标 0.3 rad** 的策略跑 5 s：关节末态均值 **+0.294 rad**（目标 0.300）。
所以 `obs → 图 → ONNX → action → ctrl → 关节` 这一段是通的；鸭子「不挣扎」是因为策略输出接近零，不是因为链断了。

**补上的对齐**（这才是这次真正做的事）

1. **`contract/policy/` 落地**（D12）：manifest 定 obs 槽顺序与缩放、history 布局、动作映射与夹取、三个频率。
   图**从 manifest 长出来**，不再写死在脚本里；manifest 与模型/preset 对不上就红着退出。
2. **夹取范围改成 `ctrlrange`**（原来是我自己编的 ±0.5 rad）。botlab 夹的就是执行器 ctrlrange（鸭子是 ±10），
   这是之前一处实打实的不一致。
3. **`history` 支持**：manifest 里 `length > 0` 就自动挂 `history` 节点，`layout` 只接受 `by-frame`（botlab 要求显式声明，选错不报错）。
4. **频率显式化并核对**：`rates: {control_hz: 50, physics_hz: 200, inference_decimation: 1}`，
   实测 `inference_hz == 50.0`（1500 帧 / 30 s，推理 1500 次），报告里带 `rates_match` 判据。
5. **报告补 `policy_manifest_digest`**：digest 链现在是 `entity → mjcf → preset → manifest → model → plan` 六层。

**和 botlab 仍然没对齐的**

- 它的 `proc-policy-input` 是「按策略装箱观测」的节点，我们现在是 `concat` 直接拼（等价，但没有「多策略共用一套 obs」的装箱概念）。
- `state_machine` / `mode_select` / `phase_clock` / `delay` / `low_pass` 这些编排与时序节点还没做（W6）。
- 它的 `control_decimation` 是 policy 每 N 个控制步推一次；我们现在 `inference_decimation` 实现了同样的节流，但还没验证过 N>1 的行为。

**结论**：链是对的，缺的是「一份真策略」。合成策略换成训练产物之前，任何「挣扎/不挣扎」都只是那份策略的脾气。
