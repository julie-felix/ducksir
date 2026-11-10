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

| 周 | 交付 | 通过判据 | 状态 |
|---|---|---|---|
| W1 11-10~11-16 | canonical→MJCF 导出端 v0（body/joint/actuator/惯性/限位） | MuJoCo `mj_loadXML` 成功，0 warning | ✅ 2026-11-10（三台机型 load 0 警告，质量对到 1e-6） |
| W2 11-17~11-23 | preset 生成器、C8（canonical 驱动序 ↔ MJCF actuator 序双射）、与 `third_party/fanhao-microduck-replica` 的结构对比脚本 | 结构对比差异逐条有解释；C8 红黄绿可跑 | ✅ 2026-11-10（与上游 MJCF 结构完全一致；C8 落地） |
| W3 11-24~11-30 | **PD 跟踪 + 物理链路验证**（两跑法，见 §9 W3）+ `--report` 落地 | **止损点**（2026-11-10 改定）：原判据「PD 保持零位站 10 s」已实测不可达，站立并入 W5 有策略时验 | ✅ 2026-11-10（链路稳态 0.0031 rad；物理接触比 0.98） |
| W4 12-01~12-07 | blob 格式定版；C++ 核骨架（arena/ABI/loader）+ 8 节点；numpy oracle 起步 | 8 节点与 numpy 逐点误差 ≤1e-6 | ✅ 2026-11-10（8 节点对拍 0.000e+00；blob 定版） |
| W5 12-08~12-14 | 27 节点全量 + ORT `create/prepare/run`；一份现成 ONNX 在 native MuJoCo 里跑 30 s | 不炸；action 与 numpy 参考一致；曲线能看 | ✅ 2026-11-10（真策略 61 维、站立 1.000；对拍 0.000e+00） |
| W6 12-15~12-21 | Python DSL 编译器（声明→图）+ manifest 读取 + 图校验器（同源同一套 schema） | 一个真项目从 DSL 编到 blob 一条命令 | ✅ 2026-11-10（15 节点；14 状态 / 7 策略状态机跑通） |
| W7 12-22~12-28 | wasm 构建（`libonnxruntime_webassembly.a`）+ 浏览器查看器（复用 `client/viewer`，WS 推状态） | 浏览器里看到同一份图在动 | ✅ 2026-11-10（wasm 逐位相同；浏览器物理+推理） |
| W8 12-29~01-04 | **交互通道**：键盘/滑条 → `commands`、策略/模式切换 | 能用键盘把鸭子开起来、能切策略 | ✅ 2026-11-10（浏览器里状态机自动切策略；参数热更新仍未做） |
| W9 01-05~01-11 | `--report` 完整字段、`docs/node-parity.md` 收尾、demo | 一份 ONNX + 一张图：主机与浏览器行为一致，且浏览器里可驾驶 | 🟡 部分（report 有最小字段；node-parity 已更新；demo 已可跑） |
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

### W5c（2026-11-10）把 botlab 的真策略接进来

庆喆问：「现在的策略是从哪来的，botlab 里边那个 onnx 不是可以用吗」。答：之前那份是**我生成的合成策略**（往零位拉），
仓里没有训练产物。这次把 botlab 的真策略接进来了。

**怎么拿到的**：botlab 产品站把模板按 `templates/microduck/<相对路径>` 直接暴露（实测 200），
所以 `walking.onnx` / `microduck.xml` / `microduck.preset.yaml` / `microduck.py` / `microduck_policy.py` 都能直接取。
上游是 <https://huggingface.co/spaces/pollen-robotics/microduck-simulator>，**无根 LICENSE** → 只做本地对照，模型不入库。

**对齐到了什么程度**（逐条对照它们的源码，不是猜）

| 项 | botlab | 我们 | 结论 |
|---|---|---|---|
| obs 维度与顺序 | 3+3+14+14+14+13 = **61** | 同 | ✅ 逐项一致 |
| 角速度来源 | IMU 陀螺仪 `imu_ang_vel`（site 系） | 导出端补了 `<site>` + `<sensor>`，读同一传感器 | ✅（之前我们导出的 MJCF 里一个 sensor 都没有） |
| 重力 | `projected_gravity(target="trunk_base")` | 同 | ✅ |
| 关节位置 | `joint_position(default=default_angles)` = q − preset.default | 同 | ✅ |
| 动作 | `ctrl = default + action_scale * action[motor_order]` | 图里 `affine` 做，clip 到 `ctrlrange` | ✅ |
| 频率 | control 50 / physics 200 / 每控制步推理 | 同，实测 inference_hz = 50.0 | ✅ |
| 模型 | `assets/models/*.onnx`，`obs[1,61] → actions[1,14]` | 同一份文件 | ✅ |
| 位形 | preset `defaultJointPositions` + `initialQpos`（= MJCF keyframe STAND） | 新增 `contract/robot/postures/microduck-stand.yaml` | ✅ |
| 模型 | pollen 的 MJCF | 我们的导出与它**结构完全一致**（compare_mjcf：body/joint/轴/限位/质量全对） | ✅ |

**结果**

```
策略闭环 20 s @ 50/200 Hz · 图 13 节点
  |action| 0.3229 rad｜实际力矩 0.0993 Nm
  基座高度 终 +0.124 最低 +0.112 m｜直立度 终 +0.996
  关节 std 0.0914 rad｜摆幅 0.399 rad   ← 真的在迈步，不是僵住
```

**没对上的**：指令尺度。同样这份 walking 策略，command=0.2 时几乎不动（8 s 位移 1 cm），0.5 时迈步明显
（8 s 2.08 m），0.6 反而走 6.7 m——链路是通的，但「UI 的 m/s」和「策略吃到的数值」之间差一个系数或标定，
而 botlab 的 `commands()` 代码里没有任何缩放，所以这一层要么在它的运行时里，要么在训练配置里。
**不猜**：先按 `--vx 0.5` 出片，把它记成待查项（下一步去 HF Space 的源码里找）。

### W6（2026-11-10）编排节点 + 多策略 policy_bank

botlab 的 MicroDuck 编排用的是 `state_machine + mode_select + policy_bank + phase_clock + where + constant`。
先把**节点**补齐，多策略跑通：

**新增 5 个节点**（C 核 + numpy oracle 双实现，对拍仍是 0.000e+00）

| id | 节点 | 用途 |
|---|---|---|
| 9 | `constant` | 常值向量（模式号、被动指令） |
| 10 | `select` | 按 mode 选一路（≤3 路，多了折叠）；不匹配输出全零或 passive 那一路 |
| 11 | `where` | `cond ? a : b`，两边都算不短路 |
| 12 | `phase_clock` | 步数 → [0,1) 相位（ground_pick 用） |
| 13 | `add` | 逐元素相加，分块 select 后消零用 |

**多策略 policy_bank 跑通**：`contract/policy/duck-multipolicy.manifest.yaml` 把 botlab 的 **7 份 ONNX**
挂在同一个后端上（`ds_ort_create_multi`，`model` 节点的 `model_id` 就是下标），同一份 61 维观测喂进去，
`select` 按 mode 选一路。对应它的 `policy_bank(mode, inputs={...})`。

**验收**

- 多策略 mode=0 与单模型 walking 的 action **逐步完全相同**（同一张图、同一份 obs，5 步差 0.00e+00）。
- mode=1 换成 sitstand，行为明显不同（直立度 0.884 vs 0.032），回放有两条片源。
- `--mode` 现在是常量节点；**动态切换要等 `state_machine`（下一步）**。

**踩到的一个坑**：ORT 多模型后端里把槽拷回基座 ctx，析构时同一个 session 被释放两次 → `free(): invalid pointer`。
改成持槽指针、不拷贝。凡是「共享 env/opts、独立 session」的结构，析构必须只释放一次。

### W6b（2026-11-10）状态机 + 14 状态 / 7 策略的动态编排

把 botlab 的 `microduck.py` / `microduck_policy.py` 那张图用本仓 DSL **逐条重写**：

- 新增 **`state_machine`（14）**：输出 `[state, steps]`，三类转移（command 脉冲 / after_steps / when），
  同类按声明顺序取首个命中，`reset_steps` 默认 True。状态跨步持久（和 history 一样在加载时分配）。
- 新增 **`compare`（15）**：`gt/gte/lt/lte/eq` → 0/1，给 `when` 当条件（tipped / upright / ~upright）。
- `reorder` 放宽成 gather（允许子集），这样「从 2 维里取 state」不再需要专门节点。

`sim/host/run_fsm.py` 里逐项照抄它们的常量与转移表：14 个状态号、22 条转移、`SIT_ENTER_STEPS=40` 等 8 个阈值、
状态→策略的映射（WALK/POST_KICK/WALK_RESET → walking，SIT_* → sitstand，ROLL* → roll，RECOVER* → stand …）。

**验收**

```
状态机图 36 节点｜1000/1000 帧｜z 终 +0.085 最低 +0.032｜直立 终 +0.909
状态轨迹：3.0s→1, 3.8s→2
```

第 3 秒的「坐下/站起」脉冲把状态从 0（WALK）推到 1（SIT_ENTER），40 步后再推到 2（SIT）——
**与它们的表一致**。第 8 秒发的「起身」脉冲没反应也是对的：它们的表里 `RECOVER` 只从 `WALK` 出发，
在 SIT 里按它不会被响应（这是它们的行为，不是我们的 bug）。

对拍：`fsm` 图（命令/计时/条件三类转移各走一遍）C 核 vs oracle **0.000e+00**。

### W7（2026-11-10）wasm 端：同一份核编给浏览器

**交付**

- `engine/Makefile` 加 `wasm` 目标（em++，`MODULARIZE` + `EXPORT_ES6` + `addFunction`），产物 `engine/build/wasm/{ducksim.mjs,ducksim.wasm}`（15 KB）。
- `engine/wasm/harness.mjs`：node 里加载 wasm、喂 plan blob 与 obs 序列、打印 action。自带 `--js-model` 用来验证推理回调。
- `sim/test/test_wasm_parity.py`：native（ctypes）vs wasm（node）对拍。
- `docs/env.md` 补 emsdk 安装与「下载要绕代理」的坑。

**验收**

```
native 30 步｜wasm 30 步｜节点 14｜10 → 26
最大偏差 0.000e+00   PASS 逐位相同
```

图里带 affine / clip / history / select / phase_clock / pass-through model——覆盖面足够说明「不是只跑得动最简单的图」。

**推理回调那条缝也验了**：`ds_backend` 用 `addFunction` 注入 JS 函数，接 `y = 2x + 1`，输出与期望逐位相同。
浏览器里把那个函数换成 ORT Web 的 `session.run` 就是完整链路——**这条是本项目最该有、botlab 做不到的保证**：
它两份引擎（TS/Python）靠 golden trace 对拍，而那份测试在开源包里默认 skip；我们是一份 C++ 走两个编译目标。

**还没做**

- 浏览器里的真推理（ORT Web 的 npm 包 + 页面接线）；现在是 node 里的等价验证。
- `viewer` 接 wasm：现在回放还是「宿主算好轨迹 → 浏览器放」，不是「浏览器里算」。

### W7b（2026-11-10）浏览器里真跑推理

**做的**：把 onnxruntime-web 的运行时 vendor 进 `client/viewer/vendor/ort/`（只取 wasm-only 那套：入口 + 胶水 + 14 MB .wasm），
新增 `client/viewer/live.{html,js}` 与 `live-core.mjs`：浏览器里加载 wasm 核 + ORT Web，喂宿主采的 61 维观测，
和宿主算出的 action 逐点比。素材由 `sim/host/export_browser_check.py` 生成。

**一个必须做的设计妥协**：ORT Web 的 `session.run` 是**异步**的，而 C 核是同步的。
所以在**推理边界**把图切成两段：`pre.blob`（原始 obs → 策略输入）、`post.blob`（原始 action → ctrl），
浏览器负责中间的 await。核本身一行没改。

**判据分两层**（别混成一句「一致」）：

| 层 | 判据 | 实测 |
|---|---|---|
| 图运算 native vs wasm | 逐位相同（0） | **0.000e+00** |
| 推理 native vs ORT Web | ≤1e-5（kernel 不同） | **1.192e-6** |

botlab 自己的 torch↔onnxruntime 奇偶校验阈值是 1e-4；我们这条更严。

**踩的坑**：`ort.min.mjs` 是含 WebGPU/WebNN 的默认构建，会去加载 `jsep` 那套胶水（29 MB）——缺文件直接报
「no available backend found」。要用 `ort.wasm.min.mjs` + `ort-wasm-simd-threaded.{mjs,wasm}` 那一套。

### W7c（2026-11-10）物理也搬进浏览器

**做的**：`client/viewer/play.html` + `play.js` + `play-core.mjs`——浏览器里同时跑 **MuJoCo wasm 物理 + wasm 执行核 + ORT Web 推理**，
three.js 只负责画。键盘 `W/S` 调前进指令，`R` 复位。这就是 botlab 那个「打开网页就能开鸭子」的原味体验。

素材三样，两条命令复现（都不入库）：

```bash
python3 tools/vendor_mujoco_web.py                    # @mujoco/mujoco 4.0.0 的 js+wasm（11 MB）+ 38 个 STL（21 MB）
PYTHONPATH=$HOME/.local/share/ducksir/pylibs python3 sim/host/export_browser_check.py   # plan blob + obs + model
```

**几个必须知道的实现细节**

1. **MuJoCo 只吃 STL/OBJ**，而 viewer 用的是 GLB。所以物理侧单独拷一份 STL 到 `client/viewer/assets/mjcf/`，
   MJCF 用 `--mesh-dir .` 导出，运行时按名字塞进 `MjVFS`。
   **`MjVFS.addBuffer` 不接受 `./name` 这种带前缀的键**（error 2），只能加裸文件名。
2. **`mjtObj` 的枚举是包装对象**，传给 `mj_name2id` 要取 `.value`，直接传会报 `Cannot convert "[object Object]" to int`。
3. **模型字段是普通 TypedArray**（`Float64Array` / `Int32Array`），`data.qpos[i]` 直接读写；别用 `.set(i, v)`
   （那是 TypedArray 的 `set(array, offset)`，会静默做错事）。
4. 渲染不再用 FK：每个 link 的 group 直接吃 MuJoCo 的 `xpos/xquat`，`robotGroup` 那层仍负责 Z-up → Y-up。

**验收（无头，`node sim/test/test_play_core.mjs`）**

```
模型：nbody=16 njnt=15 nq=21 nu=14 nsensor=5
观测：61 维｜前 6 项 0.000, 0.000, 0.000, 0.000, 0.000, -1.000
物理 200 步后基座高度 0.0349 m  PASS
```

浏览器里的最终画面需要你打开看：`python3 -m http.server 8123` → `/client/viewer/play.html`。
（页面里 `data/live/`、`vendor/mujoco/`、`assets/mjcf/` 都是上面两条命令的产物。）

### W7d（2026-11-10）浏览器页的三次修正

1. **`play.html` 缺 importmap**（根因）。`OrbitControls.js` / `GLTFLoader.js` 的第一行都是
   `import ... from 'three'`（裸标识符），`index.html` 里有 importmap、`play.html` 漏了 →
   **整个模块图解析失败，`play.js` 一行都没跑**，遮罩永远停在初始文案。补上 importmap、`play.js` 改用裸标识符。
2. **分步进度 + 每步超时**：卡住时能看到停在哪一步、超时变成明确错误（`stage()` / `step()`）。
   GLB 网格（70 个 geom / 22 MB）改成后台加载，不再挡住「就绪」。
3. **手机端**：`100dvh` 替代 `100vh`、`touch-action: none`、加「− 慢 / + 快 / 复位」三个触屏按钮
   （手机上没有 `W/S`）。同时给导出端加了 `--physics-only`：**视觉 geom 不导出**（浏览器里视觉走 GLB，
   STL 只喂物理），浏览器要下的 STL 从 **38 个 / 21 MB 降到 9 个 / 7.1 MB**——手机上是决定性的。

物理没变：`node sim/test/test_play_core.mjs` 仍是 16 body / 14 actuator / 5 sensor，200 步 PASS。

### W8（2026-11-10）浏览器交互：策略切换 + 前进指令

**做的**（都在 `client/viewer/play.html`，手机也能用）：

| 交互 | 宿主侧对应 | 实现 |
|---|---|---|
| `+ 快 / − 慢`、`W/S` | `commands` 的 forward_velocity | 直接改观测第 49 维 |
| 下拉 + 数字键 `1..7` | botlab 的 `policy_bank(mode)` | 7 份 ONNX 各自一个 ORT Web 会话，切换时清 `last_action` |
| `R` / 「复位」 | 工程 reset | 位形回 `initialQpos`、速度清零、指令归零 |

**验收**（`node sim/test/test_browser_policies.mjs`）

```
  walk         5 步 · 6 ms · 有限=true · 动作跨度 3.305
  sitstand     5 步 · 1 ms · 有限=true · 动作跨度 1.830
  kick_left    5 步 · 1 ms · 有限=true · 动作跨度 3.452
  kick_right   5 步 · 0 ms · 有限=true · 动作跨度 2.679
  roll         5 步 · 1 ms · 有限=true · 动作跨度 3.159
  ground_pick  5 步 · 1 ms · 有限=true · 动作跨度 1.985
  stand        5 步 · 0 ms · 有限=true · 动作跨度 2.630
7 个策略全部可加载并出有限动作 PASS
```

**还没做**：**自动状态机切换**（tipped/upright 条件、计时转移）仍只在宿主侧（`run_fsm.py`）。
浏览器里现在是「手动切策略」——和 botlab 的键盘行为一致（它也是按键发 `policy_command`），
差别在于它把状态机也跑在图里，我们这一步把 FSM 留在宿主。

---

## 10. 现状（2026-11-10 收尾盘点）

**能跑通的完整链**（两条，同一份 C++ 核）：

| 在哪跑 | 命令 / 入口 | 干什么 |
|---|---|---|
| 宿主 | `python3 sim/host/run_fsm.py --mjcf build/mjcf/duck-hd1910-all.xml --preset build/preset/duck-hd1910.preset.yaml` | 14 状态 / 7 策略状态机，按指令自切换，出报告与回放轨迹 |
| 宿主 | `python3 sim/host/run_policy.py --manifest contract/policy/duck-walk.manifest.yaml` | 单策略闭环（61 维 obs） |
| 浏览器 | 面板「机器人」→「浏览器跑」（`client/viewer/play.html`） | MuJoCo wasm 物理 + wasm 核 + ORT Web，`W/S` 调指令、`1..7` 切策略 |

**硬指标**（都不是「看着像」）

| 对拍 | 判据 | 实测 |
|---|---|---|
| C 核 vs numpy oracle（8/15 节点） | ≤1e-6 | **0.000e+00** |
| native vs wasm（图运算） | 逐位相同 | **0.000e+00** |
| native vs ORT Web（推理） | ≤1e-5 | **1.192e-6** |
| 导出 MJCF vs 上游 MJCF | 结构一致 | body/joint/轴/限位/质量全对 |
| 回放 FK vs MuJoCo | ≤0.1 mm | **0.000 mm** |

**两端对拍**（同一串指令脉冲，`node sim/test/test_fsm_parity.mjs`）

| 项 | 结果 |
|---|---|
| 状态迁移 | **逐帧相同**（3.00s→1，3.82s→2，8.00s→3，10.02s→10，10.06s→0） |
| 末态基座高度 | 宿主 0.121 m ／ 浏览器 0.117 m（差 4 mm） |
| 中间帧最大差 | 0.049 m —— 接触密集的步态里 1e-6 的推理差会被放大，属混沌，不作判据 |

**还没做的**（按价值排）

1. **`--report` 完整字段**：现在有 digest 链与 metrics/verdict，缺逐段耗时与 `policy_manifest_digest` 的校核。
2. **参数热更新**（改 `kp`/`actionScale`/clip 立刻生效）：现在要重编 blob 或重启页面。
3. **板端**：aarch64 `.so` 交叉编译与实跑（本机没有交叉工具链，得先装）。
4. **多策略清单单一真源**：`policies.json` 现在是脚本里手列的 7 个文件，应改为从 `duck-multipolicy.manifest.yaml` 派生。
5. **`docs/node-parity.md` 收尾**：56 行里还有 ❌23 条（多数是 botlab 侧独有、我们不需要的节点）。

### W8b（2026-11-10）状态机也搬进浏览器

**做法**：状态机不吃策略输出，只吃「指令脉冲 + 姿态 + vx」，所以它可以**整块放进核里**跑，
在推理边界之前一次性算完。新增 `sim/host/export_browser_fsm.py` 编出 `fsm.blob`：

```
输入 (3)   [policy_command 脉冲, 重力 z 分量, 前进指令 vx]
输出 (15)  [state, policy_mode, command(13)]
```

`command(13)` 直接照 botlab 的 `_command()`：行走用 `[vx,0…]`、坐/站用 `[1,0…]`、
地面拾取用 `[phase,0…]`（`phase_clock(steps, 200)`）。JS 每控制步：跑 `fsm.blob` → 拿 state/mode/command →
组 61 维 obs → `pre` → 推理（按 mode 选会话）→ `post` → ctrl。

**验收**（无渲染的端到端，`node sim/test/test_browser_fsm_run.mjs`）

```
状态轨迹： t=0.00s → 0，t=3.00s → 1，t=3.82s → 2
末态：state=2 策略=sitstand 基座 z=0.060 m
PASS：脉冲触发状态机切策略，动作进了物理
```

第 3 秒按一下「坐下/站起」→ 状态 1（SIT_ENTER，切到 sitstand 策略）→ 40 步后状态 2（SIT），基座从 0.117 掉到 0.060 m——
**它真的坐下了**。这与宿主侧 `run_fsm.py` 的轨迹一致。

**浏览器键位**（照 botlab `config/io.yaml`）：`W/S` 前进、`X` 坐/站、`Z` 左踢、`C` 右踢、`V` 翻滚、`B` 地面拾取、`R` 起身/复位。
下拉/数字键仍可手动指定策略，按任一策略键即回到状态机。

### W8c（2026-11-10）「一上去就打滚」的三层真因

现象：面板「浏览器跑」一打开，鸭子翻着滚。查下来是**三层叠加**，都不是渲染问题：

1. **`data/live/duck.json` 是手搓的**，preset 更新后没跟着变，仍是**全零位形**（`initialQpos` 连基座高度都是 0）——
   浏览器从「躺平的位形」起步，必然打滚。现在 `export_browser_check.py` 每次导出都重写这份元数据。
2. **preset 丢了站姿**：`mjcf_smoke.py` 重新生成 preset 时没带 `--default-pose`，
   `defaultJointPositions` 退回全零、`initialQpos` 的基座也变 0。位形文件还顺手改名成
   `contract/robot/postures/<robot-id>.yaml`（原来叫 `microduck-stand.yaml`，smoke 按 id 找不到它）。
3. **MuJoCo 版本没对齐**：浏览器 vendor 的是 `@mujoco/mujoco` 4.0.0，宿主 Python 是 3.17.0。
   同一串 ctrl 重放，200 步后基座差 **85 mm**（接触密集的步态直接失稳）。
   vendor 脚本固定成 **3.17.0** 后，同条件单步 qpos 差 **0.00e+00**、200 步差 **0.00000 m**——**逐位相同**。

**回归守卫**：`node sim/test/test_play_stance.mjs` —— 用浏览器那套（物理 + fsm + pre/post + ORT Web）跑 5 s walk，
初始 z 必须 > 0.1、末态直立度 > 0.9。它同时钉住「初始位形是否过期」和「策略能不能撑住」。

**顺手改的**：浏览器主循环从「每帧推 4 步物理、到点才推理」改成**严格 4:1**（一个控制步 = 1 次推理 + 恰好 4 步物理），
因为帧率一掉，原来那套会让每个控制步覆盖 8/12 步物理，策略看到的是被拉快的动力学。

### W8d（2026-11-10）「坐下成功、起身就滚」

真因是**指令映射抄错了一半**。botlab 的 `_command()` 里：

```
WALK / POST_KICK / WALK_RESET → velocity_command
SIT                            → sit_command = [1, 0…]
GROUND_PICK                    → [phase, 0…]
其余一切（含 SIT_ENTER / SIT_EXIT / 踢 / 翻滚 / 起身） → 零指令
```

我原来把 `SIT_ENTER` 和 `SIT_EXIT` 也给了 `[1,0…]`——于是「起身」阶段策略仍以为在坐，站不起来，
2 秒后状态机照表切到 `WALK_RESET → WALK`，行走策略接到一个还蹲着的姿态，直接翻。

顺带补了另一条抄漏的：`clear_last_action`（`(state==SIT_ENTER & steps==0) | WALK_RESET | RECOVER_ENTER` 时把
观测里的 `last_action` 清零）。漏了它，切策略的第一帧会把上一个策略的动作喂给新策略。

两处都改后：

```
状态轨迹：t=3.00s→1，t=3.82s→2，t=8.00s→3，t=10.02s→10，t=10.06s→0
坐下时 z=0.060 m → 末态 z=0.124 m，直立度=0.999（策略 walk）  PASS
```

`fsm.blob` 因此从 15 维输出变成 16 维（多了 `clear_last_action` 这一位）。

**回归守卫**：`node sim/test/test_play_sitstand.mjs`（坐 → 起身，要求末态直立度 > 0.9）。

**已知不一致（记账）**：宿主 `run_fsm.py` 的观测里，13 维指令**始终**是 `[vx,0…]`，
没做上面那套按状态切换；它对坐/起身这条用例恰好也能过（末态直立 0.999），但严格说和 botlab 不同口径。

### W8e（2026-11-10）「行走这么歪」= 指令超出训练范围

先照着上游把循环原样重写一遍（`app/src/game/game.js` 的 `buildObs` + `ctrl = DEFAULT_POSE + act`），
拿同一份 `walking.onnx` 跑：

| cmd[0] | 上游算法 | 我们的图 |
|---|---|---|
| 0.0 | x +0.00，yaw +0.0° | x +0.00，yaw +0.0° |
| 0.25 | x **+1.01**，yaw −7.2° | x **+1.01**，yaw +5.1° |
| 0.5 | x +2.59，yaw **+51.6°** | x +2.64，yaw **+43.6°** |

两条路一致 → **我们的管线没错**。问题在指令范围：上游 `constants.js` 定死

```
VEL_FWD = 0.25   VEL_BACK = -0.2   VEL_ANG = 1.0
```

而浏览器页的 `+ 快` 一路加到 **1.0**，策略在训练分布外就会「一边走一边拐」——这就是「行走这么歪」。
现在把 vx 夹到 `[-0.2, 0.25]`、步进 0.05，并补上转向（`A/D`、`←/→` 两个触屏键，范围 ±1.0）。

**两个查出来的细节**

1. **策略的 +yaw 是向右**：`wz=+1.0` → yaw −151°，`wz=−1.0` → yaw +104°。所以「左转」要发负值。
2. 早前几次扫参得出「0.25 以下不动、0.5 会拐」的怪结论，是**我自己的测试脚本喂错了 `last_action`**
   （把 `ctrl` 当成了 policy 的原始 action）。用正确口径重跑后，0.25 就是 1.00 m／10 s 的直线行走。

**回归守卫**：`node sim/test/test_play_walkcmd.mjs` —— 范围内走直线（|yaw|<15°、|y|<0.4 m）、转向 ±20° 以上且符号正确。

### W8f（2026-11-10）手机端与响应式

「浏览器跑」这一页原来在手机上是勉强能用：状态卡压着画面、按钮一行塞五个、`100vh` 被地址栏吃掉一截、
渲染像素比固定 2（中端机掉到 20 fps）。重做了布局与渲染参数：

**布局**

- `viewport-fit=cover` + `env(safe-area-inset-*)`：刘海、圆角、底部手势条都不挡内容。
- 顶部状态卡：`backdrop-filter` 毛玻璃、`max-width: min(78vw, 340px)`，窄屏自动收到 `100vw - 20px`。
- 底部改成**固定操作坞**（`#dock`）：两行触控按钮（转向 / 加减速 / 坐站 / 踢 / 翻滚 / 复位），
  每个 **≥ 46 px** 高，`touch-action: manipulation` 去掉双击缩放延迟。
- 触屏设备（`hover: none and pointer: coarse`）**隐藏键盘提示**，把空间让给按钮。
- 横屏矮屏（`max-height: 460px`）把操作坞转成横向一条、隐藏标题，画面占比最大化。
- 原来只有键盘的 `X/Z/V` 现在都有按钮：坐站、踢腿、翻滚 —— 手机上不再需要外接键盘。

**渲染**

- 像素比自适应：触屏 `min(dpr, 1.5)`、桌面 `min(dpr, 2)`；抗锯齿在手机上关掉。
- **自适应降载**：连续 2 s 低于 26 fps 就把像素比降 0.25，最多降两档（只降不升，避免抖动）。
- 尺寸跟随 `clientWidth/Height` + `ResizeObserver` + `orientationchange` + `visualViewport.resize`
  （iOS 地址栏收放不会触发 `resize`）。

**静态守卫**：`node sim/test/test_play_page_ids.mjs` —— 检查 `play.js` 里 `$('id')` 引用的每个 id 都在 `play.html` 里存在
（这次改版最容易漏的就是按钮换 id，浏览器只会在运行时静默失败）。实测 17 个 id / 12 处引用全部命中。

**面板壳**（`client/lib/client.js`）：窄屏（≤640px）下工具栏换行、隐藏说明文字、按钮加高到 40 px。

### W9（2026-11-10）宿主 ↔ 浏览器全链路对拍

物理对拍（重放同一串 ctrl，差 0.00000 m）只证明「同一个控制序列下两端物理一致」；
这一轮把**整条链**也对上了：状态机、按状态给的指令、`clear_last_action`、策略推理、控制映射。

先修了宿主侧两处口径（此前和 botlab 不一致）：

1. 观测里 13 维指令**恒为** `[vx,0…]` → 改成按状态给（只有 SIT 给 `[1,0…]`，GROUND_PICK 给相位，其余零）。
2. `last_action` 之前填的是 **affine 之后**的 ctrl → 改成填**策略原始输出**
   （图输出因此多带一段 `原始 action`，维度 `n*2+3`）。

改完两端同一串脉冲（3 s 坐下、8 s 起身、18 s）：

```
宿主状态迁移：  3.00s→1，3.82s→2，8.00s→3，10.02s→10，10.06s→0
浏览器状态迁移：3.00s→1，3.82s→2，8.00s→3，10.02s→10，10.06s→0   ← 逐帧相同
末态基座高度：  宿主 0.121 m ／ 浏览器 0.117 m
```

**判据分两层**：状态迁移逐帧相同（控制逻辑等价的硬指标）+ 末态高度差 < 3 cm。
中间帧差到 0.049 m 是混沌放大——接触密集的步态对 1e-6 的推理差异敏感到这个程度，不当 bug 处理。

### W8g（2026-11-10）契约成为产物

**问题**：图的形状（观测偏移、`fsm` 输出的 `[state, mode, command(13), clear]`）写在**消费端**代码里
（`play-core.mjs` 的 `obs[0..2]`、`play.js` 的 `fsmOut[0]`/`slice(2,15)`/`fsmOut[15]`，以及手列的 `policies.json`）。
改 Python 生成新 IR 之后，端侧用旧偏移读新 blob——**不报错，只是行为诡异**。当天的指令映射漂移就是这么来的。

**做法**：编译时把形状一并产出，端侧变成通用执行器。

- 新增 `sim/plan/contract.py`：`plan_entry()` 记形状 + digest + 具名槽位，`build_contract()` 组装并**自检**
  （槽位不越界、obs 总长 == `pre.input_dim`、策略字段齐全）。
- 新增 `sim/host/export_artifacts.py`：一个入口产出 `pre/post/fsm.blob` + 7 个模型 + `contract.json`。
- `client/viewer/play-core.mjs`：不再有任何偏移数字——`setContract()` 校验契约，
  `makeObs()` 按**观测配方**取数（`sensor:` / `gravity:body:` / `qpos:joints` / `qvel:joints` / `last_action` / `fsm_slot:`），
  `checkPlanAgainstContract()` 在加载后比对维度。
- `client/viewer/play.js`：plan 文件、维度、槽位、策略清单、频率全部来自契约；载入后立刻校验。
- `export_browser_check.py` 不再写 `duck.json` / `policies.json`（契约取代），只留宿主参照素材。

**验收**：新增 `node sim/test/test_contract_guard.mjs` —— 维度不符 / 槽位越界 / 缺字段 / kind 不对 / 未知 plan /
未知槽位，六种篡改都必须**报错**。实测全过。其余 8 个浏览器与对拍测试在重构后全绿（含全链路对拍：状态迁移逐帧相同）。

**收益**：加策略、改 `fsm` 输出维度、换观测布局——只改 Python/YAML 后重新导出，端侧零改动。

### W8h（2026-11-10）宿主接入契约 + 两处漏改

1. **宿主接入契约**：`sim/host/contract_obs.py` 提供和浏览器同一套 `makeObs()`（按配方取数），
   `run_fsm.py` 改成读 `contract.json` 组观测（`--contract` 参数，缺了直接报错而不是退回旧口径）。
   观测配方不再手写——`export_artifacts.py` 从 **manifest 的 obs 段派生**，manifest 才是声明源。
2. **两处漏改（都是重构后端的）**：`play.js` 里还留着 `new Float32Array(N_JOINTS)`（`play-core` 已不再导出它）→
   页面直接抛 `ReferenceError: N_JOINTS is not defined`。改成载入契约后按 `contract.robot.joints.length` 建数组。
3. **静态守卫加一条**：`test_play_page_ids.mjs` 现在还检查「`play.js` 从 `play-core` 导入的每个名字都真有导出」——
   这类漏改不靠跑起来才发现。11 个导入名全过。

### W9b（2026-11-10）统一 CLI 三件套

照 botlab 的 `run/build/validate` 收了一条统一入口 `bin/ducksir`（薄包装 `tools/cli/ducksir.py`）：

| 子命令 | 干什么 | 为什么需要 |
|---|---|---|
| `validate` | 实体 C1–C9 + manifest 校验 + 图编译 + MJCF load，四步一次跑完 | 以前散在三个脚本，且只有编译期才发现 |
| `build` | 产出 `build/release/<时间戳>/`：`duck.xml` + `preset.yaml` + 3 个 blob + 7 个模型 + `contract.json` + **`release.json`**（每个文件的 sha256） | 「从声明产出可分发的包」这条链以前是断的 |
| `run` | 单策略闭环 / 14 状态机（透传到老的 `run_policy.py` / `run_fsm.py`） | 免去记一堆参数 |

**验收**：`bash sim/test/test_cli.sh` 四条全过（validate / build / run --policy / run --fsm），产物 13 个文件带摘要。

**还没做**（比 botlab 少的）：`deploy` / `agent` / `target` / `control`（部署子系统整体推迟，见 `docs/botlab-diff.md` §5），
`ui`（我们有面板，不需要它的 UI 快照管理）。

### W9c（2026-11-10）golden 快照

照 botlab 的 `_runtime/golden_check.py` 补了「先记录后验证」这一层。

**分工**：`test_core_vs_oracle.py` 证明「**算得对**」（对着 numpy 参考实现重算期望值）；
`test_golden.py` 证明「**没变**」（对着上一次的 C 核输出逐位比）。两条都要过。

**做法**

- `sim/plan/golden.py`：`record()` 跑 C 核把输出存成快照；`check()` 重放并逐值比，
  报「第几帧第几个分量差多少」；`cases_from_release()` 从 release 目录构造确定性用例
  （普通 plan 用固定种子随机帧，fsm 用结构化场景播种：站立/按坐站/前进/转向/倾倒/接近正立）。
- 快照存 `contract/plan/fixtures/release-golden.json`（22.6 KB，入库）。值用 Python float 字面量——
  float32 转 float 再序列化**精确往返**，所以既可读（能 diff）又不丢精度。
- 入口两处：`bin/ducksir golden [--update]` 与 `python3 sim/test/test_golden.py [--update-golden]`。

**验收**

```
PASS  3 个 plan 与快照逐位相同：pre(6 帧), post(6 帧), fsm(36 帧)
```

**证明它真的会报警**：手动把快照里 `post` 第 2 帧第 3 个分量加 1e-6，再跑：

```
FAIL  post：第 2 帧第 3 个分量差 1.000e-06（现 0.947509766 / 存 0.947510766）
```

**规矩**：改图/改核之后若**有意**改变行为，跑 `--update` 重新记录，并在提交说明里写清为什么变；
不确定的时候不要更新快照——那是把 bug 固化成基准。

### W9d（2026-11-10）面板指标 schema + 实验台页 + 薄 Python 门面

**1. 指标 schema**（`sim/host/panel.py`）：固定 8 个键——`control_hz` / `physics_hz` /
`realtime_factor` / `control_p95_ms` / `deadline_miss` / `base_height_min` / `upright_final` / `distance`。
`run_policy.py` 在每个控制步掐表，`check_panel()` 校验「键齐、有限、不是 bool」。
`test_panel.py` 还校验**页面标签表与 schema 的键完全一致**——两边各写一份是迟早会漂的。

**2. 实验台页**（`client/viewer/inspect.html`）：读一次运行的 `report.json`，摊开四块——
面板指标卡、判据表（pass/fail 上色）、摘要链（plan / manifest / model 的 sha256 前 16 位）、
以及**能真跑的 Python 片段**（不是伪代码，是 `sim/sdk` 里那层门面的用法）。
面板工具栏加了「实验台」页签。

**3. 薄 Python 门面**（`sim/sdk/ducksir.py`）：

```python
from ducksir import Duck
d = Duck()
d.joints(); d.default_pose(); d.contract_doc()
r = d.run(seconds=10, vx=0.25)     # 返回报告 dict，含 panel 八项
```

**只做薄的一层**（不支持 `Simulation` 的 scene/motion/sensor 装配）：厚门面要等多机器人 + 场景 + 重定向都落地才有意义。

**验收**

```
python3 sim/test/test_panel.py              全部通过（schema / 缺键 / NaN / 页面标签对齐）
node  sim/test/test_play_page_ids.mjs       导入 11 名、planPath 3 个、inspect 6 处引用 全命中
Duck().run(seconds=2) → panel {rtf 165.891, p95 0.13 ms, deadline_miss 0, upright 0.9999}
```

**还没做**：参数可改（现在只读展示）、指标历史/曲线（现在是一次运行的一张快照）。

### W10（2026-11-11）「写代码 → 启动 → 跑仿真」

照 botlab 那条链做出来，但按我们自己定的取舍：

| 决定 | 选择 | 理由 |
|---|---|---|
| 入口 | `ducksir run project.py`，循环归 CLI | 循环里一堆不该让用户写对的东西（频率比、推理节流、读契约、报告、中断） |
| 脚本写什么 | **只声明**（链式 `Project`），不写 run | 「跑多久/开不开浏览器」应该由命令参数决定，改它们不用改代码 |
| Python 每步逃生口 | **不做**；表达不了就扩图 | 跨端一致性优先；加节点有三条准入标准（写进 `contract/plan/README.md`） |
| 显示 | 起本机显示服务 + 自动开浏览器实时跟随 | 照 botlab `--native` 的形状；但用 **SSE** 而不是 WebSocket（无依赖、标准库够用） |

**新增**

- `sim/plan/graph.py`：高层构图 API。`Graph.standard(...)` 与 `run_policy.build_plan()` **产出逐字节相同的 blob**
  （`test_graph_equiv.py` 直接比字节）。
- `sim/sdk/ducksir.py` 里的 `Project`：链式声明 `policy/command/rates/seconds`，`describe()` 给 runner 用；
  频率不成整数倍、没登记策略都会**早报错**。
- `sim/host/run_project.py`：加载脚本 → 建物理 → 编译图 → ORT 会话 → 循环（`--seconds`/`--steps` 双上限）→
  报告；Ctrl-C **先落盘**、报告记 `interrupted: true`、退出码 130；`--pace auto`（开显示时按实时，否则全速）。
- `sim/host/display.py`：本机显示服务，`/scene`（body 名序）+ `/stream`（SSE，`{t, pos, quat}`）+ `/static/*`；
  只绑 127.0.0.1，同时供 `follow.html`。
- `client/viewer/follow.html`：实时跟随页，只做「连流 → 按帧设每个 link 的位姿 → HUD」。
- CLI：`run` 给项目脚本，老的 `--policy/--fsm` 改名 `ducksir sim`（实现一行未删）。

**验收**

```
python3 sim/test/test_graph_equiv.py      builder 与 ir.Plan 的 blob 完全相同（764 B，13 节点）
python3 sim/test/test_project_run.py      正常跑 / Ctrl-C 落盘退出码 130 / 写错早报错
python3 sim/test/test_follow_stream.py    /follow 可访问、SSE 12 帧、pos%3==0、quat%4==0、时间戳单调
bash sim/test/test_cli.sh                 validate / build / sim / run 全过
```

**还没做**：多策略在项目里切换（要接状态机）、参数热更新、板端。显示用 SSE 而非 botlab 的 WebSocket——
形状对齐、实现更薄，但**不是二进制协议**，将来要推高帧率/大模型时可能要换。
