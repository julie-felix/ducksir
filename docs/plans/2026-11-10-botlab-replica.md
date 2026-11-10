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
| W3 11-24~11-30 | MuJoCo 里 PD 保持 `defaultJointPositions` 站 10 s；`--report` 骨架 | **止损点**：站不住就停下来修 canonical/exporter，不进入 C++ |
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
