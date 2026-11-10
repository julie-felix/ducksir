# 阶段 2「数字描述」计划

> 状态：草案，待拍板项见 §7。依据：2026-11-08 录音（16:38）+ 当日对 `fanhao375/microduck-replica` 的实查。
> 对齐 [docs/flow.md](../flow.md) 第 2、3 步，不改流水线顺序。

## 0. 三句话

1. fanhao 仓库里**没有 URDF**。这只鸭子的几何与运动学真相是 6 个 MJCF，由 `onshape-to-robot` 从 Onshape 文档自动生成；网页里那台 3D 鸭子是再派生出来的一份 `model.json`。
2. 所以「找 URDF」真正要定的是**鸭子用哪个格式当输入**。录音里的结论（统一格式由我们定）指向第三条路：canonical 自己定义，URDF 和 MJCF 都只当**导入端**。
3. 本阶段对外只有一个产物：`robot.dt.yaml`（`base` + `overlay` → `resolved`）加一份结构检查报告。训练要的 MJCF 是它的**导出物**，不是它本身。

## 1. 事实：鸭子现在有哪些描述文件

| 资产 | 路径 | 格式 | 有什么 | 能不能直接训练 |
|---|---|---|---|---|
| 训练用 MJCF（6 个变体） | `software/training/src/mjlab_microduck/robot/microduck/robot_{walk,allcollisions}[_rollers][_backlash].xml` | MuJoCo XML | 14 个驱动关节 + 被动机 + 15 个刚体（含惯量张量）；6 个变体差在碰撞几何、滚轮、回差 | **能**，mjlab 现在就在吃 |
| 生成配置 | 同目录 `config_mjcf_walk.json` 等 | onshape-to-robot 配置 | Onshape 文档 URL、STL 简化上限、碰撞体白名单、三个附加 xml、7 条 sed 后处理 | 是唯一能复现 MJCF 的配方 |
| 参数补充 | `joints_properties.xml`、`additional.xml`、`sensors.xml`、`joints_properties.xml` | MuJoCo default class | 执行器 class（`chosen_actuator`：damping 0.053 / frictionloss 0.0048 / armature 0.0018 / kp 0.55 / forcerange 0.96 N·m）、自碰撞、site | 已含在 MJCF 里 |
| 网页 3D 鸭子 | `software/studio/web/model/model.json` + `meshes.bin(.gz)` | 自定义 JSON（bodies / meshes / joint_ids） | 16 个 body、28 个 mesh、15 个关节（多一个嘴）、**舵机 ID 映射** | 只用于显示 |
| 训练前数据清单 | `docs/HD-1910训练前数据清单.md` | Markdown 表 | 几何、质量、惯量、14 关节零位/限位表、舵机规格、IMU 安装 —— 已按「出处」逐格标注 | 人读，机器不可读 |
| 舵机辨识参数 | `robot/hd1910/1910_m6.json`（LuwuDynamics 1910 BAM M6） | BAM 参数 JSON | 电阻、Kt、摩擦、延迟等 6 级模型参数 | 训练工程锁的是它 |
| 上游同源 | `pollen-robotics/microduck_rl`（develop） | 也只有 MJCF | 与 fanhao 同族，无 URDF | — |
| CAD | `fanhao375/microduck-replica-cad` | SolidWorks 截图/PDF/3mf | 装配图、打印盘、BOM；真 CAD 在 Onshape | 不含描述文件 |

**两个已经暴露的口径差（正是要建 canonical 的理由）：**

- 整机质量：MJCF 15 个刚体求和 **737.2 g**；训练前数据清单写 **737.1 g**；网页 `model.json` 的注记写 **744 g**。同一个模型三个数。
- 网页 `model.json` 的嘴（ID 34）是**补出来的**铰链，位置靠估；策略里没有这个关节。谁读谁都得知道它不算数。

另外：这套几何、质量、惯量都是 **XL330 版仿真值**，实机是飞特 HD-1910（单只 18 g → 21 g，外形薄 3 mm）。所以这一版孪生只能叫「对齐 CAD」，**不能叫「对齐这台机器」**。

## 2. 缺口：录音里 felix 要的四样，现在各在哪

| 要的东西 | 现状 | 位置 |
|---|---|---|
| 拓扑 / 轴 / 限位 / 质量 / 惯量 / mesh | 有 | MJCF |
| 并联/闭环关节解算 | MJCF 里是 `equality` 约束 + 被动铰链（嘴的闭环连杆就是这样），URDF 表达不了 | `additional.xml` |
| 转子惯量 | 只有 `armature`（0.0018，齿轮反射后等效值），不是电机转子惯量 | `joints_properties.xml` |
| 减速比 | **不在模型里**，只有规格书 320:1 | 规格书 / 数据清单 |
| 峰值/额定转速、堵转力矩、Kt、R | 规格书有，模型里只有 `forcerange 0.96 N·m` | 规格书 / BAM JSON |
| 控制延迟、摩擦辨识值 | BAM M6 JSON 里有，训练配置另有 `delay 3–6 步` | `1910_m6.json` |
| 编码器零位、正方向、机械限位 | **全部待实测** | 数据清单第 3 节 |
| IMU 安装位姿、足底摩擦 | IMU 有 mount 四元数；足底 μ 待实测 | 数据清单第 5 节 |
| 回差 | 有 `_backlash` 变体（每个舵机关节加一根被动铰链），数值是拍的 | `robot_walk_backlash.xml` |

结论：**MJCF 覆盖了「运动学 + 惯量」，覆盖不了「执行器 + 标定」**。后者现在是散在 Markdown 表、规格书 PDF、BAM JSON 三处的。这正是录音里 julie 说的「统一格式要装下这些东西」，也是 felix 说的「URDF + 并联 + 电机参数」——两者不冲突，差的是**谁来当真相**。

## 3. 决策：canonical 是我们自己的 schema，不是 URDF

`GLOSSARY.md` 现在写的是「canonical 是 URDF/xacro，USD、MJCF 由它导出」。**这句与录音结论相反，建议改**（见 §7-1）。

理由：

1. URDF 装不下执行器与标定增量（§2 那张表），装不下并联约束；把 URDF 当真相 = 逼所有人往 XML 里塞私货 `<!-- -->`，或者维护一份永远不同步的旁注。
2. 我们真正要喂的仿真器是 **MuJoCo/mjlab**，它吃 MJCF。让 URDF 当真相等于每次训练都做一次有损转换。
3. 标定/辨识必须能**只写增量**（flow.md 已定：base 只读，只写 overlay）。URDF 没有 overlay 的概念。
4. 格式选型要看的是「谁能装下全部字段 + 谁能被 diff + 谁能被脚本校验」，不是社区用得多不多。

| 候选 | 装得下全部字段 | 人可读/diff | 生态 | 结论 |
|---|---|---|---|---|
| URDF (+xacro) | 否（无执行器/无并联/无 overlay） | 中 | 最广 | **只当导入端**，可选导出 |
| MJCF | 中（有 actuator、equality、可选 site/camera） | 是 | MuJoCo 第一 | **必做导出端**；也当导入端（鸭子就用它进） |
| USD | 是（可挂任意 attribute） | 差（二进制层） | Isaac/Omniverse | 后置，第二导出端 |
| 自定 YAML + JSON Schema | 是 | 最好（一行一值一出处） | 无，但有 Python/pydantic 就够 | **canonical** |

对 felix 那条路径的处理：`URDF + 并联解算 + 电机参数 → 训练` 依然是有效流程，它对应到我们这里就是 **URDF importer → overlay 里的并联解算结果 → exporter 出 MJCF → 训练**。步骤没变，只是多了一层可校验的中间格式。

## 4. schema v0：什么是一只机器人

### 4.0 先把三个词分开（canonical / 类 / schema）

录音里 julie 说「定义这个类，什么是一个机器人」，说的是**内容**；canonical 说的是**地位**。分开看：

| 词 | 回答的问题 | 形态 | 谁用 |
|---|---|---|---|
| **类**（class / 数据模型） | 「一台机器人有哪些字段、什么类型、谁嵌谁」 | Python `dataclass` / Rust struct / TS interface | 人读、代码 import |
| **schema** | 同一个定义的**语言中立、可校验**版本 | JSON Schema / protobuf / XSD | 机器：校验、代码生成、前后端对齐 |
| **canonical** | 「在 URDF / MJCF / USD / YAML 之间，哪一份算数」 | 无形态，是一条**制度**:唯一、权威、其余皆导出 | 流程与人 |
| **instance** | 具体某台机器人的数据 | `robot.base.yaml` | 训练/仿真/客户端 |

关系是包含不是并列：`canonical 角色` → 由 `schema` 定义结构 → 各语言生成 `类` → 存出 `instance`。

**为什么不能只写类**：三平面跨语言（training 是 Python、client 是 TS、runtime 是 C++/Rust）。类只能在自己的语言里当真相，跨语言必须降到 schema，再由 schema 生成三份类。pydantic 正好把两者合一（写 `BaseModel` 既得类也得 JSON Schema），所以 Python 侧首选它，但真相文件仍是 schema。

**三层校验，别混成一层**：

| 层 | 管什么 | 例子 |
|---|---|---|
| schema | 结构合法：字段在、类型对、必填没缺 | `mass` 是 number；`joints[].type` 在枚举里 |
| validator | 物理/跨字段合法：数值讲得通 | 惯量矩阵正定且满足三角不等式；`limit.min < limit.max`；树无环；`servo_id` 唯一 |
| provenance | 每个值**凭什么算数** | 737.2 g 是 `mjcf` 求和、744 g 是网页注记 —— 冲突时按 `source` 等级裁决 |

所以 §0 那个「三个质量」的坑，schema 管不了，validator 也管不了，只有 provenance 能回答。

## 4.1 字段表

录音里 julie 的说法是「定义一个类，什么是一个机器人」。落成八个块，每块都带出处。

| 块 | 字段（示意） | 说明 |
|---|---|---|
| `robot` | `id` `name` `version` `source_ref` `license` `generator{name,version,commit}` `created_at` `operator` | 身份与可复现性：别人拿到这个文件能重跑出同样的 MJCF |
| `frames` | `world` `root` `axes`(X 前/Y 左/Z 上) `units`(m,rad,kg,N·m,s) `quat_order` `positive_direction` | 约定层。单位错一次，后面全错 |
| `links[]` | `name` `parent` `transform{xyz,rpy}` `mass` `com` `inertia{ixx..iyz}` `visual{mesh,scale}` `collision{mesh,type}` `material{friction,restitution}` | 刚体。质量/惯量必须能标「来源」 |
| `joints[]` | `name` `type`(revolute/prismatic/ball/wheel/backlash/passive) `parent` `child` `axis{origin,dir}` `limit{mech,soft,safe}` `zero_offset` `direction` `transmission`(direct/parallel/closed_loop) `backlash` `servo_id` `actuated` | 关节。`zero_offset`/`direction` 属于 overlay，不写进 base |
| `actuators[]` | `model` `bus{protocol,baud,id}` `gear_ratio` `rotor_inertia` `torque{rated,peak}` `speed{rated,peak}` `kt` `resistance` `supply{vin,vin_min}` `control{mode,kp,kv,kd}` `delay` `friction{bam_model,params}` `params_ref` | 执行器/电机层。**这是 URDF 装不下的那部分** |
| `sensors[]` | `imu{model,mount_pos,mount_quat,fusion,noise}` `camera{intrinsics,extrinsics}` `foot_contact[]` | 传感与安装。IMU 的 mount 四元数错 90°，训练白费 |
| `provenance[]` | 每个标量一条：`path` `value` `unit` `source` `date` `note` | `source ∈ {spec, cad, mjcf, measured, identified, guessed}` |
| `state` | `status ∈ {draft, validated, aligned, calibrated, identified, trainable}` `blockers[]` | 这道门什么时候算过 |

三件套（沿用 flow.md 的规矩）：

```
robot.base.yaml      只读，导入时冻结（从 MJCF/URDF 拷进来，不许手改）
robot.overlay.yaml   只写增量：零位、方向、限位、辨识参数、并联解算结果
      ↓ resolve
robot.resolved.yaml  校验通过的那一份，训练/导出只认它
      ↓ export
robot_walk.xml / robot.usda / (robot.urdf 可选)
```

机器可读的部分（JSON Schema + 枚举）后续落在 `contract/`，与关节表同源；本阶段先在 `hardware/` 下用 YAML + pydantic 跑通。

## 5. 四个里程碑

| # | 范围 | 产物 | 验收判据 |
|---|---|---|---|
| **M1**（今天，2 小时版） | schema v0 + 鸭子这一台进来 | `robot.base.yaml` + `robot.resolved.yaml`（鸭子，14 关节）+ 结构检查脚本雏形 + 一张和 MJCF 的对照表 | 从 base 重新导出的 MJCF 与 fanhao 原文件在**拓扑/轴/限位/质量**上逐项一致（数值容差 1e-6）；`737.2 g` 这个数只有一处出处 |
| **M2**（本周） | URDF 导入 + 校验器 + 社区语料 | `importer_urdf.py`、`validate.py`、20 个社区模型的导入结果与失败清单 | 20 个里 ≥15 个能导入并显示；失败的要能说出**卡在哪个字段**，不是「报错了」 |
| **M3**（下周） | overlay 闭环 + MJCF 导出 | 零位/方向/限位回填后 `mjlab` 能直接吃导出的 MJCF；导出与训练工程跑通一次站立 baseline | sim 里站得住，且 `resolved.yaml` → MJCF → mjlab 全程无需手改 |
| **M4**（硬件到后） | 辨识回填 + 契约派生 | BAM M6 结构进 `actuators[]`；`contract/` 的关节序、obs/act 定义由 `resolved` 生成 | 真机实测值覆盖 `source: measured/identified`；`contract/` 与 `resolved` 不再各写一份 |

M2 的「社区语料」不只是演示（录音里 julie 说的「把社区所有机器人文件导进来展示」），它是**schema 的压力测试**：每遇到一个装不下的字段，就是 schema 缺一块。

## 6. 今天就做（录音里那个「今天」的版本）

1. 把 fanhao 的三个关键文件拉进 `hardware/duck-2026-11/`：`robot_walk.xml`、`joints_properties.xml`、`1910_m6.json`（连同上面的 header 注释一起，出处不能丢）。
2. 手写 `robot.base.yaml`：只填 `robot` / `frames` / `links` / `joints` / `actuators` 五块，`provenance` 每格写 `mjcf` 或 `spec`。
3. 写 `validate.py` 四条检查：树无环且父指针完整、惯量正定且满足三角不等式、限位 `min<max`、`servo_id` 唯一且与 `joints[]` 一一对应。
4. 写 `export_mjcf.py` 最小版：只处理 `<body>` 树、`<joint>`、`<inertial>`、`<geom type="mesh">`；不处理 actuator —— 先把「能读能写」跑通。
5. 跑对照：导出 MJCF 与原 MJCF 做结构 diff，把差异写成一张表（预计差异：`class` 展开方式、mesh 路径、site）。
6. 显示：复用 `model.json` 的路子（bodies + joint_ids + meshes），先出静态树，不做渲染优化。

今天**不做**：URDF importer、USD、辨识、overlay 合并逻辑。

## 7. 需要拍板

1. **改 `GLOSSARY.md` 的 canonical 定义**（URDF → 自定 schema）。建议改，改完 `flow.md` 里「USD/MJCF 都是导出产物」要补一句「URDF 是导入端」。
2. **鸭子用哪个 MJCF 变体当 `base`**：我倾向 `robot_walk.xml`（训练主线、质量最小、无滚轮），`allcollisions` 和 `_backlash` 当 overlay/导出选项。风险：站立 baseline 用的是哪个变体，要跟训练工程确认。
3. **要不要给鸭子真回生成一份 URDF**：`onshape-to-robot` 能把 `outputFormat` 换成 `urdf`，从**同一个 Onshape 文档**再生成，保真度最高（不是从 MJCF 反推）。前提是拿到该文档的 Onshape API key 与访问权限。这是一次性成本，收益是「有一份生态通用的 URDF 可以对外」。
4. **社区语料清单与许可**：要挑哪些机器人（主流人形/四足），各自模型文件的许可证能不能进我们的仓库。`LAFAN1` 那种非商用许可的坑提前避开。

## 8. 坑（按踩到的概率排）

- **单位**：MJCF 用 m/rad，URDF 用 m/rad 但 xacro 里常混 mm；STL 单位也常是 mm。一次错 1000 倍。
- **mesh 与仿真解耦**：网页 `model.json` 的 mesh 已经简化过一轮（`meshes.bin`），不能拿来当碰撞体。
- **1 个模型 6 个变体**：schema 里要么允许 `variants[]`，要么把变体当 overlay 的一层。别在 base 里塞条件分支。
- **被动/回差关节**：`passive_<joint>_backlash`、`passive_*wheel` 是 `actuated: false` 的铰链，策略的 14 维动作里没有它们。关节序一旦混进被动件，三平面立刻错位。
- **嘴（ID 34）**：不在策略里，但实物有、网页有。归类为 `passive` + `servo_id: 34`，并标 `not_in_policy`。
- **URDF 没有并联**：遇到闭环连杆只能降级成「两根开链 + 外部约束」，导入时必须**显式记录降级**，不能静默。
- **数值容差**：惯量矩阵做 diff 要按相对误差，不然舍入就差一堆。
- **许可**：MJCF 派生自 Pollen（Apache-2.0），挪进来要保留 NOTICE；我们自己的 YAML 覆盖层另标。

## 相关

- 流水线：[docs/flow.md](../flow.md)
- 术语：[GLOSSARY.md](../../GLOSSARY.md)
- 外部依据：`fanhao375/microduck-replica`（2026-11-08 实查）、`pollen-robotics/microduck_rl`（develop）、`LuwuDynamics/xgoduck_rl@326d77a1`
