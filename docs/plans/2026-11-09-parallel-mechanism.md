# 并联机构（AB / PR 两套坐标）

> 状态：**结构已落，映射未解**。2026-11-09 与庆喆逐条拍板（Q7–Q20）。
> 关联：[数字描述执行计划](2026-11-09-robot-entity.md)、[GLOSSARY](../../GLOSSARY.md)。

## 0. 这件事到底在解决什么

G1 的踝和腰，**实物是并联、模型是串联**：

| | 实物 | 官方模型（URDF/MJCF） | SDK |
|---|---|---|---|
| 踝（×2） | 两个电机经连杆驱动一块平台，2 自由度 | `ankle_pitch → ankle_roll` 串联链 | `mode_pr=PR/AB` 切换解释 |
| 腰（×1，29DOF） | 同上 | `waist_pitch → waist_roll` | 同上 |

所以 `mode_pr` 不是「两种机型」，而是**同一组电机在两套坐标里的两种读数**：

```
AB 坐标（电机角）  ──f──►  PR 坐标（平台角，等价于串联模型的关节角）
      ▲                              ▲
   真机、辨识                      训练、URDF、策略
```

宇树给了几何（官方文档 + CAD），没给 `f`——SDK 原文「需自行计算并联机构运动学」。

## 1. 决策

| # | 决定 | 理由 |
|---|---|---|
| Q7 | AB 与 PR **都是真的**，各标各的，中间一条显式映射 | 任选一侧，另一侧就变成口口相传的换算 |
| Q12 | **不新增 type**：同一个 `g1-29dof` 里加 `mechanisms[]` + `preset: ab` | PR/AB 是同一台机器的两套坐标；新 type 会让每个模式复制一台机器 |
| Q13 | 先落**拓扑**，`coupling.kind: unresolved`；几何量测单列一步 | 官方 CAD 无装配树、无零件名，量测要人判读 |
| Q14 | 仿真/验证用 **MuJoCo**；Newton 只记一笔 | 本机无 NVIDIA GPU（Intel Arrow Lake 核显），Newton 要 CUDA；且只有 SolverMuJoCo 完整执行 equality，还忽略闭环关节的驱动与限位；mjlab 仍是训练栈 |
| Q15 | 验收先用「能加载、能跑 5 s 不发散」，误差阈值等第一批数据按 3σ 定（量级参考：位置 ≤1 mm、姿态 ≤0.2°） | 几何本身没核实，先定死阈值等于给假精度 |
| Q16 | 客户端**不写通用约束求解器**，将来写「这一种 2-DOF 连杆的闭式解」 | 通用求解器在浏览器里是负担，第二个机构还不存在 |
| Q17 | 官方 CAD **不入库**，只记 URL + sha256 | 宇树产品的结构件 CAD，没有明确再分发条款 |
| Q18 | 映射**自己写**，MuJoCo 建并联模型做对照（两条独立路径互验） | Newton 不提供 AB→PR 的函数，它解的是动力学 |
| Q20 | CAD 判读现在做，产出「疑似零件 → 尺寸」清单给人确认 | 没确认的数不进 canonical |

## 2. 硬事实（可复核）

- 官方文档：<https://support.unitree.com/home/zh/G1_developer/basic_motion_routine>（「G1 踝关节采用并联机构」；PR 对应 URDF；AB 需自行计算）
- 索引表：<https://github.com/unitreerobotics/unitree_mujoco/blob/main/unitree_robots/g1/g1_joint_index_dds.md>
  29DOF：`4=L_ANKLE_PITCH/B`、`5=L_ANKLE_ROLL/A`、`10=R_ANKLE_PITCH/B`、`11=R_ANKLE_ROLL/A`、`13=WAIST_ROLL/A`、`14=WAIST_PITCH/B`
  → **规律：A ↔ roll，B ↔ pitch**。AB 模式不是多出电机，是同一槽位换解释。
- 两个官方仓库（unitree_ros / unitree_mujoco）0 处 `<equality>`；社区 feidedao 那份并联踝模型作者自标「动力学不可信」，只能当几何参考。
- MJCF 里表达闭环只有两种可用写法：`<connect>`（Cassie 踝、Robotiq 2F85 都用它）与 joint equality（Robotiq 还有）；SDF 的 `must_be_loop_joint` 是 Simbody 专用开关，**不是**通用闭环机制。
- Newton v1.7.0：读 MJCF 默认 `convert_mjc_equality_constraints=True`，CONNECT → BALL 闭环关节；但显式 equality 只有 `SolverMuJoCo` 后端执行，且**闭环关节的驱动与限位被忽略**。

## 3. 官方 CAD 判读（待确认）

`G1脚踝并联结构.stp`：STEP AP214、NX 11 导出、单位 mm、60,241 个点、**7 个 MANIFOLD_SOLID_BREP、无装配层级**、零件名是 NX 特征名（认不出身份）。

| # | 包围盒 X (mm) | Y (mm) | Z (mm) | 尺寸 dx,dy,dz | 点数 | 初判（待确认） |
|---|---|---|---|---|---|---|
| 1 | -160.0..-93.7 | -6.6..75.4 | -779.5..-580.2 | 66.4, 82.0, **199.3** | 351 | 连杆/摇臂候选（细长） |
| 2 | -138.8..-124.7 | 54.7..61.0 | -623.7..-612.7 | 14.0, 6.3, 11.1 | 51 | 小件（销/螺钉） |
| 3 | -132.0..-127.8 | 14.6..23.6 | -777.3..-763.4 | 4.2, 9.0, 13.9 | 51 | 小件 |
| 4 | -208.8..-89.7 | -64.1..56.6 | -786.4..-746.7 | **119.1, 120.7**, 39.7 | 108 | 踝平台/脚板候选（扁平大方块） |
| 5 | -120.6..-91.2 | 4.5..80.3 | -574.2..-521.5 | 29.4, 75.8, 52.7 | 569 | 电机座/轴承座候选 |
| 6 | -143.3..-115.1 | 37.9..62.6 | -636.1..-594.0 | 28.2, 24.7, 42.2 | 403 | 电机座/轴承座候选 |
| 7 | -148.6..0.4 | -80.7..111.6 | -871.3..-18.8 | 149.0, 192.3, **852.5** | 5360 | 跨 852 mm，像小腿段/支架，不像踝 |

复现：`python3 tools/cad/step_solids.py "<x.stp>"`。颜色到实体的对应还没解 `STYLED_ITEM` 链（7 种颜色，可能与 7 个实体一一对应）。

**判读之后才能真正推 `f`**：需要至少「平台 + 两根连杆 + 两个电机轴心」的位置与长度，而这份文件没有零件名。

## 4. 已落地

| 文件 | 内容 |
|---|---|
| `contract/robot/specs/g1-parallel.yaml` | 手写机构块：3 个机制（左踝/右踝/腰）、PR 关节、AB 电机 index、`coupling: unresolved`、CAD 来源 |
| `contract/robot/types/g1-29dof.yaml` | `mechanisms[]` 3 条 + `presets.ab`；来源标 `robot.mechanisms_spec` |
| `tools/robot/importer_urdf.py` | `--mechanisms <spec>` 合并机构块，并给 `W007`（机构来自手写 spec，不是 URDF） |
| `tools/robot/check_entity.py` | 新增 **C6**：PR 关节必须存在且 actuated、AB 电机恰好 A+B、index 不重复、`coupling.kind` 合法；`unresolved` 算信息不算失败 |
| `tools/cad/step_solids.py` | STEP 按实体量尺寸（上面的表由它产出） |
| `client/viewer` | 侧栏「并联机构」：平台 / PR 关节 / AB 电机 / coupling 状态；未解算就写「只显示 PR 串联等效」 |

![G1 并联机构在查看器里](images/g1-mechanisms.png)

## 5. 下一步（按顺序）

1. **判读 CAD**：你确认上表哪两根是连杆、哪个是平台；我据此量出连杆长度与铰点。
2. **写 `f`**：2-DOF 平面并联的闭式解 + 数值兜底；参数进 `mechanisms[].coupling`。
3. **建对照**：MuJoCo 里用 `equality/connect` 复现并联踝，跑 AB 网格，比平台位姿。
4. **viewer 接上**：`coupling` 有解析式后，滑条切 AB 也能动（现在只显示 PR）。
5. **限位**：AB 模式下电机行程受平台限位约束，`limit` 仍按 PR 关节定义（Newton 那条坑：闭环关节的限位会被忽略）。

## 6. 未定

- 左右踝是否镜像（映射能不能共用一份参数）。
- 腰部连杆与踝不同构，映射不能照抄。
- `presets.ab` 只是声明，contract 侧「策略吃哪张坐标表」还没定。
- CAD 里 7 个实体的颜色对应关系没解。
- Newton 何时引入：等有 NVIDIA 机器、且有「批量扫参数」的需求时再说。
