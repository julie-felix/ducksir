# 阶段 2「数字描述」执行计划

> 状态：**第一版已跑通**（鸭子 + G1 都能导入，检查全过，鸭子 round-trip 零差异）。
> 本文是当前阶段的执行口径；长期路线仍见 [2026-11-08 计划](2026-11-08-digital-description.md)（它的 M1–M4 成立，只是被这一阶段截断了前一段）。
> 决策来自 2026-11-09 的逐条拍板，编号 D1–D20，未定项在末尾。

## 0. 这一阶段只做五件事

1. 建起机器人的**数字描述**（实体文件）。
2. 在客户端**展示**它（3D + 树）。
3. **可视化调整关节**（滑条 + 姿态快照）。
4. **修改**数字描述（只改值，写 overlay）。
5. **导入**两种格式：URDF、MJCF。首批两台机器：`g1_29dof_rev_1_0`（宇树官方）与 microduck（fanhao 复刻）。

明确不做：训练、runtime、contract 的 obs/act、USD/SDF、SDK 参数、标定阈值、保真度矩阵、社区语料、完整 validator。

## 1. 决策（D1–D20）

| # | 决定 | 理由一句话 |
|---|---|---|
| D1 | 「数字机器人」= 静态描述本身；MJCF 与 3D 都是它的投影 | 一个源，三种投影；实时状态不入库 |
| D2 | 分 `type` 与 `instance` 两层，`session` 不落盘 | 不分开，第一台机器的标定会污染型号文件 |
| D3 | canonical = `base`（导入生成、只读）+ `overlay`（人写增量）；`resolved` 是派生物，**不入库** | 生成物入库会诱人手改，一改就是第三个真相 |
| D4 | canonical 载体是**自定 YAML**，字段名向 MJCF/URDF 借 | 只有 provenance 与 overlay 是四种格式都没有的 |
| D5 | 九块：`robot / frames / links / joints / actuators / sensors / bus / presets / provenance` | 模板化：URDF 装不下执行器，MJCF 装不下标定 |
| D6 | 关节按**语义顺序**（按运动链分组）排 canonical order | 加关节时位置是确定的；机器侧排列随时能生成 |
| D7 | 变体用 `presets`（碰撞体/滚轮/回差开关），训练默认 `walk` | 六个变体共享九成字段，复制六份必漂 |
| D8 | preset 不得改 actuated 集合、不得改质量惯量 | 否则它就不是 preset，是新 type |
| D9 | provenance 等级 `measured > identified > spec > cad > mjcf ≈ urdf > guessed`；同级冲突**报错** | 三个质量数的教训 |
| D10 | contract 引用实体（`ref` + `hash`），不复制数值 | 一个数两个家 = 换舵机时漏改一处 |
| D11 | 名字由 SDK 提供；对不上**硬失败**；别名必须落文件 | 少一个关节继续跑，真机上表现为「某条腿软」 |
| D12 | 零位/方向**只归 overlay**，SDK 侧配置由 overlay 生成 | SDK 的标定文件不可 diff、刷机就没 |
| D13 | 单位与向量约定在 SDK 边界统一为 SI；我们声明期望并做 conformance 探针 | 转换点越少，忘转的机会越少 |
| D14 | mesh：显示用 GLB（离线转换、减面），碰撞用原 STL；实体只存路径 + sha256 | G1 的 37 个碰撞体直接复用视觉 mesh，两套引用必须分开 |
| D15 | 客户端自写 FK（three.js），MuJoCo 只做离线比对 | 编辑要即时反馈；WASM 每次改都要重编 XML |
| D16 | v1 **只改值**（零位/限位/名称/顺序/质量/惯量），不许增删关节或重连 | 没有 validator 时，改值最坏是数错，改拓扑最坏是树坏 |
| D17 | 编辑写 overlay，不碰 base | base 是导入产物，改了重新导入就没了 |
| D18 | 导入成功 = **拓扑与关节完整**；其余缺失进 warnings、UI 标黄 | URDF 天生缺执行器，全字段要求会让 G1 进不来 |
| D19 | 保留最小 MJCF 导出，只当导入的单元测试 | 没有导出，导入对错只能靠肉眼 |
| D20 | 验收：两台机器都能开、拖关节、看限位、改一个值、导出、再导回一致 | 只有闭环能证明导入是对的 |

## 2. 数据流

```
third_party/<源>（原样入库，保留出处与许可）
        │  importer_{mjcf,urdf}.py
        ▼
contract/robot/types/<type>.yaml          base：入库、只读、CI 比对
contract/robot/instances/<ID>.yaml         overlay：稀疏覆盖（本轮鸭子先不给 instance）
        │  resolve（确定性，产物不入库）
        ▼
客户端（three.js + 自写 FK）：树 / 3D / 属性面板 + 滑条 + 姿态快照
        │  export_mjcf.py
        ▼
最小 MJCF ──→ 再导入 ──→ compare_entities.py   ← 这就是验收
```

## 3. 已落地

| 文件 | 作用 |
|---|---|
| `contract/robot/types/duck-hd1910.yaml` | 鸭子（15 link / 14 驱动关节 / 14 执行器 / 737.24318 g） |
| `contract/robot/types/g1-29dof.yaml` | G1（39 link / 38 关节，29 驱动 / 0 执行器 / 33.34114202 kg） |
| `third_party/fanhao-microduck-replica/` | `robot_walk.xml` 等 8 个描述文件 + 许可原文 |
| `third_party/unitree-g1/` | `g1_29dof_rev_1_0.urdf` 与其 MJCF 孪生、`motor_info.md`、README、BSD-3 |
| `tools/robot/entity.py` | SI 表、provenance、姿态/惯量换算（只依赖标准库 + PyYAML） |
| `tools/robot/importer_mjcf.py` | MJCF → 实体（default 类继承、diaginertia+quat、执行器、site/sensor） |
| `tools/robot/importer_urdf.py` | URDF → 实体（inertial rpy 旋转、fixed joint 保留） |
| `tools/robot/check_entity.py` | C1 树 / C2 名字 / C3 单位 / C4 出处冲突 / C5 preset |
| `tools/robot/export_mjcf.py` | 实体 → 最小 MJCF |
| `tools/robot/compare_entities.py` | 两份实体比对（质量/质心/惯量 Frobenius/拓扑/轴/限位/增益） |

## 4. 三个实测发现（都影响 schema）

**F1｜整机质量口径统一了。** 鸭子 15 个刚体求和 **737.24318 g**，与 2026-11-08 计划里的 737.2 g 一致；网页 `model.json` 那个 744 g 是注记值，provenance 等级低于 `mjcf`，按 D9 直接输掉，不需要人裁。

**F2｜同一条机器，两个官方文件对「怎么切 link」不一致。** G1 的 URDF 39 个 link、MJCF 30 个 body：9 个 `fixed` link（`head_link`、`d435_link`、`mid360_link`、`imu_in_*`、`logo_link`、`pelvis_contour_link`、左右 `rubber_hand`）被焊进了父 body。于是 `torso_link` 质量 6.78 → 7.818 kg、两个 `wrist_yaw_link` 各 +0.17 kg，质心与惯量也随之变；但**总质量完全一致**（33.341142 kg）。

结论：**link 的切分是建模选择，不是物理事实**。所以 `links[]` 里要能表达「这一条含哪些被焊死的子件」，跨格式比对必须先对齐切分方式，否则会得到 30 条假的「不一致」。

**F3｜两种表示各有会静默出错的坑。**
- URDF 的 `<inertial><origin rpy>` 允许旋转，惯量写在旋转后的坐标系里。G1 这一版恰好全是零（所以第一次比对看不出来），但 schema 必须支持，否则带安装角的 link 惯量主轴直接错。
- MJCF 常写 `diaginertia` + `quat`（G1 的 `waist_roll_link` 就是 `diaginertia="8.245e-06 7.079e-06 6.339e-06" quat="0.5 0.5 -0.5 0.5"`）。**不乘这个 quat，ixx 与 izz 会互换**，看上去「数值都在」，实际上主轴错了。
- MJCF 只保留 6 位有效数字：`wrist_roll` 限位 URDF 是 ±1.972222054、MJCF 是 ±1.97222；惯量非对角项被四舍五入掉。跨格式比对必须给容差，否则永远在报 2e-6 的假差异。

修掉 F3 的两条之后，G1 两条路的不一致从 38 项降到 9 项，且 9 项全部落在「焊接」这一件事上。

## 5. 怎么跑

```bash
# 鸭子（MJCF）
python3 tools/robot/importer_mjcf.py third_party/fanhao-microduck-replica/robot_walk.xml \
  -o contract/robot/types/duck-hd1910.yaml --entity-id duck-hd1910 --type-id duck-hd1910 \
  --license "Apache-2.0（双许可）" --source-ref "<upstream url>" --assets-dir assets/microduck \
  --preset walk --bam-json third_party/fanhao-microduck-replica/1910_m6.json

# G1（URDF）
python3 tools/robot/importer_urdf.py third_party/unitree-g1/g1_29dof_rev_1_0.urdf \
  -o contract/robot/types/g1-29dof.yaml --entity-id g1-29dof --type-id g1-29dof \
  --license BSD-3-Clause --source-ref "<upstream url>" --assets-dir assets/g1

# 检查（CI 就这一条）
python3 tools/robot/check_entity.py "contract/robot/types/*.yaml"

# 鸭子 round-trip：导出 → 再导入 → 比
python3 tools/robot/export_mjcf.py contract/robot/types/duck-hd1910.yaml -o /tmp/duck_rt.xml
python3 tools/robot/importer_mjcf.py /tmp/duck_rt.xml -o /tmp/duck_rt.yaml \
  --entity-id duck-hd1910-rt --type-id duck-hd1910 --license same --assets-dir assets/microduck
python3 tools/robot/compare_entities.py contract/robot/types/duck-hd1910.yaml /tmp/duck_rt.yaml

# G1 跨格式：URDF 一条路 vs 同机 MJCF
python3 tools/robot/importer_mjcf.py third_party/unitree-g1/g1_29dof_rev_1_0.xml -o /tmp/g1-mjcf.yaml \
  --entity-id g1-29dof-mjcf --type-id g1-29dof --license BSD-3-Clause --assets-dir assets/g1
python3 tools/robot/compare_entities.py contract/robot/types/g1-29dof.yaml /tmp/g1-mjcf.yaml --allow-fixed-missing
```

## 6. 未定（别当结论）

1. **JSON Schema 还没落**。现在只有 `check_entity.py` 那五条；「schema 是语言中立的那一份」这句话目前没有载体。下一步要么写 `contract/robot/schema/robot.v0.schema.json`，要么承认 v0 只有 Python 类。
2. **焊接口径**。G1 的 F2 说明 `links[]` 需要 `welded_from: [...]` 之类的表达；要不要在实体里保留被焊的子件（显示用）而导出时再合并，未定。
3. **鸭子不给 instance**（本轮明确），所以 overlay 合并逻辑只写在文档里，没有真实用例驱动。
4. **mesh 还没入库**：`assets/` 下为空，实体里只有引用（`W004`）。要用 3D 显示时必须先拉一次并按 D14 转 GLB。
5. **`bus` 块空着**：等 SDK 的统一 interface 规范。
6. **执行器单位**：`kp` 记成 `N*m/rad`、力限幅用 `N*m`；`ctrlrange` 对 position 执行器是角度，对 motor 是力矩——现在没区分，导 G1 的 `motor` 时会混。

## 相关

- 长期路线：[2026-11-08 计划](2026-11-08-digital-description.md)
- 流水线：[flow.md](../flow.md) 阶段 2
- 术语：[GLOSSARY.md](../../GLOSSARY.md)
