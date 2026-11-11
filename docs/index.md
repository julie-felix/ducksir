# 文档路由表

一页一行，一句话。找不到就从这里猜，猜不到直接问。

## 入口

- [README](../README.md) — 项目定位、目标、里程碑。
- [AGENTS.md](../AGENTS.md) — 给 agent 的仓库地图与工作约定。
- [GLOSSARY.md](../GLOSSARY.md) — 术语表，含「别用这个词」和已解决的歧义。
- [CONTRIBUTING.md](../CONTRIBUTING.md) — 分支、提交、PR 规则。
- [`contract/policy/README.md`](../contract/policy/README.md) — 策略清单：obs 槽顺序/缩放/历史布局、动作映射与夹取、三个频率；图从它长出来。
- [`contract/plan/README.md`](../contract/plan/README.md) — 计算图契约：blob 布局、8 个节点语义、版本与 digest、加载时拒绝什么。
- [`engine/README.md`](../engine/README.md) — C++ 执行核：单文件 / arena / 零全局态，只做 obs→action，构建与对拍怎么跑。
- [`sim/README.md`](../sim/README.md) — 宿主仿真与报告：物理在宿主、执行核只做 obs→action；PD 跟踪/物理链路两条跑法。
- [`docs/env.md`](env.md) — 本机 Python 依赖（`PYTHONPATH` 目录）、mesh 格式（MuJoCo 不认 glb）、冒烟/preset/结构对比三条命令与产物位置。

## 流程与边界

- [`docs/flow.md`](flow.md) — 从硬件到真机的完整流水线，每步的产物与通过判据。
- [`docs/plans/2026-11-09-robot-entity.md`](plans/2026-11-09-robot-entity.md) — **上一阶段**（已完成）：数字描述 + 可视化 + 改值 + 导入；D1–D20 决策、三个实测发现、怎么跑。
- [`docs/plans/2026-11-10-botlab-replica.md`](plans/2026-11-10-botlab-replica.md) — **当前阶段**（W1–W7 已完成）：复刻 BotLab 的编排能力（`sim/` + `engine/`），D1–D20、里程碑状态与「现状/下一步」见 §4 与 §10。
- [`docs/node-parity.md`](node-parity.md) — botlab 56 个节点与本仓集成情况的对照表（✅25 / ⇄2 / ⏳2 / ❌27）。
- [`docs/botlab-diff.md`](botlab-diff.md) — **与 botlab 的子系统级差异**（SDK 门面/CLI/golden/部署/场景/项目模型），三档标注行为等价·形态不等价·真缺口。
- [`docs/botlab-support-matrix.md`](botlab-support-matrix.md) — **botlab 支持哪些项目 / 能力面对照**：项目=目录+声明、模板（microduck、go2w-lab2mj）、观测源/相机/Motion/场景/HIL 的逐项差距与优先级。
- [`docs/notes/botlab-templates-inventory.md`](notes/botlab-templates-inventory.md) — **8 个 botlab 模板的全量清点**：包已下载（220 MB 在 `third_party/`）、逐项目文件结构/模型 IO 维度/配置/动作素材，末尾 5 个缺口与推荐顺序。
- [`docs/plans/2026-11-09-parallel-mechanism.md`](plans/2026-11-09-parallel-mechanism.md) — 并联机构：G1 踝/腰实物并联、模型串联；PR/AB 两套坐标 + coupling；官方 CAD 7 实体判读表。
- [`docs/plans/2026-11-08-digital-description.md`](plans/2026-11-08-digital-description.md) — 长期路线（M1–M4）：鸭子只有 MJCF 没有 URDF，canonical 自定 schema。前一段已被 11-09 取代。
- [`docs/out-of-scope/`](out-of-scope/README.md) — 讨论过但否掉的提案，提之前先读。

## 未定（别当结论）

- 并联：AB↔PR 的 `coupling` 全是 `unresolved`；CAD 7 个实体的零件身份待确认；左右踝是否镜像、腰部映射形状未定。
- 数字描述：JSON Schema 还没落（现在只有 `check_entity.py` 的 C1–C7 七条检查，W2 加 C8）；G1 的「焊接」口径（URDF 39 link vs MJCF 30 body）怎么在实体里表达；`bus` 块空着等 SDK 规范。
- 客户端：3D 查看器已进面板；读取通道只读，写回走同源 API 是第二步。

- 硬件路线：kissqy 还是 fanhao375。
- 舵机线：原厂 XL330 还是飞特 HD1910。
- 训练方案：运动学重定向 / 优化式 / 学习式；mjlab PPO / SuperMimic 世界模型。
- 参考动作来源：自采 / 视频 / 公开数据（LAFAN1 是 CC BY-NC-ND，SMPL-X 非商用）。
- 开源版本到底给了哪些结构件、哪些要自己补。
- `contract/robot/types/tienkung-lite.yaml` — 天工 Lite（官方 URDF 导入，22 link / 20 驱动），并联踝见 `specs/tienkung-lite-parallel.yaml`
