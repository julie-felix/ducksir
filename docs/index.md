# 文档路由表

一页一行，一句话。找不到就从这里猜，猜不到直接问。

## 入口

- [README](../README.md) — 项目定位、目标、里程碑。
- [AGENTS.md](../AGENTS.md) — 给 agent 的仓库地图与工作约定。
- [GLOSSARY.md](../GLOSSARY.md) — 术语表，含「别用这个词」和已解决的歧义。
- [CONTRIBUTING.md](../CONTRIBUTING.md) — 分支、提交、PR 规则。

## 流程与边界

- [`docs/flow.md`](flow.md) — 从硬件到真机的完整流水线，每步的产物与通过判据。
- [`docs/plans/2026-11-09-robot-entity.md`](plans/2026-11-09-robot-entity.md) — **当前阶段**：数字描述 + 可视化 + 改值 + 导入；D1–D20 决策、三个实测发现、怎么跑。
- [`docs/plans/2026-11-08-digital-description.md`](plans/2026-11-08-digital-description.md) — 长期路线（M1–M4）：鸭子只有 MJCF 没有 URDF，canonical 自定 schema。前一段已被 11-09 取代。
- [`docs/out-of-scope/`](out-of-scope/README.md) — 讨论过但否掉的提案，提之前先读。

## 未定（别当结论）

- 数字描述：JSON Schema 还没落（现在只有 `check_entity.py` 五条检查）；G1 的「焊接」口径（URDF 39 link vs MJCF 30 body）怎么在实体里表达；mesh 未入库；`bus` 块空着等 SDK 规范。
- 客户端：3D 面板还没写；读取通道先只读，写回走同源 API 是第二步。

- 硬件路线：kissqy 还是 fanhao375。
- 舵机线：原厂 XL330 还是飞特 HD1910。
- 训练方案：运动学重定向 / 优化式 / 学习式；mjlab PPO / SuperMimic 世界模型。
- 参考动作来源：自采 / 视频 / 公开数据（LAFAN1 是 CC BY-NC-ND，SMPL-X 非商用）。
- 开源版本到底给了哪些结构件、哪些要自己补。
