# AGENTS.md

这是 DuckSIR 的入口，只做路由，不放内容。第一次进来先读 [docs/index.md](docs/index.md)。

## 这个仓库是什么

基于 MicroDuck **开源版本**手搓的机器鸭，加一条「重定向 → 训练 → sim2sim → sim2real」的运控流水线。硬件、训练、端侧、客户端在一个 monorepo 里。

## 结构

| 路径 | 放什么 |
|---|---|
| `docs/index.md` | 文档路由表，**从这里开始** |
| `contract/` | 三平面共享契约（关节表、schema、manifest），机器可读 |
| `hardware/` | BOM、结构件、板卡、标定记录 |
| `training/` | 重定向、跟踪训练、评估、导出 |
| `runtime/` | 端侧：总线、安全层、推理 |
| `client/` | DSH 插件 |
| `docs/sop/` | 人执行的操作手册 |
| `docs/adr/` | 决策记录（含理由与否掉的方案） |
| `docs/out-of-scope/` | 讨论过但否掉的提案，先读再提 |

## 两条硬边界

见 [README](README.md) 第 0 节（复刻开源版本、客户端不进控制回路）。

## 工作约定

- 改结论先改 docs，再改代码；改动同步 `docs/index.md` 那一行。
- 没定的事写「未定」，不要装作定了。未定清单在 `docs/index.md` 末尾。
- 文档用中文，代码标识符、schema 字段、路径用英文。
- 不编造 MicroDuck 的规格。没核实的数不进文档。
- 入口只写一次。发现两处写同一件事，删掉一处。
- **不直接 push main**：开分支 → PR → squash 合并。规则见 [CONTRIBUTING.md](CONTRIBUTING.md)。
