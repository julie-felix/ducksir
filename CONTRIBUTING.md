# 协作规则

`main` 是主分支，受保护：**不能直接 push，必须走分支 + PR**。

## 分支

从 `main` 开，命名带类型：

```
feat/<短语>   新功能
fix/<短语>    修 bug
docs/<短语>   只改文档
chore/<短语>  杂务（配置、脚本、依赖）
```

## 提交信息

`type: 摘要`，中文摘要，动词开头。

```
docs: 补流水线第 6 步的辨识判据
feat: contract 加参考动作 schema
fix: 修正关节表里第 7 维的顺序错误
```

一个提交只做一件事。改五个文件只为一个目的，算一件事；顺手改的格式，另开一个提交。

## PR

- 标题和主提交一致。
- 描述按 `.github/pull_request_template.md` 填：**改了什么 / 为什么 / 怎么验证 / 影响面**。
- 没有「怎么验证」的 PR 不合规。文档改动就写「读了哪几节、哪句改了」，代码改动写「跑了什么」。
- 小步提交。超过 400 行的 PR，先拆。

## 合并

- 用 **squash merge**，源分支合并后删除。
- 合并前 `main` 必须是绿的（等 CI 有内容后再启用这条）。
- 不 force push 到已开的 PR 分支之外的分支；不删 `main`。

## 本地钩子

```bash
bash scripts/install-hooks.sh
```

装完 `.githooks/pre-push` 生效，直接 push `main` 会被本地拦下（服务端保护是第二道）。确实需要绕过时：

```bash
DUCKSIR_ALLOW_MAIN_PUSH=1 git push origin main
```

## 服务端保护

**状态：2026-11-07 已启用**（`enforce_admins: true`，管理员也不能绕过）。实测直推被拒：`GH006: Protected branch update failed`。

仓库已从私有转为**公开**：GitHub Free 套餐的私有仓库不支持分支保护，转公开后该功能立即可用，且不花钱。如果哪天要转回私有，需要先把 org 升级到 Team，否则保护会失效。

由 `scripts/protect-main.sh` 应用（需要 `GITHUB_TOKEN`，权限要 Administration: write）。它会设：禁止直推、禁止 force push、禁止删分支、必须走 PR、线性历史、只允许 squash 合并、合并后自动删源分支。

因为只有一个人开发，**required approvals 设 0**：GitHub 不允许自己批准自己的 PR，设 1 会导致自己永远合不进去。「必须走 PR」这条仍然生效。
