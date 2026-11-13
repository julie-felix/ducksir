# ADR-001：把显示/验证/训练/导入统一到一个 python 后端

- 日期：2026-11-13
- 状态：已决定（庆喆批准）
- 相关：`docs/notes/g1-botlab.md` §22–§24（三次事故现场）

## 背景

现在一条「在浏览器里看机器人动」的链路是四层、两种语言：

```
python CLI (run_bundle/run_project)  →  node 插件宿主 (spawn + SSE/反代)  →  静态页  →  DSH shell
```

2026-11-12/13 连着出了三个事故，都不是算法问题：

1. `meshes.json` 列了 557 个文件（含子目录地形），浏览器逐个下，3 分钟没跑起来；
2. `mimic:*` 观测浏览器没实现，错误提示在下载之后才出；
3. `/ducksir-live/bundles` 返回 `{name, base}`，页面按 `path` 取、宿主按路径拼 → `HTTP 500: bundle 不存在`。

共同点：**每层都有自己的“bundle 是什么”**，而且没有任何东西会在形状不一致时报错；
AI 只能靠“桩宿主”自证，真实装配路径要等人来点。

## 决定

1. **一个 python 进程**（stdlib `http.server`，零依赖）承载：静态页、直播 SSE、作业（仿真/训练/导入）、文件树、诊断。
2. **node 插件宿主退化为同源反代**：只负责 https、鉴权、iframe 同源，不再持有业务逻辑。
3. **每个长任务是一个子进程 + 一条作业记录**，日志逐行转 SSE。
4. **CLI 保留为唯一真路径**：HTTP 是 CLI 的包装，AI 可以用 curl 和 CLI 互相验证。
5. **训练与导入也进来**（庆喆明确要求）。

## AI 友好的六条判据（验收标准）

| # | 判据 | 怎么算通过 |
|---|---|---|
| 1 | 可观测 | 每个失败都有 `{error:{code,message,detail}}`；页面只负责显示，不负责推理 |
| 2 | 可驱动 | `tools/dev/serve_smoke.sh` 用 curl 一条龙走完：health → bundles → 起作业 → 读到帧 → 取消 |
| 3 | 单一入口 | `python3 sim/host/serve.py --port 8899`（或 `ducksir serve`）起全栈；不再需要记四层 |
| 4 | 契约显式 | 端点返回字段固定（`base` 必填、`id` 必填）；缺字段在启动/请求期就 400 |
| 5 | 可自动断言 | 冒烟脚本有退出码；CI 能跑纯 HTTP 部分（不需要浏览器） |
| 6 | 影响面 | 新页面只调 `/api/*`，不改 node 侧 |

## 接口（第一版）

```
GET  /api/health                       → {ok, version, secretRequired, jobs}
GET  /api/bundles                      → {bundles:[{name, base}]}          base 必填
POST /api/jobs   {op, args}            → {id}
GET  /api/jobs                         → {jobs:[...]}
GET  /api/jobs/<id>                    → {id, op, status, startedAt, endedAt, exitCode, error}
GET  /api/jobs/<id>/stream             → text/event-stream（stdout/stderr 逐行）
POST /api/jobs/<id>/cancel             → {ok}
GET  /files?path=build/release/x/...   → 只读，白名单 build/ 与 runs/
```

- `op` 第一版：`bundle_live`（= `run_bundle.py --stdio --pace`）。
- 之后的 `op`：`sim`（项目仿真）、`export`（导出 bundle）、`import`（机器人导入）、`train`（训练）。
- 并发：**单并发**，后来者排队（`status=queued`），可取消。
- 鉴权：启动时生成一次性 secret 写入 `~/.local/share/ducksir/serve.secret`，请求带 `X-Ducksir-Secret`；
  仅绑 `127.0.0.1`。

## 后果

- 页面从“自己拼 URL + 猜形状”变成“只认 `/api/*` 的固定字段”；
- node 侧从 ~300 行降到代理级别；
- 训练/导入进入 HTTP 后，**长任务必须有小样本模式**（`--steps N` / `--dry-run`），否则 E2E 跑不起；
- CI 能加一条 `serve_smoke.sh`（纯 python + curl，不需要浏览器）。

## 不在范围

- 不做作业持久化恢复（进程重启后作业丢失，`runs/jobs/` 里留记录）；
- 不做多机/分布式；
- 不替换 `training/` 里的训练框架，只是包装入口。
