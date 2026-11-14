# 工作台（studio.html）

- 入口：`/ducksir-static/client/viewer/studio.html`
- 起仿真：`POST /api/jobs {op:'sim', args:{project, seconds}}`
- 探针：`node tools/dev/probe.mjs --base <serve> --probes tools/dev/probes/studio.json`

## 控件

| id | 作用 |
|---|---|
| `#btn-start` / `#btn-stop` | 启动 / 停止工程仿真 |
| 滑杆 | 参数（前进/转向） |

## `window.__probe.state()`

`{sim, running, z, upright, speed, frames, error}`

## 已知坑

- **相机跟随**：机器人始终居中，像素几乎不动 → 断言要用 `moved`（场景对象坐标变了），不是 `changed`（像素差）；
- 场景 `race`：作业刚起时 `/api/project/scene` 可能 503，页面要重试（已重试 10 次）；
- 切换模型时要清场景 + 关旧流（`clearScene()` + `sceneGen`），否则会叠出多个机器人。
