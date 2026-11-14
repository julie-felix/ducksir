# 宿主直播（host.html）

- 入口：`/ducksir-static/client/viewer/host.html`（面板里带 `?api=/ducksir-live`）
- 起服务：`python3 sim/host/serve.py --port 8899 --no-secret`
- 探针：`node tools/dev/probe.mjs --base http://127.0.0.1:8899 --probes tools/dev/probes/host-live.json`
- 切换模型：`probes/host-switch.json`

## 控件

| id | 作用 | 备注 |
|---|---|---|
| `#bundle` | 选 bundle | 来自 `/api/bundles` |
| `#start` | 启动 | POST `/api/jobs {op:'bundle_live'}` |
| `#fwd` `#yaw` `#lat` | 滑杆 | min/max/step/default 来自 `contract.json` 的 `robot.ui` |
| `#speed` | 高速模式 | `ui.speedMode` 为真才显示，走指令第 4 维 |
| `#pause` `#zero` | 暂停 / 归零 | |
| `#depth` | 深度小窗 | 每 3 个控制步一帧，**显示原始归一化深度**（模型吃的是 affine 之后的） |

键盘：`W/S` 前后、`A/D` 横移、`Q/E` 转向——**但以 `ui.bindings` 为准**，
PHP 模板里 `A/D` 就是转向（见 [botlab-templates.md](botlab-templates.md)）。

按住加速：`ui.rampRate`（4/s 渐变；0 = 按键瞬时），松手停在当前值。

## `window.__probe.state()`

| 字段 | 含义 |
|---|---|
| `sim` / `frames` | 仿真时间 / 收到帧数 |
| `z` / `upright` | 基座高度 / 直立度 |
| `yawDeg` | 航向角（度，逆时针为正） |
| `meshes` / `sceneGeoms` | 机器人网格数 / 场景几何数 |
| `cmd` | 当前指令 `{forward, yaw}` |
| `sceneDump`（`window.__sceneDump`） | 场景几何的 pos/size，排查"地形没渲染"用 |

## 已知坑

- **相机必须跟随**：固定相机会让机器人走出画面（`updateCamera` 用 three 世界坐标，不是 MuJoCo `xpos`）；
- **平面 size 可能是 0**：botlab 场景写 `[0,0,0.05]`，按 `size*2` 建会得到 0×0 方片 → 用 40×40 兜底；
- **场景几何不一定在 world 上**：PHP 的地形挂在静态 body，靠"机器人子树之外"判定；
- **SSE 长行偶发截断**：解析要 try/catch，否则整页卡死。
