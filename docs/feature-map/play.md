# 浏览器跑 bundle（play.html）

- 入口：`/ducksir-static/client/viewer/play.html?bundle=/ducksir-static/build/release/<name>/`
- 物理与推理都在浏览器里（wasm），不需要后端
- 探针：`node tools/dev/probe.mjs --base <serve> --probes tools/dev/probes/play-bundle.json`

## 控件

| id | 作用 |
|---|---|
| `#btn-play` | 开始 / 暂停（暂停后仿真时间不前进） |
| `#btn-auto` | 自动走（vx 到 0.25） |
| `#btn-slow` `#btn-fast` `#btn-reset` | 速度档 / 归零 |

## `window.__probe.state()`

`{sim, vx, wz, paused, autoWalk, ctrl, z, upright, strategy, error}`

## 已知坑

- 页面加载慢（GLB + onnx），探针必须 `await __probe.state().frames > N`，不能固定 sleep；
- 推理次数 `infer` 用来判断"策略真的在跑"；
- `mimic:*` 的 bundle 在浏览器里**不跑**（obs 需要深度/动作，直接早退）。
