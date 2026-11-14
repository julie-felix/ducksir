# ADR-002：UI / 渲染 / 实时交互怎么验

- 日期：2026-11-13
- 状态：已决定（庆喆要求“把流程建立好”）
- 相关：`docs/adr/001-unified-python-backend.md`、`docs/notes/g1-botlab.md` §26

## 背景

一天里出了两次「AI 说好了、人一看不对」：

1. `meshes.json` 557 个文件 → 页面卡在下载（AI 只验了后端）；
2. 机器人被渲染成**躺倒**（AI 只验了「帧在流」，没验「画得对」）。

肉眼一看就发现的问题，检查没抓到。根因：**断言停在了“数据有没有”这一层，没有落到“画出来/交互起来对不对”**。

## 决定：三层断言 + 页面自述

每个有画面的页面，验证分三层，越往下越贵、越靠后：

| 层 | 验什么 | 用什么 | 能不能进 CI |
|---|---|---|---|
| L1 数据 | 后端状态、业务指标（z/upright/vx/帧号） | `curl` + `serve_smoke.sh` | ✅ 能 |
| L2 语义 | DOM 文本、`window.__probe` 里的数字、无 console error | CDP（`tools/dev/probe.mjs`） | ⚠️ 要浏览器，本地/按需 |
| L3 画面 | 非空像素、目标占比、相邻帧差异（“在动”）、截图留证 | CDP 截图 + 像素统计 | ⚠️ 同上 |
| L4 基线图 | 与历史基准比对 | 只在相机固定时用，不做全像素 | ❌ 不进 CI |

**L3 的判据选“像素统计”，不选“像素比对”**：相机、光照、粒子细节一变基线就红，维护成本远高于收益。
真正要防的是「一片黑 / 机器人放倒 / 没动」，这三件事像素统计都能抓（非空、占比、帧差）。

## 页面约定：`window.__probe`

有画面的页面必须暴露一个只读对象，供探针断言：

```js
window.__probe = {
  state() {            // 返回值必须可 JSON 序列化
    return {
      sim: 12.5,         // 仿真秒数
      frames: 380,       // 已收到/已渲染的帧数
      z: 0.752,          // 业务指标（能算就算）
      upright: 0.99,
      meshes: 35,        // 场景里实际挂上去的几何数量
      error: null,       // 页面级错误（有就必须能读到）
    };
  },
};
```

断言只写数字和文本，不写“看起来对不对”。

## 探针脚本：`probes/<page>.json`

```json
{
  "url": "/ducksir-static/client/viewer/host.html",
  "steps": [
    { "wait": 3000 },
    { "assert": "js", "expr": "__probe.state().meshes > 10", "why": "网格挂上了" },
    { "wait": 6000 },
    { "assert": "js", "expr": "__probe.state().frames > 100", "why": "帧在流" },
    { "assert": "js", "expr": "__probe.state().z > 0.5 && __probe.state().upright > 0.8", "why": "站着" },
    { "assert": "pixels", "minRatio": 0.02, "why": "画面不是空的" },
    { "assert": "changed", "why": "相邻两帧有差异（在动）" },
    { "shot": "host-live" }
  ]
}
```

跑法：

```
node tools/dev/probe.mjs --base http://127.0.0.1:8899 --probes tools/dev/probes/host-live.json
```

失败时退出码 1，并把截图路径打出来；`--shot-dir` 默认 `/tmp/probes`。

## 交互怎么验

交互（点击/按键/拖拽）都用「**动作 → 状态变化**」断言，不截图猜：

| 动作 | 断言 |
|---|---|
| 点「启动」 | `state().frames` 从 0 变 >0，且 `error == null` |
| 按 `W` | `state().command.forward > 0`（或 vx 变化） |
| 点「暂停」 | 1 s 后 `state().sim` 不变 |
| 拖拽视角 | `state().camera` 前后不同（页面暴露相机位置） |

## 视觉兜底（不进 CI）

截图留下来，AI 用看图能力做一次人工复核（这一层不做回归）。
规矩：**AI 说自己验过，必须同时给出 L2 数字和 L3 截图**；只有截图没数字 = 没验。

## 落地（2026-11-13）

- `tools/dev/probe.mjs`：CDP 探针，读 `probes/*.json`，支持 `wait / click / key / assert(js|pixels|changed) / shot`；
- 页面约定已落到 `host.html`：`window.__probe.state()` 返回 `{sim, step, frames, z, upright, meshes, bodies, error}`；
- 第一个探针 `tools/dev/probes/host-live.json`，8 条断言全 PASS：

```
PASS  页面暴露了 __probe
PASS  网格挂上了（>10）
PASS  帧在流（>60）
PASS  基座 >0.5 m（没躺下）
PASS  直立 >0.8
PASS  画面不是空的        非空 4.7%
PASS  画面在变化（在动）  逐像素平均差 0.85
PASS  无 console 错误
```

两个实现细节（都是踩出来的）：

1. **WebGL canvas 读像素要 `preserveDrawingBuffer: true`**，否则探针采样到全 0、报“画面是空的”假失败；
2. 「在动」不能只看亮度均值（机器人摆臂时均值几乎不变）——改成**同画布采样两次做逐像素平均绝对差**。

## 三个页面 + 三条交互探针（2026-11-13）

| 探针 | 覆盖 |
|---|---|
| `probes/host-live.json` | 宿主直播：网格挂上、帧在流、基座/直立、画面非空、**画面在变** |
| `probes/play-bundle.json` | 浏览器跑：wasm 物理 + 推理、画面非空、**点「自动走」→ vx>0.1**、**点「暂停」→ 仿真时间不再前进** |
| `probes/studio.json` | 工作台：点「启动仿真」→ 场景 body 就绪、帧数、基座/直立、画面非空、**场景对象在动** |

`node tools/dev/probe.mjs --base http://127.0.0.1:8899 --probes host-live.json,play-bundle.json,studio.json` 全 PASS。

### 写探针时又踩的四个坑

1. **`moved` 与 `changed` 要分开**：相机跟随的页面（studio）里机器人始终居中，像素几乎不动；
   这时要断言**场景对象的坐标在变**（`window.__dbg.state().sample`），而不是像素差。
2. **一个探针一个浏览器**：同一个 Chrome 跑三个页面，第三个的 WebGL 上下文会丢（帧还在流、画面冻住），
   看起来像页面 bug。
3. **`wait` 不如 `await`**：实时页面加载 20 s+，固定 sleep 会误判；探针要轮询
   `__probe.state().frames > N` 这种可观测条件。
4. **改了 studio 的 `/api/project/stream` 走「作业日志」后**，页面要解包 `{"line": ...}`，
   否则拿到启动日志行会 `f.t.toFixed` 报错——而错误被 try/catch 吞掉，表现是「画面不动」。

### 顺手修的两个真 bug

- **SSE 代理不能 `read(1024)`**（等凑满才返回 → 页面只有 ~5 Hz，最后像卡死）；改 `readline()` 逐行转发。
- **studio 的场景 race**：作业刚起、`/api/project/scene` 还没 200，页面静默拿空场景
  → `groups` 全在 (0,0,0)。改成重试 10 次。

## 不在范围

- 不做像素级 golden（相机一变就废）；
- 不做跨浏览器矩阵（只有 chromium）；
- 不把浏览器探针塞进 CI（需要 WebGL + 显存，跑不稳）。
