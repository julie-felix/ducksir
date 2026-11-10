# wasm 端（W7）

同一份 `src/core.cpp` 编成 wasm，浏览器里跑同一张图。目标只有一条：**native 与 wasm 算出同一份 action**。

## 构建

```bash
make -C engine wasm        # 需要 emsdk（见 docs/env.md），产物在 engine/build/wasm/
```

## 跑

```bash
node engine/wasm/harness.mjs build/plan.blob build/obs.json
```

## 推理怎么接

浏览器里 onnxruntime 走的是 ORT Web（JS API），没有 C 头文件可用，
所以 wasm 端按设计 D8 的退路走：**核不链 ORT，`model` 节点的推理由 JS 通过 `ds_backend.onnx_run` 回调注入**。
native 端才链 `libonnxruntime.so`（`libducksim_ort.so`）。

## 两条已验证的事

**1. native 与 wasm 逐位相同**（`sim/test/test_wasm_parity.py`）

同一份 `core.cpp`、同一张 14 节点的图（affine / clip / history / select / phase_clock / pass-through model）、
同一条 30 步随机 obs：**最大偏差 0.000e+00**。靠 `-ffp-contract=off` + 单线程 + 定点偏移。

**2. JS 推理回调这条缝是通的**

`ds_backend` 是个 C 结构 `{void* ctx; int (*onnx_run)(...)}`。wasm 里用 `addFunction` 把 JS 函数注册进去、
把指针写进第二个字段，`model` 节点就会回调它。实测把 `--js-model 2,1`（y=2x+1）接进去，输出与期望**逐位相同**。
浏览器里把那个函数换成 ORT Web 的 `session.run` 即可，核不用改。

```bash
node engine/wasm/harness.mjs plan.blob obs.json --js-model 2,1
```

## 浏览器实链路（`client/viewer/live.html`）

页面上点一下，就在浏览器里跑：**wasm 核 + ORT Web 推理**，喂宿主采的 61 维观测，和宿主算出的 action 比。

需要先备两样（都不入库）：

```bash
python3 tools/vendor_ort_web.py                 # 抓 onnxruntime-web 运行时到 client/viewer/vendor/ort/
PYTHONPATH=$HOME/.local/share/ducksir/pylibs python3 sim/host/export_browser_check.py   # 生成 data/live/
```

图在**推理边界**切成两段：`pre.blob`（原始 obs → 策略输入）与 `post.blob`（原始 action → ctrl）。
原因很实在：ORT Web 的 `session.run` 是异步的，我们的 C 核是同步的；把边界放这儿，两边都自然。

实测（node 里跑同一份代码，`sim/test/test_browser_infer.mjs`）：

```
浏览器链：60 步｜10 节点｜61→14｜11 ms
与宿主的 action 最大偏差 1.192e-6（阈 1e-5）
```

**判据分两层，别混**：
1. **图运算** native vs wasm 是**逐位相同**（`test_wasm_parity.py`，0.000e+00）；
2. **推理**换成 ORT Web 后 kernel 不同（native AVX vs wasm SIMD），末位会差 ~1e-6，阈值定 1e-5
   （botlab 自己的 torch↔onnxruntime 奇偶校验是 1e-4，我们更严）。
