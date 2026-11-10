# engine — ducksim 执行核

一份 C++20 源码，编成 `.so` 给主机和板端、编成 wasm 给浏览器（W7）。核的边界只有一件事：**obs → action**。

## 硬约束（改之前先读）

1. **单文件**：`src/core.cpp`，目标 ≤2000 行。现在 462 行。
2. **arena 分配**：`ds_load` 里一次分配，`ds_step` 里没有 malloc。
3. **零全局态**：所有状态在 `ds_plan` 里，可以同时存在多份（多机器人、多策略）。
4. **确定性**：节点按声明顺序执行，固定偏移，编译时 `-ffp-contract=off` 关掉 FMA 收缩。
   同一 blob + 同一 obs 序列，任何平台算出同一份 action。
5. **不碰物理、不链 MuJoCo**（D9）；**推理只经 `ds_backend.onnx_run` 回调**（D8）。

## 构建

```bash
cd engine && make          # → build/libducksim.so（核心，零依赖）
make ort                   # → build/libducksim_ort.so（可选：链 onnxruntime）
make test                  # 需要 numpy（见 docs/env.md）→ 与 oracle 对拍
```

`libducksim_ort.so` 单独一个库是刻意的：核心要能编到 wasm、能在板子上跑，不能背 ORT 这个依赖；
浏览器那边推理走的是 ORT Web，另一条路（D8）。

没有 cmake：本机只有 g++ 13 与 make，再加一层构建系统就是多一个要维护的坑。出 wasm 在 W7 单独写。

## 接口

见 `include/ducksim.h`。最小的用起来是这样：

```c
char err[256];
ds_plan* p = ds_load(blob, blob_len, err, sizeof err);   // 失败返回 NULL，err 给原因
ds_info info; ds_info_of(p, &info);                      // input_dim / output_dim / arena_len
ds_step(p, obs, action, &backend);                       // 每步一次
ds_free(p);
```

返回码都是负数（`DS_ERR_*`），别把 0 以外的当成功。

## 与 Python 侧的关系

| 谁 | 干什么 |
|---|---|
| `sim/plan/ir.py` | 声明图 → JSON → blob，带结构校验 |
| `sim/plan/oracle.py` | numpy 参考实现（语义的可执行规格），只用于对拍 |
| `sim/test/test_core_vs_oracle.py` | ctypes 加载本核，跑同一份图，逐点比 |
| `contract/plan/README.md` | blob 布局与 8 个节点的语义（**定版在这**） |

对拍当前是 **0.000e+00**，不是「差不多」——float32 逐位一致。以后改任何节点语义，先改 oracle，再改核，再把这条跑绿。

## ORT 后端（`src/ort_backend.cpp`）

对核只暴露 `ds_backend`：`ds_ort_create(path)` 建实例，`ds_step` 里核调 `onnx_run`。

三条实现约束：

1. **只收单输入单输出、float32、静态形状**——动态维度直接拒绝（不然每步都要重新算形状，且不确定）。
2. **张量按模型声明的形状建**（`[1,34]` 不是 `[34]`）。踩过一次：摊平成一维，ORT 直接报形状不匹配，
   核收到的是 `DS_ERR_GRAPH`，跑到第二步才暴露。
3. **intra/inter op 线程锁 1**：多线程归约会让同一份输入算出不同末位，而我们对拍判据是 1e-6、目标是逐位一致。
