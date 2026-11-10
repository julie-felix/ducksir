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
cd engine && make          # → build/libducksim.so
make test                  # 需要 numpy（见 docs/env.md）→ 与 oracle 对拍
```

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
