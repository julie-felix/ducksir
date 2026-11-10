# 计算图契约（plan IR v1）

这里定的是**编排层的硬契约**：Python 怎么写图、C++ 核怎么读图。改这里的任何一条，三端都要重编、所有产物要重发。

- 源码：Python 声明（`sim/plan/ir.py`）→ **JSON**（可 diff、可 review、可给 AI 读）
- 产物：**blob**（定长记录 + 固定偏移，C 核零解析直接跑）
- 语义规格：`sim/plan/oracle.py`（numpy 参考实现，逐点对拍 ≤1e-6）
- 执行核：`engine/`（C++20，单文件、arena、零全局态）

## 1. 版本与协商

| 字段 | 值 | 说明 |
|---|---|---|
| `version` | 1 | plan 格式版本；不兼容变更才 +1 |
| `abi` | 1 | 执行核 ABI；与 `ducksim.h` 的 `DUCKSIM_ABI` 对齐 |
| `required[]` | `(type_id, min_version)` | 图用到的节点类型与最低节点版本 |
| `digest` | FNV-1a 64 | 覆盖头之后的全部字节。**非加密**，只防「包被改过 / 版本串了」 |

加载策略是**整份拒绝**，不是跑一半：核遇到不认识的节点类型、或自身版本低于 `min_version`、或 digest 对不上，`ds_load` 直接返回 NULL 并给出原因。

## 2. blob 布局

头固定 48 字节：

| 偏移 | 大小 | 字段 |
|---|---|---|
| 0 | 4 | magic `"DSIM"` |
| 4 | 2 | `version` |
| 6 | 2 | `abi` |
| 8 | 2 | `node_count` |
| 10 | 2 | `required_count` |
| 12 | 2 | `input_dim`（obs 长度，float 数） |
| 14 | 2 | `output_dim`（action 长度） |
| 16 | 4 | `arena_len`（图用到的 float 总数） |
| 20 | 4 | `param_len`（参数字符数） |
| 24 | 4 | `flags`（保留，现为 0） |
| 28 | 4 | reserved0 |
| 32 | 4 | reserved1 |
| 36 | 4 | reserved2 |
| 40 | 8 | `digest` |

之后依次是：

```
required 表：required_count × 4B  { u16 type, u16 min_version }
节点表    ：node_count     × 32B  见下
参数表    ：param_len      × 4B   float32
```

节点记录 32 字节：

| 偏移 | 大小 | 字段 |
|---|---|---|
| 0 | 2 | `type` |
| 2 | 1 | `n_in`（≤4） |
| 3 | 1 | `flags`（bit0 pass-through / bit1 native-only） |
| 4 | 16 | `in[4]` × {u16 off, u16 len} |
| 20 | 2 | `out_off` |
| 22 | 2 | `out_len` |
| 24 | 2 | `param_off` |
| 26 | 2 | `param_len` |
| 28 | 4 | 补齐 |

**所有 `off/len` 都是 float 下标**，指向同一个 arena 或参数表。arena 在 `ds_load` 时一次性分配，`ds_step` 里一次 malloc 都没有。

## 3. 八个节点（W4 的节点集）

| id | 名字 | 语义 | 参数 |
|---|---|---|---|
| 1 | `obs` | 从 obs 缓冲搬一段：`out = obs[src_off : src_off+len]` | `[src_off, len]` |
| 2 | `concat` | 按输入顺序拼接（1–4 个） | — |
| 3 | `affine` | `out = x * scale + offset` | `scale[len], offset[len]` |
| 4 | `clip` | `out = clamp(x, lo, hi)` | `lo[len], hi[len]` |
| 5 | `reorder` | `out[i] = x[perm[i]]` | `perm[len]`（0..n-1 的排列，存成 float） |
| 6 | `history` | 环形缓冲，输出最近 `frames` 帧按 **by-frame** 铺开（最老在前） | `[frames]` |
| 7 | `model` | 调宿主推理后端：`onnx_run(model_id, in, out)` | `[model_id]` |
| 8 | `out_action` | 唯一的 sink，`action = x` | — |
| 9 | `constant` | 常值向量 | 值本身 |
| 10 | `select` | 按 mode 选一路（≤3 路）；不匹配输出全零或 passive 那一路 | `mode值×n, width, passive` |
| 11 | `where` | `cond ? a : b`（两边都算，不短路） | — |
| 12 | `phase_clock` | `fmod(steps, period)/period`，落在 [0,1) | `[period_steps]` |
| 13 | `add` | 逐元素相加（≤4 路），分块 select 后用它消零 | — |
| 14 | `state_machine` | 状态机，输出 2 维 `[state, steps]`；转移按「command 脉冲 / after_steps / when」三类，同类按声明顺序取首个命中 | 表长、初始态、条件数 + 每条转移 6 个数 |
| 15 | `compare` | 两路 1 维比较，输出 0/1（`gt/gte/lt/lte/eq`） | `[op]` |

两条容易踩的：

1. **`history` 首步是「前面全零」**：缓冲在 `ds_reset`（以及加载后）清零，首步输出 = `[0 … 0, x]`。这条必须两端一致，否则策略看到的历史会差一截。
2. **`model` 不自己推理**：核里没有 onnxruntime，只有一个 C 函数指针（`ds_backend.onnx_run`）。`pass-through` flag 只给测试用（不调后端、直接拷贝），出产物前必须清掉。

`obs` 只有一个通用的「搬运」节点，不做 `joint_pos` / `projected_gravity` 这种语义节点——语义由上层 manifest 决定（D12），核只管偏移。这样加一个观测不用改核。

## 4. 加载时核会拒绝什么

```
magic 不是 DSIM / version 不是 1 / abi 不匹配 / digest 不匹配
required 里有未知节点类型，或 min_version 高于核版本
节点类型未知 / n_in > 4 / 输出或参数越界
输入不是任何**前序**节点的输出（声明顺序必须是拓扑序）
out_action 不是恰好一个，或长度 ≠ output_dim
history 缺 frames、frames 非法、out_len 不能被 frames 整除
model 节点既没有 pass-through 又没有 model_id
```

这些在 Python 侧（`ir.validate`）也有同一套，早一步挡住；核里的是第二道。

## 5. 怎么跑

```bash
# 造图并编译成 blob（只依赖标准库）
python3 sim/test/test_blob_roundtrip.py

# 与 numpy oracle 逐点对拍（要 numpy + g++）
cd engine && make && cd ..
PYTHONPATH=$HOME/.local/share/ducksir/pylibs python3 sim/test/test_core_vs_oracle.py
```

当前结果：`simple` / `rich` 两个图各 40 步，**最大偏差 0.000e+00**（float32 逐位一致）。

## 契约（`contract.json`）

图编译出来是一组 blob，但**光有 blob 端侧没法用**——它还得知道「输入第几维是什么、输出第几段是什么」。
以前这些偏移写在消费端代码里（浏览器 `play-core.mjs` / `play.js`），结果是：改 Python 生成新 IR 之后，
端侧用旧偏移读新 blob，**不报错，只是行为诡异**（2026-11-10 的指令映射漂移就是这么来的）。

现在编译时把这些形状一并产出。`sim/plan/contract.py` 生成、`sim/host/export_artifacts.py` 落盘：

```json
{
  "kind": "ducksir-contract",
  "version": 1,
  "rates": { "control_hz": 50, "physics_hz": 200, "inference_decimation": 1 },
  "plans": {
    "pre":  { "file": "pre.blob",  "input_dim": 61, "output_dim": 61, "digest": "…",
              "slots": { "in:obs": [0, 61] } },
    "post": { "file": "post.blob", "input_dim": 14, "output_dim": 14, "digest": "…",
              "slots": { "in:action": [0, 14], "out:ctrl": [0, 14] } },
    "fsm":  { "file": "fsm.blob",  "input_dim": 4,  "output_dim": 16, "digest": "…",
              "slots": { "in:policy_command": [0,1], "in:gravity_z": [1,1],
                         "in:vx": [2,1], "in:wz": [3,1],
                         "out:state": [0,1], "out:mode": [1,1],
                         "out:command": [2,13], "out:clear_last_action": [15,1] } }
  },
  "policies": [{ "key": "walking", "mode": 0, "file": "walking.onnx" }],
  "obs": [{ "name": "imu_ang_vel", "slot": [0,3], "source": "sensor:imu_ang_vel" }],
  "states": { "count": 14, "initial": 0 },
  "robot": { "joints": [...], "default_q": [...], "initial_qpos": [...], "ctrlrange": [...] }
}
```

消费方（浏览器 / 宿主 / 板端）只做三件事：

1. 读契约；
2. `ds_load` 之后核对 `ds_info` 的维度与契约一致（不一致直接拒绝启动）；
3. 按**名字**取槽位（`slots["out:command"]`），不写任何偏移数字。

`obs` 是**观测配方**：端侧照 `source` 取值（`sensor:<名>` / `gravity:body:<名>` / `qpos:joints` /
`qvel:joints` / `last_action` / `fsm_slot:<槽位>`），所以改观测顺序也不用动端侧代码。

**收益**：加策略、改 `fsm` 输出维度、换观测布局——全部只改 Python/YAML 后重新导出；
端侧零改动，且「漂移」从静默失败变成加载即报错。守卫测试：`node sim/test/test_contract_guard.mjs`。

**已知边界**：宿主侧 `run_policy.py` / `run_fsm.py` 目前仍按 manifest 自己组观测（不走契约），
两边口径靠对拍测试（`test_fsm_parity.mjs`）钉住，尚未合并到契约驱动。

## 加一个新节点：准入标准

Q9 定下「不开 Python 每步逃生口，表达不了就扩图」之后，节点集合就是**唯一的扩展出口**。
但节点不是随便加的——加一个要同时改：C 核、numpy oracle、wasm 构建、契约、浏览器通用执行器、golden 快照。
所以三条缺一不可：

1. **跨端可用**：纯张量运算、无 I/O，native 与 wasm 都能跑。带 I/O 或依赖宿主的节点会破坏「两端逐位一致」。
2. **可对拍**：能在 `sim/plan/oracle.py` 里写出参考实现。写不出参考实现的节点，等于不能验证。
3. **有真实需求**：至少有一个策略/项目现在就卡在它上面，不为「以后可能用」而加。

加完的检查单：`test_core_vs_oracle.py`（对拍）、`test_wasm_parity.py`（逐位）、
`bin/ducksir golden --update`（有意改变行为才更新，并写清原因）、`node-parity.md` 补一行。
