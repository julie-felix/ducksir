# 环境：Python 依赖与 mesh 格式

一页讲清「这台机器上怎么跑得动」。不写到这里的东西，下次重装就会再踩一遍。

## 1. Python 依赖（本机没有 pip、没有 uv）

依赖不是装在 site-packages 里，而是**一个目录 + `PYTHONPATH`**：

```bash
export PYTHONPATH=$HOME/.local/share/ducksir/pylibs
```

| 包 | 版本 | 说明 |
|---|---|---|
| `mujoco` | 3.17.0 | 物理；`MjModel.from_xml_path` / `mj_step` |
| `numpy` | 2.5.4 | mujoco 的硬依赖 |
| `pyyaml` | 系统自带 | `tools/robot/*.py` 用它读 canonical |

- 这个目录原先在 `/tmp/pylibs`（2026-11-09 做并联对拍时放的），`/tmp` 会被清，2026-11-10 已迁到 `~/.local/share/ducksir/pylibs`（131 MB）。
- **重建**：任何一台能装 pip 的机器上 `python3 -m pip install --target <dir> mujoco numpy`，然后把 `<dir>` 拷过来；离线环境同理，拷目录即可，不需要在目标机联网。
- 没装 mujoco 时 `tools/robot/mjcf_smoke.py` 会跳过并 exit 0（CI runner 就不装）。

## 1b. onnxruntime（推理）与 onnx（造测试模型）

ORT 有两份东西，位置分开：

| 东西 | 位置 | 干什么 |
|---|---|---|
| **Python 包**（`onnxruntime` / `onnx` / `ml_dtypes` / `protobuf` / `sympy` / `packaging`） | `$PYTHONPATH` 那套 pylibs | numpy 侧参考推理、造测试模型 |
| **ORT SDK**（头文件 + `libonnxruntime.so`） | `~/.local/share/ducksir/ort` | `engine/src/ort_backend.cpp` 编译链接 |

```bash
# SDK 是 GitHub release 里的 tgz（wheel 不带头文件），版本与 Python 包对齐 1.31.1
curl -sSL --noproxy '*' -o /tmp/ort.tgz \
  https://github.com/microsoft/onnxruntime/releases/download/v1.31.1/onnxruntime-linux-x64-1.31.1.tgz
mkdir -p ~/.local/share/ducksir/ort && tar xzf /tmp/ort.tgz -C ~/.local/share/ducksir/ort --strip-components=1

# Python 包：直接从 PyPI 下 wheel 解开到这个目录（本机没有 pip）
#   onnxruntime, onnx, protobuf, ml_dtypes(cp312), sympy, packaging, typing_extensions
```

换机器时用 `DUCKSIR_ORT=/path make -C engine ort`。

## 2. mesh：MuJoCo 不认 glb

`assets/*` 里全是给查看器用的 **GLB**（`tools/robot/stl2glb.py` 转出来的），而 MuJoCo 只吃 **STL / OBJ / MSH**。三个来源：

| 机型 | mesh 来源 | 格式 |
|---|---|---|
| 鸭子 | `third_party/fanhao-microduck-replica/assets/` | 原始 `.stl`（38 个） |
| G1 | `third_party/unitree-g1/meshes/` | 原始 `.STL`（35 个，**扩展名大写**，MuJoCo 区分大小写） |
| 天工 Lite | 没有原始 STL，只有 `assets/tienkung/*.glb` | 用 `tools/robot/glb2stl.py` 现转 |

```bash
# 天工的 glb → stl（写进 build/，已 gitignore）
python3 tools/robot/glb2stl.py assets/tienkung build/meshes/tienkung
```

两个硬限制（都踩过）：

1. **STL 解码器上限 20 万面/文件**，而给查看器的模型是重模——天工那块 `waist_link` 有 66.8 万面，直接转必炸。`glb2stl.py` 默认做顶点聚类简化（`--max-faces 150000`，0 = 不简化），腰被并到 2.07 万面。
2. **meshdir 写绝对路径不可提交**。校验产物统一写 `build/mjcf/<robot>.xml`，meshdir 用相对路径指回去，这样整棵树可以换机器。

## 3. 冒烟：三台机型能不能加载

```bash
export PYTHONPATH=$HOME/.local/share/ducksir/pylibs
python3 tools/robot/mjcf_smoke.py
```

它做的事：导出三台机型 → `MjModel.from_xml_path` → 抓 stderr 数 MuJoCo 警告 → 比总质量（判据 1e-6）。当前结果（2026-11-10）：

```
机型             load   body  jnt  act        质量(kg)     canonical warn
duck-hd1910    OK       16   15   14    0.73724318    0.73724318    0
g1-29dof       OK       40   30    0   33.34114202   33.34114202    0
tienkung-lite  OK       23   21    0   42.51599707   42.51599707    0
3/3 通过
```

`act=0` 那两台不是 bug：它们的 canonical 里 `actuators` 还是空的，补 `kp/kd` 是 W2–W3 的活（见计划 D13）。

## 4. 产物与三个命令

```bash
export PYTHONPATH=$HOME/.local/share/ducksir/pylibs

# 一条命令出全套：mjcf + preset + 校验
python3 tools/robot/mjcf_smoke.py

# 用「全碰撞集」导出（摔倒/上机前验证用；默认那条是 walk 子集，摔到头会穿地）
python3 tools/robot/export_mjcf.py contract/robot/types/duck-hd1910.yaml \
    -o build/mjcf/duck-hd1910-all.xml --mesh-dir third_party/fanhao-microduck-replica/assets \
    --mesh-ext stl --collision-set all

# 单独生成 preset（13 个 botlab 字段）
python3 tools/robot/export_preset.py contract/robot/types/duck-hd1910.yaml \
    --mjcf build/mjcf/duck-hd1910.xml -o build/preset/duck-hd1910.preset.yaml

# 回放画面与物理一致性（viewer FK vs MuJoCo xpos）
python3 tools/robot/check_replay_fk.py client/viewer/data/duck-hd1910.json \
    client/viewer/data/replay-duck-pd.json build/mjcf/duck-hd1910.xml

# 造一份合成测试策略（14 维关节，把关节往零位拉）
python3 sim/plan/make_test_policy.py -o build/models/duck-hold.onnx

# 策略闭环：obs → C 核（含 ONNX 推理）→ action → MuJoCo（W5 验收）
python3 sim/host/run_policy.py --mjcf build/mjcf/duck-hd1910-all.xml \
    --preset build/preset/duck-hd1910.preset.yaml \
    --manifest contract/policy/duck-hold.manifest.yaml \
    --seconds 30 --report build/report/policy.json \
    --trajectory client/viewer/data/replay-duck-policy.json

# 与上游 MJCF 比结构
python3 tools/robot/compare_mjcf.py build/mjcf/duck-hd1910.xml \
    third_party/fanhao-microduck-replica/robot_walk.xml
```

产物全部落 `build/`（gitignore）：`build/mjcf/<robot>.xml`、`build/meshes/<robot>/`、`build/preset/<robot>.preset.yaml`。
**preset 是派生物，不手改**；它带 `generated.entity_sha256` 与 `generated.mjcf_sha256`，能追到是哪份实体、哪份导出。

## 5. 仿真跑法

宿主物理与报告在 `sim/`：[`sim/README.md`](../sim/README.md) 有两条现成命令（链路验证 / 物理验证）。
依赖同上（`PYTHONPATH=$HOME/.local/share/ducksir/pylibs`），前置产物由 `tools/robot/mjcf_smoke.py` 生成。

## 6. botlab 模板资产（真策略从哪来）

botlab 的产品站把模板按 `templates/<id>/<相对路径>` 直接暴露，MicroDuck 那套可以白拿（2026-11-10 实测 200）：

```bash
B=https://botlab.enkeebot.com/templates/microduck
curl -sSL -x http://127.0.0.1:7890 -o build/models/walking.onnx $B/assets/models/walking.onnx
curl -sSL -x http://127.0.0.1:7890 -o /tmp/microduck.xml      $B/assets/robot/MicroDuck/microduck.xml
curl -sSL -x http://127.0.0.1:7890 -o /tmp/microduck.preset.yaml $B/microduck.preset.yaml
curl -sSL -x http://127.0.0.1:7890 -o /tmp/microduck.py        $B/microduck.py
```

七个策略：`walking / sitstand / kick-left / kick-right / roll / ground-pick / stand`，全部 `obs[1,61] → actions[1,14]`。
上游是 <https://huggingface.co/spaces/pollen-robotics/microduck-simulator>，**没有根 LICENSE**：只做本地对照，不入库、不对外分发。

## 7. emscripten（wasm 端）

`make -C engine wasm` 需要 emsdk。本机装法是 git clone + `./emsdk install latest`（约 240 MB，走代理慢，第一次要十几分钟）：

```bash
git clone --depth 1 https://github.com/emscripten-core/emsdk.git ~/emsdk
cd ~/emsdk && ./emsdk install latest && ./emsdk activate latest
```

Makefile 里默认取 `~/emsdk/upstream/emscripten/em++`，换路径用 `EMXX=/path make -C engine wasm`。

**下载必须绕代理**（2026-11-10 实测）：emsdk 从 `storage.googleapis.com` 拉 322 MB 的 wasm-binaries，
走 Clash 代理只有 **31 KB/s**，直连 **5.8 MB/s**（快 186 倍）。所以装的时候要把代理变量摘掉：

```bash
env -u https_proxy -u HTTPS_PROXY -u http_proxy -u HTTP_PROXY -u all_proxy -u ALL_PROXY ./emsdk install latest
```

不摘的话 `./emsdk install` 会卡在几 MB 十几分钟，看起来像断网。
产物在 `engine/build/wasm/`（`ducksim.mjs` + `ducksim.wasm`），跑法见 `engine/wasm/README.md`。

**不在 CI 里跑**：runner 没有 emsdk，`test_wasm_parity.py` 缺产物会自动跳过（exit 0）。

## 7.5 统一入口 `ducksir`

```bash
bin/ducksir validate              # 实体 C1–C9 + manifest + 图编译 + MJCF load，一次查完
bin/ducksir build                 # 产出 build/release/<时间戳>/：duck.xml + preset + blob + 模型 + contract.json + release.json
# 编译（Python 只在这里出现）：
bin/ducksir build --project sim/test/fixtures/project_demo.py --out build/release/demo
# 运行（只吃产物）：
bin/ducksir run build/release/demo --seconds 20 --vx 0.25
bin/ducksir run build/release/demo --seconds 18 --fsm --pulses 3:1,8:1
bin/ducksir run sim/test/fixtures/project_demo.py --seconds 20   # 兼容老路径（逐步淘汰）
bin/ducksir run project.py --no-browser --report build/report/p.json
bin/ducksir sim --seconds 20 --vx 0.25      # 单策略闭环（原 `run --policy`）
bin/ducksir sim --fsm --pulses 3:1,8:1      # 14 状态机（原 `run --fsm`）
bin/ducksir golden                # 与快照逐位比（改核后跑）
bin/ducksir golden --update       # 有意改行为时重新记录基准
```

面板「实验台」读一次运行的报告：`bin/ducksir run --report client/viewer/data/live/report.json` 之后刷新即可。
Python 里用薄门面：

```python
import sys; sys.path.insert(0, "sim/sdk")
from ducksir import Duck
d = Duck(); print(d.joints()); report = d.run(seconds=10, vx=0.25)
```

退出码统一（0 通过 / 1 失败）。都是薄包装（`tools/cli/ducksir.py`），真现在老脚本上。

`run` 跑的是**项目脚本**（只声明，不启动）：

```python
# project.py
import sys; sys.path.insert(0, "sim/sdk")
from ducksir import Project
project = (Project("duck-hd1910").policy("walking").command(forward=0.25).rates(control_hz=50, physics_hz=200))
```

面板里也能用：工具栏「工作台」页签 → 填项目路径 → 点「启动仿真」（插件会在空闲端口起进程并代理，
所以手机走同源地址就能用）。显示：`run` 默认起本机显示服务（SSE，只绑 127.0.0.1）并打开 `follow.html` 实时跟随；
`--no-open` 只起服务不开浏览器、`--no-browser` 全关。`--pace realtime|max|auto` 控制节拍。
冒烟：`bash sim/test/test_cli.sh`。

## 8. 浏览器里跑物理 + 推理（素材）

```bash
python3 tools/vendor_mujoco_web.py     # @mujoco/mujoco 4.0.0 的 js+wasm（11 MB）+ 38 个 STL（21 MB）
python3 tools/vendor_ort_web.py        # onnxruntime-web 的 wasm-only 运行时（14 MB）
# 策略包：plan blob + 模型 + **契约**（contract.json，唯一真源）
PYTHONPATH=$HOME/.local/share/ducksir/pylibs python3 sim/host/export_artifacts.py
# 宿主参照运行素材（obs / actions / meta），给 live.html 做自算比对
PYTHONPATH=$HOME/.local/share/ducksir/pylibs python3 sim/host/export_browser_check.py
# 浏览器版 MJCF（只导碰撞 mesh，STL 从 21 MB 降到 7.1 MB）
PYTHONPATH=$HOME/.local/share/ducksir/pylibs python3 tools/robot/export_mjcf.py \
    contract/robot/types/duck-hd1910.yaml -o client/viewer/data/mjcf-browser/duck.xml \
    --mesh-dir . --mesh-ext stl --collision-set all --physics-only
```

三样都不入库（`.gitignore` 里有）：`client/viewer/vendor/{mujoco,ort}/`、`client/viewer/assets/mjcf/`、`client/viewer/data/live/*.onnx`。
页面：`/client/viewer/play.html`（物理+推理，键盘开鸭子）、`/client/viewer/live.html`（只做自算比对）。

**都直连下载**：npm registry 与 storage.googleapis.com 直连比走代理快一个数量级。

**版本必须对齐**：`@mujoco/mujoco` 的版本要和宿主 Python 的 `mujoco` **完全一致**（现在都是 3.17.0）。
4.0.0 的浏览器版跑同一串 ctrl，200 步后基座差 85 mm——接触密集的步态直接失稳。物理版本是配方的一部分。
