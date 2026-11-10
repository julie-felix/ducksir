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

# 与上游 MJCF 比结构
python3 tools/robot/compare_mjcf.py build/mjcf/duck-hd1910.xml \
    third_party/fanhao-microduck-replica/robot_walk.xml
```

产物全部落 `build/`（gitignore）：`build/mjcf/<robot>.xml`、`build/meshes/<robot>/`、`build/preset/<robot>.preset.yaml`。
**preset 是派生物，不手改**；它带 `generated.entity_sha256` 与 `generated.mjcf_sha256`，能追到是哪份实体、哪份导出。

## 5. 仿真跑法

宿主物理与报告在 `sim/`：[`sim/README.md`](../sim/README.md) 有两条现成命令（链路验证 / 物理验证）。
依赖同上（`PYTHONPATH=$HOME/.local/share/ducksir/pylibs`），前置产物由 `tools/robot/mjcf_smoke.py` 生成。
