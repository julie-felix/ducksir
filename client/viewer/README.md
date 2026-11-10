# 数字描述查看器

静态页，不打包、不需要 node_modules：three.js 已 vendor 在 `vendor/three/`（r169，MIT）。

```bash
# 在仓库根起一个静态服务（不能直接双击 index.html：ES module 和 fetch 都不支持 file://）
python3 -m http.server 8123 --bind 127.0.0.1
```

打开 <http://127.0.0.1:8123/client/viewer/index.html>

- `?robot=g1-29dof` 直接选机型（可选 `duck-hd1910`）
- `?pose=left_knee_joint:-60,left_hip_pitch_joint:30` 带一个姿态（角度制），用于截图和回归

数据来自 `tools/robot/export_viewer.py` 生成的 `data/*.json`；改了实体之后要重跑：

```bash
python3 tools/robot/export_viewer.py "contract/robot/types/*.yaml"
```

约定：实体是 Z 上（x 前 / y 左 / z 上），three 是 Y 上，`robotGroup.rotation.x = -π/2` 只是显示换算，不改实体。
「贴地」是按最低点把整机压回地面的显示辅助，不代表物理；关掉它看到的是真实 FK。

## 并联叠加层

带 `mechanisms[].coupling.kind: analytic` 的机型（`g1-29dof`、`ankle-rig-tg10`、`ankle-rig-g1`）
会在侧栏出现「并联踝（实时解算）」：A/B 与 roll/pitch 四个滑条双向联动，几何直接读实体里的
`coupling.params`，不再单独存一份。

rig 机型没有 vendored 的 STL，加载失败时会用一个 14 mm 的示意盒代替每个 link，几何点不受影响。

## 标准视角

`沿轴看 / 侧看 / 俯视` 三个按钮按**机器人自身的轴**算方向（实体是 Z-up：x 前 / y 左 / z 上），
不是拿世界坐标硬套——整机显示时绕 X 转了 -90°，直接套世界轴会得到"歪"的视角。

| 按钮 | 相机方向 | 用途 |
|---|---|---|
| 沿轴看 | 机身 +x（正前方） | 看踝的销轴 / 左右对称 |
| 侧看 | 机身 +y（左侧） | 看 pitch 方向、腿的轮廓 |
| 俯视 | 机身 +z（正上方），故意偏 6° | 看脚的左右偏差 |

`camera.up` 固定用世界 +Y（= 机身 +z）。俯视时如果相机正好落在 up 轴上，OrbitControls 的极角会退化
（万向锁，拖动时画面乱翻），所以俯视会偏 6°，不会正对着头顶。

`?view=alongX|alongY|top` 可以直接带参数打开；并排对比时左右两边一起转。

## 并排对比换机型

换机型时右半边会被**拆掉重建**（`splitReset()`）：否则会拿上一台机器人当"MuJoCo 那一边"
（比如先看天工再切 G1，就变成 G1 对比天工）。

`?selftest=switch` 可复现这个场景：加载天工 → 自动切到 G1，控制台打印切换前后的关节数与叠加层数量。

## 轨迹回放

工具栏「回放」旁边有个下拉，片源来自 `data/replay-index.json`（`{"clips":[{label, file, robot}]}`），
现在两条：

| 片源 | 是什么 |
|---|---|
| 链路验证 · 关重力，只动腿（5 s） | 默认。腿上打正弦、关重力，看机构动得对不对，不会被重力放倒 |
| 物理验证 · 有重力，会倒（10 s） | 真物理：全零位形不是站姿，鸭子会被放倒。注意它倒的时候**头会插进地里**——因为 10/15 个 link 没有碰撞体（C9 检查会报），不是渲染错 |

两条都用 `check_replay_fk.py` 验过与 MuJoCo 逐 body 一致（0.000 mm）。


工具栏「回放」读 `data/replay-duck-pd.json`（`sim/host/pd_hold.py --trajectory` 导出的轨迹），
播放/暂停 + 时间轴拖动。帧里只带「自由基座位姿 + 驱动关节角」，其余几何全部复用本页自己的 FK——
所以看到的不是另一套渲染，就是实体描述本身。回放时「贴地」自动关掉（要的是真实位姿）。

重新生成（默认就是物理验证那一条，10 s 正弦、有重力、有地面）：

```bash
export PYTHONPATH=$HOME/.local/share/ducksir/pylibs
python3 sim/host/pd_hold.py --mjcf build/mjcf/duck-hd1910.xml \
    --preset build/preset/duck-hd1910.preset.yaml \
    --seconds 10 --target sine --amp 0.2 --gravity on --floor \
    --trajectory client/viewer/data/replay-duck-pd.json
```

也可以 `?replay=./data/<别的>.json` 指定文件，或把按钮换成别的机型轨迹——机型和轨迹的关节名对不上会直接报错。

### 取景与抬高（2026-11-10 第二次修）

- `robotGroup.position` 在回放起点**必须归零**：加载时「贴地」把整组压低了 `lift`（鸭子是 -13 mm），
  而回放的真实高度已经在位姿里，不清掉就会整体偏移。
- 取景按**整条轨迹的包围盒**算，不按首帧：首帧鸭子还在 0.25 m 空中，按它取景之后一落地就掉出画面中心，
  看着像「陷进地板里」。相机同时抬高到斜上方，grid（y=0）就不会在视觉上横穿机身。

### 回放为什么改了根节点的写法

2026-11-10 第一版回放把自由基座位姿写到了 `robotGroup.position` 上，而根 link（鸭子的 `trunk_base`）本身还带着
`link.xyz = [0, 0, 0.12]`——两边都加，整机就比物理里高 12 cm，看起来像浮在地板上。现在根节点自己吃 qpos，
`robotGroup` 只负责 Z-up → Y-up 那层显示旋转。

顺手修的观感问题：回放开始时自动关「模型半透明」并重算一次视角（退出回放还原）。

### 怎么证明画面对得上物理

```bash
export PYTHONPATH=$HOME/.local/share/ducksir/pylibs
python3 tools/robot/check_replay_fk.py \
    client/viewer/data/duck-hd1910.json \
    client/viewer/data/replay-duck-pd.json \
    build/mjcf/duck-hd1910.xml
# 首/中/末三帧，逐个 body 比，最大偏差 0.000 mm → PASS
```

它把页面那套 FK 复刻成 Python，用同一帧的 qpos 让 MuJoCo 算 `xpos` 对照。
**肉眼看不出「渲染错」还是「模型本来躺成那样」时，跑这个用毫米说话。**
