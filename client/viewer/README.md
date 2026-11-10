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
