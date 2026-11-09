# 解算器 vs MuJoCo 对拍报告

命令：`PYTHONPATH=/tmp/pylibs python3 tools/parallel/compare.py <entity.yaml> <rig.xml> --steps 4000`

| 机型 | pitch 最大误差 | roll 最大误差 | 综合 | 牛顿不收敛 | 判定 |
|---|---|---|---|---|---|
| tg10 | 5.566e-06 rad | 6.309e-05 rad | 6.309e-05 rad | 0 | PASS |
| g1 | 2.377e-03 rad | 8.568e-04 rad | 2.377e-03 rad | 0 | FAIL |

阈值来自计划 Q9：PR 误差 ≤ 1e-4 rad。tg10 通过；g1 未通过，原因见计划文档 §3.8。

## MuJoCo 对照（在机器人页面里）

对照不再单独开页，直接就在查看器里：侧栏每个并联机构下面有「**MuJoCo 对照（400 帧）**」。

- 勾上之后**模型按 MuJoCo 的位姿动**，叠加层（推杆/销/平台）画的是**我们的解**；
- 时间轴、播放/暂停都在那里，读数给出这一帧两边的 roll/pitch 与差值（典型 0.5 mrad）；
- 四个滑条会同步显示我们的解，方便对照；
- URL 参数 `?trace=120` 可以直接钉在某一帧。

轨迹数据由 `tools/parallel/export_viz.py` 生成（同一串电机角 θ(t) 喂两边）：

```bash
PYTHONPATH=/tmp/pylibs python3 tools/parallel/export_viz.py \
    contract/robot/types/ankle-rig-tg10.yaml /tmp/rig_tg10_nomesh.xml \
    -o client/viewer/data/parallel-tg10.json
```

数据挂在 spec 的 `coupling.params.mujoco_trace` 上：G1 两个踝用 `parallel-g1.json`，
天工 Lite 两个踝用 `parallel-tg10.json`（几何来自 rig，标 assumed）。

## 实时叠加层（在 G1 模型上拖 A/B）

主查看器打开 G1 后，侧栏会出现「并联踝（实时解算，拖 A/B 看脚怎么动）」：

```
https://100.110.238.101:3080/ducksir-static/client/viewer/index.html?robot=g1-29dof&focus=left_ankle_pitch_joint&par=18,12
```

- **左右踝各一组**（左绿、右橙），推杆/曲柄/销/平台画在各自的 ankle pitch 关节坐标系里，和模型同框；
- **四个滑条双向联动**：拖 A/B 电机 → 正解出 roll/pitch；拖 roll/pitch → 逆解回 A/B。浏览器里跑的是和 Python 同一套解算；
- 头部「模型半透明」开关把机体压到 22% 不透明，方便看里面的连杆；
- 坐标系对齐的依据：rig 的 `roll_link` 位移 `(0,0,-0.017558)` 与 G1 URDF 的 `left_ankle_roll_joint` **完全一致**，
  所以 rig 的踝就是按 G1 踝的坐标系装的。

几何数据由 `tools/parallel/export_overlay.py` 导出（含一份 MuJoCo 查表，暂未用于交互）。

## 交互细节

- 左侧模型半透明后能看到踝内的两根推杆；绿灯=左踝、橙灯=右踝。
- 四个滑条任意拖，另一边自动跟着算（`parFw` / `parIk`，浮点往返误差 0）。
- 右侧按矢状面镜像；写关节时 roll 取反（`left_ankle_roll_joint = -x.roll`），这是镜像机构的约定，
  与真机符号是否一致需要实物确认（未定）。
- URL 参数：`?robot=g1-29dof&focus=left_ankle_pitch_joint&parL=20,10&parR=-20,-10`。

## 天工（TG10-00）也接进来了

`ankle-rig-tg10` 现在是查看器里的一台机型（下拉里叫 `TG10-00-ankle`），
并联机构 `tg10_ankle` 是 analytic，几何来自 rig 自己的 XML：

```
https://100.110.238.101:3080/ducksir-static/client/viewer/index.html?robot=ankle-rig-tg10&par=25,18
```

- 拖 A/B（或不拖，直接拖 roll/pitch）看两根推杆怎么把脚带起来；
- `passive_pr: true`：rig 里踝的两个关节是被动的，真正驱动的是两个电机，闭链靠 `<connect>` 闭合；
- rig 的 STL 不在仓库里，所以模型用**示意盒**（每个 link 一个 14 mm 灰盒）表示，几何点仍然准确；
- 侧栏会标「N 项假设」，点开 spec 能看到具体是哪几条。

两个 rig 的 spec 在 `contract/robot/specs/ankle-rig-parallel.yaml`，按机型 id 分组。

## 天工 Lite（官方 URDF）

```
https://100.110.238.101:3080/ducksir-static/client/viewer/index.html?robot=tienkung-lite&focus=ankle_pitch_l_joint&parL=20,12
```

- 机型来自官方仓库 [Open-X-Humanoid/TienKung_URDF](https://github.com/Open-X-Humanoid/TienKung_URDF) 的 `lite_urdf_publish`（OpenAtom OHL 1.0）；
- 22 link / 21 joint / 20 驱动 / 42.52 kg；踝在 URDF 里是串联的 `ankle_pitch → ankle_roll`；
- 并联结构补在 `contract/robot/specs/tienkung-lite-parallel.yaml`：左右踝 analytic，连杆几何沿用
  讲者的 TC10-00 装配 rig —— 依据是 rig 的 roll 相对 pitch 位移 `(0,0,0)` 与 Lite 的 `ankle_roll_*_joint` origin 一致；
- `ab_unresolved: true`：Lite URDF 里没有电机关节，A/B 槽位尚未确定，所以侧栏 AB 读数显示「-」。
