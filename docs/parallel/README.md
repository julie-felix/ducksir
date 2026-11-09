# 解算器 vs MuJoCo 对拍报告

命令：`PYTHONPATH=/tmp/pylibs python3 tools/parallel/compare.py <entity.yaml> <rig.xml> --steps 4000`

| 机型 | pitch 最大误差 | roll 最大误差 | 综合 | 牛顿不收敛 | 判定 |
|---|---|---|---|---|---|
| tg10 | 5.566e-06 rad | 6.309e-05 rad | 6.309e-05 rad | 0 | PASS |
| g1 | 2.377e-03 rad | 8.568e-04 rad | 2.377e-03 rad | 0 | FAIL |

阈值来自计划 Q9：PR 误差 ≤ 1e-4 rad。tg10 通过；g1 未通过，原因见计划文档 §3.8。

## 可视化（左边解算 / 右边 MuJoCo）

打开：

```
https://100.110.238.101:3080/ducksir-static/client/viewer/parallel.html
```

同一串电机角 θ(t) 同时喂两边：左边平台的位姿由我们的解算给出，右边直接用 MuJoCo 报的位姿。
两边的推杆销 B、平台点 C 都画出来，所以能直接看见差别在哪。右上角实时显示两边的 roll/pitch 与**杆长误差**
（我们 0 nm，MuJoCo 是 µm 级），下方是两条曲线的对比。

数据由 `tools/parallel/export_viz.py` 生成：

```bash
PYTHONPATH=/tmp/pylibs python3 tools/parallel/export_viz.py \
    contract/robot/types/ankle-rig-tg10.yaml /tmp/rig_tg10_nomesh.xml \
    -o client/viewer/data/parallel-tg10.json
```

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
