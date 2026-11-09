# 解算器 vs MuJoCo 对拍报告

命令：`PYTHONPATH=/tmp/pylibs python3 tools/parallel/compare.py <entity.yaml> <rig.xml> --steps 4000`

| 机型 | pitch 最大误差 | roll 最大误差 | 综合 | 牛顿不收敛 | 判定 |
|---|---|---|---|---|---|
| tg10 | 5.566e-06 rad | 6.309e-05 rad | 6.309e-05 rad | 0 | PASS |
| g1 | 2.377e-03 rad | 8.568e-04 rad | 2.377e-03 rad | 0 | FAIL |

阈值来自计划 Q9：PR 误差 ≤ 1e-4 rad。tg10 通过；g1 未通过，原因见计划文档 §3.8。

## MuJoCo 对照：并排对比

点右上角「**并排对比**」（或 URL 加 `&split=1`）：

- **左**：机器人模型 + 绿色连杆 = 我们的解算；
- **右**：同一台机器人（半透明，18% 不透明）+ 橙色连杆 = MuJoCo 的结果；
- 拖 A/B 或 roll/pitch 时**两边同时**更新，右侧标题实时显示两边角度与差值（典型 0.3–0.8 mrad）；
- 手机上是上下排布（不是左右），侧栏移到下方，viewport < 760px 自动切换；
- 头部「同步视角」默认勾选：转一边另一边跟着转，取景完全一致（`?selftest=1` 会打印两边相机参数，
  正常情况下完全相同）。

MuJoCo 侧用 **41×41 的 (θA,θB) → (roll,pitch) 查表** 双线性插值——浏览器跑不了 MuJoCo。
表由 `tools/parallel/export_mujoco_grid.py` 生成：rig 的 XML 里执行器是**位置伺服**，
直接把目标角写进 `ctrl`、跑 600 步收敛，读 roll/pitch。

```bash
PYTHONPATH=/tmp/pylibs python3 tools/parallel/export_mujoco_grid.py \
    contract/robot/types/ankle-rig-g1.yaml /tmp/rig_g1_nomesh.xml \
    --grid 41 --span 0.5 -o client/viewer/data/mujoco-grid-g1.json
```

与解算器的差：g1 rig 最大 1.6 mrad、tg10 rig 最大 3.0 mrad——量级和前面查的软约束偏差一致。
表挂在 spec 的 `coupling.params.mujoco_grid` 上（G1 两个踝、天工 Lite 两个踝、两台 rig 各自一份）。

## 天工 Lite（官方 URDF）

```
https://100.110.238.101:3080/ducksir-static/client/viewer/index.html?robot=tienkung-lite&focus=ankle_pitch_l_joint&parL=20,12
```

- 机型来自官方仓库 [Open-X-Humanoid/TienKung_URDF](https://github.com/Open-X-Humanoid/TienKung_URDF) 的 `lite_urdf_publish`（OpenAtom OHL 1.0）；
- 22 link / 21 joint / 20 驱动 / 42.52 kg；踝在 URDF 里是串联的 `ankle_pitch → ankle_roll`；
- 并联结构补在 `contract/robot/specs/tienkung-lite-parallel.yaml`：左右踝 analytic，连杆几何沿用
  讲者的 TC10-00 装配 rig —— 依据是 rig 的 roll 相对 pitch 位移 `(0,0,0)` 与 Lite 的 `ankle_roll_*_joint` origin 一致；
- `ab_unresolved: true`：Lite URDF 里没有电机关节，A/B 槽位尚未确定，所以侧栏 AB 读数显示「-」。
