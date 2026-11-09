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
