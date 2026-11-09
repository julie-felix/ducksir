# contract/robot —— 数字描述

一台机器人一份 YAML。**`types/` 下的文件是导入生成的，不要手改**：改了下次重跑 importer 就没了，
而且 CI 会比对「重跑结果」和「入库文件」，对不上就报错。

```
types/<type>.yaml        base：结构真相（links / joints / actuators / frames / presets / provenance）
instances/<ID>.yaml      overlay：这台机器的增量（零位、方向、实测限位、辨识值）。鸭子还没有真机，暂缺
```

重新生成（命令与理由见 [阶段 2 执行计划](../../docs/plans/2026-11-09-robot-entity.md)）：

```bash
python3 tools/robot/importer_mjcf.py third_party/fanhao-microduck-replica/robot_walk.xml \
  -o contract/robot/types/duck-hd1910.yaml --entity-id duck-hd1910 --type-id duck-hd1910 \
  --license "Apache-2.0（双许可）" --assets-dir assets/microduck --preset walk \
  --bam-json third_party/fanhao-microduck-replica/1910_m6.json

python3 tools/robot/importer_urdf.py third_party/unitree-g1/g1_29dof_rev_1_0.urdf \
  -o contract/robot/types/g1-29dof.yaml --entity-id g1-29dof --type-id g1-29dof \
  --license BSD-3-Clause --assets-dir assets/g1

python3 tools/robot/check_entity.py "contract/robot/types/*.yaml"
```

读的时候记住三件事：

1. **单位一律 SI**（m / kg / rad / s / N / N·m），四元数一律 `wxyz`。
2. **每个数值都有出处**，在 `provenance[]` 里；同级冲突会直接让检查失败，不会替你选一个。
3. **`warnings[]` 不是噪音**，它是「这份描述哪里还不完整」的清单（缺 mesh、URDF 没有执行器参数、只导入了一个变体）。
