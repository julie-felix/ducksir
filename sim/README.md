# sim/ — 编排与仿真运行时

这一层是「把策略放进物理里跑」的地方。边界（2026-11-10 定的，见 `docs/plans/2026-11-10-botlab-replica.md`）：

- **物理在宿主，执行核不碰物理**（D9）：MuJoCo 由这里的 Python 拉起，C++ 核只做 `obs → action`。
- **图是产物，Python DSL 是源码**：W6 起，编排写在 `sim/` 的 Python 里，编译成图由核执行。
- **报告是产物**（D17）：每次跑都出机器可读 JSON，带 digest 链（entity / mjcf / preset）。

## 现在有什么

| 路径 | 作用 |
|---|---|
| `host/pd_hold.py` | W3 的验收脚本：PD 跟踪 + 物理链路验证，出 `--report` JSON 与 `--trajectory` 轨迹（给查看器回放） |

## 怎么跑

```bash
export PYTHONPATH=$HOME/.local/share/ducksir/pylibs

# 链路验证：关重力、阶跃目标，判稳态误差
python3 sim/host/pd_hold.py --mjcf build/mjcf/duck-hd1910.xml \
    --preset build/preset/duck-hd1910.preset.yaml \
    --seconds 5 --target step --amp 0.2 --gravity off --no-floor \
    --report build/report/pd_hold_chain.json

# 物理验证：开重力、有地面、正弦目标，判不发散/有接触/不穿地
python3 sim/host/pd_hold.py --mjcf build/mjcf/duck-hd1910.xml \
    --preset build/preset/duck-hd1910.preset.yaml \
    --seconds 10 --target sine --amp 0.2 --gravity on --floor \
    --report build/report/pd_hold_physics.json
```

前置产物（`build/mjcf/*.xml`、`build/preset/*.yaml`）由 `tools/robot/mjcf_smoke.py` 一次生成，见 `docs/env.md`。

## 还没做

- 站立/行走：要策略，PD 管不了自由基座的平衡（全零位形不是站姿，实测见计划 §9）。
- 观测语义现在内联在 `pd_hold.py` 里，W6 迁到 `contract/policy/manifest`。
- 地面是脚本临时加的（`--floor` 用 MjSpec 注入平面）；真正的场景文件还没定。

回放这条链路：`pd_hold.py --trajectory client/viewer/data/replay-duck-pd.json` →
查看器工具栏「回放」。这样 W4 之后写 C++ 核时，验收除了报告里的数字，还有一个能用眼睛看的东西。

回放链路验一次（把「看着对不对」变成数字）：

```bash
python3 tools/robot/check_replay_fk.py client/viewer/data/duck-hd1910.json \
    client/viewer/data/replay-duck-pd.json build/mjcf/duck-hd1910.xml
```

两条片源的区别写在 `client/viewer/README.md`：关重力那条看机构，有重力那条会倒（物理如此，没策略）。
