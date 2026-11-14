# botlab 模板一览（查表用）

接新模型时先看这张表，别再从 `controller.py` 一行行读。
**这张表存在的理由**：A/D 到底是横移还是转向，我改了三次才对——真相一直写在模板的
`config/io.yaml` 里，但它没被任何表或断言管住。

## 键位与参数（以模板 `config/io.yaml` 为准，不是 botlab 全局默认）

| 模板 | 键位 | ramp | 前进区间 | 横向区间 | 转向区间 | 备注 |
|---|---|---|---|---|---|---|
| G1-Hiking-Parkour | 没写 bindings → 用全局默认：W/S 前后、A/D 横移、Q/E 转向 | 4/s | 0 … 0.5 | 0 … 0（滑杆隐藏） | −1 … 1 | 平地/楼梯，深度 32×18 |
| G1-PHP-Parkour | `target_yaw_rate: increase KeyA decrease KeyD`（**A/D 是转向**），无横向绑定 | **0（瞬时）** | 0 … 1.0 | −1 … 1（无键盘绑定） | −1 … 1 | 106×60 → 裁 98×58 → 缩 87×58 |
| G1-GMTrack | 无键位（跟踪参考动作） | — | 无指令 | — | — | obs 1728 |
| Instinct-SuperMimic | 无键位（播放动作） | — | 无指令 | — | — | obs 1990，含深度 576 |
| UniSuperMimic | 无键位（播放动作） | — | 无指令 | — | — | obs 1720 |

**botlab 全局默认（模板没写 bindings 时）**：

```
forward: KeyW  backward: KeyS
left:    KeyA  right:    KeyD     ← 横向平移（指令第 2 维）
yawLeft: KeyQ  yawRight: KeyE     ← 转向（指令第 3 维）
rampRate 默认 4/s；模板写 0 表示按键瞬时到上限
```

## 观测布局速查

| 模板 | 总维 | 组成 | 历史 |
|---|---|---|---|
| GMTrack | 1728 | proprio 640 + action 290 + 命令窗 798 | 10 帧 by-term，**最老在前 + 首帧填满** |
| Hiking-Parkour | 896 | state 768 + latent 128（深度 8 帧 stride 5） | state 8 帧，首帧填满 |
| PHP-Parkour | 140 | grav 3 + angvel 3 + q 29 + qd 29 + action 29 + one-hot 11 + pad 4 + latent 32 | 无历史；latent `delay(7)` |
| Instinct | 1990 | jp/jv/pos/rot 670 + depth 576 + hist 744 | 历史在 runner 里拼 |
| UniSuperMimic | 1720 | 99×10 历史 + 10×73 goals | t=(k−1)·0.02 |

## 动作映射（三个都踩过坑）

| 模板 | 目标公式 | 坑 |
|---|---|---|
| GMTrack | `q_ref(t+0.02) + scale·action`，再 `[motor_order]` | q/qd 用 **qpos 序**（不是契约 driven 序） |
| Hiking-Parkour | `default + scale·action[motor_order]` | **不能整条再重排**（差 0.16 rad） |
| PHP-Parkour | `default + (scale·action)[motor_order]` | PD 用**原生 qpos 序**；tau 写 `ctrl[actuatorJointOrder[i]]` |

## 录制对拍（每个模板都这么干）

```bash
# 1. 在 /tmp/<name>-run 里跑模板，hook 住 ort.run + mj_step
PYTHONPATH=/tmp/botlab-py/src:$HOME/.local/share/ducksir/pylibs MUJOCO_GL=egl \
  BOTLAB_STEPS=40 python3 -u -c "import patch_rec; from botlab._cli import main; \
  import sys; sys.argv=['botlab','run','--headless','/tmp/<name>-run']; main()"

# 2. 切 3 步样本（进 CI）
python3 tools/dev/make_parity_sample.py --bundle g1-php --format php \
  --trace /tmp/php.jsonl --obs-dim 140 --out third_party/parity/g1-php.json

# 3. 跑对拍
python3 sim/test/test_parity.py
```

**注意**：replay 的 `m.opt.timestep` 必须设成 `1/physics_hz`（0.005），
用模型默认的 0.002 会得到 0.12 的假误差——这一条坑了我一轮。

## 站点侧资源在哪

| 资源 | 位置 |
|---|---|
| 模板源码 | `https://botlab.enkeebot.com/templates/<slug>/…`（hiking：`g1-hiking-parkour`） |
| 内置 preset（机器人/相机/地形） | `assets/low-code-<hash>.js` 里的 `"<name>":{kind:…,config:…}` |
| 程序化地形生成器 | `assets/mujocoWorker-<hash>.js` 的 `hn()`（楼梯）/ `gn()`（坡与箱） |
| 机器人 MJCF + mesh | `https://botlab.enkeebot.com/models/g1/instinctmj/…` |

**记住**：`hn()` 传出的 size 是**全长**，最后统一在 `mn()` 里 `×0.5` 变成 MJCF 半长。

## 录制源限制（2026-11-14）

| 模板 | 能否录制 | 卡在哪 |
|---|---|---|
| GMTrack / Hiking / PHP | ✅ | — |
| WBC | ❌ | 模板用 `robot.commands(mappings=...)`，0.4.1 没有 |
| Instinct | ❌ | 场景无执行器，依赖浏览器 `autoActuators` 生成 |
| AMP / MJAMP | ❌ | 没有随模板发布的资产包 |
| UniSuperMimic | ⚠️ | 模板在，但要确认深/动作资产 |

拿不到录制的，用**金标回归**替代：`tools/dev/make_golden.py` 生成轨迹、
`sim/test/test_golden_bundles.py` 重跑对比（差 >1e-6 报警）。
