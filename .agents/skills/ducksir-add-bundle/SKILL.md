---
name: ducksir-add-bundle
description: 把一个 botlab 模板接成 ducksir bundle 的固定流程（资产、preset、manifest、obs 模块、runner 分支、bundle、探针、文档），含每步验收数字与已知坑。
---

# 接一个 botlab 模板

先读 [docs/feature-map/botlab-templates.md]（键位/参数区间/obs 布局/动作公式）。
**不要**从 `controller.py` 一行行读起。

## 七步

| # | 做什么 | 验收 |
|---|---|---|
| 1 | 解包模板到 `third_party/botlab-templates/unpacked/<Tpl>`；资产进 `third_party/<name>/` | 文件齐（模型/场景/preset） |
| 2 | 提取 preset（站点资源在 `low-code-*.js`）→ `build/preset/<bundle>.preset.yaml` | `actionScale/kp/kd/orders/initialQpos` 有值 |
| 3 | 写 `contract/policy/<bundle>.manifest.yaml`（obs 槽位、action mapping、rates、scene、parity） | `yaml.safe_load` 过 |
| 4 | 写 `sim/host/<name>_obs.py`（观测装配，纯函数） | 单测/对拍 step-0 差 ≤2e-5 |
| 5 | runner 分支（观测 + 动作映射 + ctrl 槽位） | 开环重放 ≤1e-4 |
| 6 | `export_artifacts.py` 出 bundle | `robot.xml` 的 nu/nq 对、网格齐 |
| 7 | 探针 + feature map + scorecard | `probe.mjs` 全 PASS |

## 必须进 CI 的三样

1. `third_party/parity/<bundle>.json` + manifest `parity` 段 → `test_parity.py`；
2. bundle 加进 `sim/test/test_g1_wbc_bundle.py` 的 `BUNDLES`（需要 EGL 的加进 `_RENDER_BUNDLES`）；
3. `docs/feature-map/botlab-templates.md` 补一行。

## 动作映射的三个坑（都踩过）

- `default + scale·action[motor_order]` —— **不能整条再重排**（差 0.16 rad）；
- PD 的 qpos 要用**原生序**（有 `actuatorJointOrder` 时），否则第 1 步 qvel 飙到 29；
- tau 写 `ctrl[actuatorJointOrder[i]]`，不是 `ctrl[i]`。

## 场景与地形

- 上游 MJCF 有 `<include>` → export 要跟进、并把 include 里的 `meshdir` 改成 `.`；
- 上游 MJCF 没有 `<actuator>` → 按契约补 motor/position；
- 纹理（`botworld_*.png`）也要打包；
- 程序化地形照 `mujocoWorker-*.js` 的 `hn()/gn()` 复刻，**size 是全长、`mn()` 里才 ×0.5**。
