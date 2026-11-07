# DuckSIR

一只自己搭的机器鸭，和它背后那条从**硬件**到**强化学习运控**再到**真机部署**的完整流水线。

硬件从 [MicroDuck](https://github.com/pollen-robotics/microduck) 的**开源版本**出发手搓；客户端做成一组 [DeepSeek Harness](https://github.com/deepseek-ai) 插件；训练、端侧、客户端三块代码放在这一个 monorepo 里。

> 当前状态：**v0 只有文档**。代码、BOM、标定记录都还没进来。

---

## 0. 复刻对象：开源版本，不是商用原机

这条先定死，因为后面每个决定都受它约束：

- **复刻对象是官方公开的开源设计**：仓库里的软件（Apache-2.0）、公开的硬件设计、公开的固件与标定流程。
- **不是**照着 $399 商用整机做逆向或"尽量像"。商用整机只当对照——用来判断"我们这一版跟原厂差在哪"，不拿来当参考设计。
- 直接后果：只做开源设计里写明的部件；开源没给的部分（结构件细节、某些板卡）由我们自己补，并且**补的部分要单独记录**，不能假装它是"官方设计"。
- 另一条后果：整个项目对外只能声称"复刻开源版本"，不能声称"复刻 MicroDuck 整机"。

## 1. 我们要拿到什么

不是一个能跑 demo 的机器人，而是一条**可重复的运控工程流水线**：

1. **整机能走**：手搓的机器鸭在真实地面上连续行走 10 分钟不摔。
2. **能跳舞**：能编排的多段动作，不是单条策略碰运气。
3. **有 SOP**：从台架联调、标定、训练到 sim2real 的每一步都有可执行、有验收标准、有失败模式记录的文档。
4. **有客户端**：通过 DSH 插件完成「看状态 / 查策略 / 跑预检 / 发起训练」这一类审阅与调度动作。

一条隐含约束：**DSH 客户端不进实时控制回路**。50 Hz 以上的策略环跑在端侧，客户端只做控制面（指挥、审阅、发起任务、看结果）。

## 2. 三平面 + 一条契约

```
        ┌──────────────────────────────┐
        │  client/   DSH 插件（TypeScript）│  人在这里：看、查、发起、审阅
        └───────────────┬──────────────┘
                        │ 契约（关节顺序 / 观测动作定义 / 参考动作 / 策略清单 / 遥测）
        ┌───────────────┴──────────────┐
        │  training/  云端或工作站（Python）│  重定向 → 跟踪训练 → 评估 → 导出 ONNX
        └───────────────┬──────────────┘
                        │ 策略产物 = ONNX + manifest（含 obs/action 语义、关节序、训练配置）
        ┌───────────────┴──────────────┐
        │  runtime/   端侧（SBC 上）      │  DXL 总线、IMU、安全层、推理、50 Hz 环
        └──────────────────────────────┘
```

三个平面各写各的代码，唯一共享的是 **contract**：一份机器可读的契约，回答「第 7 个关节叫什么、动作向量第几维是它、零位多少、限位多少、策略要什么观测、参考动作长什么样」。

不共享契约的项目，最后一定会死在「仿真里第 7 维和真机第 7 维不是同一个关节」上。所以契约先行，代码后写。

## 3. 仓库布局（计划）

```
ducksir/
├── docs/          架构、里程碑、决策记录、SOP 库
├── hardware/      BOM、结构件来源、板卡、标定记录（标定 = 带 scope 的 YAML）
├── training/      重定向、跟踪训练、评估、ONNX 导出
├── runtime/       端侧：总线抽象、安全层、策略推理、日志（MCAP）
├── client/        DSH 插件（TypeScript），调 contract 做审阅与调度
├── contract/      三平面共享契约：关节表、schema、policy manifest
└── tools/         校验、标定换算、烧录、批处理脚本
```

目录名和包管理器（pnpm workspace + Python 包）在写第一行代码前会再过一遍，不排除调整。

## 4. 硬件路线：还没定，先摆选项

MicroDuck 的门槛从来不是舵机或打印件，而是 **IMU 挂在哪块板上、跟哪版固件、哪版标定配**。四条路线：

| 路线 | 做法 | 大致成本 | 对我们的意义 |
|---|---|---|---|
| 官方整机 | 直接买 $399 | ≈¥2,850 | 只当对照机，不算复刻对象 |
| kissqy 复刻 | 自制 HAT/IMU/HUB + 飞特 HD1910 + Radxa ZERO 3W，镜像+中文中控开箱能走 | ≈¥2,937 | 最省心的全流程中文复刻 |
| fanhao375 复刻 | 飞特 HD-1910 整机已实机验证，含 CAD/BOM/装配教程/训练平台 | 待核 | 结构件与装配资料更完整，值得优先评估 |
| 帆哥 / jyg9 | OpenRB-150 绕开 HAT；或自制 imu_to_dxl 转接板 | >¥4,000 / 板级 | 学原理用，工程上多一层风险 |

**待定**：整机走 kissqy 还是 fanhao375 路线；舵机用原厂 XL330 还是飞特 HD1910（两者 ID/零位/协议不同，教程不能混）。这个决定直接影响 BOM、打印件安装位、标定与镜像，选错等于整条链重来。

已确认的硬事实（写进文档前逐条核过）：

- 开源版本：15 个 Dynamixel（行走 14 + 可选嘴部 1），25 cm / 800 g，摄像头 + LiDAR + 两个 IMU，50 Hz 策略环，7 个出厂策略。
- 官方软件栈是 **Rust**（`duck-control` / `robotd` / runtime），训练仓 `microduck_rl` 用 mjlab（MuJoCo Warp，Apache-2.0）。客户端与端侧接的是这套 runtime，不是另起一套 Python 控制环。
- HAT 本身换过件（IMU 型号与 I²C 地址随批次变），所以复刻必须自制板、换总线架构或改固件。
- 板、固件、打印件安装位、标定四者同版本，换一版就废。

## 5. 训练：方向未定

现在只列候选，不做决定。等硬件和台架稳定了再定，避免先锁死再返工。

重定向（把人类/视频/动捕动作变成机器人关节轨迹）至少有三条路可试：

- **运动学重定向**（IK 类，如 GMR 思路）：最快出参考轨迹，代价是脚滑、穿地这类物理不一致会传下去。
- **优化式重定向**（多接触全身轨迹优化，如 [KDMR](https://arxiv.org/abs/2603.09956)）：参考天然物理可行，代价是每个新动作都要重解，且代码未必开源。
- **学习式重定向**：监督学映射、RL 学 retargeter、或端到端不显式重定向，三条子路线各有取舍。

跟踪训练同理，至少两条路：mjlab PPO（官方栈，确定性高）与 [SuperMimic](https://arxiv.org/abs/2610.28480) 式可微世界模型（省训练成本）。两者不冲突，先跑哪条看算力和时间。

未定事项记在这，定了再写进仓库：参考动作从哪来（自采/视频/公开数据，注意 LAFAN1 是 CC BY-NC-ND、SMPL-X 非商用）、重定向与跟踪谁先做、算力预算。

## 6. SOP：这个项目的核心资产

每份 SOP 固定六段：**前置条件 / 步骤 / 验收标准 / 失败模式 / 相关工件（含 hash 与版本）/ 变更记录**。

现在能列出的候选（占位，等实机再写）：

- S-01 台架联调：单关节扫描 → 零位与限位核对 → IMU 朝向与噪声
- S-02 标定：把「我以为的」写成 YAML，再从真机回读校验
- S-03 训练作业：manifest（镜像、种子、并行数、评估协议）与产物归档
- S-04 sim2real 上线前检查：关节序、观测归一化、延迟、安全限幅
- S-05 摔倒与恢复：日志（MCAP）采集、归因、复现
- S-06 重定向验证：脚滑/穿透/接触相位/节奏检查（等定了重定向方案再细化）

## 7. 里程碑

| 阶段 | 交付 | 验收 |
|---|---|---|
| M0 硬件到位 | BOM 冻结、装配、单关节+IMU 联调 | 15 个关节逐个可控，IMU 朝向正确 |
| M1 仿真基线 | 官方 RL 环境跑通，站立+行走 | 仿真里连续走 60 s 不摔 |
| M2 真机行走 | sim2real 第一版 | 真机连续走 10 分钟 |
| M3 舞蹈 | 编舞 + 跟踪 | 一套 ≥30 s 动作，可重复触发 |
| M4 闭环 | DSH 客户端 + 训练流水线 + SOP 库 | 从客户端发起一次训练到真机验证全程可追溯 |

## 8. 接下来先做的事

1. 定硬件路线（kissqy vs fanhao375），出冻结 BOM；先核清开源版本到底给了哪些结构件、哪些要自己补。
2. 写 `contract/`：关节表 + 观测/动作 schema + policy manifest。
3. 在 DSH 里起一个最小客户端：能读契约、能查策略清单，先不碰控制。
4. 打通「官方训练仓库 → 导出 → 端侧加载」最小闭环，训练方案等这一步跑通再定。

## 9. 参考

- [pollen-robotics/microduck](https://github.com/pollen-robotics/microduck) — 官方开源仓库（Rust runtime、deploy、docs），Apache-2.0
- [pollen-robotics/microduck_rl](https://github.com/pollen-robotics/microduck_rl) — 官方 RL 环境（mjlab），Apache-2.0
- [kissqy/Microduck-kissqy](https://github.com/kissqy/Microduck-kissqy) — 飞特 HD1910 中文全流程复刻
- [fanhao375/microduck-replica](https://github.com/fanhao375/microduck-replica) — HD-1910 实机验证复刻（CAD/BOM/教程）
- [AI-FanGe/Microduck-build-tutorial](https://github.com/AI-FanGe/Microduck-build-tutorial) — OpenRB-150 路线保姆教程
- [KDMR](https://arxiv.org/abs/2603.09956) — 多接触全身轨迹优化做重定向
- [Retargeting Matters](https://arxiv.org/pdf/2510.02252) — 重定向质量决定跟踪上限
- [RobotDancing](https://www.ieee-ras.org/news/robotdancing-residual-action-reinforcement-learning-enables-robust-long-horizon-humanoid-motion-tracking/) — residual-action RL 长时程跟踪
- [SuperMimic](https://arxiv.org/abs/2610.28480) — 可微世界模型替代仿真器
- [ExBody2](https://arxiv.org/abs/2412.13196) — 表达性全身控制，按关键点跟踪

## License

待定（官方硬件/软件为 Apache-2.0；本仓库许可在代码落地前补）。
