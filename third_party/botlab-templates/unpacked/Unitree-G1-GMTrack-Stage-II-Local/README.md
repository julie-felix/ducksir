# Unitree G1 GMTrack Stage-II（仅 FAT 预览）

> ⚠️ **本目录是内部资产的 FAT 预览**：策略权重、参考动作与机器人模型均不得进入
> `dev` / `main`。合 `main` 前必须先在 `fat` 上移除整个目录与对应 locale key
> （`git rm -r public/templates/g1-gmtrack` + 删除 `node.g1GmtrackTemplate`
> 两行）。推送本目录需用户明确授权，属于 `bl-git-hygiene` 中「fat 不直接提交」的
> 显式豁免。

GMTrack（mjlab/rsl-rl 训练的 Unitree G1 全身运动跟踪 Stage-II 策略
`backflip_model_17000`）在 BotLab 浏览器运行时的复现，主打参考动作
`seed-backflip__flip_360_005__A415`（后空翻）。

- 复现指标（vs mjlab golden，2026-10-21，botlab-python @ 46b0ead）：
  backflip clip（325 步全程）MPJPE botlab↔golden **27.97 mm**（golden 自身
  ↔reference 90.88 mm——后空翻是高动态动作，BotLab 轨迹与 mjlab 保持一致）。
- 完整验证工程（golden dump、verify_tracking、export_onnx、probe 工具）在
  `~/Project/g1-botlab-local`（本地-only，不入库）。
- 观测 ABI：单输入 [1,1728] = proprio_hist(640) | action_hist(290) |
  command_window(798)；G1 token 无 heading 6D 段
  （[v(3),w(3),g(3),q_ref(29)]）；per-joint action scale 0.25·effort/stiffness
  在 controller.py 常量里；执行器为原生 `<position>`，`PolicyController(control=)`
  直写位置目标（先按 policy 序求和再重排 motor 序）。
- 轨迹选择说明：只收 stage2 模型在 mjlab 里验证「完整跑完」的 clip
  （webplay_backflip_single）；其它 backflip（如 flip_360_001__A304 golden 293 mm）
  与 walk（stage2 弱项，golden 80 mm / BotLab 发散）均验证过不达标，故不放。
- 调试：Plot 面板可按关节名添加 qpos/qvel 曲线；「参考动作关节」组添加 `ref <joint>`
  与同名 qpos 曲线同图叠加；图层的「Reference Ghost」开关控制 X-ray 半透明参考
  机器人叠加（始终贴着真机器人显示）。
