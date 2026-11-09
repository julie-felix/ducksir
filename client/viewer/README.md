# 数字描述查看器

静态页，不打包、不需要 node_modules：three.js 已 vendor 在 `vendor/three/`（r169，MIT）。

```bash
# 在仓库根起一个静态服务（不能直接双击 index.html：ES module 和 fetch 都不支持 file://）
python3 -m http.server 8123 --bind 127.0.0.1
```

打开 <http://127.0.0.1:8123/client/viewer/index.html>

- `?robot=g1-29dof` 直接选机型（可选 `duck-hd1910`）
- `?pose=left_knee_joint:-60,left_hip_pitch_joint:30` 带一个姿态（角度制），用于截图和回归

数据来自 `tools/robot/export_viewer.py` 生成的 `data/*.json`；改了实体之后要重跑：

```bash
python3 tools/robot/export_viewer.py "contract/robot/types/*.yaml"
```

约定：实体是 Z 上（x 前 / y 左 / z 上），three 是 Y 上，`robotGroup.rotation.x = -π/2` 只是显示换算，不改实体。
「贴地」是按最低点把整机压回地面的显示辅助，不代表物理；关掉它看到的是真实 FK。
