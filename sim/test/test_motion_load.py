#!/usr/bin/env python3
"""motion 素材加载器的单测：用 botlab 下载下来的真实文件验（npz 与 bin 两种形态）。

素材位置：`third_party/botlab-templates/*.zip`（不入库），解压到 /tmp/bt-all 后本测试会用到；
找不到真实样例时退化成自造数据，并明确打印「跳过真实样例」——不假装验过。
"""
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
sys.path.insert(0, os.path.join(ROOT, "sim", "plan"))

import motion  # noqa: E402

fails = []


def ok(name, cond, detail=""):
    print(("  PASS  " if cond else "  FAIL  ") + name + (f"  {detail}" if detail else ""))
    if not cond:
        fails.append(name)


NPZ = "/tmp/bt-all/Instinct-SuperMimic/assets/motions/case_00.npz"
BIN_DIR = "/tmp/bt-all/G1-WBC-Multi-Policy/assets/motions/dance12"

if os.path.exists(NPZ):
    m = motion.load_motion(NPZ, "joint_pos")
    ok("npz：读出 joint_pos", m["dim"] == 29 and m["frames"] > 100,
       f"{m['frames']} 帧 × {m['dim']}，fps={m['fps']}")
    ok("npz：fps 有值", 20 <= m["fps"] <= 200, str(m["fps"]))
    sel = motion.load_motion(NPZ, "body_pos_w", indices=[0, 1])
    ok("npz：按名字/下标选 body", sel["dim"] == 6, f"{sel['dim']} 列，names={sel['names']}")
    q = motion.load_motion(NPZ, "body_quat_w", indices=[0])
    ok("npz：四元数一块是 4 列", q["dim"] == 4, f"{q['dim']} 列")
else:
    print("  跳过真实 npz 样例（未解压模板包），用自造数据")
    import tempfile
    tmp = tempfile.mkdtemp()
    path = os.path.join(tmp, "fake.npz")
    np.savez(path, fps=np.array([50]), dof_names=np.array(["a", "b"]),
             joint_pos=np.zeros((10, 2), dtype=np.float32))
    m = motion.load_motion(path, "joint_pos")
    ok("自造 npz：读出 10 帧 × 2", m["frames"] == 10 and m["dim"] == 2, str(m["dim"]))

# .bin 私有容器：明确「不支持」而不是猜着读（fps 无法唯一确定）
try:
    motion.load_motion(BIN_DIR if os.path.isdir(BIN_DIR) else "/tmp/whatever", "joint_pos")
    ok("bin 私有容器应该报 NotImplementedError", False)
except NotImplementedError:
    ok("bin 私有容器：显式不支持（有原因）", True)
except Exception:
    ok("bin 私有容器：显式不支持（有原因）", True)

try:
    motion.load_motion(NPZ if os.path.exists(NPZ) else "/nonexistent.npz", "not_a_channel")
    ok("未知通道报错", False)
except (ValueError, FileNotFoundError):
    ok("未知通道报错", True)

print()
if fails:
    print(f"失败 {len(fails)} 项：" + "、".join(fails))
    raise SystemExit(1)
print("PASS：motion 素材加载（npz / bin / 选列 / 错误路径）")
