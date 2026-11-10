#!/usr/bin/env python3
"""MJCF 冒烟：三台机型导出 → MuJoCo 加载 → 打印一篇表。

    PYTHONPATH=$HOME/.local/share/ducksir/pylibs python3 tools/robot/mjcf_smoke.py

判据（W1 验收）：
1. 每台机型 `MjModel.from_xml_path` 成功；
2. 加载期 **0 条 MuJoCo 警告**（mju_warning 走 stderr，这里 dup2 抓下来数条数）；
3. 总质量与 canonical 里 `mass` 求和一致（1e-6）。

产物写进 `build/`（已 gitignore）：`build/mjcf/<robot>.xml` + `build/meshes/<robot>/`。
mesh 用 STL —— MuJoCo 不认 glb；鸭子与 G1 用 `third_party` 里的原始 STL，
天工只有 glb，先由 `tools/robot/glb2stl.py` 转一份（含顶点聚类简化，STL 解码器上限 20 万面）。

没有 mujoco 就跳过（exit 0），因为 CI runner 不装它。
"""

from __future__ import annotations

import os
import subprocess
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
sys.path.insert(0, HERE)

import entity as E  # noqa: E402

ROBOTS = [
    # (机型, 实体文件, mesh 目录, mesh 扩展名)
    ("duck-hd1910", "contract/robot/types/duck-hd1910.yaml", "third_party/fanhao-microduck-replica/assets", "stl"),
    ("g1-29dof", "contract/robot/types/g1-29dof.yaml", "third_party/unitree-g1/meshes", "STL"),
    ("tienkung-lite", "contract/robot/types/tienkung-lite.yaml", "build/meshes/tienkung", "stl"),
]


def ensure_meshes(name: str, mesh_dir: str) -> None:
    """天工的 mesh 只有 glb，缺了就转一份。"""
    if os.path.isdir(mesh_dir) and any(f.lower().endswith(".stl") for f in os.listdir(mesh_dir)):
        return
    if name == "tienkung-lite":
        src = os.path.join(ROOT, "assets/tienkung")
        print(f"  · 生成 {mesh_dir}（glb → stl）")
        subprocess.run([sys.executable, os.path.join(HERE, "glb2stl.py"), src, mesh_dir, "-q"], check=True)
        return
    raise SystemExit(f"{name}: 缺 mesh 目录 {mesh_dir}")


def export(robot: str, entity: str, mesh_dir: str, ext: str, out: str) -> None:
    rel = os.path.relpath(os.path.join(ROOT, mesh_dir), os.path.dirname(out))
    subprocess.run(
        [sys.executable, os.path.join(HERE, "export_mjcf.py"), os.path.join(ROOT, entity),
         "-o", out, "--mesh-dir", rel, "--mesh-ext", ext],
        check=True, stdout=subprocess.DEVNULL,
    )


def main() -> int:
    try:
        import mujoco
    except ImportError:
        print("跳过：没装 mujoco（见 docs/env.md 的 PYTHONPATH）")
        return 0

    build = os.path.join(ROOT, "build")
    mjcf_dir = os.path.join(build, "mjcf")
    os.makedirs(mjcf_dir, exist_ok=True)

    rows, bad = [], 0
    for name, entity, mesh_dir, ext in ROBOTS:
        ensure_meshes(name, os.path.join(ROOT, mesh_dir))
        out = os.path.join(mjcf_dir, name + ".xml")
        export(name, entity, mesh_dir, ext, out)

        doc = E.read_yaml(os.path.join(ROOT, entity))
        want = sum(l.get("mass") or 0.0 for l in doc["links"])

        # 抓 stderr 数警告
        err = tempfile.TemporaryFile()
        saved = os.dup(2)
        os.dup2(err.fileno(), 2)
        try:
            m = mujoco.MjModel.from_xml_path(out)
            got_mass = float(m.body_mass.sum())
            nbody, njnt, nu = m.nbody, m.njnt, m.nu
            ok, note = True, ""
        except Exception as e:  # noqa: BLE001
            ok, note = False, str(e).splitlines()[0][:80]
            got_mass = nbody = njnt = nu = 0
        finally:
            os.dup2(saved, 2)
            os.close(saved)
        err.seek(0)
        warns = [l for l in err.read().decode("utf-8", "replace").splitlines() if l.strip()]
        err.close()

        mass_ok = ok and abs(got_mass - want) < 1e-6
        if not ok or not mass_ok or warns:
            bad += 1
        rows.append((name, ok, nbody, njnt, nu, f"{got_mass:.8f}", f"{want:.8f}", len(warns), note))

    print(f"{'机型':<15}{'load':<6}{'body':>5}{'jnt':>5}{'act':>5}{'质量(kg)':>14}{'canonical':>14}{'warn':>5}  备注")
    for r in rows:
        print(f"{r[0]:<15}{'OK' if r[1] else 'FAIL':<6}{r[2]:>5}{r[3]:>5}{r[4]:>5}{r[5]:>14}{r[6]:>14}{r[7]:>5}  {r[8]}")
    print(f"\n{len(rows) - bad}/{len(rows)} 通过" + ("" if not bad else "  ← 有失败"))
    return 1 if bad else 0


if __name__ == "__main__":
    raise SystemExit(main())
