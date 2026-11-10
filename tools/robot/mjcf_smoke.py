#!/usr/bin/env python3
"""MJCF + preset 冒烟：导出 → 加载 → 校验，三台机型。

    PYTHONPATH=$HOME/.local/share/ducksir/pylibs python3 tools/robot/mjcf_smoke.py

判据（W1 + W2 验收）：
1. 每台机型 `MjModel.from_xml_path` 成功，且加载期 **0 条 MuJoCo 警告**
   （mju_warning 走 stderr，这里 dup2 抓下来数条数）；
2. 总质量与 canonical 里 `mass` 求和一致（1e-6）；
3. 生成的 preset 与加载后的模型对得上：
   `actionDimension == nu`、`actuatorJointOrder` 逐条指向正确的关节（ctrl 序 ↔ 驱动序双射）、
   `jointQposOffset` 与实际 `jnt_qposadr` 一致。

产物进 `build/`（已 gitignore）：`build/mjcf/<robot>.xml`、`build/meshes/<robot>/`、`build/preset/<robot>.preset.yaml`。
没有 mujoco 就跳过（exit 0）——CI runner 不装它，而 preset 的 qpos 偏移要靠真模型核。
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
    if os.path.isdir(mesh_dir) and any(f.lower().endswith(".stl") for f in os.listdir(mesh_dir)):
        return
    if name == "tienkung-lite":
        print(f"  · 生成 {mesh_dir}（glb → stl）")
        subprocess.run([sys.executable, os.path.join(HERE, "glb2stl.py"),
                        os.path.join(ROOT, "assets/tienkung"), mesh_dir, "-q"], check=True)
        return
    raise SystemExit(f"{name}: 缺 mesh 目录 {mesh_dir}")


def export(entity: str, mesh_dir: str, ext: str, out: str) -> None:
    rel = os.path.relpath(os.path.join(ROOT, mesh_dir), os.path.dirname(out))
    subprocess.run([sys.executable, os.path.join(HERE, "export_mjcf.py"), os.path.join(ROOT, entity),
                    "-o", out, "--mesh-dir", rel, "--mesh-ext", ext],
                   check=True, stdout=subprocess.DEVNULL)


def export_collision_all(entity: str, mesh_dir: str, ext: str, out: str) -> str:
    """有碰撞变体就再出一份 -all.xml，返回状态说明（没有变体就返回空串）。"""
    import os as _os
    ident = _os.path.basename(entity).rsplit(".", 1)[0]
    variant = _os.path.join(ROOT, "contract/robot/variants", f"{ident}.all.yaml")
    if not _os.path.exists(variant):
        return ""
    rel = _os.path.relpath(_os.path.join(ROOT, mesh_dir), _os.path.dirname(out))
    subprocess.run([sys.executable, _os.path.join(HERE, "export_mjcf.py"), _os.path.join(ROOT, entity),
                    "-o", out, "--mesh-dir", rel, "--mesh-ext", ext,
                    "--collision-set", "all", "--collision-variant", variant],
                   check=True, stdout=subprocess.DEVNULL)
    return " +all"


def export_preset(entity: str, mjcf: str, out: str) -> None:
    subprocess.run([sys.executable, os.path.join(HERE, "export_preset.py"), os.path.join(ROOT, entity),
                    "--mjcf", os.path.join(ROOT, mjcf), "-o", out],
                   check=True, stdout=subprocess.DEVNULL)


def check_preset(mujoco, m, preset_path: str, driven: list[str]) -> tuple[str, str]:
    """返回 (状态, 说明)。状态：OK / BLOCKED（canonical 没执行器参数）/ DIFF。"""
    import yaml

    with open(preset_path, encoding="utf-8") as f:
        p = yaml.safe_load(f)
    if m.nu == 0:
        # 没有执行器参数就没法生成可用的 preset —— 这是 W2–W3 的已知洞，不是导出错
        return "BLOCKED", f"nu=0：canonical 没有 actuators（{len(driven)} 个驱动关节待补）"
    if p["actionDimension"] != m.nu:
        return "DIFF", f"actionDimension={p['actionDimension']} 但 nu={m.nu}"
    order = p["actuatorJointOrder"]
    if len(order) != m.nu:
        return "DIFF", f"actuatorJointOrder 长度 {len(order)} != nu {m.nu}"
    for i in range(m.nu):
        jid = int(m.actuator_trnid[i][0])
        name = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_JOINT, jid)
        want = driven[order[i]]
        if name != want:
            return "DIFF", f"第 {i} 个 actuator 指到 {name}，preset 说是 {want}"
        if int(m.jnt_qposadr[jid]) != int(p["jointQposOffset"][i]):
            return "DIFF", f"{name} 的 qpos 偏移实际 {int(m.jnt_qposadr[jid])} != preset {p['jointQposOffset'][i]}"
    return "OK", ""


def main() -> int:
    try:
        import mujoco
    except ImportError:
        print("跳过：没装 mujoco（见 docs/env.md 的 PYTHONPATH）")
        return 0

    build = os.path.join(ROOT, "build")
    mjcf_dir, preset_dir = os.path.join(build, "mjcf"), os.path.join(build, "preset")
    os.makedirs(mjcf_dir, exist_ok=True)
    os.makedirs(preset_dir, exist_ok=True)

    rows, bad = [], 0
    for name, entity, mesh_dir, ext in ROBOTS:
        ensure_meshes(name, os.path.join(ROOT, mesh_dir))
        out = os.path.join(mjcf_dir, name + ".xml")
        export(entity, mesh_dir, ext, out)
        pre = os.path.join(preset_dir, name + ".preset.yaml")
        export_preset(entity, os.path.relpath(out, ROOT), pre)
        all_note = export_collision_all(entity, mesh_dir, ext,
                                        os.path.join(mjcf_dir, name + "-all.xml"))

        doc = E.read_yaml(os.path.join(ROOT, entity))
        want = sum(l.get("mass") or 0.0 for l in doc["links"])
        driven = [j["name"] for j in doc["joints"] if j.get("actuated")]

        err = tempfile.TemporaryFile()
        saved = os.dup(2)
        os.dup2(err.fileno(), 2)
        try:
            m = mujoco.MjModel.from_xml_path(out)
            got_mass = float(m.body_mass.sum())
            nbody, njnt, nu = m.nbody, m.njnt, m.nu
            ok = True
            preset_state, note = check_preset(mujoco, m, pre, driven)
        except Exception as e:  # noqa: BLE001
            ok, note = False, str(e).splitlines()[0][:70]
            preset_state = "DIFF"
            got_mass = nbody = njnt = nu = 0
        finally:
            os.dup2(saved, 2)
            os.close(saved)
        err.seek(0)
        warns = [ln for ln in err.read().decode("utf-8", "replace").splitlines() if ln.strip()]
        err.close()

        mass_ok = ok and abs(got_mass - want) < 1e-6
        if not (ok and mass_ok and preset_state != "DIFF" and not warns):
            bad += 1
        rows.append((name, ok, nbody, njnt, nu, f"{got_mass:.8f}", preset_state, len(warns),
                     (note + all_note).strip()))

    print(f"{'机型':<15}{'load':<6}{'body':>5}{'jnt':>5}{'act':>5}{'质量(kg)':>14}{'preset':>8}{'warn':>5}  备注")
    for r in rows:
        print(f"{r[0]:<15}{'OK' if r[1] else 'FAIL':<6}{r[2]:>5}{r[3]:>5}{r[4]:>5}{r[5]:>14}{r[6]:>8}{r[7]:>5}  {r[8]}")
    print(f"\n{len(rows) - bad}/{len(rows)} 通过" + ("" if not bad else "  ← 有失败"))
    return 1 if bad else 0


if __name__ == "__main__":
    raise SystemExit(main())
