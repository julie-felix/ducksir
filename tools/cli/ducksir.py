#!/usr/bin/env python3
"""ducksir 的统一入口：`validate` / `build` / `run` 三个子命令。

    python3 tools/cli/ducksir.py validate
    python3 tools/cli/ducksir.py build
    python3 tools/cli/ducksir.py run --policy walk --seconds 20 --vx 0.25
    python3 tools/cli/ducksir.py run --fsm --pulses 3:1,8:1

为什么要有它（2026-11-10 的决定）：以前「声明合法吗 / 图能编吗 / MJCF 能 load 吗」散在三个脚本里，
而且只有编译期才发现；`build` 这条「从声明产出可分发的包」更是断的。三条子命令覆盖日常 90%，
退出码统一（0 通过 / 1 失败），输出统一中文。
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import subprocess
import sys
import tempfile
import time

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
sys.path.insert(0, os.path.join(ROOT, "sim", "plan"))
sys.path.insert(0, os.path.join(ROOT, "sim", "host"))
sys.path.insert(0, os.path.join(ROOT, "tools", "robot"))

DEFAULT_ROBOT = "contract/robot/types/duck-hd1910.yaml"
DEFAULT_PRESET = "build/preset/duck-hd1910.preset.yaml"
DEFAULT_MJCF = "build/mjcf/duck-hd1910-all.xml"
DEFAULT_MANIFEST = "contract/policy/duck-walk.manifest.yaml"
MESH_DIR = "third_party/fanhao-microduck-replica/assets"


def say(msg: str = "") -> None:
    print(msg)


def ok(msg: str) -> None:
    print(f"  PASS  {msg}")


def bad(msg: str) -> None:
    print(f"  FAIL  {msg}")


def _py(script: str, *args: str, quiet: bool = True) -> subprocess.CompletedProcess:
    cmd = [sys.executable, os.path.join(ROOT, script), *args]
    if quiet:
        return subprocess.run(cmd, cwd=ROOT, text=True, capture_output=True)
    return subprocess.run(cmd, cwd=ROOT, text=True)


def _sha256(path: str) -> str:
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


# ───────────────────────── validate ─────────────────────────
def cmd_validate(args) -> int:
    failures = 0
    say("== 1/4 实体声明（C1–C9）")
    r = _py("tools/robot/check_entity.py", args.robot)
    if r.returncode == 0:
        ok(f"{args.robot} 通过")
    else:
        bad("实体声明有问题：")
        print((r.stdout or r.stderr).strip())
        failures += 1

    say("== 2/4 policy manifest")
    man = preset = doc = driven = None
    try:
        import yaml
        import entity as E
        import run_policy as RP
        with open(os.path.join(ROOT, args.preset), encoding="utf-8") as f:
            preset = yaml.safe_load(f)
        with open(os.path.join(ROOT, args.manifest), encoding="utf-8") as f:
            man = yaml.safe_load(f)
        doc = E.read_yaml(os.path.join(ROOT, preset["generated"]["from"]))
        driven = [j["name"] for j in doc["joints"] if j.get("actuated")]
        RP.check_manifest(man, preset, len(driven))
        ok(f"{args.manifest}（{len(driven)} 个驱动关节）")
    except SystemExit as e:
        bad(f"manifest 不合法：{e}")
        failures += 1
    except Exception as e:  # noqa: BLE001
        bad(f"manifest 读不了：{e}")
        failures += 1

    say("== 3/4 图能编（plan blob）")
    try:
        import ir
        import run_policy as RP
        acts = doc["actuators"]
        blob = ir.compile_blob(RP.build_plan(
            man, preset, len(driven),
            [float(a["ctrlrange"][0]) for a in acts],
            [float(a["ctrlrange"][1]) for a in acts]))
        info = ir.read_blob(blob)
        ok(f"编译通过：{len(info['nodes'])} 节点 {info['input_dim']} → {info['output_dim']}，{len(blob)} B")
    except Exception as e:  # noqa: BLE001
        bad(f"图编不出来：{e}")
        failures += 1

    say("== 4/4 MJCF 能被 MuJoCo 加载")
    try:
        import mujoco
        with tempfile.TemporaryDirectory() as tmp:
            out = os.path.join(tmp, "duck.xml")
            r = _py("tools/robot/export_mjcf.py", args.robot, "-o", out,
                    "--mesh-dir", os.path.join(ROOT, MESH_DIR),
                    "--mesh-ext", "stl", "--collision-set", "all")
            if r.returncode != 0:
                raise RuntimeError((r.stderr or r.stdout).strip()[:400])
            spec = mujoco.MjSpec.from_file(out)
            g = spec.worldbody.add_geom()
            g.type = mujoco.mjtGeom.mjGEOM_PLANE
            g.size = [4.0, 4.0, 0.1]
            g.condim = 3
            m = spec.compile()
            ok(f"load 成功：nbody={m.nbody} njnt={m.njnt} nu={m.nu} nsensor={m.nsensor}")
    except Exception as e:  # noqa: BLE001
        bad(f"MJCF 加载失败：{e}")
        failures += 1

    say()
    say("全部通过" if failures == 0 else f"失败 {failures} 项")
    return 0 if failures == 0 else 1


# ───────────────────────── build ─────────────────────────
def cmd_build(args) -> int:
    out_dir = args.out or os.path.join("build/release", time.strftime("%Y%m%d-%H%M%S"))
    out_abs = os.path.join(ROOT, out_dir)
    os.makedirs(out_abs, exist_ok=True)
    say(f"== 产出到 {out_dir}")

    steps = [
        ("MJCF（碰撞集 all）", ["tools/robot/export_mjcf.py", args.robot,
                             "-o", os.path.join(out_dir, "duck.xml"),
                             "--mesh-dir", os.path.join(ROOT, MESH_DIR),
                             "--mesh-ext", "stl", "--collision-set", "all"]),
        ("preset", ["tools/robot/export_preset.py", args.robot,
                    "--mjcf", os.path.join(out_dir, "duck.xml"),
                    "--default-pose", "contract/robot/postures/duck-hd1910.yaml",
                    "-o", os.path.join(out_dir, "preset.yaml")]),
        ("策略包（blob + 模型 + 契约）", ["sim/host/export_artifacts.py",
                                   "--manifest", args.manifest,
                                   "--preset", os.path.join(out_dir, "preset.yaml"),
                                   "--out", out_dir]),
    ]
    failures = 0
    for label, argv in steps:
        r = _py(argv[0], *argv[1:])
        if r.returncode == 0:
            ok(label)
        else:
            bad(f"{label}：{(r.stderr or r.stdout).strip()[:300]}")
            failures += 1
    if failures:
        say()
        say(f"失败 {failures} 项")
        return 1

    index = {"built_at": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
             "robot": os.path.basename(args.robot).rsplit(".", 1)[0],
             "manifest": args.manifest,
             "artifacts": []}
    for name in sorted(os.listdir(out_abs)):
        path = os.path.join(out_abs, name)
        if os.path.isfile(path):
            index["artifacts"].append({"name": name, "bytes": os.path.getsize(path),
                                       "sha256": _sha256(path)})
    with open(os.path.join(out_abs, "release.json"), "w", encoding="utf-8") as f:
        json.dump(index, f, ensure_ascii=False, indent=2)
    ok(f"release.json（{len(index['artifacts'])} 个文件）")
    say()
    say(f"完成 → {out_dir}")
    return 0


# ───────────────────────── golden ─────────────────────────
def cmd_golden(args) -> int:
    import importlib
    sys.path.insert(0, os.path.join(ROOT, "sim", "plan"))
    golden = importlib.import_module("golden")
    rel = os.path.join(ROOT, args.release)
    if not os.path.exists(os.path.join(rel, "contract.json")):
        bad(f"缺 {args.release}/contract.json —— 先跑 `bin/ducksir build --out {args.release}`")
        return 2
    cases = golden.cases_from_release(args.release)
    fixture = os.path.join(ROOT, "contract/plan/fixtures/release-golden.json")
    if args.update:
        golden.record(cases, fixture, note="由 bin/ducksir golden --update 记录")
        ok("已更新快照 → contract/plan/fixtures/release-golden.json")
        return 0
    problems = golden.check(cases, fixture)
    if problems:
        for prob in problems:
            bad(prob)
        say()
        say("快照对不上。若是**有意**改图，跑 `bin/ducksir golden --update` 并说明原因。")
        return 1
    ok("与快照逐位相同：" + ", ".join(f"{n}({len(f)} 帧)" for n, _, f in cases))
    return 0


# ───────────────────────── run ─────────────────────────
def cmd_run(args) -> int:
    if args.fsm:
        argv = ["--mjcf", args.mjcf, "--preset", args.preset,
                "--seconds", str(args.seconds), "--pulses", args.pulses]
        if args.report:
            argv += ["--report", args.report]
        if args.trajectory:
            argv += ["--trajectory", args.trajectory]
        return _py("sim/host/run_fsm.py", *argv, quiet=False).returncode

    argv = ["--mjcf", args.mjcf, "--preset", args.preset,
            "--manifest", args.manifest, "--seconds", str(args.seconds)]
    if args.vx is not None:
        argv += ["--vx", str(args.vx)]
    if args.report:
        argv += ["--report", args.report]
    if args.trajectory:
        argv += ["--trajectory", args.trajectory]
    return _py("sim/host/run_policy.py", *argv, quiet=False).returncode


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(prog="ducksir", description="机器人流水线的统一入口")
    sub = ap.add_subparsers(dest="cmd", required=True)

    def common(p):
        p.add_argument("--robot", default=DEFAULT_ROBOT)
        p.add_argument("--preset", default=DEFAULT_PRESET)
        p.add_argument("--manifest", default=DEFAULT_MANIFEST)

    v = sub.add_parser("validate", help="声明 / 图 / MJCF 三件事一次查完")
    common(v)
    v.set_defaults(fn=cmd_validate)

    b = sub.add_parser("build", help="从声明产出一包（blob + 模型 + 契约 + release 索引）")
    common(b)
    b.add_argument("--out", help="输出目录（默认 build/release/<时间戳>）")
    b.set_defaults(fn=cmd_build)

    r = sub.add_parser("run", help="跑策略或状态机（宿主）")
    common(r)
    r.add_argument("--mjcf", default=DEFAULT_MJCF)
    r.add_argument("--seconds", type=float, default=20.0)
    r.add_argument("--vx", type=float, help="前进指令（walk 用）")
    r.add_argument("--fsm", action="store_true", help="跑 14 状态机而不是单策略")
    r.add_argument("--pulses", default="3:1,8:1", help="--fsm 的指令脉冲，秒:值")
    r.add_argument("--report", help="写 JSON 报告")
    r.add_argument("--trajectory", help="写回放轨迹 JSON")
    r.set_defaults(fn=cmd_run)

    g = sub.add_parser("golden", help="golden 快照比对（改核后确认行为没变）")
    g.add_argument("--update", action="store_true", help="重新记录基准（会写文件）")
    g.add_argument("--release", default="build/release/test", help="从哪个 release 目录取 plan")
    g.set_defaults(fn=cmd_golden)

    args = ap.parse_args(argv)
    return args.fn(args)


if __name__ == "__main__":
    raise SystemExit(main())
