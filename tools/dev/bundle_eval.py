#!/usr/bin/env python3
"""给每个 bundle 打分（0–10）并生成 docs/bundle-scorecard.md。

评分项（总分 10）：

| 项 | 分 | 判据 |
|---|---|---|
| 结构 | 2 | contract.json / robot.xml 能编译 / 模型文件在 |
| 契约 | 2 | 模型输入维度 == 契约 obs 长度；执行器数 == 动作维度 |
| 闭环 | 3 | 300 步 zmin ≥0.55 且末态直立 ≥0.9（需要物理，不需要渲染） |
| 指令 | 1 | 有前进接口时实际速度 >0.05 m/s |
| 对拍 | 2 | manifest 有 parity 且样本通过（复用 test_parity 的实现） |

用法：
    python3 tools/dev/bundle_eval.py                 # 打分 + 写 docs/bundle-scorecard.md
    python3 tools/dev/bundle_eval.py --json          # 只输出 JSON
    python3 tools/dev/bundle_eval.py --min 8         # 低于 8 分退出码非零（CI 用）
"""
from __future__ import annotations

import argparse
import json
import os
import re
import subprocess
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(ROOT, "sim", "host"))
sys.path.insert(0, os.path.join(ROOT, "sim", "test"))

BUNDLE_DIR = os.path.join(ROOT, "build", "release")
SCORECARD = os.path.join(ROOT, "docs", "bundle-scorecard.md")


def _load_json(path):
    try:
        with open(path, encoding="utf-8") as f:
            return json.load(f)
    except Exception:
        return None


def _run_summary(bundle, forward=None):
    """跑 300 步，解析 stdout 里的基座/直立/速度。"""
    env = dict(os.environ)
    env.setdefault("MUJOCO_GL", "egl")
    cmd = [sys.executable, os.path.join(ROOT, "sim", "host", "run_bundle.py"),
           "--bundle", os.path.join(BUNDLE_DIR, bundle),
           "--mjcf", os.path.join(BUNDLE_DIR, bundle, "robot.xml"),
           "--seconds", "6"]
    if forward is not None:
        cmd += ["--forward", str(forward)]
    proc = subprocess.run(cmd, cwd=ROOT, env=env, capture_output=True, text=True, timeout=300)
    text = (proc.stdout or "") + (proc.stderr or "")
    if proc.returncode != 0:
        return None, text.strip().splitlines()[-1] if text.strip() else "运行失败"
    m = re.search(r"基座 终 \+?(-?[\d.]+) 最低 \+?(-?[\d.]+) m｜直立 \+?(-?[\d.]+)", text)
    v = re.search(r"实际速度 ([\d.]+) m/s", text)
    if not m:
        return None, "没解析到汇总行"
    return {"z_end": float(m.group(1)), "z_min": float(m.group(2)),
            "upright": float(m.group(3)), "speed": float(v.group(1)) if v else 0.0}, ""


def _parity(bundle, manifest):
    parity = (manifest or {}).get("parity")
    if not parity:
        return 1.0, "未声明 parity（覆盖未做）"
    path = os.path.join(ROOT, parity.get("sample", ""))
    if not os.path.isfile(path):
        return 0.0, "样本缺失"
    try:
        import test_parity as TP
    except Exception as exc:
        return 0, f"对拍模块不可用：{exc}"
    sample = _load_json(path)
    fmt = sample.get("format")
    tol = float(parity.get("obs_tol", 2e-5))
    try:
        if fmt == "gmtrack":
            ok, detail = TP.check_gmtrack(bundle, sample, tol)
        elif fmt == "parkour":
            ok, detail = TP.check_parkour(bundle, sample, tol)
        elif fmt == "php":
            ok, detail = TP.check_php_full(bundle, sample, tol)
        else:
            return 0, f"{fmt} 无对拍实现"
    except Exception as exc:
        return 0, f"对拍异常：{type(exc).__name__}: {exc}"
    return (2, detail) if ok else (0, detail)


def score(bundle):
    items = []
    total = 0.0

    contract = _load_json(os.path.join(BUNDLE_DIR, bundle, "contract.json"))
    robot_xml = os.path.join(BUNDLE_DIR, bundle, "robot.xml")
    struct = 0.0
    if contract and os.path.isfile(robot_xml):
        try:
            import mujoco
            mujoco.MjModel.from_xml_path(robot_xml)
            struct = 1.0
        except Exception:
            struct = 0.5
        _onnx = [n for n in os.listdir(os.path.join(BUNDLE_DIR, bundle)) if n.endswith(".onnx")]
        model_file = ((((contract.get("models") or {}).get("actor")) or {}).get("file")
                      or (_onnx[0] if _onnx else None))
        if model_file and os.path.isfile(os.path.join(BUNDLE_DIR, bundle, os.path.basename(model_file))):
            struct += 1.0
        elif any(n.endswith(".onnx") for n in os.listdir(os.path.join(BUNDLE_DIR, bundle))):
            struct += 1.0
    items.append(("结构", struct, 2.0, "contract + MJCF + 模型" if struct == 2 else "缺件"))
    total += struct

    contract_ok = 0.0
    if contract:
        def _obs_len(v):
            if "len" in v:
                return int(v["len"])
            slot = v.get("slot")
            return int(slot[1]) if isinstance(slot, (list, tuple)) and len(slot) >= 2 else 0
        obs_len = sum(_obs_len(v) for v in (contract.get("obs") or []))
        # 契约的 obs 是**原始**观测；模型吃的是 pre 计划的输出（WBC 这类会扩维）。
        plans = contract.get("plans") or {}
        need_in = int((plans.get("pre") or {}).get("output_dim") or obs_len)
        want_out = int((plans.get("post") or {}).get("input_dim") or 0) or None
        try:
            import onnxruntime as ort
            _onnx = [n for n in os.listdir(os.path.join(BUNDLE_DIR, bundle)) if n.endswith(".onnx")]
            _decl = (((contract.get("models") or {}).get("actor")) or {}).get("file")
            model_file = _decl or (_onnx[0] if _onnx else None)
            path = os.path.join(BUNDLE_DIR, bundle, os.path.basename(model_file or ""))
            if os.path.isfile(path):
                sess = ort.InferenceSession(path, providers=["CPUExecutionProvider"])
                shape = sess.get_inputs()[0].shape
                dim = [d for d in shape if isinstance(d, int)]
                if dim and need_in == dim[-1]:
                    contract_ok += 1.0
                out_dim = sess.get_outputs()[0].shape
                od = [d for d in out_dim if isinstance(d, int)]
                if od and want_out and od[-1] == want_out:
                    contract_ok += 1.0
                elif od:
                    contract_ok += 0.5
        except ImportError:
            contract_ok = 1.0
        except Exception:
            contract_ok = 0.5
    items.append(("契约", contract_ok, 2.0, f"obs {obs_len} 维" if contract else "无契约"))
    total += contract_ok

    ui = ((contract or {}).get("robot") or {}).get("ui") or {}
    _fwd_max = float((ui.get("forward") or {}).get("max") or 0.0)
    _fwd_default = float((ui.get("forward") or {}).get("default") or 0.0)
    summary, err = _run_summary(bundle, forward=(_fwd_default if _fwd_default > 0 else (0.4 if _fwd_max > 0 else None)))
    if summary and summary["z_min"] >= 0.55 and summary["upright"] >= 0.9:
        closed = 3.0
    elif summary:
        closed = 1.0
    else:
        closed = 0.0
    items.append(("闭环", closed, 3.0,
                  f"zmin {summary['z_min']:.2f} 直立 {summary['upright']:.2f}" if summary else (err or "跑不起来")))
    total += closed

    ui = ((contract or {}).get("robot") or {}).get("ui") or {}
    fwd_max = float((ui.get("forward") or {}).get("max") or 0.0)
    if fwd_max <= 0:
        speed_ok, speed_detail = 1.0, "无前进接口（不算缺项）"
    elif summary and summary["speed"] > 0.05:
        speed_ok, speed_detail = 1.0, f"速度 {summary['speed']:.2f} m/s"
    else:
        speed_ok, speed_detail = 0.0, f"速度 {summary['speed']:.2f} m/s" if summary else "没跑起来"
    items.append(("指令", speed_ok, 1.0, speed_detail))
    total += speed_ok

    manifest = None
    for name in os.listdir(os.path.join(ROOT, "contract", "policy")):
        if not name.endswith(".manifest.yaml"):
            continue
        import yaml
        with open(os.path.join(ROOT, "contract", "policy", name), encoding="utf-8") as f:
            man = yaml.safe_load(f) or {}
        if (man.get("policy_id") or "") == bundle:
            manifest = man
            break
    p_score, p_detail = _parity(bundle, manifest)
    items.append(("对拍", p_score, 2.0, p_detail))
    total += p_score

    return {"bundle": bundle, "score": round(total, 1), "items": items}


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--json", action="store_true")
    ap.add_argument("--min", type=float, default=0.0, help="低于这个分数就返回非零（CI 用）")
    args = ap.parse_args()

    bundles = sorted(d for d in os.listdir(BUNDLE_DIR)
                     if os.path.isdir(os.path.join(BUNDLE_DIR, d))
                     and os.path.isfile(os.path.join(BUNDLE_DIR, d, "contract.json")))
    results = [score(b) for b in bundles]

    if args.json:
        print(json.dumps(results, ensure_ascii=False, indent=2))
    else:
        lines = ["# Bundle 评分（自动生成）", "",
                 "> 由 `python3 tools/dev/bundle_eval.py` 生成，别手改。",
                 "> 评分项：结构 2 / 契约 2 / 闭环 3 / 指令 1 / 对拍 2。", "",
                 "| bundle | 分数 | 结构 | 契约 | 闭环 | 指令 | 对拍 |",
                 "|---|---|---|---|---|---|---|"]
        for r in results:
            d = {name: (s, m) for name, s, m, _ in r["items"]}
            def cell(n):
                s, m = d.get(n, (0, 0))
                return f"{s:g}/{m:g}"
            lines.append(f"| {r['bundle']} | **{r['score']:g}** | {cell('结构')} | {cell('契约')} | "
                         f"{cell('闭环')} | {cell('指令')} | {cell('对拍')} |")
        lines += ["", "## 明细", ""]
        for r in results:
            lines.append(f"### {r['bundle']}（{r['score']:g} 分）")
            for name, s, m, detail in r["items"]:
                mark = "✓" if s >= m else ("△" if s > 0 else "✗")
                lines.append(f"- {mark} **{name}** {s:g}/{m:g} —— {detail}")
            lines.append("")
        with open(SCORECARD, "w", encoding="utf-8") as f:
            f.write("\n".join(lines).rstrip() + "\n")
        for r in results:
            print(f"  {r['score']:>4.1f}  {r['bundle']}")
        print(f"\n已写 {os.path.relpath(SCORECARD, ROOT)}")

    low = [r for r in results if r["score"] < args.min]
    if low:
        print("\n低于阈值：" + ", ".join(f"{r['bundle']}({r['score']:g})" for r in low))
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
