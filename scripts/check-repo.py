#!/usr/bin/env python3
"""仓库骨架检查。零依赖，只用标准库。

检查三件事：
1. 必需文件在不在；
2. 所有 Markdown 相对链接是否指向真实文件；
3. 每个被引用的目录（以 / 结尾）是否真有 README.md。
"""
import re
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
REQUIRED = [
    "README.md",
    "AGENTS.md",
    "GLOSSARY.md",
    "CONTRIBUTING.md",
    "docs/index.md",
    "docs/flow.md",
    ".github/pull_request_template.md",
]
LINK = re.compile(r"\[[^\]]*\]\(([^)\s]+)\)")


def tracked() -> list[str]:
    out = subprocess.run(
        ["git", "ls-files"], cwd=ROOT, capture_output=True, text=True, check=True
    )
    return [line for line in out.stdout.splitlines() if line]


def main() -> int:
    files = tracked()
    if not files:
        print("✗ git ls-files 是空的，是不是不在仓库根？")
        return 1

    errors: list[str] = []

    for name in REQUIRED:
        if name not in files:
            errors.append(f"缺少必需文件: {name}")

    for name in (f for f in files if f.endswith(".md")):
        text = (ROOT / name).read_text(encoding="utf-8")
        for target in LINK.findall(text):
            if target.startswith(("http://", "https://", "mailto:", "#")):
                continue
            path = target.split("#")[0]
            if not path:
                continue
            resolved = (ROOT / name).parent / path
            if path.endswith("/"):
                resolved = resolved / "README.md"
            if not resolved.exists():
                errors.append(f"{name}: 链接目标不存在 -> {target}")

    if errors:
        print(f"✗ {len(errors)} 个问题：")
        for e in errors:
            print(f"  - {e}")
        return 1

    print(f"✓ {len(files)} 个文件，链接全部可达")
    return 0


if __name__ == "__main__":
    sys.exit(main())
