#!/usr/bin/env bash
# 把 .githooks 设成本仓库的 hooks 目录。
set -euo pipefail
cd "$(dirname "$0")/.."

git config core.hooksPath .githooks
chmod +x .githooks/pre-push
echo "✓ core.hooksPath = .githooks"
echo "  现在直接 push main 会被拦；绕过：DUCKSIR_ALLOW_MAIN_PUSH=1"
