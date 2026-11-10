#!/usr/bin/env bash
# CLI 冒烟：validate / build / run 三条都要能以 0 退出，且 build 产出交接契约。
#   bash sim/test/test_cli.sh
set -u
cd "$(dirname "$0")/../.." || exit 2
fail=0
check() {
  local label="$1"; shift
  if "$@" >/tmp/cli_out 2>&1; then
    echo "  PASS  $label"
  else
    echo "  FAIL  $label"
    tail -5 /tmp/cli_out | sed 's/^/        /'
    fail=$((fail + 1))
  fi
}
check "validate" python3 tools/cli/ducksir.py validate
check "build" python3 tools/cli/ducksir.py build --out build/release/cli-test
check "sim --policy" python3 tools/cli/ducksir.py sim --seconds 2 --vx 0.25
check "sim --fsm" python3 tools/cli/ducksir.py sim --fsm --seconds 4 --pulses "1:1"
check "run 项目脚本" python3 tools/cli/ducksir.py run sim/test/fixtures/project_demo.py --seconds 1 --no-browser
check "golden" python3 tools/cli/ducksir.py golden --release build/release/cli-test

if [ -f build/release/cli-test/contract.json ] && [ -f build/release/cli-test/release.json ]; then
  echo "  PASS  产物齐（contract.json + release.json）"
else
  echo "  FAIL  产物缺"; fail=$((fail + 1))
fi
echo
[ "$fail" -eq 0 ] && echo "PASS：CLI 三件套可用" || echo "失败 $fail 项"
exit $([ "$fail" -eq 0 ] && echo 0 || echo 1)
