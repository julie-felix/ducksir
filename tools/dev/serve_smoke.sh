#!/usr/bin/env bash
# 统一后端的 curl 冒烟：不需要浏览器，靠退出码判定。
#
#   bash tools/dev/serve_smoke.sh                 # 自己起服务（随机端口）跑一遍
#   PORT=8899 bash tools/dev/serve_smoke.sh       # 打已经在跑的服务
#
# 覆盖 ADR-001 的判据 1/2/5：错误结构化、可 curl 驱动、可自动断言。
set -uo pipefail

cd "$(dirname "$0")/../.."
ROOT=$(pwd)
PORT=${PORT:-0}
OWN=0
FAIL=0

ok()   { printf '  \033[32mPASS\033[0m  %s\n' "$1"; }
bad()  { printf '  \033[31mFAIL\033[0m  %s\n' "$1"; FAIL=$((FAIL + 1)); }
note() { printf '        %s\n' "$1"; }

if [ "$PORT" = "0" ]; then
  PORT=$(python3 - <<'PY'
import socket
s = socket.socket(); s.bind(('127.0.0.1', 0)); print(s.getsockname()[1]); s.close()
PY
)
  python3 sim/host/serve.py --port "$PORT" --no-secret > /tmp/ducksir-serve-smoke.log 2>&1 &
  OWN=$!
  trap 'kill $OWN 2>/dev/null' EXIT
  for _ in $(seq 1 40); do
    curl -s -o /dev/null "http://127.0.0.1:$PORT/api/health" && break
    sleep 0.1
  done
fi

BASE="http://127.0.0.1:$PORT"

echo "serve_smoke · $BASE"

# 1) health
H=$(curl -s "$BASE/api/health")
echo "$H" | grep -q '"ok": true' && ok "health 200 且 ok" || bad "health：$H"

# 2) bundles：base 必填（判据 4）
B=$(curl -s "$BASE/api/bundles")
if echo "$B" | grep -q '"base"'; then
  ok "bundles 每条都有 base"
else
  bad "bundles 缺 base：${B:0:120}"
fi
NAME=$(echo "$B" | python3 -c 'import sys,json; b=json.load(sys.stdin)["bundles"]; print(b[0]["name"] if b else "")')
[ -n "$NAME" ] && ok "至少有一个 bundle（$NAME）" || bad "没有 bundle"

# 3) 不存在的路径必须 400 且错误结构化（判据 4：不猜）
CODE=$(curl -s -o /tmp/path_err.json -w '%{http_code}' -X POST -H 'content-type: application/json' \
  -d '{"op":"bundle_live","args":{"bundle":"build/release/__nope__"}}' "$BASE/api/jobs")
[ "$CODE" = "400" ] && ok "不存在的 bundle → 400" || bad "应当 400，实得 $CODE"
grep -q '"code": "bad_request"' /tmp/path_err.json && ok "错误结构含 code" || bad "错误结构不对：$(cat /tmp/path_err.json)"

# 4) 三种形状都收：这里用 base 形状起一个短作业，读到帧，再取消（判据 2）
BASE_FORM="/ducksir-static/build/release/$NAME/"
J=$(curl -s -X POST -H 'content-type: application/json' \
  -d "{\"op\":\"bundle_live\",\"args\":{\"bundle\":\"$BASE_FORM\",\"seconds\":30}}" "$BASE/api/jobs")
ID=$(echo "$J" | python3 -c 'import sys,json; print(json.load(sys.stdin).get("id",""))')
[ -n "$ID" ] && ok "作业已提交（$ID）" || { bad "作业提交失败：$J"; }

ST=$(curl -s "$BASE/api/jobs/$ID")
echo "$ST" | grep -qE '"status": "(running|queued|done)"' && ok "作业状态可查" || bad "状态查询异常：$ST"

FRAMES=$(timeout 12 curl -sN "$BASE/api/jobs/$ID/stream" | grep -c 'type.*pose' || true)
[ "${FRAMES:-0}" -ge 3 ] && ok "SSE 收到 $FRAMES 帧位姿" || bad "SSE 帧数不足（$FRAMES）"

# 4a) 业务指标（判据 5）：从 pose 流里算出的基座高度/直立度必须合理
MET=$(curl -s "$BASE/api/jobs/$ID")
CHECK=$(echo "$MET" | python3 -c '
import json, sys
job = json.load(sys.stdin)
m = job.get("metrics") or {}
z, up = m.get("z"), m.get("upright")
if z is None:
    print("NONE|" + str(job.get("status"))); sys.exit(0)
print(("OK" if (z > 0.5 and up > 0.8) else "BAD") + f"|{z}|{up}")
')
case "$CHECK" in
  OK*)   ok "业务指标合理（${CHECK#OK|}）" ;;
  BAD*)  bad "业务指标异常（${CHECK#BAD|}：基座/直立不合理）" ;;
  *)     bad "读不到 metrics（$CHECK）" ;;
esac

CANCEL=$(curl -s -X POST "$BASE/api/jobs/$ID/cancel")
echo "$CANCEL" | grep -q '"ok": true' && ok "取消返回 ok" || note "取消返回：$CANCEL"

# 4b) 工程仿真（studio 的兼容层）：op=sim 起得来，/api/project/status 能查到
SIM=$(curl -s -X POST -H 'content-type: application/json' \
  -d '{"op":"sim","args":{"project":"sim/test/fixtures/project_demo.py","seconds":30}}' "$BASE/api/jobs")
SIMID=$(echo "$SIM" | python3 -c 'import sys,json; print(json.load(sys.stdin).get("id",""))')
[ -n "$SIMID" ] && ok "op=sim 作业已提交（$SIMID）" || bad "op=sim 提交失败：$SIM"
sleep 2
PS=$(curl -s "$BASE/api/project/status")
echo "$PS" | grep -q '"running": true' && ok "/api/project/status 运行中" || bad "project status 异常：$PS"
SC=$(curl -s -o /dev/null -w '%{http_code}' "$BASE/api/project/scene")
[ "$SC" = "200" ] && ok "/api/project/scene 200" || bad "project scene 应 200，实得 $SC"
curl -s -X POST "$BASE/api/jobs/$SIMID/cancel" > /dev/null

# 5) 文件树白名单（判据：安全边界）
CODE=$(curl -s -o /dev/null -w '%{http_code}' "$BASE/files?path=../../etc/passwd")
[ "$CODE" = "403" ] && ok "越界路径 → 403" || bad "越界路径应当 403，实得 $CODE"
CODE=$(curl -s -o /dev/null -w '%{http_code}' "$BASE/files?path=build/release")
[ "$CODE" = "200" ] && ok "白名单内可读" || bad "白名单内读失败（$CODE）"

echo
if [ "$FAIL" -gt 0 ]; then
  echo "失败 $FAIL 项（日志 /tmp/ducksir-serve-smoke.log）"
  exit 1
fi
echo "PASS：统一后端 curl 冒烟全过"
