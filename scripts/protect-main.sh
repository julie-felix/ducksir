#!/usr/bin/env bash
# 给 main 加分支保护：禁止直推、必须走 PR。
# 需要 GITHUB_TOKEN（classic: repo 范围 + admin；fine-grained: Administration: write）。
set -euo pipefail

OWNER="${OWNER:-julie-felix}"
REPO="${REPO:-ducksir}"
BRANCH="${BRANCH:-main}"
API="https://api.github.com"

if [ -z "${GITHUB_TOKEN:-}" ]; then
  echo "缺少 GITHUB_TOKEN。" >&2
  echo "用法：GITHUB_TOKEN=xxx bash scripts/protect-main.sh" >&2
  exit 1
fi

auth=(-H "Authorization: Bearer ${GITHUB_TOKEN}" -H "Accept: application/vnd.github+json")

echo "== 仓库设置：只允许 squash，合并后删源分支 =="
curl -fsS -X PATCH "${API}/repos/${OWNER}/${REPO}" "${auth[@]}" \
  -d '{"allow_squash_merge":true,"allow_merge_commit":false,"allow_rebase_merge":false,"delete_branch_on_merge":true,"default_branch":"main"}' \
  | python3 -c 'import json,sys; d=json.load(sys.stdin); print("ok:", d["full_name"], "default:", d["default_branch"])'

echo "== 分支保护：${BRANCH} =="
# required_approving_review_count 设 0：单人仓库，GitHub 不允许自己批准自己；
# 「必须走 PR」由 required_pull_request_reviews 这个对象存在来保证，与批准数无关。
curl -fsS -X PUT "${API}/repos/${OWNER}/${REPO}/branches/${BRANCH}/protection" "${auth[@]}" \
  -d '{
    "required_status_checks": null,
    "enforce_admins": true,
    "required_pull_request_reviews": {
      "dismiss_stale_reviews": true,
      "require_code_owner_reviews": false,
      "required_approving_review_count": 0
    },
    "restrictions": null,
    "allow_force_pushes": false,
    "allow_deletions": false,
    "required_conversation_resolution": true,
    "required_linear_history": true
  }' \
  | python3 -c 'import json,sys; d=json.load(sys.stdin); print("ok: 保护已启用 | 直推:", d["allow_force_pushes"]["enabled"] or "禁止", "| 删除:", d["allow_deletions"]["enabled"] or "禁止")'

echo "== 回读 =="
curl -fsS "${API}/repos/${OWNER}/${REPO}/branches/${BRANCH}/protection" "${auth[@]}" \
  | python3 -c '
import json, sys
d = json.load(sys.stdin)
def flag(v):
    if isinstance(v, dict):
        return v.get("enabled")
    return v
pr = d.get("required_pull_request_reviews")
print("必须走 PR:", "on (批准数=%s)" % pr["required_approving_review_count"] if pr else "off")
print("enforce_admins (管理员也受约束):", flag(d.get("enforce_admins")))
print("allow_force_pushes (应为 off):", flag(d.get("allow_force_pushes")))
print("allow_deletions (应为 off):", flag(d.get("allow_deletions")))
print("线性历史:", flag(d.get("required_linear_history")))
print("合并前须解决对话:", flag(d.get("required_conversation_resolution")))
'
