// 实验台：把一次运行的报告摊开。数据来自 `bin/ducksir run ... --report client/viewer/data/live/report.json`。
import { PANEL_META } from './panel-meta.mjs';

const $ = (id) => document.getElementById(id);

function cell(k, v) {
  const el = document.createElement('div');
  el.className = 'card';
  const meta = PANEL_META[k] || { label: k, unit: '' };
  const num = typeof v === 'number' ? (Number.isInteger(v) ? v : v.toFixed(3)) : v;
  el.innerHTML = `<div class="k">${meta.label}</div><div class="v">${num}<small>${meta.unit || ''}</small></div>`;
  return el;
}

function row(table, k, v, cls = '') {
  const tr = document.createElement('tr');
  tr.innerHTML = `<th>${k}</th><td class="${cls}">${v}</td>`;
  table.appendChild(tr);
}

const SNIPPET = `# 和 bin/ducksir 是同一份实现
import sys; sys.path.insert(0, "sim/sdk")
from ducksir import Duck

d = Duck()                       # 默认 duck-hd1910
print(len(d.joints()), "个驱动关节")
report = d.run(seconds=10, vx=0.25)
print(report["panel"]["realtime_factor"], "× 实时")`;

async function main() {
  let report = null;
  try {
    const res = await fetch('./data/live/report.json');
    if (!res.ok) throw new Error(`report.json 读不到（${res.status}）`);
    report = await res.json();
  } catch (e) {
    $('busy').textContent = e.message + ' —— 先跑 bin/ducksir run --report client/viewer/data/live/report.json';
    return;
  }
  $('snippet').textContent = SNIPPET;

  const panel = report.panel || {};
  const box = $('metrics');
  for (const [k, v] of Object.entries(panel)) box.appendChild(cell(k, v));

  const vt = $('verdict');
  for (const [k, v] of Object.entries(report.verdict || {})) {
    row(vt, k, v ? 'pass' : 'fail', v ? 'ok' : 'bad');
  }

  const ids = $('ids');
  row(ids, '策略', report.policy_id || '—');
  row(ids, '步数', report.steps ?? '—');
  row(ids, '图节点', report.nodes ?? '—');
  row(ids, 'plan 摘要', `<code>${(report.plan_digest || '').slice(0, 16)}…</code>`);
  row(ids, 'manifest 摘要', `<code>${(report.policy_manifest_digest || '').slice(0, 16)}…</code>`);
  row(ids, '模型摘要', `<code>${(report.model_digest || '').slice(0, 16)}…</code>`);

  $('busy').hidden = true;
  $('content').hidden = false;
}

main();
