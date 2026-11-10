// 逐个验浏览器侧的 7 个策略：pre → ORT → post，各跑 5 步，确认加载得出有限 action。
//   node sim/test/test_browser_policies.mjs
import { readFileSync } from 'node:fs';
import { resolve } from 'node:path';
import { pathToFileURL } from 'node:url';
import { runBrowserCheck } from '../../client/viewer/live-core.mjs';

const ROOT = resolve(import.meta.dirname, '../..');
const LIVE = resolve(ROOT, 'client/viewer/data/live');
const read = (p) => new Uint8Array(readFileSync(p));

const { default: createDucksim } = await import(
  pathToFileURL(resolve(ROOT, 'client/viewer/vendor/ducksim/ducksim.mjs')).href);
const ort = await import(pathToFileURL(resolve(ROOT, 'client/viewer/vendor/ort/ort.wasm.min.mjs')).href);
ort.env.wasm.wasmPaths = pathToFileURL(resolve(ROOT, 'client/viewer/vendor/ort/')).href + '/';
ort.env.wasm.numThreads = 1;
if (ort.env.wasm.proxy !== undefined) ort.env.wasm.proxy = false;

const pre = read(resolve(LIVE, 'pre.blob'));
const post = read(resolve(LIVE, 'post.blob'));
const obsSeq = JSON.parse(readFileSync(resolve(LIVE, 'obs.json'), 'utf8')).slice(0, 5);
const contract = JSON.parse(readFileSync(resolve(LIVE, 'contract.json'), 'utf8'));
const policies = contract.policies;

let bad = 0;
for (const pol of policies) {
  const res = await runBrowserCheck({
    createDucksim, ort, preBlob: pre, postBlob: post, obsSeq,
    modelBytes: read(resolve(LIVE, pol.file)), expected: null,
  });
  const finite = res.actions.every((a) => a.every((v) => Number.isFinite(v)));
  const span = Math.max(...res.actions.map((a) => Math.max(...a) - Math.min(...a)));
  if (!finite) bad += 1;
  console.log(`  ${pol.key.padEnd(12)} ${res.steps} 步 · ${res.ms} ms · 有限=${finite} · 动作跨度 ${span.toFixed(3)}`);
}
console.log(bad === 0 ? '\n7 个策略全部可加载并出有限动作 PASS' : `\n${bad} 个失败`);
process.exit(bad === 0 ? 0 : 1);
