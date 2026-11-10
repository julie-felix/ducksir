// 模拟「浏览器跑」页的加载序列（不起 DOM）：契约 → 每个 plan 的字节 → 每个模型 → wasm 核加载。
// 目的：把「页面能不能进到就绪」这件事在 node 里验掉，而不是打开手机看。
//   node sim/test/test_play_load.mjs
import { readFileSync } from 'node:fs';
import { resolve } from 'node:path';
import { pathToFileURL } from 'node:url';
import { setContract, checkPlanAgainstContract } from '../../client/viewer/play-core.mjs';
import { planPath, policyPath } from '../../client/viewer/assets-map.mjs';

const ROOT = resolve(import.meta.dirname, '../..');
const VIEWER = resolve(ROOT, 'client/viewer');
const LIVE = resolve(VIEWER, 'data/live');

const contract = JSON.parse(readFileSync(resolve(LIVE, 'contract.json'), 'utf8'));
setContract(contract);

const { default: createDucksim } = await import(
  pathToFileURL(resolve(VIEWER, 'vendor/ducksim/ducksim.mjs')).href);
const ort = await import(pathToFileURL(resolve(VIEWER, 'vendor/ort/ort.wasm.min.mjs')).href);
ort.env.wasm.wasmPaths = pathToFileURL(resolve(VIEWER, 'vendor/ort/')).href + '/';
ort.env.wasm.numThreads = 1;
if (ort.env.wasm.proxy !== undefined) ort.env.wasm.proxy = false;

const wasm = await createDucksim();
const load = (rel) => {
  const blob = new Uint8Array(readFileSync(resolve(VIEWER, rel)));
  const p = wasm._malloc(blob.length); wasm.HEAPU8.set(blob, p);
  const e = wasm._malloc(512); const h = wasm._ds_load(p, blob.length, e, 512);
  if (!h) throw new Error("ds_load " + rel + ": " + wasm.UTF8ToString(e));
  wasm._free(p); wasm._free(e);
  const ip = wasm._malloc(24); wasm._ds_info_of(h, ip);
  const info = new Uint16Array(wasm.HEAPU8.buffer, ip, 12);
  const out = { handle: h, input_dim: info[3], output_dim: info[4] };
  wasm._free(ip);
  return out;
};

let bad = 0;
for (const name of Object.keys(contract.plans)) {
  try {
    const info = load(planPath(contract, name));
    checkPlanAgainstContract(name, info);
    console.log(`  PASS  plan ${name}：${info.input_dim} → ${info.output_dim}（与契约一致）`);
  } catch (e) {
    bad += 1; console.log(`  FAIL  plan ${name}：` + e.message);
  }
}

for (const pol of contract.policies) {
  try {
    const buf = new Uint8Array(readFileSync(resolve(VIEWER, policyPath(contract, pol.key))));
    const sess = await ort.InferenceSession.create(buf,
      { executionProviders: ['wasm'], graphOptimizationLevel: 'all' });
    const dims = sess.inputNames.length && sess.inputNames[0];
    console.log(`  PASS  策略 ${pol.key}：会话建好（输入 ${dims}）`);
  } catch (e) {
    bad += 1; console.log(`  FAIL  策略 ${pol.key}：` + e.message);
  }
}

console.log(bad === 0 ? '\nPASS：页面加载序列在 node 里走通（契约 → plan → 模型）' : `\n${bad} 条失败`);
process.exit(bad === 0 ? 0 : 1);
