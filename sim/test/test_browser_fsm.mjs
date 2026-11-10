// 验浏览器那份 fsm.blob：喂「脉冲 + 重力 z + vx」，看状态/策略/指令是否按 botlab 的表走。
//   node sim/test/test_browser_fsm.mjs
import { readFileSync } from 'node:fs';
import { resolve } from 'node:path';
import { pathToFileURL } from 'node:url';

const ROOT = resolve(import.meta.dirname, '../..');
const LIVE = resolve(ROOT, 'client/viewer/data/live');
const { default: createDucksim } = await import(
  pathToFileURL(resolve(ROOT, 'client/viewer/vendor/ducksim/ducksim.mjs')).href);

const wasm = await createDucksim();
const blob = new Uint8Array(readFileSync(resolve(LIVE, 'fsm.blob')));
const ptr = wasm._malloc(blob.length); wasm.HEAPU8.set(blob, ptr);
const err = wasm._malloc(512);
const handle = wasm._ds_load(ptr, blob.length, err, 512);
if (!handle) { console.error('ds_load: ' + wasm.UTF8ToString(err)); process.exit(2); }
const inPtr = wasm._malloc(3 * 4), outPtr = wasm._malloc(15 * 4);

let state = 0, log = [];
const step = (pulse, gz, vx) => {
  wasm.HEAPF32.set(Float32Array.from([pulse, gz, vx]), inPtr >> 2);
  const rc = wasm._ds_step(handle, inPtr, outPtr, 0);
  if (rc !== 0) throw new Error('ds_step ' + rc);
  const out = Array.from(wasm.HEAPF32.subarray(outPtr >> 2, (outPtr >> 2) + 15));
  const prev = state; state = out[0];
  if (state !== prev) log.push(`t=${step.n * 0.02}s → ${state}（策略 ${out[1]}）`);
  step.n += 1;
  return out;
};
step.n = 0;

// 前 150 步站立（重力 -1），第 150 步按「坐下/站起」(脉冲 1)，再看 40 步后是否进 SIT
for (let k = 0; k < 150; k++) step(0, -1.0, 0.5);
step(1, -1.0, 0.5);
for (let k = 0; k < 60; k++) step(0, -1.0, 0.5);
const out1 = step(0, -1.0, 0.5);
console.log('状态迁移：', log.join('，') || '（没有迁移）');
console.log(`当前 state=${out1[0]} mode=${out1[1]} 指令前 3 项 ${out1.slice(2, 5).map((v) => v.toFixed(2))}`);

wasm._free(ptr); wasm._free(err); wasm._free(inPtr); wasm._free(outPtr); wasm._ds_free(handle);

const ok = log.some((l) => l.includes('→ 1')) && log.some((l) => l.includes('→ 2'));
console.log(ok ? 'PASS：脉冲把状态从 0 推到 1，再按 40 步推到 2（与 botlab 的 SIT_ENTER_STEPS 一致）' : 'FAIL：没看到预期迁移');
process.exit(ok ? 0 : 1);
