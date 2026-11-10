// 无头验证浏览器那条链：wasm 核 + ORT Web（node 里跑同一份 dist），比宿主的 action。
//   node sim/test/test_browser_infer.mjs
import { readFileSync } from 'node:fs';
import { resolve } from 'node:path';
import { pathToFileURL } from 'node:url';
import { runBrowserCheck } from '../../client/viewer/live-core.mjs';

const ROOT = resolve(import.meta.dirname, '../..');
const LIVE = resolve(ROOT, 'client/viewer/data/live');
const read = (p) => readFileSync(p);

const { default: createDucksim } = await import(
  pathToFileURL(resolve(ROOT, 'client/viewer/vendor/ducksim/ducksim.mjs')).href);
const ort = await import(pathToFileURL(resolve(ROOT, 'client/viewer/vendor/ort/ort.wasm.min.mjs')).href);
ort.env.wasm.wasmPaths = pathToFileURL(resolve(ROOT, 'client/viewer/vendor/ort/')).href + '/';
ort.env.wasm.numThreads = 1;
if (ort.env.wasm.proxy !== undefined) ort.env.wasm.proxy = false;

const res = await runBrowserCheck({
  createDucksim,
  ort,
  preBlob: read(resolve(LIVE, 'pre.blob')),
  postBlob: read(resolve(LIVE, 'post.blob')),
  obsSeq: JSON.parse(read(resolve(LIVE, 'obs.json'))),
  modelBytes: read(resolve(LIVE, 'model.onnx')),
  expected: JSON.parse(read(resolve(LIVE, 'actions.json'))),
});

console.log(`浏览器链：${res.steps} 步｜${res.nodes} 节点｜${res.inputDim}→${res.outputDim}｜${res.ms} ms`);
// 判据分两层：
//  1) **图运算**（affine/clip/…）native 与 wasm 是逐位相同的（见 test_wasm_parity.py，0.000e+00）；
//  2) **推理**换成浏览器 ORT Web 之后，kernel 不同（native AVX vs wasm SIMD），末位会差。
//     这里用 1e-5，跟 botlab 自己的奇偶校验阈值（torch↔onnxruntime 1e-4）同量级、更严。
const TOL = 1e-5;
console.log(`与宿主的 action 最大偏差 ${res.maxDiff.toExponential(3)}（阈 ${TOL.toExponential(0)}）  ` +
  (res.maxDiff <= TOL ? 'PASS（图运算逐位相同，推理差来自 kernel）' : 'FAIL'));
process.exit(res.maxDiff <= TOL ? 0 : 1);
