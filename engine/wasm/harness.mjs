// wasm 端的跑法：加载 ducksim.mjs（emcc 产物），喂一份 plan blob 和一条 obs 序列，打印 action。
//
//   node engine/wasm/harness.mjs <plan.blob> <obs.json> [--backend-passthrough]
//
// 用途是「同一份 C++ 源码在 native 和 wasm 上算出同一份 action」这条对拍（sim/test/test_wasm_parity.py）。
// 真推理在浏览器里由 ORT Web 通过 `ds_backend.onnx_run` 回调注入，这里用 pass-through 图代替。

import { readFileSync } from 'node:fs';
import { resolve } from 'node:path';
import { pathToFileURL } from 'node:url';

// 产物默认在 ../build/wasm/，可用 DUCKSIM_WASM 指到别处
const modulePath = process.env.DUCKSIM_WASM || resolve(import.meta.dirname, '../build/wasm/ducksim.mjs');
const { default: Module } = await import(pathToFileURL(modulePath).href);

const [blobPath, obsPath] = process.argv.slice(2);
const wasm = await Module();

const blob = readFileSync(blobPath);
const obsSeq = JSON.parse(readFileSync(obsPath, 'utf8'));

const ptr = wasm._malloc(blob.length);
wasm.HEAPU8.set(blob, ptr);
const errPtr = wasm._malloc(512);
const handle = wasm._ds_load(ptr, blob.length, errPtr, 512);
if (!handle) {
  const msg = wasm.UTF8ToString(errPtr);
  console.error('ds_load 失败：' + msg);
  process.exit(2);
}
wasm._free(ptr);
wasm._free(errPtr);

const infoPtr = wasm._malloc(24);
wasm._ds_info_of(handle, infoPtr);
const info = new Uint16Array(wasm.HEAPU8.buffer, infoPtr, 12);
const nodeCount = info[2], inputDim = info[3], outputDim = info[4];
wasm._free(infoPtr);

// ds_backend 是个 C 结构 {void* ctx; int (*onnx_run)(...)}。
// 默认留空（图里 model 节点必须标 pass-through）；给 --js-model 时装一个 JS 回调，
// 用来验证「浏览器里由 ORT Web 注入推理」这条缝是通的（真推理换成 ORT Web 的 session.run 即可）。
const backendPtr = wasm._malloc(16);
wasm.HEAPU8.fill(0, backendPtr, backendPtr + 16);
const jsModelArg = process.argv.indexOf('--js-model');
if (jsModelArg > 0) {
  const [k, b] = process.argv[jsModelArg + 1].split(',').map(Number);
  const fn = (ctx, modelId, inPtr, inLen, outPtr, outLen) => {
    for (let i = 0; i < outLen; i++) {
      const x = wasm.HEAPF32[(inPtr >> 2) + (i % inLen)];
      wasm.HEAPF32[(outPtr >> 2) + i] = k * x + b;
    }
    return 0;
  };
  const fnPtr = wasm.addFunction(fn, 'iiiiiii');
  wasm.HEAPU32[(backendPtr >> 2) + 1] = fnPtr;   // onnx_run
}

const inPtr = wasm._malloc(inputDim * 4);
const outPtr = wasm._malloc(outputDim * 4);
const outs = [];
for (const obs of obsSeq) {
  wasm.HEAPF32.set(Float32Array.from(obs), inPtr / 4);
  const rc = wasm._ds_step(handle, inPtr, outPtr, backendPtr);
  if (rc !== 0) {
    console.error('ds_step 返回 ' + rc);
    process.exit(3);
  }
  outs.push(Array.from(wasm.HEAPF32.subarray(outPtr / 4, outPtr / 4 + outputDim)));
}
wasm._free(inPtr);
wasm._free(outPtr);
wasm._free(backendPtr);
wasm._ds_free(handle);

console.log(JSON.stringify({ nodes: nodeCount, input_dim: inputDim, output_dim: outputDim, actions: outs }));
