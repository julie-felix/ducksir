// 浏览器里「自己算一遍」：wasm 核跑图 + ORT Web 做推理，喂宿主采的 obs，比宿主的 action。
// 与 DOM 无关，所以 node 里也能直接跑（sim/test/test_browser_infer.mjs 是它的无头验证）。
//
// 图在推理边界切成两段：`pre.blob`（原始 obs → 策略输入）与 `post.blob`（原始 action → ctrl）。
// 原因很实在：ORT Web 的 `session.run` 是异步的，而我们的 C 核是同步的；把边界放在这里，
// 核保持同步、浏览器负责 await，两边都不别扭。

export async function runBrowserCheck({
  createDucksim, ort, preBlob, postBlob, obsSeq, modelBytes, expected,
  ortOptions = {},
}) {
  const wasm = await createDucksim();

  const loadPlan = (bytes) => {
    const p = wasm._malloc(bytes.length);
    wasm.HEAPU8.set(bytes, p);
    const err = wasm._malloc(512);
    const h = wasm._ds_load(p, bytes.length, err, 512);
    if (!h) {
      const msg = wasm.UTF8ToString(err);
      wasm._free(p); wasm._free(err);
      throw new Error('ds_load 失败：' + msg);
    }
    wasm._free(p); wasm._free(err);
    const ip = wasm._malloc(24);
    wasm._ds_info_of(h, ip);
    const info = new Uint16Array(wasm.HEAPU8.buffer, ip, 12);
    const out = { handle: h, nodes: info[2], inDim: info[3], outDim: info[4] };
    wasm._free(ip);
    return out;
  };

  const pre = loadPlan(preBlob);
  const post = loadPlan(postBlob);

  const session = await ort.InferenceSession.create(modelBytes, {
    executionProviders: ['wasm'],
    graphOptimizationLevel: 'all',
    ...ortOptions,
  });
  const inputName = session.inputNames[0];

  const callPlan = (plan, obs) => {
    const ip = wasm._malloc(plan.inDim * 4);
    const op = wasm._malloc(plan.outDim * 4);
    wasm.HEAPF32.set(Float32Array.from(obs), ip >> 2);
    const rc = wasm._ds_step(plan.handle, ip, op, 0);
    const out = Array.from(wasm.HEAPF32.subarray(op >> 2, (op >> 2) + plan.outDim));
    wasm._free(ip); wasm._free(op);
    if (rc !== 0) throw new Error('ds_step 返回 ' + rc);
    return out;
  };

  const actions = [];
  const t0 = Date.now();
  for (const obs of obsSeq) {
    const policyInput = callPlan(pre, obs);
    const feeds = { [inputName]: new ort.Tensor('float32', Float32Array.from(policyInput), [1, policyInput.length]) };
    const res = await session.run(feeds);
    const rawAction = Array.from(res[session.outputNames[0]].data);
    actions.push(callPlan(post, rawAction));
  }
  const ms = Date.now() - t0;

  let maxDiff = 0;
  if (expected) {
    for (let k = 0; k < actions.length; k++) {
      for (let i = 0; i < actions[k].length; i++) {
        maxDiff = Math.max(maxDiff, Math.abs(actions[k][i] - expected[k][i]));
      }
    }
  }

  wasm._ds_free(pre.handle);
  wasm._ds_free(post.handle);
  return { actions, maxDiff, steps: actions.length, nodes: pre.nodes, ms,
           inputDim: pre.inDim, outputDim: post.outDim };
}
