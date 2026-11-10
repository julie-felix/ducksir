// 浏览器里跑一遍：wasm 核（ducksim）+ ORT Web（本地 vendor），和宿主算出的 action 比。
// 素材由 `python3 sim/host/export_browser_check.py` 生成到 data/live/。
import { runBrowserCheck } from './live-core.mjs';

const $ = (id) => document.getElementById(id);
const log = (msg) => { const el = $('log'); el.textContent += (el.textContent ? '\n' : '') + msg; };
const setRow = (id, v) => { $(id).textContent = v; };

const TOL = 1e-5;

async function bytes(url) {
  const r = await fetch(url);
  if (!r.ok) throw new Error(`${url} 读不到（${r.status}）——先跑 export_browser_check.py`);
  return new Uint8Array(await r.arrayBuffer());
}

async function main() {
  $('run').addEventListener('click', async () => {
    $('run').disabled = true;
    $('log').textContent = '';
    try {
      log('加载 wasm 核…');
      const { default: createDucksim } = await import('./vendor/ducksim/ducksim.mjs');
      log('加载 ORT Web…');
      const ort = await import('./vendor/ort/ort.wasm.min.mjs');
      ort.env.wasm.wasmPaths = new URL('./vendor/ort/', location.href).href;
      ort.env.wasm.numThreads = 1;
      if (ort.env.wasm.proxy !== undefined) ort.env.wasm.proxy = false;

      log('读素材（plan / obs / model）…');
      const [preBlob, postBlob, obsSeq, modelBytes, expected, meta] = await Promise.all([
        bytes('./data/live/pre.blob'),
        bytes('./data/live/post.blob'),
        fetch('./data/live/obs.json').then((r) => r.json()),
        bytes('./data/live/model.onnx'),
        fetch('./data/live/actions.json').then((r) => r.json()),
        fetch('./data/live/meta.json').then((r) => r.json()),
      ]);
      setRow('f-src', `${meta.policy_id} · ${meta.steps} 步 · vx=${meta.vx}`);
      log(`素材就绪：pre ${preBlob.length} B / post ${postBlob.length} B / 模型 ${(modelBytes.length / 1024).toFixed(0)} KB`);

      const res = await runBrowserCheck({
        createDucksim, ort, preBlob, postBlob, obsSeq, modelBytes, expected,
      });
      setRow('f-nodes', `${res.nodes} 节点 · ${res.inputDim} → ${res.outputDim}`);
      setRow('f-ms', `${res.ms} ms（${res.steps} 步）`);
      const ok = res.maxDiff <= TOL;
      const el = $('f-diff');
      el.className = ok ? 'ok' : 'bad';
      el.textContent = `${res.maxDiff.toExponential(3)}（阈 ${TOL.toExponential(0)}）${ok ? ' ✓' : ' ✗'}`;
      log(ok
        ? 'PASS：图运算与 native 逐位相同（0.000e+00），这里的差只来自推理 kernel（native AVX vs wasm SIMD）。'
        : 'FAIL：偏差超过阈值，检查 pre/post 段与模型是否同一份。');
    } catch (e) {
      log('失败：' + (e && e.message ? e.message : e));
    } finally {
      $('run').disabled = false;
    }
  });
}

main();
