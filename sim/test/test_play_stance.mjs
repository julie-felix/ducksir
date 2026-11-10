// 回归守卫：浏览器那套（MuJoCo wasm + fsm + pre/post + ORT Web）跑 5 s walk，鸭子应该站着。
//
// 为什么需要它：`data/live/duck.json` 曾经是手搓的，preset 更新后没跟着变、仍是全零位形——
// 于是浏览器里一上去就打滚（2026-11-10 踩过）。这条把「初始位形 + 站姿 + 策略」三件一起钉住。
//   node sim/test/test_play_stance.mjs
import { readFileSync, readdirSync } from 'node:fs';
import { resolve } from 'node:path';
import { pathToFileURL } from 'node:url';
import { loadDuck, makeObs, stepPhysics, read, setContract, getContract, slot, readSlot } from '../../client/viewer/play-core.mjs';

const ROOT = resolve(import.meta.dirname, '../..');
const LIVE = resolve(ROOT, 'client/viewer/data/live');
const STL = resolve(ROOT, 'client/viewer/assets/mjcf');

const { default: loadMujoco } = await import(pathToFileURL(resolve(ROOT, 'client/viewer/vendor/mujoco/mujoco.js')).href);
const { default: createDucksim } = await import(pathToFileURL(resolve(ROOT, 'client/viewer/vendor/ducksim/ducksim.mjs')).href);
const ort = await import(pathToFileURL(resolve(ROOT, 'client/viewer/vendor/ort/ort.wasm.min.mjs')).href);
ort.env.wasm.wasmPaths = pathToFileURL(resolve(ROOT, 'client/viewer/vendor/ort/')).href + '/';
ort.env.wasm.numThreads = 1;
if (ort.env.wasm.proxy !== undefined) ort.env.wasm.proxy = false;

const contract = JSON.parse(readFileSync(resolve(LIVE, 'contract.json'), 'utf8'));
setContract(contract);
const meta = contract.robot;
const policies = contract.policies;
const modeToKey = Object.fromEntries(policies.map((p) => [p.mode, p.key]));
const assets = {};
for (const f of readdirSync(STL)) if (f.toLowerCase().endsWith('.stl')) assets[f] = new Uint8Array(readFileSync(resolve(STL, f)));

const duck = await loadDuck({ loadMujoco, xml: readFileSync(resolve(ROOT, 'client/viewer/data/mjcf-browser/duck.xml'), 'utf8'), assets });
const { mj, model, data } = duck;
for (let i = 0; i < model.nq; i++) data.qpos[i] = meta.initial_qpos[i];
mj.mj_forward(model, data);
const z0 = read(data.qpos, 2);

const wasm = await createDucksim();
const loadPlan = (file) => {
  const b = new Uint8Array(readFileSync(resolve(LIVE, file)));
  const p = wasm._malloc(b.length); wasm.HEAPU8.set(b, p);
  const e = wasm._malloc(512); const h = wasm._ds_load(p, b.length, e, 512);
  if (!h) throw new Error('ds_load ' + wasm.UTF8ToString(e));
  wasm._free(p); wasm._free(e);
  const ip = wasm._malloc(24); wasm._ds_info_of(h, ip);
  const i = new Uint16Array(wasm.HEAPU8.buffer, ip, 12);
  const o = { handle: h, inDim: i[3], outDim: i[4] }; wasm._free(ip); return o;
};
// fsm 的输入按契约槽位塞，不写偏移
function fsmInput(pulse, gz, vx, wz) {
  const buf = new Array(slot('fsm', 'in:vx')[0] + 2).fill(0);
  const put = (name, v) => { buf[slot('fsm', name)[0]] = v; };
  put('in:policy_command', pulse); put('in:gravity_z', gz); put('in:vx', vx); put('in:wz', wz);
  return buf;
}

const runPlan = (pl, input) => {
  const ip = wasm._malloc(pl.inDim * 4), op = wasm._malloc(pl.outDim * 4);
  wasm.HEAPF32.set(Float32Array.from(input), ip >> 2);
  const rc = wasm._ds_step(pl.handle, ip, op, 0);
  const out = Array.from(wasm.HEAPF32.subarray(op >> 2, (op >> 2) + pl.outDim));
  wasm._free(ip); wasm._free(op);
  if (rc !== 0) throw new Error('ds_step ' + rc);
  return out;
};
const pre = loadPlan('pre.blob'), post = loadPlan('post.blob'), fsm = loadPlan('fsm.blob');
const sessions = {};
for (const pol of policies) {
  sessions[pol.key] = await ort.InferenceSession.create(
    new Uint8Array(readFileSync(resolve(LIVE, pol.file))), { executionProviders: ['wasm'], graphOptimizationLevel: 'all' });
}

let lastAction = new Float32Array(14), active = 'walk';
const bid = duck.bodyId('trunk_base');
for (let k = 0; k < 250; k++) {   // 5 s@50Hz
  const qx = read(data.xquat, bid * 4 + 1), qy = read(data.xquat, bid * 4 + 2);
  const f = runPlan(fsm, fsmInput(0, -(1 - 2 * (qx * qx + qy * qy)), 0.5));
  active = modeToKey[f[1]] || active;
  const obs = makeObs(duck, { lastAction, vx: 0.5, command: readSlot(f, 'fsm', 'out:command') });
  const inp = runPlan(pre, obs);
  const sess = sessions[active];
  const res = await sess.run({ [sess.inputNames[0]]: new ort.Tensor('float32', Float32Array.from(inp), [1, inp.length]) });
  const action = Array.from(res[sess.outputNames[0]].data);
  lastAction.set(action);
  const ctrl = runPlan(post, action);
  for (let j = 0; j < model.nu; j++) data.ctrl[j] = ctrl[j];
  stepPhysics(mj, model, data, 4);
}
const z = read(data.qpos, 2);
const qx = read(data.xquat, bid * 4 + 1), qy = read(data.xquat, bid * 4 + 2);
const upright = 1 - 2 * (qx * qx + qy * qy);
console.log(`初始 z=${z0.toFixed(3)} m → 5 s 后 z=${z.toFixed(3)} m，直立度=${upright.toFixed(3)}（策略 ${active}）`);
const ok = z0 > 0.1 && z > 0.1 && upright > 0.9;
console.log(ok ? 'PASS：初始位形合理、策略能把鸭子撑住' : 'FAIL：初始位形或 preset 过期（先跑 mjcf_smoke.py + export_browser_check.py）');
process.exit(ok ? 0 : 1);
