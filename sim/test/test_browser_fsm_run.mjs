// 端到端（无渲染）：MuJoCo wasm 物理 + fsm.blob 状态机 + pre/post 图 + ORT Web 推理。
// 验「按键脉冲 → 状态机切策略 → 动作进物理」这条链在浏览器那套代码里是通的。
//   node sim/test/test_browser_fsm_run.mjs
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
for (const f of readdirSync(STL)) if (f.endsWith('.stl')) assets[f] = new Uint8Array(readFileSync(resolve(STL, f)));
const duck = await loadDuck({ loadMujoco, xml: readFileSync(resolve(ROOT, 'client/viewer/data/mjcf-browser/duck.xml'), 'utf8'), assets });
const { mj, model, data } = duck;
for (let i = 0; i < model.nq; i++) data.qpos[i] = meta.initial_qpos[i];
mj.mj_forward(model, data);

const wasm = await createDucksim();
const loadPlan = (file) => {
  const blob = new Uint8Array(readFileSync(resolve(LIVE, file)));
  const p = wasm._malloc(blob.length); wasm.HEAPU8.set(blob, p);
  const err = wasm._malloc(512);
  const h = wasm._ds_load(p, blob.length, err, 512);
  if (!h) throw new Error('ds_load ' + wasm.UTF8ToString(err));
  wasm._free(p); wasm._free(err);
  const ip = wasm._malloc(24); wasm._ds_info_of(h, ip);
  const info = new Uint16Array(wasm.HEAPU8.buffer, ip, 12);
  const out = { handle: h, inDim: info[3], outDim: info[4] };
  wasm._free(ip);
  return out;
};
// fsm 的输入按契约槽位塞（不写偏移）
function fsmInput(pulse, gz, vx, wz = 0) {
  const buf = new Array(slot('fsm', 'in:vx')[0] + 2).fill(0);
  const put = (name, v) => { buf[slot('fsm', name)[0]] = v; };
  put('in:policy_command', pulse); put('in:gravity_z', gz); put('in:vx', vx); put('in:wz', wz);
  return buf;
}

const runPlan = (plan, input) => {
  const ip = wasm._malloc(plan.inDim * 4), op = wasm._malloc(plan.outDim * 4);
  wasm.HEAPF32.set(Float32Array.from(input), ip >> 2);
  const rc = wasm._ds_step(plan.handle, ip, op, 0);
  const out = Array.from(wasm.HEAPF32.subarray(op >> 2, (op >> 2) + plan.outDim));
  wasm._free(ip); wasm._free(op);
  if (rc !== 0) throw new Error('ds_step ' + rc);
  return out;
};

const pre = loadPlan('pre.blob'), post = loadPlan('post.blob'), fsm = loadPlan('fsm.blob');
const sessions = {};
for (const pol of policies) {
  sessions[pol.key] = await ort.InferenceSession.create(
    new Uint8Array(readFileSync(resolve(LIVE, pol.file))),
    { executionProviders: ['wasm'], graphOptimizationLevel: 'all' });
}

let lastAction = new Float32Array(14);
let pulse = 0, vx = 0.5, active = 'walk';
const log = [], states = [];
const bid = duck.bodyId('trunk_base');
for (let k = 0; k < 260; k++) {
  if (k === 150) pulse = 1;                       // 按一下「坐下/站起」
  const qw = read(data.xquat, bid * 4), qx = read(data.xquat, bid * 4 + 1);
  const qy = read(data.xquat, bid * 4 + 2);
  const gz = -(1 - 2 * (qx * qx + qy * qy));
  const f = runPlan(fsm, fsmInput(pulse, gz, vx));
  pulse = 0;
  if (states.length === 0 || states[states.length - 1] !== f[0]) log.push(`t=${(k / 50).toFixed(2)}s → ${f[0]}`);
  states.push(f[0]);
  active = modeToKey[f[1]] || active;

  const obs = makeObs(duck, { lastAction, vx, command: readSlot(f, 'fsm', 'out:command') });
  const inp = runPlan(pre, obs);
  const sess = sessions[active];
  const res = await sess.run({ [sess.inputNames[0]]:
    new ort.Tensor('float32', Float32Array.from(inp), [1, inp.length]) });
  const action = Array.from(res[sess.outputNames[0]].data);
  lastAction.set(action);
  const ctrl = runPlan(post, action);
  for (let j = 0; j < model.nu; j++) data.ctrl[j] = ctrl[j];
  stepPhysics(mj, model, data);
}
const z = read(data.qpos, 2);
console.log('状态轨迹：', log.join('，') || '（无迁移）');
console.log(`末态：state=${states[states.length - 1]} 策略=${active} 基座 z=${z.toFixed(3)} m`);
const ok = log.some((l) => l.endsWith('→ 1')) && Number.isFinite(z);
console.log(ok ? 'PASS：脉冲触发状态机切策略，动作进了物理' : 'FAIL');
process.exit(ok ? 0 : 1);
