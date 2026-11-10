// 回归守卫：指令范围和走向。
// 上游 constants.js 定的范围是 VEL_FWD=0.25 / VEL_BACK=-0.2 / VEL_ANG=1.0；
// 在范围内应该走直线（10 s 内 |yaw| < 15°、|y| < 0.4 m），超出范围会「一边走一边拐」。
//   node sim/test/test_play_walkcmd.mjs
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
const yawOf = (bid) => {
  const w = read(data.xquat, bid * 4), x = read(data.xquat, bid * 4 + 1);
  const y = read(data.xquat, bid * 4 + 2), z = read(data.xquat, bid * 4 + 3);
  return Math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)) * 180 / Math.PI;
};
const bid = duck.bodyId('trunk_base');

async function run(vx, wz, secs) {
  for (let i = 0; i < model.nq; i++) data.qpos[i] = meta.initial_qpos[i];
  for (let i = 0; i < model.nv; i++) data.qvel[i] = 0;
  mj.mj_forward(model, data);
  let lastAction = new Float32Array(14);
  const p0 = [read(data.qpos, 0), read(data.qpos, 1)], y0 = yawOf(bid);
  for (let k = 0; k < secs * 50; k++) {
    const qx = read(data.xquat, bid * 4 + 1), qy = read(data.xquat, bid * 4 + 2);
    const f = runPlan(fsm, fsmInput(0, -(1 - 2 * (qx * qx + qy * qy)), vx, wz));
    if (readSlot(f, 'fsm', 'out:clear_last_action')[0]) lastAction.fill(0);
    const active = modeToKey[readSlot(f, 'fsm', 'out:mode')[0]] || 'walk';
    const obs = makeObs(duck, { lastAction, vx, command: readSlot(f, 'fsm', 'out:command') });
    const inp = runPlan(pre, obs);
    const sess = sessions[active];
    const res = await sess.run({ [sess.inputNames[0]]: new ort.Tensor('float32', Float32Array.from(inp), [1, inp.length]) });
    const action = Array.from(res[sess.outputNames[0]].data);
    lastAction.set(action);
    const ctrl = runPlan(post, action);
    for (let j = 0; j < model.nu; j++) data.ctrl[j] = ctrl[j];
    stepPhysics(mj, model, data, 4);
  }
  return { dx: read(data.qpos, 0) - p0[0], dy: read(data.qpos, 1) - p0[1], dyaw: yawOf(bid) - y0 };
}

let bad = 0;
const box = await run(0.25, 0, 10);
const straight = Math.abs(box.dyaw) < 15 && Math.abs(box.dy) < 0.4 && box.dx > 0.5;
console.log(`vx=0.25 wz=0    10 s：x ${box.dx.toFixed(2)}｜y ${box.dy.toFixed(2)}｜yaw ${box.dyaw.toFixed(1)}°  ${straight ? 'PASS 走直线' : 'FAIL 走歪了'}`);
if (!straight) bad += 1;
for (const [vx, wz] of [[0.25, 1.0], [0.25, -1.0]]) {
  const t = await run(vx, wz, 6);
  // 策略的 +yaw 是向右：正的 wz 应该让 yaw 减小
  const ok = Math.abs(t.dyaw) > 20 && Math.sign(t.dyaw) === -Math.sign(wz);
  console.log(`vx=${vx} wz=${wz.toFixed(1)}  6 s：yaw ${t.dyaw.toFixed(1)}°  ${ok ? 'PASS 会转' : 'FAIL'}`);
  if (!ok) bad += 1;
}
process.exit(bad === 0 ? 0 : 1);
