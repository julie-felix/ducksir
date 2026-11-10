// 宿主 ↔ 浏览器 全链路对拍：同一串指令脉冲，两边的状态轨迹与基座高度要对得上。
//
// 为什么值得单独一条：物理对拍（重放 ctrl）已经 0.00000 m，但那证明的是「同一个 ctrl 序列」；
// 这条证明的是**整条链**——状态机、指令按状态给、last_action 的清零、策略推理、控制映射，全都一致。
// 两边唯一的差异是推理 kernel（native vs ORT Web，~1e-6）。
//   node sim/test/test_fsm_parity.mjs
import { readFileSync, readdirSync, existsSync } from 'node:fs';
import { resolve } from 'node:path';
import { pathToFileURL } from 'node:url';
import { execFileSync } from 'node:child_process';
import { loadDuck, makeObs, stepPhysics, read, setContract, getContract, slot, readSlot } from '../../client/viewer/play-core.mjs';

const ROOT = resolve(import.meta.dirname, '../..');
const LIVE = resolve(ROOT, 'client/viewer/data/live');
const STL = resolve(ROOT, 'client/viewer/assets/mjcf');
const PULSES = '3:1,8:1';          // 3 s 坐下，8 s 起身
const SECONDS = 18;
const HOST_OUT = '/tmp/host_fsm_parity.json';

// ── 0) bundle 跑一遍（目标形态：同一份编译产物给两个 target）────────
const BUNDLE = 'build/release/test';
const BUNDLE_OUT = '/tmp/bundle_fsm_parity.json';
execFileSync('python3', [
  'sim/host/run_bundle.py', '--bundle', BUNDLE,
  '--seconds', String(SECONDS), '--fsm', '--pulses', PULSES, '--report', BUNDLE_OUT,
], { cwd: ROOT, stdio: ['ignore', 'pipe', 'inherit'] });
const bundleReport = JSON.parse(readFileSync(BUNDLE_OUT, 'utf8'));
const bundleTrans = bundleReport.state_transitions || [];

// ── 1) 宿主跑一遍 ───────────────────────────────────────────────
execFileSync('python3', [
  'sim/host/run_fsm.py',
  '--mjcf', 'build/mjcf/duck-hd1910-all.xml',
  '--preset', 'build/preset/duck-hd1910.preset.yaml',
  '--seconds', String(SECONDS), '--pulses', PULSES, '--vx', '0.25',
  '--trajectory', HOST_OUT,
], { cwd: ROOT, stdio: ['ignore', 'pipe', 'inherit'] });
const host = JSON.parse(readFileSync(HOST_OUT, 'utf8'));
const hostFrames = host.frames;
const hostStates = host.states ?? [];

// ── 2) 浏览器那套跑一遍 ─────────────────────────────────────────
const contract = JSON.parse(readFileSync(resolve(LIVE, 'contract.json'), 'utf8'));
setContract(contract);
const meta = contract.robot;
const policies = contract.policies;
const modeToKey = Object.fromEntries(policies.map((p) => [p.mode, p.key]));
const { default: loadMujoco } = await import(pathToFileURL(resolve(ROOT, 'client/viewer/vendor/mujoco/mujoco.js')).href);
const { default: createDucksim } = await import(pathToFileURL(resolve(ROOT, 'client/viewer/vendor/ducksim/ducksim.mjs')).href);
const ort = await import(pathToFileURL(resolve(ROOT, 'client/viewer/vendor/ort/ort.wasm.min.mjs')).href);
ort.env.wasm.wasmPaths = pathToFileURL(resolve(ROOT, 'client/viewer/vendor/ort/')).href + '/';
ort.env.wasm.numThreads = 1;
if (ort.env.wasm.proxy !== undefined) ort.env.wasm.proxy = false;

const assets = {};
for (const f of readdirSync(STL)) if (f.toLowerCase().endsWith('.stl')) assets[f] = new Uint8Array(readFileSync(resolve(STL, f)));
const duck = await loadDuck({ loadMujoco, xml: readFileSync(resolve(ROOT, 'client/viewer/data/mjcf-browser/duck.xml'), 'utf8'), assets });
const { mj, model, data } = duck;
for (let i = 0; i < model.nq; i++) data.qpos[i] = meta.initial_qpos[i];
mj.mj_forward(model, data);

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

const pulses = Object.fromEntries(PULSES.split(',').map((kv) => kv.split(':').map(Number)));
let lastAction = new Float32Array(14), pulse = 0;
const bid = duck.bodyId('trunk_base');
const browserZ = [], browserStates = [];

for (let k = 0; k < SECONDS * 50; k++) {
  // 脉冲的 key 是「秒」，这里是控制步，要换算
  if (pulses[k / 50] !== undefined) pulse = pulses[k / 50];   // 秒 key 精确命中该控制步
  const qx = read(data.xquat, bid * 4 + 1), qy = read(data.xquat, bid * 4 + 2);
  const f = runPlan(fsm, fsmInput(pulse, -(1 - 2 * (qx * qx + qy * qy)), 0.25, 0));
  pulse = 0;
  browserStates.push(f[0]);
  if (readSlot(f, 'fsm', 'out:clear_last_action')[0]) lastAction.fill(0);
  const active = modeToKey[readSlot(f, 'fsm', 'out:mode')[0]] || 'walk';
  const obs = makeObs(duck, { lastAction, vx: 0.25, command: readSlot(f, 'fsm', 'out:command') });
  const inp = runPlan(pre, obs);
  const sess = sessions[active];
  const res = await sess.run({ [sess.inputNames[0]]: new ort.Tensor('float32', Float32Array.from(inp), [1, inp.length]) });
  const action = Array.from(res[sess.outputNames[0]].data);
  lastAction.set(action);
  const ctrl = runPlan(post, action);
  for (let j = 0; j < model.nu; j++) data.ctrl[j] = ctrl[j];
  stepPhysics(mj, model, data, 4);
  browserZ.push(read(data.qpos, 2));
}
const hz = hostFrames.map((f) => f.z ?? f.pos[2]);
const n = Math.min(hz.length, browserZ.length);
let maxDiff = 0;
for (let i = 0; i < n; i++) maxDiff = Math.max(maxDiff, Math.abs(hz[i] - browserZ[i]));
const zEndHost = hz[n - 1], zEndBrowser = browserZ[n - 1];
console.log(`宿主机 z 终 ${zEndHost.toFixed(3)}｜浏览器 z 终 ${zEndBrowser.toFixed(3)}｜逐帧最大差 ${maxDiff.toFixed(3)} m`);
const hostStateSeq = hostStates.join(',');
const browserStateSeq = browserStates.join(',');
const firstDiff = hostStates.findIndex((v, i) => v !== browserStates[i]);
const trans = (a) => a.map((v, i) => (i && v !== a[i - 1] ? `${(i / 50).toFixed(2)}s→${v}` : null)).filter(Boolean).join('，');
console.log(`bundle 状态迁移：${bundleTrans.join('，') || '（无）'}`);
console.log(`宿主状态迁移：${trans(hostStates)}`);
console.log(`浏览器状态迁移：${trans(browserStates)}`);
console.log(`状态序列一致：${firstDiff < 0 ? '是' : `否（第一处不同在第 ${firstDiff} 步 = ${(firstDiff / 50).toFixed(2)}s：宿主 ${hostStates[firstDiff]} / 浏览器 ${browserStates[firstDiff]}）`}`);
// 判据分两层（别混成一句「一致」）：
//   1) 状态迁移必须**逐帧相同**——这是控制逻辑等价的硬指标；
//   2) 末态基座高度差 < 3 cm —— 轨迹本身会因为 1e-6 的推理差在接触密集的步态里放大，
//      中间帧差到几厘米是混沌，不是 bug（值照打出来，别拿来当判据）。
const browserTrans = browserStates.map((v, i) => (i && v !== browserStates[i - 1]
  ? `${(i / 50).toFixed(2)}s→${v}` : null)).filter(Boolean);
const bundleSame = bundleTrans.join('|') === browserTrans.join('|');
console.log(`bundle ↔ 浏览器状态迁移一致：${bundleSame ? '是' : '否'}`);
const statesSame = hostStateSeq === browserStateSeq;
const ok = statesSame && bundleSame && Math.abs(zEndHost - zEndBrowser) < 0.03 && zEndBrowser > 0.1;
console.log(ok
  ? 'PASS：状态迁移逐帧相同、末态高度一致（中间帧差属混沌放大）'
  : 'FAIL：两端对不上');
process.exit(ok ? 0 : 1);
