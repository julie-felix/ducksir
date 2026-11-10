// 无头验证「浏览器物理」这条路：MuJoCo wasm 建模型 + 跑物理 + 组观测。
//   node sim/test/test_play_core.mjs
import { readFileSync, readdirSync } from 'node:fs';
import { resolve } from 'node:path';
import { pathToFileURL } from 'node:url';
import { loadDuck, makeObs, stepPhysics } from '../../client/viewer/play-core.mjs';

const ROOT = resolve(import.meta.dirname, '../..');
const MJCF = resolve(ROOT, 'client/viewer/data/mjcf-browser/duck.xml');
const STL = resolve(ROOT, 'client/viewer/assets/mjcf');
const LIVE = resolve(ROOT, 'client/viewer/data/live');

const { default: loadMujoco } = await import(
  pathToFileURL(resolve(ROOT, 'client/viewer/vendor/mujoco/mujoco.js')).href);

const assets = {};
for (const f of readdirSync(STL)) {
  if (f.toLowerCase().endsWith('.stl')) assets[f] = new Uint8Array(readFileSync(resolve(STL, f)));
}

const duck = await loadDuck({ loadMujoco, xml: readFileSync(MJCF, 'utf8'), assets });
const { mj, model, data } = duck;
console.log(`模型：nbody=${model.nbody} njnt=${model.njnt} nq=${model.nq} nu=${model.nu} nsensor=${model.nsensor}`);

const meta = JSON.parse(readFileSync(resolve(LIVE, 'duck.json'), 'utf8'));
for (let i = 0; i < model.nq; i++) data.qpos[i] = meta.initialQpos[i];
mj.mj_forward(model, data);

// 用一个「保住站姿」的最简控制器代替策略，只为验证物理与观测能跑通
const obs0 = makeObs(duck, { joints: meta.joints, defaultQ: meta.defaultQ, lastAction: null, vx: 0 });
console.log(`观测：${obs0.length} 维｜前 6 项 ${Array.from(obs0.slice(0, 6)).map((v) => v.toFixed(3)).join(', ')}`);

for (let k = 0; k < 200; k++) {
  for (let j = 0; j < model.nu; j++) {
    data.ctrl[j] = meta.defaultQ[j];
  }
  stepPhysics(mj, model, data);
}
const z = data.qpos[2];
const finite = Number.isFinite(z);
console.log(`物理 200 步后基座高度 ${z.toFixed(4)} m  ${finite ? 'PASS' : 'FAIL'}`);
process.exit(finite ? 0 : 1);
