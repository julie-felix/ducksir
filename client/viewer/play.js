// 浏览器里开鸭子：MuJoCo wasm 跑物理 + wasm 执行核跑图 + ORT Web 跑推理，three.js 只负责画。
//
// 素材：
//   vendor/mujoco/{mujoco.js,mujoco.wasm}      ← tools/vendor_mujoco_web.py
//   assets/mjcf/*.stl                          ← 同上（MuJoCo 只吃 STL，viewer 的 GLB 不动）
//   data/mjcf-browser/duck.xml                 ← tools/robot/export_mjcf.py --mesh-dir . --mesh-ext stl
//   data/live/{pre,post}.blob + model.onnx + duck.json ← sim/host/export_browser_check.py

import * as THREE from './vendor/three/three.module.js';
import { OrbitControls } from './vendor/three/controls/OrbitControls.js';
import { GLTFLoader } from './vendor/three/loaders/GLTFLoader.js';
import { loadDuck, makeObs, stepPhysics, read, OBS_DIM, N_JOINTS } from './play-core.mjs';

const $ = (id) => document.getElementById(id);
const T0 = performance.now();
/** 往遮罩上写一行阶段进度（带毫秒时间戳），这样「卡住」时能看到卡在哪一步。 */
function stage(text) {
  const el = $('busy');
  if (el) el.textContent = `${text}  ·  +${((performance.now() - T0) / 1000).toFixed(1)} s`;
  console.log('[play]', text);
}
/** 包一层超时：卡住时变成一条明确的错误，而不是永远「载入中」。 */
function step(label, promise, timeoutMs) {
  stage(label);
  return new Promise((resolve, reject) => {
    const timer = setTimeout(() => reject(new Error(`${label} 超时（${timeoutMs / 1000}s）`)), timeoutMs);
    Promise.resolve(promise).then(
      (v) => { clearTimeout(timer); resolve(v); },
      (e) => { clearTimeout(timer); reject(e); },
    );
  });
}
addEventListener('error', (e) => stage('脚本出错：' + (e.message || e.error)));
addEventListener('unhandledrejection', (e) => stage('Promise 未处理：' + (e.reason && e.reason.message || e.reason)));

const CONTROL_HZ = 50;
const PHYSICS_HZ = 200;
const DECIM = PHYSICS_HZ / CONTROL_HZ;

async function bytes(url) {
  const r = await fetch(url);
  if (!r.ok) throw new Error(`${url} 读不到（${r.status}）`);
  return new Uint8Array(await r.arrayBuffer());
}
async function json(url) { return (await fetch(url)).json(); }

// ---------- 场景 ----------
const canvas = $('c');
const renderer = new THREE.WebGLRenderer({ canvas, antialias: true });
renderer.setPixelRatio(Math.min(devicePixelRatio, 2));
const scene = new THREE.Scene();
scene.background = new THREE.Color(0x0b0f14);
scene.add(new THREE.HemisphereLight(0xffffff, 0x404850, 1.1));
const key = new THREE.DirectionalLight(0xffffff, 1.6); key.position.set(0.6, 1.2, 0.8); scene.add(key);
const camera = new THREE.PerspectiveCamera(38, 1, 0.01, 50);
const controls = new OrbitControls(camera, canvas);
camera.position.set(0.55, 0.35, 0.75);
controls.target.set(0, 0.12, 0);
const grid = new THREE.GridHelper(6, 60, 0x2a333f, 0x1b222a); scene.add(grid);
const robotGroup = new THREE.Group();
robotGroup.rotation.x = -Math.PI / 2;      // 实体 Z-up → three Y-up
scene.add(robotGroup);

function resize() {
  const w = canvas.clientWidth, h = canvas.clientHeight;
  renderer.setSize(w, h, false);
  camera.aspect = w / Math.max(h, 1);
  camera.updateProjectionMatrix();
}
addEventListener('resize', resize);

// ---------- 载入 ----------
const state = { vx: 0, lastAction: new Float32Array(N_JOINTS), simTime: 0, error: null };

async function main() {
  try {
    const [{ default: loadMujoco }, { default: createDucksim }, ort, meta, viewData, xml] = await step(
      '读元数据（duck.json / viewer data / MJCF）',
      Promise.all([
      import('./vendor/mujoco/mujoco.js'),
      import('./vendor/ducksim/ducksim.mjs'),
      import('./vendor/ort/ort.wasm.min.mjs'),
      json('./data/live/duck.json'),
      json('./data/duck-hd1910.json'),
      fetch('./data/mjcf-browser/duck.xml').then((r) => r.text()),
    ]), 30000);
    ort.env.wasm.wasmPaths = new URL('./vendor/ort/', location.href).href;
    ort.env.wasm.numThreads = 1;
    if (ort.env.wasm.proxy !== undefined) ort.env.wasm.proxy = false;

    const stlNames = [...new Set(xml.match(/\b[a-zA-Z0-9_]+\.stl\b/g) || [])];
    const assets = {};
    let done = 0;
    await step(`下载 ${stlNames.length} 个 STL（约 21 MB）`, Promise.all(stlNames.map(async (n) => {
      assets[n] = await bytes('./assets/mjcf/' + n);
      done += 1;
      stage(`下载 STL ${done}/${stlNames.length}`);
    })), 120000);

    const duck = await step('MuJoCo 解析 MJCF + 网格（这步最慢，十几秒正常）',
      loadDuck({ loadMujoco, xml, assets }), 60000);
    const { mj, model, data } = duck;
    for (let i = 0; i < model.nq; i++) data.qpos[i] = meta.initialQpos[i];
    mj.mj_forward(model, data);

    // 渲染：每个 link 一个 group，里面按 geom 偏移放 GLB；位置直接吃 MuJoCo 的 xpos/xquat
    const loader = new GLTFLoader();
    const bodies = viewData.links.map((l) => l.name);
    const bodyOf = {};
    for (const name of bodies) {
      const bid = mj.mj_name2id(model, duck.objId('mjOBJ_BODY'), name);
      if (bid < 0) continue;
      const group = new THREE.Group();
      group.name = name;
      robotGroup.add(group);
      bodyOf[name] = { group, bid };
    }
    const geoms = viewData.links.reduce((acc, l) => acc + (l.geoms || []).length, 0);
    let gdone = 0;
    const loadGeoms = Promise.all(viewData.links.flatMap((link) => (link.geoms || []).map(async (g) => {
      const entry = bodyOf[link.name];
      if (!entry) return;
      const gltf = await loader.loadAsync(g.mesh);   // 形如 ../../assets/microduck/x.glb（相对本页）
      const inst = gltf.scene.clone(true);
      inst.position.fromArray(g.xyz);
      const [qw, qx, qy, qz] = g.quat;
      inst.quaternion.set(qx, qy, qz, qw);
      inst.traverse((o) => { if (o.isMesh) { o.material = new THREE.MeshStandardMaterial({
        color: new THREE.Color(g.color[0] ?? 0.7, g.color[1] ?? 0.7, g.color[2] ?? 0.7),
        metalness: 0.15, roughness: 0.65 }); } });
      entry.group.add(inst);
      gdone += 1;
      if (gdone % 12 === 0 || gdone === geoms) stage(`GLB ${gdone}/${geoms}`);
    }))).catch((e) => console.warn('GLB 加载有问题（不影响物理）', e));

    stage('加载执行核（wasm）与 plan');
    // 执行核：pre（obs → 策略输入）与 post（原始 action → ctrl）
    const wasm = await createDucksim();
    const loadPlan = (blob) => {
      const p = wasm._malloc(blob.length); wasm.HEAPU8.set(blob, p);
      const err = wasm._malloc(512);
      const h = wasm._ds_load(p, blob.length, err, 512);
      if (!h) throw new Error('ds_load：' + wasm.UTF8ToString(err));
      wasm._free(p); wasm._free(err);
      const ip = wasm._malloc(24); wasm._ds_info_of(h, ip);
      const info = new Uint16Array(wasm.HEAPU8.buffer, ip, 12);
      const out = { handle: h, inDim: info[3], outDim: info[4] };
      wasm._free(ip);
      return out;
    };
    const pre = loadPlan(await bytes('./data/live/pre.blob'));
    const post = loadPlan(await bytes('./data/live/post.blob'));
    stage('创建 ORT Web 会话');
    const session = await ort.InferenceSession.create(await bytes('./data/live/model.onnx'),
      { executionProviders: ['wasm'], graphOptimizationLevel: 'all' });
    const inputName = session.inputNames[0], outputName = session.outputNames[0];

    const runPlan = (plan, input) => {
      const ip = wasm._malloc(plan.inDim * 4), op = wasm._malloc(plan.outDim * 4);
      wasm.HEAPF32.set(Float32Array.from(input), ip >> 2);
      const rc = wasm._ds_step(plan.handle, ip, op, 0);
      const out = Array.from(wasm.HEAPF32.subarray(op >> 2, (op >> 2) + plan.outDim));
      wasm._free(ip); wasm._free(op);
      if (rc !== 0) throw new Error('ds_step ' + rc);
      return out;
    };

    // 键盘
    addEventListener('keydown', (e) => {
      if (e.key === 'w' || e.key === 'W') state.vx = Math.min(1, state.vx + 0.05);
      else if (e.key === 's' || e.key === 'S') state.vx = Math.max(-0.2, state.vx - 0.05);
      else if (e.key === 'r' || e.key === 'R') {
        for (let i = 0; i < model.nq; i++) data.qpos[i] = meta.initialQpos[i];
        for (let i = 0; i < model.nv; i++) data.qvel[i] = 0;
        state.vx = 0; state.lastAction.fill(0); state.simTime = 0;
        mj.mj_forward(model, data);
      }
    });

    stage('就绪（网格还在后台加载）');
    $('busy').style.display = 'none';
    loadGeoms.then(() => stage('网格加载完成'));

    // 主循环：50 Hz 控制（异步推理）+ 200 Hz 物理 + 渲染
    let lastInfer = 0, ctrl = 0;
    const t0 = performance.now();
    const loop = async () => {
      const now = performance.now();
      if (now - lastInfer >= 1000 / CONTROL_HZ) {
        lastInfer = now;
        const obs = makeObs(duck, { joints: meta.joints, defaultQ: meta.defaultQ,
                                    lastAction: state.lastAction, vx: state.vx });
        const policyInput = runPlan(pre, obs);
        const feeds = { [inputName]: new ort.Tensor('float32', Float32Array.from(policyInput), [1, policyInput.length]) };
        const res = await session.run(feeds);
        const action = Array.from(res[outputName].data);
        const ctrlOut = runPlan(post, action);
        for (let j = 0; j < model.nu; j++) data.ctrl[j] = ctrlOut[j];
        state.lastAction.set(action);
        state.simTime += 1 / CONTROL_HZ;
        ctrl++;
      }
      stepPhysics(mj, model, data, DECIM);

      // 渲染
      for (const name of Object.keys(bodyOf)) {
        const { group, bid } = bodyOf[name];
        group.position.set(read(data.xpos, bid * 3), read(data.xpos, bid * 3 + 1), read(data.xpos, bid * 3 + 2));
        group.quaternion.set(read(data.xquat, bid * 4 + 1), read(data.xquat, bid * 4 + 2),
                             read(data.xquat, bid * 4 + 3), read(data.xquat, bid * 4));
      }
      resize();
      controls.update();
      renderer.render(scene, camera);

      const z = read(data.qpos, 2);
      const wall = (performance.now() - t0) / 1000;
      $('stat').textContent = `基座 ${z.toFixed(3)} m · 仿真 ${state.simTime.toFixed(1)} s · ${ctrl} 次推理`;
      $('vx').textContent = `前进指令 ${state.vx.toFixed(2)} m/s`;
      $('perf').textContent = `墙钟 ${wall.toFixed(1)} s（${(state.simTime / wall || 0).toFixed(2)}× 实时）`;
      requestAnimationFrame(loop);
    };
    loop();
  } catch (e) {
    console.error(e);
    stage('载入失败：' + (e && e.message ? e.message : e));
  }
}

main();
