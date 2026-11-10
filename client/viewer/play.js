// 浏览器里开鸭子：MuJoCo wasm 跑物理 + wasm 执行核跑图 + ORT Web 跑推理，three.js 只负责画。
//
// 素材：
//   vendor/mujoco/{mujoco.js,mujoco.wasm}      ← tools/vendor_mujoco_web.py
//   assets/mjcf/*.stl                          ← 同上（MuJoCo 只吃 STL，viewer 的 GLB 不动）
//   data/mjcf-browser/duck.xml                 ← tools/robot/export_mjcf.py --mesh-dir . --mesh-ext stl
//   data/live/{pre,post}.blob + model.onnx + duck.json ← sim/host/export_browser_check.py

import * as THREE from 'three';
import { OrbitControls } from './vendor/three/controls/OrbitControls.js';
import { GLTFLoader } from './vendor/three/loaders/GLTFLoader.js';
import { loadDuck, makeObs, stepPhysics, read, setContract, getContract, slot, readSlot,
         checkPlanAgainstContract, obsDim, upright } from './play-core.mjs';
import { planPath, policyPath } from './assets-map.mjs';

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

// 频率来自契约（rates），这里只是兜底默认值，载入后会按契约覆盖
let CONTROL_HZ = 50;
let DECIM = 4;

async function bytes(url) {
  const r = await fetch(url);
  if (!r.ok) throw new Error(`${url} 读不到（${r.status}）`);
  return new Uint8Array(await r.arrayBuffer());
}
async function json(url) { return (await fetch(url)).json(); }

// ---------- 场景 ----------
const canvas = $('c');
const isTouch = matchMedia('(hover: none) and (pointer: coarse)').matches;
const renderer = new THREE.WebGLRenderer({ canvas, antialias: !isTouch, powerPreference: 'high-performance' });
// 手机上像素比给到 1.5 就够（2 及以上在 60 Hz 下很吃 GPU）；桌面保留 2
let pixelRatio = Math.min(devicePixelRatio || 1, isTouch ? 1.5 : 2);
renderer.setPixelRatio(pixelRatio);
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
  const w = canvas.clientWidth || window.innerWidth;
  const h = canvas.clientHeight || window.innerHeight;
  if (w === 0 || h === 0) return;
  renderer.setSize(w, h, false);
  camera.aspect = w / h;
  camera.updateProjectionMatrix();
}
addEventListener('resize', resize);
addEventListener('orientationchange', () => setTimeout(resize, 120));
// iOS 地址栏收放不改 window.innerHeight，得盯 visualViewport
if (window.visualViewport) window.visualViewport.addEventListener('resize', resize);
if (window.ResizeObserver) new ResizeObserver(resize).observe(canvas);

// ---------- 载入 ----------
// 指令范围照上游 app/src/game/constants.js：VEL_FWD 0.25 / VEL_BACK -0.2 / VEL_ANG 1.0。
// 超出这个范围策略没见过——它会一边走一边拐（用户看到的就是「走得歪」）。
const LIM = { fwd: 0.25, back: -0.2, ang: 1.0 };
// 关节数来自契约（载入后按契约重设这个数组），这里先占位
const state = { vx: 0, wz: 0, lastAction: new Float32Array(0), simTime: 0, error: null };

async function main() {
  try {
    const [{ default: loadMujoco }, { default: createDucksim }, ort, contract, viewData, xml] = await step(
      '读契约 / 网格索引 / MJCF',
      Promise.all([
      import('./vendor/mujoco/mujoco.js'),
      import('./vendor/ducksim/ducksim.mjs'),
      import('./vendor/ort/ort.wasm.min.mjs'),
      json('./data/live/contract.json'),
      json('./data/duck-hd1910.json'),
      fetch('./data/mjcf-browser/duck.xml').then((r) => r.text()),
    ]), 30000);
    setContract(contract);
    const meta = contract.robot;
    const rates = contract.rates;
    state.lastAction = new Float32Array(meta.joints.length);
    CONTROL_HZ = rates.control_hz;
    DECIM = Math.max(1, Math.round(rates.physics_hz / rates.control_hz));

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
    for (let i = 0; i < model.nq; i++) data.qpos[i] = meta.initial_qpos[i];
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
      const out = { handle: h, input_dim: info[3], output_dim: info[4] };
      wasm._free(ip);
      return out;
    };
    // plan 的形状全部来自契约：文件、维度、具名槽位。加载后立刻对一次维度——
    // 不一致就报错，而不是用旧偏移去读新 blob。
    const pre = loadPlan(await bytes(planPath(contract, 'pre')));
    const post = loadPlan(await bytes(planPath(contract, 'post')));
    const fsm = loadPlan(await bytes(planPath(contract, 'fsm')));
    stage(`创建 ORT Web 会话（${contract.policies.length} 个策略）`);
    checkPlanAgainstContract('pre', pre);
    checkPlanAgainstContract('post', post);
    checkPlanAgainstContract('fsm', fsm);
    const policyList = contract.policies;
    const sessions = {};
    for (const pol of policyList) {
      const buf = await bytes(policyPath(contract, pol.key));
      sessions[pol.key] = await ort.InferenceSession.create(buf,
        { executionProviders: ['wasm'], graphOptimizationLevel: 'all' });
    }
    const modeToKey = {};
    for (const pol of policyList) modeToKey[pol.mode] = pol.key;
    let manual = null;            // 非空 = 手动选的策略（下拉/数字键）；按键后回到状态机
    let active = policyList[0].key;
    const inputName = sessions[active].inputNames[0];
    const outputName = sessions[active].outputNames[0];
    const picker = $('policy');
    for (const pol of policyList) {
      const opt = document.createElement('option');
      opt.value = pol.key;
      opt.textContent = `${pol.mode} · ${pol.key}`;
      picker.appendChild(opt);
    }
    picker.value = active;
    picker.onchange = () => { manual = picker.value; active = manual; state.lastAction.fill(0); };

    // 策略指令脉冲：对应 botlab config/io.yaml 的 policy_command
    const KEY_CMD = { x: 1, z: 2, c: 3, v: 4, b: 5, r: 6 };
    let pendingCmd = 0;

    const runPlan = (plan, input) => {
      const ip = wasm._malloc(plan.input_dim * 4), op = wasm._malloc(plan.output_dim * 4);
      wasm.HEAPF32.set(Float32Array.from(input), ip >> 2);
      const rc = wasm._ds_step(plan.handle, ip, op, 0);
      const out = Array.from(wasm.HEAPF32.subarray(op >> 2, (op >> 2) + plan.output_dim));
      wasm._free(ip); wasm._free(op);
      if (rc !== 0) throw new Error('ds_step ' + rc);
      return out;
    };

    // 键盘
    addEventListener('keydown', (e) => {
      const k = e.key.toLowerCase();
      if (k === 'w') state.vx = Math.min(LIM.fwd, state.vx + 0.05);
      else if (k === 's') state.vx = Math.max(LIM.back, state.vx - 0.05);
      // 实测：策略的 +yaw 是向右（+1.0 → yaw −151°），所以「左转」发负值
      else if (k === 'a') state.wz = -LIM.ang;
      else if (k === 'd') state.wz = LIM.ang;
      else if (k === 'r') resetSim();
      else if (/^[1-7]$/.test(e.key)) {
        const idx = Number(e.key) - 1;
        if (policyList[idx]) { manual = policyList[idx].key; active = manual; $('policy').value = active; state.lastAction.fill(0); }
      } else if (KEY_CMD[k] !== undefined) {
        pendingCmd = KEY_CMD[k];
        manual = null;          // 按策略键即回到状态机
      }
    });

    stage('就绪（网格还在后台加载）');
    const bump = (d) => { state.vx = Math.max(LIM.back, Math.min(LIM.fwd, state.vx + d)); };
    const resetSim = () => {
      for (let i = 0; i < model.nq; i++) data.qpos[i] = meta.initial_qpos[i];
      for (let i = 0; i < model.nv; i++) data.qvel[i] = 0;
      state.vx = 0; state.lastAction.fill(0); state.simTime = 0;
      mj.mj_forward(model, data);
    };
    $('btn-slow').onclick = () => bump(-0.05);
    $('btn-fast').onclick = () => bump(+0.05);
    $('btn-reset').onclick = resetSim;
    // 触屏上的动作键 = 发一次策略脉冲（和键盘 X / Z / V 等价）
    $('btn-sit').onclick = () => { pendingCmd = 1; manual = null; };
    $('btn-kick').onclick = () => { pendingCmd = 2; manual = null; };
    $('btn-roll').onclick = () => { pendingCmd = 4; manual = null; };
    // 转向按钮是「按住生效」：手机上没有键盘，松手即归零
    for (const [id, sign] of [['btn-left', -1], ['btn-right', 1]]) {
      const el = $(id);
      if (!el) continue;
      const on = (e) => { e.preventDefault(); state.wz = LIM.ang * sign; };
      const off = () => { state.wz = 0; };
      el.addEventListener('pointerdown', on);
      el.addEventListener('pointerup', off);
      el.addEventListener('pointerleave', off);
      el.addEventListener('pointercancel', off);
    }

    addEventListener('keyup', (e) => {
      const k = e.key.toLowerCase();
      if (k === 'a' || k === 'd') state.wz = 0;
    });

    $('busy').classList.add('gone');
    loadGeoms.then(() => stage('网格加载完成'));

    // 主循环：严格 4:1 的物理:控制比（200/50 Hz）。
    //
    // 之前这里是「每帧推 4 步物理 + 到点才推理」，移动端帧率一掉，每个控制步就变成 8/12 步物理，
    // 策略看到的是被拉快的动力学 → 上去就打滚。现在用累加器：**一个控制步 = 1 次推理 + 恰好 4 步物理**。
    const STEP_DT = 1 / CONTROL_HZ;
    let acc = 0, last = performance.now(), ctrl = 0, ctrlWindowStart = performance.now(), ctrlWindowCount = 0, realHz = 0;
    const t0 = performance.now();

    async function controlStep() {
      // 1) 状态机：吃「脉冲 + 重力 z + vx」，吐 [state, mode, command(13)]
      const bid = duck.bodyId('trunk_base');
      const qw = read(data.xquat, bid * 4), qx = read(data.xquat, bid * 4 + 1);
      const qy = read(data.xquat, bid * 4 + 2);
      // 重力在世界系是 (0,0,-1)，它在机体系的 z 分量 = -R[2][2] = -(1-2(qx²+qy²))
      const gz = -(1 - 2 * (qx * qx + qy * qy));
      // 输入按槽位塞（名字来自契约），输出按槽位取——端侧不写任何偏移数字
      const fsmStream = new Array(fsm.input_dim).fill(0);
      const put = (name, vals) => { const [off] = slot('fsm', name); vals.forEach((v, i) => { fsmStream[off + i] = v; }); };
      put('in:policy_command', [pendingCmd]);
      put('in:gravity_z', [gz]);
      put('in:vx', [state.vx]);
      put('in:wz', [state.wz]);
      const fsmOut = runPlan(fsm, fsmStream);
      pendingCmd = 0;
      const st = readSlot(fsmOut, 'fsm', 'out:state')[0];
      const mode = readSlot(fsmOut, 'fsm', 'out:mode')[0];
      const cmd13 = readSlot(fsmOut, 'fsm', 'out:command');
      state.fsm = { state: st, mode };
      // botlab 的 `where(clear_last_action, 0, last_action)`：SIT_ENTER 第一步 / WALK_RESET / RECOVER_ENTER 要清零。
      // 漏了它，坐完站起来的第一帧会把坐姿策略的动作喂给行走策略，直接顶飞（2026-11-10 踩过）。
      if (readSlot(fsmOut, 'fsm', 'out:clear_last_action')[0]) state.lastAction.fill(0);
      if (!manual) active = modeToKey[mode] || active;

      // 2) obs（61 维）：指令段用状态机给的 command
      const obs = makeObs(duck, { lastAction: state.lastAction, vx: state.vx, command: cmd13 });
      const policyInput = runPlan(pre, obs);
      const sess = sessions[active];
      const res = await sess.run({ [sess.inputNames[0]]:
        new ort.Tensor('float32', Float32Array.from(policyInput), [1, policyInput.length]) });
      const action = Array.from(res[sess.outputNames[0]].data);
      const ctrlOut = runPlan(post, action);
      for (let j = 0; j < model.nu; j++) data.ctrl[j] = ctrlOut[j];
      state.lastAction.set(action);

      // 3) 恰好 DECIM 步物理
      stepPhysics(mj, model, data, DECIM);
      state.simTime += STEP_DT;
      ctrl += 1;
    }

    function render() {
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
      const fs = state.fsm || { state: '—', mode: '—' };
      $('vx').textContent = `vx ${state.vx.toFixed(2)} · yaw ${state.wz.toFixed(2)} · 状态 ${fs.state} · 策略 ${active}`
        + (manual ? '（手动）' : '');
      $('perf').textContent = `墙钟 ${wall.toFixed(1)} s（${(state.simTime / wall || 0).toFixed(2)}× 实时，控制 ${realHz.toFixed(0)} Hz）`;
    }

    // 自适应画质：连续 2 s 低于 26 fps 就降一档像素比（1.5 → 1.15 → 1），只降不升
    let fpsFrames = 0, fpsSince = performance.now(), downgrades = 0;
    function adaptQuality(now) {
      fpsFrames += 1;
      const dt = now - fpsSince;
      if (dt < 2000) return;
      const fps = (fpsFrames * 1000) / dt;
      fpsFrames = 0; fpsSince = now;
      if (fps < 26 && downgrades < 2 && pixelRatio > 1.02) {
        pixelRatio = Math.max(1, pixelRatio - 0.25);
        renderer.setPixelRatio(pixelRatio);
        downgrades += 1;
        resize();
      }
    }

    async function tick() {
      const now = performance.now();
      acc += Math.min(0.25, (now - last) / 1000);
      last = now;
      // 每次最多补 3 步，避免慢设备上「越补越慢」的死循环
      let budget = 3;
      while (acc >= STEP_DT && budget-- > 0) {
        acc -= STEP_DT;
        try {
          await controlStep();
        } catch (e) {
          console.error(e);
          break;
        }
        ctrlWindowCount += 1;
      }
      if (now - ctrlWindowStart >= 1000) {
        realHz = (ctrlWindowCount * 1000) / (now - ctrlWindowStart);
        ctrlWindowStart = now;
        ctrlWindowCount = 0;
      }
      adaptQuality(now);
      render();
      requestAnimationFrame(tick);
    }
    tick();
  } catch (e) {
    console.error(e);
    stage('载入失败：' + (e && e.message ? e.message : e));
  }
}

main();
