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
import { tensorLens, splitPacked, concatOutputs, isSingleTensor } from './model-io.mjs';
import { isTorque, ctrlFromAction, torqueFromTarget } from './actuators.mjs';
import { loadMotion, Playhead, motionRef } from './motion.mjs';

const $ = (id) => document.getElementById(id);

// `?bundle=<url 前缀>`：直接跑编译产物（工作台用），不依赖 data/live 里那份手搓素材。
// 运行期只吃 bundle：contract.json + pre/post/fsm.blob + *.onnx。
const BUNDLE = new URLSearchParams(location.search).get('bundle');
const DATA = BUNDLE ? (BUNDLE.endsWith('/') ? BUNDLE : BUNDLE + '/') : './data/live/';
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
// 手机上 1.0 就够：AGL/GLB 多、像素比一高帧率就塌，而帧率会反过来拖慢仿真（循环是 rAF 驱动）
let pixelRatio = Math.min(devicePixelRatio || 1, isTouch ? 1.0 : 2);
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
// 速度指令范围：从契约读（鸭子 0.25、G1 能到 2.5），加载契约后覆盖
let LIM = { fwd: 0.25, back: -0.2, ang: 1.0, step: 0.05 };
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
      json(DATA + 'contract.json'),
      json('./data/duck-hd1910.json'),
      // bundle 自带 robot.xml + meshes/（G1 这种），否则退回手搓素材的 duck.xml
      fetch(BUNDLE ? DATA + 'robot.xml' : './data/mjcf-browser/duck.xml')
        .then((r) => (r.ok ? r.text() : fetch('./data/mjcf-browser/duck.xml').then((x) => x.text()))),
    ]), 30000);
    setContract(contract);
    const meta = contract.robot;
    if (meta.commands) {
      const c = meta.commands;
      LIM = {
        fwd: c.forward_max ?? LIM.fwd,
        back: c.forward_min ?? c.back_max ?? LIM.back,
        ang: c.yaw_max ?? LIM.ang,
        step: c.step ?? LIM.step,
      };
      const hint = document.getElementById('hint-vx');
      if (hint) hint.textContent = `前进 ±${LIM.step}（${LIM.back} … ${LIM.fwd}）`;
    }
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
    // bundle 里网格在 DATA + 'meshes/'，手搓素材在 ./assets/mjcf/
    const meshBase = BUNDLE ? DATA : './assets/mjcf/';
    const needFetch = BUNDLE ? await json(DATA + 'meshes.json') : stlNames;
    await step(`下载 ${needFetch.length} 个 STL`, Promise.all(needFetch.map(async (n) => {
      assets[n] = await bytes(meshBase + n);
      done += 1;
      stage(`下载 STL ${done}/${needFetch.length}`);
    })), 180000);

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
    // bundle 里没有手搓的 GLB 索引（G1 这种）：直接用 MuJoCo 自带的网格数据建 mesh
    const useMujocoMeshes = bodies.some((n) => mj.mj_name2id(model, duck.objId('mjOBJ_BODY'), n) < 0)
      || !viewData.links || viewData.links.length === 0;
    if (useMujocoMeshes) {
      const built = renderFromModel({ THREE, mj, model, data, duck, robotGroup, scene });
      // 让渲染循环沿用同一套 bodyOf：每个 body 一个 group，直接读 xpos/xquat
      for (let b = 1; b < model.nbody; b += 1) {
        const nm = mj.mj_id2name(model, duck.objId('mjOBJ_BODY'), b);
        if (nm) bodyOf[nm] = { group: built.groups[b], bid: b };
      }
      // 相机按模型大小取景：body 原点在 Z-up 系里，先换到世界系（robotGroup 有 -90° 绕 X）
      mj.mj_forward(model, data);   // 取景前先刷一次前向，geom_xpos 才是有效值
      const bbox = new THREE.Box3();
      for (let gi = 0; gi < model.ngeom; gi += 1) {
        if (model.geom_contype[gi] !== 0 || model.geom_conaffinity[gi] !== 0) continue;
        bbox.expandByPoint(new THREE.Vector3(
          read(data.geom_xpos, gi * 3), read(data.geom_xpos, gi * 3 + 1), read(data.geom_xpos, gi * 3 + 2)));
      }
      const size3 = bbox.isEmpty() ? new THREE.Vector3(0.4, 0.4, 1.3) : bbox.getSize(new THREE.Vector3());
      const zSpan = Math.max(0.9, Math.max(size3.x, size3.y, size3.z));
      const localCenter = bbox.isEmpty() ? new THREE.Vector3(0, 0, 0.7) : bbox.getCenter(new THREE.Vector3());
      robotGroup.updateMatrixWorld(true);          // 不刷新的话 localToWorld 用的是过期矩阵
      const worldCenter = robotGroup.localToWorld(localCenter.clone());
      controls.target.copy(worldCenter);
      const dist = zSpan * 1.9;
      camera.position.set(worldCenter.x + dist * 0.85, worldCenter.y + dist * 0.42, worldCenter.z + dist * 1.15);
      camera.updateProjectionMatrix();
      controls.update();
      stage('渲染：用 MuJoCo 网格');
    }
    const geoms = useMujocoMeshes ? 0 : viewData.links.reduce((acc, l) => acc + (l.geoms || []).length, 0);
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
    // 动作参考（WBC/AMP）：契约里有 robot.motion 就从 bundle 的 motion/ 读
    let motion = null;
    let playhead = null;
    const motionCfg = contract.robot && contract.robot.motion;
    if (motionCfg && motionCfg.path) {
      try {
        const base = motionCfg.path.startsWith('/') ? motionCfg.path : DATA + motionCfg.path.replace(/\/?$/, '/');
        motion = await loadMotion(base);
        playhead = new Playhead(motion.jointPos.frames,
          { start: 0, frameInterval: motionCfg.frame_interval || 5 });
        playhead.pause(true);   // 进 WBC 前参考停在起点
        state.motion = motion;
        state.playhead = playhead;
      } catch (err) {
        console.warn('[motion] 动作参考加载失败：', err && err.message);
        motion = null;
        playhead = null;
      }
    }

    const pre = loadPlan(await bytes(planPath(contract, 'pre', DATA)));
    const post = loadPlan(await bytes(planPath(contract, 'post', DATA)));
    const fsm = loadPlan(await bytes(planPath(contract, 'fsm', DATA)));
    checkPlanAgainstContract('pre', pre);
    checkPlanAgainstContract('post', post);
    checkPlanAgainstContract('fsm', fsm);
    const policyList = contract.policies;

    // 惰性建会话：以前一次性建 7 个（手机上要几秒、几十 MB），但同一时刻只用一个。
    // 现在用到哪个建哪个，切换策略时才付这个代价。
    const sessions = {};
    async function getSession(key) {
      if (sessions[key]) return sessions[key];
      stage(`创建 ORT Web 会话（${key}）`);
      const buf = await bytes(policyPath(contract, key, DATA));
      const sess = await ort.InferenceSession.create(buf,
        { executionProviders: ['wasm'], graphOptimizationLevel: 'all' });
      // 模型可能是「一进一出」，也可能是递归策略那种「obs+h+c → action+h+c」。
      // 核只认单张量，所以先算好每个张量的宽度，跑的时候按它切分/合并。
      // metadata 在某些 ORT Web 版本上拿不到：拿不到就退回「一进一出」的老路径，
      // 不因为元数据缺失把整页弄挂（老模型本来也是一进一出）。
      try {
        sess.duckInLens = tensorLens(sess.inputMetadata, sess.inputNames);
        sess.duckOutLens = tensorLens(sess.outputMetadata, sess.outputNames);
      } catch (e) {
        sess.duckInLens = null;
        sess.duckOutLens = null;
        stage('（取不到张量元数据，按单张量处理）');
      }
      sessions[key] = sess;
      return sess;
    }
    const modeToKey = {};
    for (const pol of policyList) modeToKey[pol.mode] = pol.key;
    let manual = null;            // 非空 = 手动选的策略（下拉/数字键）；按键后回到状态机
    let active = policyList[0].key;
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
      if (k === 'w') state.vx = Math.min(LIM.fwd, state.vx + LIM.step);
      else if (k === 's') state.vx = Math.max(LIM.back, state.vx - LIM.step);
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

    await getSession(policyList[0].key);   // 先把默认策略建好，第一帧别等
    stage('就绪（网格还在后台加载）');
    const bump = (d) => { state.vx = Math.max(LIM.back, Math.min(LIM.fwd, state.vx + d)); };
    const resetSim = () => {
      for (let i = 0; i < model.nq; i++) data.qpos[i] = meta.initial_qpos[i];
      for (let i = 0; i < model.nv; i++) data.qvel[i] = 0;
      state.vx = 0; state.lastAction.fill(0); state.simTime = 0;
      mj.mj_forward(model, data);
    };
    $('btn-slow').onclick = () => bump(-LIM.step);
    $('btn-fast').onclick = () => bump(+LIM.step);
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
      // 动作参考：进入 WBC 后播放（起步先停几帧让机器人站住，和宿主一致）
      let motionRefValue = null;
      if (motion && playhead) {
        // 起步先停 10 个控制步让机器人站稳，再放参考（和宿主 DUCK_WBC_SETTLE 一致）
        if (playhead.paused && ctrl >= 10) playhead.pause(false);
        const baseQuat = [0, 1, 2, 3].map((k) => read(duck.data.qpos, 3 + k));
        motionRefValue = motionRef(motion, playhead, baseQuat);
        playhead.advance();
      }
      const obs = makeObs(duck, { lastAction: state.lastAction, vx: state.vx, command: cmd13,
                                  motionRef: motionRefValue });
      const policyInput = runPlan(pre, obs);
      const sess = await getSession(active);
      const packed = Float32Array.from(policyInput);
      let action;
      // 递归策略（loco 的 h/c）：输入不止一个时，把上一拍的 h/c 回填进去
      const extraInputs = sess.inputNames.slice(1);
      if (extraInputs.length > 0) {
        if (!sess.recurrent) sess.recurrent = {};
        const feeds = { [sess.inputNames[0]]: new ort.Tensor('float32', packed, [1, packed.length]) };
        extraInputs.forEach((name, i) => {
          if (!sess.recurrent[name]) {
            const meta = sess.inputMetadata?.[sess.inputNames.indexOf(name)];
            const dims = meta && meta.dims && meta.dims.length ? meta.dims.map((d) => (d > 0 ? d : 1)) : [1, 1, 256];
            sess.recurrent[name] = new ort.Tensor('float32', new Float32Array(dims.reduce((a, b) => a * b, 1)), dims);
          }
          feeds[name] = sess.recurrent[name];
        });
        const res = await sess.run(feeds);
        // 按位置回填：out[1] → in[1]（宿主 run_bundle 同规则）
        extraInputs.forEach((name, i) => {
          const key = sess.outputNames[i + 1];
          if (key && res[key]) sess.recurrent[name] = res[key];
        });
        action = Array.from(res[sess.outputNames[0]].data);
      } else if (!sess.duckInLens || isSingleTensor(sess.duckInLens)) {
        const res = await sess.run({ [sess.inputNames[0]]:
          new ort.Tensor('float32', packed, [1, packed.length]) });
        action = sess.duckOutLens
          ? Array.from(concatOutputs(res, sess.duckOutLens))
          : Array.from(res[sess.outputNames[0]].data);
      } else {
        // 多张量（递归策略）：按模型声明切分打包输入，跑完按输出顺序拼回一条
        const feed = {};
        for (const t of splitPacked(packed, sess.duckInLens)) {
          feed[t.name] = new ort.Tensor('float32', Float32Array.from(t.data), t.shape);
        }
        const res = await sess.run(feed);
        action = sess.duckOutLens
          ? Array.from(concatOutputs(res, sess.duckOutLens))
          : Array.from(res[sess.outputNames[0]].data);
      }
      let ctrlOut = runPlan(post, action);
      const robotCfg = getContract().robot;
      // WBC：目标基准是 referenceJointPositions（不是 default）。post 按 default 算好了，这里补差值。
      if (motion && robotCfg.reference_joint_positions && robotCfg.reference_joint_positions.length) {
        const ref = robotCfg.reference_joint_positions;
        const def = robotCfg.default_q || [];
        ctrlOut = ctrlOut.map((v, i) => v + ((ref[i] ?? 0) - (def[i] ?? 0)));
      }
      // 物理：两种执行器语义分开写，**每个控制步恰好 DECIM 个物理子步**（不多不少）
      for (let sub = 0; sub < DECIM; sub += 1) {
        if (isTorque(getContract())) {
          // 扭矩：每个子步用最新 q/qd 重新算 PD（控制步算一次会很快倒）
          const q = new Float64Array(model.nu);
          const qd = new Float64Array(model.nu);
          for (let j = 0; j < model.nu; j += 1) {
            const jid = duck.jointId(robotCfg.joints[j]);
            q[j] = read(data.qpos, read(model.jnt_qposadr, jid));
            qd[j] = read(data.qvel, read(model.jnt_dofadr, jid));
          }
          // ctrlOut 已经是目标角（post 里做过 default + scale×action），不能再缩放一次
          const ctrl = torqueFromTarget(ctrlOut, q, qd, {
            kp: robotCfg.kp, kd: robotCfg.kd, effortLimit: robotCfg.effort_limit,
          });
          for (let j = 0; j < model.nu; j += 1) data.ctrl[j] = ctrl[j];
        } else {
          for (let j = 0; j < model.nu; j += 1) data.ctrl[j] = ctrlOut[j];
        }
        stepPhysics(mj, model, data, 1);
      }
      for (let i = 0; i < action.length; i += 1) {
        action[i] = Math.max(-100, Math.min(100, action[i]));
      }
      state.lastAction.set(action);

      state.simTime += STEP_DT;
      ctrl += 1;
    }

    // 相机跟随：机器人会跳/走，静止相机很快就出画。保持用户当前的观察偏移，只平移目标点。
    const pelvisId = mj.mj_name2id(model, duck.objId('mjOBJ_BODY'), 'pelvis');
    const followTarget = new THREE.Vector3();
    function render() {
      for (const name of Object.keys(bodyOf)) {
        const { group, bid } = bodyOf[name];
        group.position.set(read(data.xpos, bid * 3), read(data.xpos, bid * 3 + 1), read(data.xpos, bid * 3 + 2));
        group.quaternion.set(read(data.xquat, bid * 4 + 1), read(data.xquat, bid * 4 + 2),
                             read(data.xquat, bid * 4 + 3), read(data.xquat, bid * 4));
      }
      if (pelvisId >= 0) {
        robotGroup.updateMatrixWorld(true);
        followTarget.set(read(data.xpos, pelvisId * 3), read(data.xpos, pelvisId * 3 + 1),
                         read(data.xpos, pelvisId * 3 + 2));
        robotGroup.localToWorld(followTarget);
        const offset = camera.position.clone().sub(controls.target);
        controls.target.lerp(followTarget, 0.08);
        camera.position.copy(controls.target).add(offset);
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
      // 每帧最多补的步数：手机上渲染只有 10–20 fps 时，预算太小会让仿真「越跑越慢」。
      // 8 步 ≈ 160 ms 仿真，仍能挡住「一次补几千步」的雪崩。
      let budget = isTouch ? 8 : 4;
      while (acc >= STEP_DT && budget-- > 0) {
        acc -= STEP_DT;
        try {
          await controlStep();
        } catch (e) {
          console.error(e);
          // 把错误摆到 HUD 上：不要让页面看起来「在跑但其实没步进」
          const line = document.getElementById('view-msg') || document.getElementById('stat');
          if (line) line.textContent = `仿真出错：${e && e.message ? e.message : e}`;
          state.error = e;
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

/**
 * 兜底渲染：把 MuJoCo 的网格几何体直接建成 three.js Mesh（bundle 里没有手搓 GLB 索引时用）。
 * 数据都在 model 里：geom_type / geom_dataid / mesh_vertadr / mesh_vertnum / mesh_faceadr / mesh_facenum。
 */
function renderFromModel({ THREE, mj, model, data, duck, robotGroup, scene }) {
  const objId = duck.objId;
  const bodyCount = model.nbody;
  const groups = [];
  for (let b = 0; b < bodyCount; b += 1) {
    const g = new THREE.Group();
    const nm = mj.mj_id2name(model, objId('mjOBJ_BODY'), b);
    g.name = nm || `body_${b}`;
    robotGroup.add(g);
    groups.push(g);
  }
  const meshCache = new Map();
  const mjGEOM_MESH = mj.mjtGeom.mjGEOM_MESH.value ?? 7;
  for (let gi = 0; gi < model.ngeom; gi += 1) {
    // 只画视觉 geom（contype = conaffinity = 0）。碰撞体会把「脚」画成薄壳/方块，看着像残影。
    if (model.geom_contype[gi] !== 0 || model.geom_conaffinity[gi] !== 0) continue;
    const type = model.geom_type[gi];
    const bodyId = model.geom_bodyid[gi];
    const group = groups[bodyId];
    if (!group) continue;
    let geometry = null;
    if (type === mjGEOM_MESH) {
      const mid = model.geom_dataid[gi];
      if (mid >= 0) {
        if (!meshCache.has(mid)) {
          const vAdr = model.mesh_vertadr[mid];
          const vNum = model.mesh_vertnum[mid];
          const fAdr = model.mesh_faceadr[mid];
          const fNum = model.mesh_facenum[mid];
          const pos = new Float32Array(vNum * 3);
          for (let i = 0; i < vNum * 3; i += 1) pos[i] = model.mesh_vert[vAdr * 3 + i];
          const idx = new Uint32Array(fNum * 3);
          for (let i = 0; i < fNum * 3; i += 1) idx[i] = model.mesh_face[fAdr * 3 + i];
          const geo = new THREE.BufferGeometry();
          geo.setAttribute('position', new THREE.BufferAttribute(pos, 3));
          geo.setIndex(new THREE.BufferAttribute(idx, 1));
          geo.computeVertexNormals();
          meshCache.set(mid, geo);
        }
        geometry = meshCache.get(mid);
      }
    } else if (type === (mj.mjtGeom.mjGEOM_SPHERE.value ?? 2)) {
      geometry = new THREE.SphereGeometry(model.geom_size[gi * 3], 12, 8);
    } else if (type === (mj.mjtGeom.mjGEOM_CAPSULE.value ?? 3)) {
      geometry = new THREE.CapsuleGeometry(model.geom_size[gi * 3], model.geom_size[gi * 3 + 1], 6, 12);
    } else if (type === (mj.mjtGeom.mjGEOM_BOX.value ?? 6)) {
      geometry = new THREE.BoxGeometry(model.geom_size[gi * 3] * 2,
        model.geom_size[gi * 3 + 1] * 2, model.geom_size[gi * 3 + 2] * 2);
    }
    if (!geometry) continue;
    const rgba = [0, 1, 2, 3].map((k) => model.geom_rgba[gi * 4 + k]);
    const mat = new THREE.MeshStandardMaterial({
      color: new THREE.Color(rgba[0], rgba[1], rgba[2]),
      transparent: rgba[3] < 0.99, opacity: rgba[3],
      metalness: 0.15, roughness: 0.65,
    });
    const mesh = new THREE.Mesh(geometry, mat);
    // 几何体在 body 系里的位姿（geom_pos / geom_quat）
    mesh.position.set(model.geom_pos[gi * 3], model.geom_pos[gi * 3 + 1], model.geom_pos[gi * 3 + 2]);
    mesh.quaternion.set(model.geom_quat[gi * 4 + 1], model.geom_quat[gi * 4 + 2],
      model.geom_quat[gi * 4 + 3], model.geom_quat[gi * 4]);
    group.add(mesh);
  }
  return { groups, count: meshCache.size };
}
