// 宿主直播：物理和推理都在宿主（MuJoCo + ONNX），浏览器只收位姿并画。
//
// 为什么要有这条路径：Instinct 要深度渲染、Uni 要 1720 维拼装，浏览器端还没实现；
// 让宿主算、浏览器看，是「所有 bundle 都能动起来」的最短路径。
//
// 协议见 sim/host/host_stream.py：一条 scene(JSON) + 一条 binary(网格) + 每帧 pose(JSON)。

import * as THREE from 'three';
import { OrbitControls } from './vendor/three/controls/OrbitControls.js';

const $ = (id) => document.getElementById(id);
const params = new URLSearchParams(location.search);
const WS_PORT = params.get('ws') ? Number(params.get('ws')) : 0;
const BUNDLE = params.get('bundle') || '';
// 同源后端默认 /api；被插件面板嵌时由 ?api=/ducksir-live 指到反代前缀
const API = (params.get('api') || '').replace(/\/$/, '');
let loopEnabled = params.get('loop') !== '0';
if (BUNDLE) $('bundle-name').textContent = BUNDLE.split('/').filter(Boolean).pop() || '';

const scene = new THREE.Scene();
scene.background = new THREE.Color(0x0b0f14);
const camera = new THREE.PerspectiveCamera(45, innerWidth / innerHeight, 0.05, 200);
camera.position.set(2.2, 1.4, 2.6);
// preserveDrawingBuffer：探针要把 canvas 内容画进 2D canvas 做像素统计，
// 不开的话 WebGL 缓冲在合成后就丢了（读到全 0）。
const renderer = new THREE.WebGLRenderer({ canvas: $('c'), antialias: true, preserveDrawingBuffer: true });
renderer.setSize(innerWidth, innerHeight, false);
renderer.setPixelRatio(Math.min(devicePixelRatio, 2));

const controls = new OrbitControls(camera, renderer.domElement);
controls.target.set(0, 0.8, 0);
controls.enableDamping = true;

scene.add(new THREE.HemisphereLight(0xffffff, 0x223044, 1.1));
const dir = new THREE.DirectionalLight(0xffffff, 1.2);
dir.position.set(3, 6, 4);
scene.add(dir);

const grid = new THREE.GridHelper(12, 24, 0x2a3542, 0x1a222c);
scene.add(grid);

const root = new THREE.Group();
// MuJoCo 是 z-up，three.js 是 y-up：整棵树绕 X 转 -90°，之后所有位姿都能直接用
// MuJoCo 世界坐标（不转的话机器人会「躺」在地上——这坑今天踩了一次）。
root.rotation.x = -Math.PI / 2;
scene.add(root);

let bodies = [];          // [{id,name,parent,group}]
let groupsById = new Map();
let pelvisId = -1;
let poseSeen = 0;
let lastPose = null;
let meshCount = 0;

function makeGeometry(g, model, meshBytes) {
  if (g.kind === 7 /* MESH */ && g.mesh != null) {
    const m = model.meshes[String(g.mesh)];
    if (!m) return null;
    const vertFloats = m.vertCount * 3;
    const faceCount = m.faceCount * 3;
    const verts = new Float32Array(meshBytes, m.vertOffset, vertFloats);
    const faces = new Uint32Array(meshBytes, m.faceOffset, faceCount);
    const geo = new THREE.BufferGeometry();
    geo.setAttribute('position', new THREE.BufferAttribute(verts.slice(), 3));
    geo.setIndex(new THREE.BufferAttribute(faces.slice(), 1));
    geo.computeVertexNormals();
    return geo;
  }
  if (g.kind === 0) {
    // MuJoCo 平面：局部 xy 平面、法线 +z；root 已整体转到 y-up，所以直接建就好
    const geo = new THREE.PlaneGeometry(g.size[0] * 2, g.size[1] * 2);
    return geo;
  }
  if (g.kind === 2) return new THREE.SphereGeometry(g.size[0], 14, 10);
  if (g.kind === 3) return new THREE.CapsuleGeometry(g.size[0], g.size[1], 6, 14);
  if (g.kind === 6) return new THREE.BoxGeometry(g.size[0] * 2, g.size[1] * 2, g.size[2] * 2);
  return null;
}

function base64ToArrayBuffer(b64) {
  const bin = atob(b64);
  const out = new Uint8Array(bin.length);
  for (let i = 0; i < bin.length; i += 1) out[i] = bin.charCodeAt(i);
  return out.buffer;
}

/** 把上一台机器人的几何从场景里摘掉并释放显存。 */
function clearScene() {
  for (const g of groupsById.values()) {
    g.traverse((o) => {
      if (o.isMesh) {
        o.geometry?.dispose?.();
        if (Array.isArray(o.material)) o.material.forEach((m) => m.dispose?.());
        else o.material?.dispose?.();
      }
    });
    root.remove(g);
  }
  groupsById = new Map();
  bodies = [];
  poseSeen = 0;
  lastPose = null;
  meshCount = 0;
}

function buildScene(model, meshBytes) {
  clearScene();                 // 会清空 bodies；所以下面必须重新赋值（否则 applyPose 拿不到 body 列表）
  bodies = model.bodies;
  // 先建 body 组，再按 parent 挂上去
  groupsById = new Map();
  for (const b of model.bodies) {
    const g = new THREE.Group();
    g.name = b.name;
    groupsById.set(b.id, g);
  }
  // 宿主发的是**世界坐标**位姿，所以每个 body 组都必须直接挂在 root 下：
  // 如果按父子关系嵌套，父 body 的位姿会被子 body 再叠加一次，整机就散成碎片。
  for (const b of model.bodies) root.add(groupsById.get(b.id));
  let built = 0;
  let sceneCount = 0;
  for (const g of model.geoms) {
    const geo = makeGeometry(g, model, meshBytes);
    if (!geo) continue;
    const rgba = g.rgba || [0.8, 0.8, 0.85, 1];
    const isWorld = Boolean(g.world);
    // 地面/地形压暗一点，别抢机器人的视觉重点
    const tint = isWorld ? 0.55 : 1.0;
    const mat = new THREE.MeshStandardMaterial({
      color: new THREE.Color(rgba[0] * tint, rgba[1] * tint, rgba[2] * tint),
      transparent: rgba[3] < 0.99, opacity: rgba[3],
      metalness: isWorld ? 0.05 : 0.15, roughness: isWorld ? 0.95 : 0.6,
      side: isWorld ? THREE.DoubleSide : THREE.FrontSide,
    });
    const mesh = new THREE.Mesh(geo, mat);
    mesh.position.set(g.pos[0], g.pos[1], g.pos[2]);
    mesh.quaternion.set(g.quat[1], g.quat[2], g.quat[3], g.quat[0]);
    if (isWorld) {
      root.add(mesh);           // 世界几何直接挂 root（body 0 也有组，但这样更省事）
      sceneCount += 1;
    } else {
      const host = groupsById.get(g.body);
      (host || root).add(mesh);
      built += 1;
    }
  }
  meshCount = built;
  window.__sceneGeoms = sceneCount;
  pelvisId = model.bodies.find((b) => b.name === 'pelvis')?.id
    ?? model.bodies.find((b) => /torso|trunk/.test(b.name))?.id ?? -1;
  console.log(`[host] 场景建好：${model.bodies.length} body / ${built} geom`);
}

function applyPose(msg) {
  const { xpos, xquat } = msg;
  for (let i = 0; i < bodies.length; i += 1) {
    const g = groupsById.get(bodies[i].id);
    if (!g) continue;
    const p = i * 3; const q = i * 4;
    if (q + 3 >= xquat.length) break;
    g.position.set(xpos[p], xpos[p + 1], xpos[p + 2]);
    g.quaternion.set(xquat[q + 1], xquat[q + 2], xquat[q + 3], xquat[q]);
  }
  poseSeen += 1;
  lastPose = msg;
  // 业务指标（判据 5）：基座高度与直立度，页面自己也能算，和后端 /api/jobs 里的对得上
  const pelvis = groupsById.get(bodies[1] && bodies[1].id);
  let zText = '';
  if (pelvis) {
    const wp = new THREE.Vector3();
    pelvis.getWorldPosition(wp);
    // three 里 y 是上；直立度 = 机体自身 up 向量在世界 up 上的投影
    // 几何在 MuJoCo 轴系里（机体自身 up = 局部 +z），整棵树被 root 转了 -90°，
    // 所以「机体 up 在世界里的方向」= q_world * (0,0,1)，再取世界 +y 分量。
    const worldUp = new THREE.Vector3(0, 0, 1)
      .applyQuaternion(pelvis.getWorldQuaternion(new THREE.Quaternion()));
    zText = ` · 基座 ${wp.y.toFixed(3)} m · 直立 ${worldUp.y.toFixed(3)}`;
  }
  $('line').textContent = `t=${msg.t.toFixed(2)} s · 第 ${msg.step} 步 · 收到 ${poseSeen} 帧${zText}`;
  // 第一帧把相机拉到能看清的距离（否则默认视角里机器人只有指甲盖大）
  if (poseSeen === 1 && pelvisId >= 0) {
    const g0 = groupsById.get(pelvisId);
    if (g0) {
      const wp = new THREE.Vector3();
      g0.getWorldPosition(wp);
      controls.target.copy(wp);
      // 相机放在机器人**后方**：这样正前方（+x）的地形/楼梯才在画面中间
      camera.position.set(wp.x - 3.4, wp.y + 1.3, wp.z);
    }
  }
  // 相机跟着 pelvis 走（不然机器人走出画面）
  if (pelvisId >= 0) {
    const g = groupsById.get(pelvisId);
    if (g) {
      const wp = new THREE.Vector3();
      g.getWorldPosition(wp);
      controls.target.lerp(wp, 0.08);
    }
  }
}

// ADR-002：给探针一个只读自述对象。断言只写数字，不写“看起来对不对”。
window.__probe = {
  state() {
    const pelvisId = bodies[1] ? bodies[1].id : null;
    const g = pelvisId != null ? groupsById.get(pelvisId) : null;
    let z = null; let upright = null;
    if (g) {
      const wp = new THREE.Vector3();
      g.getWorldPosition(wp);
      z = Number(wp.y.toFixed(4));
      const up = new THREE.Vector3(0, 0, 1).applyQuaternion(g.getWorldQuaternion(new THREE.Quaternion()));
      upright = Number(up.y.toFixed(4));
    }
    return {
      sim: lastPose ? lastPose.t : 0,
      step: lastPose ? lastPose.step : 0,
      frames: poseSeen,
      z, upright,
      meshes: meshCount,
      sceneGeoms: window.__sceneGeoms || 0,
      cmd: { forward: Number(($('fwd') || {}).value || 0), yaw: Number(($('yaw') || {}).value || 0) },
      paused,
      bodies: bodies.length,
      groups: groupsById.size,
      error: null,
    };
  },
};

let pendingScene = null;
function connect() {
  const ws = new WebSocket(`ws://${location.hostname}:${WS_PORT}/`);
  ws.binaryType = 'arraybuffer';
  ws.onopen = () => { $('status').textContent = `已连接 ws://${location.hostname}:${WS_PORT}`; };
  ws.onclose = (e) => {
    console.warn('[host] ws 关闭', e.code, e.reason);
    $('status').textContent = `宿主断开（code ${e.code}${e.reason ? '：' + e.reason : ''}）`;
  };
  ws.onerror = () => { $('status').textContent = '连不上宿主：先跑 run_bundle.py --stream'; };
  ws.onmessage = (ev) => {
    if (typeof ev.data === 'string') {
      const wrap = JSON.parse(ev.data);
      if (wrap.type === 'end') {
        es.close();
        $('status').textContent = `这一轮结束（${wrap.status}）`;
        if (loopEnabled) { $('status').textContent = '跑完一轮，重新开始…'; setTimeout(startBundle, 600); }
        return;
      }
      if (typeof wrap.line !== 'string') return;
      let msg;
      try { msg = JSON.parse(wrap.line); }
      catch (e) {          // 长行偶尔被截断（SSE 背压）：跳过这一条，别让整个页面挂掉
        window.__parseErrors = (window.__parseErrors || 0) + 1;
        return;
      }
      if (msg.type === 'scene') {
        pendingScene = msg;
        if (msg.meshBase64) {     // SSE 路径：场景和网格一起送
          bodies = msg.bodies;
          buildScene(msg, base64ToArrayBuffer(msg.meshBase64));
          pendingScene = null;
          $('busy').classList.add('gone');
        }
      } else if (msg.type === 'pose') {
        if (pendingScene) $('busy').classList.add('gone');
        applyPose(msg);
      }
      return;
    }
    // binary：网格数据
    if (pendingScene) {
      bodies = pendingScene.bodies;
      buildScene(pendingScene, ev.data);
      pendingScene = null;
      $('busy').classList.add('gone');
    }
  };
}

function tick() {
  controls.update();
  renderer.render(scene, camera);
  requestAnimationFrame(tick);
}

addEventListener('resize', () => {
  camera.aspect = innerWidth / innerHeight;
  camera.updateProjectionMatrix();
  renderer.setSize(innerWidth, innerHeight, false);
});

let currentJob = null;
let currentEs = null;      // 同一时刻只允许一条流
let sceneGen = 0;          // 每次启动 +1；旧流的回调一律作废
let paused = false;

/** 把当前滑杆值发给宿主（唯一控制入口）。 */
async function pushControl(patch = {}) {
  if (!currentJob) return;
  const body = {
    forward: Number($('fwd') ? $('fwd').value : 0),
    yaw: Number($('yaw') ? $('yaw').value : 0),
    paused,
    ...patch,
  };
  $('fwd-v').textContent = body.forward.toFixed(2);
  $('yaw-v').textContent = body.yaw.toFixed(2);
  try {
    await fetch(`${API}/api/jobs/${currentJob}/control`, {
      method: 'POST', headers: { 'content-type': 'application/json' },
      body: JSON.stringify(body),
    });
  } catch (e) { console.warn('[host] 控制失败', e); }
}

function wireControls() {
  for (const id of ['fwd', 'yaw']) {
    const el = $(id);
    if (el) el.addEventListener('input', () => pushControl());
  }
  const pauseBtn = $('pause');
  if (pauseBtn) pauseBtn.onclick = () => {
    paused = !paused;
    pauseBtn.textContent = paused ? '▶ 继续' : '⏸ 暂停';
    pauseBtn.classList.toggle('held', paused);
    pushControl();
  };
  const zero = $('zero');
  if (zero) zero.onclick = () => {
    if ($('fwd')) $('fwd').value = '0';
    if ($('yaw')) $('yaw').value = '0';
    pushControl();
  };
  addEventListener('keydown', (e) => {
    const step = 0.05;
    const fwd = $('fwd'); const yaw = $('yaw');
    if (!fwd || !yaw) return;
    if (e.key === 'w') fwd.value = String(Math.min(2.5, Number(fwd.value) + step));
    else if (e.key === 's') fwd.value = String(Math.max(-0.8, Number(fwd.value) - step));
    else if (e.key === 'a') yaw.value = String(Math.max(-3.14, Number(yaw.value) - step));
    else if (e.key === 'd') yaw.value = String(Math.min(3.14, Number(yaw.value) + step));
    else return;
    pushControl();
  });
}
async function startBundle() {
  const bundle = $('bundle').value;
  if (!bundle) { $('status').textContent = '先选一个 bundle'; return; }
  $('status').textContent = '启动中…';
  // 只发名字：后端会去 build/release/ 找（ADR-001 判据 4：形状固定，不猜路径）
  const r = await fetch(`${API}/api/jobs`, {
    method: 'POST', headers: { 'content-type': 'application/json' },
    body: JSON.stringify({ op: 'bundle_live', args: { bundle: bundle.split('/').pop(), seconds: 120 } }),
  });
  if (!r.ok) {
    const body = await r.json().catch(() => ({}));
    const e = body.error || {};
    $('status').textContent = `启动失败：HTTP ${r.status} · ${e.code || ''} ${e.message || ''}`;
    return;
  }
  const out = await r.json();
  // 切模型/重启：把上一条流关掉，旧回调靠 gen 判断作废
  if (currentEs) { try { currentEs.close(); } catch (e) { /* noop */ } currentEs = null; }
  const gen = ++sceneGen;
  clearScene();
  currentJob = out.id;
  paused = false;
  if ($('pause')) { $('pause').textContent = '⏸ 暂停'; $('pause').classList.remove('held'); }
  pushControl();
  $('status').textContent = `已启动（job ${out.id}），接流中…`;
  const es = new EventSource(`${API}/api/jobs/${out.id}/stream`);
  currentEs = es;
  es.onmessage = (ev) => {
    if (gen !== sceneGen) return;      // 旧作业的消息：忽略，别把画面/状态搅乱
    try {
      const wrap = JSON.parse(ev.data);
      if (wrap.type === 'end') {
        es.close();
        $('status').textContent = `这一轮结束（${wrap.status}）`;
        if (loopEnabled) { $('status').textContent = '跑完一轮，重新开始…'; setTimeout(startBundle, 600); }
        return;
      }
      if (typeof wrap.line !== 'string') return;
      let msg;
      try { msg = JSON.parse(wrap.line); }
      catch (e) {          // 长行偶尔被截断（SSE 背压）：跳过这一条，别让整个页面挂掉
        window.__parseErrors = (window.__parseErrors || 0) + 1;
        return;
      }
      if (msg.type === 'scene') {
        buildScene(msg, msg.meshBase64 ? base64ToArrayBuffer(msg.meshBase64) : new ArrayBuffer(0));
        $('busy').classList.add('gone');
      } else if (msg.type === 'pose') {
        $('busy').classList.add('gone');
        applyPose(msg);
      } else if (msg.type === 'done') {
        es.close();
        if (loopEnabled) {
          $('status').textContent = '跑完一轮，重新开始…';
          setTimeout(startBundle, 600);
        } else {
          $('status').textContent = '跑完了（勾上「循环」会自动重来）';
        }
      }
    } catch (e) { console.warn('[host] 消息解析失败', e); }
  };
  es.onerror = () => {
    if (gen !== sceneGen) return;
    es.close();
    if (loopEnabled) {
      // 一次跑完（默认 120 s）就自动重来：不循环的话，600 s 的仿真在宿主里 20 秒就跑完了
      $('status').textContent = '跑完一轮，重新开始…';
      setTimeout(startBundle, 600);
    } else {
      $('status').textContent = '跑完了（关掉「循环」就是单次）';
    }
  };
}

async function loadBundles() {
  try {
    const res = await fetch(`${API}/api/bundles`);
    if (!res.ok) {
      $('status').textContent = `读 bundle 列表失败：HTTP ${res.status}`
        + (res.status === 401 ? '（没登录）' : res.status === 404 ? '（后端没起：python3 sim/host/serve.py --port 8899）' : '');
      return;
    }
    const data = await res.json();
    const list = data.bundles || [];
    const sel = $('bundle');
    // 只发名字：后端负责去 build/release/ 找（ADR-001 判据 4）
    sel.innerHTML = list.map((b) => `<option value="${b.name}">${b.name}</option>`).join('')
      || '<option value="">（没有 bundle，先导出）</option>';
    $('status').textContent = list.length ? `找到 ${list.length} 个 bundle，正在启动…` : '没有 bundle';
    return list;
  } catch (e) {
    $('status').textContent = '读不到 bundle 列表：' + (e && e.message ? e.message : e);
  }
}

// 有 ?ws= 就走本地 ws（调试用）；否则走同源 SSE（面板里用这个，https 也不会被拦）
if (WS_PORT) {
  connect();
} else {
  // SSE 模式：不要用全屏遮罩挡住状态文字（否则失败原因被盖住，只能看到「连接宿主…」）
  $('busy').classList.add('gone');
  wireControls();
  loadBundles().then(() => {
    const sel = $('bundle');
    if (sel && sel.value) startBundle();      // 默认自动开第一个，进页面就能动
  });
  const loopBox = $('loop');
  if (loopBox) { loopBox.checked = loopEnabled; loopBox.onchange = () => { loopEnabled = loopBox.checked; }; }
  $('start').onclick = startBundle;
}
tick();
