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
let loopEnabled = params.get('loop') !== '0';
if (BUNDLE) $('bundle-name').textContent = BUNDLE.split('/').filter(Boolean).pop() || '';

const scene = new THREE.Scene();
scene.background = new THREE.Color(0x0b0f14);
const camera = new THREE.PerspectiveCamera(45, innerWidth / innerHeight, 0.05, 200);
camera.position.set(2.2, 1.4, 2.6);
const renderer = new THREE.WebGLRenderer({ canvas: $('c'), antialias: true });
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
scene.add(root);

let bodies = [];          // [{id,name,parent,group}]
let groupsById = new Map();
let pelvisId = -1;
let poseSeen = 0;

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

function buildScene(model, meshBytes) {
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
  for (const g of model.geoms) {
    const geo = makeGeometry(g, model, meshBytes);
    if (!geo) continue;
    const rgba = g.rgba || [0.8, 0.8, 0.85, 1];
    const mat = new THREE.MeshStandardMaterial({
      color: new THREE.Color(rgba[0], rgba[1], rgba[2]),
      transparent: rgba[3] < 0.99, opacity: rgba[3],
      metalness: 0.15, roughness: 0.6,
    });
    const mesh = new THREE.Mesh(geo, mat);
    mesh.position.set(g.pos[0], g.pos[1], g.pos[2]);
    mesh.quaternion.set(g.quat[1], g.quat[2], g.quat[3], g.quat[0]);
    const host = groupsById.get(g.body);
    (host || root).add(mesh);
    built += 1;
  }
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
  $('line').textContent = `t=${msg.t.toFixed(2)} s · 第 ${msg.step} 步 · 收到 ${poseSeen} 帧`;
  // 第一帧把相机拉到能看清的距离（否则默认视角里机器人只有指甲盖大）
  if (poseSeen === 1 && pelvisId >= 0) {
    const g0 = groupsById.get(pelvisId);
    if (g0) {
      const wp = new THREE.Vector3();
      g0.getWorldPosition(wp);
      controls.target.copy(wp);
      camera.position.set(wp.x + 1.8, wp.y + 0.5, wp.z + 1.8);
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
      const msg = JSON.parse(ev.data);
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

async function startBundle() {
  const bundle = $('bundle').value;
  if (!bundle) { $('status').textContent = '先选一个 bundle'; return; }
  $('status').textContent = '启动中…';
  const r = await fetch('/ducksir-live/bundle/start', {
    method: 'POST', headers: { 'content-type': 'application/json' },
    body: JSON.stringify({ bundle, seconds: 120 }),
  });
  const out = await r.json().catch(() => ({ ok: false, error: '返回不是 JSON' }));
  if (!out.ok) { $('status').textContent = '启动失败：' + (out.error || ''); return; }
  $('status').textContent = '已启动，接流中…';
  const es = new EventSource('/ducksir-live/bundle/stream');
  es.onmessage = (ev) => {
    try {
      const msg = JSON.parse(ev.data);
      if (msg.type === 'scene') {
        bodies = msg.bodies;
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
    const res = await (await fetch('/ducksir-live/bundles')).json();
    const list = res.bundles || [];
    const sel = $('bundle');
    sel.innerHTML = list.map((b) => `<option value="${b.path || b.name}">${b.name}</option>`).join('')
      || '<option value="">（没有 bundle，先导出）</option>';
    $('status').textContent = list.length ? `找到 ${list.length} 个 bundle` : '没有 bundle';
  } catch {
    $('status').textContent = '读不到 bundle 列表（插件宿主没起？）';
  }
}

// 有 ?ws= 就走本地 ws（调试用）；否则走同源 SSE（面板里用这个，https 也不会被拦）
if (WS_PORT) {
  connect();
} else {
  loadBundles();
  const loopBox = $('loop');
  if (loopBox) { loopBox.checked = loopEnabled; loopBox.onchange = () => { loopEnabled = loopBox.checked; }; }
  $('start').onclick = startBundle;
}
tick();
