// 实时跟随：从宿主（CPython + C 核）收每帧 body 位姿，只负责画。
//
// 协议（sim/host/display.py）：先 GET /scene 拿 body 名序，再 SSE /stream 收
// {t, pos:[x0,y0,z0, x1,…], quat:[w0,x0,y0,z0, w1,…]}，下标对应 scene.bodies。
// 和 botlab 的 `--native` 一样：物理在 Python 侧，浏览器只是只读渲染客户端。
import * as THREE from 'three';
import { OrbitControls } from '/static/vendor/three/controls/OrbitControls.js';
import { GLTFLoader } from '/static/vendor/three/loaders/GLTFLoader.js';

const $ = (id) => document.getElementById(id);
// runner 直开模式：GLB 从 /static/assets 拿（显示服务把仓库 assets/ 挂在这里）
const glb = (rel) => '/static/assets/' + rel.replace(/^\.\.\/\.\.\/assets\//, '').replace(/^\.\.\/assets\//, '');
const canvas = $('c');
const renderer = new THREE.WebGLRenderer({ canvas, antialias: true });
renderer.setPixelRatio(Math.min(devicePixelRatio || 1, 2));

const scene = new THREE.Scene();
scene.background = new THREE.Color(0x0b0f14);
scene.add(new THREE.HemisphereLight(0xffffff, 0x404850, 1.1));
const key = new THREE.DirectionalLight(0xffffff, 1.5);
key.position.set(0.6, 1.2, 0.8);
scene.add(key);
const camera = new THREE.PerspectiveCamera(38, 1, 0.01, 50);
const controls = new OrbitControls(camera, canvas);
camera.position.set(0.55, 0.35, 0.75);
controls.target.set(0, 0.12, 0);
scene.add(new THREE.GridHelper(6, 60, 0x2a333f, 0x1b222a));

const robotGroup = new THREE.Group();
robotGroup.rotation.x = -Math.PI / 2;   // 实体 Z-up → three Y-up
scene.add(robotGroup);

const bodyIndex = new Map();   // body 名 → 在帧数组里的下标
const groups = new Map();      // body 名 → THREE.Group
let frames = 0, lastT = 0, hz = 0, hzSince = performance.now(), hzFrames = 0;

function resize() {
  const w = canvas.clientWidth || innerWidth, h = canvas.clientHeight || innerHeight;
  renderer.setSize(w, h, false);
  camera.aspect = w / h;
  camera.updateProjectionMatrix();
}
addEventListener('resize', resize);

async function buildRobot() {
  const viewData = await (await fetch('/static/data/duck-hd1910.json')).json();
  const loader = new GLTFLoader();
  await Promise.all(viewData.links.flatMap((link) => (link.geoms || []).map(async (g) => {
    let group = groups.get(link.name);
    if (!group) {
      group = new THREE.Group();
      group.name = link.name;
      robotGroup.add(group);
      groups.set(link.name, group);
    }
    const gltf = await loader.loadAsync(g.mesh.replace('../../assets/', '/static/assets/').replace('../assets/', '/static/assets/'));
    const inst = gltf.scene.clone(true);
    inst.position.fromArray(g.xyz);
    const [qw, qx, qy, qz] = g.quat;
    inst.quaternion.set(qx, qy, qz, qw);
    inst.traverse((o) => {
      if (o.isMesh) {
        o.material = new THREE.MeshStandardMaterial({
          color: new THREE.Color(g.color?.[0] ?? 0.7, g.color?.[1] ?? 0.7, g.color?.[2] ?? 0.7),
          metalness: 0.15, roughness: 0.65,
        });
      }
    });
    group.add(inst);
  })));
  return viewData;
}

function applyFrame(frame) {
  for (const [name, group] of groups) {
    const i = bodyIndex.get(name);
    if (i === undefined) continue;
    const p = i * 3, q = i * 4;
    group.position.set(frame.pos[p], frame.pos[p + 1], frame.pos[p + 2]);
    group.quaternion.set(frame.quat[q + 1], frame.quat[q + 2], frame.quat[q + 3], frame.quat[q]);
  }
  frames += 1;
  hzFrames += 1;
  lastT = frame.t;
  const now = performance.now();
  if (now - hzSince > 1000) {
    hz = (hzFrames * 1000) / (now - hzSince);
    hzSince = now;
    hzFrames = 0;
  }
  $('stat').textContent = `t=${lastT.toFixed(2)}s · ${frames} 帧 · ${hz.toFixed(0)} Hz`;
  $('msg').classList.add('gone');
}

function loop() {
  resize();
  controls.update();
  renderer.render(scene, camera);
  requestAnimationFrame(loop);
}

async function main() {
  try {
    const sc = await (await fetch('/scene')).json();
    (sc.bodies || []).forEach((name, i) => bodyIndex.set(name, i));
    await buildRobot();
    loop();

    const src = new EventSource('/stream');
    src.onmessage = (ev) => {
      try { applyFrame(JSON.parse(ev.data)); }
      catch (e) { console.warn('帧解析失败', e); }
    };
    src.onerror = () => { $('msg').textContent = '显示服务断开了（宿主停了？）'; $('msg').classList.remove('gone'); };
  } catch (e) {
    $('msg').textContent = '连不上显示服务：' + (e?.message || e);
  }
}

main();
