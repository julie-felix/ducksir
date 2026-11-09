// 数字描述查看器：读实体导出的 JSON，自己算 FK（不依赖 MuJoCo）。
// 结构：父 link 组 → 关节原点变换 → 关节转动 → 几何 + 子 link。
import * as THREE from 'three';
import { OrbitControls } from './vendor/three/controls/OrbitControls.js';
import { GLTFLoader } from './vendor/three/loaders/GLTFLoader.js';

const $ = (id) => document.getElementById(id);
const canvas = $('c');
const renderer = new THREE.WebGLRenderer({ canvas, antialias: true });
renderer.setPixelRatio(Math.min(devicePixelRatio, 2));
renderer.outputColorSpace = THREE.SRGBColorSpace;

const scene = new THREE.Scene();
scene.background = new THREE.Color(0x0f1216);
scene.add(new THREE.HemisphereLight(0xdfe9ff, 0x20262e, 1.15));
const key = new THREE.DirectionalLight(0xffffff, 1.5);
key.position.set(2.5, 4, 2);
scene.add(key);
const fill = new THREE.DirectionalLight(0x88aaff, 0.55);
fill.position.set(-3, 1.5, -2);
scene.add(fill);

const camera = new THREE.PerspectiveCamera(38, 1, 0.01, 200);
camera.position.set(1.1, 0.9, 1.6);
const controls = new OrbitControls(camera, canvas);
controls.enableDamping = true;
controls.dampingFactor = 0.08;

const grid = new THREE.GridHelper(6, 60, 0x2a333f, 0x1b222a);
scene.add(grid);

const robotGroup = new THREE.Group();
// 实体是 Z 上（x 前 / y 左 / z 上），three 是 Y 上：这里只做显示换算，不改实体
robotGroup.rotation.x = -Math.PI / 2;
scene.add(robotGroup);

const loader = new GLTFLoader();
const glbCache = new Map();
const materialCache = new Map();

function materialFor(rgba) {
  const k = rgba.map((v) => v.toFixed(3)).join(',');
  if (!materialCache.has(k)) {
    materialCache.set(k, new THREE.MeshStandardMaterial({
      color: new THREE.Color(rgba[0], rgba[1], rgba[2]),
      metalness: 0.15,
      roughness: 0.62,
      transparent: rgba[3] < 1,
      opacity: rgba[3],
    }));
  }
  return materialCache.get(k);
}

async function loadGlb(url) {
  if (!glbCache.has(url)) glbCache.set(url, loader.loadAsync(url));
  return glbCache.get(url);
}

function resize() {
  const r = canvas.getBoundingClientRect();
  renderer.setSize(r.width, r.height, false);
  camera.aspect = r.width / Math.max(r.height, 1);
  camera.updateProjectionMatrix();
}
addEventListener('resize', resize);

// ---------- 状态 ----------
let current = null; // { data, joints: Map(name -> {obj, qOrigin, axis, type, angle}), groundOffset: number }
let groundClamp = true;
let clampPending = false;

function scheduleGroundClamp() {
  if (!groundClamp || clampPending) return;
  clampPending = true;
  requestAnimationFrame(() => {
    clampPending = false;
    robotGroup.updateMatrixWorld(true);
    const box = new THREE.Box3().setFromObject(robotGroup);
    if (!box.isEmpty()) robotGroup.position.y = (current?.groundOffset ?? 0) - box.min.y;
  });
}

function q(quat) {
  // 实体里的四元数是 wxyz，three 是 xyzw
  return new THREE.Quaternion(quat[1], quat[2], quat[3], quat[0]);
}

function fitView(padding = 1.7) {
  const box = new THREE.Box3().setFromObject(robotGroup);
  if (box.isEmpty()) return;
  const size = box.getSize(new THREE.Vector3());
  const center = box.getCenter(new THREE.Vector3());
  const dist = Math.max(size.x, size.y, size.z) * padding;
  controls.target.copy(center);
  camera.position.set(center.x + dist * 0.8, center.y + dist * 0.25, center.z + dist);
  camera.near = Math.max(0.005, dist / 100);
  camera.far = dist * 30;
  camera.updateProjectionMatrix();
  controls.update();
}

function setJointAngle(jointName, angle) {
  const j = current.joints.get(jointName);
  if (!j) return;
  j.angle = angle;
  if (j.type === 'prismatic') {
    const v = j.axis.clone().multiplyScalar(angle).applyQuaternion(j.qOrigin);
    j.obj.position.copy(j.origin).add(v);
  } else {
    j.obj.quaternion.copy(j.qOrigin).multiply(
      new THREE.Quaternion().setFromAxisAngle(j.axis, angle));
  }
  scheduleGroundClamp();
}

async function buildRobot(data) {
  robotGroup.clear();
  const jointsByChild = new Map();
  for (const j of data.joints) jointsByChild.set(j.child, j);

  const byName = new Map(data.links.map((l) => [l.name, l]));
  const objects = new Map();
  const jointState = new Map();
  const jobs = [];

  const walk = (link, parentObj) => {
    const jdef = jointsByChild.get(link.name);
    const obj = new THREE.Group();
    obj.name = link.name;

    if (!link.parent || link.parent === 'world') {
      obj.position.fromArray(link.xyz);
      obj.quaternion.copy(q(link.quat));
    } else {
      const origin = new THREE.Vector3().fromArray(link.xyz);
      const qOrigin = q(link.quat);
      obj.position.copy(origin);
      obj.quaternion.copy(qOrigin);
      if (jdef) {
        const axis = new THREE.Vector3().fromArray(jdef.axis).normalize();
        jointState.set(jdef.name, {
          obj, origin, qOrigin, axis, type: jdef.type, angle: 0,
          lower: jdef.lower, upper: jdef.upper, def: jdef,
        });
      }
    }
    parentObj.add(obj);
    objects.set(link.name, obj);

    for (const g of link.geoms || []) {
      jobs.push(loadGlb(g.mesh).then((gltf) => {
        const inst = gltf.scene.clone(true);
        inst.position.fromArray(g.xyz);
        inst.quaternion.copy(q(g.quat));
        const mat = materialFor(g.color);
        inst.traverse((o) => {
          if (o.isMesh) { o.material = mat; o.castShadow = false; o.receiveShadow = false; }
        });
        obj.add(inst);
      }).catch((err) => console.warn('mesh 加载失败', g.mesh, err)));
    }
    return obj;
  };

  const rootLink = byName.get(data.root) || data.links.find((l) => !l.parent || l.parent === 'world');
  const build = (link) => {
    const parentObj = link.parent && link.parent !== 'world' ? objects.get(link.parent) : robotGroup;
    const obj = walk(link, parentObj);
    for (const child of data.links) {
      if (child.parent === link.name) build(child);
    }
    return obj;
  };
  build(rootLink);

  current = { data, joints: jointState, groundOffset: 0 };
  // 第一次赋值：需要 current 已就绪
  for (const name of jointState.keys()) setJointAngle(name, 0);

  await Promise.all(jobs);

  // 落地：把最低点挪到 y=0，并把偏移记下来给「贴地」用
  robotGroup.updateMatrixWorld(true);
  const box = new THREE.Box3().setFromObject(robotGroup);
  const lift = box.isEmpty() ? 0 : -box.min.y;
  robotGroup.position.y = lift;
  if (current) current.groundOffset = lift;
  return jointState;
}

function renderJointList(data) {
  const wrap = $('joints');
  wrap.innerHTML = '';
  for (const j of data.joints) {
    if (!j.actuated) continue;
    const lo = j.lower ?? -Math.PI / 2;
    const hi = j.upper ?? Math.PI / 2;
    const div = document.createElement('div');
    div.className = 'joint';
    div.innerHTML = `
      <div class="name"><span>${j.name}</span><span data-val>0.0°</span></div>
      <input type="range" min="${lo}" max="${hi}" step="0.005" value="0"
             data-joint="${j.name}" aria-label="${j.name}" />`;
    const input = div.querySelector('input');
    const out = div.querySelector('[data-val]');
    input.addEventListener('input', () => {
      const a = parseFloat(input.value);
      setJointAngle(j.name, a);
      out.textContent = `${(a * 180 / Math.PI).toFixed(1)}°`;
    });
    wrap.appendChild(div);
  }
}

function renderTree(data) {
  const kids = new Map();
  for (const l of data.links) {
    const p = l.parent || null;
    if (!kids.has(p)) kids.set(p, []);
    kids.get(p).push(l);
  }
  const lines = [];
  const walk = (parent, depth) => {
    for (const l of kids.get(parent) || []) {
      lines.push(`${'  '.repeat(depth)}${depth ? '└ ' : ''}${l.name}${l.mass ? `  ${(l.mass * 1000).toFixed(0)}g` : ''}`);
      walk(l.name, depth + 1);
    }
  };
  walk(null, 0);
  walk('world', 0);
  $('tree').textContent = lines.join('\n');
}

function renderMeta(data) {
  const s = data.summary || {};
  $('summary').innerHTML = `
    <div class="row"><span>link</span><b>${s.links ?? '-'}</b></div>
    <div class="row"><span>关节（驱动）</span><b>${s.joints ?? '-'}（${s.actuated_joints ?? '-'}）</b></div>
    <div class="row"><span>整机质量</span><b>${s.total_mass ? (s.total_mass * 1000).toFixed(1) + ' g' : '-'}</b></div>
    <div class="row"><span>根 / 挂接</span><b>${data.root} / ${data.world_attachment}</b></div>`;
  $('warnings').innerHTML = (data.warnings || []).length
    ? data.warnings.map((w) => `<div class="warn">${w.code} · ${w.path}<br>${w.message}</div>`).join('')
    : '<div class="row">无</div>';
}

async function show(meta) {
  $('busy').style.display = 'grid';
  const data = await (await fetch(`./data/${meta.file}`)).json();
  robotGroup.position.set(0, 0, 0);
  await buildRobot(data);
  renderJointList(data);
  // ?pose=knee_l:-40,hip_l:20（角度制），用于截图核对 FK，也可当姿态快照的雏形
  const pose = new URLSearchParams(location.search).get('pose');
  if (pose) {
    for (const pair of pose.split(',')) {
      const [name, deg] = pair.split(':');
      const angle = (parseFloat(deg) || 0) * Math.PI / 180;
      setJointAngle(name.trim(), angle);
      const input = document.querySelector(`input[data-joint="${name.trim()}"]`);
      if (input) {
        input.value = String(angle);
        input.parentElement.querySelector('[data-val]').textContent = `${(angle * 180 / Math.PI).toFixed(1)}°`;
      }
    }
  }
  renderTree(data);
  renderMeta(data);
  fitView();
  $('busy').style.display = 'none';
}

function animate() {
  requestAnimationFrame(animate);
  controls.update();
  renderer.render(scene, camera);
}

(async function main() {
  resize();
  animate();
  const index = await (await fetch('./data/index.json')).json();
  const sel = $('robot');
  const wanted = new URLSearchParams(location.search).get('robot');
  for (const r of index.robots) {
    const opt = document.createElement('option');
    opt.value = r.id;
    opt.textContent = `${r.name}（${r.links} link / ${r.joints} joint）`;
    sel.appendChild(opt);
  }
  sel.addEventListener('change', () => {
    const meta = index.robots.find((r) => r.id === sel.value);
    if (meta) show(meta);
  });
  $('reset').addEventListener('click', () => {
    for (const name of current?.joints.keys() ?? []) {
      setJointAngle(name, 0);
      const input = document.querySelector(`input[data-joint="${name}"]`);
      if (input) { input.value = 0; input.parentElement.querySelector('[data-val]').textContent = '0.0°'; }
    }
  });
  $('fit').addEventListener('click', () => fitView());
  $('ground').addEventListener('change', (e) => {
    groundClamp = e.target.checked;
    if (!current) return;
    if (groundClamp) scheduleGroundClamp();
    else robotGroup.position.y = current.groundOffset;
  });
  const first = index.robots.find((r) => r.id === wanted) || index.robots[0];
  if (first) { sel.value = first.id; await show(first); }
})();
