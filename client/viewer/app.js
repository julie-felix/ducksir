// 数字描述查看器：读实体导出的 JSON，自己算 FK（不依赖 MuJoCo）。
// 结构：父 link 组 → 关节原点变换 → 关节转动 → 几何 + 子 link。
import * as THREE from 'three';
import { OrbitControls } from './vendor/three/controls/OrbitControls.js';
import { GLTFLoader } from './vendor/three/loaders/GLTFLoader.js';

const $ = (id) => document.getElementById(id);
addEventListener('error', (e) => { const b = $('busy'); if (b) { b.style.display = 'grid'; b.textContent = '出错：' + (e.message || e.error); } });
addEventListener('unhandledrejection', (e) => { const b = $('busy'); if (b) { b.style.display = 'grid'; b.textContent = '加载失败：' + (e.reason && (e.reason.message || e.reason)); } });
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
// 注：GLB 若带 Draco（step2glb 默认会压）必须挂 DRACOLoader；
// 我们的 CAD 资产用 tools/cad 流程先剥掉 Draco，所以这里不需要。
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

/**
 * 量「模型自身」的最低点：先把整组挪回 y=0 再测，否则量到的是含位移的世界包围盒，
 * 每帧自减一次就会来回跳（这就是拖滑条时机器人「闪」的原因）。
 * @returns {number} 把最低点压到 y=0 需要的位移。
 */
function measureLift() {
  const keep = robotGroup.position.y;
  robotGroup.position.y = 0;
  robotGroup.updateMatrixWorld(true);
  const box = new THREE.Box3().setFromObject(robotGroup);
  const lift = box.isEmpty() ? 0 : -box.min.y;
  robotGroup.position.y = keep;
  return lift;
}

function scheduleGroundClamp() {
  if (!groundClamp || clampPending) return;
  clampPending = true;
  requestAnimationFrame(() => {
    clampPending = false;
    if (!current) return;
    current.groundOffset = measureLift();
    robotGroup.position.y = current.groundOffset;
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
  const lift = measureLift();
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
  const mechs = data.mechanisms || []
  $('mechanisms').innerHTML = mechs.length
    ? mechs.map((m) => {
      const pr = ((m.charts || {}).pr || {}).joints || []
      const motors = ((m.charts || {}).ab || {}).motors || []
      const kind = ((m.coupling || {}).kind) || '?'
      const label = kind === 'unresolved' ? '未解算（只显示 PR 串联等效）' : kind
      return `<div class="joint"><div class="name"><span>${m.id}</span><span>${kind}</span></div>
        <div class="row" style="font-size:11.5px"><span>平台</span><b>${m.platform || '-'}</b></div>
        <div class="row" style="font-size:11.5px"><span>PR 关节</span><b>${pr.join(' / ') || '-'}</b></div>
        <div class="row" style="font-size:11.5px"><span>AB 电机</span><b>${motors.map((x) => x.ab + '@' + x.index).join(' / ') || '-'}</b></div>
        <div class="warn">${label}</div></div>`
    }).join('')
    : '<div class="row">无（开链机型）</div>'

  $('warnings').innerHTML = (data.warnings || []).length
    ? data.warnings.map((w) => `<div class="warn">${w.code} · ${w.path}<br>${w.message}</div>`).join('')
    : '<div class="row">无</div>';
}

async function showStatic(meta) {
  $('busy').style.display = 'grid';
  robotGroup.clear();
  current = null;
  $('busy').textContent = '下载 GLB…';
  const gltf = await loader.loadAsync(meta.glb);
  $('busy').textContent = '解析完成，建场景…';
  robotGroup.add(gltf.scene);
  robotGroup.rotation.x = 0;
  robotGroup.position.set(0, 0, 0);
  const box = new THREE.Box3().setFromObject(gltf.scene);
  const size = box.getSize(new THREE.Vector3());
  robotGroup.position.y -= box.min.y;
  // 轴线叠加：毫米 → 米；点侧栏条目可以「只看这一条」，静态图看不清的靠这个解决
  if (meta.axes && meta.axes.length) {
    const group = new THREE.Group();
    window.__axisObjs = new Map();
    for (const a of meta.axes) {
      const src = a.center || a.point; // center 是「投影到零件中心」的点，画针更贴
      const p = new THREE.Vector3(src[0] / 1000, src[1] / 1000, src[2] / 1000);
      const d = new THREE.Vector3(a.dir[0], a.dir[1], a.dir[2]).normalize();
      // 用细圆柱代替线段：WebGL 的 linewidth 不生效，1px 线在实体上是「看不见」的
      const half = 0.075;
      const geo = new THREE.CylinderGeometry(0.0015, 0.0015, half * 2, 12);
      const line = new THREE.Mesh(geo, new THREE.MeshBasicMaterial({ color: a.color, depthTest: false, transparent: true, opacity: 0.95 }));
      line.renderOrder = 999;  // 穿过实体也能看见，否则针藏在孔里等于没画
      line.position.copy(p);
      line.quaternion.setFromUnitVectors(new THREE.Vector3(0, 1, 0), d);
      const c = document.createElement('canvas');
      c.width = 160; c.height = 80;
      const ctx = c.getContext('2d');
      ctx.fillStyle = 'rgba(8,10,14,0.85)'; ctx.fillRect(0, 0, 160, 80);
      ctx.fillStyle = a.color; ctx.font = 'bold 46px monospace';
      ctx.fillText(a.label, 10, 56);
      const tex = new THREE.CanvasTexture(c);
      const sprite = new THREE.Sprite(new THREE.SpriteMaterial({ map: tex, depthTest: false }));
      sprite.scale.set(0.055, 0.0275, 1);
      sprite.position.copy(p);  // 贴住铰点本身；之前沿轴偏移 10 cm，标签看着像「没标到」
      group.add(line);
      group.add(sprite);
      window.__axisObjs.set(a.label, { line, sprite, base: a.color });
    }
    robotGroup.add(group);
  }

  // 默认只显示第一条：这个机构 17 条轴挤在 10 cm 里，全显示必然重叠
  const wantedAxis = new URLSearchParams(location.search).get('axis');
  let selectedAxis = wantedAxis || (meta.axes && meta.axes[0] && meta.axes[0].label) || null;
  const axisRows = (meta.axes || []).map((a) =>
    `<div class="row axis-row" data-axis="${a.label}" style="cursor:pointer;font-size:11.5px">` +
    `<span style="color:${a.color}">${a.label}</span><b>r=${a.r_mm}mm</b></div>`).join('');
  $('joints').innerHTML = axisRows
    ? '<div class="row" style="font-size:11px">点一条 = 只看它，再点恢复</div>' + axisRows
    : '<div class="row">静态模型（无关节）</div>';
  function applyAxisSelection() {
    for (const [name, o] of window.__axisObjs) {
      const on = !selectedAxis || selectedAxis === name;
      o.line.material.color.set(on ? o.base : '#232a33');
      o.sprite.visible = on;
      o.line.scale.setScalar(on ? 1 : 0.5);
    }
    for (const row of document.querySelectorAll('.axis-row')) {
      const on = row.dataset.axis === selectedAxis;
      row.style.background = on ? 'rgba(77,163,255,0.14)' : 'transparent';
      row.style.borderRadius = '4px';
    }
  }
  for (const row of document.querySelectorAll('.axis-row')) {
    row.addEventListener('click', () => {
      const label = row.dataset.axis;
      selectedAxis = selectedAxis === label ? null : label;
      applyAxisSelection();
    });
  }
  applyAxisSelection();
  $('mechanisms').innerHTML = '<div class="row">—</div>';
  $('tree').textContent = '（静态 GLB，无骨架）';
  $('summary').innerHTML = `
    <div class="row"><span>格式</span><b>STEP → GLB</b></div>
    <div class="row"><span>包围盒</span><b>${size.x.toFixed(2)} × ${size.y.toFixed(2)} × ${size.z.toFixed(2)} m</b></div>
    <div class="row"><span>节点</span><b>${gltf.scene.children.length}</b></div>`;
  $('warnings').innerHTML = '<div class="warn">这份 CAD 由 step2glb 从 STEP 转换，许可未知，只用于判读，不进实体</div>';
  fitView();
  $('busy').style.display = 'none';
}

async function show(meta) {
  if (meta.static) return showStatic(meta);
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
  const focus = new URLSearchParams(location.search).get('focus');
  if (focus) {
    const jn = current.joints.get(focus);
    if (jn) {
      const p = new THREE.Vector3();
      jn.obj.getWorldPosition(p);
      controls.target.copy(p);
      camera.position.copy(p).add(new THREE.Vector3(0.34, -0.42, 0.16));
      camera.near = 0.005; camera.far = 20; camera.updateProjectionMatrix();
      controls.update();
    }
  }
  $('busy').style.display = 'none';
  initParallelOverlay(data).catch((e) => console.warn('并联叠加层初始化失败', e));
}

/** 一键切到标准视角：alongX（沿销轴看）/ alongY（侧看）/ top（俯视）。 */
function setView(kind) {
  const box = new THREE.Box3().setFromObject(robotGroup);
  if (box.isEmpty()) return;
  const c = box.getCenter(new THREE.Vector3());
  const size = box.getSize(new THREE.Vector3());
  const dist = Math.max(size.x, size.y, size.z) * 1.9;
  const dir = kind === 'alongX' ? new THREE.Vector3(1, 0, 0)
    : kind === 'alongY' ? new THREE.Vector3(0, 1, 0)
      : new THREE.Vector3(0, 0, 1);
  camera.position.copy(c).addScaledVector(dir, dist);
  camera.up.set(0, kind === 'top' ? 1 : 0, kind === 'top' ? 0 : 1);
  if (kind === 'top') camera.up.set(0, 1, 0);
  camera.lookAt(c);
  controls.target.copy(c);
  controls.update();
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
    opt.textContent = r.static ? r.name : `${r.name}（${r.links} link / ${r.joints} joint）`;
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
  for (const [id, kind] of [['view-x', 'alongX'], ['view-y', 'alongY'], ['view-top', 'top']]) {
    const el = $(id);
    if (el) el.addEventListener('click', () => setView(kind));
  }
  $('ground').addEventListener('change', (e) => {
    groundClamp = e.target.checked;
    if (!current) return;
    if (groundClamp) scheduleGroundClamp();
    else robotGroup.position.y = 0;
  });
  const first = index.robots.find((r) => r.id === wanted) || index.robots[0];
  if (first) { sel.value = first.id; await show(first); }
})();

// ---------------------------------------------------------------------------
// 并联踝叠加层：把两根定长推杆画到模型上，拖 A/B 电机角，脚掌按解算动。
// 几何数据来自 tools/parallel/export_overlay.py（坐标系 = ankle pitch 关节中心）。
// ---------------------------------------------------------------------------
const PAR = { data: null, group: null, th: [0, 0], meshes: null, roll: 0, pitch: 0 };

const rotAxis = (axis, ang) => new THREE.Quaternion().setFromAxisAngle(
  new THREE.Vector3(axis[0], axis[1], axis[2]).normalize(), ang);

function parPlatformPoint(x, c0) {
  const P = PAR.data.platform;
  const Pp = new THREE.Vector3(...P.pitch_pivot);
  const Pr = new THREE.Vector3(...P.roll_pivot);
  const qp = rotAxis(P.pitch_axis, x[1]);
  const qr = rotAxis(P.roll_axis, x[0]);
  const v = new THREE.Vector3(...c0).sub(Pr).applyQuaternion(qr).add(Pr).sub(Pp)
    .applyQuaternion(qp).add(Pp);
  return v;
}

function parPinPoint(i, theta) {
  const ch = PAR.data.chains[i];
  const A = new THREE.Vector3(...ch.A);
  return new THREE.Vector3(...ch.v).applyQuaternion(rotAxis(ch.axis, theta)).add(A);
}

function parRodErr(th, i, x) {
  const c = parPlatformPoint(x, PAR.data.platform.c0[i]);
  return parPinPoint(i, th[i]).distanceTo(c) - PAR.data.chains[i].rod;
}

/** 正解：牛顿迭代（平方残差），和 Python 里 solver.fw 同一套。 */
function parFw(th, x0 = [0, 0]) {
  let x = [...x0];
  for (let it = 0; it < 40; it++) {
    const r = [0, 1].map((i) => {
      const c = parPlatformPoint(x, PAR.data.platform.c0[i]);
      const d = parPinPoint(i, th[i]).sub(c);
      return d.dot(d) - PAR.data.chains[i].rod ** 2;
    });
    if (Math.hypot(r[0], r[1]) < 1e-14) return x;
    const eps = 1e-6, J = [[0, 0], [0, 0]];
    for (let i = 0; i < 2; i++) {
      for (let k = 0; k < 2; k++) {
        const xp = [...x], xm = [...x];
        xp[k] += eps; xm[k] -= eps;
        J[i][k] = (parRodErr2(th, i, xp) - parRodErr2(th, i, xm)) / (2 * eps);
      }
    }
    const det = J[0][0] * J[1][1] - J[0][1] * J[1][0];
    if (Math.abs(det) < 1e-18) break;
    const dx = [-(J[1][1] * r[0] - J[0][1] * r[1]) / det,
                -(-J[1][0] * r[0] + J[0][0] * r[1]) / det];
    x = [x[0] + dx[0], x[1] + dx[1]];
  }
  return x;
}
function parRodErr2(th, i, x) {              // 平方形式，给牛顿用
  const c = parPlatformPoint(x, PAR.data.platform.c0[i]);
  const d = parPinPoint(i, th[i]).sub(c);
  return d.dot(d) - PAR.data.chains[i].rod ** 2;
}

function buildOverlayGeometry() {
  const g = new THREE.Group();
  const mat = (c, o = {}) => new THREE.MeshStandardMaterial({ color: c, metalness: .2, roughness: .5, ...o });
  const cyl = (r) => { const geo = new THREE.CylinderGeometry(r, r, 1, 10, 1, true);
    geo.translate(0, 0.5, 0); return geo; };
  const nodes = [];
  PAR.data.chains.forEach((ch, i) => {
    const A = new THREE.Vector3(...ch.A);
    const motor = new THREE.Mesh(new THREE.CylinderGeometry(0.018, 0.018, 0.034, 18), mat(0x2c3644));
    motor.position.copy(A);
    motor.quaternion.setFromUnitVectors(new THREE.Vector3(0, 1, 0),
      new THREE.Vector3(...ch.axis).normalize());
    g.add(motor);
    const crank = new THREE.Mesh(cyl(0.0028), mat(0x9fb0c4));
    const rod = new THREE.Mesh(cyl(0.0035), mat(0x5ad1a0, { emissive: 0x143528 }));
    const pinB = new THREE.Mesh(new THREE.SphereGeometry(0.005, 12, 10), mat(0xeef4fa));
    const pinC = new THREE.Mesh(new THREE.SphereGeometry(0.005, 12, 10), mat(0xeef4fa));
    g.add(crank, rod, pinB, pinC);
    nodes.push({ A, crank, rod, pinB, pinC });
  });
  const plate = new THREE.Mesh(new THREE.BoxGeometry(0.1, 0.06, 0.008), mat(0x39505f, { transparent: true, opacity: .85 }));
  g.add(plate);
  return { group: g, nodes, plate };
}

function updateOverlay() {
  if (!PAR.meshes) return;
  const th = PAR.th;
  const x = parFw(th, [PAR.roll, PAR.pitch]);
  PAR.roll = x[0]; PAR.pitch = x[1];
  const C = [0, 1].map((i) => parPlatformPoint(x, PAR.data.platform.c0[i]));
  PAR.meshes.nodes.forEach((n, i) => {
    const B = parPinPoint(i, th[i]);
    const place = (mesh, a, b, r) => {
      const dir = b.clone().sub(a);
      mesh.position.copy(a);
      mesh.quaternion.setFromUnitVectors(new THREE.Vector3(0, 1, 0), dir.clone().normalize());
      mesh.scale.set(1, dir.length(), 1);
    };
    place(n.crank, n.A, B);
    place(n.rod, B, C[i]);
    n.pinB.position.copy(B);
    n.pinC.position.copy(C[i]);
  });
  const P = PAR.data.platform;
  const Pp = new THREE.Vector3(...P.pitch_pivot), Pr = new THREE.Vector3(...P.roll_pivot);
  const mid = C[0].clone().add(C[1]).multiplyScalar(0.5);
  const ex = C[1].clone().sub(C[0]).normalize();
  const ey = new THREE.Vector3(...P.pitch_axis).applyQuaternion(rotAxis(P.pitch_axis, x[1]));
  ey.addScaledVector(ex, -ey.dot(ex)).normalize();
  const ez = new THREE.Vector3().crossVectors(ex, ey).normalize();
  PAR.meshes.plate.matrixAutoUpdate = false;
  PAR.meshes.plate.matrix.copy(new THREE.Matrix4().makeBasis(ex, ey, ez).setPosition(mid));
  // 把脚掌摆到解出来的 roll/pitch
  setJointAngle('left_ankle_roll_joint', x[0]);
  setJointAngle('left_ankle_pitch_joint', x[1]);
  const out = $('par-out');
  if (out) out.innerHTML = `roll <b>${(x[0] * 180 / Math.PI).toFixed(1)}°</b> · `
    + `pitch <b>${(x[1] * 180 / Math.PI).toFixed(1)}°</b><br>`
    + `<span style="color:#7f8b9b">杆长误差 ${(Math.max(Math.abs(parRodErr(th, 0, x)), Math.abs(parRodErr(th, 1, x))) * 1e9).toFixed(1)} nm</span>`;
}

async function initParallelOverlay(data) {
  const sec = $('par-sec'), panel = $('par-panel');
  if (!sec || !panel) return;
  if (data.id !== 'g1-29dof') { sec.style.display = 'none'; panel.style.display = 'none'; return; }
  if (!PAR.data) {
    try {
      PAR.data = await (await fetch('./data/parallel-g1-overlay.json')).json();
    } catch (e) { console.warn('并联叠加层数据缺失', e); return; }
  }
  const pitchJoint = current.joints.get('left_ankle_pitch_joint');
  if (!pitchJoint) return;
  const parentObj = pitchJoint.obj.parent;      // 挂在膝 link 上，避免跟着 pitch 转
  if (PAR.group) parentObj.remove(PAR.group);
  PAR.meshes = buildOverlayGeometry();
  PAR.group = PAR.meshes.group;
  PAR.group.position.set(0, -9.4445e-5, -0.30001);   // ankle pitch 关节中心在膝坐标系里的位置
  parentObj.add(PAR.group);
  sec.style.display = ''; panel.style.display = '';

  panel.innerHTML = `
    <div class="joint"><div class="name"><span>A 电机（右）</span><span data-a>0.0°</span></div>
      <input type="range" min="-35" max="35" step="0.5" value="0" id="par-a"></div>
    <div class="joint"><div class="name"><span>B 电机（左）</span><span data-b>0.0°</span></div>
      <input type="range" min="-35" max="35" step="0.5" value="0" id="par-b"></div>
    <div id="par-out" style="font-size:12px;color:#d7dee8;padding:4px 2px"></div>`;
  const sync = () => {
    PAR.th = [+$('par-a').value * Math.PI / 180, +$('par-b').value * Math.PI / 180];
    panel.querySelector('[data-a]').textContent = `${$('par-a').value}°`;
    panel.querySelector('[data-b]').textContent = `${$('par-b').value}°`;
    updateOverlay();
  };
  const preset = new URLSearchParams(location.search).get('par');
  if (preset) {
    const [a, b] = preset.split(',').map((v) => parseFloat(v) || 0);
    $('par-a').value = String(a); $('par-b').value = String(b);
  }
  $('par-a').addEventListener('input', sync);
  $('par-b').addEventListener('input', sync);
  sync();
}
