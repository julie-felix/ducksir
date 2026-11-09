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
      }).catch((err) => {
        console.warn('mesh 加载失败，用示意盒兜底', g.mesh, err);
        const box = new THREE.Mesh(new THREE.BoxGeometry(0.014, 0.014, 0.014),
          new THREE.MeshStandardMaterial({ color: 0x8593a6, metalness: .2, roughness: .6 }));
        box.position.fromArray(g.xyz);
        box.quaternion.copy(q(g.quat));
        obj.add(box);
      }));
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
        <div class="row" style="font-size:11.5px"><span>AB 电机</span><b>${motors.map((x) => x.ab + '@' + (x.index ?? x.joint ?? '?')).join(' / ') || '-'}</b></div>
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
// 并联踝叠加层：两根定长推杆画在模型上；A/B 与 roll/pitch 双向拖拽（左右都支持）。
//
// 几何不是另存一份：直接从实体里的 `mechanisms[].coupling.params` 读
// （contract/robot/types/g1-29dof.yaml，坐标系 = ankle pitch 关节中心）。
// ---------------------------------------------------------------------------
const PAR = { sides: {}, translucent: false };

const parV = (a) => new THREE.Vector3(a[0], a[1], a[2]);
const parN = (a) => new THREE.Vector3(a[0], a[1], a[2]).normalize();
const parRot = (axis, ang) => new THREE.Quaternion().setFromAxisAngle(parN(axis), ang);

/** 实体里的 params → 这份代码用的形状（a → A）。 */
function parGeom(params) {
  return {
    chains: params.chains.map((c) => ({ A: c.a, axis: c.axis, v: c.v, rod: c.rod })),
    platform: params.platform,
  };
}

/** 平台点 C(x)：先绕 roll 轴转，再绕 pitch 轴转（与 Python solver 的 FK 一致）。 */
function parPlatformPoint(g, x, c0) {
  const P = g.platform;
  const Pp = parV(P.pitch_pivot), Pr = parV(P.roll_pivot);
  return parV(c0).sub(Pr).applyQuaternion(parRot(P.roll_axis, x[0])).add(Pr).sub(Pp)
    .applyQuaternion(parRot(P.pitch_axis, x[1])).add(Pp);
}
/** 推杆销 B(θ)。 */
function parPinPoint(g, i, theta) {
  const ch = g.chains[i];
  return parV(ch.v).applyQuaternion(parRot(ch.axis, theta)).add(parV(ch.A));
}
/** 约束残差（平方形式，和雅可比推导一致）。 */
function parRes(g, th, x, i) {
  const d = parPinPoint(g, i, th[i]).sub(parPlatformPoint(g, x, g.platform.c0[i]));
  return d.dot(d) - g.chains[i].rod ** 2;
}
function parJacobian(g, th, x) {
  const eps = 1e-6, J = [[0, 0], [0, 0]];
  for (let i = 0; i < 2; i++) {
    for (let k = 0; k < 2; k++) {
      const xp = [...x], xm = [...x];
      xp[k] += eps; xm[k] -= eps;
      J[i][k] = (parRes(g, th, xp, i) - parRes(g, th, xm, i)) / (2 * eps);
    }
  }
  return J;
}
/** 正解 θ→x：牛顿迭代。 */
function parFw(g, th, x0 = [0, 0], maxIter = 40) {
  let x = [...x0];
  for (let it = 0; it < maxIter; it++) {
    const r = [parRes(g, th, x, 0), parRes(g, th, x, 1)];
    if (Math.hypot(r[0], r[1]) < 1e-14) break;
    const J = parJacobian(g, th, x);
    const det = J[0][0] * J[1][1] - J[0][1] * J[1][0];
    if (Math.abs(det) < 1e-18) break;
    x = [x[0] - (J[1][1] * r[0] - J[0][1] * r[1]) / det,
         x[1] - (-J[1][0] * r[0] + J[0][0] * r[1]) / det];
  }
  return x;
}
/** 逆解 x→θ：αcosθ + βsinθ = k 的闭式解（每个电机独立）。 */
function parIk(g, x, hint = [0, 0]) {
  const C = [0, 1].map((i) => parPlatformPoint(g, x, g.platform.c0[i]));
  return g.chains.map((ch, i) => {
    const A = parV(ch.A), axis = parN(ch.axis), v = parV(ch.v);
    const w = C[i].clone().sub(A);
    const uv = axis.dot(v);
    const p = v.clone().addScaledVector(axis, -uv);
    const qv = new THREE.Vector3().crossVectors(axis, v);
    const alpha = w.dot(p), beta = w.dot(qv);
    const k = (w.dot(w) + v.dot(v) - ch.rod * ch.rod) / 2 - w.dot(axis) * uv;
    const Rr = Math.hypot(alpha, beta);
    if (Rr < 1e-12) return hint[i];
    const phi = Math.atan2(beta, alpha);
    const d = Math.acos(Math.max(-1, Math.min(1, k / Rr)));
    const near = (t) => Math.abs(((t - hint[i] + Math.PI) % (2 * Math.PI) + 2 * Math.PI) % (2 * Math.PI) - Math.PI);
    return near(phi + d) <= near(phi - d) ? phi + d : phi - d;
  });
}

const PAR_MAT = {
  motor: new THREE.MeshStandardMaterial({ color: 0x2c3644, metalness: .25, roughness: .6 }),
  crank: new THREE.MeshStandardMaterial({ color: 0x9fb0c4, metalness: .3, roughness: .5 }),
  pin: new THREE.MeshStandardMaterial({ color: 0xeef4fa, metalness: .1, roughness: .4 }),
};
function parCylGeo(r) {
  const g = new THREE.CylinderGeometry(r, r, 1, 10, 1, true);
  g.translate(0, 0.5, 0);
  return g;
}

function parBuildSide(geom, mirror) {
  const g = new THREE.Group();
  const rodMat = new THREE.MeshStandardMaterial({
    color: mirror ? 0xffb35a : 0x5ad1a0, emissive: mirror ? 0x3a2210 : 0x143528,
    metalness: .2, roughness: .5, side: THREE.DoubleSide });
  const nodes = geom.chains.map((ch) => {
    const A = parV(ch.A);
    const motor = new THREE.Mesh(new THREE.CylinderGeometry(0.018, 0.018, 0.034, 18), PAR_MAT.motor);
    motor.position.copy(A);
    motor.quaternion.setFromUnitVectors(new THREE.Vector3(0, 1, 0), parN(ch.axis));
    const crank = new THREE.Mesh(parCylGeo(0.0028), PAR_MAT.crank);
    const rod = new THREE.Mesh(parCylGeo(0.0035), rodMat);
    const pinB = new THREE.Mesh(new THREE.SphereGeometry(0.005, 12, 10), PAR_MAT.pin);
    const pinC = new THREE.Mesh(new THREE.SphereGeometry(0.005, 12, 10), PAR_MAT.pin);
    g.add(motor, crank, rod, pinB, pinC);
    return { A, crank, rod, pinB, pinC };
  });
  const plate = new THREE.Mesh(new THREE.BoxGeometry(0.075, 0.045, 0.006),
    new THREE.MeshStandardMaterial({ color: mirror ? 0xb5772f : 0x39505f,
      transparent: true, opacity: .55, side: THREE.DoubleSide }));
  g.add(plate);
  if (mirror) g.scale.y = -1;
  return { group: g, nodes, plate };
}

function parApply(side, th, x) {
  const g = side.geom;
  const C = [0, 1].map((i) => parPlatformPoint(g, x, g.platform.c0[i]));
  side.nodes.forEach((n, i) => {
    const B = parPinPoint(g, i, th[i]);
    const put = (mesh, a, b) => {
      const dir = b.clone().sub(a);
      mesh.position.copy(a);
      mesh.quaternion.setFromUnitVectors(new THREE.Vector3(0, 1, 0), dir.clone().normalize());
      mesh.scale.set(1, dir.length(), 1);
    };
    put(n.crank, n.A, B);
    put(n.rod, B, C[i]);
    n.pinB.position.copy(B);
    n.pinC.position.copy(C[i]);
  });
  const P = g.platform;
  const mid = C[0].clone().add(C[1]).multiplyScalar(0.5);
  const ex = C[1].clone().sub(C[0]).normalize();
  const ey = parV(P.pitch_axis).applyQuaternion(parRot(P.pitch_axis, x[1]));
  ey.addScaledVector(ex, -ey.dot(ex)).normalize();
  const ez = new THREE.Vector3().crossVectors(ex, ey).normalize();
  side.plate.matrixAutoUpdate = false;
  side.plate.matrix.copy(new THREE.Matrix4().makeBasis(ex, ey, ez).setPosition(mid));
}

/** 把解算出的 roll/pitch 写到模型关节上（右侧按镜像取反 roll）。 */
function applyJointPair(side, x) {
  setJointAngle(side.rollJoint, side.mirror ? -x[0] : x[0]);
  setJointAngle(side.pitchJoint, x[1]);
}

function parMakeUI(side, host) {
  const deg = (r) => r * 180 / Math.PI;
  const rad = (d) => d * Math.PI / 180;
  const wrap = document.createElement('div');
  wrap.className = 'par-side';
  wrap.innerHTML = `
    <div class="par-title">${side.label}</div>
    ${['A 电机', 'B 电机', 'roll', 'pitch'].map((n, k) => `
      <div class="joint"><div class="name"><span>${n}</span><span data-v="${k}">0.0°</span></div>
        <input type="range" data-k="${k}" min="${k < 2 ? -35 : -30}" max="${k < 2 ? 35 : (k === 2 ? 30 : 45)}"
               step="0.5" value="0"/></div>`).join('')}
    <div class="par-out" data-out></div>`;
  host.appendChild(wrap);
  const inputs = [...wrap.querySelectorAll('input')];
  const outs = [...wrap.querySelectorAll('[data-v]')];
  const out = wrap.querySelector('[data-out]');
  const state = { th: [0, 0], x: [0, 0] };

  const paint = () => {
    [0, 1, 2, 3].forEach((k) => {
      const v = k < 2 ? deg(state.th[k]) : deg(state.x[k - 2]);
      inputs[k].value = String(v.toFixed(1));
      outs[k].textContent = `${v.toFixed(1)}°`;
    });
    const err = Math.max(...[0, 1].map((i) =>
      Math.abs(parRes(side.geom, state.th, state.x, i)) / (2 * side.geom.chains[i].rod)));
    out.innerHTML = `<span style="color:#7f8b9b">杆长误差 ${(err * 1e9).toFixed(1)} nm`
      + (side.assumed ? ` · ${side.assumed} 项假设` : '') + `</span>`;
    parApply(side, state.th, state.x);
    applyJointPair(side, state.x);
  };

  inputs.forEach((input) => input.addEventListener('input', () => {
    const k = +input.dataset.k;
    if (k < 2) {
      state.th[k] = rad(parseFloat(input.value));
      state.x = parFw(side.geom, state.th, state.x);
    } else {
      state.x[k - 2] = rad(parseFloat(input.value));
      state.th = parIk(side.geom, state.x, state.th);
    }
    paint();
  }));

  side.state = state; side.paint = paint;
  paint();
}

/** 模型半透明：所有 mesh 材质降不透明度（保留原值以便还原）。 */
function setTranslucent(on) {
  PAR.translucent = on;
  robotGroup.traverse((o) => {
    if (!o.isMesh) return;
    const mats = Array.isArray(o.material) ? o.material : [o.material];
    for (const m of mats) {
      if (m.userData.origOpacity === undefined) m.userData.origOpacity = m.opacity;
      m.transparent = on || m.transparent;
      m.opacity = on ? 0.22 : m.userData.origOpacity;
      m.depthWrite = !on;
      m.needsUpdate = true;
    }
  });
}

async function initParallelOverlay(data) {
  const sec = $('par-sec'), panel = $('par-panel');
  if (!sec || !panel) return;
  const mechs = (data.mechanisms || []).filter((m) => (m.coupling || {}).kind === 'analytic');
  if (!mechs.length) { sec.style.display = 'none'; panel.style.display = 'none'; return; }
  sec.style.display = ''; panel.style.display = '';
  panel.innerHTML = '';
  PAR.sides = {};

  for (const mech of mechs) {
    const params = mech.coupling.params || {};
    const pr = (mech.charts || {}).pr || {};
    const joints = pr.joints || [];
    const sem = pr.semantic || [];
    const pitchJoint = joints[sem.indexOf('pitch')];
    const rollJoint = joints[sem.indexOf('roll')];
    if (!pitchJoint || !rollJoint || !current.joints.get(pitchJoint)) continue;

    const mirror = !!params.mirror;
    const side = { prefix: mech.id, label: mech.id, mirror, pitchJoint, rollJoint,
                   geom: parGeom(params), assumed: (mech.coupling.assumed || []).length };
    Object.assign(side, parBuildSide(side.geom, mirror));

    // 挂在 pitch 关节的父 link 上（不跟着 pitch 转），偏移 = pitch 子 link 在父系里的位置
    const jdef = data.joints.find((j) => j.name === pitchJoint) || {};
    const childLink = data.links.find((l) => l.name === jdef.child);
    side.group.position.set(0, 0, 0);
    if (childLink) side.group.position.set(...childLink.xyz);
    current.joints.get(pitchJoint).obj.parent.add(side.group);

    parMakeUI(side, panel);
    PAR.sides[mech.id] = side;
  }
  if (!Object.keys(PAR.sides).length) { sec.style.display = 'none'; panel.style.display = 'none'; return; }

  const tbox = document.getElementById('translucent');
  if (tbox && tbox.checked) setTranslucent(true);
  const qs = new URLSearchParams(location.search);
  for (const [key, id] of [['parL', null], ['par', null], ['parR', null]]) {
    const v = qs.get(key);
    if (!v) continue;
    const ids = Object.keys(PAR.sides);
    const target = key === 'parR' ? ids[1] : ids[0];
    if (!target) continue;
    const [a, b] = v.split(',').map((x) => parseFloat(x) || 0);
    const st = PAR.sides[target].state;
    st.th = [a * Math.PI / 180, b * Math.PI / 180];
    st.x = parFw(PAR.sides[target].geom, st.th, st.x);
    PAR.sides[target].paint();
  }
  if (qs.has('selftest')) {
    for (const [id, side] of Object.entries(PAR.sides)) {
      for (const x of [[0.2, 0.1], [-0.15, 0.25], [0.3, -0.2]]) {
        const th = parIk(side.geom, x, [0, 0]);
        const back = parFw(side.geom, th, x);
        console.log(`SELFTEST ${id} x=`, x.map((v) => +v.toFixed(4)),
          'th=', th.map((v) => +v.toFixed(5)), 'roundtrip=', back.map((v) => +v.toFixed(6)),
          'err=', Math.max(Math.abs(back[0] - x[0]), Math.abs(back[1] - x[1])).toExponential(2));
      }
    }
  }
}

(function hookTranslucent() {
  const box = document.getElementById('translucent');
  if (box) box.addEventListener('change', () => setTranslucent(box.checked));
})();
