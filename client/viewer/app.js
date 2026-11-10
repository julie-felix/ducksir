// 数字描述查看器：读实体导出的 JSON，自己算 FK（不依赖 MuJoCo）。
// 结构：父 link 组 → 关节原点变换 → 关节转动 → 几何 + 子 link。
import * as THREE from 'three';
import { OrbitControls } from './vendor/three/controls/OrbitControls.js';
import { GLTFLoader } from './vendor/three/loaders/GLTFLoader.js';

const $ = (id) => document.getElementById(id);

const SPLIT = { on: false, renderer: null, scene: null, camera: null, controls: null,
                objs: [], grid: new Map(), data: new Map() };

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

function setJointIn(joints, jointName, angle) {
  const j = joints.get(jointName);
  if (!j) return;
  j.angle = angle;
  if (j.type === 'prismatic') {
    const v = j.axis.clone().multiplyScalar(angle).applyQuaternion(j.qOrigin);
    j.obj.position.copy(j.origin).add(v);
  } else {
    j.obj.quaternion.copy(j.qOrigin).multiply(
      new THREE.Quaternion().setFromAxisAngle(j.axis, angle));
  }
}

function setJointAngle(jointName, angle) {
  if (!current) return;
  setJointIn(current.joints, jointName, angle);
  scheduleGroundClamp();
}

async function buildInto(data, group) {
  group.clear();
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
    const parentObj = link.parent && link.parent !== 'world' ? objects.get(link.parent) : group;
    const obj = walk(link, parentObj);
    for (const child of data.links) {
      if (child.parent === link.name) build(child);
    }
    return obj;
  };
  build(rootLink);
  await Promise.all(jobs);
  return { joints: jointState, objects };
}

async function buildRobot(data) {
  robotGroup.clear();
  const { joints } = await buildInto(data, robotGroup);
  current = { data, joints, groundOffset: 0 };
  for (const name of joints.keys()) setJointAngle(name, 0);
  const lift = measureLift();
  robotGroup.position.y = lift;
  if (current) current.groundOffset = lift;
  return joints;
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
  splitReset();                       // 换机型：右半边必须重来，否则会拿旧的机器人对比
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
  {
    const vq = new URLSearchParams(location.search).get('view');
    if (vq) setTimeout(() => setView(vq), 200);
  }
  {
    const tb = document.getElementById('translucent');
    if (tb) {
      if (new URLSearchParams(location.search).has('opaque')) tb.checked = false;
      setTranslucent(tb.checked);
    }
  }
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
  initParallelOverlay(data)
    .then(() => splitEnsure())
    .catch((e) => console.warn('并联叠加层初始化失败', e));
}

/** 一键切到标准视角：alongX（沿销轴看）/ alongY（侧看）/ top（俯视）。 */
/**
 * 标准视角。方向要按**机器人自身的轴**算：实体是 Z-up（x 前 / y 左 / z 上），
 * 显示时整机绕 X 转了 -90°，所以不能直接拿世界坐标系当机身轴。
 *   alongX = 沿销轴看（机构最清楚）
 *   alongY = 侧看（从机器人左侧看）
 *   top    = 俯视（从机身正上方看）
 */
function setView(kind) {
  const box = new THREE.Box3().setFromObject(robotGroup);
  if (box.isEmpty()) return;
  const c = box.getCenter(new THREE.Vector3());
  const size = box.getSize(new THREE.Vector3());
  const dist = Math.max(size.x, size.y, size.z) * 1.9;

  const q = new THREE.Quaternion();
  robotGroup.getWorldQuaternion(q);

  const bodyDir = kind === 'alongX' ? new THREE.Vector3(1, 0, 0)   // 机身 x（前）
    : kind === 'alongY' ? new THREE.Vector3(0, 1, 0)               // 机身 y（左）
      : new THREE.Vector3(0, 0, 1);                                // 机身 z（上）

  // up 固定用世界 +Y（= 机身 +z）。俯视时相机若正好落在 up 轴上，OrbitControls 的极角会退化
  // （万向锁：拖动时画面乱翻），所以俯视故意偏 6°。
  camera.up.set(0, 1, 0);
  let dir = bodyDir.clone().applyQuaternion(q).normalize();
  if (kind === 'top') {
    const tiltAxis = new THREE.Vector3(1, 0, 0).applyQuaternion(q).normalize();
    const a = 6 * Math.PI / 180;
    dir.multiplyScalar(Math.cos(a)).addScaledVector(tiltAxis, Math.sin(a)).normalize();
  }
  camera.position.copy(c).addScaledVector(dir, dist);
  camera.lookAt(c);
  controls.target.copy(c);
  controls.update();
  if (SPLIT.on && SPLIT.ready) camSync(camera, controls, SPLIT.camera, SPLIT.controls);
}


// ---------------------------------------------------------------------------
// 轨迹回放：读 sim/host/pd_hold.py --trajectory 导出的 json
//
// 帧里只有「自由基座位姿 + 驱动关节角」，其余全部复用页面自己的 FK —— 这样回放看到的就是
// 实体描述本身，不是另一套渲染。贴地（groundClamp）自动关掉：回放要的是真实位姿。
// ---------------------------------------------------------------------------
const REPLAY = { on: false, frames: [], meta: null, i: 0, playing: false, fps: 50, acc: 0, base: null };

function replayQuat(qwxyz) {
  // 实体 Z-up → three Y-up：先把整组按 -90° 绕 x 转过来，再叠加实体的旋转
  return new THREE.Quaternion().setFromEuler(new THREE.Euler(-Math.PI / 2, 0, 0))
    .multiply(new THREE.Quaternion(qwxyz[1], qwxyz[2], qwxyz[3], qwxyz[0]));
}

function applyReplayFrame(i) {
  if (!current || !REPLAY.frames.length) return;
  const f = REPLAY.frames[Math.max(0, Math.min(i, REPLAY.frames.length - 1))];
  const names = REPLAY.meta.joints;
  for (let k = 0; k < names.length; k++) setJointIn(current.joints, names[k], f.q[k]);
  // 实体坐标 (x,y,z) → three 世界 (x, z, -y)
  robotGroup.position.set(f.pos[0], f.pos[2], -f.pos[1]);
  robotGroup.quaternion.copy(replayQuat(f.quat));
  REPLAY.i = i;
  const slider = $('replay-t');
  if (slider && Number(slider.value) !== i) slider.value = String(i);
  const info = $('replay-info');
  if (info) info.textContent = `${f.t.toFixed(2)} s / ${REPLAY.frames[REPLAY.frames.length - 1].t.toFixed(2)} s`;
}

function stopReplay() {
  REPLAY.on = false; REPLAY.playing = false; REPLAY.frames = []; REPLAY.meta = null;
  if ($('replay-box')) $('replay-box').style.display = 'none';
  if ($('replay')) $('replay').textContent = '回放';
  $('ground').checked = true; groundClamp = true;
  robotGroup.position.set(0, groundClamp ? (current?.groundOffset ?? 0) : 0, 0);
  robotGroup.quaternion.setFromEuler(new THREE.Euler(-Math.PI / 2, 0, 0));
}

async function startReplay(url, preloaded) {
  const res = preloaded ? null : await fetch(url);
  if (res && !res.ok) throw new Error(`回放文件读不到：${url}（${res.status}）`);
  const traj = preloaded || await res.json();
  if (!current) throw new Error('先加载机型');
  const mine = new Set(current.joints.keys());
  const missing = traj.meta.joints.filter((n) => !mine.has(n));
  if (missing.length) throw new Error(`机型对不上，缺关节：${missing.slice(0, 3).join(', ')}…`);
  REPLAY.frames = traj.frames;
  REPLAY.meta = traj.meta;
  REPLAY.fps = traj.meta.control_hz || 50;
  REPLAY.on = true;
  REPLAY.playing = true;
  REPLAY.acc = 0;
  groundClamp = false;
  $('ground').checked = false;
  $('replay-box').style.display = 'flex';
  $('replay').textContent = '退出回放';
  $('play').textContent = '暂停';
  const slider = $('replay-t');
  slider.max = String(Math.max(0, traj.frames.length - 1));
  slider.value = '0';
  const info = $('replay-info');
  if (info) {
    const m = traj.meta;
    info.textContent = `${m.target} amp=${m.amp} gravity=${m.gravity} · 0.00 s`;
  }
  applyReplayFrame(0);
}

/** 每帧调用：按真实时间推进回放时钟 */
function tickReplay(dt) {
  if (!REPLAY.on || !REPLAY.playing || !REPLAY.frames.length) return;
  REPLAY.acc += dt * REPLAY.fps;
  if (REPLAY.acc < 1) return;
  let step = Math.floor(REPLAY.acc);
  REPLAY.acc -= step;
  let next = REPLAY.i + step;
  if (next >= REPLAY.frames.length) next = 0;   // 循环
  applyReplayFrame(next);
}

let lastSize = { w: 0, h: 0 };
let lastFrameMs = performance.now();
function animate() {
  const nowMs = performance.now();
  const dt = Math.min(0.1, (nowMs - lastFrameMs) / 1000);
  lastFrameMs = nowMs;
  tickReplay(dt);
  // 右半边机器人每帧跟随左半边的位姿（贴地/回零会改 robotGroup 的位移）
  if (SPLIT.on && SPLIT.root) {
    SPLIT.root.position.copy(robotGroup.position);
    SPLIT.root.rotation.copy(robotGroup.rotation);
  }
  const cw = canvas.clientWidth, ch = canvas.clientHeight;
  if (cw !== lastSize.w || ch !== lastSize.h) {
    lastSize = { w: cw, h: ch };
    resize();
  }
  if (SPLIT.on && SPLIT.renderer) {
    const cv2 = $('c2'), r2 = cv2.getBoundingClientRect();
    if (r2.width > 0) {
      SPLIT.renderer.setSize(r2.width, r2.height, false);
      SPLIT.camera.aspect = r2.width / Math.max(r2.height, 1);
      SPLIT.camera.updateProjectionMatrix();
      SPLIT.controls.update();
      SPLIT.renderer.render(SPLIT.scene, SPLIT.camera);
    }
  }
  requestAnimationFrame(animate);
  controls.update();
  renderer.render(scene, camera);
}

(async function main() {
  resize();
  $('split').addEventListener('click', () => splitToggle());
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
  $('replay').addEventListener('click', async () => {
    try {
      if (REPLAY.on) { stopReplay(); return; }
      const url = new URLSearchParams(location.search).get('replay') || './data/replay-duck-pd.json';
      // 轨迹属于哪台机型就先切过去，不然会报「关节对不上」这种没信息量的错
      const traj = await (await fetch(url)).json();
      const want = traj.meta && traj.meta.robot;
      if (want && sel.value !== want) {
        const meta = index.robots.find((r) => r.id === want);
        if (!meta) throw new Error(`没有机型 ${want}`);
        sel.value = want;
        await show(meta);
      }
      await startReplay(url, traj);
    } catch (e) { alert('回放失败：' + (e.message || e)); }
  });
  $('play').addEventListener('click', () => {
    if (!REPLAY.on) return;
    REPLAY.playing = !REPLAY.playing;
    $('play').textContent = REPLAY.playing ? '暂停' : '播放';
  });
  $('replay-t').addEventListener('input', (e) => {
    if (!REPLAY.on) return;
    REPLAY.playing = false; $('play').textContent = '播放';
    applyReplayFrame(Number(e.target.value));
  });
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

function parBuildSide(geom, mirror, tint) {
  const g = new THREE.Group();
  const base = tint || (mirror ? 0xffb35a : 0x5ad1a0);
  const rodMat = new THREE.MeshStandardMaterial({
    color: base, emissive: tint ? 0x3a2210 : (mirror ? 0x3a2210 : 0x143528),
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
  const plateGeo = new THREE.BoxGeometry(0.075, 0.045, 0.006);
  const plate = new THREE.LineSegments(new THREE.EdgesGeometry(plateGeo),
    new THREE.LineBasicMaterial({ color: tint || (mirror ? 0xffb35a : 0x5ad1a0),
      transparent: true, opacity: .75 }));
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
/** 机构坐标系 → 机器人坐标系。镜像侧 roll 与 pitch 都要取反（镜像反转手性，单反一个轴会变成不对称）。 */
function parToRobot(side, v) {
  return side.mirror ? [-v[0], -v[1]] : [v[0], v[1]];
}

function applyJointPair(side, x) {
  const [roll, pitch] = parToRobot(side, x);
  setJointAngle(side.rollJoint, roll);
  setJointAngle(side.pitchJoint, pitch);
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
        <input type="range" data-k="${k}" min="${k < 2 ? -28 : -30}" max="${k < 2 ? 28 : (k === 2 ? 30 : 45)}"
               step="0.5" value="0"/></div>`).join('')}
    <div class="par-out" data-out></div>
    <div class="par-lim" data-lim style="display:none"></div>`;
  host.appendChild(wrap);
  const inputs = [...wrap.querySelectorAll('input')];
  const outs = [...wrap.querySelectorAll('[data-v]')];
  const out = wrap.querySelector('[data-out]');
  const state = { th: [0, 0], x: [0, 0] };
  const lim = wrap.querySelector('[data-lim]');
  // 行程：电机 ±28°（= MuJoCo 查表范围 ±0.5 rad），roll ±30°，pitch -30°~45°
  const LIM_TH = 28 * Math.PI / 180, LIM_X = [-30 * Math.PI / 180, 30 * Math.PI / 180,
                                              -30 * Math.PI / 180, 45 * Math.PI / 180];
  const inRange = (st) => Math.abs(st.th[0]) <= LIM_TH + 1e-9 && Math.abs(st.th[1]) <= LIM_TH + 1e-9
    && st.x[0] >= LIM_X[0] - 1e-9 && st.x[0] <= LIM_X[1] + 1e-9
    && st.x[1] >= LIM_X[2] - 1e-9 && st.x[1] <= LIM_X[3] + 1e-9;
  const showLim = (msg) => {
    if (!msg) { lim.style.display = 'none'; return; }
    lim.textContent = msg;
    lim.style.display = '';
  };

  const paint = () => {
    const thr = parToRobot(side, state.th);          // 面板一律显示机身系
    const xr = parToRobot(side, state.x);
    [0, 1, 2, 3].forEach((k) => {
      const v = k < 2 ? deg(thr[k]) : deg(xr[k - 2]);
      inputs[k].value = String(v.toFixed(1));
      outs[k].textContent = `${v.toFixed(1)}°`;
    });
    const err = Math.max(...[0, 1].map((i) =>
      Math.abs(parRes(side.geom, state.th, state.x, i)) / (2 * side.geom.chains[i].rod)));
    out.innerHTML = `<span style="color:#7f8b9b">杆长误差 ${(err * 1e9).toFixed(1)} nm`
      + (side.assumed ? ` · ${side.assumed} 项假设` : '') + `</span>`;
    parApply(side, state.th, state.x);
    applyJointPair(side, state.x);
    if (SPLIT.on && SPLIT.ready && side.gridFile) splitUpdate(side);
  };

  inputs.forEach((input) => input.addEventListener('input', () => {
    const k = +input.dataset.k;
    const prev = { th: [...state.th], x: [...state.x] };
    const v = rad(parseFloat(input.value));
    if (k < 2) {
      state.th[k] = side.mirror ? -v : v;            // 面板是机身系，换回机构系
      state.x = parFw(side.geom, state.th, state.x);
    } else {
      state.x[k - 2] = side.mirror ? -v : v;
      state.th = parIk(side.geom, state.x, state.th);
    }
    // 解出来的另一侧超出滑条行程 → 不接受这一步，把输入退回去并提示
    if (!inRange(state)) {
      state.th = prev.th; state.x = prev.x;
      paint();
      const which = [0, 1].some((i) => Math.abs(state.th[i]) > LIM_TH) ? '电机' : '关节';
      showLim(`⚠ 这一步会让${which}超出滑条行程，已退回上一步`);
      input.value = String(deg(k < 2 ? parToRobot(side, state.th)[k]
        : parToRobot(side, state.x)[k - 2]).toFixed(1));
      return;
    }
    showLim('');
    paint();
  }));

  side.state = state; side.paint = paint; side.wrap = wrap;
  paint();
}

/** 模型半透明：所有 mesh 材质降不透明度（保留原值以便还原）。 */
function setTranslucent(on) {
  PAR.translucent = on;
  robotGroup.traverse((o) => {
    if (!o.isMesh) return;
    const mats = Array.isArray(o.material) ? o.material : [o.material];
    for (const m of mats) {
      if (m.userData.origOpacity === undefined) m.userData.origOpacity = 1;
      if (m.userData.origTransparent === undefined) m.userData.origTransparent = m.transparent;
      m.transparent = on ? true : m.userData.origTransparent;
      m.opacity = on ? 0.22 : m.userData.origOpacity;
      m.depthWrite = on ? false : true;
      m.needsUpdate = true;
    }
  });
}

/** 把叠加层挂到某套 joints 的 pitch 关节父 link 上；返回 side 对象。 */
function parAttach(sideDef, joints, data, tint) {
  const side = { ...sideDef };
  Object.assign(side, parBuildSide(side.geom, side.mirror, tint));
  const jdef = data.joints.find((j) => j.name === side.pitchJoint) || {};
  const childLink = data.links.find((l) => l.name === jdef.child);
  const pj = joints.get(side.pitchJoint);
  if (!pj) return null;
  side.group.position.set(0, 0, 0);
  if (childLink) side.group.position.set(...childLink.xyz);
  pj.obj.parent.add(side.group);
  return side;
}

async function initParallelOverlay(data) {
  const sec = $('par-sec'), panel = $('par-panel');
  if (!sec || !panel) return;
  const mechs = (data.mechanisms || []).filter((m) => (m.coupling || {}).kind === 'analytic');
  if (!mechs.length) { sec.style.display = 'none'; panel.style.display = 'none'; return; }
  sec.style.display = ''; panel.style.display = '';
  panel.innerHTML = '';
  PAR.sides = {};
  PAR.mechs = [];

  for (const mech of mechs) {
    const params = mech.coupling.params || {};
    const pr = (mech.charts || {}).pr || {};
    const seq = pr.joints || [];
    const sem = pr.semantic || [];
    const pitchJoint = seq[sem.indexOf('pitch')];
    const rollJoint = seq[sem.indexOf('roll')];
    if (!pitchJoint || !rollJoint || !current.joints.get(pitchJoint)) continue;

    const sideDef = { prefix: mech.id, label: mech.id, mirror: !!params.mirror,
                      pitchJoint, rollJoint, geom: parGeom(params),
                      assumed: (mech.coupling.assumed || []).length,
                      gridFile: params.mujoco_grid || null };
    const side = parAttach(sideDef, current.joints, data);
    if (!side) continue;
    parMakeUI(side, panel);
    PAR.sides[mech.id] = side;
    PAR.mechs.push({ side, data, mech });
  }
  if (!Object.keys(PAR.sides).length) { sec.style.display = 'none'; panel.style.display = 'none'; return; }

  const tbox = document.getElementById('translucent');
  if (tbox && tbox.checked) setTranslucent(true);
  const qs = new URLSearchParams(location.search);
  for (const [key] of [['parL'], ['par'], ['parR']]) {
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
    // 行程守卫自检：把 pitch 推到 45°，应该被退回并提示
    const first = Object.values(PAR.sides)[0];
    if (first && first.wrap) {
      const pitchInput = first.wrap.querySelector('input[data-k="3"]');
      pitchInput.value = '45';
      pitchInput.dispatchEvent(new Event('input'));
      const limEl = first.wrap.querySelector('[data-lim]');
      console.log('LIMIT-TEST pitch=45° 之后：状态 pitch',
        (first.state.x[1] * 180 / Math.PI).toFixed(2), '电机',
        first.state.th.map((v) => (v * 180 / Math.PI).toFixed(1)).join('/'),
        '提示:', limEl.style.display === 'none' ? '无' : limEl.textContent);
      const okInput = first.wrap.querySelector('input[data-k="0"]');
      okInput.value = '20';
      okInput.dispatchEvent(new Event('input'));
      console.log('LIMIT-TEST 电机=20° 之后：roll/pitch',
        first.state.x.map((v) => (v * 180 / Math.PI).toFixed(2)).join('/'),
        '提示:', limEl.style.display === 'none' ? '无' : limEl.textContent);
    }
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
  if (qs.has('split')) setTimeout(() => { if (!SPLIT.on) splitToggle(); }, 600);
  if (qs.get('selftest') === 'switch') {
    setTimeout(() => {
      console.log('SWITCH before: left', current.data.id, 'split joints',
        SPLIT.joints ? SPLIT.joints.size : '-');
      const sel = document.getElementById('robot');
      sel.value = 'g1-29dof';
      sel.dispatchEvent(new Event('change'));
      setTimeout(() => {
        console.log('SWITCH after: left', current.data.id, 'split joints',
          SPLIT.joints ? SPLIT.joints.size : '-', 'sides', SPLIT.sides.length);
      }, 4000);
    }, 2500);
  }
  if (qs.has('selftest')) setTimeout(() => {
    const f = (v) => `(${v.toArray().map((x) => x.toFixed(3)).join(',')})`;
    console.log('CAM left pos', f(camera.position), 'target', f(controls.target), 'fov', camera.fov);
    if (SPLIT.camera) console.log('CAM right pos', f(SPLIT.camera.position), 'target', f(SPLIT.controls.target), 'fov', SPLIT.camera.fov);
    if (SPLIT.root) console.log('CAM rootY', SPLIT.root.position.y, 'robotY', robotGroup.position.y);
    const r1 = canvas.getBoundingClientRect();
    const r2 = document.getElementById('c2').getBoundingClientRect();
    console.log('SIZES left', r1.width.toFixed(0), r1.height.toFixed(0), 'aspect', camera.aspect.toFixed(3),
      '| right', r2.width.toFixed(0), r2.height.toFixed(0), 'aspect', SPLIT.camera ? SPLIT.camera.aspect.toFixed(3) : '-');
  }, 2000);
}


// ---------------------------------------------------------------------------
// 并排对比：左边我们的解算（模型 + 绿色连杆），右边 MuJoCo（同一台机器人 + 橙色连杆）。
// MuJoCo 侧用 41×41 的 (θA,θB)→(roll,pitch) 查表双线性插值。
// ---------------------------------------------------------------------------
function splitInit() {
  if (SPLIT.renderer) return;
  const cv = $('c2');
  SPLIT.renderer = new THREE.WebGLRenderer({ canvas: cv, antialias: true });
  SPLIT.renderer.setPixelRatio(Math.min(devicePixelRatio, 2));
  SPLIT.scene = new THREE.Scene();
  SPLIT.scene.background = new THREE.Color(0x0b0e12);
  SPLIT.scene.add(new THREE.HemisphereLight(0xdfe9f5, 0x1a2230, 1.2));
  const dl = new THREE.DirectionalLight(0xffffff, 1.0);
  dl.position.set(0.6, -0.8, 1.1);
  SPLIT.scene.add(dl);
  SPLIT.camera = new THREE.PerspectiveCamera(38, 1, 0.004, 50);
  SPLIT.controls = new OrbitControls(SPLIT.camera, cv);
  SPLIT.controls.enableDamping = true;
  SPLIT.data = new Map();
  SPLIT.sides = [];
  SPLIT.ready = false;
}

async function splitGrid(side) {
  if (!side.gridFile) return null;
  if (!SPLIT.data.has(side.gridFile)) {
    try {
      SPLIT.data.set(side.gridFile, await (await fetch(`./data/${side.gridFile}`)).json());
    } catch (e) { console.warn('MuJoCo 查表缺失', side.gridFile, e); return null; }
  }
  return SPLIT.data.get(side.gridFile);
}

/** 双线性插值：电机角 → MuJoCo 的 roll/pitch。 */
function splitLookup(g, th) {
  const n = g.n, span = g.span;
  const u = (th[0] + span) / (2 * span) * (n - 1);
  const v = (th[1] + span) / (2 * span) * (n - 1);
  const i = Math.max(0, Math.min(n - 2, Math.floor(u)));
  const j = Math.max(0, Math.min(n - 2, Math.floor(v)));
  const fu = Math.max(0, Math.min(1, u - i)), fv = Math.max(0, Math.min(1, v - j));
  const at = (a, b) => g.grid[a][b];
  const lerp = (a, b, t) => a + (b - a) * t;
  return [0, 1].map((k) => lerp(
    lerp(at(i, j)[k], at(i + 1, j)[k], fu),
    lerp(at(i, j + 1)[k], at(i + 1, j + 1)[k], fu), fv));
}

async function splitToggle() {
  SPLIT.on = !SPLIT.on;
  $('split').classList.toggle('on', SPLIT.on);
  document.body.classList.toggle('split-on', SPLIT.on);
  $('view-r').style.display = SPLIT.on ? '' : 'none';
  requestAnimationFrame(() => { lastSize = { w: 0, h: 0 }; resize(); });
  $('cap-l').textContent = SPLIT.on ? '我们的解算' : '我们的解算';
  if (!SPLIT.on) return;
  splitInit();
  await splitEnsure();
}

/** 按当前机型建好右半边（换机型后要重来）。 */
async function splitEnsure() {
  if (!SPLIT.on) return;
  splitInit();
  if (!SPLIT.ready && !SPLIT.building && current) {
    SPLIT.building = true;
    SPLIT.root = new THREE.Group();
    SPLIT.scene.add(SPLIT.root);
    const { joints } = await buildInto(current.data, SPLIT.root);
    SPLIT.joints = joints;
    SPLIT.root.position.y = current.groundOffset || 0;
    SPLIT.root.traverse((o) => {          // 右半边机器人做半透明，能看见连杆
      if (!o.isMesh) return;
      o.material = o.material.clone();
      o.material.transparent = true;
      o.material.opacity = 0.22;
      o.material.depthWrite = false;
    });
    for (const m of PAR.mechs || []) {
      const side = parAttach({ ...m.side, geom: m.side.geom }, joints, m.data, 0xffb35a);
      if (side) SPLIT.sides.push(side);
    }
    SPLIT.ready = true;
    SPLIT.building = false;
  }
  // 两边镜头同朝向同距离
  const focusOn = (joints, cam, ctl) => {
    const pj = joints && joints.get((PAR.mechs[0] || {}).side.pitchJoint);
    const p = new THREE.Vector3();
    if (pj) pj.obj.getWorldPosition(p);
    else p.set(0, 0, 0);
    const dir = new THREE.Vector3(0.6, -0.7, 0.45).normalize();
    ctl.target.copy(p);
    cam.position.copy(p).addScaledVector(dir, 0.62);
    cam.near = 0.004; cam.far = 50; cam.updateProjectionMatrix();
    ctl.update();
  };
  if (SPLIT.root) {
    SPLIT.root.position.copy(robotGroup.position);
    SPLIT.root.rotation.copy(robotGroup.rotation);
    SPLIT.root.updateMatrixWorld(true);
  }
  focusOn(current.joints, camera, controls);
  focusOn(SPLIT.joints, SPLIT.camera, SPLIT.controls);
  if (!SPLIT.camHooked) { hookCamSync(); SPLIT.camHooked = true; }
  camSync(camera, controls, SPLIT.camera, SPLIT.controls);
  for (const side of Object.values(PAR.sides)) splitUpdate(side);
}

/** 换机型：把右半边拆干净（否则会变成「G1 对比天工」）。 */
function splitReset() {
  if (SPLIT.root && SPLIT.scene) {
    SPLIT.scene.remove(SPLIT.root);
    SPLIT.root.traverse((o) => {
      if (o.isMesh) {
        o.geometry?.dispose?.();
        if (Array.isArray(o.material)) o.material.forEach((m) => m.dispose?.());
        else o.material?.dispose?.();
      }
    });
  }
  SPLIT.root = null;
  SPLIT.joints = null;
  SPLIT.sides = [];
  SPLIT.ready = false;
  SPLIT.building = false;
  SPLIT.data = new Map();
}

/** 更新右半边：机器人与连杆都按 MuJoCo 的 roll/pitch。 */
async function splitUpdate(side) {
  if (!SPLIT.on || !SPLIT.ready || !side.gridFile) return;
  if (SPLIT.root) {
    SPLIT.root.position.copy(robotGroup.position);   // 跟着「贴地」一起动
    SPLIT.root.rotation.copy(robotGroup.rotation);   // Z-up → Y-up 的显示旋转也要一致
  }
  const g = await splitGrid(side);
  if (!g) return;
  const twin = SPLIT.sides.find((s) => s.prefix === side.prefix);
  const mj = splitLookup(g, side.state.th);
  if (twin) {
    const [roll, pitch] = parToRobot(twin, mj);
    setJointIn(SPLIT.joints, twin.rollJoint, roll);
    setJointIn(SPLIT.joints, twin.pitchJoint, pitch);
    parApply(twin, side.state.th, mj);
  }
  const firstId = (PAR.mechs && PAR.mechs[0]) ? PAR.mechs[0].side.prefix : null;
  if (!firstId || side.prefix === firstId) parTraceCaption(side, mj, g);
}

function parTraceCaption(side, mj, g) {
  const ours = parToRobot(side, side.state.x), eq = parToRobot(side, mj);
  const dr = ours[0] - eq[0], dp = ours[1] - eq[1];
  $('cap-r').textContent = `MuJoCo · roll ${(mj[0] * 180 / Math.PI).toFixed(2)}° `
    + `pitch ${(mj[1] * 180 / Math.PI).toFixed(2)}° · 差 `
    + `${(Math.max(Math.abs(dr), Math.abs(dp)) * 1e3).toFixed(2)} mrad`
    + (g ? ` · ${g.n}×${g.n} 查表` : '');
}

// ---------------------------------------------------------------------------
// 两边相机同步：把「相对各自 target 的球坐标」复制过去，取景完全一致。
// ---------------------------------------------------------------------------
let camGuard = false;
function camSync(srcCam, srcCtl, dstCam, dstCtl) {
  if (camGuard) return;
  camGuard = true;
  // 直接把「相对 target 的位移」搬过去：比换算球坐标更不容易出偏差
  // 两边机器人放在同一处（落地偏移同步过），所以 target 也可以直接照抄
  dstCtl.target.copy(srcCtl.target);
  dstCam.position.copy(srcCam.position);
  dstCam.up.copy(srcCam.up);
  dstCam.fov = srcCam.fov;
  dstCam.near = srcCam.near;
  dstCam.far = srcCam.far;
  dstCam.updateProjectionMatrix();
  dstCtl.update();
  camGuard = false;
}
function hookCamSync() {
  const cb = $('camsync');
  const enabled = () => cb ? cb.checked : true;
  controls.addEventListener('change', () => {
    if (!enabled() || !SPLIT.on || !SPLIT.ready || camGuard) return;
    camSync(camera, controls, SPLIT.camera, SPLIT.controls);
  });
  if (SPLIT.controls) {
    SPLIT.controls.addEventListener('change', () => {
      if (!enabled() || !SPLIT.on || !SPLIT.ready || camGuard) return;
      camSync(SPLIT.camera, SPLIT.controls, camera, controls);
    });
  }
}

(function hookTranslucent() {
  const box = document.getElementById('translucent');
  if (box) box.addEventListener('change', () => setTranslucent(box.checked));
})();
