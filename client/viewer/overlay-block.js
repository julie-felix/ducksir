// ---------------------------------------------------------------------------
// 并联踝叠加层：两根定长推杆画在模型上；A/B 与 roll/pitch 双向拖拽（左右都支持）。
// 几何数据来自 tools/parallel/export_overlay.py（坐标系 = ankle pitch 关节中心）。
// ---------------------------------------------------------------------------
const PAR = { data: null, sides: {}, translucent: false };

const parV = (a) => new THREE.Vector3(a[0], a[1], a[2]);
const parN = (a) => new THREE.Vector3(a[0], a[1], a[2]).normalize();
const parRot = (axis, ang) => new THREE.Quaternion().setFromAxisAngle(parN(axis), ang);

/** 平台点 C(x)：先绕 roll 轴转，再绕 pitch 轴转（与 Python solver 的 FK 一致）。 */
function parPlatformPoint(x, c0) {
  const P = PAR.data.platform;
  const Pp = parV(P.pitch_pivot), Pr = parV(P.roll_pivot);
  return parV(c0).sub(Pr).applyQuaternion(parRot(P.roll_axis, x[0])).add(Pr).sub(Pp)
    .applyQuaternion(parRot(P.pitch_axis, x[1])).add(Pp);
}
/** 推杆销 B(θ)。 */
function parPinPoint(i, theta) {
  const ch = PAR.data.chains[i];
  return parV(ch.v).applyQuaternion(parRot(ch.axis, theta)).add(parV(ch.A));
}
/** 约束残差（平方形式，和 jac 的推导一致）。 */
function parRes(th, x, i) {
  const d = parPinPoint(i, th[i]).sub(parPlatformPoint(x, PAR.data.platform.c0[i]));
  return d.dot(d) - PAR.data.chains[i].rod ** 2;
}
function parJacobian(th, x) {
  const eps = 1e-6, J = [[0, 0], [0, 0]];
  for (let i = 0; i < 2; i++) {
    for (let k = 0; k < 2; k++) {
      const xp = [...x], xm = [...x];
      xp[k] += eps; xm[k] -= eps;
      J[i][k] = (parRes(th, xp, i) - parRes(th, xm, i)) / (2 * eps);
    }
  }
  return J;
}
const parDet = (J) => J[0][0] * J[1][1] - J[0][1] * J[1][0];

/** 正解 θ→x：牛顿迭代。 */
function parFw(th, x0 = [0, 0], maxIter = 40) {
  let x = [...x0];
  for (let it = 0; it < maxIter; it++) {
    const r = [parRes(th, x, 0), parRes(th, x, 1)];
    if (Math.hypot(r[0], r[1]) < 1e-14) break;
    const J = parJacobian(th, x), det = parDet(J);
    if (Math.abs(det) < 1e-18) break;
    x = [x[0] - (J[1][1] * r[0] - J[0][1] * r[1]) / det,
         x[1] - (-J[1][0] * r[0] + J[0][0] * r[1]) / det];
  }
  return x;
}
/** 逆解 x→θ：αcosθ + βsinθ = k 的闭式解（每个电机独立）。 */
function parIk(x, hint = [0, 0]) {
  const C = [0, 1].map((i) => parPlatformPoint(x, PAR.data.platform.c0[i]));
  return PAR.data.chains.map((ch, i) => {
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
    const a = phi + d, b = phi - d;
    const near = (t) => Math.abs(((t - hint[i] + Math.PI) % (2 * Math.PI) + 2 * Math.PI) % (2 * Math.PI) - Math.PI);
    return near(a) <= near(b) ? a : b;
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

/** 建一侧的叠加层（mirror=true 时按矢状面镜像几何）。 */
function parBuildSide(side) {
  const g = new THREE.Group();
  const rodMat = new THREE.MeshStandardMaterial({
    color: side.mirror ? 0xffb35a : 0x5ad1a0, emissive: side.mirror ? 0x3a2210 : 0x143528,
    metalness: .2, roughness: .5, side: THREE.DoubleSide });
  const nodes = PAR.data.chains.map((ch) => {
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
  const plate = new THREE.Mesh(new THREE.BoxGeometry(0.1, 0.06, 0.008),
    new THREE.MeshStandardMaterial({ color: side.mirror ? 0xb5772f : 0x39505f,
      transparent: true, opacity: .8, side: THREE.DoubleSide }));
  g.add(plate);
  if (side.mirror) g.scale.y = -1;
  return { group: g, nodes, plate };
}

function parApply(side, th, x) {
  const C = [0, 1].map((i) => parPlatformPoint(x, PAR.data.platform.c0[i]));
  side.nodes.forEach((n, i) => {
    const B = parPinPoint(i, th[i]);
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
  const P = PAR.data.platform;
  const mid = C[0].clone().add(C[1]).multiplyScalar(0.5);
  const ex = C[1].clone().sub(C[0]).normalize();
  const ey = parV(P.pitch_axis).applyQuaternion(parRot(P.pitch_axis, x[1]));
  ey.addScaledVector(ex, -ey.dot(ex)).normalize();
  const ez = new THREE.Vector3().crossVectors(ex, ey).normalize();
  side.plate.matrixAutoUpdate = false;
  side.plate.matrix.copy(new THREE.Matrix4().makeBasis(ex, ey, ez).setPosition(mid));
}

/** 一侧的完整交互：四个滑条（A/B/roll/pitch）互相联动。 */
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
    inputs[0].value = String(deg(state.th[0]).toFixed(1));
    inputs[1].value = String(deg(state.th[1]).toFixed(1));
    inputs[2].value = String(deg(state.x[0]).toFixed(1));
    inputs[3].value = String(deg(state.x[1]).toFixed(1));
    outs[0].textContent = `${deg(state.th[0]).toFixed(1)}°`;
    outs[1].textContent = `${deg(state.th[1]).toFixed(1)}°`;
    outs[2].textContent = `${deg(state.x[0]).toFixed(1)}°`;
    outs[3].textContent = `${deg(state.x[1]).toFixed(1)}°`;
    const err = Math.max(Math.abs(parRes(state.th, state.x, 0)), Math.abs(parRes(state.th, state.x, 1)));
    out.innerHTML = `<span style="color:#7f8b9b">杆长误差 ${(Math.sqrt(Math.abs(err)) * 1e9).toFixed(1)} nm</span>`;
    parApply(side, state.th, state.x);
    applyJointPair(side, state.x);
  };

  inputs.forEach((input) => input.addEventListener('input', () => {
    const k = +input.dataset.k;
    if (k < 2) {                                   // 拖电机：正解
      state.th[k] = rad(parseFloat(input.value));
      state.x = parFw(state.th, state.x);
    } else {                                       // 拖关节：逆解
      state.x[k - 2] = rad(parseFloat(input.value));
      state.th = parIk(state.x, state.th);
    }
    paint();
  }));

  side.state = state; side.paint = paint;
  paint();
}

/** 把解算出的 roll/pitch 写到模型关节上（左侧原样，右侧镜像）。 */
function applyJointPair(side, x) {
  const roll = side.mirror ? -x[0] : x[0];
  setJointAngle(`${side.prefix}_ankle_roll_joint`, roll);
  setJointAngle(`${side.prefix}_ankle_pitch_joint`, x[1]);
}

/** 模型半透明：所有 mesh 材质克隆一份并降不透明度。 */
function setTranslucent(on) {
  PAR.translucent = on;
  robotGroup.traverse((o) => {
    if (!o.isMesh) return;
    const mats = Array.isArray(o.material) ? o.material : [o.material];
    for (const m of mats) {
      if (!m.userData.origOpacity && m.userData.origOpacity !== 0) m.userData.origOpacity = m.opacity;
      m.transparent = on || m.userData.origTransparent;
      m.opacity = on ? 0.22 : m.userData.origOpacity;
      m.depthWrite = !on;
      m.needsUpdate = true;
    }
  });
}

async function initParallelOverlay(data) {
  const sec = $('par-sec'), panel = $('par-panel');
  if (!sec || !panel) return;
  const supported = data.id === 'g1-29dof';
  if (!supported) { sec.style.display = 'none'; panel.style.display = 'none'; return; }
  if (!PAR.data) {
    try {
      PAR.data = await (await fetch('./data/parallel-g1-overlay.json')).json();
    } catch (e) { console.warn('并联叠加层数据缺失', e); return; }
  }
  sec.style.display = ''; panel.style.display = '';
  panel.innerHTML = '';
  PAR.sides = {};

  for (const [prefix, label, mirror] of [['left', '左踝', false], ['right', '右踝', true]]) {
    const side = { prefix, label, mirror };
    const built = parBuildSide(side);
    const pj = current.joints.get(`${prefix}_ankle_pitch_joint`);
    if (!pj) continue;
    Object.assign(side, built);
    // 挂在膝 link 上（不跟着 pitch 转），偏移 = ankle pitch 关节中心在膝系里的位置
    const link = data.links.find((l) => l.name === `${prefix}_ankle_pitch_link`);
    side.group.position.set(0, 0, 0);
    if (link) side.group.position.set(...link.xyz);
    pj.obj.parent.add(side.group);
    parMakeUI(side, panel);
    PAR.sides[prefix] = side;
  }
}

// 头部「半透明」开关
(function hookTranslucent() {
  const box = document.getElementById('translucent');
  if (!box) return;
  box.addEventListener('change', () => setTranslucent(box.checked));
})();
