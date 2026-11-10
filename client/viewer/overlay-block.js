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
  setJointAngle(`${side.prefix}_ankle_roll_joint`, side.mirror ? -x[0] : x[0]);
  setJointAngle(`${side.prefix}_ankle_pitch_joint`, x[1]);
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
  const mechs = data.mechanisms || [];
  const analytical = mechs.filter((m) => (m.coupling || {}).kind === 'analytic');
  if (!analytical.length) { sec.style.display = 'none'; panel.style.display = 'none'; return; }
  sec.style.display = ''; panel.style.display = '';
  panel.innerHTML = '';
  PAR.sides = {};

  for (const [prefix, label, mirror] of [['left', '左踝', false], ['right', '右踝', true]]) {
    const mech = mechs.find((m) => m.id === `ankle_${prefix}`);
    const params = mech && (mech.coupling || {}).kind === 'analytic' && mech.coupling.params;
    if (!params) continue;
    const pj = current.joints.get(`${prefix}_ankle_pitch_joint`);
    const link = data.links.find((l) => l.name === `${prefix}_ankle_pitch_link`);
    if (!pj) continue;
    const side = { prefix, label, mirror, geom: parGeom(params),
                   assumed: (mech.coupling.assumed || []).length };
    Object.assign(side, parBuildSide(side.geom, mirror));
    side.group.position.set(0, 0, 0);
    if (link) side.group.position.set(...link.xyz);
    pj.obj.parent.add(side.group);           // 挂在膝 link，不跟着 pitch 转
    parMakeUI(side, panel);
    PAR.sides[prefix] = side;
  }

  const tbox = document.getElementById('translucent');
  if (tbox && tbox.checked) setTranslucent(true);
  const qs = new URLSearchParams(location.search);
  for (const [key, prefix] of [['parL', 'left'], ['par', 'left'], ['parR', 'right']]) {
    const v = qs.get(key);
    if (!v || !PAR.sides[prefix]) continue;
    const [a, b] = v.split(',').map((x) => parseFloat(x) || 0);
    const st = PAR.sides[prefix].state;
    st.th = [a * Math.PI / 180, b * Math.PI / 180];
    st.x = parFw(PAR.sides[prefix].geom, st.th, st.x);
    PAR.sides[prefix].paint();
  }
  if (qs.has('selftest')) {
    for (const [prefix, side] of Object.entries(PAR.sides)) {
      for (const x of [[0.2, 0.1], [-0.15, 0.25], [0.3, -0.2]]) {
        const th = parIk(side.geom, x, [0, 0]);
        const back = parFw(side.geom, th, x);
        console.log(`SELFTEST ${prefix} x=`, x.map((v) => +v.toFixed(4)),
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
