// 浏览器里「物理 + 推理」共用的核心：MuJoCo wasm 建模型、观测装配、单步物理。
//
// 关键约定（2026-11-10 起）：**观测布局和图的槽位不写在这里，写在契约里**。
// 契约由 `sim/host/export_artifacts.py` 生成到 `data/live/contract.json`，
// 这里只做「按配方取数」——改观测顺序、加策略、改 fsm 输出维度，都不用动本文件。

import { quatToMat as q2m } from './motion.mjs';

let CONTRACT = null;

/** 契约自检：与 Python 侧 `contract.validate_contract()` 同一套规则（端侧也要会挑错）。 */
export function validateContract(doc) {
  const errs = [];
  if (!doc || doc.kind !== 'ducksir-contract') errs.push('不是 ducksir-contract');
  const plans = (doc && doc.plans) || {};
  for (const [name, p] of Object.entries(plans)) {
    for (const [slotName, range] of Object.entries(p.slots || {})) {
      const [off, len] = range;
      const limit = slotName.startsWith('out:') ? p.output_dim : p.input_dim;
      if (off < 0 || len <= 0 || off + len > limit) {
        errs.push(`plan ${name} 的槽位 ${slotName} 越界：${off}+${len} > ${limit}`);
      }
    }
  }
  const obs = (doc && doc.obs) || [];
  if (obs.length) {
    const end = obs.reduce((m, o) => Math.max(m, o.slot[0] + o.slot[1]), 0);
    const pre = plans.pre;
    if (pre && end !== pre.input_dim) errs.push(`obs 总长 ${end} != pre.input_dim ${pre.input_dim}`);
  }
  for (const pol of (doc && doc.policies) || []) {
    for (const key of ['key', 'mode', 'file']) {
      if (!(key in pol)) errs.push(`策略条目缺字段 ${key}`);
    }
  }
  return errs;
}

/** 载入契约（一次性）。所有 `makeObs` / 槽位查询都依赖它。不合法的契约直接拒绝。 */
export function setContract(doc) {
  const problems = validateContract(doc);
  if (problems.length) throw new Error('契约不合法：' + problems.join('；'));
  CONTRACT = doc;
}
export function getContract() {
  if (!CONTRACT) throw new Error('还没 setContract()');
  return CONTRACT;
}

/** 取某个 plan 的具名槽位 `[偏移, 长度]`（名字形如 `out:state`）。 */
export function slot(planName, slotName) {
  const p = getContract().plans[planName];
  if (!p) throw new Error(`契约里没有 plan ${planName}`);
  const s = p.slots && p.slots[slotName];
  if (!s) throw new Error(`plan ${planName} 没有槽位 ${slotName}`);
  return s;
}

/** 按槽位从数组里取一段。 */
export function readSlot(arr, planName, slotName) {
  const [off, len] = slot(planName, slotName);
  return Array.from(arr).slice(off, off + len);
}

/** 观测总长（= 契约里 obs 配方覆盖到的最大末端）。 */
export function obsDim() {
  return getContract().obs.reduce((m, o) => Math.max(m, o.slot[0] + o.slot[1]), 0);
}

// 绑定的字段都是普通 TypedArray（Int32Array / Float64Array），直接下标即可。
export const read = (arr, i) => (arr && typeof arr.get === 'function' ? arr.get(i) : arr[i]);

/** 把一张平面拼进 MJCF 的 worldbody（我们导出的机器人没有场景）。 */
export function withFloor(xml, rgba = '0.55 0.55 0.58 1') {
  return xml.replace(
    '<worldbody>',
    `<worldbody>\n    <geom name="floor" type="plane" size="4 4 0.1" rgba="${rgba}" condim="3"/>`,
  );
}

/**
 * 建模型 + 数据。`assets` 是 `{文件名: Uint8Array}`，全部塞进 MjVFS（MJCF 里 meshdir="."）。
 */
export async function loadDuck({ loadMujoco, xml, assets }) {
  const mj = await loadMujoco();
  const vfs = new mj.MjVFS();
  // 只加裸文件名：MuJoCo 的 VFS 拒绝 "./name" 这种带前缀的键（error 2）。
  for (const [name, bytes] of Object.entries(assets)) vfs.addBuffer(name, bytes);
  const model = mj.MjModel.from_xml_string(withFloor(xml), vfs);
  model.opt.timestep = 1 / 200;
  const data = new mj.MjData(model);

  const objId = (key) => (mj.mjtObj[key] && mj.mjtObj[key].value !== undefined ? mj.mjtObj[key].value : mj.mjtObj[key]);
  const jointId = (name) => mj.mj_name2id(model, objId('mjOBJ_JOINT'), name);
  const sensorId = (name) => mj.mj_name2id(model, objId('mjOBJ_SENSOR'), name);
  const bodyId = (name) => mj.mj_name2id(model, objId('mjOBJ_BODY'), name);

  return { mj, model, data, jointId, sensorId, bodyId, objId };
}

/** 检查 plan 头里的维度与契约一致；不一致直接抛错（把「静默漂移」变成「加载即失败」）。 */
export function checkPlanAgainstContract(planName, info) {
  const p = getContract().plans[planName];
  if (!p) throw new Error(`契约里没有 plan ${planName}`);
  if (info.input_dim !== p.input_dim || info.output_dim !== p.output_dim) {
    throw new Error(
      `plan ${planName} 维度不符：blob ${info.input_dim}→${info.output_dim}，` +
      `契约 ${p.input_dim}→${p.output_dim}。多半是改了 Python 但没重新导出契约。`);
  }
}

/**
 * 按契约里的观测配方装配观测。
 *
 * 配方条目形如 `{name, slot:[off,len], source:"sensor:imu_ang_vel"}`，支持的 source：
 * `sensor:<名>`、`gravity:body:<名>`、`qpos:joints`、`qvel:joints`、`last_action`、`fsm_slot:<槽位名>`，
 * 以及对齐 botlab `Robot.body_*` / `base_height` 的 `body:pos|quat|vel|angvel:<body>`、`base:z`。
 * `contact:` 暂只在宿主端支持（浏览器侧要算 mjContact 的偏移，没核对前不猜）。
 */
export function makeObs(duck, { lastAction, vx, command, motionRef: motionRefValue } = {}) {
  const { model, data, jointId, sensorId, bodyId } = duck;
  const c = getContract();
  const robot = c.robot;
  const obs = new Float32Array(obsDim());

  for (const item of c.obs) {
    const [off, len] = item.slot;
    const src = item.source || '';
    if (src.startsWith('sensor:')) {
      const sid = sensorId(src.slice(7));
      const adr = read(model.sensor_adr, sid);
      for (let k = 0; k < len; k++) obs[off + k] = read(data.sensordata, adr + k);
    } else if (src.startsWith('gravity:body:')) {
      // 重力在世界系是 (0,0,-1)，它在机体系的表达 = R^T·g，第 r 个分量 = -R[2][r]
      const bid = bodyId(src.slice(13));
      const w = read(data.xquat, bid * 4), x = read(data.xquat, bid * 4 + 1);
      const y = read(data.xquat, bid * 4 + 2), z = read(data.xquat, bid * 4 + 3);
      const R = [
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
      ];
      for (let r = 0; r < 3; r++) obs[off + r] = -R[2][r];
    } else if (src === 'qpos:joints') {
      // order=policy：策略训练时用的是策略序（G1 的 dof_mapping 不是恒等），按 motor_order 取
      for (let k = 0; k < len; k++) {
        const idx = item.order === 'policy' ? robot.motor_order[k] : k;
        const qa = read(model.jnt_qposadr, jointId(robot.joints[idx]));
        const base = item.minus === 'default' ? (robot.default_q[idx] || 0)
          : item.minus === 'reference' ? ((robot.reference_joint_positions || [])[idx] || 0) : 0;
        obs[off + k] = read(data.qpos, qa) - base;
      }
    } else if (src === 'qvel:joints') {
      for (let k = 0; k < len; k++) {
        const idx = item.order === 'policy' ? robot.motor_order[k] : k;
        const va = read(model.jnt_dofadr, jointId(robot.joints[idx]));
        obs[off + k] = read(data.qvel, va);
      }
    } else if (src.startsWith('body:')) {
      // 与宿主 contract_obs 同语义：位置/四元数世界系，速度取 cvel（前 3 角、后 3 线）
      const parts = src.split(':');
      const kind = parts[1];
      const bid = bodyId(parts[2]);
      if (kind === 'pos') {
        for (let k = 0; k < 3; k++) obs[off + k] = read(data.xpos, bid * 3 + k);
      } else if (kind === 'quat') {
        for (let k = 0; k < 4; k++) obs[off + k] = read(data.xquat, bid * 4 + k);
      } else if (kind === 'vel') {
        for (let k = 0; k < 3; k++) obs[off + k] = read(data.cvel, bid * 6 + 3 + k);
      } else if (kind === 'angvel') {
        // 与宿主同语义：默认转机体系（botlab 的 body_angular_velocity(frame="local")）
        const w = [0, 1, 2].map((k) => read(data.cvel, bid * 6 + k));
        const mode = item.frame || 'local';
        if (mode === 'local') {
          const q = [0, 1, 2, 3].map((k) => read(data.xquat, bid * 4 + k));
          const R = q2m(q);
          for (let r = 0; r < 3; r++) {
            obs[off + r] = R[0][r] * w[0] + R[1][r] * w[1] + R[2][r] * w[2];
          }
        } else {
          for (let k = 0; k < 3; k++) obs[off + k] = w[k];
        }
      } else {
        throw new Error(`不认识的 body 观测：${src}`);
      }
    } else if (src.startsWith('contact:')) {
      // 浏览器端还没核对 mjContact 的字段偏移，宁可不做也不给错数：显式报错，配方改走宿主。
      throw new Error(`浏览器端暂不支持 ${src}（contact 仅在宿主端点可用）`);
    } else if (src === 'base:z') {
      obs[off] = read(data.qpos, 2);
    } else if (src === 'command:velocity') {
      const c = command || [];
      for (let k = 0; k < 3; k++) obs[off + k] = c[k] || 0;
    } else if (src === 'motion_ref') {
      if (!motionRefValue) throw new Error('观测配方需要 motion_ref，但调用方没给');
      for (let k = 0; k < len; k++) obs[off + k] = motionRefValue[k] || 0;
    } else if (src === 'last_action') {
      for (let k = 0; k < len; k++) obs[off + k] = lastAction ? lastAction[k] || 0 : 0;
    } else if (src.startsWith('fsm_slot:')) {
      const name = src.slice(9);
      const vals = name === 'command' ? command : null;
      if (!vals) throw new Error(`观测配方引用 ${src}，但调用方没给`);
      for (let k = 0; k < len; k++) obs[off + k] = vals[k] || 0;
    } else {
      throw new Error(`观测配方里有不认识的 source：${src}`);
    }
  }
  return obs;
}

/** 一步物理：4 个 200 Hz 子步 = 1 个 50 Hz 控制步。 */
export function stepPhysics(mj, model, data, decimation = 4) {
  for (let k = 0; k < decimation; k++) mj.mj_step(model, data);
}

/** 由 trunk 的姿态算「直立度」（重力 z 分量，-1 = 完全正立）。 */
export function upright(duck, bodyName = 'trunk_base') {
  const bid = duck.bodyId(bodyName);
  const qx = read(duck.data.xquat, bid * 4 + 1), qy = read(duck.data.xquat, bid * 4 + 2);
  return -(1 - 2 * (qx * qx + qy * qy));
}
