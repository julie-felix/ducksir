// 浏览器里「物理 + 推理」共用的核心：MuJoCo wasm 建模型、观测组装、单步控制。
// 与 DOM 无关，node 也能跑（sim/test/test_play_core.mjs 用它做无头验证）。

export const OBS_DIM = 61;
export const N_JOINTS = 14;

// 绑定的字段都是普通 TypedArray（Int32Array / Float64Array），直接下标即可。
// 留一层 read() 是为了兼容将来可能的包装类型。
export const read = (arr, i) => (arr && typeof arr.get === 'function' ? arr.get(i) : arr[i]);
export const write = (arr, i, v) => { if (arr && typeof arr.set === 'function' && arr.length === undefined) arr.set(i, v); else arr[i] = v; };

/** 把一张平面拼进 MJCF 的 worldbody（我们导出的机器人没有场景）。 */
export function withFloor(xml, rgba = '0.55 0.55 0.58 1') {
  return xml.replace(
    '<worldbody>',
    `<worldbody>\n    <geom name="floor" type="plane" size="4 4 0.1" rgba="${rgba}" condim="3"/>`,
  );
}

/**
 * 建模型 + 数据。`assets` 是 `{文件名: Uint8Array}`，全部塞进 MjVFS（MJCF 里 meshdir="."）。
 * @param {object} opts
 * @param {Function} opts.loadMujoco `import('@mujoco/mujoco')` 的默认导出
 * @param {string} opts.xml  MJCF 文本
 * @param {Record<string, Uint8Array>} opts.assets
 */
export async function loadDuck({ loadMujoco, xml, assets }) {
  const mj = await loadMujoco();
  const vfs = new mj.MjVFS();
  // 只加裸文件名：MuJoCo 的 VFS 拒绝 "./name" 这种带前缀的键（error 2），
  // 而 MJCF 里 meshdir="." 时会按裸文件名去找。
  for (const [name, bytes] of Object.entries(assets)) vfs.addBuffer(name, bytes);
  const model = mj.MjModel.from_xml_string(withFloor(xml), vfs);
  model.opt.timestep = 1 / 200;
  const data = new mj.MjData(model);

  // mjtObj 的枚举值是包装对象（不是裸 number），要取 .value 才能传给 mj_name2id
  const objId = (key) => (mj.mjtObj[key] && mj.mjtObj[key].value !== undefined ? mj.mjtObj[key].value : mj.mjtObj[key]);
  const jointId = (name) => mj.mj_name2id(model, objId('mjOBJ_JOINT'), name);
  const sensorId = (name) => mj.mj_name2id(model, objId('mjOBJ_SENSOR'), name);
  const bodyId = (name) => mj.mj_name2id(model, objId('mjOBJ_BODY'), name);

  return { mj, model, data, jointId, sensorId, bodyId, objId };
}

/** 组装 61 维观测（顺序与 contract/policy/duck-walk.manifest.yaml 一致）。 */
export function makeObs({ mj, model, data, jointId, sensorId, bodyId }, { joints, defaultQ, lastAction, vx }) {
  const obs = new Float32Array(OBS_DIM);
  const sid = sensorId('imu_ang_vel');
  const adr = read(model.sensor_adr, sid);
  for (let k = 0; k < 3; k++) obs[k] = read(data.sensordata, adr + k);

  // 重力在 trunk_base 系下：R^T · (0,0,-1)
  const bid = bodyId('trunk_base');
  const q = [read(data.xquat, bid * 4), read(data.xquat, bid * 4 + 1),
             read(data.xquat, bid * 4 + 2), read(data.xquat, bid * 4 + 3)]; // wxyz
  const [w, x, y, z] = q;
  const R = [
    [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
    [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
    [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
  ];
  for (let r = 0; r < 3; r++) obs[3 + r] = -R[2][r];   // R^T·(0,0,-1) 的第 r 个分量 = -R[2][r]

  for (let k = 0; k < N_JOINTS; k++) {
    const jid = jointId(joints[k]);
    const qa = read(model.jnt_qposadr, jid);
    const va = read(model.jnt_dofadr, jid);
    obs[6 + k] = read(data.qpos, qa) - defaultQ[k];
    obs[20 + k] = read(data.qvel, va);
    obs[34 + k] = lastAction ? lastAction[k] : 0;
  }
  obs[48] = vx || 0;
  return obs;
}

/** 一步物理：4 个 200 Hz 子步 = 1 个 50 Hz 控制步。 */
export function stepPhysics(mj, model, data, decimation = 4) {
  for (let k = 0; k < decimation; k++) mj.mj_step(model, data);
}

/** 直立度（机体 z 轴与世界 y 轴的点积近似；这里直接用 xquat 算 up·Y）。 */
export function uprightFromQuat(data, bodyId) {
  const bid = bodyId('trunk_base');
  const w = read(data.xquat, bid * 4), x = read(data.xquat, bid * 4 + 1);
  const y = read(data.xquat, bid * 4 + 2), z = read(data.xquat, bid * 4 + 3);
  return 1 - 2 * (x * x + z * z);   // y 分量相关的近似，够看趋势
}
