// 执行器语义（浏览器侧）——与宿主 `sim/host/actuators.py` **同一套公式**。
//
// | 模型 | MJCF | ctrl 是什么 | PD 在哪 |
// |---|---|---|---|
// | 位置 | `<position kp kv>` | 关节目标角 | MuJoCo 每个物理子步算 |
// | 扭矩 | `<motor ctrlrange>` | 力矩 | 我们每个物理子步算（**不能**每控制步算一次） |
//
// 实测（2026-11-12，见 docs/notes/g1-botlab.md §9）：扭矩模型下「每控制步算一次 PD」只能撑 20 步，
// 「每物理子步重算」能撑 82 步。两端必须都按子步算，否则同一份 bundle 两边行为不同。

/** 策略序 → 电机序：out[motorOrder[i]] = action[i]。 */
export function policyToMotor(action, motorOrder) {
  if (!motorOrder || motorOrder.length !== action.length
      || motorOrder.every((m, i) => m === i)) {
    return Float64Array.from(action);
  }
  const out = new Float64Array(motorOrder.length);
  for (let i = 0; i < motorOrder.length; i += 1) out[motorOrder[i]] = action[i];
  return out;
}

/** 目标角 = 默认角 + 缩放 × 动作（电机序）。 */
export function targetFromAction(action, { motorOrder, defaultQ, actionScale }) {
  const mapped = policyToMotor(action, motorOrder);
  const n = mapped.length;
  const out = new Float64Array(n);
  for (let i = 0; i < n; i += 1) {
    const base = defaultQ && defaultQ[i] !== undefined ? defaultQ[i] : 0;
    const gain = actionScale && actionScale[i] !== undefined ? actionScale[i] : 1;
    out[i] = base + gain * mapped[i];
  }
  return out;
}

/** tau = kp·(target − q) − kd·qd，再按力矩上限截断。 */
export function torqueFromTarget(target, qpos, qvel, { kp, kd, effortLimit }) {
  const n = target.length;
  const out = new Float64Array(n);
  for (let i = 0; i < n; i += 1) {
    const kP = kp && kp[i] !== undefined ? kp[i] : 0;
    const kD = kd && kd[i] !== undefined ? kd[i] : 0;
    let tau = kP * (target[i] - qpos[i]) - kD * qvel[i];
    const eff = effortLimit && effortLimit[i] !== undefined ? effortLimit[i] : 0;
    if (eff > 0) tau = Math.max(-eff, Math.min(eff, tau));
    out[i] = tau;
  }
  return out;
}

/** 契约里的 robot 段是否声明了扭矩执行器。 */
export function isTorque(contract) {
  return String(contract?.robot?.actuator_mode || 'position').toLowerCase() === 'torque';
}

/**
 * 按契约算出要写进 `data.ctrl` 的数组。
 * 返回 `{ ctrl, target }`；扭矩模式下 ctrl 是力矩，位置模式下 ctrl 就是 target。
 */
export function ctrlFromAction(contract, action, qpos, qvel) {
  const robot = contract?.robot || {};
  const target = targetFromAction(action, {
    motorOrder: robot.motor_order,
    defaultQ: robot.default_q,
    actionScale: robot.action_scale,
  });
  if (!isTorque(contract)) return { ctrl: target, target };
  const ctrl = torqueFromTarget(target, qpos, qvel, {
    kp: robot.kp,
    kd: robot.kd,
    effortLimit: robot.effort_limit,
  });
  return { ctrl, target };
}
