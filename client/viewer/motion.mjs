// 动作参考（WBC / AMP）在浏览器侧的最小实现——与宿主 `sim/plan/motion.py` + `sim/host/wbc_obs.py`
// **同一套公式**，两边算出来的 67 维必须一致（有 cross-end parity 测试守着）。
//
// `.bin` 容器格式（2026-11-12 从 dance12 解出来，fps 用「位置求导 = 速度」验证到 0.00e+0）：
//   `NPZ\0` 魔数 + u32 头 + 行主序 float32
//   头长度 = {16,20,24,28,32} 里**唯一**能让 (文件长 − h) 被 (4 × 每帧宽度) 整除的那个
//   每帧宽度：关节通道 = dof 数；body 通道 = body 数 × 3（位置）/ × 4（四元数）
//
// 通道名（与 meta 对齐）：joint_pos / joint_vel / body_pos_w / body_quat_w

const MAGIC = [0x4e, 0x50, 0x5a, 0x00]; // "NPZ\0"

function headerLen(buffer, widthFloats) {
  const size = buffer.byteLength;
  for (const [i, b] of MAGIC.entries()) {
    if (new Uint8Array(buffer, i, 1)[0] !== b) {
      throw new Error('不是 NPZ 容器（魔数不符）');
    }
  }
  const cands = [];
  for (const h of [16, 20, 24, 28, 32]) {
    const rest = size - h;
    if (rest > 0 && rest % (4 * widthFloats) === 0) cands.push(h);
  }
  if (cands.length !== 1) {
    throw new Error(`.bin 头长度不唯一（候选 ${JSON.stringify(cands)}，宽 ${widthFloats}）`);
  }
  return cands[0];
}

/** 读一个 .bin 通道 → {frames, width, data: Float32Array（行主序）}。 */
export function parseBin(buffer, widthFloats) {
  const head = headerLen(buffer, widthFloats);
  const frames = (buffer.byteLength - head) / (4 * widthFloats);
  const data = new Float32Array(buffer, head, frames * widthFloats);
  return { frames, width: widthFloats, data };
}

/**
 * 读一个动作目录：`metadata.json` 给名字表，各通道一个 .bin。
 * `fetchImpl` 可注入（测试用），默认 `fetch`。
 */
export async function loadMotion(base, { fetchImpl = fetch } = {}) {
  const meta = await (await fetchImpl(base + 'metadata.json')).json();
  const dofNames = meta.dof_names || meta.joint_names || [];
  const bodyNames = meta.body_names || [];
  const get = async (name, width) => {
    const res = await fetchImpl(`${base}${name}.bin`);
    const buf = await res.arrayBuffer();
    return parseBin(buf, width);
  };
  return {
    meta,
    dofNames,
    bodyNames,
    fps: Number(meta.fps) || null,
    jointPos: await get('joint_pos', dofNames.length),
    jointVel: await get('joint_vel', dofNames.length),
    bodyPos: await get('body_pos_w', bodyNames.length * 3),
    bodyQuat: await get('body_quat_w', bodyNames.length * 4),
  };
}

// ---- 播放头（与宿主 wbc_obs.Playhead 同语义）----

export class Playhead {
  constructor(frameCount, { start = 0, frameInterval = 5, end = null } = {}) {
    if (frameCount < 2) throw new Error('动作参考至少 2 帧');
    this.end = end ?? frameCount - 1;
    this.frameInterval = Math.max(1, frameInterval | 0);
    this.referIdx = Math.max(1, start | 0);
    this.paused = false;
  }

  clamp(i) {
    if (i >= this.end) return this.end;
    if (i <= 1) return 1;
    return i;
  }

  /** 返回 [idx, lastIdx]；暂停时两者相同。 */
  current() {
    const idx = this.clamp(this.referIdx);
    if (this.paused) return [idx, idx];
    return [idx, this.clamp(idx - this.frameInterval)];
  }

  advance() { if (!this.paused) this.referIdx += 1; }
  pause(flag = true) { this.paused = !!flag; }
  atEnd() { return this.referIdx >= this.end; }
}

// ---- 参考段 67 维（与宿主 reference_block 同公式）----

export function quatMul(a, b) {
  const [aw, ax, ay, az] = a;
  const [bw, bx, by, bz] = b;
  return [
    aw * bw - ax * bx - ay * by - az * bz,
    aw * bx + ax * bw + ay * bz - az * by,
    aw * by - ax * bz + ay * bw + az * bx,
    aw * bz + ax * by - ay * bx + az * bw,
  ];
}
export const quatConj = (q) => [q[0], -q[1], -q[2], -q[3]];

export function quatToMat(q) {
  const [w, x, y, z] = q;
  return [
    [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
    [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
    [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
  ];
}

export function yawQuat(q) {
  const [w, x, y, z] = q;
  const yaw = Math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z));
  return [Math.cos(yaw / 2), 0, 0, Math.sin(yaw / 2)];
}

function matT(m) {
  return [0, 1, 2].map((i) => [0, 1, 2].map((j) => m[j][i]));
}
function matMul(a, b) {
  return a.map((row, i) => [0, 1, 2].map((j) => row[0] * b[0][j] + row[1] * b[1][j] + row[2] * b[2][j]));
}
function matVec(m, v) {
  return [0, 1, 2].map((i) => m[i][0] * v[0] + m[i][1] * v[1] + m[i][2] * v[2]);
}

/** 参考段 67 维：与 botlab 引擎 `write_motion_reference()` 逐字同公式。
 *
 *     last_inv    = quat_inverse(last_quat)
 *     target_pos  = quat_apply(last_inv, cur_pos - last_pos)
 *     target_quat = last_inv * cur_quat
 *     6D          = mat[0], mat[1], mat[3], mat[4], mat[6], mat[7]
 *
 * `baseQuat` 只为兼容保留：这一版**不做基座偏航对齐**。
 */
export function referenceBlock({
  jointPos, jointVel, anchorPos, anchorQuat, lastAnchorPos, lastAnchorQuat, baseQuat, pause = false,
}) {
  const out = new Float32Array(67);
  out.set(jointPos, 0);
  if (!pause) out.set(jointVel, 29);
  const lastInv = quatConj(lastAnchorQuat);
  const R = quatToMat(lastInv);
  const dp = [anchorPos[0] - lastAnchorPos[0],
              anchorPos[1] - lastAnchorPos[1],
              anchorPos[2] - lastAnchorPos[2]];
  out[58] = R[0][0] * dp[0] + R[0][1] * dp[1] + R[0][2] * dp[2];
  out[59] = R[1][0] * dp[0] + R[1][1] * dp[1] + R[1][2] * dp[2];
  out[60] = R[2][0] * dp[0] + R[2][1] * dp[1] + R[2][2] * dp[2];
  const m = quatToMat(quatMul(lastInv, anchorQuat));
  out[61] = m[0][0]; out[62] = m[0][1];
  out[63] = m[1][0]; out[64] = m[1][1];
  out[65] = m[2][0]; out[66] = m[2][1];
  return out;
}

/** 从已加载的动作里取某帧某通道的切片（锚点默认第 0 个 body，与参考实现 `_anchor_idx=0` 一致）。 */
export function frameSlice(channel, frame, width) {
  const base = frame * width;
  return channel.data.subarray(base, base + width);
}

/** 一次算好当前参考段：动作 + 播放头 + 基座四元数 → 67 维。 */
export function motionRef(motion, playhead, baseQuat, anchor = 0) {
  const [idx, last] = playhead.current();
  const nb = motion.bodyNames.length;
  const ap = frameSlice(motion.bodyPos, idx, nb * 3).slice(anchor * 3, anchor * 3 + 3);
  const aq = frameSlice(motion.bodyQuat, idx, nb * 4).slice(anchor * 4, anchor * 4 + 4);
  const lp = frameSlice(motion.bodyPos, last, nb * 3).slice(anchor * 3, anchor * 3 + 3);
  const lq = frameSlice(motion.bodyQuat, last, nb * 4).slice(anchor * 4, anchor * 4 + 4);
  return referenceBlock({
    jointPos: frameSlice(motion.jointPos, idx, motion.dofNames.length),
    jointVel: frameSlice(motion.jointVel, idx, motion.dofNames.length),
    anchorPos: ap, anchorQuat: aq,
    lastAnchorPos: lp, lastAnchorQuat: lq,
    baseQuat,
    pause: playhead.paused,
  });
}
