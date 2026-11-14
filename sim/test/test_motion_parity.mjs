// 跨端一致性：浏览器 `client/viewer/motion.mjs` 与宿主 `sim/plan/motion.py` + `sim/host/wbc_obs.py`
// 必须把同一份动作素材解析成同样的数值、算出同样的 67 维参考。
//
//   node sim/test/test_motion_parity.mjs
//
// 没有本地素材（/tmp/bt-all/.../dance12）时跳过。
import { execFileSync } from 'node:child_process';
import { existsSync, readFileSync } from 'node:fs';
import { dirname, join, resolve } from 'node:path';
import { fileURLToPath, pathToFileURL } from 'node:url';

const HERE = dirname(fileURLToPath(import.meta.url));
const ROOT = resolve(HERE, '../..');
const MOTION = '/tmp/bt-all/G1-WBC-Multi-Policy/assets/motions/dance12';

if (!existsSync(join(MOTION, 'metadata.json'))) {
  console.log('  跳过：没有本地动作素材（/tmp/bt-all/.../dance12）');
  process.exit(0);
}

const { loadMotion, parseBin, Playhead, motionRef, referenceBlock } =
  await import(pathToFileURL(join(ROOT, 'client/viewer/motion.mjs')).href);

let fails = 0;
const ok = (name, cond, detail = '') => {
  console.log(`  ${cond ? 'PASS' : 'FAIL'}  ${name}${detail ? `  ${detail}` : ''}`);
  if (!cond) fails += 1;
};

// 文件读取当 fetch
const fetchImpl = async (url) => {
  const buf = readFileSync(String(url).replace(/^file:\/\//, ''));
  return {
    json: async () => JSON.parse(buf.toString('utf8')),
    arrayBuffer: async () => buf.buffer.slice(buf.byteOffset, buf.byteOffset + buf.byteLength),
  };
};

const motion = await loadMotion(MOTION + '/', { fetchImpl });
ok('动作帧数 6574', motion.jointPos.frames === 6574, String(motion.jointPos.frames));
ok('关节通道 29 列', motion.jointPos.width === 29, String(motion.jointPos.width));
ok('body 通道 = 30 × 3 / 30 × 4',
  motion.bodyPos.width === 90 && motion.bodyQuat.width === 120,
  `${motion.bodyPos.width}/${motion.bodyQuat.width}`);

// 与 Python 解码逐值比对（取第 1 帧与第 100 帧的关节位置 + 第 5 帧 pelvis 四元数）
const py = `
import json, sys
sys.path.insert(0, ${JSON.stringify(join(ROOT, 'sim/plan'))})
import motion as M
d = ${JSON.stringify(MOTION)}
jp = M.load_motion(d, 'joint_pos')['data']
bq = M.load_motion(d, 'body_quat_w', indices=['pelvis'])['data']
print(json.dumps({"jp1": jp[1].tolist(), "jp100": jp[100].tolist(), "bq5": bq[5].tolist()}))
`;
const ref = JSON.parse(execFileSync('python3', ['-c', py], { encoding: 'utf8' }));
const diff = (a, b) => Math.max(...a.map((v, i) => Math.abs(v - b[i])));
ok('第 1 帧关节位置与宿主一致', diff(ref.jp1, Array.from(motion.jointPos.data.subarray(29, 58))) < 1e-6);
ok('第 100 帧关节位置与宿主一致', diff(ref.jp100, Array.from(motion.jointPos.data.subarray(100 * 29, 101 * 29))) < 1e-6);
ok('第 5 帧 pelvis 四元数与宿主一致',
  diff(ref.bq5, Array.from(motion.bodyQuat.data.subarray(5 * 120, 5 * 120 + 4))) < 1e-6);

// 参考段 67 维：与宿主 reference_block 同输入同输出
const ph = new Playhead(motion.jointPos.frames, { start: 10, frameInterval: 5 });
ok('播放头 idx=10 → (10,5)', JSON.stringify(ph.current()) === '[10,5]', JSON.stringify(ph.current()));
const baseQuat = [0.999, -0.01, 0.02, 0.03];
const mine = motionRef(motion, ph, baseQuat);
const py2 = `
import json, sys
sys.path.insert(0, ${JSON.stringify(join(ROOT, 'sim/plan'))})
sys.path.insert(0, ${JSON.stringify(join(ROOT, 'sim/host'))})
import motion as M, wbc_obs as W, numpy as np
d = ${JSON.stringify(MOTION)}
meta = json.load(open(d + '/metadata.json'))
names = meta['body_names']
jp = M.load_motion(d, 'joint_pos')['data']
jv = M.load_motion(d, 'joint_vel')['data']
bp = M.load_motion(d, 'body_pos_w', indices=names)['data'].reshape(-1, len(names), 3)
bq = M.load_motion(d, 'body_quat_w', indices=names)['data'].reshape(-1, len(names), 4)
ph = W.Playhead(jp.shape[0], start=10, frame_interval=5)
idx, last = ph.current()
ref = W.reference_block(joint_pos=jp[idx], joint_vel=jv[idx],
                        anchor_pos=bp[idx][0], anchor_quat=bq[idx][0],
                        last_anchor_pos=bp[last][0], last_anchor_quat=bq[last][0],
                        base_quat=np.array(${JSON.stringify(baseQuat)}), pause=False)
print(json.dumps(ref.tolist()))
`;
const theirs = JSON.parse(execFileSync('python3', ['-c', py2], { encoding: 'utf8' }));
const dref = diff(Array.from(mine), theirs);
ok('参考段 67 维与宿主一致', dref < 1e-5, `max|Δ|=${dref.toExponential(2)}`);

// 暂停时关节速度为 0
const phPaused = new Playhead(motion.jointPos.frames, { start: 10, frameInterval: 5 });
phPaused.pause(true);
const paused = motionRef(motion, phPaused, baseQuat);
ok('暂停时参考速度全 0', Array.from(paused.subarray(29, 58)).every((v) => v === 0));

// 容器头长度：宽度 29 → 24 字节
const raw = readFileSync(join(MOTION, 'joint_pos.bin'));
const parsed = parseBin(raw.buffer.slice(raw.byteOffset, raw.byteOffset + raw.byteLength), 29);
ok('头长度 24 字节（与宿主一致）', parsed.frames === 6574, String(parsed.frames));

if (fails) { console.log(`\n失败 ${fails} 项`); process.exit(1); }
console.log('\nPASS：浏览器与宿主的动作解析 / 参考段一致');
