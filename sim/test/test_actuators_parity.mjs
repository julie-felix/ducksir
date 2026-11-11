// 两端一致性：浏览器 `client/viewer/actuators.mjs` 与宿主 `sim/host/actuators.py`
// 必须用同一套公式算 ctrl（位置 / 扭矩），否则同一份 bundle 两边行为不同。
//
// 做法：用同一组随机输入跑两边，逐元素比对（容差 1e-12，都是 float64 的纯算术）。
//
//   node sim/test/test_actuators_parity.mjs
import { execFileSync } from 'node:child_process';
import { readFileSync, writeFileSync } from 'node:fs';
import { tmpdir } from 'node:os';
import { join } from 'node:path';
import { pathToFileURL } from 'node:url';
import { resolve, dirname } from 'node:path';
import { fileURLToPath } from 'node:url';

const HERE = dirname(fileURLToPath(import.meta.url));
const ROOT = resolve(HERE, '../..');
const { ctrlFromAction } = await import(pathToFileURL(join(ROOT, 'client/viewer/actuators.mjs')).href);

let fails = 0;
const ok = (name, cond, detail = '') => {
  console.log(`  ${cond ? 'PASS' : 'FAIL'}  ${name}${detail ? `  ${detail}` : ''}`);
  if (!cond) fails += 1;
};

// 固定的伪随机输入（不用 Math.random，保证可复现）
const n = 29;
const rnd = (seed) => {
  let s = seed;
  return () => {
    s = (s * 1103515245 + 12345) % 2147483648;
    return s / 2147483648 - 0.5;
  };
};
const r = rnd(42);
const action = Array.from({ length: n }, () => r() * 4);
const qpos = Array.from({ length: n }, () => r() * 0.5);
const qvel = Array.from({ length: n }, () => r() * 2);
const defaultQ = Array.from({ length: n }, () => r() * 0.3);
const actionScale = Array.from({ length: n }, () => 0.2 + Math.abs(r()) * 0.4);
const kp = Array.from({ length: n }, () => 20 + Math.abs(r()) * 200);
const kd = Array.from({ length: n }, () => 1 + Math.abs(r()) * 5);
const effortLimit = Array.from({ length: n }, () => 30 + Math.abs(r()) * 60);
const motorOrder = [0, 6, 12, 1, 7, 13, 2, 8, 14, 3, 9, 15, 22, 4, 10, 16, 23, 5, 11, 17, 24,
  18, 25, 19, 26, 20, 27, 21, 28];

const mkContract = (mode) => ({
  robot: {
    joints: Array.from({ length: n }, (_, i) => `j${i}`),
    default_q: defaultQ,
    action_scale: actionScale,
    actuator_mode: mode,
    kp, kd, effort_limit: effortLimit,
    motor_order: motorOrder,
  },
});

// 浏览器侧
const jsPos = ctrlFromAction(mkContract('position'), action, qpos, qvel);
const jsTor = ctrlFromAction(mkContract('torque'), action, qpos, qvel);

// 宿主侧：把同一组输入喂给 python
const py = `
import json, sys
sys.path.insert(0, ${JSON.stringify(join(ROOT, 'sim/host'))})
import actuators as A
inp = json.load(open(sys.argv[1]))
preset = {"actuatorMode": inp["mode"], "defaultJointPositions": inp["defaultQ"],
          "actionScale": inp["actionScale"], "kp": inp["kp"], "kd": inp["kd"],
          "effortLimit": inp["effortLimit"], "motorJointOrder": inp["motorOrder"]}
ctrl, target = A.ctrl_from_action(inp["action"], preset, qpos=inp["qpos"], qvel=inp["qvel"])
print(json.dumps({"ctrl": list(map(float, ctrl)), "target": list(map(float, target))}))
`;
const inputPath = join(tmpdir(), 'actuator-parity.json');
writeFileSync(inputPath, JSON.stringify({
  action, qpos, qvel, defaultQ, actionScale, kp, kd, effortLimit, motorOrder, mode: 'torque',
}));
const out = JSON.parse(execFileSync('python3', ['-c', py, inputPath], { encoding: 'utf8' }));
const maxDiff = (a, b) => {
  let m = 0;
  for (let i = 0; i < a.length; i += 1) m = Math.max(m, Math.abs(a[i] - b[i]));
  return m;
};

ok('位置模式：浏览器返回同一 target', maxDiff(jsPos.ctrl, jsPos.target) === 0);
const dPos = (() => {
  // 位置模式下 python 的 ctrl 也应等于 target；用 torque 输入文件的 mode 覆盖
  const inp = JSON.parse(readFileSync(inputPath, 'utf8'));
  inp.mode = 'position';
  writeFileSync(inputPath, JSON.stringify(inp));
  const p2 = JSON.parse(execFileSync('python3', ['-c', py, inputPath], { encoding: 'utf8' }));
  return maxDiff(jsPos.ctrl, p2.ctrl);
})();
ok('位置模式：两端 ctrl 完全一致', dPos === 0, `max|Δ|=${dPos.toExponential(2)}`);

ok('扭矩模式：两端 target 一致', maxDiff(jsTor.target, out.target) === 0,
  `max|Δ|=${maxDiff(jsTor.target, out.target).toExponential(2)}`);
ok('扭矩模式：两端 ctrl 一致', maxDiff(jsTor.ctrl, out.ctrl) === 0,
  `max|Δ|=${maxDiff(jsTor.ctrl, out.ctrl).toExponential(2)}`);
ok('扭矩模式确实被 effortLimit 截断',
  jsTor.ctrl.some((v, i) => Math.abs(v) === effortLimit[i]), '');
ok('两种模式的 ctrl 不同（语义真的分开了）', maxDiff(jsTor.ctrl, jsPos.ctrl) > 1e-6);

if (fails) { console.log(`\n失败 ${fails} 项`); process.exit(1); }
console.log('\nPASS：浏览器与宿主的执行器语义逐元素一致');
