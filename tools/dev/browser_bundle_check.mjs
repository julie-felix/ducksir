#!/usr/bin/env node
// 浏览器侧跑 bundle 的自检：起静态站 → 无头 Chrome 打开 play.html?bundle=… → 读 HUD → 判定。
//
//   node tools/dev/browser_bundle_check.mjs [bundle 相对路径]
//
// 默认检查 `build/release/g1-wbc`。判据与宿主一致：基座高度 > 0.55 m（没倒）。
// 需要 repo 根目录能静态服务（脚本自己起 python3 -m http.server，跑完关掉）。
import { spawn } from 'node:child_process';
import { dirname, join, resolve } from 'node:path';
import { fileURLToPath } from 'node:url';

const HERE = dirname(fileURLToPath(import.meta.url));
const ROOT = resolve(HERE, '../..');
const BUNDLE = process.argv[2] || 'build/release/g1-wbc';
const PORT = 8831;
const URL = `http://127.0.0.1:${PORT}/client/viewer/play.html?bundle=/${BUNDLE.replace(/^\/+/, '')}/`;
const MIN_Z = Number(process.env.MIN_Z || 0.55);
// EXPECT_FALL=1：这个策略本来就站不住（loco 零速指令在 botlab 里也会倒），只验「页面在跑」
const EXPECT_FALL = process.env.EXPECT_FALL === '1';
const WAIT_MS = Number(process.env.WAIT_MS || 90000);
const RUN_MS = Number(process.env.RUN_MS || 20000);

const server = spawn('python3', ['-m', 'http.server', String(PORT), '--bind', '127.0.0.1'],
  { cwd: ROOT, stdio: 'ignore' });
const sleep = (ms) => new Promise((r) => setTimeout(r, ms));
await sleep(1200);

let code = 0;
try {
  const evalCode = `(async () => {
    await new Promise(r => setTimeout(r, ${RUN_MS}));
    const t = document.body.innerText.replace(/\\n+/g, ' | ');
    return JSON.stringify({
      z: (t.match(/基座 ([0-9.]+) m/) || [])[1],
      sim: (t.match(/仿真 ([0-9.]+) s/) || [])[1],
      policy: (t.match(/策略 ([a-z0-9-]+)/i) || [])[1],
    });
  })()`;
  const args = [join(HERE, 'webcheck.mjs'), URL, '--wait', String(WAIT_MS), '--eval', evalCode];
  const child = spawn(process.execPath, args, {
    cwd: ROOT,
    // 每次用新 profile：复用旧 profile 会命中浏览器缓存，跑到过期的 play.js
    env: {
      ...process.env,
      CDP_PORT: process.env.CDP_PORT || '9741',
      CDP_PROFILE: process.env.CDP_PROFILE || `/tmp/cdp-bundle-check-${Date.now()}`,
    },
    stdio: ['ignore', 'pipe', 'pipe'],
  });
  let out = '';
  child.stdout.on('data', (b) => { out += b.toString(); });
  child.stderr.on('data', (b) => { out += b.toString(); });
  const done = await new Promise((r) => child.on('exit', r));

  const m = out.match(/"z":"([0-9.]+)"/);
  const sim = (out.match(/"sim":"([0-9.]+)"/) || [])[1];
  const ranEnough = Number(sim || 0) >= 5;
  if (!ranEnough) {
    console.error(`页面只跑了 ${sim || 0} s 仿真，不能算过（可能还没开始步进，或加载失败）`);
    code = 1;
  }
  if (!m) {
    console.error('没读到 HUD：\n' + out.slice(-1200));
    code = 1;
  } else {
    const z = Number(m[1]);
    console.log(`${BUNDLE}：基座 ${z.toFixed(3)} m（仿真实时 ${sim || '?'} s，exit ${done}）`);
    if (EXPECT_FALL) {
      console.log(z <= MIN_Z
        ? `PASS：按预期倒下了（这个策略本来就站不住，和 botlab 一致）`
        : `注意：没倒（z=${z.toFixed(3)}），但这不构成失败`);
    } else if (!(z > MIN_Z)) {
      console.error(`FAIL：基座 ${z.toFixed(3)} m ≤ ${MIN_Z} m（倒了）`);
      code = 1;
    } else if (ranEnough) {
      console.log(`PASS：浏览器里站稳（> ${MIN_Z} m）`);
    }
  }
} finally {
  server.kill('SIGTERM');
}
process.exit(code);
