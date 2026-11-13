#!/usr/bin/env node
// UI / 渲染 / 交互探针（ADR-002）。
//
// 为什么不是「截图比对」：相机/光照/像素细节一变基线就红。这里断言的是
//   ①页面自述的数字（window.__probe.state()）
//   ②画面「非空 / 有变化」的像素统计
//   ③交互后的状态变化
// 截图只作为留证（给人看），不参与判定。
//
//   node tools/dev/probe.mjs --base http://127.0.0.1:8899 \
//        --probes tools/dev/probes/host-live.json [--shot-dir /tmp/probes] [--keep-open]

import { spawn } from 'node:child_process';
import fs from 'node:fs';
import path from 'node:path';

const opt = (name, dflt = null) => {
  const i = process.argv.indexOf(`--${name}`);
  return i >= 0 ? process.argv[i + 1] : dflt;
};
const flag = (name) => process.argv.includes(`--${name}`);

const BASE = (opt('base', 'http://127.0.0.1:8899')).replace(/\/$/, '');
const PROBE_FILES = (opt('probes', '')).split(',').filter(Boolean);
const SHOT_DIR = opt('shot-dir', '/tmp/probes');
const PORT = Number(process.env.CDP_PORT || 9333);
const PROFILE = process.env.CDP_PROFILE || `/tmp/cdp-probe-${Date.now()}`;
const VIEW = opt('view', '1280,900').split(',').map(Number);

if (!PROBE_FILES.length) {
  console.error('用法：node tools/dev/probe.mjs --probes tools/dev/probes/xxx.json [--base URL]');
  process.exit(2);
}
fs.mkdirSync(SHOT_DIR, { recursive: true });

function launchChrome(port, profile) {
  const proc = spawn('/usr/bin/google-chrome', [
  '--headless=new',
  `--remote-debugging-port=${port}`,
  `--user-data-dir=${profile}`,
  '--no-first-run', '--no-default-browser-check', '--no-sandbox', '--disable-dev-shm-usage',
  '--ignore-certificate-errors',
  '--enable-unsafe-swiftshader',
  `--window-size=${VIEW[0]},${VIEW[1]}`,
  'about:blank',
  ], { stdio: ['ignore', 'ignore', 'pipe'] });
  proc.stderr.on('data', (d) => { const t = String(d); if (/DevTools|error|Error/.test(t)) console.error('[chrome]', t.trim().slice(0, 160)); });
  return proc;
}

const sleep = (ms) => new Promise((r) => setTimeout(r, ms));

async function httpJson(port, p, method = 'GET') {
  const res = await fetch(`http://127.0.0.1:${port}${p}`, { method });
  return res.json();
}

async function waitTarget(port) {
  for (let i = 0; i < 150; i += 1) {
    try {
      const list = await httpJson(port, '/json/list');
      const page = list.find((t) => t.type === 'page');
      if (page) return page;
    } catch { /* 还没起 */ }
    await sleep(100);
  }
  throw new Error('chrome 没起来');
}

function client(wsUrl) {
  const ws = new WebSocket(wsUrl);
  let id = 0;
  const pending = new Map();
  const logs = [];
  ws.addEventListener('message', (ev) => {
    const msg = JSON.parse(ev.data);
    if (msg.id && pending.has(msg.id)) {
      const { resolve, reject } = pending.get(msg.id);
      pending.delete(msg.id);
      msg.error ? reject(new Error(msg.error.message)) : resolve(msg.result);
      return;
    }
    if (msg.method === 'Runtime.consoleAPICalled' && msg.params.type === 'error') {
      logs.push(msg.params.args.map((a) => a.value ?? a.description ?? '').join(' '));
    }
    if (msg.method === 'Runtime.exceptionThrown') {
      logs.push('EXCEPTION: ' + (msg.params.exceptionDetails?.exception?.description || ''));
    }
  });
  const ready = new Promise((r) => ws.addEventListener('open', r));
  const send = (method, params = {}) => new Promise((resolve, reject) => {
    const mid = ++id;
    pending.set(mid, { resolve, reject });
    ws.send(JSON.stringify({ id: mid, method, params }));
  });
  return { ready, send, logs, close: () => ws.close() };
}

async function evaluate(c, expr) {
  const r = await c.send('Runtime.evaluate', {
    expression: expr, returnByValue: true, awaitPromise: true,
  });
  if (r.exceptionDetails) throw new Error(r.exceptionDetails.text + ' ' + (r.exceptionDetails.exception?.description || ''));
  return r.result?.value;
}

/** 截一张图，返回 {非空像素比例, 亮度均值, 原始 base64} */
async function shot(c, name) {
  const { data } = await c.send('Page.captureScreenshot', { format: 'png' });
  const buf = Buffer.from(data, 'base64');
  const file = path.join(SHOT_DIR, `${name}-${Date.now()}.png`);
  fs.writeFileSync(file, buf);
  // 用页面里的 canvas 做像素统计（比解 PNG 简单，也够用）
  const stat = await evaluate(c, `(() => {
    const cv = document.querySelector('canvas');
    if (!cv) return { ratio: 0, mean: 0 };
    const g = document.createElement('canvas');
    const W = 160, H = 100;
    g.width = W; g.height = H;
    const ctx = g.getContext('2d');
    ctx.drawImage(cv, 0, 0, W, H);
    const d = ctx.getImageData(0, 0, W, H).data;
    let nonEmpty = 0, sum = 0;
    for (let i = 0; i < d.length; i += 4) {
      const v = (d[i] + d[i + 1] + d[i + 2]) / 3;
      sum += v;
      if (v > 24) nonEmpty += 1;
    }
    return { ratio: nonEmpty / (W * H), mean: sum / (W * H) };
  })()`);
  return { file, ...stat };
}

async function runProbe(page, spec, label) {
  const url = new URL(spec.url, BASE).toString();
  console.log(`\n▶ ${label}  ${url}`);
  await page.send('Page.enable');
  await page.send('Runtime.enable');
  await page.send('Page.navigate', { url });
  await sleep(1200);

  let fails = 0;
  let last = null;
  const ok = (why, detail = '') => console.log(`  \x1b[32mPASS\x1b[0m  ${why}${detail ? '  ' + detail : ''}`);
  const bad = (why, detail = '') => { fails += 1; console.log(`  \x1b[31mFAIL\x1b[0m  ${why}${detail ? '  ' + detail : ''}`); };

  for (const step of spec.steps) {
    if (step.wait) await sleep(step.wait);
    // 轮询等待（实时页面加载慢，固定 sleep 会误判）
    if (step.await) {
      const deadline = Date.now() + (step.timeout ?? 30000);
      let val = false;
      while (Date.now() < deadline) {
        try { val = await evaluate(page, step.await); } catch { val = false; }
        if (val) break;
        await sleep(250);
      }
      val ? ok(step.why || `等到 ${step.await}`) : bad(step.why || `等到 ${step.await}`, '超时');
    }
    if (step.capture) {
      await evaluate(page, `window.__cap = window.__cap || {}; window.__cap[${JSON.stringify(step.capture)}] = (${step.expr});`);
    }
    if (step.click) {
      await evaluate(page, `(() => {
        const el = document.querySelector(${JSON.stringify(step.click)});
        if (!el) throw new Error('找不到 ' + ${JSON.stringify(step.click)});
        el.click();
      })()`);
      await sleep(step.after ?? 500);
    }
    if (step.key) {
      await evaluate(page, `window.dispatchEvent(new KeyboardEvent('keydown', { key: ${JSON.stringify(step.key)} }))`);
      await sleep(step.after ?? 300);
    }
    if (step.assert === 'js') {
      let val;
      try { val = await evaluate(page, step.expr); } catch (e) { bad(step.why || step.expr, String(e.message).slice(0, 120)); continue; }
      val ? ok(step.why || step.expr, `= ${JSON.stringify(val)}`) : bad(step.why || step.expr, `= ${JSON.stringify(val)}`);
    }
    if (step.assert === 'pixels') {
      const s = await shot(page, 'pixels');
      last = s;
      (s.ratio >= (step.minRatio ?? 0.02))
        ? ok(step.why || '画面非空', `非空 ${(s.ratio * 100).toFixed(1)}%`)
        : bad(step.why || '画面非空', `非空 ${(s.ratio * 100).toFixed(1)}% < ${(step.minRatio ?? 0.02) * 100}%  ${s.file}`);
    }
    if (step.assert === 'changed') {
      // 一次 evaluate 里完成「采样 → 等 gap → 再采样 → 求逐像素平均差」，
      // 避免两次跨进程取大数组带来的时序/序列化问题。
      const gap = step.gap ?? 1500;
      const diff = await evaluate(page, `(async () => {
        const grab = () => {
          const cv = document.querySelector('canvas');
          const g = document.createElement('canvas');
          g.width = 160; g.height = 100;
          const ctx = g.getContext('2d');
          ctx.drawImage(cv, 0, 0, 160, 100);
          return ctx.getImageData(0, 0, 160, 100).data;
        };
        const st = () => (window.__probe ? window.__probe.state() : {});
        const s0 = st();
        const a = grab();
        await new Promise((r) => setTimeout(r, ${gap}));
        const s1 = st();
        const b = grab();
        let sum = 0, n = 0;
        for (let i = 0; i < a.length; i += 4) {
          sum += Math.abs(a[i] - b[i]) + Math.abs(a[i + 1] - b[i + 1]) + Math.abs(a[i + 2] - b[i + 2]);
          n += 3;
        }
        return { diff: sum / n, s0, s1 };
      })()`);
      const d = Number(diff.diff);
      const tail = `frames ${diff.s0.frames}→${diff.s1.frames}｜sim ${diff.s0.sim}→${diff.s1.sim}`;
      (d > (step.minDiff ?? 0.5))
        ? ok(step.why || '画面在变化', `逐像素平均差 ${d.toFixed(2)}｜${tail}`)
        : bad(step.why || '画面在变化', `两帧几乎一样（平均差 ${d.toFixed(2)}）｜${tail}`);
      await shot(page, 'changed');
    }
    if (step.assert === 'moved') {
      // 「几何在变」：相机跟随的页面（studio）里机器人始终居中，像素几乎不动，
      // 这时要断言的是场景对象自身的坐标在变（expr 返回可序列化值）。
      const gap = step.gap ?? 1500;
      const r = await evaluate(page, `(async () => {
        const pick = () => { try { return JSON.stringify(${step.expr}); } catch (e) { return 'ERR:' + e.message; } };
        const a = pick();
        await new Promise((r) => setTimeout(r, ${gap}));
        const b = pick();
        return { same: a === b, a, b };
      })()`);
      (!r.same)
        ? ok(step.why || '几何在变', r.b.slice(0, 90))
        : bad(step.why || '几何在变', `采样没变：${r.a.slice(0, 90)}`);
    }
    if (step.shot) {
      const s = await shot(page, step.shot);
      console.log(`        截图 ${s.file}`);
    }
  }

  const errs = page.logs.filter((l) => !/WebGL|Swiftshader|GPU stall/i.test(l));
  errs.length === 0 ? ok('无 console 错误') : bad('无 console 错误', errs.slice(0, 3).join(' | '));
  return fails;
}

async function main() {
  process.env.NO_PROXY = '127.0.0.1,localhost';
  process.env.no_proxy = '127.0.0.1,localhost';
  let fails = 0;
  // 一个探针一个干净浏览器：WebGL 上下文跑久了会丢（帧还在流、画面冻住），
  // 那不是页面的 bug，却会把「画面在变化」判成失败。
  for (const [i, file] of PROBE_FILES.entries()) {
    const spec = JSON.parse(fs.readFileSync(file, 'utf8'));
    const port = PORT + i;
    const profile = `${PROFILE}-${i}`;
    const chrome = launchChrome(port, profile);
    try {
      const target = await waitTarget(port);
      const page = client(target.webSocketDebuggerUrl);
      await page.ready;
      fails += await runProbe(page, spec, path.basename(file));
      page.close();
    } catch (e) {
      console.log(`  \x1b[31mFAIL\x1b[0m  ${path.basename(file)} 跑不起来：${e.message}`);
      fails += 1;
    } finally {
      chrome.kill('SIGTERM');
      await sleep(400);
    }
  }
  console.log(`\n${fails === 0 ? 'PASS' : `失败 ${fails} 项`}｜截图目录 ${SHOT_DIR}`);
  process.exit(fails === 0 ? 0 : 1);
}

main().catch((e) => { console.error(e); process.exit(2); });
