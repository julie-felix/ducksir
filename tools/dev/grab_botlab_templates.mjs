#!/usr/bin/env node
// 逐个打开 BotLab 的 8 个示例项目，用「文件 → 保存项目」把它们打包下载下来。
//
//   node tools/dev/grab_botlab_templates.mjs /tmp/botlab-downloads
//   WANT=g1-wbc,microduck node tools/dev/grab_botlab_templates.mjs /tmp/only-two   # 只补某几个
//
// 依赖：本机 Chrome（/usr/bin/google-chrome）。用 CDP 真实鼠标事件点菜单，
// 下载目录靠 Browser.setDownloadBehavior 指定。产物是 .botlab.zip，解压后就是完整项目。
// 依赖 webcheck 同一套 CDP 手法；下载行为用 Browser.setDownloadBehavior 指定目录。
import { spawn } from 'node:child_process';
import { mkdirSync, readdirSync, statSync } from 'node:fs';
import { setTimeout as sleep } from 'node:timers/promises';

const OUT = process.argv[2] || '/tmp/botlab-downloads';
mkdirSync(OUT, { recursive: true });

const PORT = Number(process.env.CDP_PORT || 9700);
const chrome = spawn('/usr/bin/google-chrome', [
  '--headless=new', `--remote-debugging-port=${PORT}`, `--user-data-dir=${process.env.CDP_PROFILE || '/tmp/cdp-grab'}`,
  '--no-first-run', '--no-default-browser-check', '--disable-gpu', '--no-sandbox', '--disable-dev-shm-usage',
  '--ignore-certificate-errors', '--window-size=1280,900', 'about:blank',
], { stdio: ['ignore', 'ignore', 'pipe'] });

const base = `http://127.0.0.1:${PORT}`;
const waitChrome = async () => {
  for (let i = 0; i < 100; i++) {
    try { await (await fetch(`${base}/json/version`)).json(); return true; } catch { await sleep(100); }
  }
  return false;
};

function cdp(wsUrl) {
  const ws = new WebSocket(wsUrl);
  let id = 0;
  const pending = new Map();
  ws.addEventListener('message', (e) => {
    const m = JSON.parse(e.data);
    if (m.id && pending.has(m.id)) {
      const { resolve, reject } = pending.get(m.id);
      pending.delete(m.id);
      m.error ? reject(new Error(JSON.stringify(m.error))) : resolve(m.result);
    }
  });
  const ready = new Promise((r) => ws.addEventListener('open', r));
  return {
    send: async (method, params = {}) => {
      await ready;
      const mid = ++id;
      return new Promise((res, rej) => { pending.set(mid, { resolve: res, reject: rej }); ws.send(JSON.stringify({ id: mid, method, params })); });
    },
    close: () => ws.close(),
  };
}

const click = async (c, x, y) => {
  for (const type of ['mousePressed', 'mouseReleased']) {
    await c.send('Input.dispatchMouseEvent', { type, x, y, button: 'left', clickCount: 1 });
  }
  await sleep(700);
};

// 只补两个还没下来的：Instinct-SuperMimic（第 4 项）与 UniSuperMimic（第 6 项）
const ALL = [
  'blank', 'g1-gmtrack', 'g1-hiking-parkour', 'g1-instinct-supermimic', 'g1-php-parkour',
  'g1-unisupermimic', 'g1-wbc', 'go2w-lab2mj', 'microduck',
];
const WANT = (process.env.WANT || '').split(',').filter(Boolean);
const EXAMPLES = WANT.length ? WANT : ALL.slice(1);

const main = async () => {
  if (!(await waitChrome())) throw new Error('chrome 没起来');
  const t = await fetch(`${base}/json/new?${encodeURIComponent('https://botlab.enkeebot.com/')}`, { method: 'PUT' }).then((r) => r.json());
  const c = cdp(t.webSocketDebuggerUrl);
  await c.send('Page.enable');
  await c.send('Runtime.enable');
  await c.send('Browser.setDownloadBehavior', { behavior: 'allow', downloadPath: OUT });
  await sleep(15000);

  for (let i = 0; i < EXAMPLES.length; i += 1) {
    const name = EXAMPLES[i];
    if (!ALL.includes(name)) throw new Error('未知示例 ' + name);
    // 文件 → 打开示例 → 第 i 项（空白项目是第 0 项，所以 +1）
    await click(c, 122, 24);
    const y = 345 + 32 * (ALL.indexOf(name));
    await click(c, 205, y);
    process.stdout.write(`[open] ${name} (y=${y}) … `);
    await sleep(9000);
    const cur = await c.send('Runtime.evaluate', {
      expression: `document.querySelector('header, [class*=topbar], body').innerText.slice(0, 60).replace(/\\s+/g,' ')`,
      returnByValue: true,
    });
    console.log('→', String(cur.result.value).slice(0, 40));
    // 文件 → 保存项目
    await click(c, 122, 24);
    await click(c, 160, 246);
    await sleep(30000);
  }

  console.log('下载目录内容：');
  for (const f of readdirSync(OUT)) {
    const p = `${OUT}/${f}`;
    console.log(' ', f, statSync(p).size, 'B');
  }
  c.close();
  chrome.kill('SIGKILL');
  process.exit(0);
};

main().catch((e) => { console.error('失败：', e.message); chrome.kill('SIGKILL'); process.exit(1); });
