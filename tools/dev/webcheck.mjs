#!/usr/bin/env node
// 用 Chrome DevTools Protocol 自己开页面、跑 JS、抓控制台错误、截图。
// 不依赖 puppeteer（用 Node 22 内置 WebSocket + CDP 的 HTTP 接口）。
//
//   node tools/dev/webcheck.mjs <url> [--mobile] [--wait 8000] [--shot /tmp/x.png] [--eval "js"]
//
// 为什么有它：手机上看不到控制台，「页面白屏/没反应」只能靠猜。这个脚本用 Chrome CDP
// （Node 22 自带 WebSocket，不需要 puppeteer）把页面打开、跑 JS、抓控制台异常、截图，
// 于是「这类问题能不能自己验证」变成了 yes。
//
// 端口/配置目录可用 CDP_PORT / CDP_PROFILE 覆盖，避免多个实例互相踩。
import { spawn } from 'node:child_process';
import { writeFileSync } from 'node:fs';
import { setTimeout as sleep } from 'node:timers/promises';

const args = process.argv.slice(2);
const url = args[0];
const opt = (name, def) => {
  const i = args.indexOf(name);
  return i >= 0 ? args[i + 1] : def;
};
const mobile = args.includes('--mobile');
const waitMs = Number(opt('--wait', '8000'));
const shot = opt('--shot', null);
const save = opt('--save', null);
const evalJs = opt('--eval', null);
const clicks = (opt('--clicks', '') || '').split(';').filter(Boolean).map((pair) => {
  const [x, y] = pair.split(',').map(Number);
  return { x, y };
});
const clickWait = Number(opt('--click-wait', '900'));

const PORT = Number(process.env.CDP_PORT || 9333);
const userDir = process.env.CDP_PROFILE || '/tmp/cdp-profile';
const chrome = spawn('/usr/bin/google-chrome', [
  '--headless=new',
  `--remote-debugging-port=${PORT}`,
  `--user-data-dir=${userDir}`,
  '--no-first-run', '--no-default-browser-check', '--disable-gpu', '--no-sandbox', '--disable-dev-shm-usage',
  '--ignore-certificate-errors',
  ...(mobile ? ['--window-size=390,844'] : ['--window-size=1280,900']),
  'about:blank',
], { stdio: ['ignore', 'ignore', 'pipe'] });

const base = `http://127.0.0.1:${PORT}`;

async function http(path) {
  const res = await fetch(base + path);
  return res.json();
}

async function waitChrome() {
  for (let i = 0; i < 100; i += 1) {
    try {
      await http('/json/version');
      return true;
    } catch {
      await sleep(100);
    }
  }
  return false;
}

function cdp(wsUrl) {
  const ws = new WebSocket(wsUrl);
  let id = 0;
  const pending = new Map();
  const events = [];
  ws.addEventListener('message', (e) => {
    const msg = JSON.parse(e.data);
    if (msg.id && pending.has(msg.id)) {
      const { resolve, reject } = pending.get(msg.id);
      pending.delete(msg.id);
      if (msg.error) reject(new Error(JSON.stringify(msg.error)));
      else resolve(msg.result);
    } else if (msg.method) {
      events.push(msg);
    }
  });
  const ready = new Promise((resolve) => ws.addEventListener('open', resolve));
  return {
    events,
    send: async (method, params = {}) => {
      await ready;
      const mid = ++id;
      return new Promise((resolve, reject) => {
        pending.set(mid, { resolve, reject });
        ws.send(JSON.stringify({ id: mid, method, params }));
      });
    },
    close: () => ws.close(),
  };
}

const main = async () => {
  if (!(await waitChrome())) throw new Error('Chrome 没起来');
  const target = await fetch(`${base}/json/new?${encodeURIComponent(url)}`, { method: 'PUT' }).then((r) => r.json());
  const c = cdp(target.webSocketDebuggerUrl);
  await c.send('Runtime.enable');
  await c.send('Log.enable');
  await c.send('Page.enable');
  if (mobile) {
    await c.send('Emulation.setDeviceMetricsOverride', {
      width: 390, height: 844, deviceScaleFactor: 2, mobile: true,
    });
  }
  await sleep(waitMs);

  for (const pt of clicks) {
    for (const type of ['mousePressed', 'mouseReleased']) {
      await c.send('Input.dispatchMouseEvent', {
        type, x: pt.x, y: pt.y, button: 'left', clickCount: 1,
      });
    }
    console.log(`[click] ${pt.x},${pt.y}`);
    await sleep(clickWait);
  }

  const logs = c.events
    .filter((e) => e.method === 'Runtime.consoleAPICalled' || e.method === 'Log.entryAdded'
      || e.method === 'Runtime.exceptionThrown')
    .map((e) => {
      if (e.method === 'Runtime.consoleAPICalled') {
        return `[console.${e.params.type}] ` + e.params.args.map((a) => a.value ?? a.description ?? '').join(' ');
      }
      if (e.method === 'Log.entryAdded') return `[log.${e.params.entry.level}] ${e.params.entry.text}`;
      return `[exception] ${e.params.exceptionDetails?.text} ${e.params.exceptionDetails?.exception?.description || ''}`;
    });

  const info = await c.send('Runtime.evaluate', {
    expression: `(() => {
      const g = (id) => document.getElementById(id);
      const box = (el) => el ? { w: Math.round(el.getBoundingClientRect().width), h: Math.round(el.getBoundingClientRect().height) } : null;
      const out = {
        title: document.title,
        conn: g('conn')?.textContent || null,
        stat: g('stat')?.textContent || null,
        hud: g('hud-line')?.textContent || null,
        bundle: g('bundle')?.value || null,
        frame: (() => { const f = g('run-frame'); return f ? { src: f.src, shown: f.style.display, ...box(f) } : null; })(),
        canvas: box(g('c')),
        viewMsg: g('view-msg')?.textContent || null,
        viewMsgShown: g('view-msg') ? getComputedStyle(g('view-msg')).display : null,
        view: box(g('view')),
        mid: box(g('mid')),
      };
      return JSON.stringify(out);
    })()`,
    returnByValue: true,
  });

  let extra = null;
  if (evalJs) {
    const r = await c.send('Runtime.evaluate', { expression: evalJs, returnByValue: true, awaitPromise: true });
    extra = r.result?.value ?? r.result?.description ?? null;
  }

  if (shot) {
    const { data } = await c.send('Page.captureScreenshot', { format: 'png' });
    writeFileSync(shot, Buffer.from(data, 'base64'));
  }

  console.log('== DOM/布局 ==');
  console.log(info.result.value);
  if (extra !== null) {
    const text = typeof extra === 'string' ? extra : JSON.stringify(extra);
    if (save) {
      const { writeFileSync } = await import('node:fs');
      writeFileSync(save, text);
      console.log(`== eval == 已写入 ${save}（${text.length} 字符）`);
    } else {
      console.log('== eval ==');
      console.log(text);
    }
  }
  console.log('== 控制台/异常 ==');
  console.log(logs.length ? logs.slice(-30).join('\n') : '（无）');

  c.close();
  chrome.kill('SIGKILL');
  process.exit(0);
};

main().catch((e) => {
  console.error('CDP 失败：', e.message);
  chrome.kill('SIGKILL');
  process.exit(1);
});
