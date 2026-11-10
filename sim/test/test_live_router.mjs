// 插件侧实时后端（/ducksir-live）的 headless 测试：
// 真起一个仿真进程，走它自己的 HTTP 代理验「启动 → 状态 → 场景 → 帧 → 停止」。
//   node sim/test/test_live_router.mjs
import { createLive } from '../../client/src/live.mjs';
import { resolve } from 'node:path';
import http from 'node:http';

const ROOT = resolve(import.meta.dirname, '../..');
const live = createLive({ repo: ROOT });

// 起一个最小 HTTP 服务把 live.handle 挂到 /ducksir-live
const server = http.createServer((req, res) => {
  const url = new URL(req.url ?? '/', 'http://localhost');
  const rest = url.pathname.replace(/^\/ducksir-live/, '') + url.search;
  live.handle(req, res, rest).catch((e) => { res.writeHead(500); res.end(String(e)); });
});
await new Promise((r) => server.listen(0, '127.0.0.1', r));
const port = server.address().port;
const base = `http://127.0.0.1:${port}/ducksir-live`;

let bad = 0;
const ok = (label, cond, detail = '') => {
  console.log(`  ${cond ? 'PASS' : 'FAIL'}  ${label}${detail ? '  ' + detail : ''}`);
  if (!cond) bad += 1;
};
const getJson = async (p, init) => {
  const r = await fetch(base + p, init);
  return r.json();
};

ok('未启动时 status 报告未运行', (await getJson('/status')).running === false);

const started = await getJson('/start', {
  method: 'POST', headers: { 'content-type': 'application/json' },
  body: JSON.stringify({ project: 'sim/test/fixtures/project_demo.py', seconds: 60 }),
});
ok('/start 成功', started.ok === true, JSON.stringify(started).slice(0, 80));

const st = await getJson('/status');
ok('status 报告运行中且有端口', st.running === true && st.port > 0, `port=${st.port}`);

const scene = await getJson('/scene');
ok('/scene 代理通，返回 body 列表', Array.isArray(scene.bodies) && scene.bodies.length > 0, `${scene.bodies?.length} bodies`);

const files = await getJson('/files');
ok('/files 代理通', Array.isArray(files.files) && files.files.length > 0, files.files?.slice(0, 2));

// SSE：读几行
const frames = await new Promise((resolveFrames) => {
  const out = [];
  http.get(base + '/stream', (res) => {
    res.on('data', (chunk) => {
      const text = chunk.toString();
      for (const line of text.split('\n')) {
        if (line.startsWith('data: ')) out.push(line.slice(6));
      }
      if (out.length >= 3) { res.destroy(); resolveFrames(out); }
    });
  }).on('error', () => resolveFrames(out));
  setTimeout(() => resolveFrames(out), 5000);
});
ok('/stream 代理出帧', frames.length >= 3, `${frames.length} 帧`);

const bundles = await getJson('/bundles');
const names = (bundles.bundles || []).map((b) => b.name);
ok('/bundles 列出可运行的包', names.length > 0, names.slice(0, 4));

const stopped = await getJson('/stop', { method: 'POST' });
ok('/stop 成功', stopped.ok === true);
ok('停止后 status 未运行', (await getJson('/status')).running === false);

server.close();
console.log(bad === 0 ? '\nPASS：插件侧实时后端可用（手机能走同源地址）' : `\n${bad} 条失败`);
process.exit(bad === 0 ? 0 : 1);
