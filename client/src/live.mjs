/**
 * 工作台的「实时后端」：在插件侧起一个仿真进程，并把它的显示端点代理到同源路径下。
 *
 * 为什么需要这层：`ducksir run` 起的显示服务绑在跑 CLI 那台机器的 127.0.0.1 上，
 * 手机/别的机器根本访问不到。这里改成「插件宿主负责起进程 + 反向代理」，面板和手机就能用同源地址。
 *
 *   POST /ducksir-live/start   {project?, seconds?, port?}
 *   POST /ducksir-live/stop
 *   GET  /ducksir-live/status
 *   GET  /ducksir-live/{scene,stream,files,file,command}   → 转发给 runner 的显示服务
 */

import { createRequire } from 'node:module'
import { spawn } from 'node:child_process'
import http from 'node:http'
import os from 'node:os'
import path from 'node:path'

const require = createRequire(import.meta.url)

const PROXY_PATHS = ['/scene', '/stream', '/files', '/file', '/command']

/** 找一个空闲端口（绑 0 让系统分配，然后读回来）。 */
function freePort() {
  return new Promise((resolve, reject) => {
    const srv = http.createServer()
    srv.once('error', reject)
    srv.listen(0, '127.0.0.1', () => {
      const { port } = srv.address()
      srv.close(() => resolve(port))
    })
  })
}

function waitReady(port, timeoutMs = 15000) {
  const end = Date.now() + timeoutMs
  return new Promise((resolve) => {
    const tick = () => {
      const req = http.get({ host: '127.0.0.1', port, path: '/scene', timeout: 800 }, (res) => {
        res.resume()
        resolve(res.statusCode === 200)
      })
      req.on('error', retry)
      req.on('timeout', () => { req.destroy(); retry() })
      function retry() {
        if (Date.now() > end) resolve(false)
        else setTimeout(tick, 150)
      }
    }
    tick()
  })
}

/**
 * 拼出子进程要用的 PYTHONPATH。
 *
 * 插件是 dsh web 起的，它的环境里通常没有 `docs/env.md` 那套依赖路径，
 * 直接 spawn 会死在 `ModuleNotFoundError: numpy`。这里按文档约定补上：
 * `~/.local/share/ducksir/pylibs`，可用 `DUCKSIR_PYLIBS` 覆盖。
 */
function pythonPath() {
  const libs = process.env.DUCKSIR_PYLIBS || path.join(os.homedir(), '.local/share/ducksir/pylibs')
  const parts = [libs, process.env.PYTHONPATH || ''].filter(Boolean)
  return parts.join(path.delimiter)
}

/**
 * 建一个实时后端。
 * @param {object} options
 * @param {string} options.repo 仓库根目录。
 * @param {string} [options.python] python 可执行文件。
 * @param {string[]} [options.env] 需要透传的环境（如 PYTHONPATH）。
 */
let SINGLETON = null
const STARTED_AT = Date.now()
let REQUESTS = 0

/**
 * 取进程级单例。
 *
 * 为什么不是每次注册都 new：dsh 的 webServer 注册可能被重复执行（热重载 / 多次 inject），
 * 每个实例各自持有 `session`，于是「A 实例启动的仿真，B 实例的 /status 说不存在」——
 * 表现就是「明明在跑却说连接不上」（2026-11-11 排到的坑）。
 */
export function getLive(options) {
  if (!SINGLETON) SINGLETON = createLive(options)
  return SINGLETON
}

export function createLive({ repo, python = 'python3' }) {
  /** @type {{child: import('node:child_process').ChildProcess, port: number, project: string}|null} */
  let session = null
  let lastError = ''

  async function start({ project = 'sim/test/fixtures/project_demo.py', seconds = 600 } = {}) {
    console.log('[ducksir-live] start', project)
    await stop()
    const port = await freePort()
    const args = [path.join(repo, 'sim/host/run_project.py'),
      '--project', project, '--seconds', String(seconds), '--port', String(port), '--no-open']
    const child = spawn(python, args, {
      cwd: repo,
      env: { ...process.env, PYTHONPATH: pythonPath() },
      stdio: ['ignore', 'pipe', 'pipe'],
    })
    let log = ''
    child.stdout.on('data', (d) => { log = (log + d).slice(-4000) })
    child.stderr.on('data', (d) => { log = (log + d).slice(-4000) })
    child.on('exit', () => { if (session && session.child === child) session = null })
    const ready = await waitReady(port)
    if (!ready) {
      lastError = log || '启动超时'
      console.warn('[ducksir-live] 启动失败：' + lastError.slice(-400))
      child.kill('SIGINT')
      return { ok: false, error: lastError }
    }
    console.log('[ducksir-live] ready on', port)
    session = { child, port, project }
    return { ok: true, port, project }
  }

  async function stop() {
    if (!session) return { ok: true, stopped: false }
    const { child } = session
    session = null
    child.kill('SIGINT')
    await new Promise((r) => {
      const t = setTimeout(() => { child.kill('SIGKILL'); r() }, 4000)
      child.once('exit', () => { clearTimeout(t); r() })
    })
    return { ok: true, stopped: true }
  }

  function status() {
    return { running: Boolean(session), port: session?.port ?? null, project: session?.project ?? null, error: lastError }
  }

  /**
   * 把请求转发给 runner 的显示服务（SSE 需要逐行 pipe，不能缓冲）。
   * `rest` 是去掉 `/ducksir-live` 前缀后的路径——**必须用它**，不能用 `req.url`
   * （那是外层地址，转发过去会 404，2026-11-11 踩过）。
   */
  function proxy(req, res, rest) {
    if (!session) {
      res.writeHead(503, { 'content-type': 'application/json; charset=utf-8' })
      res.end(JSON.stringify({ error: '没有在跑的仿真', hint: 'POST /ducksir-live/start' }))
      return
    }
    const upstream = http.request({
      host: '127.0.0.1', port: session.port, path: rest, method: req.method,
      headers: { ...req.headers, host: `127.0.0.1:${session.port}` },
    }, (up) => {
      res.writeHead(up.statusCode ?? 502, up.headers)
      up.pipe(res)
    })
    upstream.on('error', (e) => {
      res.writeHead(502, { 'content-type': 'application/json; charset=utf-8' })
      res.end(JSON.stringify({ error: String(e) }))
    })
    req.pipe(upstream)
  }

  /**
   * `/ducksir-live` 前缀的处理入口。
   * @param {import('node:http').IncomingMessage} req 请求。
   * @param {import('node:http').ServerResponse} res 响应。
   * @param {string} rest 前缀之后的部分（含查询串）。
   */
  async function handle(req, res, rest) {
    const [pathname] = rest.split('?')
    const json = (code, body) => {
      res.writeHead(code, { 'content-type': 'application/json; charset=utf-8' })
      res.end(JSON.stringify(body))
    }
    REQUESTS += 1
    if (pathname === '/status') return json(200, status())
    if (pathname === '/bundles') {
      // 列出可运行的 bundle：build/release/* 里有 contract.json 的目录
      const fs = await import('node:fs/promises')
      const base = path.join(repo, 'build', 'release')
      let names = []
      try {
        names = (await fs.readdir(base, { withFileTypes: true }))
          .filter((d) => d.isDirectory())
          .map((d) => d.name)
      } catch { /* 没有目录就是空列表 */ }
      const out = []
      for (const name of names) {
        const dir = path.join(base, name)
        let hasContract = false
        try { await fs.access(path.join(dir, 'contract.json')); hasContract = true } catch { /* noop */ }
        if (!hasContract) continue
        let meta = null
        try { meta = JSON.parse(await fs.readFile(path.join(dir, 'project.json'), 'utf8')) } catch { /* 可选 */ }
        out.push({ name, policies: (meta?.policies || []).map((p) => p.key), base: `/ducksir-static/build/release/${name}/` })
      }
      return json(200, { bundles: out })
    }
    if (pathname === '/diag') {
      return json(200, {
        uptime_s: Math.round((Date.now() - STARTED_AT) / 1000),
        pid: process.pid,
        python,
        repo,
        pythonpath: pythonPath(),
        requests: REQUESTS,
        running: Boolean(session),
        port: session?.port ?? null,
        last_error: lastError,
        node: process.version,
      })
    }
    if (pathname === '/start' && req.method === 'POST') {
      let body = ''
      for await (const chunk of req) body += chunk
      let opts = {}
      try { opts = body ? JSON.parse(body) : {} } catch { return json(400, { error: 'body 不是 JSON' }) }
      const out = await start(opts)
      return json(out.ok ? 200 : 500, out)
    }
    if (pathname === '/stop' && req.method === 'POST') return json(200, await stop())
    if (PROXY_PATHS.includes(pathname)) return proxy(req, res, rest)
    return json(404, { error: '没这个端点', path: pathname })
  }

  return { handle, start, stop, status }
}

export { PROXY_PATHS }
