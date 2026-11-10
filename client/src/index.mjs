/**
 * ducksir-client —— ducksir 的 DSH 插件，对应三平面里的 client。
 *
 * 硬边界（README 第 0 节）：客户端只做控制面——看状态、查策略、跑预检、发起训练。
 * 不进实时控制回路，不参与端侧 50 Hz 的推理。
 *
 * 现在只有一个 ducksir_status：把仓库里五个平面的落地情况读出来。
 * 它是「Day-1 预检台」的第一块砖，同时是这个插件能否被加载、能否跑通的冒烟测试。
 * 真正的预检要等 contract/ 有内容之后再挂上去（见 README「下一步」）。
 */

import { execFile } from 'node:child_process'
import { createReadStream } from 'node:fs'
import { readdir, stat as statFile } from 'node:fs/promises'
import path from 'node:path'
import { fileURLToPath } from 'node:url'

/** Loader 身份。 */
export const name = 'ducksir-client'
/** 没有 tools 就没有可注册的东西，直接不启用。 */
export const inject = ['tools']

/** 本文件在 <repo>/client/src/index.mjs，往上两级就是仓库根。 */
const HERE = path.dirname(fileURLToPath(import.meta.url))
/** 允许用 DUCKSIR_ROOT 指到别的 checkout（同一份插件接多个仓库时用）。 */
const REPO = path.resolve(process.env.DUCKSIR_ROOT ?? path.join(HERE, '..', '..'))

/** 五个平面：四份实现 + 一份契约。判据只看「有没有东西」，不看内容。 */
const PLANES = [
  { key: 'contract', label: '契约' },
  { key: 'hardware', label: '硬件' },
  { key: 'training', label: '训练' },
  { key: 'runtime', label: '端侧' },
  { key: 'client', label: '客户端' },
]

/** 计数时跳过的目录：依赖、产物、缓存，都不算「这个平面有东西」。 */
const SKIP = new Set([
  'node_modules', '.git', 'dist', 'build', '__pycache__',
  '.venv', 'venv', '.pnpm-store', '.pytest_cache', 'runs', 'outputs', 'wandb',
])

const MAX_ENTRIES = 5000
const MAX_DEPTH = 4

/**
 * 跑一条命令，失败也当结果，不抛。
 * @param {string} cmd - 可执行文件。
 * @param {string[]} args - 参数。
 * @returns {Promise<{ok: boolean, out: string}>} ok=false 表示命令不存在或退出码非 0。
 */
function run(cmd, args) {
  return new Promise((resolve) => {
    execFile(cmd, args, { cwd: REPO, timeout: 5000, maxBuffer: 4 * 1024 * 1024 }, (error, stdout) => {
      resolve(error ? { ok: false, out: '' } : { ok: true, out: stdout.trim() })
    })
  })
}

/**
 * 数一个平面目录里的真实文件数（有上限，防大仓库把工具拖死）。
 * @param {string} dir - 绝对路径。
 * @returns {Promise<number>} 文件数；目录不存在返回 0。
 */
async function countFiles(dir) {
  let total = 0
  let seen = 0
  const walk = async (current, depth) => {
    if (depth > MAX_DEPTH || seen > MAX_ENTRIES) return
    let entries
    try {
      entries = await readdir(current, { withFileTypes: true })
    } catch {
      return
    }
    for (const entry of entries) {
      seen += 1
      if (seen > MAX_ENTRIES) return
      if (entry.name.startsWith('.') || SKIP.has(entry.name)) continue
      if (entry.isDirectory()) await walk(path.join(current, entry.name), depth + 1)
      else if (entry.isFile()) total += 1
    }
  }
  await walk(dir, 0)
  return total
}

/** 读 git 三件事：当前分支、最后一条提交、脏文件数。 */
async function readGit() {
  const branch = await run('git', ['rev-parse', '--abbrev-ref', 'HEAD'])
  const head = await run('git', ['log', '-1', '--format=%h %s'])
  const status = await run('git', ['status', '--porcelain'])
  const dirty = status.ok ? status.out.split('\n').filter(Boolean).length : 0
  return {
    branch: branch.ok ? branch.out : '(不在 git 仓库里)',
    head: head.ok ? head.out : '',
    dirty,
  }
}

/**
 * 汇总仓库状态。纯读，不改任何东西。
 * @returns {Promise<object>} 工具输出对象。
 */
async function readStatus() {
  const git = await readGit()
  const planes = {}
  for (const plane of PLANES) {
    planes[plane.key] = { label: plane.label, files: await countFiles(path.join(REPO, plane.key)) }
  }
  // client 自己一定在，不算进「缺什么」。
  const missing = PLANES.filter((p) => p.key !== 'client' && planes[p.key].files === 0).map((p) => p.key)
  return { repo: REPO, git, planes, missing }
}

/**
 * 把状态渲染成人能扫一眼的几行。
 * @param {object} value - readStatus 的结果。
 * @returns {Array<{type: 'text', text: string}>} content part 数组。
 */
function renderStatus(value) {
  const lines = [
    `ducksir @ ${value.git.branch}${value.git.head ? ` · ${value.git.head}` : ''}${value.git.dirty ? ` · 脏 ${value.git.dirty}` : ''}`,
    `repo: ${value.repo}`,
    '',
  ]
  for (const plane of PLANES) {
    const { files, label } = value.planes[plane.key]
    lines.push(`${label.padEnd(3, '　')}  ${files === 0 ? '✗ 空' : `✓ ${files} 文件`}`)
  }
  lines.push('')
  lines.push(value.missing.length === 0 ? '✓ 四个平面都有东西了' : `缺 ${value.missing.length} 个平面：${value.missing.join(', ')}`)
  return [{ type: 'text', text: lines.join('\n') }]
}

/** ducksir_status：读仓库五个平面的落地情况。 */
function statusTool() {
  return {
    name: 'ducksir_status',
    description:
      '读 ducksir 仓库（contract/hardware/training/runtime/client 五个平面）的落地情况、git 分支与脏文件数。只读，不改任何东西。想知道「现在这个仓库到什么程度了」时用它。',
    parameters: {
      type: 'object',
      additionalProperties: false,
      properties: {},
    },
    output: {
      schema: {
        type: 'object',
        additionalProperties: false,
        properties: {
          repo: { type: 'string' },
          git: {
            type: 'object',
            additionalProperties: false,
            properties: {
              branch: { type: 'string' },
              head: { type: 'string' },
              dirty: { type: 'integer' },
            },
          },
          planes: { type: 'object', additionalProperties: true },
          missing: { type: 'array', items: { type: 'string' } },
        },
      },
      render: (_args, value) => renderStatus(value),
    },
    timeoutMs: 15000,
    isConcurrencySafe: () => true,
    async execute() {
      return readStatus()
    },
  }
}

/** 静态路由前缀：查看器从这里取 html / js / glb。 */
const STATIC_ROUTE = '/ducksir-static'
/** 只放这两棵子树，别的路径一律 403 —— 这条路由等于一个只读的小静态服务器。 */
const STATIC_ALLOWED = ['client/viewer', 'assets']
const MIME = {
  '.html': 'text/html; charset=utf-8',
  '.js': 'text/javascript; charset=utf-8',
  '.mjs': 'text/javascript; charset=utf-8',
  '.css': 'text/css; charset=utf-8',
  '.json': 'application/json; charset=utf-8',
  '.glb': 'model/gltf-binary',
  '.gltf': 'model/gltf+json',
  '.png': 'image/png',
  '.svg': 'image/svg+xml',
  '.wasm': 'application/wasm',
  '.map': 'application/json',
}

/**
 * 服务查看器的静态文件：`/ducksir-static/client/viewer/index.html`、
 * `/ducksir-static/assets/g1/*.glb`。只读、只 GET/HEAD、只许两棵子树。
 * @param {import('node:http').IncomingMessage} req - 请求。
 * @param {import('node:http').ServerResponse} res - 响应。
 */
async function serveStatic(req, res) {
  if (req.method !== 'GET' && req.method !== 'HEAD') {
    res.writeHead(405, { allow: 'GET, HEAD' })
    res.end()
    return
  }
  let rel = ''
  try {
    rel = decodeURIComponent(new URL(req.url ?? '/', 'http://localhost').pathname.slice(STATIC_ROUTE.length))
  } catch {
    res.writeHead(400)
    res.end()
    return
  }
  const target = path.resolve(REPO, rel.replace(/^\/+/, ''))
  const inside = STATIC_ALLOWED.some((dir) => {
    const root = path.join(REPO, dir)
    return target === root || target.startsWith(root + path.sep)
  })
  if (!inside) {
    res.writeHead(403)
    res.end()
    return
  }
  let info
  try {
    info = await statFile(target)
  } catch {
    res.writeHead(404)
    res.end()
    return
  }
  if (!info.isFile()) {
    res.writeHead(404)
    res.end()
    return
  }
  res.writeHead(200, {
    'content-type': MIME[path.extname(target).toLowerCase()] ?? 'application/octet-stream',
    'content-length': info.size,
    'cache-control': 'no-cache',
    'x-content-type-options': 'nosniff',
  })
  if (req.method === 'HEAD') {
    res.end()
    return
  }
  createReadStream(target).on('error', () => res.destroy()).pipe(res)
}

/**
 * 插件入口。
 * @param {object} ctx - cordis 上下文。
 */
export function apply(ctx) {
  ctx.tools.register(statusTool())
  // webServer 只在 web profile 里有：用 ctx.inject 起一个子 fiber，
  // 服务不在时子 fiber 一直等（等于这个入口不存在），工具照常可用。
  ctx.inject(['webServer'], (child) => {
    child.effect(
      () => child.webServer.register({ kind: 'prefix', path: STATIC_ROUTE, handler: serveStatic }),
      'ducksir.static'
    )
  })
}

export { readStatus }
