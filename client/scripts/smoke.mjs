/**
 * 冒烟测试：不装进 profile 也能验。
 *
 * 假 ctx → 注册 → 执行 → 渲染，四步都断言一遍。
 * 跑法：node client/scripts/smoke.mjs（或 npm run smoke）
 */

import assert from 'node:assert/strict'

import { apply, inject, name } from '../src/index.mjs'

const tools = []
apply({ tools: { register: (tool) => tools.push(tool) } })

assert.equal(name, 'ducksir-client', '插件名变了？')
assert.deepEqual(inject, ['tools'], 'inject 声明变了？')
assert.equal(tools.length, 1, 'apply 应该只注册一个工具')

const [tool] = tools
assert.equal(tool.name, 'ducksir_status')
assert.equal(typeof tool.execute, 'function')
assert.ok(tool.output?.schema, '缺 output.schema')
assert.equal(typeof tool.output.render, 'function')

const value = await tool.execute({})
assert.ok(typeof value.repo === 'string' && value.repo.length > 0, 'repo 没解析出来')
assert.ok(value.planes?.client?.files > 0, 'client 平面至少该有文件（就是插件自己）')
assert.ok(Array.isArray(value.missing), 'missing 要是数组')
assert.equal(typeof value.git.branch, 'string')

const parts = tool.output.render({}, value)
assert.ok(Array.isArray(parts) && parts[0]?.type === 'text', 'render 要返回 content part 数组')

console.log(parts[0].text)
console.log('\n✓ 注册 / 执行 / 渲染 全通过')
