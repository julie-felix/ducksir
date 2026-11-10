// 静态检查：play.js 里 $('xxx') 引用的每个 id 都必须在 play.html 里存在。
// 页面改版最容易漏的就是这个（按钮换 id、遮罩换 class），而浏览器只会在运行时静默失败。
//   node sim/test/test_play_page_ids.mjs
import { readFileSync } from 'node:fs';
import { resolve } from 'node:path';

const ROOT = resolve(import.meta.dirname, '../..');
const html = readFileSync(resolve(ROOT, 'client/viewer/play.html'), 'utf8');
const js = readFileSync(resolve(ROOT, 'client/viewer/play.js'), 'utf8');

const ids = new Set(html.match(/\bid="([^"]+)"/g).map((m) => m.slice(4, -1)));
const core = readFileSync(resolve(ROOT, 'client/viewer/play-core.mjs'), 'utf8');
const exported = new Set([...core.matchAll(/export\s+(?:async\s+)?(?:function|const|let|class)\s+([A-Za-z_$][\w$]*)/g)]
  .map((m) => m[1]));
const imported = (js.match(/import\s*\{([^}]+)\}\s*from\s*'\.\/play-core\.mjs'/) || [, ''])[1]
  .split(',').map((x) => x.trim().split(/\s+as\s+/)[0]).filter(Boolean);
const badImports = imported.filter((n) => !exported.has(n));
const refs = new Set([...js.matchAll(/\$\('([^']+)'\)/g)].map((m) => m[1]));
const missing = [...refs].filter((r) => !ids.has(r));

console.log(`页面 id ${ids.size} 个（${[...ids].join(', ')}）`);
console.log(`脚本引用 ${refs.size} 个`);
if (badImports.length) {
  console.log('play.js 从 play-core 导入了不存在的名字：' + badImports.join(', '));
  console.log('（play-core 实际导出：' + [...exported].join(', ') + '）');
  process.exit(1);
}
console.log(`导入检查：${imported.length} 个名字全部在 play-core 里有导出`);

// planFile('xxx') 的字面参数必须在契约里存在（这一处曾经写成 planFile(pol.file)，运行时才炸）
const contract = JSON.parse(readFileSync(resolve(ROOT, 'client/viewer/data/live/contract.json'), 'utf8'));
const planNames = new Set(Object.keys(contract.plans));
// play.js 现在通过 assets-map 的 planPath(contract, 'x') 取 plan
const called = new Set([...js.matchAll(/planPath\(contract, '([^']+)'\)/g)].map((m) => m[1]));
const badPlans = [...called].filter((n) => !planNames.has(n));
if (badPlans.length) {
  console.log('play.js 请求了契约里不存在的 plan：' + badPlans.join(', '));
  process.exit(1);
}
console.log(`planPath 检查：${[...called].join(', ') || '（无）'} 都在契约里`);

if (missing.length) {
  console.log('缺失：' + missing.join(', '));
  console.log('FAIL：脚本引用了页面里不存在的 id');
  process.exit(1);
}
console.log('PASS：脚本引用的 id 全部存在');

// 实验台也要对得上（同一类漏改：js 里 $('x') 而 html 没有）
const inspectHtml = readFileSync(resolve(ROOT, 'client/viewer/inspect.html'), 'utf8');
const inspectJs = readFileSync(resolve(ROOT, 'client/viewer/inspect.js'), 'utf8');
const iIds = new Set(inspectHtml.match(/\bid="([^"]+)"/g).map((m) => m.slice(4, -1)));
const iRefs = new Set([...inspectJs.matchAll(/\$\('([^']+)'\)/g)].map((m) => m[1]));
const iMissing = [...iRefs].filter((r) => !iIds.has(r));
if (iMissing.length) {
  console.log('inspect.js 引用了不存在的 id：' + iMissing.join(', '));
  process.exit(1);
}
console.log(`inspect 页：${iRefs.size} 处引用全部命中`);
