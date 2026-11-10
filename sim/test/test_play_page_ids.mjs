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

if (missing.length) {
  console.log('缺失：' + missing.join(', '));
  console.log('FAIL：脚本引用了页面里不存在的 id');
  process.exit(1);
}
console.log('PASS：脚本引用的 id 全部存在');
