// 覆盖「契约名字 → 素材路径」这一层（页面里最容易错、而且只有跑到才炸的地方）。
//   node sim/test/test_assets_map.mjs
import { readFileSync, existsSync } from 'node:fs';
import { resolve } from 'node:path';
import { planPath, policyPath } from '../../client/viewer/assets-map.mjs';

const ROOT = resolve(import.meta.dirname, '../..');
const LIVE = resolve(ROOT, 'client/viewer/data/live');
const contract = JSON.parse(readFileSync(resolve(LIVE, 'contract.json'), 'utf8'));

let bad = 0;
const ok = (label, fn, expectThrow = false) => {
  let err = null;
  try { fn(); } catch (e) { err = e; }
  const pass = expectThrow ? Boolean(err) : !err;
  console.log(`  ${pass ? 'PASS' : 'FAIL'}  ${label}${err ? ' → ' + err.message.slice(0, 60) : ''}`);
  if (!pass) bad += 1;
};

console.log('磁盘上真存在：');
for (const name of Object.keys(contract.plans)) {
  ok(`plan ${name} 文件存在`, () => {
    const p = planPath(contract, name);
    if (!existsSync(resolve(ROOT, 'client/viewer', p))) throw new Error('缺文件 ' + p);
  });
}
for (const pol of contract.policies) {
  ok(`策略 ${pol.key} 文件存在`, () => {
    const p = policyPath(contract, pol.key);
    if (!existsSync(resolve(ROOT, 'client/viewer', p))) throw new Error('缺文件 ' + p);
  });
}

console.log('错误输入要报清楚：');
ok('未知 plan', () => planPath(contract, 'nope'), true);
ok('把策略文件名当 plan 名', () => planPath(contract, contract.policies[0].file), true);
ok('未知策略', () => policyPath(contract, 'nope'), true);

console.log(bad === 0 ? '\nPASS：契约名字到路径的映射全对' : `\n${bad} 条失败`);
process.exit(bad === 0 ? 0 : 1);
