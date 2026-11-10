// 契约守卫：加载时维度/digest 对不上必须**报错**，而不是用旧偏移去读新 blob。
//
// 这条测试存在的理由（2026-11-10）：契约以前写在消费端代码里，改 Python 生成新 IR 后
// 浏览器用旧偏移读新 blob，不报错、只是行为诡异（指令映射漂移就是这么来的）。
//   node sim/test/test_contract_guard.mjs
import { readFileSync } from 'node:fs';
import { resolve } from 'node:path';
import { setContract, getContract, checkPlanAgainstContract, slot, obsDim, makeObs }
  from '../../client/viewer/play-core.mjs';

const ROOT = resolve(import.meta.dirname, '../..');
const raw = readFileSync(resolve(ROOT, 'client/viewer/data/live/contract.json'), 'utf8');
const good = JSON.parse(raw);

let bad = 0;
const check = (label, fn, expectThrow) => {
  let threw = null;
  try { fn(); } catch (e) { threw = e; }
  const ok = expectThrow ? Boolean(threw) : !threw;
  console.log(`  ${ok ? 'PASS' : 'FAIL'}  ${label}${threw ? ' → ' + threw.message.slice(0, 70) : ''}`);
  if (!ok) bad += 1;
};

console.log('正常契约：');
setContract(good);
check('加载', () => getContract(), false);
check('pre 维度 61→61 校验通过', () => checkPlanAgainstContract('pre', { input_dim: 61, output_dim: 61 }), false);
check('obsDim = 61', () => { if (obsDim() !== 61) throw new Error('obsDim=' + obsDim()); }, false);
check('取 fsm.out:state 槽位', () => { const s = slot('fsm', 'out:state'); if (s[0] !== 0) throw new Error('偏移 ' + s[0]); }, false);

console.log('篡改后：');
const tweak = (mutate) => { const c = structuredClone(good); mutate(c); return c; };
setContract(good);
check('维度不符（契约说 61，blob 说 64）',
  () => checkPlanAgainstContract('pre', { input_dim: 64, output_dim: 61 }), true);
check('槽位越界（obs 总长 999）',
  () => setContract(tweak((c) => { c.obs[0].slot = [0, 999]; })), true);
check('缺策略字段',
  () => setContract(tweak((c) => { delete c.policies[0].file; })), true);
check('kind 不对',
  () => setContract(tweak((c) => { c.kind = 'something-else'; })), true);
check('未知 plan',
  () => checkPlanAgainstContract('nope', { input_dim: 1, output_dim: 1 }), true);
check('未知槽位',
  () => slot('fsm', 'out:nope'), true);

console.log(bad === 0 ? '\nPASS：契约不一致都会在加载/校验时报错' : `\n${bad} 条失败`);
process.exit(bad === 0 ? 0 : 1);
