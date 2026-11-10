// 「契约里的名字 → 素材路径」的唯一入口。
//
// 为什么单独一个模块：2026-11-10 连续两次都是这里出的错——
//  1) `planFile(pol.file)`：把策略文件名当成 plan 名去查契约 → undefined.file；
//  2) `N_JOINTS` 未定义：重构后消费端还引用旧导出。
// 这类错误只有真正跑到那行才炸，而页面没法在 node 里跑。所以把「查路径」抽成纯函数，
// 由 `node sim/test/test_assets_map.mjs` 覆盖。

const DIR = './data/live/';

// `dir` 可选：跑 bundle 时传 bundle 目录（例如 /ducksir-static/build/release/demo/）

/** plan 名 → 路径。契约里没有就直接报清楚有哪些。 */
export function planPath(contract, name, dir = DIR) {
  const p = contract && contract.plans && contract.plans[name];
  if (!p) {
    const have = Object.keys((contract && contract.plans) || {}).join(', ') || '（空）';
    throw new Error(`契约里没有 plan "${name}"（有：${have}）`);
  }
  return dir + p.file;
}

/** 策略 key 或文件名 → 路径。`{key, mode, file}` 的条目来自 contract.policies。 */
export function policyPath(contract, keyOrFile, dir = DIR) {
  const list = (contract && contract.policies) || [];
  const pol = list.find((p) => p.key === keyOrFile) || list.find((p) => p.file === keyOrFile);
  if (!pol) throw new Error(`契约里没有策略 "${keyOrFile}"`);
  return dir + pol.file;
}

/** obs 配方 → 需要的源（给加载前的自检用）。 */
export function obsSources(contract) {
  return (contract && contract.obs ? contract.obs : []).map((o) => o.source);
}
