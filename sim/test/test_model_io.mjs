// model-io.mjs 单测：多张量模型的切分/合并（浏览器侧与宿主侧同一套约定）。
//
//   in  = [obs, h, c]
//   out = [action, h_new, c_new]
import { tensorLens, splitPacked, concatOutputs, isSingleTensor } from '../../client/viewer/model-io.mjs';

let fails = 0;
const ok = (name, cond, detail = '') => {
  console.log(`  ${cond ? 'PASS' : 'FAIL'}  ${name}${detail ? `  ${detail}` : ''}`);
  if (!cond) fails += 1;
};

// 真实的 G1-WBC loco_0731 形状：obs[1,96] + h_in[1,1,256] + c_in[1,1,256]
const inMeta = [{ dims: [1, 96] }, { dims: [1, 1, 256] }, { dims: [1, 1, 256] }];
const outMeta = [{ dims: [1, 29] }, { dims: [1, 1, 256] }, { dims: [1, 1, 256] }];
const inLens = tensorLens(inMeta, ['obs', 'h_in', 'c_in']);
const outLens = tensorLens(outMeta, ['actions', 'h_out', 'c_out']);

ok('输入形状解析', inLens.map((t) => t.len).join(',') === '96,256,256',
  inLens.map((t) => `${t.name}${t.shape.join('x')}`).join(' '));
ok('输出形状解析', outLens.map((t) => t.len).join(',') === '29,256,256');

const packed = new Float32Array(96 + 256 + 256);
for (let i = 0; i < packed.length; i += 1) packed[i] = i;
const parts = splitPacked(packed, inLens);
ok('切分：obs 段是前 96 个', parts[0].data.length === 96 && parts[0].data[95] === 95);
ok('切分：h 段从 96 开始', parts[1].data.length === 256 && parts[1].data[0] === 96);
ok('切分：c 段从 352 开始', parts[2].data[0] === 352);
ok('切分：拼回去与原来一致',
  [...parts[0].data, ...parts[1].data, ...parts[2].data].every((v, i) => v === packed[i]));

// 长度不符要报错（契约与模型不一致时宁可炸掉）
let threw = false;
try { splitPacked(new Float32Array(10), inLens); } catch { threw = true; }
ok('打包长度不符 → 抛错', threw);

const results = {
  actions: { data: new Float32Array(29).fill(1) },
  h_out: { data: new Float32Array(256).fill(2) },
  c_out: { data: new Float32Array(256).fill(3) },
};
const merged = concatOutputs(results, outLens);
ok('合并：总长 541', merged.length === 541);
ok('合并：顺序是 action → h → c',
  merged[0] === 1 && merged[29] === 2 && merged[29 + 256] === 3);

ok('单张量判定', isSingleTensor([{ len: 61 }]) && !isSingleTensor(inLens));

// 动态维度按 1 处理（我们不支持真动态，但要给出确定行为）
const dyn = tensorLens([{ dims: ['batch', 61] }], ['obs']);
ok('动态维度当 1 处理', dyn[0].len === 61 && dyn[0].shape[0] === 1, JSON.stringify(dyn[0].shape));

if (fails) { console.log(`\n失败 ${fails} 项`); process.exit(1); }
console.log('\nPASS：model-io（多张量切分 / 合并 / 校验）');
