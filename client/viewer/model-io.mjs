// 多个张量的模型（LSTM/GRU）在浏览器里怎么喂：与宿主 ORT 后端同一套约定。
//
// 核（wasm 里的 C 核）只认「一条输入、一条输出」：
//     in  = [obs, h, c]
//     out = [action, h_new, c_new]
// 所以这里的职责就一件事：按模型自己声明的张量把这条打包数组切开 / 合回去。
//
// 纯函数、不依赖 ort / wasm，便于单测（sim/test/test_model_io.mjs）。

/** 从 ORT Web 的 metadata 取每个张量的元素数；动态维度按 1 处理（我们的模型都是静态的）。 */
export function tensorLens(metadata, names) {
  const out = [];
  for (let i = 0; i < names.length; i += 1) {
    const meta = metadata && metadata[i] ? metadata[i] : null;
    let dims = meta && meta.dims ? meta.dims : null;
    if (!dims) throw new Error(`模型第 ${i} 个张量没有 dims 元数据，无法切分`);
    let len = 1;
    const shape = [];
    for (const d of dims) {
      const v = Number(d) > 0 ? Number(d) : 1;
      len *= v;
      shape.push(v);
    }
    out.push({ name: names[i], shape, len });
  }
  return out;
}

/**
 * 把打包输入切成每个张量一份。
 * `lens` 来自 `tensorLens(sess.inputMetadata, sess.inputNames)`；总长必须等于打包长度。
 */
export function splitPacked(packed, lens) {
  const total = lens.reduce((s, t) => s + t.len, 0);
  if (total !== packed.length) {
    throw new Error(`模型输入合计 ${total} ≠ 打包输入 ${packed.length}（契约与模型不一致）`);
  }
  const out = [];
  let off = 0;
  for (const t of lens) {
    out.push({ ...t, data: packed.subarray(off, off + t.len) });
    off += t.len;
  }
  return out;
}

/** 把多个输出按声明顺序拼回一条（核期望的顺序就是模型的输出顺序）。 */
export function concatOutputs(results, lens) {
  const total = lens.reduce((s, t) => s + t.len, 0);
  const buf = new Float32Array(total);
  let off = 0;
  for (const t of lens) {
    const value = results[t.name];
    if (!value || !value.data) throw new Error(`模型没有输出 ${t.name}`);
    buf.set(value.data, off);
    off += t.len;
  }
  return buf;
}

/** 单张量模型的快捷判断：不需要切分，直接一条进一条出。 */
export function isSingleTensor(lens) {
  return lens.length === 1;
}
