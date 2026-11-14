import json, os
import numpy as np
import onnxruntime as ort
import mujoco
OUT = os.environ.get('DUCK_CAP', '/tmp/hiking.jsonl')
_orig_run = ort.InferenceSession.run
_orig_step = mujoco.mj_step
st = {'n': 0, 'fh': open(OUT, 'w'), 'qpos': None, 'qvel': None}
def step(m, d):
    _orig_step(m, d)
    st['qpos'] = [round(float(x), 6) for x in d.qpos]
    st['qvel'] = [round(float(x), 6) for x in d.qvel]
mujoco.mj_step = step
def run(self, names, feeds, opts=None):
    res = _orig_run(self, names, feeds, opts)
    rec = {'step': st['n'], 'qpos': st['qpos'], 'qvel': st['qvel'],
           'in': {}, 'out': {}}
    for k, v in feeds.items():
        arr = np.asarray(v).reshape(-1)
        rec['in'][k] = {'shape': list(np.asarray(v).shape), 'vals': [round(float(x), 6) for x in arr[:12000]]}
    for k, v in zip(names or [o.name for o in self.get_outputs()], res):
        rec['out'][k] = [round(float(x), 6) for x in np.asarray(v).reshape(-1)[:64]]
    st['fh'].write(json.dumps(rec) + '\n'); st['fh'].flush(); st['n'] += 1
    return res
ort.InferenceSession.run = run
print('[patch_rec2] ready', flush=True)
