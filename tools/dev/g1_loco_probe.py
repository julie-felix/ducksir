#!/usr/bin/env python3
"""G1 的 loco 策略 sim2sim 探针：按 `ccrpRepo/wbc_fsm` 的参考规格跑一遍，看站不站得住。

    python3 tools/dev/g1_loco_probe.py

结论（2026-11-12）：**跑得通，但站不住**（z 0.75 → 0.12）。已排除 obs 顺序/重力/映射/action_scale/Kp-Kd。
待排除：初始位形的进入方式、力矩限幅、接触参数、cmd 平滑。详见 docs/notes/g1-botlab.md §6。
"""
import numpy as np, yaml, mujoco, onnxruntime as ort

DQ_POLICY = np.array([-0.2,0.0,0.0,0.42,-0.23,0.0,
                      -0.2,0.0,0.0,0.42,-0.23,0.0,
                      0.0,0.0,0.0,
                      0.35,0.18,0.0,0.87,0.0,0.0,0.0,
                      0.35,-0.18,0.0,0.87,0.0,0.0,0.0], np.float64)
MAPPING = [0,6,12, 1,7,13, 2,8,14, 3,9,15,22, 4,10,16,23, 5,11,17,24, 18,25, 19,26, 20,27, 21,28]
KP = np.array([200,150,150,200,20,20, 200,150,150,200,20,20, 200,200,200,
               100,100,50,50,40,40,40, 100,100,50,50,40,40,40], np.float64)
KD = np.array([5,5,5,5,2,2, 5,5,5,5,2,2, 5,5,5,
               2,2,2,2,2,2,2, 2,2,2,2,2,2,2], np.float64)
ACTION_SCALE = 0.25

pre = yaml.safe_load(open('/tmp/bt-all/G1-WBC-Multi-Policy/g1-wbc.preset.yaml'))
sess = ort.InferenceSession('/tmp/bt-all/G1-WBC-Multi-Policy/assets/models/loco_0731.onnx',
                            providers=['CPUExecutionProvider'])
ni = [i.name for i in sess.get_inputs()]; no = [o.name for o in sess.get_outputs()]

def quat2mat(q):
    w,x,y,z=q
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
                     [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
                     [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])

spec = mujoco.MjSpec.from_file('build/mjcf/g1-29dof-botlab.xml')
g = spec.worldbody.add_geom(); g.type=mujoco.mjtGeom.mjGEOM_PLANE; g.size=[6,6,.1]; g.condim=3
m = spec.compile(); d = mujoco.MjData(m); m.opt.timestep = 1/200
# 用 loco.json 那套 gains（不是 MJCF 里的 position 增益）
for i in range(m.nu):
    m.actuator_gainprm[i][0] = KP[i]
    m.actuator_biasprm[i][1] = -KP[i]
    m.actuator_biasprm[i][2] = -KD[i]
# 初始位形用 policy 序的默认角映射到 motor 序
q0 = np.zeros(29)
for i in range(29): q0[MAPPING[i]] = DQ_POLICY[i]
d.qpos[7:36] = q0
d.qpos[0:3] = [0,0,0.75]; d.qpos[3:7] = [1,0,0,0]
mujoco.mj_forward(m, d)

h = np.zeros((1,1,256), np.float32); c = np.zeros((1,1,256), np.float32)
last = np.zeros(29, np.float32)
zs=[]; ups=[]
for k in range(300):
    R = quat2mat(d.qpos[3:7])
    grav = R.T @ np.array([0,0,-1.0])
    q_pol = np.array([d.qpos[7+MAPPING[i]] for i in range(29)]) - DQ_POLICY
    qd_pol = np.array([d.qvel[6+MAPPING[i]] for i in range(29)])
    cmd = np.zeros(3)
    obs = np.concatenate([d.qvel[3:6], grav, cmd, q_pol, qd_pol, last]).astype(np.float32)
    obs = np.clip(obs, -100, 100)
    out = sess.run(no, {ni[0]: obs.reshape(1,-1), ni[1]: h, ni[2]: c})
    act = np.clip(np.asarray(out[0]).reshape(-1), -100, 100).astype(np.float32)
    h = np.asarray(out[1]); c = np.asarray(out[2]); last = act
    ctrl = np.zeros(29)
    for i in range(29):
        ctrl[MAPPING[i]] = act[i] * ACTION_SCALE + DQ_POLICY[i]
    d.ctrl[:] = ctrl
    for _ in range(4): mujoco.mj_step(m, d)
    zs.append(float(d.qpos[2])); ups.append(float(1-2*(d.qpos[4]**2+d.qpos[5]**2)))
print('z 起 0.75 →', round(zs[-1],3), '最低', round(min(zs),3), '最高', round(max(zs),3))
print('直立', round(ups[-1],3), '| 存活', len(zs), '步')
