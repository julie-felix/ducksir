#!/usr/bin/env python3
"""观测源（source）单测：契约里的每种 source 都要能按语义取到数。

不需要 MuJoCo：用假 `view` 直接验证 `contract_obs.make_obs` 的取值与偏移。
覆盖新增的 body:pos/quat/vel/angvel、contact:、base:z——这些是对齐 botlab
`Robot.body_position/body_quaternion/body_linear_velocity/body_angular_velocity/contact/base_height` 的。
"""
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
sys.path.insert(0, os.path.join(ROOT, "sim", "host"))

import contract_obs as CO  # noqa: E402

fails = []


def ok(name, cond, detail=""):
    print(("  PASS  " if cond else "  FAIL  ") + name + (f"  {detail}" if detail else ""))
    if not cond:
        fails.append(name)


class Data:
    qpos = np.array([0.5, 0.1, 0.123, 1.0, 0.0, 0.0, 0.0])
    qvel = np.zeros(6)


BODIES = {
    "trunk": {"xpos": np.array([1.0, 2.0, 0.117]), "xquat": np.array([1.0, 0.0, 0.0, 0.0]),
              "cvel": np.array([0.1, 0.2, 0.3, 4.0, 5.0, 6.0]), "contacts": 2},
    "foot": {"xpos": np.array([1.0, 2.0, 0.0]), "xquat": np.array([1.0, 0.0, 0.0, 0.0]),
             "cvel": np.zeros(6), "contacts": 0},
}

VIEW = {
    "data": Data(),
    "qpos_off": [0, 1],
    "dof_off": [0, 1],
    "sensor_adr": lambda nm: 0,
    "quat_of": lambda b: BODIES[b]["xquat"],
    "body": lambda b: BODIES[b],
}


def doc_with(entries):
    return {"robot": {"joints": ["a", "b"], "default_q": [0.0, 0.0]}, "obs": entries}


def run(name, entry, expect, length=None):
    doc = doc_with([entry])
    try:
        obs = CO.make_obs(doc, VIEW, last_action=np.zeros(2, dtype=np.float32),
                          command=np.zeros(3, dtype=np.float32))
    except SystemExit as e:  # 配方不认识会直接退出，这里兜住
        ok(name, False, str(e))
        return
    ok(name, np.allclose(obs[:len(expect)], expect, atol=1e-6),
       f"{np.round(obs[:len(expect)], 3)} vs {np.round(expect, 3)}")


run("body:pos", {"name": "p", "slot": [0, 3], "source": "body:pos:trunk"}, [1.0, 2.0, 0.117])
run("body:quat", {"name": "q", "slot": [0, 4], "source": "body:quat:trunk"}, [1.0, 0.0, 0.0, 0.0])
run("body:vel（cvel 后 3 是线速度）", {"name": "v", "slot": [0, 3], "source": "body:vel:trunk"},
    [4.0, 5.0, 6.0])
run("body:angvel（cvel 前 3 是角速度）", {"name": "w", "slot": [0, 3], "source": "body:angvel:trunk"},
    [0.1, 0.2, 0.3])
run("contact:有接触 → 1", {"name": "c", "slot": [0, 1], "source": "contact:trunk"}, [1.0])
run("contact:无接触 → 0", {"name": "c0", "slot": [0, 1], "source": "contact:foot"}, [0.0])
run("base:z", {"name": "z", "slot": [0, 1], "source": "base:z"}, [0.123])

print()
if fails:
    print(f"失败 {len(fails)} 项：" + "、".join(fails))
    raise SystemExit(1)
print("PASS：观测源语义正确（body 位姿/速度、接触、基座高度）")
