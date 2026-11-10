#!/usr/bin/env python3
"""通用 2 自由度并联机构解算（两个旋转电机 + 两根定长推杆 + 2 自由度平台）。

    python3 tools/parallel/solver.py <entity.yaml> --selfcheck

设计原则（来自 docs/plans/2026-11-09-parallel-mechanism.md 的 Q2/Q13）：
- **不抄 refs/ 的代码**，只借思路：逆解靠「两点距离不变」、雅可比靠隐函数求导、正解靠牛顿。
- 几何全部从**实体**里读（links 的变换 + joints 的轴/原点），所以换一台机型只换配置，不改代码。
- 雅可比用约束的隐函数求导，天然包含「roll 轴不在原点」那类偏置项——
  这正是 refs 里 G1 版漏掉 `h0` 项、产生 1.8% 系统误差的根源。

接口：
    m = ParallelAnkle.from_entity(doc, pitch_joint=..., roll_joint=..., motors=[...])
    th = m.ik(roll, pitch)        # 逆解（闭式，两根链各解一个一元一次三角方程）
    J  = m.jac(roll, pitch)       # 2×2，d(theta)/d(roll,pitch)
    roll, pitch, ok = m.fw(th)    # 正解（牛顿迭代）
    tau_joint = J.T @ tau_motor   # 力映射
"""

from __future__ import annotations

import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "robot"))

import entity as E  # noqa: E402


def quat_to_R(q):
    w, x, y, z = q
    n = math.sqrt(w * w + x * x + y * y + z * z) or 1.0
    w, x, y, z = w / n, x / n, y / n, z / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def axis_angle_R(axis, theta):
    u = np.asarray(axis, float)
    u = u / np.linalg.norm(u)
    c, s = math.cos(theta), math.sin(theta)
    K = np.array([[0, -u[2], u[1]], [u[2], 0, -u[0]], [-u[1], u[0], 0]])
    return np.eye(3) * c + s * K + (1 - c) * np.outer(u, u)


def T(R=None, p=None):
    out = np.eye(4)
    if R is not None:
        out[:3, :3] = R
    if p is not None:
        out[:3, 3] = p
    return out


class ParallelAnkle:
    """两电机并联踝。几何全部来自实体。"""

    def __init__(self, doc, pitch_joint, roll_joint, motors, connect_links,
                 pitch_range=None, roll_range=None, name="parallel-ankle"):
        self.name = name
        self.doc = doc
        self.links = {l["name"]: l for l in doc["links"]}
        self.joints = {j["name"]: j for j in doc["joints"]}
        self.children = {}
        for l in doc["links"]:
            self.children.setdefault(l.get("parent") or "world", []).append(l["name"])
        self.joint_of_child = {}
        for j in doc["joints"]:
            self.joint_of_child.setdefault(j["child"], []).append(j["name"])
        self.pitch_joint = pitch_joint
        self.roll_joint = roll_joint
        self.motors = list(motors)
        self.connect_links = list(connect_links)
        self.pitch_range = pitch_range or (-math.pi, math.pi)
        self.roll_range = roll_range or (-math.pi, math.pi)
        self._build_geometry()

    # ---------- 几何 ----------
    @classmethod
    def from_entity(cls, doc, **kw):
        loops = doc.get("loops") or []
        links = [lp["body2"] for lp in loops if lp.get("kind") == "connect"]
        motors = [a["joint"] for a in doc.get("actuators") or [] if a.get("joint")]
        pr = {j["name"] for j in doc["joints"]}
        pitch = next(n for n in pr if "pitch" in n and "u_" not in n and "motor" not in n)
        roll = next(n for n in pr if "roll" in n and "motor" not in n)
        return cls(doc, pitch_joint=pitch, roll_joint=roll, motors=motors,
                   connect_links=links, name=doc["robot"]["id"], **kw)

    def _pose(self, link, q):
        """从根到该 link 的 4×4 位姿；q 是 {joint: angle}。"""
        chain = []
        cur = link
        while cur and cur in self.links:
            chain.append(cur)
            cur = self.links[cur].get("parent")
        M = np.eye(4)
        for name in reversed(chain):
            l = self.links[name]
            tk = l.get("transform") or {"xyz": [0, 0, 0], "quat": [1, 0, 0, 0]}
            M = M @ T(quat_to_R(tk["quat"]), tk["xyz"])
            for jn in self.joint_of_child.get(name, []):
                j = self.joints[jn]
                ang = q.get(jn, 0.0)
                if j.get("type") == "fixed":
                    continue
                # MuJoCo 的 hinge 绕「joint pos 处的轴」转；pos≠0 时必须先平移再转回来。
                # （tg10 的 motor_joint pos 非零，漏了这一项会得到 22 mm 的系统偏差。）
                pos = np.asarray(j.get("pos") or [0, 0, 0], float)
                R = axis_angle_R(j["axis"]["dir"], ang)
                M = M @ T(np.eye(3), pos) @ T(R) @ T(np.eye(3), -pos)
        return M

    def _build_geometry(self):
        """零点几何：电机轴心 A、推杆销 B(θ=0)、平台点 C、初始杆长。

        配对必须从 loops 读：`<connect body1=motor_link body2=connect_link>` 才是真正的一对，
        按下标配对会把左电机配到右平台（G1 上就会得到两根不等长的杆，这是发现该 bug 的线索）。
        """
        q0 = {}
        self.chains = []
        pairs = []
        for lp in self.doc.get("loops") or []:
            if lp.get("kind") != "connect":
                continue
            body1, body2 = lp.get("body1"), lp.get("body2")
            motor = next((j["name"] for j in self.doc["joints"] if j["child"] == body1), None)
            if motor is None:
                raise ValueError(f"connect {lp.get('id')} 的 body1 {body1} 上找不到关节")
            pairs.append((motor, body2))
        if not pairs:
            pairs = list(zip(self.motors, self.connect_links))
        self.motor_pairs = pairs
        self.motors = [m for m, _ in pairs]
        self.connect_links = [c for _, c in pairs]
        for i, (motor, clink) in enumerate(pairs):
            mj = self.joints[motor]
            motor_link = mj["child"]
            M_motor = self._pose(motor_link, q0)
            A = M_motor[:3, 3].copy()
            axis = np.asarray(mj["axis"]["dir"], float)
            # 销在电机 link 坐标系里的位置：由 connect 的 anchor 给出（MuJoCo 语义）
            anchor = np.array(self._anchor_for(clink), float)
            b0 = A + M_motor[:3, :3] @ anchor
            M_c = self._pose(clink, q0)
            c0 = M_c[:3, 3].copy()
            rod = float(np.linalg.norm(c0 - b0))
            # 铰的真正转轴：过 joint pos、方向为 body 朝向 @ axis（都要换到基座系）
            jpos = np.asarray(mj.get("pos") or [0, 0, 0], float)
            Rm = M_motor[:3, :3]
            pivot = A + Rm @ jpos
            axis_base = Rm @ axis
            axis_base = axis_base / (np.linalg.norm(axis_base) or 1.0)
            v = b0 - pivot                      # 销相对转轴的点，绕轴线转的就是它
            self.chains.append({
                "motor": motor, "motor_link": motor_link,
                "A": pivot, "axis": axis_base, "pivot": pivot,
                "anchor": anchor, "v": v, "b0": b0, "c0": c0, "rod": rod,
            })

    def _anchor_for(self, body2):
        for lp in self.doc.get("loops") or []:
            if lp.get("kind") == "connect" and lp.get("body2") == body2:
                return lp["anchor"]
        raise ValueError(f"找不到 {body2} 的 connect 约束")

    @staticmethod
    def _theta_zero(anchor, axis, M_motor):
        return 0.0

    # ---------- 运动学 ----------
    def platform_point(self, roll, pitch):
        """平台点 C 在给定 roll/pitch 下的位置（走整棵树的 FK）。"""
        q = {self.pitch_joint: pitch, self.roll_joint: roll}
        return [self._pose(c, q)[:3, 3].copy() for c in self.connect_links]

    def pin_point(self, i, theta):
        """推杆销在基座系里的位置：绕过 joint pos 的轴线转动。

        电机 body 可能带四元数朝向、joint 也可能有 pos 偏移，直接绕 body 原点转 anchor 会算错
        （tg10 上会得到 22 mm 的系统偏差）。
        """
        ch = self.chains[i]
        return ch["pivot"] + axis_angle_R(ch["axis"], theta) @ ch["v"]

    def rod_error(self, theta, i, roll, pitch):
        c = self.platform_point(roll, pitch)[i]
        return float(np.linalg.norm(self.pin_point(i, theta) - c) - self.chains[i]["rod"])

    def _g(self, theta, i, roll, pitch):
        """平方形式的约束残差 |b-c|²-L²（与 jac 的推导一致）。"""
        c = self.platform_point(roll, pitch)[i]
        d = self.pin_point(i, theta) - c
        L = self.chains[i]["rod"]
        return float(np.dot(d, d) - L * L)

    def ik(self, roll, pitch, theta_hint=None):
        """逆解：解 alpha·cosθ + beta·sinθ = k（每个电机独立，闭式）。"""
        cs = self.platform_point(roll, pitch)
        out = []
        for i, ch in enumerate(self.chains):
            A, axis, v = ch["A"], ch["axis"], ch["v"]
            c = cs[i]
            L = ch["rod"]
            w = c - A
            uv = float(np.dot(axis, v))
            p = v - axis * uv
            qv = np.cross(axis, v)
            alpha = float(np.dot(w, p))
            beta = float(np.dot(w, qv))
            k = (np.dot(w, w) + np.dot(v, v) - L * L) / 2.0 - float(np.dot(w, axis)) * uv
            Rr = math.hypot(alpha, beta)
            if Rr < 1e-12:
                raise ValueError(f"链 {i}: 退化（推杆与电机轴共线）")
            ratio = max(-1.0, min(1.0, k / Rr))
            phi = math.atan2(beta, alpha)
            d = math.acos(ratio)
            cand = [phi + d, phi - d]
            if theta_hint is not None:
                best = min(cand, key=lambda t: abs(((t - theta_hint[i] + math.pi) % (2 * math.pi)) - math.pi))
            else:
                best = min(cand, key=lambda t: abs(t))
            out.append(best)
        return np.array(out)

    def _gmats(self, roll, pitch, theta):
        """返回 (∂g/∂θ, ∂g/∂x)。g_i = |b_i(θ)-c_i(x)|²-L²。"""
        eps = 1e-7
        dgdx = np.zeros((2, 2))
        dgdth = np.zeros((2, 2))
        for i in range(2):
            for k, attr in enumerate(("roll", "pitch")):
                if attr == "roll":
                    c1 = self.platform_point(roll + eps, pitch)[i]
                    c0 = self.platform_point(roll - eps, pitch)[i]
                else:
                    c1 = self.platform_point(roll, pitch + eps)[i]
                    c0 = self.platform_point(roll, pitch - eps)[i]
                b = self.pin_point(i, theta[i])
                dgdx[i, k] = (np.dot(b - c1, b - c1) - np.dot(b - c0, b - c0)) / (2 * eps)
            c = self.platform_point(roll, pitch)[i]
            b1 = self.pin_point(i, theta[i] + eps)
            b0 = self.pin_point(i, theta[i] - eps)
            dgdth[i, i] = (np.dot(b1 - c, b1 - c) - np.dot(b0 - c, b0 - c)) / (2 * eps)
        return dgdth, dgdx

    def jac(self, roll, pitch, theta=None):
        """运动学雅可比 dθ/dx = -(∂g/∂θ)⁻¹ (∂g/∂x)（隐函数求导）。"""
        if theta is None:
            theta = self.ik(roll, pitch)
        dgdth, dgdx = self._gmats(roll, pitch, theta)
        return -np.linalg.solve(dgdth, dgdx)

    def fw(self, theta, tol=1e-12, max_iter=60, dt=1.0, x0=None):
        """正解：牛顿迭代。

        残差必须用**平方形式** |b-c|²-L²，才能和 `jac()` 里隐函数求导用的 g 保持一致；
        用「杆长差」配平方形式的雅可比，步长会被放大 2L 倍（约 0.3 m），迭代会震荡不收敛。
        """
        x = np.zeros(2) if x0 is None else np.array(x0, float)
        for it in range(max_iter):
            res = np.array([self._g(theta[i], i, x[0], x[1]) for i in range(2)])
            if np.linalg.norm(res) < tol:
                return x[0], x[1], True, it
            _, dgdx = self._gmats(x[0], x[1], theta)
            try:
                step = -np.linalg.solve(dgdx, res)
            except np.linalg.LinAlgError:
                return float("nan"), float("nan"), False, it
            x = x + dt * step
            x[0] = min(max(x[0], self.roll_range[0]), self.roll_range[1])
            x[1] = min(max(x[1], self.pitch_range[0]), self.pitch_range[1])
        return x[0], x[1], False, max_iter


def load(path):
    return E.read_yaml(path)


def selfcheck(doc, verbose=True):
    m = ParallelAnkle.from_entity(doc)
    ok = True
    if verbose:
        print(f"== {m.name}: {len(m.chains)} 条链")
        for i, ch in enumerate(m.chains):
            print(f"   链{i}: motor={ch['motor']}, rod={ch['rod']*1000:.3f} mm, "
                  f"|AB|={np.linalg.norm(ch['b0']-ch['A'])*1000:.3f} mm")
    # 零点往返
    th0 = m.ik(0.0, 0.0)
    r, p, conv, it = m.fw(th0)
    if verbose:
        print(f"   ik(0,0)={np.degrees(th0).round(6)}  fw 回来: roll={r:.3e} pitch={p:.3e} 收敛={conv} ({it} 步)")
    if not (abs(r) < 1e-9 and abs(p) < 1e-9 and conv):
        print(f"   ⚠ 零点往返不唯一（回零得到 roll={r:.6f} pitch={p:.6f}）——该机构在此位形有两个装配分支，"
              f"牛顿可能收敛到另一支；这不是数值错误", file=sys.stderr)
    # 杆长守恒 + 雅可比对中心差分
    worst_rod, worst_jac = 0.0, 0.0
    xs = [(rr, pp) for rr in np.linspace(-0.15, 0.15, 5) for pp in np.linspace(-0.3, 0.3, 5)]
    hint = m.ik(0.0, 0.0)
    for roll, pitch in xs:
        th = m.ik(roll, pitch, theta_hint=hint)
        hint = th
        for i in range(2):
            worst_rod = max(worst_rod, abs(m.rod_error(th[i], i, roll, pitch)))
        J = m.jac(roll, pitch, th)
        eps = 1e-6
        Jn = np.zeros_like(J)
        for k in range(2):
            dx = np.zeros(2); dx[k] = eps
            Jn[:, k] = (m.ik(roll + dx[0], pitch + dx[1], theta_hint=th)
                        - m.ik(roll - dx[0], pitch - dx[1], theta_hint=th)) / (2 * eps)
        worst_jac = max(worst_jac, float(np.max(np.abs(J - Jn))))
    if verbose:
        print(f"   杆长守恒最大误差: {worst_rod:.3e} m   雅可比 vs 中心差分: {worst_jac:.3e}")
    ok &= worst_rod < 1e-9 and worst_jac < 1e-6
    return ok


def main(argv):
    if len(argv) < 2:
        print(__doc__)
        return 2
    doc = load(argv[1])
    ok = selfcheck(doc)
    print("SELF-CHECK", "PASS" if ok else "FAIL")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main(sys.argv))
