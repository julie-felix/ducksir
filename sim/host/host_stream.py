#!/usr/bin/env python3
"""把宿主的物理状态实时推给浏览器：一个只用标准库的 WebSocket 服务。

为什么不用「浏览器自己跑物理」：像 Instinct 这种要**深度渲染**、Uni 那种要 1720 维拼装的
策略，浏览器端还没有实现。让宿主算、浏览器只看，是让所有 bundle 都能「动起来」的最短路径。

协议（一条 JSON 头 + 一块二进制网格，之后每帧一条 JSON 位姿）：

1. 连接后宿主先发 `{"type":"scene", "bodies":[...], "geoms":[...], "meshes":{...}, "meshBytes":N}`
   紧跟着一条 **binary** 帧：所有网格的 float32 顶点 + uint32 面片，按 JSON 里的 offset 切。
2. 之后每帧 `{"type":"pose", "t":12.34, "xpos":[...], "xquat":[...]}`（扁平数组，按 body id 排）。

    python3 sim/host/host_stream.py --self-test     # 不需要浏览器，验帧格式
"""
from __future__ import annotations

import argparse
import base64
import hashlib
import json
import socket
import struct
import threading
import time


GUID = "258EAFA5-E914-47DA-95CA-C5AB0DC85B11"


def _recv_exact(sock: socket.socket, n: int) -> bytes:
    buf = b""
    while len(buf) < n:
        chunk = sock.recv(n - len(buf))
        if not chunk:
            raise ConnectionError("连接断开")
        buf += chunk
    return buf


def _encode(payload: bytes, opcode: int = 1) -> bytes:
    head = bytearray([0x80 | opcode])
    n = len(payload)
    if n < 126:
        head.append(n)
    elif n < 1 << 16:
        head.append(126)
        head += struct.pack("!H", n)
    else:
        head.append(127)
        head += struct.pack("!Q", n)
    return bytes(head) + payload


def _read_frame(sock: socket.socket):
    b1, b2 = _recv_exact(sock, 2)
    opcode = b1 & 0x0F
    masked = b2 & 0x80
    n = b2 & 0x7F
    if n == 126:
        n = struct.unpack("!H", _recv_exact(sock, 2))[0]
    elif n == 127:
        n = struct.unpack("!Q", _recv_exact(sock, 8))[0]
    mask = _recv_exact(sock, 4) if masked else b""
    data = _recv_exact(sock, n) if n else b""
    if masked:
        data = bytes(c ^ mask[i % 4] for i, c in enumerate(data))
    return opcode, data


class HostStream:
    """极简广播服务器：只发不收（浏览器发来的消息一律丢弃，除了 ping）。"""

    def __init__(self, port: int = 0, host: str = "127.0.0.1"):
        self.server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server.bind((host, port))
        self.server.listen(8)
        self.port = self.server.getsockname()[1]
        self.clients: list[socket.socket] = []
        # 后连上来的客户端也要能立刻看到画面：把场景和最后一帧位姿存着
        self.scene_text: str | None = None
        self.scene_blob: bytes | None = None
        self.last_pose: str | None = None
        self._lock = threading.Lock()
        # 所有帧必须整帧写出去：握手线程和主循环线程会同时写同一个 socket，
        # 不加锁就会把两帧的字节交错，浏览器只能以 1006 关连接（python 客户端反而看不出）。
        self._send_lock = threading.Lock()
        self._thread = threading.Thread(target=self._accept_loop, daemon=True)
        self._thread.start()

    # ---- 连接 ----
    def _accept_loop(self):
        while True:
            try:
                conn, _ = self.server.accept()
            except OSError:
                return
            threading.Thread(target=self._handshake, args=(conn,), daemon=True).start()

    def _handshake(self, conn: socket.socket):
        try:
            req = b""
            while b"\r\n\r\n" not in req:
                chunk = conn.recv(4096)
                if not chunk:
                    return
                req += chunk
            headers = {}
            for line in req.decode("latin-1").split("\r\n")[1:]:
                if ":" in line:
                    k, v = line.split(":", 1)
                    headers[k.strip().lower()] = v.strip()
            key = headers.get("sec-websocket-key")
            if not key:
                conn.close()
                return
            accept = base64.b64encode(hashlib.sha1((key + GUID).encode()).digest()).decode()
            conn.sendall((
                "HTTP/1.1 101 Switching Protocols\r\n"
                "Upgrade: websocket\r\nConnection: Upgrade\r\n"
                f"Sec-WebSocket-Accept: {accept}\r\n\r\n").encode())
            with self._lock:
                self.clients.append(conn)
            try:
                with self._send_lock:
                    if self.scene_text is not None:
                        conn.sendall(_encode(self.scene_text.encode(), 1))
                        if self.scene_blob:
                            conn.sendall(_encode(self.scene_blob, 2))
                    if self.last_pose is not None:
                        conn.sendall(_encode(self.last_pose.encode(), 1))
            except OSError:
                pass
            while True:
                opcode, _ = _read_frame(conn)
                if opcode == 0x8:
                    break
        except Exception as _e:            # 调试用：把服务端异常打出来
            print(f"[host_stream] 客户端异常：{type(_e).__name__}: {_e}", flush=True)
        finally:
            with self._lock:
                if conn in self.clients:
                    self.clients.remove(conn)
            try:
                conn.close()
            except OSError:
                pass

    # ---- 发送 ----
    def set_scene(self, text: str, blob: bytes):
        self.scene_text = text
        self.scene_blob = blob

    def send(self, text: str, binary: bytes | None = None):
        try:
            import json as _json
            if _json.loads(text).get("type") == "pose":
                self.last_pose = text
        except Exception:
            pass
        payload = _encode(text.encode(), 1)
        blob = _encode(binary, 2) if binary is not None else b""
        dead = []
        with self._lock:
            targets = list(self.clients)
        for conn in targets:
            try:
                with self._send_lock:
                    conn.sendall(payload)
                    if blob:
                        conn.sendall(blob)
            except OSError as _e:
                print(f"[host_stream] 发送失败：{type(_e).__name__}: {_e}", flush=True)
                dead.append(conn)
        for conn in dead:
            with self._lock:
                if conn in self.clients:
                    self.clients.remove(conn)
            try:
                conn.close()
            except OSError:
                pass

    def close(self):
        try:
            self.server.close()
        except OSError:
            pass


def build_scene_b64(model, mujoco) -> str:
    """给 stdio/SSE 用：场景 + 网格一起 base64 成一行 JSON（浏览器不用再下 STL）。"""
    scene, blob = build_scene(model, mujoco)
    scene["meshBase64"] = base64.b64encode(blob).decode("ascii")
    return json.dumps(scene, separators=(",", ":"))


def build_scene(model, mujoco) -> tuple[dict, bytes]:
    """把模型里**视觉 geom** 抽成浏览器能直接建的场景（网格走二进制 blob）。"""
    bodies = []
    for b in range(model.nbody):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, b) or f"body_{b}"
        bodies.append({"id": b, "name": name, "parent": int(model.body_parentid[b])})

    # 兼容 numpy 视图的两种形状：有的 mujoco 版本给 (ngeom, k)，有的给扁平 (ngeom*k)
    import numpy as _np
    size = _np.asarray(model.geom_size).reshape(model.ngeom, -1)
    pos = _np.asarray(model.geom_pos).reshape(model.ngeom, -1)
    quat = _np.asarray(model.geom_quat).reshape(model.ngeom, -1)
    rgba = _np.asarray(model.geom_rgba).reshape(model.ngeom, -1)
    # 同样按扁平处理：mesh_vert/mesh_face 在有些版本是 (n,3) 的二维视图
    mesh_vert = _np.asarray(model.mesh_vert).reshape(-1)
    mesh_face = _np.asarray(model.mesh_face).reshape(-1)
    meshes, chunks = {}, []
    offset = 0
    geoms = []
    for g in range(model.ngeom):
        if model.geom_contype[g] != 0 or model.geom_conaffinity[g] != 0:
            continue                      # 只发视觉 geom（碰撞体会画出「薄壳脚」）
        kind = int(model.geom_type[g])
        entry = {
            "body": int(model.geom_bodyid[g]),
            "kind": kind,
            "size": [round(float(size[g][i]), 6) for i in range(3)],
            "pos": [round(float(pos[g][i]), 6) for i in range(3)],
            "quat": [round(float(quat[g][i]), 6) for i in range(4)],
            "rgba": [round(float(rgba[g][i]), 3) for i in range(4)],
        }
        if kind == int(mujoco.mjtGeom.mjGEOM_MESH):
            mid = int(model.geom_dataid[g])
            if mid < 0:
                continue
            entry["mesh"] = mid
            if mid not in meshes:
                v_adr, v_num = int(model.mesh_vertadr[mid]), int(model.mesh_vertnum[mid])
                f_adr, f_num = int(model.mesh_faceadr[mid]), int(model.mesh_facenum[mid])
                verts = mesh_vert[v_adr * 3:(v_adr + v_num) * 3].astype("<f4").tobytes()
                faces = mesh_face[f_adr * 3:(f_adr + f_num) * 3].astype("<u4").tobytes()
                meshes[mid] = {"vertOffset": offset, "vertCount": v_num,
                               "faceOffset": offset + len(verts), "faceCount": f_num}
                chunks.append(verts)
                chunks.append(faces)
                offset += len(verts) + len(faces)
        geoms.append(entry)

    scene = {"type": "scene", "bodies": bodies, "geoms": geoms, "meshes": meshes,
             "meshBytes": offset}
    return scene, b"".join(chunks)


def pose_message(step: int, sim_time: float, xpos, xquat) -> str:
    import numpy as _np
    flat_pos = _np.asarray(xpos, dtype=float).reshape(-1)
    flat_quat = _np.asarray(xquat, dtype=float).reshape(-1)
    return json.dumps({
        "type": "pose", "step": step, "t": round(float(sim_time), 4),
        "xpos": [round(float(v), 5) for v in flat_pos],
        "xquat": [round(float(v), 5) for v in flat_quat],
    }, separators=(",", ":"))


def _self_test() -> int:
    """不需要浏览器：起服务 → 连上去 → 检查 scene/pose 帧能被正确解析。"""
    import socket as _s
    srv = HostStream(0)
    c = _s.create_connection(("127.0.0.1", srv.port))
    c.sendall(("GET / HTTP/1.1\r\nHost: x\r\nUpgrade: websocket\r\n"
               "Connection: Upgrade\r\nSec-WebSocket-Key: dGhlIHNhbXBsZSBub25jZQ==\r\n"
               "Sec-WebSocket-Version: 13\r\n\r\n").encode())
    head = b""
    while b"\r\n\r\n" not in head:
        head += c.recv(1024)
    assert head.startswith(b"HTTP/1.1 101"), head[:40]
    time.sleep(0.2)
    srv.send(json.dumps({"type": "scene", "bodies": [{"id": 0, "name": "world", "parent": 0}],
                         "geoms": [], "meshes": {}, "meshBytes": 0}), b"")
    srv.send(pose_message(3, 0.06, [0.1, 0.2, 0.3], [1, 0, 0, 0]))
    seen = []
    c.settimeout(2.0)
    while len(seen) < 2:
        opcode, data = _read_frame(c)
        if opcode == 1:
            seen.append(json.loads(data)["type"])
    c.close()
    srv.close()
    ok = seen == ["scene", "pose"]
    print(("PASS" if ok else "FAIL") + f"：收到帧 {seen}")
    return 0 if ok else 1


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--self-test", action="store_true")
    ap.add_argument("--port", type=int, default=0)
    args = ap.parse_args()
    if args.self_test:
        raise SystemExit(_self_test())
    srv = HostStream(args.port)
    print(f"ws://127.0.0.1:{srv.port}/（Ctrl-C 结束）")
    try:
        while True:
            time.sleep(1.0)
    except KeyboardInterrupt:
        srv.close()
