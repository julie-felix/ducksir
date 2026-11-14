#!/usr/bin/env python3
"""深度相机：逐行照 botlab 运行时的实现（`botlab/_runtime/_observations.py`）。

它们不是「往模型里加一个相机」，而是：

1. 算出相机的 **pos / forward / up**（three.js 轴系 → MuJoCo 轴系的转换）；
2. 用 `mjCAMERA_FREE` 让 MuJoCo 建场景，再把 `renderer.scene.camera[*].pos/forward/up`
   和 frustum（near/far/bottom/top/width）直接写进去；
3. 深度线性化：临时把 `model.vis.map.znear/zfar` 改成 `near/extent`、`far/extent`；
4. `enable_depth_rendering()` 渲染，随后全部还原。

帧尺寸是「全幅」48×27（`fullWidth/fullHeight`），裁 2,2 → 44×23，再缩到 32×18 才是观测。

    MUJOCO_GL=egl python3 -c "import depth_camera"
"""
from __future__ import annotations

import math

import numpy as np

DEFAULT_NEAR = 0.05
DEFAULT_FAR = 100.0
DEFAULT_FOV = 45.0


def _three_to_mujoco(v):
    v = np.asarray(v, dtype=np.float64)
    return np.array([v[0], -v[2], v[1]], dtype=np.float64)


def _mujoco_to_three(v):
    v = np.asarray(v, dtype=np.float64)
    return np.array([v[0], v[2], -v[1]], dtype=np.float64)


def _quat_to_matrix(q):
    w, x, y, z = (float(c) for c in np.asarray(q, dtype=np.float64).reshape(-1)[:4])
    n = w * w + x * x + y * y + z * z
    if n <= 0:
        return np.eye(3)
    s = 2.0 / n
    return np.array([
        [1 - s * (y * y + z * z), s * (x * y - z * w), s * (x * z + y * w)],
        [s * (x * y + z * w), 1 - s * (x * x + z * z), s * (y * z - x * w)],
        [s * (x * z - y * w), s * (y * z + x * w), 1 - s * (x * x + y * y)],
    ], dtype=np.float64)


def local_rotation(spec: dict) -> np.ndarray:
    """`cameraRotationMode: xyaxes`：两个轴从 MuJoCo 转到 three.js 再拼成基。"""
    raw = spec.get("cameraXyaxes") or []
    if spec.get("cameraRotationMode") != "xyaxes" or len(raw) != 6:
        return np.eye(3)
    x = _mujoco_to_three(raw[0:3])
    y = _mujoco_to_three(raw[3:6])
    x = x / np.linalg.norm(x)
    y = y / np.linalg.norm(y)
    z = np.cross(x, y)
    z = z / np.linalg.norm(z)
    y = np.cross(z, x)              # 再正交化，和 botlab 一致
    y = y / np.linalg.norm(y)
    # 列基（axis 作列），不是行基——写错会得到「位置对、朝向错」的经典症状
    return np.column_stack([x, y, z])


def camera_pose(body_pos, body_quat, spec: dict):
    """返回 (camera_position, forward, up)，都是 MuJoCo 世界坐标。"""
    target_matrix = _quat_to_matrix(body_quat)
    frame = target_matrix
    offset_frame = target_matrix if spec.get("cameraOffsetFrame") == "body" else frame
    offset = np.asarray(spec.get("cameraOffset") or [0.0, 0.0, 0.0], dtype=np.float64)
    rot = local_rotation(spec)
    forward = frame @ _three_to_mujoco(rot @ np.array([0.0, 0.0, -1.0]))
    up = frame @ _three_to_mujoco(rot @ np.array([0.0, 1.0, 0.0]))
    position = np.asarray(body_pos, dtype=np.float64) + offset_frame @ offset
    return position, forward, up


class DepthCamera:
    """按 botlab 的口径渲染深度：全幅 → (可选) 裁剪 → 缩放。"""

    def __init__(self, model, spec: dict):
        from mujoco.rendering.classic.renderer import Renderer

        self.model = model
        self.spec = spec
        self.width = int(spec.get("fullWidth", 48))
        self.height = int(spec.get("fullHeight", 27))
        self.near = float(spec.get("near", DEFAULT_NEAR))
        self.far = float(spec.get("far", DEFAULT_FAR))
        self.fov = float(spec.get("fov", DEFAULT_FOV))
        self.horizontal_fov = spec.get("horizontalFov")
        self.groups = list(spec.get("raycastGeomGroups") or [])
        # 他们场景里有一块地形摆在 x=+500，把 stat.extent 撑到 ~757；
        # 深度线性化用 extent，所以这里允许显式覆盖，保证和他们的数值口径一致。
        forced = spec.get("extentOverride")
        if forced:
            model.stat.extent = float(forced)
        self.crop_x = int(spec.get("cropX", 0))
        self.crop_y = int(spec.get("cropY", 0))
        self.out_w = int(spec.get("width", 32))
        self.out_h = int(spec.get("height", 18))
        self.gauss_kernel = int(spec.get("depthGaussianKernelSize") or 0)
        self.gauss_sigma = float(spec.get("depthGaussianSigma") or 1.0)
        self.renderer = Renderer(model, self.height, self.width)
        self.option = None
        if self.groups:
            import mujoco
            self.option = mujoco.MjvOption()
            self.option.geomgroup[:] = [1 if g in set(self.groups) else 0 for g in range(6)]

    def render(self, data) -> np.ndarray:
        import mujoco

        cam = mujoco.MjvCamera()
        cam.type = mujoco.mjtCamera.mjCAMERA_FREE
        prev_fovy = float(self.model.vis.global_.fovy)
        self.model.vis.global_.fovy = self.fov
        try:
            self.renderer.update_scene(data, camera=cam, scene_option=self.option)
        finally:
            self.model.vis.global_.fovy = prev_fovy

        half_h = self.near * math.tan(math.radians(self.fov) / 2.0)
        half_w = (0.0 if self.horizontal_fov is None else
                  self.near * math.tan(math.radians(float(self.horizontal_fov)) / 2.0))
        body_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, self.spec["target"])
        pos, fwd, up = camera_pose(data.xpos[body_id], data.xquat[body_id], self.spec)
        for side in (0, 1):
            view = self.renderer.scene.camera[side]
            view.pos[:] = pos
            view.forward[:] = fwd
            view.up[:] = up
            view.frustum_near = self.near
            view.frustum_far = self.far
            view.frustum_bottom = -half_h
            view.frustum_top = half_h
            view.frustum_center = 0.0
            view.frustum_width = half_w
            view.orthographic = 0

        extent = float(self.model.stat.extent) or 1.0
        prev = (float(self.model.vis.map.znear), float(self.model.vis.map.zfar))
        self.model.vis.map.znear = self.near / extent
        self.model.vis.map.zfar = self.far / extent
        self.renderer.enable_depth_rendering()
        try:
            depth = np.asarray(self.renderer.render(), dtype=np.float32)
        finally:
            self.model.vis.map.znear, self.model.vis.map.zfar = prev
            self.renderer.disable_depth_rendering()
        return depth

    def process(self, full: np.ndarray, out_w: int = 32, out_h: int = 18) -> np.ndarray:
        """全幅 → 裁窗口 → `clip(m/depthScale, 0, 1)` → 缩放到观测尺寸。

        实测（与 botlab 真值比）：裁 (2,2)、`depthScale=2`、`align_corners=False` 的双线性，
        残差 0.0986、相关 0.9993；换成 align_corners=True 差 0.1246。
        """
        h, w = full.shape
        if self.spec.get("cropMode") == "offset":
            # parkour 的 preset 里 cropX/cropY 是**左上角偏移**（64×36 → 裁 32×18）
            window = full[self.crop_y:self.crop_y + int(self.out_h),
                          self.crop_x:self.crop_x + int(self.out_w)]
        else:
            window = full[self.crop_y:h - self.crop_y, self.crop_x:w - self.crop_x]
        if self.gauss_kernel >= 3:
            # botlab 对裁剪后的深度做 3×3 高斯（σ=1）再进网络
            import numpy as _np
            r = self.gauss_kernel // 2
            g = _np.exp(-(_np.arange(-r, r + 1) ** 2) / (2 * self.gauss_sigma ** 2))
            g /= g.sum()
            padded = _np.pad(window, ((r, r), (r, r)), mode="edge")
            window = sum(g[i] * sum(g[j] * padded[i:i + window.shape[0], j:j + window.shape[1]]
                                   for j in range(self.gauss_kernel))
                         for i in range(self.gauss_kernel))
        scale = float(self.spec.get("depthScale") or 1.0)
        img = np.clip(window.astype(np.float64) / scale, 0.0, 1.0)
        return resize_depth(img, out_w, out_h, align_corners=False)


def resize_depth(depth: np.ndarray, out_w: int, out_h: int, *, align_corners: bool = True) -> np.ndarray:
    src_h, src_w = depth.shape
    if align_corners:
        ys = np.linspace(0, src_h - 1, out_h)
        xs = np.linspace(0, src_w - 1, out_w)
    else:                                    # torch/OpenCV 的 align_corners=False 口径
        ys = (np.arange(out_h) + 0.5) * src_h / out_h - 0.5
        xs = (np.arange(out_w) + 0.5) * src_w / out_w - 0.5
        ys = np.clip(ys, 0, src_h - 1)
        xs = np.clip(xs, 0, src_w - 1)
    y0 = np.floor(ys).astype(int); y1 = np.minimum(y0 + 1, src_h - 1)
    x0 = np.floor(xs).astype(int); x1 = np.minimum(x0 + 1, src_w - 1)
    wy = (ys - y0)[:, None]; wx = (xs - x0)[None, :]
    top = depth[y0][:, x0] * (1 - wx) + depth[y0][:, x1] * wx
    bot = depth[y1][:, x0] * (1 - wx) + depth[y1][:, x1] * wx
    return (top * (1 - wy) + bot * wy).astype(np.float32)
