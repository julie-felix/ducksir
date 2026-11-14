"""Project-local resource preparation for this template's fixed robot.

NumPy implementation of the source conventions described in csv_config.json.
Kinematic constants are exported from this project's MJCF, not inferred from CSV.
The caller stores only the returned prepared NPZ; source files are never modified.
"""
import base64
import csv
import io
import json
import math
import numpy as np


def unit(q):
    length = np.linalg.norm(q, axis=-1, keepdims=True)
    if np.any(length < 1e-10):
        raise ValueError("CSV contains a zero quaternion")
    return q / length


def multiply(a, b):
    aw, av = a[..., :1], a[..., 1:]
    bw, bv = b[..., :1], b[..., 1:]
    return np.concatenate([aw*bw - np.sum(av*bv, axis=-1, keepdims=True), aw*bv + bw*av + np.cross(av, bv)], axis=-1)


def rotate(q, v):
    v = np.broadcast_to(v, np.broadcast_shapes(q.shape[:-1]+(3,), v.shape))
    cross = 2*np.cross(q[..., 1:], v)
    return v + q[..., :1]*cross + np.cross(q[..., 1:], cross)


def angular(q, half_edges):
    prev = np.concatenate([q[:1], q[:-1]])
    following = np.concatenate([q[1:], q[-1:]])
    inverse = prev.copy(); inverse[..., 1:] *= -1
    delta = unit(multiply(following, inverse))
    delta = np.where(delta[..., :1] < 0, -delta, delta)
    n = np.linalg.norm(delta[..., 1:], axis=-1, keepdims=True)
    angle = 2*np.arctan2(n, delta[..., :1])
    result = delta[..., 1:] * np.divide(angle, n, out=np.full_like(n, 2), where=n>1e-10) * 25
    if not half_edges: result[[0, -1]] *= 2
    return result


def velocity(x, half_edges):
    v = (np.concatenate([x[1:], x[-1:]]) - np.concatenate([x[:1], x[:-1]]))*25
    if not half_edges: v[[0, -1]] *= 2
    return v


def resample(p, r, q, fps, include_endpoint):
    span = (len(q)-1)/fps*50
    count = math.floor(span)+1 if include_endpoint else math.ceil(span)
    if count < 2: raise ValueError("CSV must produce at least two 50 Hz frames")
    f = np.minimum(np.arange(count)*fps/50, len(q)-1)
    lo = np.minimum(f.astype(int), len(q)-2); hi = lo+1
    t = (f-lo)[:, None]
    a, b = unit(r[lo]), unit(r[hi])
    dot = np.sum(a*b, axis=-1, keepdims=True)
    b = np.where(dot < 0, -b, b); dot = np.abs(dot).clip(0, 1)
    angle = np.arccos(dot); sine = np.sin(angle)
    left = np.divide(np.sin((1-t)*angle), sine, out=1-t.copy(), where=sine>1e-6)
    right = np.divide(np.sin(t*angle), sine, out=t.copy(), where=sine>1e-6)
    return (1-t)*p[lo]+t*p[hi], unit(left*a+right*b), (1-t)*q[lo]+t*q[hi]


def forward(p, r, q, joints, skeleton, half_edges):
    positions, rotations, velocities, angulars = [], [], [], []
    qd = velocity(q, half_edges)
    for body in skeleton:
        parent = body['parent']
        if parent == -1:
            bp, br, bv, bw = p, r, velocity(p, half_edges), angular(r, half_edges)
        else:
            offset = rotate(rotations[parent], np.array(body['position']))
            bp = positions[parent]+offset
            br = multiply(rotations[parent], unit(np.array(body['quaternion'])))
            bv = velocities[parent]+np.cross(angulars[parent], offset)
            bw = angulars[parent]
            if body['joint'] is not None:
                index = joints.index(body['joint'])
                axis = np.array(body['axis']); axis /= np.linalg.norm(axis)
                angle = q[:, index:index+1]/2
                bw = bw + rotate(br, axis)*qd[:, index:index+1]
                br = multiply(br, np.concatenate([np.cos(angle), np.sin(angle)*axis], axis=1))
        positions.append(bp); rotations.append(unit(br)); velocities.append(bv); angulars.append(bw)
    bp, bq = np.stack(positions, axis=1), np.stack(rotations, axis=1)
    if half_edges: return bp, bq, velocity(bp, True), angular(bq, True)
    return bp, bq, np.stack(velocities, axis=1), np.stack(angulars, axis=1)


def convert(context):
    settings = json.loads(context['files']['csv_config.json'])
    kin = json.loads(context['files']['motion_kinematics.json'])
    path = context['path']
    if '/' in path.removeprefix('assets/motions/') or not path.startswith('assets/motions/'):
        raise ValueError('CSV must be directly inside assets/motions/')
    joints = kin['joints']
    source = read_source(context, settings, joints)
    if isinstance(source, dict) and 'needsInput' in source: return source
    scene = scene_input(context)
    if 'needsInput' in scene: return scene
    if isinstance(source, dict):
        return {**source, 'inputOptions': scene}
    p, r, q, fps, pelvis_root, resolved = source
    if not math.isfinite(fps) or fps <= 0: raise ValueError('source_fps must be positive')
    if len(q) < 2 or not all(np.isfinite(x).all() for x in (p, r, q)):
        raise ValueError('Source requires at least two finite numeric frames')
    if np.any(np.abs(np.linalg.norm(r, axis=1)-1) > 1e-3):
        raise ValueError('Source requires normalized root quaternions')
    p, r, q = resample(p, r, q, fps, INCLUDE_ENDPOINT)
    if pelvis_root:
        # Motion Studio G1 is pelvis-rooted; this model reverses the waist chain
        # and uses torso_link as its free root. Its waist coordinates have the
        # opposite sign. Solve the target torso pose that preserves the source
        # pelvis world pose using this project's own kinematic constants.
        for name in ['waist_yaw_joint', 'waist_roll_joint', 'waist_pitch_joint']:
            q[:, joints.index(name)] *= -1
        zero = np.zeros_like(p)
        identity = np.tile([1., 0., 0., 0.], (len(q), 1))
        local_p, local_r, _, _ = forward(zero, identity, q, joints, kin['bodies'], True)
        pelvis = [b['name'] for b in kin['bodies']].index('pelvis')
        inverse = local_r[:, pelvis].copy(); inverse[:, 1:] *= -1
        r = unit(multiply(r, inverse))
        p = p - rotate(r, local_p[:, pelvis])
    bp, bq, bv, bw = forward(p, r, q, joints, kin['bodies'], not INCLUDE_ENDPOINT)
    names = [b['name'] for b in kin['bodies']]
    bp, bq, bv, bw, names = finish(bp, bq, bv, bw, names, settings)
    stream = io.BytesIO()
    np.savez_compressed(stream, fps=np.array([50]), dof_names=np.array(joints), body_names=np.array(names),
                        joint_pos=q.astype(np.float32), joint_vel=velocity(q, not INCLUDE_ENDPOINT).astype(np.float32),
                        body_pos_w=bp.astype(np.float32), body_quat_w=bq.astype(np.float32),
                        body_lin_vel_w=bv.astype(np.float32), body_ang_vel_w=bw.astype(np.float32))
    return {'path': path[:-4]+'.npz', 'base64': base64.b64encode(stream.getvalue()).decode('ascii'), 'inputOptions': {**resolved, **scene}}


def scene_input(context):
    """An export's axis convention does not bind it to this finite terrain.

    This is an explicit scene/placement choice, not a claim of contact support or
    policy accuracy. Never move the reference or create a floor implicitly.
    """
    if context.get('options', {}).get('scene_binding') == 'current-scene-world':
        return {'scene_binding': 'current-scene-world'}
    return {'needsInput': {
        'message': 'Instinct 使用有限地形，新动作未与地形绑定，原位置可能没有支撑。请先检查场景和落点。只有明确要在当前场景保留原坐标试运行时才继续；否则取消，先调整场景或动作位置。继续不代表跟踪质量达标。 / Finite terrain: choose the intended scene and placement before a trial. Cancel to adjust the scene first; this does not validate support contacts or tracking quality.',
        'fields': [{'name': 'scene_binding', 'label': '场景与动作位置 / Scene and placement', 'type': 'select',
                    'options': [{'value': 'current-scene-world', 'label': '在当前场景按原坐标试运行（不移动动作） / Trial in current scene at original coordinates'}]}]}}

# InstinctMJ supermimic-v0.1 soma.csv conventions; see NOTICE.py (CC BY-NC 4.0).
INCLUDE_ENDPOINT = False


def read_csv(text, path, joints):
    if not path.endswith('soma.csv'): raise ValueError('Instinct CSV must use the upstream soma.csv format and filename suffix')
    rows = list(csv.reader(line for line in text.splitlines() if line.strip()))
    if not rows or len(rows[0]) != 7+len(joints) or any(len(row) != len(rows[0]) for row in rows[1:]):
        raise ValueError('Instinct soma.csv requires a header: frame, XYZ centimetres, Euler XYZ degrees, 29 named joint degrees')
    names = [name.strip().removesuffix('_dof') for name in rows[0][7:]]
    if len(set(names)) != len(names) or set(names) != set(joints):
        raise ValueError('Instinct soma.csv joint headers must match the project joints exactly')
    data = np.asarray(rows[1:], dtype=float)
    if data.ndim != 2: raise ValueError('Instinct soma.csv needs numeric data rows')
    e = np.deg2rad(data[:, 4:7])/2
    x,y,z = e[:, 0],e[:, 1],e[:, 2]
    cx,cy,cz,sx,sy,sz = np.cos(x),np.cos(y),np.cos(z),np.sin(x),np.sin(y),np.sin(z)
    r = np.stack([cx*cy*cz+sx*sy*sz, sx*cy*cz-cx*sy*sz, cx*sy*cz+sx*cy*sz, cx*cy*sz-sx*sy*cz], axis=1)
    return data[:, 1:4]*.01, r, np.deg2rad(data[:, [7+names.index(n) for n in joints]])


def finish(bp, bq, bv, bw, names, settings):
    # Source positions are already in this example scene's world frame.
    if settings['coordinates'] != 'scene-world': raise ValueError('Instinct CSV positions must use scene-world coordinates')
    tracked = ['pelvis', 'torso_link', 'left_shoulder_roll_link', 'right_shoulder_roll_link',
               'left_elbow_link', 'right_elbow_link', 'left_wrist_yaw_link', 'right_wrist_yaw_link',
               'left_hip_roll_link', 'right_hip_roll_link', 'left_knee_link', 'right_knee_link',
               'left_ankle_roll_link', 'right_ankle_roll_link']
    lift = .1+max(0, -float(bp[0, [names.index(n) for n in tracked], 2].min()))
    root = bp[:, :1].copy(); root[:, :, 2] += lift
    return (np.concatenate([root,bp],axis=1), np.concatenate([bq[:, :1],bq],axis=1),
            np.concatenate([bv[:, :1],bv],axis=1), np.concatenate([bw[:, :1],bw],axis=1), ['__reset_root__',*names])


MOTION_STUDIO_G1_JOINTS = ['left_hip_pitch_joint', 'left_hip_roll_joint', 'left_hip_yaw_joint', 'left_knee_joint', 'left_ankle_pitch_joint', 'left_ankle_roll_joint', 'right_hip_pitch_joint', 'right_hip_roll_joint', 'right_hip_yaw_joint', 'right_knee_joint', 'right_ankle_pitch_joint', 'right_ankle_roll_joint', 'waist_yaw_joint', 'waist_roll_joint', 'waist_pitch_joint', 'left_shoulder_pitch_joint', 'left_shoulder_roll_joint', 'left_shoulder_yaw_joint', 'left_elbow_joint', 'left_wrist_roll_joint', 'left_wrist_pitch_joint', 'left_wrist_yaw_joint', 'right_shoulder_pitch_joint', 'right_shoulder_roll_joint', 'right_shoulder_yaw_joint', 'right_elbow_joint', 'right_wrist_roll_joint', 'right_wrist_pitch_joint', 'right_wrist_yaw_joint']


def read_source(context, settings, joints):
    path = context['path']
    if path.endswith('.npz'):
        raw = context.get('bytes')
        if raw is None: raw = base64.b64decode(context['base64'], validate=True)
        with np.load(io.BytesIO(raw), allow_pickle=False) as source:
            keys = set(source.files)
            if 'qpos' not in keys:
                required = {'fps', 'dof_names', 'body_names', 'joint_pos', 'joint_vel', 'body_pos_w', 'body_quat_w', 'body_lin_vel_w', 'body_ang_vel_w'}
                missing = sorted(required - keys)
                if missing: raise ValueError('Unsupported NPZ: missing prepared fields '+', '.join(missing)+'; not a declared Motion Studio qpos archive')
                return {'path': path, 'base64': base64.b64encode(raw).decode('ascii')}
            if 'motion_studio_qpos_layout' not in keys or source['motion_studio_qpos_layout'].tolist() != ['quaternion_first']:
                raise ValueError('qpos NPZ requires an explicit Motion Studio quaternion_first layout marker')
            for name in ['joint_names', 'fps']:
                if name not in keys: raise ValueError('Motion Studio NPZ missing '+name)
            names = source['joint_names'].tolist()
            if not isinstance(names, list) or len(names) != len(joints) or len(set(names)) != len(names) or set(names) != set(joints):
                raise ValueError('Source joint_names must match the project robot joints exactly')
            a = source['qpos']
            if a.ndim != 2 or a.shape[1] != 7+len(names): raise ValueError('Expected root WXYZ, XYZ, and named joint columns; object tracks need their own processor')
            f = source['fps']
            if f.size != 1: raise ValueError('fps must contain one number')
            fps = float(f.reshape(-1)[0])
            p, r, q = a[:, 4:7].copy(), a[:, :4].copy(), a[:, [7+names.index(n) for n in joints]].copy()
        resolved = input_options(context, settings, csv_input=False, embedded_fps=fps)
        if 'needsInput' in resolved: return resolved
    else:
        resolved = input_options(context, settings, csv_input=True)
        if 'needsInput' in resolved: return resolved
        text = context['text'].lstrip('\ufeff')
        fps = float(resolved['source_fps'])
        if resolved['source_format'] == 'legacy':
            p, r, q = read_csv(text, path, joints)
        else:
            rows = list(csv.reader(line for line in text.splitlines() if line.strip() and not line.lstrip().startswith('#')))
            if not rows or any(len(row) != 36 for row in rows): raise ValueError('Motion Studio G1 CSV requires 36 columns: XYZ metres, XYZW quaternion, 29 joint radians')
            a = np.asarray(rows, dtype=float)
            names = MOTION_STUDIO_G1_JOINTS
            if set(names) != set(joints): raise ValueError('Motion Studio G1 CSV joint contract does not match this robot')
            p, r, q = a[:, :3].copy(), a[:, [6, 3, 4, 5]].copy(), a[:, [7+names.index(n) for n in joints]].copy()
    if resolved['source_coordinates'] == 'stage-y-up':
        p = p[:, [0, 2, 1]] * [1, -1, 1]
        r = r[:, [0, 1, 3, 2]] * [1, 1, -1, 1]
    return p, r, q, fps, resolved['source_format'] == 'motionstudio-g1', resolved


def input_options(context, settings, *, csv_input, embedded_fps=None):
    """Resolve explicit evidence; neither filenames nor poses identify coordinates."""
    path = context['path']
    metadata = []
    for name, text in context['files'].items():
        if name.split('/')[-1] != 'motion-studio-project.json': continue
        manifest = json.loads(text)
        for asset in manifest.get('assets', []):
            if str(asset.get('sourcePath', '')).split('/')[-1] == path.split('/')[-1]:
                metadata.append(asset.get('properties', {}))
    declared = {}
    if metadata:
        if any(m != metadata[0] for m in metadata[1:]):
            raise ValueError('Conflicting export metadata for this filename; select the matching source manifest')
        m = metadata[0]
        if m.get('robotType') != 'unitree_g1' or m.get('motionDataRole') != 'retargeted':
            raise ValueError('Export metadata does not declare a retargeted unitree_g1 motion')
        expected_order = 'xyzw' if csv_input else 'wxyz'
        if m.get('quaternionOrder', expected_order) != expected_order:
            raise ValueError('Export metadata quaternion order conflicts with the declared source format')
        declared['source_format'] = 'motionstudio-g1'
        if m.get('fps') is not None: declared['source_fps'] = m['fps']
        # The manifest describes the exported bytes, not the viewer's up axis.
        # Motion Studio preserves source coordinates when serializing CSV/NPZ.
        axes = m.get('axisMapping')
        if axes == {'x': '+x', 'y': '+y', 'z': '+z'}:
            declared['source_coordinates'] = 'stage-y-up'
        elif axes == {'x': '+x', 'y': '-z', 'z': '+y'}:
            declared['source_coordinates'] = 'scene-z-up'
        elif axes is not None:
            raise ValueError('Unsupported export axisMapping; this project supports Stage Y-up or retarget Z-up only')
    profile = {}
    if settings.get('source_format'): profile['source_format'] = settings['source_format']
    if settings.get('motion_studio_coordinates'): profile['source_coordinates'] = settings['motion_studio_coordinates']
    if settings.get('motion_studio_fps'): profile['source_fps'] = settings['motion_studio_fps']
    if not csv_input and profile.get('source_format') == 'legacy': profile = {}
    if profile.get('source_format') == 'legacy':
        profile['source_fps'] = settings['source_fps']
        if settings.get('coordinates') == 'scene-world': profile['source_coordinates'] = 'scene-z-up'
    explicit = dict(context.get('options', {}))
    # Keep the old name readable, but never infer coordinates from "default".
    # Motion Studio exports can preserve either source basis; FPS also varies.
    if explicit.get('source_format') == 'motionstudio-g1-default':
        explicit['source_format'] = 'motionstudio-g1'
    for key, value in declared.items():
        if key in explicit and explicit[key] != value:
            raise ValueError('Saved/explicit '+key+' conflicts with this export manifest; correct the input settings before importing')
    resolved = {**profile, **explicit, **declared}
    if not csv_input:
        if 'source_fps' in resolved and float(resolved['source_fps']) != embedded_fps:
            raise ValueError('Export metadata fps conflicts with NPZ fps')
        resolved['source_format'] = 'motionstudio-g1'
        resolved['source_fps'] = embedded_fps
    fields = []
    if resolved.get('source_format') not in ['motionstudio-g1', 'legacy']:
        fields.append({'name': 'source_format', 'label': '数据格式 / Source format', 'type': 'select', 'options': [
            {'value': 'motionstudio-g1', 'label': 'Motion Studio G1 导出 / Export'}, {'value': 'legacy', 'label': '项目原有 CSV 格式 / Legacy project CSV'}]})
    if fields:
        return {'needsInput': {'message': '请确认此文件的格式；坐标系由对应导出清单或已确认设置确定。 / Confirm the file format; coordinates come from matching export metadata or confirmed settings.', 'fields': fields}}
    if resolved.get('source_coordinates') not in ['stage-y-up', 'scene-z-up']:
        fields.append({'name': 'source_coordinates', 'label': '导出坐标系 / Export coordinates', 'type': 'select', 'options': [
            {'value': 'stage-y-up', 'label': 'Stage Y-up（Y 轴向上）'}, {'value': 'scene-z-up', 'label': 'Retarget / Scene Z-up（Z 轴向上）'}]})
    if csv_input and resolved.get('source_fps') is None:
        fields.append({'name': 'source_fps', 'label': '采样率 / FPS', 'type': 'number'})
    if [field['name'] for field in fields] == ['source_fps']:
        return {'needsInput': {'message': '格式和坐标系已确定。此 CSV 未记录帧率，请填写导出时的 FPS。 / Format and coordinates are resolved. Enter the export FPS, which this CSV does not record.', 'fields': fields}}
    if fields:
        return {'needsInput': {'message': '文件及项目配置没有完整说明以下信息。可一并导入对应的 motion-studio-project.json，或补充已知的导出设置。 / Missing source metadata: include the matching export manifest or supply the known export settings.', 'fields': fields}}
    fps = float(resolved['source_fps'])
    if not math.isfinite(fps) or fps <= 0: raise ValueError('source_fps must be positive')
    return {key: resolved[key] for key in ['source_format', 'source_coordinates', 'source_fps']}
