"""Project-local rules for already prepared NPZ references (no training/conversion)."""
import ast
import copy
import json
import math


def literals(source):
    result = {}
    for node in ast.parse(source).body:
        if isinstance(node, ast.Assign) and len(node.targets) == 1 and isinstance(node.targets[0], ast.Name):
            result[node.targets[0].id] = ast.literal_eval(node.value)
        elif isinstance(node, ast.Expr) and isinstance(node.value, ast.Constant) and isinstance(node.value.value, str):
            continue
        else:
            raise ValueError("Configuration must contain literal assignments only")
    return result


def dump(values):
    return "".join(f"{key} = {value!r}\n" for key, value in values.items())


def inspect(arrays, joints, root):
    def names(key):
        value = arrays[key]
        data = value["first"]
        if value["shape"] != [len(data)] or not all(isinstance(n, str) for n in data) or len(set(data)) != len(data):
            raise ValueError(f"Invalid or duplicate {key}")
        return data
    bodies = names("body_names")
    if names("dof_names") != joints:
        raise ValueError("NPZ joint names/order must match this project's 29 G1 joints")
    frames = arrays["joint_pos"]["shape"][0]
    if frames < 2 or len(joints) != 29:
        raise ValueError("NPZ requires at least two frames and 29 joints")
    fps = arrays["fps"]
    if fps["shape"] not in ([], [1]) or fps["first"] != [50]:
        raise ValueError("This example requires a prepared 50 Hz NPZ")
    for key, shape in {
        "joint_pos": [frames, 29], "joint_vel": [frames, 29],
        "body_pos_w": [frames, len(bodies), 3], "body_quat_w": [frames, len(bodies), 4],
        "body_lin_vel_w": [frames, len(bodies), 3], "body_ang_vel_w": [frames, len(bodies), 3],
    }.items():
        if arrays[key]["shape"] != shape or "normRange" not in arrays[key]:
            raise ValueError(f"Invalid numeric NPZ array: {key}")
    lo, hi = arrays["body_quat_w"]["normRange"]
    if abs(lo-1) > 1e-3 or abs(hi-1) > 1e-3:
        raise ValueError("NPZ requires normalized WXYZ body quaternions")
    index = bodies.index(root)
    def first(key, width):
        return arrays[key]["first"][index*width:(index+1)*width]
    q = first("body_quat_w", 4)
    w, x, y, z = q
    rotation = [[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]]
    world_w = first("body_ang_vel_w", 3)
    local_w = [sum(rotation[j][i]*world_w[j] for j in range(3)) for i in range(3)]
    qpos = first("body_pos_w", 3) + q + arrays["joint_pos"]["first"]
    qvel = first("body_lin_vel_w", 3) + local_w + arrays["joint_vel"]["first"]
    if not all(math.isfinite(v) for v in qpos+qvel):
        raise ValueError("NPZ initial state must be finite")
    return frames, qpos, qvel, bodies


def update(context):
    files, arrays = context["files"], context["arrays"]
    choices = literals(files["motion_choices.py"])
    config = literals(files["motion_config.py"])
    previous = choices["CLIPS"]
    index = int(context["parameters"].get("motion_id", previous.index(config["CLIP"]) if config["CLIP"] in previous else 0))
    selected = previous[index] if 0 <= index < len(previous) else None
    paths = {}
    for path in arrays:
        name = path.removeprefix("assets/motions/").removesuffix(".npz")
        if "/" in name or not name:
            raise ValueError("Uni motion NPZ files belong directly in assets/motions/")
        paths[name] = path
    clips = [name for name in previous if name in paths] + sorted(set(paths)-set(previous))
    labels = dict(zip(previous, choices["LABELS"]))
    steps, initial = {}, {}
    required = {"pelvis", "left_hip_roll_link", "left_knee_link", "left_ankle_roll_link",
                "right_hip_roll_link", "right_knee_link", "right_ankle_roll_link", "torso_link",
                "left_shoulder_roll_link", "left_elbow_link", "left_wrist_yaw_link",
                "right_shoulder_roll_link", "right_elbow_link", "right_wrist_yaw_link"}
    for name in clips:
        frames, qpos, qvel, bodies = inspect(arrays[paths[name]], config["JOINT_NAMES"], "pelvis")
        if not required.issubset(bodies):
            raise ValueError("Uni NPZ is missing required tracked bodies")
        steps[name] = frames
        initial[name] = (qpos, qvel)
    selected = selected if selected in clips else (clips[0] if clips else None)
    config.update(CLIP=selected, MOTION=paths[selected] if selected else None, FRAMES=steps[selected] if selected else 0)
    output = {"motion_choices.py": dump({"CLIPS": clips, "LABELS": [labels.get(n, n) for n in clips]}),
              "motion_steps.py": dump({"STEPS": steps}), "motion_config.py": dump(config)}
    if selected:
        preset = copy.deepcopy(context["presets"]["robot.preset.yaml"])
        preset.update(initialQpos=initial[selected][0], initialQvel=initial[selected][1])
        output["robot.preset.yaml"] = json.dumps(preset, indent=2)+"\n"
    return {"files": output, "selections": [{"path": paths[name], "parameter": "motion_id", "value": i} for i, name in enumerate(clips)], **({"blocked": "没有动作可运行 / No motion references. Import a prepared NPZ into assets/motions/."} if not clips else {})}
