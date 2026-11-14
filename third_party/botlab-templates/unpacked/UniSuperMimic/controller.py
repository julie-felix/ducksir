"""UniSuperMimic's 99D history + 730D goal, using public BotLab signals."""

from botlab import (
    Model,
    Motion,
    PolicyController,
    Robot,
    clock,
    concat,
    history,
    project,
    quaternion_heading,
    quaternion_inverse,
    relative_position,
    relative_quaternion,
    rotate,
    rotation_6d,
    mode_select,
)
from motion_config import JOINT_NAMES, CLIP
from motion_choices import CLIPS, LABELS
from motion_steps import STEPS

if not CLIPS:
    raise ValueError("没有动作可运行 / No motion references in assets/motions/")

project(
    "UniSuperMimic",
    control_hz=50,
    physics_hz=200,
    parameters={
        "motion_id": {
            "label": {"en": "Reference motion", "zh": "参考动作"},
            "default": CLIPS.index(CLIP),
            "options": [
                {"label": label, "value": index} for index, label in enumerate(LABELS)
            ],
        }
    },
)
robot = Robot("assets/robot/scene.xml", preset="robot.preset.yaml", name="G1")
motions = [Motion(f"assets/motions/{clip}.npz") for clip in CLIPS]
policy = Model("assets/models/policy.onnx", provider="auto")


def observations():
    # Integer control ticks keep reference sampling on the original 50 Hz grid.
    t = clock(unit="steps") * 0.02

    choice = robot.commands("motion_id")

    def sample(time, channel, indices, *, preview=False):
        values = [
            motion.sample(
                time,
                channel=channel,
                indices=indices,
                interpolation="nearest",
                preview=preview,
            )
            for motion in motions
        ]
        return mode_select(choice, values[0], dict(enumerate(values)), passive=-1)

    # One current-time cursor per clip. Future policy targets are not Ghost cursors.
    p0 = sample(t, "body_position", ["pelvis"], preview=True)
    r0 = sample(t, "body_quaternion", ["pelvis"])
    heading = quaternion_heading(r0)
    obs = concat(
        [
            robot.joint_position(default=robot.default_angles),
            robot.joint_velocity(),
            robot.base_angular_velocity(),
            robot.projected_gravity(),
            robot.last_action(),
            # Upstream uses the free-joint orientation from qpos, not cached body FK.
            rotation_6d(relative_quaternion(r0, robot.qpos([3, 4, 5, 6]))),
        ]
    )
    goals = []
    for offset in range(1, 11):
        future = t + offset * 0.02
        q = sample(future, "body_quaternion", ["pelvis"])
        goals.append(
            concat(
                [
                    relative_position(
                        sample(future, "body_position", ["pelvis"]), p0, heading
                    ),
                    rotation_6d(relative_quaternion(q, heading)),
                    rotate(
                        sample(future, "body_linear_velocity", ["pelvis"]),
                        quaternion_inverse(q),
                    ),
                    rotate(
                        sample(future, "body_angular_velocity", ["pelvis"]),
                        quaternion_inverse(q),
                    ),
                    sample(future, "joint_position", JOINT_NAMES),
                    sample(future, "joint_velocity", JOINT_NAMES) * 0.1,
                ]
            )
        )
    return concat(
        [history(obs, 10, layout="by-frame", order="oldest-first"), concat(goals)]
    )


def create_controller(sim):
    action = policy(observations(), input_name="obs", output_name="actions").action()
    # The original MJCF already contains position actuators (kp/kv/force limits).
    # Write their position targets directly, avoiding a second PD stage.
    return PolicyController(
        robot,
        control=robot.default_angles + robot.action_scale * action,
        end_after_steps={
            "parameter": "motion_id",
            "steps": {i: STEPS[clip] for i, clip in enumerate(CLIPS)},
        },
        reset_on_change={"parameter": "motion_id", "motions": dict(enumerate(motions))},
    )
