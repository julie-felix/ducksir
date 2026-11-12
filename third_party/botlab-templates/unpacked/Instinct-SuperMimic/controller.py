"""SuperMimic inference, assembled solely from existing BotLab signals."""

from botlab import (
    Model,
    Motion,
    PolicyController,
    Robot,
    Sensor,
    clock,
    concat,
    constant,
    history,
    project,
    relative_position,
    relative_quaternion,
    resize,
    tangent_normal,
    where,
    mode_select,
)
from math import ceil
from motion_choices import CONFIGS, DEFAULT_MOTION

if not CONFIGS:
    raise ValueError("没有动作可运行 / No motion references in assets/motions/")
CONFIG = CONFIGS[DEFAULT_MOTION]

project(
    "Instinct-SuperMimic",
    control_hz=50,
    physics_hz=200,
    parameters={
        "motion_id": {
            "label": {"en": "Reference motion", "zh": "参考动作"},
            "default": DEFAULT_MOTION,
            "options": [
                {"label": cfg["label"], "value": cfg["case"]} for cfg in CONFIGS
            ],
        }
    },
)
robot = Robot("assets/robot/scene.xml", preset="robot.preset.yaml", name="G1")
motions = [Motion(cfg["motion"]) for cfg in CONFIGS]
camera = Sensor(robot, preset="camera.preset.yaml", name="Depth camera")
policy = Model("assets/models/policy.onnx", provider="wasm")


def observations():
    t = clock()
    choice = robot.commands("motion_id")

    def select(values):
        return mode_select(choice, values[0], dict(enumerate(values)), passive=-1)

    def sample(time, channel, indices, *, preview=False):
        return select(
            [
                motion.sample(
                    time,
                    channel=channel,
                    indices=indices,
                    interpolation="nearest",
                    preview=preview,
                )
                for motion in motions
            ]
        )

    duration = select([constant([(cfg["frames"] - 0.5) / 50]) for cfg in CONFIGS])
    # Only the current cursor drives Ghost; future lookahead samples are invisible.
    p0 = sample(t, "body_position", ["torso_link"], preview=True)
    r0 = sample(t, "body_quaternion", ["torso_link"])
    robot_r = robot.body_quaternion("torso_link")
    jp = []
    jv = []
    positions = []
    rotations = []
    for i in range(10):
        ti = t + i * 0.1
        # Upstream rounds to 50 Hz before checking validity. Control ticks and
        # offsets are on that grid; the half-frame margin avoids float32 boundary loss.
        valid = ti.lt(duration)
        jp.append(
            where(
                valid,
                sample(ti, "joint_position", CONFIG["joint_names"])
                - constant(CONFIG["default_angles"]),
                constant(0, size=29),
            )
        )
        jv.append(
            where(
                valid,
                sample(ti, "joint_velocity", CONFIG["joint_names"]),
                constant(0, size=29),
            )
        )
        positions.append(
            relative_position(sample(ti, "body_position", ["torso_link"]), p0, r0)
        )
        rotations.append(
            where(
                valid,
                tangent_normal(
                    relative_quaternion(
                        sample(ti, "body_quaternion", ["torso_link"]), robot_r
                    )
                ),
                constant(0, size=6),
            )
        )
    image = resize(camera.depth(max_age=0), input_size=(44, 23), output_size=(32, 18))
    # Each term is separately time-major, oldest first. Default history fill
    # repeats the first observation, as mjlab's history buffer does on reset.
    # MuJoCo root angular velocity is read in body frame.
    proprio = [
        robot.projected_gravity(target="torso_link"),
        robot.base_angular_velocity(),
        robot.joint_position(default=robot.default_angles),
        robot.joint_velocity(),
        robot.last_action(),
    ]
    return concat(
        [
            concat(jp),
            concat(jv),
            concat(positions),
            concat(rotations),
            image,
            *[history(x, 8, layout="by-frame", order="oldest-first") for x in proprio],
        ]
    )


def create_controller(sim):
    action = policy(observations(), input_name="obs", output_name="actions").action()
    return PolicyController(
        robot,
        control=robot.default_angles + robot.action_scale * action,
        end_after_steps={
            "parameter": "motion_id",
            "steps": {
                cfg["case"]: ceil(cfg["frames"] * 50 / cfg["fps"]) for cfg in CONFIGS
            },
        },
        reset_on_change={"parameter": "motion_id", "motions": dict(enumerate(motions))},
    )
