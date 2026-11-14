from botlab import (
    Model,
    Motion,
    PolicyController,
    Robot,
    mode_switch,
    motion_reference,
    policy_bank,
    project,
    read_config,
    resource_config,
)
from wbc_policy import (
    MODE_TRANSITIONS,
    MOTION_COMMAND_CODES,
    joint_control,
    policies,
)


runtime = read_config("config/runtime.yaml")
project(
    runtime["name"],
    control_hz=runtime["control_hz"],
    physics_hz=runtime["physics_hz"],
    parameters=read_config("config/parameters.yaml"),
)

robot = Robot("models/g1/scene_29dof.xml", preset="g1-wbc.preset.yaml", name="Robot")
motion = Motion(
    "templates/g1-wbc/assets/motions/dance12",
    name="WBC Motion",
)
models = [
    Model(
        "assets/models/dance12_0207_1.onnx",
        name="WBC Actor",
    ),
    Model(
        "assets/models/loco_0731.onnx",
        name="Loco Actor",
    ),
    Model(
        "assets/models/amp_0309_1.onnx",
        name="AMP Actor",
    ),
    Model(
        "assets/models/Unitree-G1-AMP-Flat_model_30000.onnx",
        name="MJAMP Actor",
    ),
]


def create_controller(sim):
    # Observation
    mode_command = robot.commands(**read_config("config/io.yaml")["mode_command"])
    # Zero-speed Loco balances the free base at startup.
    mode = mode_switch(mode_command, initial=2, transitions=MODE_TRANSITIONS)
    reference = motion_reference(
        motion,
        mode=mode,
        command=mode_command,
        command_codes=MOTION_COMMAND_CODES,
        # 只有 WBC 模式（mode 3）跟踪舞蹈参考；其他模式下参考停在起点，进入 WBC 时从头播放。
        active_mode=3,
        pause=350,
        frame_interval=5,
    )
    observations = {
        "motion_reference": reference,
        # Not base_angular_velocity(): that reads qvel[3:6]; this policy keeps the
        # pelvis' local-frame body velocity (derived from cvel/xmat, one substep behind qvel).
        "angular_velocity": robot.body_angular_velocity("pelvis", frame="local"),
        "gravity": robot.projected_gravity(),
        "velocity_command": robot.commands(
            **read_config("config/io.yaml")["velocity_command"]
        ),
        "joint_position": robot.joint_position(order="native"),
        "joint_velocity": robot.joint_velocity(order="native"),
        "last_action": robot.last_action(),
    }
    # Policy
    reference_angles = resource_config(robot.id, "referenceJointPositions")
    action = policy_bank(
        mode,
        sources=observations,
        policies=policies(
            *models,
            default=robot.default_angles,
            policy_order=robot.policy_order,
            reference=reference_angles,
        ),
    ).clip(-100, 100).action()

    # Output
    target, kp, kd = joint_control(robot, mode, action, reference_angles)
    return PolicyController(robot, target=target, kp=kp, kd=kd)
