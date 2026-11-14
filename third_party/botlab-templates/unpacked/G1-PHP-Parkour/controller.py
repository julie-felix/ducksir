from botlab import (
    Model,
    PolicyController,
    Robot,
    Sensor,
    constant,
    delay,
    one_hot,
    project,
    read_config,
    resize,
    rule_index,
)
from php_policy import (
    COMMAND_RULES,
    DEPTH_AFFINE_OFFSET,
    DEPTH_AFFINE_SCALE,
)

runtime = read_config("config/runtime.yaml")
project(
    runtime["name"],
    control_hz=runtime["control_hz"],
    physics_hz=runtime["physics_hz"],
    parameters=read_config("config/parameters.yaml"),
)

robot = Robot(
    "assets/scenes/g1_with_terrain.xml",
    preset="g1-php.preset.yaml",
    name="Robot",
)
depth_camera = Sensor(**read_config("config/observation.yaml")["depth_camera"])
depth_backbone = Model(
    "assets/models/php_parkour_depth_backbone.onnx",
    provider="wasm",
)
actor = Model(
    "assets/models/php_parkour_actor.onnx",
    provider="wasm",
)


def create_controller(sim):
    # --- observation ------------------------------------------------------
    command = robot.commands(**read_config("config/io.yaml")["command"])
    command_index = rule_index(command, COMMAND_RULES, default=0)

    depth_frame = depth_camera.depth_frame(max_age=7)
    depth = resize(
        depth_frame.data,
        input_size=(98, 58),
        output_size=(87, 58),
    )
    depth = DEPTH_AFFINE_SCALE * depth + DEPTH_AFFINE_OFFSET
    latent = depth_backbone(
        depth,
        input_name="depth_image",
        output_name="depth_latent",
    )
    latent = delay(
        latent,
        total_steps=7,
        age=depth_frame.age_steps,
        prime=True,
    )

    # The actor reserves eleven command classes followed by four zero placeholders.
    action = actor(
        robot.projected_gravity(target="torso_link"),
        robot.base_angular_velocity(),
        robot.joint_position(default=robot.default_angles),
        robot.joint_velocity(),
        robot.last_action(),
        one_hot(command_index, size=11),
        constant(0, size=4),
        latent,
        input_name="obs",
        output_name="actions",
        constants={"time_step": [0.0]},
    ).action()

    # --- output -----------------------------------------------------------
    native_delta = (robot.action_scale * action)[robot.motor_order]
    return PolicyController(robot, target=robot.default_angles + native_delta)
