from botlab import Model, PolicyController, Robot, Scene, Sensor, history, project, read_config, waypoint_tracking


runtime = read_config("config/runtime.yaml")
project(
    runtime["name"],
    control_hz=runtime["control_hz"],
    physics_hz=runtime["physics_hz"],
    parameters=read_config("config/parameters.yaml"),
)

robot = Robot(
    "models/g1/instinctmj/scene_29dof_torsoBase_popsicle_with_shoe.xml",
    preset="g1-hiking-robot",
    name="Robot",
)
depth_camera = Sensor(**read_config("config/observation.yaml")["depth_camera"])
depth_encoder = Model(
    "assets/models/parkour_depth_encoder.onnx",
    name="Parkour Depth Enc",
)
actor = Model(
    "assets/models/parkour_actor.onnx",
    name="Parkour Actor",
)
scene = Scene(
    "procedural://stairs",
    preset="starter-parkour-stairs",
    name="Starter Parkour Stairs",
)


def create_controller(sim):
    # --- observation ------------------------------------------------------
    state = history([
        robot.base_angular_velocity(scale=0.25),
        robot.projected_gravity(),
        robot.commands(
            **read_config("config/io.yaml")["velocity_command"],
            waypoint=waypoint_tracking(min_forward_velocity=0.1),
        ),
        robot.joint_position(default=robot.default_angles),
        robot.joint_velocity(scale=0.05),
        robot.last_action(),
    ], 8, layout="by-term")

    depth = history(depth_camera.depth(max_age=5), 8, stride=5)

    # --- policy -----------------------------------------------------------
    latent = depth_encoder(depth)
    action = actor(state, latent).clip(-100, 100).action()

    # --- output -----------------------------------------------------------
    return PolicyController(
        robot,
        target=robot.default_angles + robot.action_scale * action[robot.motor_order],
    )
