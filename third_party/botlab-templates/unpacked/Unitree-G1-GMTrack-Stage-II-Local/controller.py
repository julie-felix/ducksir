"""Unitree G1 GMTrack Stage-II on BotLab — observation assembly mirrors
GMTrack-Stage2-Flat-Unitree-G1 (1728-dim actor ABI):

- proprio_hist (640): [projected_gravity, base_ang_vel, joint_pos_rel,
  joint_vel_rel] x 10-frame history, flattened BY TERM (mjlab convention).
- action_hist (290): raw policy actions x 10, by term.
- command_window (798): 21 reference tokens at frame offsets -10..+10; each
  token is [v_ref(3), w_ref(3), g_ref(3), q_ref(29)] — velocities and gravity
  expressed in the reference root frame at the token's own timestamp. The G1
  flat/stage-II ABI has no heading term, so unlike CASBOT02 there is no
  relative-root-orientation 6D block.

The eval harness installs the frame-0 state and advances the command to frame 1
before the first policy call, so the reference leads the clock by one control
period (REFERENCE_LEAD_S).
"""

from botlab import (
    Model,
    Motion,
    PolicyController,
    Robot,
    clock,
    concat,
    constant,
    history,
    inverse_rotate,
    project,
    read_config,
)

runtime = read_config("config/runtime.yaml")
project(
    runtime["name"],
    control_hz=runtime["control_hz"],
    physics_hz=runtime["physics_hz"],
)

robot = Robot("assets/robot/scene.xml", preset="g1.preset.yaml", name="UnitreeG1")
actor = Model("assets/models/stage2_actor.onnx", name="Stage-II Actor")
motion = Motion("assets/motions/current.npz", name="Reference Motion")

CONTROL_DT = 0.02  # 50 Hz
REFERENCE_LEAD_S = CONTROL_DT
WINDOW_RADIUS = 10
JOINT_COUNT = 29
GRAVITY_W = constant([0.0, 0.0, -1.0])

JOINTS = list(range(JOINT_COUNT))


def reference_token(t):
    ref_quat = motion.sample(t, channel="body_quaternion", indices=[0])
    ref_lin = motion.sample(t, channel="body_linear_velocity", indices=[0])
    ref_ang = motion.sample(t, channel="body_angular_velocity", indices=[0])
    q_ref = motion.sample(t, channel="joint_position", indices=JOINTS)
    return concat([
        inverse_rotate(ref_lin, ref_quat),
        inverse_rotate(ref_ang, ref_quat),
        inverse_rotate(GRAVITY_W, ref_quat),
        q_ref,
    ])


def create_controller(sim):
    t_ref = clock() + REFERENCE_LEAD_S

    command_window = concat([
        reference_token(t_ref + offset * CONTROL_DT)
        for offset in range(-WINDOW_RADIUS, WINDOW_RADIUS + 1)
    ])

    proprio = history([
        robot.projected_gravity(),
        # Not base_angular_velocity(): that reads qvel[3:6], while this reads the
        # pelvis' local-frame body velocity (derived from cvel/xmat, which lag qvel
        # by one physics substep), the signal this policy was validated with.
        robot.body_angular_velocity("pelvis", frame="local"),
        robot.joint_position(default=robot.default_angles),
        robot.joint_velocity(),
    ], 10, layout="by-term")
    action_hist = history(robot.last_action(), 10, layout="by-term")

    obs = concat([proprio, action_hist, command_window])
    action = actor(obs).action()

    q_ref_now = motion.sample(t_ref, channel="joint_position", indices=JOINTS, preview=True)
    # Raw-control path: write position targets straight into the scene's native
    # position actuators, mirroring mjlab's loop (proc-pd's explicit substep
    # torques are measurably off with these gains under implicitfast).
    # Scale in policy joint order, then reorder once into motor (actuator) order.
    target = q_ref_now + constant(robot.action_scale, size=JOINT_COUNT) * action
    return PolicyController(robot, control=target[robot.motor_order])
