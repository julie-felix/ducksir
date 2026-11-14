from botlab import constant, mode_select, resource_config


MOTION_COMMAND_CODES = {
    "pauseConfigured": 3,
    "pauseCurrent": 4,
    "resume": 5,
}

MODE_TRANSITIONS = [
    {"from": "any", "command": 1, "to": 2},
    {"from": [2, 3, 4, 5], "command": 7, "to": 0},
    {"from": 2, "command": 6, "to": 5},
    {"from": [2, 4, 5], "command": 8, "to": 3},
    {"from": [4, 5], "command": 13, "to": 2},
    {"from": 3, "command": 6, "to": 4},
    {"from": "any", "command": 2, "to": 0},
]


def policies(wbc, loco, amp, mjamp, *, default, policy_order, reference):
    def state_fields(reference, indices=policy_order, include_command=True):
        fields = [
            "angular_velocity",
            "gravity",
        ]
        if include_command:
            fields.append("velocity_command")
        fields.extend([
            {
                "source": "joint_position",
                "kind": "mapped",
                "indices": indices,
                "default": reference,
            },
            {"source": "joint_velocity", "kind": "mapped", "indices": indices},
            {"source": "last_action", "kind": "previous-action"},
        ])
        return fields

    return [
        {
            "key": "wbc",
            "mode": 3,
            "reference": "motion_reference",
            "history_frames": 4,
            "fields": state_fields(reference, include_command=False),
            "model": wbc,
        },
        {
            "key": "loco",
            "mode": 2,
            "fields": state_fields(default),
            "model": loco,
        },
        {
            "key": "amp",
            "mode": 4,
            "history_frames": 4,
            "fields": state_fields(reference),
            "model": amp,
        },
        {
            "key": "mjamp",
            "mode": 5,
            "history_frames": 4,
            "fields": state_fields(reference, indices=list(range(29))),
            "model": mjamp,
        },
    ]


def joint_control(robot, mode, action, reference_angles):
    """Apply each policy's joint targets and physical PD calibration together."""
    size = robot.action_dimension()
    common_target = 0.25 * action[robot.motor_order]
    reference_target = constant(reference_angles, size=size)
    target = mode_select(mode, constant(0, size=size), {
        2: common_target + robot.default_angles,
        (3, 4): common_target + reference_target,
        5: constant(robot.action_scale, size=size) * action + reference_target,
    })
    kp = mode_select(mode, constant(0, size=size), {
        2: constant(resource_config(robot.id, "balanceStiffness"), size=size),
        (3, 4, 5): constant(robot.kp, size=size),
    })
    kd = mode_select(mode, constant(3, size=size), {
        2: constant(resource_config(robot.id, "balanceDamping"), size=size),
        (3, 4, 5): constant(robot.kd, size=size),
    }, passive=-1)
    return target, kp, kd
