COMMAND_EPSILON = 1e-4

# Camera depth is normalized to [0, 1]. The published policy consumes
# ((depth * 3.0) - 0.3) / (3.0 - 0.3) - 0.5.
DEPTH_AFFINE_SCALE = 1.1111111111111112
DEPTH_AFFINE_OFFSET = -0.6111111111111112

def _compare(index, op, value):
    return {"index": index, "op": op, "value": value}


def _all(*conditions):
    return {"all": list(conditions)}


def _any(*conditions):
    return {"any": list(conditions)}


FORWARD = _compare(0, "gt", COMMAND_EPSILON)
LEFT = _compare(1, "gt", COMMAND_EPSILON)
RIGHT = _compare(1, "lt", -COMMAND_EPSILON)
TURN_LEFT = _compare(2, "gt", COMMAND_EPSILON)
TURN_RIGHT = _compare(2, "lt", -COMMAND_EPSILON)
HIGH_SPEED = _compare(3, "gte", 0.5)

LEFT_SKILL = _any(TURN_LEFT, _all(FORWARD, LEFT))
RIGHT_SKILL = _any(TURN_RIGHT, _all(FORWARD, RIGHT))

# Rules are first-match. This preserves the published controller's yaw >
# forward > lateral precedence while adding five to every active HIGH skill.
COMMAND_RULES = [
    {"value": 7, "when": _all(HIGH_SPEED, LEFT_SKILL)},
    {"value": 9, "when": _all(HIGH_SPEED, RIGHT_SKILL)},
    {"value": 6, "when": _all(HIGH_SPEED, FORWARD)},
    {"value": 8, "when": _all(HIGH_SPEED, LEFT)},
    {"value": 10, "when": _all(HIGH_SPEED, RIGHT)},
    {"value": 2, "when": LEFT_SKILL},
    {"value": 4, "when": RIGHT_SKILL},
    {"value": 1, "when": FORWARD},
    {"value": 3, "when": LEFT},
    {"value": 5, "when": RIGHT},
]
