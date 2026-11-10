"""最小项目脚本：测试与文档都用它当例子。

    bin/ducksir run sim/test/fixtures/project_demo.py --seconds 10
"""

import sys

sys.path.insert(0, "sim/sdk")

from ducksir import Project  # noqa: E402

project = (Project("duck-hd1910")
           .policy("walking", mode=0)
           .command(forward=0.25, yaw=0.0)
           .rates(control_hz=50, physics_hz=200)
           .seconds(10))
