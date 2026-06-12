"""API guard: NumpyStudentPolicy exposes the accumulated joint PD target.

The ROS node must publish the ABSOLUTE joint target (the sim PD contract), not
the raw delta action -- the Dynamixel driver interprets the command topic as an
absolute angle (status_publisher.h historically published mapTo2Pi absolutes;
joint_angle_command.cpp updateJoint unwrap-follows the received value). The
policy already accumulates `_joint_target += DELTA_SCALE * action[:2]`
internally; `joint_target` is its public read-only view for the node.

Run on the dev Mac (pure numpy, no ROS, no torch):
    python3 -m pytest test_np_policy_api.py -v
"""
import os

import numpy as np
import pytest

from np_policy import NumpyStudentPolicy, NOMINAL_JOINT_POS, DELTA_SCALE

HERE = os.path.dirname(os.path.abspath(__file__))


@pytest.fixture(scope="module")
def pol():
    return NumpyStudentPolicy(
        os.path.join(HERE, "weights_tcn.npz"),
        os.path.join(HERE, "weights_teacher.npz"),
        "tcn",
    )


def test_joint_target_starts_at_nominal(pol):
    pol.reset()
    np.testing.assert_allclose(pol.joint_target, [0.0, np.pi / 2.0], atol=1e-6)


def test_joint_target_returns_copy(pol):
    # mutating the returned array must not corrupt the policy's internal state
    pol.reset()
    t = pol.joint_target
    t[:] = 99.0
    np.testing.assert_allclose(pol.joint_target, NOMINAL_JOINT_POS, atol=1e-6)


def test_joint_target_advances_by_delta_scale_after_act(pol):
    pol.reset()
    proprio = np.zeros(20, dtype=np.float32)
    cmd = np.zeros(3, dtype=np.float32)
    before = pol.joint_target
    action = pol.act(proprio, cmd)
    np.testing.assert_allclose(
        pol.joint_target, before + DELTA_SCALE * action[:2], atol=1e-6)


if __name__ == "__main__":
    import sys
    sys.exit(pytest.main([__file__, "-v"]))
