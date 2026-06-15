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
    # After the first-tick seed, the target advances by delta_scale*action each
    # step. Use a zero-joint_pos obs so the seed equals nominal-equivalent [0,0]
    # and the per-step delta is the only change.
    pol.reset()
    proprio = np.zeros(20, dtype=np.float32)
    cmd = np.zeros(3, dtype=np.float32)
    pol.act(proprio, cmd)                      # tick 1: seeds target = joint_pos = [0,0]
    after_seed = pol.joint_target              # = [0,0] + delta*action1
    action2 = pol.act(proprio, cmd)            # tick 2: pure delta advance, no seed
    np.testing.assert_allclose(
        pol.joint_target, after_seed + DELTA_SCALE * action2[:2], atol=1e-6)


def test_joint_target_seeded_from_measured_joint_pos_on_first_act(pol):
    # The sim seeds _joint_pos_targets from the MEASURED joint angle at reset so
    # the history joint_pos_error starts at 0. The board reset() cannot see the
    # measurement yet, so the seed happens on the FIRST act() from proprio[9:11].
    # This is the fix for the field runaway: without it the target starts at
    # nominal [0, pi/2] while the measured arm is elsewhere (e.g. 4.294 rad),
    # feeding the policy a large false joint_pos_error every tick.
    pol.reset()
    measured = np.array([4.294, 4.24], dtype=np.float32)
    proprio = np.zeros(20, dtype=np.float32)
    proprio[9:11] = measured
    cmd = np.zeros(3, dtype=np.float32)
    pol.act(proprio, cmd)
    # after the first act, target = measured + delta*action1 (seed happened BEFORE
    # obs assembly, so the per-step delta is applied on top of the seeded value).
    # The seed itself must have moved the base from nominal to ~measured: the
    # target must be far closer to `measured` than to nominal [0, pi/2].
    assert np.linalg.norm(pol.joint_target - measured) < 0.3, (
        "first-act target %s not seeded near measured %s (still near nominal?)"
        % (pol.joint_target, measured))


@pytest.mark.xfail(reason="seed fix alone does NOT stop the runaway: with "
                          "joint_pos_error seeded to 0 the policy still emits a "
                          "saturated action (|a|~1.05) on tick 1 when all error "
                          "terms are ~0. Root cause still under investigation -- "
                          "the seed mismatch was a real but separate defect.",
                   strict=False)
def test_no_runaway_when_arm_fixed_and_attitude_level(pol):
    # The exact field condition: body roughly level (att err ~2deg), arm bolted
    # at 4.294 rad, thrusters off. With the seed fix the joint target must NOT
    # run away monotonically. Pre-fix it climbed 0.04 -> 5.48 over a few seconds.
    pol.reset()
    proprio = np.zeros(20, dtype=np.float32)
    proprio[3:6] = [-0.036, -0.02, 1.868]      # euler (roll,pitch ~0)
    proprio[6:9] = [-0.0019, -0.0008, -0.0019]  # ang vel ~0
    proprio[9:11] = [4.294, 4.24]               # measured joint (fixed)
    proprio[13] = 0.944                          # manipulability
    cmd = np.zeros(3, dtype=np.float32)
    targets = []
    for _ in range(60):
        pol.act(proprio, cmd)
        targets.append(pol.joint_target.copy())
    targets = np.array(targets)
    # joint1 target excursion from its seeded start must stay bounded (not the
    # multi-radian monotonic climb seen in the field). Allow modest motion.
    drift1 = np.max(np.abs(targets[:, 0] - targets[0, 0]))
    assert drift1 < 1.0, (
        "joint1 target drifted %.3f rad over 60 ticks -- still runaway "
        "(seed fix not effective)" % drift1)


if __name__ == "__main__":
    import sys
    sys.exit(pytest.main([__file__, "-v"]))
