"""TDD spec for ProprioBuilder: sensors -> 20D proprio block (69D attitude-only policy).

This is the ONE sim-to-real surface. Every axis/sign/frame/order here is checked against
the sim reference compute_policy_obs() (attitude_only/mdp/observations.py @ marinegym-isaaclab)
verified 2026-06-08.

Two things are tested differently:
  * Things that MUST match the sim numerically (20D order, manipulability formula,
    thruster echo, command passthrough) -> exact golden asserts.
  * Things the board ESTIMATES because it lacks the sensor (angular velocity and
    joint velocity = numerical differentiation + LPF) -> behavioural asserts
    (first step = 0, then matches a hand-computed diff/LPF recurrence). The sim has
    ground-truth p,q,r so there is no golden vector to match; we instead pin the
    exact estimator the board uses (attitude_controller.h: d/dt + LPF alpha=0.2).

The 3D integral and 46D history are NOT built here -- they are the policy runtime's
responsibility (np_policy). build_proprio is purely sensor->20D-proprio.

Run on this Mac (pure numpy, no ROS, no torch):
    python3 -m pytest test_build_proprio.py -v
    # or: python3 test_build_proprio.py
"""
import numpy as np
import pytest

from build_proprio import ProprioBuilder, manipulability, L_LINK, PROPRIO_DIM

ATOL = 1e-6
DT = 0.02            # 50 Hz, CONTROL_DT
LPF_ALPHA = 0.2      # board attitude_controller.h LPF on derived rates


# --------------------------------------------------------------- manipulability
def test_manipulability_singularity_at_zero():
    # sin(0)=0 -> w=0 (fully singular, arm straight)
    assert manipulability(0.0) == pytest.approx(0.0, abs=ATOL)


def test_manipulability_max_at_ninety_deg():
    # w = sqrt(|l1 l2 sin th2|)/sqrt(l1 l2); at th2=pi/2 sin=1 -> w=1
    assert manipulability(np.pi / 2) == pytest.approx(1.0, abs=ATOL)


def test_manipulability_matches_yoshikawa_formula():
    # exact sim formula: w = sqrt(|l1*l2*sin(th2)|)/sqrt(l1*l2), l1=l2=0.233
    th2 = 0.7
    expected = np.sqrt(abs(L_LINK * L_LINK * np.sin(th2))) / np.sqrt(L_LINK * L_LINK)
    assert manipulability(th2) == pytest.approx(expected, abs=ATOL)


def test_manipulability_clamped_to_unit_range():
    # any theta -> [0,1]
    for th in np.linspace(-2 * np.pi, 2 * np.pi, 50):
        w = manipulability(th)
        assert -ATOL <= w <= 1.0 + ATOL


# ------------------------------------------------------------- 20D layout / order
def _sensors(**over):
    """A full sensor dict; override individual fields per test."""
    s = dict(
        cmd_att=(0.0, 0.0),          # roll_att_cmd, pitch_att_cmd
        cmd_yawrate=0.0,
        euler=(0.1, 0.2, 0.3),       # roll, pitch, yaw (already board-frame corrected)
        joint_pos=(0.5, 0.7),
        thruster=(1.0, 2.0, 3.0, 4.0, 5.0, 6.0),
    )
    s.update(over)
    return s


def test_output_is_20d_float32():
    b = ProprioBuilder()
    out = b.build(_sensors())
    assert out.shape == (PROPRIO_DIM,) == (20,)
    assert out.dtype == np.float32


def test_command_block_passthrough_0_3():
    # [0:2] roll/pitch attitude command, [2] yaw-rate command. No lin-vel command.
    b = ProprioBuilder()
    out = b.build(_sensors(cmd_att=(0.44, 0.55), cmd_yawrate=0.66))
    np.testing.assert_allclose(out[0:2], [0.44, 0.55], atol=ATOL)
    assert out[2] == pytest.approx(0.66, abs=ATOL)


def test_euler_block_3_6():
    b = ProprioBuilder()
    out = b.build(_sensors(euler=(0.1, -0.2, 1.5)))
    np.testing.assert_allclose(out[3:6], [0.1, -0.2, 1.5], atol=ATOL)


def test_joint_pos_block_9_11():
    b = ProprioBuilder()
    out = b.build(_sensors(joint_pos=(0.5, 0.7)))
    np.testing.assert_allclose(out[9:11], [0.5, 0.7], atol=ATOL)


def test_manipulability_block_13_from_joint2():
    b = ProprioBuilder()
    out = b.build(_sensors(joint_pos=(0.0, np.pi / 2)))
    assert out[13] == pytest.approx(1.0, abs=ATOL)   # th2=pi/2 -> w=1


def test_thruster_block_14_20_echo():
    b = ProprioBuilder()
    out = b.build(_sensors(thruster=(1, 2, 3, 4, 5, 6)))
    np.testing.assert_allclose(out[14:20], [1, 2, 3, 4, 5, 6], atol=ATOL)


# ----------------------------------------------- angular velocity = d/dt + LPF
def test_angvel_zero_on_first_step():
    # no previous euler -> rate 0 (cold start, like board init)
    b = ProprioBuilder()
    out = b.build(_sensors(euler=(0.1, 0.2, 0.3)))
    np.testing.assert_allclose(out[6:9], [0.0, 0.0, 0.0], atol=ATOL)


def test_angvel_second_step_is_lpf_of_diff():
    b = ProprioBuilder()
    b.build(_sensors(euler=(0.0, 0.0, 0.0)))
    out = b.build(_sensors(euler=(0.02, 0.04, 0.06)))
    # raw rate = d_euler/dt; LPF: y = (1-a)*y_prev + a*raw, y_prev=0
    raw = np.array([0.02, 0.04, 0.06]) / DT
    expected = LPF_ALPHA * raw                       # y_prev was 0
    np.testing.assert_allclose(out[6:9], expected, atol=ATOL)


def test_angvel_yaw_wraps_pi():
    # yaw jump from +3.13 to -3.13 must be a small rate, not a huge one
    b = ProprioBuilder()
    b.build(_sensors(euler=(0.0, 0.0, 3.13)))
    out = b.build(_sensors(euler=(0.0, 0.0, -3.13)))
    wrapped = np.arctan2(np.sin(-3.13 - 3.13), np.cos(-3.13 - 3.13))
    expected_r = LPF_ALPHA * (wrapped / DT)
    assert out[8] == pytest.approx(expected_r, abs=1e-4)   # yaw rate at [8]


# ----------------------------------------------- joint velocity = d/dt + LPF
def test_jointvel_zero_on_first_step():
    b = ProprioBuilder()
    out = b.build(_sensors(joint_pos=(0.5, 0.7)))
    np.testing.assert_allclose(out[11:13], [0.0, 0.0], atol=ATOL)


def test_jointvel_second_step_is_lpf_of_diff():
    b = ProprioBuilder()
    b.build(_sensors(joint_pos=(0.0, 0.0)))
    out = b.build(_sensors(joint_pos=(0.01, 0.02)))
    raw = np.array([0.01, 0.02]) / DT
    expected = LPF_ALPHA * raw
    np.testing.assert_allclose(out[11:13], expected, atol=ATOL)


def test_jointvel_uses_supplied_value_when_present():
    # The driver differentiates at its true 10 Hz read rate and ships joint_vel.
    # When sensors carry joint_vel, the builder MUST use it verbatim (no re-diff),
    # avoiding the 10->50 Hz staircase. First step too: supplied value wins.
    b = ProprioBuilder()
    out = b.build(_sensors(joint_pos=(0.5, 0.7), joint_vel=(1.23, -4.56)))
    np.testing.assert_allclose(out[11:13], [1.23, -4.56], atol=ATOL)


def test_jointvel_falls_back_to_diff_without_supplied():
    # No joint_vel key -> builder differentiates (works for bench tests / no driver).
    b = ProprioBuilder()
    b.build(_sensors(joint_pos=(0.0, 0.0)))            # no joint_vel
    out = b.build(_sensors(joint_pos=(0.01, 0.02)))   # no joint_vel
    expected = LPF_ALPHA * (np.array([0.01, 0.02]) / DT)
    np.testing.assert_allclose(out[11:13], expected, atol=ATOL)


# ----------------------------------------------- no linear velocity anywhere (69D)
def test_no_linvel_in_proprio():
    # 69D attitude-only: there is NO lin-vel command and NO measured lin-vel block.
    # proprio is exactly 20D, ending at the thruster echo -- nothing past index 20.
    b = ProprioBuilder()
    out = b.build(_sensors())
    assert out.shape == (20,)
    # body state is euler(3)+angvel(3) only; arm immediately follows at index 9.
    np.testing.assert_allclose(out[9:11], [0.5, 0.7], atol=ATOL)


# ----------------------------------------------- IMU board-frame correction
# Oracle: albc_control/include/albc_control/imu_rotation.h (byte-identical
# transcription of albc_controller.cpp imuCallback). The RL field test runs
# WITHOUT albc_controller, and /hero_agent/sensors is the RAW IMU frame (both
# C++ consumers correct it themselves on receive) -- so the RL node must apply
# the same correction before building proprio, or the policy sees a frame it
# was never trained on.
def test_rotate_imu_zero_offset_negates_pitch():
    from build_proprio import rotate_imu
    # offset 0: roll passes, PITCH is NEGATED (pinned sign), yaw passes.
    out = rotate_imu(0.1, 0.2, 0.3, 0.0)
    np.testing.assert_allclose(out, [0.1, -0.2, 0.3], atol=ATOL)


def test_rotate_imu_yaw_passthrough_any_offset():
    from build_proprio import rotate_imu
    out = rotate_imu(0.5, -0.4, 1.234, np.deg2rad(45.0))
    assert out[2] == pytest.approx(1.234, abs=ATOL)


def test_rotate_imu_45deg_pinned_golden():
    from build_proprio import rotate_imu
    # hand-computed against imu_rotation.h: raw=(0.1, -0.2), c=s=sqrt(2)/2
    #   roll  = ( 0.1 - 0.2) * 0.70710678 = -0.0707106781
    #   pitch = (-0.1 - 0.2) * 0.70710678 = -0.2121320344
    out = rotate_imu(0.1, 0.2, 0.3, np.pi / 4.0)
    np.testing.assert_allclose(
        out, [-0.0707106781, -0.2121320344, 0.3], atol=1e-6)


def test_rotate_imu_matches_cpp_reference_sweep():
    from build_proprio import rotate_imu

    def ref(ROLL, PITCH, YAW, off):
        # independent transcription of imu_rotation.h rotateImu()
        raw_roll = ROLL
        raw_pitch = -(PITCH)
        c, s = np.cos(off), np.sin(off)
        return (c * raw_roll + s * raw_pitch,
                -s * raw_roll + c * raw_pitch,
                YAW)

    rng = np.random.RandomState(7)
    for _ in range(50):
        r, p, y = rng.uniform(-np.pi, np.pi, 3)
        off = rng.uniform(-np.pi, np.pi)
        np.testing.assert_allclose(
            rotate_imu(r, p, y, off), ref(r, p, y, off), atol=1e-6)


# ----------------------------------------------- thruster echo: builder-owned
# Contract (node docstring + set_last_action docstring): obs[14:20] = the 6
# thruster channels of the LAST action. The node must not have to thread the
# echo through the sensor dict every tick -- when 'thruster' is absent, the
# builder echoes set_last_action() itself (closing the node<->builder seam
# where the audit found obs[14:20] permanently stuck at zero).
def test_thruster_echo_falls_back_to_last_action():
    b = ProprioBuilder()
    b.set_last_action([0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8])
    s = _sensors()
    s.pop("thruster")
    out = b.build(s)
    np.testing.assert_allclose(out[14:20], [0.3, 0.4, 0.5, 0.6, 0.7, 0.8], atol=ATOL)


def test_thruster_echo_zeros_before_first_action():
    # cold start: no action published yet -> echo must be zeros (sim reset state)
    b = ProprioBuilder()
    s = _sensors()
    s.pop("thruster")
    out = b.build(s)
    np.testing.assert_allclose(out[14:20], np.zeros(6), atol=ATOL)


def test_explicit_thruster_key_overrides_echo():
    # bench tests / golden replays inject thruster explicitly -- that path wins
    b = ProprioBuilder()
    b.set_last_action([0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8])
    out = b.build(_sensors(thruster=(9, 9, 9, 9, 9, 9)))
    np.testing.assert_allclose(out[14:20], [9, 9, 9, 9, 9, 9], atol=ATOL)


# ----------------------------------------------- reset clears estimator state
def test_reset_clears_rate_estimators():
    b = ProprioBuilder()
    b.build(_sensors(euler=(0.0, 0.0, 0.0)))
    b.build(_sensors(euler=(0.5, 0.5, 0.5)))
    b.reset()
    out = b.build(_sensors(euler=(0.1, 0.2, 0.3)))
    np.testing.assert_allclose(out[6:9], [0.0, 0.0, 0.0], atol=ATOL)  # cold again


if __name__ == "__main__":
    import sys
    sys.exit(pytest.main([__file__, "-v"]))
