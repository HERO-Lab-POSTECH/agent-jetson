"""Unit tests for the pure homing-decision helpers (homing.py).

These are deliberately rospy-free so they run on the dev Mac AND the board
(py2.7 / numpy 1.11). They lock down the HOMING->RUNNING transition rule and
the timeout-means-HOLD rule before the node wires them in.
"""
import numpy as np

from homing import should_finish_homing, homing_timed_out

TARGET = np.array([0.0, 1.5708], dtype=np.float32)
TOL = 0.05


def test_converged_within_tol_finishes():
    jpos = np.array([0.01, 1.56], dtype=np.float32)
    assert should_finish_homing(jpos, TARGET, TOL) is True


def test_one_joint_outside_tol_not_finished():
    jpos = np.array([0.0, 3.643], dtype=np.float32)
    assert should_finish_homing(jpos, TARGET, TOL) is False


def test_both_joints_outside_tol_not_finished():
    jpos = np.array([-0.778, 3.643], dtype=np.float32)
    assert should_finish_homing(jpos, TARGET, TOL) is False


def test_exactly_on_tol_boundary_finishes():
    jpos = np.array([0.05, 1.5708 - 0.05], dtype=np.float32)
    assert should_finish_homing(jpos, TARGET, TOL) is True


def test_timeout_not_elapsed():
    assert homing_timed_out(3.0, 8.0) is False


def test_timeout_elapsed():
    assert homing_timed_out(8.5, 8.0) is True


def test_timeout_exact_boundary_not_yet():
    assert homing_timed_out(8.0, 8.0) is False
