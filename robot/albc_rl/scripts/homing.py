"""Pure decision helpers for the RL node's joint-homing startup phase.

No rospy, no numpy-version-specific calls -- runs identically on the dev Mac
and the board (py2.7 / numpy 1.11). The node imports these and keeps all ROS
side effects (publishing, logging, state transition) in rl_inference_node.py.

Convergence is judged in the MEASURED joint_pos frame (self._joint_pos, the
exact value the policy obs uses), so a sim/driver coordinate mismatch does not
matter: matching nominal in the measured frame puts obs at nominal.
"""
import numpy as np


def should_finish_homing(joint_pos, target, tol):
    """True when every measured joint is within +-tol of its nominal target.

    joint_pos, target: length-2 float32 arrays (rad). tol: scalar (rad).
    Boundary |err| == tol counts as converged. Arithmetic stays in float32 so
    boundary values are not inflated by float64 upcasting.

    The error is wrapped to [-pi, pi) before comparison. The driver
    (joint_angle_command.cpp updateMeasured) seeds each joint's cumulative
    measured angle from its first raw read, identically for both joints, so a
    physically-nominal arm can report a full-turn offset (e.g. joint1 ~6.286).
    Wrapping both joints makes a 2*pi offset read as ~0 (same atan2(sin, cos)
    rule as _wrap in build_proprio, so it matches the frame the policy obs
    uses), while a genuine error like 0.3 rad still survives the wrap. The
    elbow cannot physically reach target+2*pi, so wrapping joint2 only masks a
    measured-frame artifact, never a real out-of-range pose.

    An empty input returns False (not converged): np.all([]) is vacuously True,
    which would falsely start the policy, so guard against it explicitly.
    """
    raw = (np.asarray(joint_pos, dtype=np.float32)
           - np.asarray(target, dtype=np.float32))
    if raw.size == 0:
        return False
    wrapped = np.arctan2(np.sin(raw), np.cos(raw))
    return bool(np.all(np.abs(wrapped) <= tol))


def homing_timed_out(elapsed_s, timeout_s):
    """True when homing has run longer than timeout_s (strict >).

    On timeout the node HOLDS (no policy start) -- it does NOT force RUNNING,
    because starting the policy un-converged re-enters the OOD blowup.
    """
    return bool(elapsed_s > timeout_s)
