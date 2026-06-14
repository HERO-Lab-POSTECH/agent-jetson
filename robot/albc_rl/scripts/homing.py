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
    """
    err = np.abs(np.asarray(joint_pos, dtype=np.float32)
                 - np.asarray(target, dtype=np.float32))
    return bool(np.all(err <= tol))


def homing_timed_out(elapsed_s, timeout_s):
    """True when homing has run longer than timeout_s (strict >).

    On timeout the node HOLDS (no policy start) -- it does NOT force RUNNING,
    because starting the policy un-converged re-enters the OOD blowup.
    """
    return bool(elapsed_s > timeout_s)
