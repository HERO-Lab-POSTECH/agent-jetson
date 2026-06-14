"""Numpy deployable policy: obs assembly + buffers + encoder + actor (board runtime).

Mirror of student_inference.py's DeployedStudentPolicy, but torch-free. Loads weights
from the .npz produced by export_golden.py. Runs on Jetson TX2 (Python 3.5, numpy 1.11).

The contract constants are duplicated here (frozen at training time) so this module is
self-contained on the robot -- it does NOT import student_inference.py (which needs torch).

69D attitude-only contract (extracted from the marinegym-isaaclab attitude_only sim):
  policy obs dim : 69  (20 proprio + 46 history + 3 integral)
  proprio 20D    : [0:3] ang_cmd, [3:6] euler(r,p,y), [6:9] ang_vel_b(p,q,r),
                   [9:11] joint_pos, [11:13] joint_vel, [13] manip, [14:20] thruster
  history 18D/step: [0:2] joint_pos_error (q_des_{t-1}-joint_pos), [2:4] joint_vel,
                   [4:6] att_rp_err (wrapped), [6] yaw_rate_err, [7:10] euler,
                   [10:18] prev_action (the previous act() output)
  obs history blk: jb_hist 10D x 3 steps (30D, all steps) + act_hist 8D x 2 newest (16D)
  integral 3D    : leaky, gated (|err|<sigma), clamp +-2.0, on [roll, pitch, yaw_rate]
"""
from collections import deque

import numpy as np

import npforward as npf


# ---- numpy version floor (board deploy = 1.11; dev = 2.x) ----------------------------
# numpy < 1.11.0 silently ignores the `keepdims` kwarg on reductions (a real bug that bit
# this port: a code-reviewer caught it). The board runs 1.11.0 exactly; anything older
# would compute wrong reductions WITHOUT raising. Fail loudly at import instead.
_NUMPY_MIN = (1, 11, 0)


def _numpy_version_tuple():
    parts = []
    for tok in np.__version__.split(".")[:3]:
        num = ""
        for ch in tok:
            if ch.isdigit():
                num += ch
            else:
                break
        parts.append(int(num) if num else 0)
    while len(parts) < 3:
        parts.append(0)
    return tuple(parts)


if _numpy_version_tuple() < _NUMPY_MIN:
    raise RuntimeError(
        "numpy %s < %s: reductions silently ignore keepdims below 1.11.0 "
        "(wrong results, no error). Upgrade numpy on this host."
        % (np.__version__, ".".join(map(str, _NUMPY_MIN)))
    )


# ---- frozen contract constants (must match student_inference.py / attitude_only sim) ----
POLICY_OBS_DIM = 69
PROPRIO_DIM = 20
LATENT_DIM = 9
ACTION_DIM = 8
CONTROL_DT = 0.02

HIST_LEN = 3
HIST_ACTION_LEN = 2
HIST_STRIDE = 3
HIST_FEAT_DIM = 18          # per-step history feature width
HIST_JB_DIM = 10            # joint+body slice [0:10] used for jb_hist
INTEGRAL_DIM = 3
INTEGRAL_LEAK = 0.99
INTEGRAL_CLAMP = 2.0
INTEGRAL_GATED = True
INTEGRAL_SIGMA = np.array([0.10, 0.10, 0.10], dtype=np.float32)  # [att_rp, att_rp, yaw_vel]
NOMINAL_JOINT_POS = np.array([0.0, np.pi / 2.0], dtype=np.float32)
DELTA_SCALE = 0.10
TCN_HISTORY = 9


def _wrap_angle(a):
    """Wrap to [-pi, pi] via atan2(sin, cos) (matches torch sim exactly)."""
    return np.arctan2(np.sin(a), np.cos(a))


class NumpyStudentPolicy:
    """Drop-in numpy replacement for DeployedStudentPolicy (69D attitude-only).

    Usage on the robot:
        pol = NumpyStudentPolicy("weights_tcn.npz", "weights_teacher.npz", "tcn")
        pol.reset()
        action = pol.act(proprio_20, cmd_3)   # every control step (50 Hz)
    """

    def __init__(self, student_npz, teacher_npz, encoder_type="tcn"):
        if encoder_type not in ("tcn", "gru"):
            raise ValueError(
                "encoder_type must be 'tcn' or 'gru', got %r" % (encoder_type,))
        self.encoder_type = encoder_type

        try:
            sw = np.load(student_npz)
        except (IOError, OSError) as e:
            # weights_gru.npz is not shipped for the TCN-only deploy. Make the
            # GRU path fail with an explicit message instead of a bare load error.
            raise RuntimeError(
                "failed to load student weights %r for encoder_type=%r: %s"
                % (student_npz, encoder_type, e))
        tw = np.load(teacher_npz)

        # fail-fast BEFORE building the encoder/actor: the frozen 69D/8D contract
        # must match the loaded weights. A stale npz (e.g. the old 87D policy)
        # would otherwise blow up inside npforward's constructor with an opaque
        # KeyError, or pass __init__ and fail later in act() with a shape mismatch.
        self._assert_contract(tw, sw, encoder_type)

        self.actor = npf.TeacherActor(tw)
        if encoder_type == "tcn":
            self.encoder = npf.StudentTCN(sw)
        else:
            self.encoder = npf.StudentGRU(sw)

        self._hist_buf = deque(maxlen=HIST_LEN)
        self._integral = np.zeros(INTEGRAL_DIM, dtype=np.float32)
        self._tcn_window = deque(maxlen=TCN_HISTORY)
        self._gru_hidden = None
        self._joint_target = NOMINAL_JOINT_POS.copy()
        self._prev_action = np.zeros(ACTION_DIM, dtype=np.float32)
        self._hist_step_counter = 0
        self.reset()

    @staticmethod
    def _assert_contract(tw, sw, encoder_type):
        """Validate the loaded .npz against the frozen 69D/8D/9D contract.

        Raises ValueError with a precise message if any key is missing or any
        dimension diverges from POLICY_OBS_DIM / ACTION_DIM / LATENT_DIM. This is
        the load-time half of the contract that the module constants freeze; the
        normal (correct-weights) path is unaffected.
        """
        def need(npz, key, name):
            if key not in npz.files:
                raise ValueError("%s weights missing key %r (keys=%s)"
                                 % (name, key, list(npz.files)))
            return npz[key]

        # teacher actor: normalizer is per-obs (1, 69); actor.0 takes obs+latent (78);
        # actor.6 emits the 8D action.
        mean = need(tw, "normalizer._mean", "teacher")
        if mean.shape[-1] != POLICY_OBS_DIM:
            raise ValueError("teacher normalizer obs dim %d != POLICY_OBS_DIM %d"
                             % (mean.shape[-1], POLICY_OBS_DIM))
        a0 = need(tw, "actor.0.weight", "teacher")
        if a0.shape[1] != POLICY_OBS_DIM + LATENT_DIM:
            raise ValueError(
                "teacher actor.0 input dim %d != obs+latent %d (obs %d + latent %d)"
                % (a0.shape[1], POLICY_OBS_DIM + LATENT_DIM, POLICY_OBS_DIM, LATENT_DIM))
        a6 = need(tw, "actor.6.weight", "teacher")
        if a6.shape[0] != ACTION_DIM:
            raise ValueError("teacher actor.6 output dim %d != ACTION_DIM %d"
                             % (a6.shape[0], ACTION_DIM))

        # student encoder: input width must be POLICY_OBS_DIM, latent head emits LATENT_DIM.
        if encoder_type == "tcn":
            ct = need(sw, "channel_transform.0.weight", "student TCN")
            in_dim = ct.shape[1]
        else:
            ih = need(sw, "gru.weight_ih_l0", "student GRU")
            in_dim = ih.shape[1]
        if in_dim != POLICY_OBS_DIM:
            raise ValueError("student %s input dim %d != POLICY_OBS_DIM %d"
                             % (encoder_type, in_dim, POLICY_OBS_DIM))
        h3 = need(sw, "head.3.weight", "student %s" % encoder_type)
        if h3.shape[0] != LATENT_DIM:
            raise ValueError("student %s latent dim %d != LATENT_DIM %d"
                             % (encoder_type, h3.shape[0], LATENT_DIM))

    def reset(self):
        self._hist_buf.clear()
        for _ in range(HIST_LEN):
            self._hist_buf.append(np.zeros(HIST_FEAT_DIM, dtype=np.float32))
        self._integral[:] = 0.0
        self._tcn_window.clear()
        self._gru_hidden = None
        self._joint_target = NOMINAL_JOINT_POS.copy()
        self._prev_action[:] = 0.0
        self._hist_step_counter = 0

    @property
    def joint_target(self):
        """Accumulated absolute joint PD target (rad), copy of the internal buffer.

        This is what the sim PD controller tracked (q_des). The ROS node publishes
        THIS (absolute angle) to the Dynamixel driver -- NOT the raw delta action:
        the driver's command topic contract is an absolute angle (updateJoint
        unwrap-follows the received value).
        """
        return self._joint_target.copy()

    def act(self, proprio_20, cmd_3):
        obs69 = self._assemble_obs(proprio_20, cmd_3)
        obs_b = obs69.reshape(1, -1).astype(np.float32)

        if self.encoder_type == "tcn":
            self._tcn_window.append(obs69.astype(np.float32))
            while len(self._tcn_window) < TCN_HISTORY:
                self._tcn_window.appendleft(self._tcn_window[0])
            win = np.stack(list(self._tcn_window)).reshape(1, TCN_HISTORY, POLICY_OBS_DIM)
            z = self.encoder.forward(win)               # (1, 9)
        else:
            if self._gru_hidden is None:
                self._gru_hidden = self.encoder.init_hidden(1)
            z, self._gru_hidden = self.encoder.step(obs_b, self._gru_hidden)  # (1,9)

        obs_n = self.actor.normalize(obs_b)
        action = self.actor.act(obs_n, z).reshape(-1)   # (8,)
        action = np.clip(action, -1.0, 1.0).astype(np.float32)

        # update buffers for the NEXT step (order mirrors the sim step):
        #   1) accumulate joint PD target with this step's arm action
        #   2) remember this action as prev_action for the next history feature
        self._joint_target = self._joint_target + DELTA_SCALE * action[:2]
        self._prev_action = action.copy()
        return action

    def _assemble_obs(self, proprio_20, cmd_3):
        assert proprio_20.shape == (PROPRIO_DIM,)
        assert cmd_3.shape == (INTEGRAL_DIM,)
        proprio_20 = proprio_20.astype(np.float32)

        euler = proprio_20[3:6]                 # roll, pitch, yaw
        ang_vel_b = proprio_20[6:9]             # p, q, r
        joint_pos = proprio_20[9:11]
        joint_vel = proprio_20[11:13]

        # --- history feature (18D), computed with q_des_{t-1} (pre-PD-update target) ---
        joint_pos_error = self._joint_target - joint_pos                  # [0:2]
        att_rp_err = _wrap_angle(cmd_3[:2] - euler[:2])                   # [4:6]
        yaw_rate_err = cmd_3[2] - ang_vel_b[2]                            # [6]
        feat = np.concatenate([
            joint_pos_error,                     # 2
            joint_vel,                           # 2
            att_rp_err,                          # 2
            np.array([yaw_rate_err], dtype=np.float32),  # 1
            euler,                               # 3
            self._prev_action,                   # 8
        ]).astype(np.float32)                    # = 18

        # --- stride ring buffer: record every HIST_STRIDE-th step ---
        self._hist_step_counter += 1
        if self._hist_step_counter % HIST_STRIDE == 0:
            self._hist_buf.append(feat)

        # --- leaky gated integral on [roll_err, pitch_err, yaw_rate_err] ---
        err = np.concatenate([att_rp_err, np.array([yaw_rate_err], dtype=np.float32)])
        self._integral = INTEGRAL_LEAK * self._integral
        if INTEGRAL_GATED:
            gate = (np.abs(err) < INTEGRAL_SIGMA).astype(np.float32)
            self._integral = self._integral + gate * err * CONTROL_DT
        else:
            self._integral = self._integral + err * CONTROL_DT
        np.clip(self._integral, -INTEGRAL_CLAMP, INTEGRAL_CLAMP, out=self._integral)

        # --- obs assembly: proprio(20) + jb_hist(30) + act_hist(16) + integral(3) ---
        buf = list(self._hist_buf)
        jb_hist = np.concatenate([f[:HIST_JB_DIM] for f in buf])               # 10 x 3 = 30
        act_hist = np.concatenate([f[HIST_JB_DIM:] for f in buf[-HIST_ACTION_LEN:]])  # 8 x 2 = 16
        obs = np.concatenate([proprio_20, jb_hist, act_hist, self._integral])
        assert obs.shape[0] == POLICY_OBS_DIM, obs.shape[0]
        return obs.astype(np.float32)
