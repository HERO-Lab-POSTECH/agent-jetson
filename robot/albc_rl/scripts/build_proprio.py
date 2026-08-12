"""ProprioBuilder: robot sensors -> 69D-policy proprioception block (the sim-to-real surface).

Concrete implementation of build_proprio() for the agent-jetson UUV board, for the
69D attitude-only policy (full lin-vel-tracking removal). Depends ONLY on numpy
(board has 1.11.0, Python 3.5) so the same file runs on the robot and is unit-tested
on a dev machine without ROS/torch.

WHAT IT PRODUCES (20D proprio)
------------------------------
Mirrors attitude_only/mdp/observations.py compute_policy_obs() order EXACTLY (verified
against the sim in marinegym-isaaclab @ ksm-ubuntu, 2026-06-08):

    [0:3]   ang_cmd [roll_att, pitch_att, yaw_rate]   <- setpoint (RL node owns)
    [3:6]   euler roll,pitch,yaw (rad)                <- /hero_agent/sensors (board-frame corrected)
    [6:9]   body angular velocity p,q,r (rad/s)       <- firmware gyro (rotate_gyro'd by node); d(euler)/dt+LPF fallback if no gyro key
    [9:11]  arm joint positions (rad, cumulative)     <- /albc/joint_states
    [11:13] arm joint velocities (rad/s)              <- driver value, or ESTIMATED d(joint)/dt + LPF
    [13]    manipulability index [0,1]                <- Yoshikawa from joint2
    [14:20] thruster filtered output T0..T5           <- previous action[2:8] echo

NO linear velocity anywhere -- not in the command, not in the body state. The 69D policy
removed lin-vel tracking entirely (att-only), so the DVL-absence problem that plagued the
87D deploy (zero placeholder at obs 12:15) is gone at the model level.

INTEGRAL IS NOT BUILT HERE (key difference from the 87D version)
---------------------------------------------------------------
The 87D build recomputed the leaky integral from proprio indices (measured_indices /
command_to_integral_order). The attitude_only sim does NOT: it carries `_error_integral`
as a STATE BUFFER updated each step (albc_env.py _get_rewards), not derived from proprio.
So this builder produces ONLY the 20D proprio; the 3D integral is owned by the policy
runtime (numpy_port np_policy), which must replicate the sim recurrence:

    err   = [roll_att_err, pitch_att_err, yaw_rate_err]   (cmd - measured)
            measured = euler[roll,pitch], ang_vel_b[2]
    gate  = |err| < sigma            sigma = [0.10, 0.10, 0.10]   (att_rp/yaw_vel reward sigma)
    I     = leak * I + gate * err * dt     leak = 0.99,  dt = 0.02
    I     = clamp(I, -2.0, +2.0)           integral_clamp = 2.0, integral_gated = True

These constants are from attitude_only/config.py (verified, not guessed) -- the board
numpy runtime must use them (the old 87D INTEGRAL_CLAMP=1.0 / no-gate was WRONG for 69D).
The 46D temporal history is likewise the runtime's responsibility.

WHAT IS ESTIMATED, NOT MEASURED (and why it still matches sim-to-real)
----------------------------------------------------------------------
The board has no gyro and no joint tachometer. Angular velocity and joint velocity are
derived by differentiating the measured angle and low-pass filtering (alpha=0.2) -- the
SAME estimator the existing board controller uses (attitude_controller.h). The sim has
PhysX ground-truth rates; we accept the noise-characteristic gap as part of sim-to-real.
"""
import numpy as np

# ---- frozen geometry / estimator constants (must match sim + board) ----
L_LINK = 0.233          # HERO_AGENT_ALBC_LINK1/2_LENGTH (l1 == l2)
CONTROL_DT = 0.02       # 50 Hz
LPF_ALPHA = 0.2         # board attitude_controller.h LPF on derived rates

PROPRIO_DIM = 20        # 69D attitude-only current proprioception

# ---- thruster first-order lag (obs echo 14:20 -- must match sim byte-for-byte) ----
# The sim feeds obs[14:20] the FILTERED thruster state, not the raw command:
# marinelab ThrusterModel.apply_dynamics() runs a first-order lag each env step
# (constrained-albc envs/main/albc_env.py:563 -> apply_dynamics(action[2:], physics_dt)).
# The board previously echoed the RAW action here (std 1.0, |z| up to 8.7 vs the sim
# normalizer) -> OOD input the policy never saw in training -> thruster saturation.
# We replicate the EXACT recurrence so the board reports what the policy trained on.
#
# Constants PINNED to the live sim (verified 2026-07-02, do NOT guess):
#   ALBCThrusterCfg.time_constant_up   = 0.1   (rising: target > state)
#   ALBCThrusterCfg.time_constant_down = 0.05  (falling)
#   filter dt = physics_dt = sim.dt = 0.005  (NOT step_dt=0.02 -- apply_dynamics is
#     called with self.physics_dt; IsaacLab physics_dt == cfg.sim.dt). Getting this
#     wrong by 4x detunes the lag and reintroduces partial OOD.
#   target = raw command clamped to [-1, 1]; recurrence: s += (dt/tau)*(target - s).
THR_TAU_UP = 0.1
THR_TAU_DOWN = 0.05
THR_FILTER_DT = 0.005


def _wrap(a):
    """Wrap angle(s) to (-pi, pi]. Used so yaw-rate across the +-pi seam is small."""
    return np.arctan2(np.sin(a), np.cos(a))


def rotate_imu(ROLL, PITCH, YAW, offset_rad):
    """Board-frame IMU correction. Transcription of imu_rotation.h rotateImu().

    /hero_agent/sensors is the RAW IMU frame -- every consumer corrects it on
    receive (albc_controller imuCallback, hero_agent StateMonitor::onSensor).
    The RL node runs without albc_controller, so it applies the same correction
    here before building proprio. Sign/operation conventions are PINNED to the
    C++ oracle (do NOT "fix" without intent):

        raw_roll  =  ROLL              (no sign change)
        raw_pitch = -(PITCH)           (PITCH is NEGATED -- pinned)
        c = cos(offset_rad), s = sin(offset_rad)
        out_roll  =  c*raw_roll + s*raw_pitch
        out_pitch = -s*raw_roll + c*raw_pitch
        out_yaw   =  YAW               (passed through, no rotation)

    offset_rad is the IMU mounting yaw offset in RADIANS (board default 102 deg,
    albc_controller.yaml imu_yaw_offset). It was 45 deg until 2026-08-11, then -78 deg
    until 2026-08-12. Both moves were the same error found twice: the 2026-08-11 solve
    fixed a 123 deg rotation but landed 180 deg away, because its raised-side labels
    came from tilt_azimuth.py's inverted high_side_deg column. The live dry test on
    2026-08-12 settles it at +102 (all four tilt directions). Only the VALUE moved;
    the sign conventions above are unchanged and test_build_proprio.py still pins
    them -- note +180 deg on this offset negates out_roll and out_pitch together and
    leaves out_yaw (and rotate_gyro's out_r) alone, which is why yaw tracking never
    saw any of this.
    """
    raw_roll = ROLL
    raw_pitch = -(PITCH)
    c = np.cos(offset_rad)
    s = np.sin(offset_rad)
    return np.array([
        c * raw_roll + s * raw_pitch,
        -s * raw_roll + c * raw_pitch,
        YAW,
    ], dtype=np.float32)


def rotate_gyro(GYRO_X, GYRO_Y, GYRO_Z, offset_rad):
    """Board-frame correction for the raw gyro vector (p,q,r), sensor frame.

    Applies the SAME z-rotation as rotate_imu() does to euler roll/pitch, so the
    angular-rate frame matches the corrected attitude frame and sim root_ang_vel_b:

        raw_p =  GYRO_X
        raw_q = -(GYRO_Y)          # negated, identical to rotate_imu's raw_pitch
        c = cos(offset_rad), s = sin(offset_rad)
        out_p =  c*raw_p + s*raw_q
        out_q = -s*raw_p + c*raw_q
        out_r =  GYRO_Z            # yaw rate: rotation-axis component, INVARIANT to
                                   #   the z-axis yaw offset -> passed through verbatim.

    out_r is the policy's only yaw-tracking signal (obs[8]); keeping it the raw gyro
    truth is the whole point of this change. Pinned to the rotate_imu oracle -- do NOT
    "fix" signs without intent.
    """
    raw_p = GYRO_X
    raw_q = -(GYRO_Y)
    c = np.cos(offset_rad)
    s = np.sin(offset_rad)
    return np.array([
        c * raw_p + s * raw_q,
        -s * raw_p + c * raw_q,
        GYRO_Z,
    ], dtype=np.float32)


def manipulability(theta2):
    """Yoshikawa manipulability index, normalized to [0,1].

    w = sqrt(|l1*l2*sin(theta2)|) / sqrt(l1*l2)   (sim albc_env manipulability)
    0 at singularity (arm straight, sin=0), 1 at theta2 = +-pi/2.
    """
    w = np.sqrt(abs(L_LINK * L_LINK * np.sin(theta2)))
    return float(w / np.sqrt(L_LINK * L_LINK))


class ProprioBuilder:
    """Stateful sensor->proprio mapper. Holds previous angles for rate estimation.

    Stateful (not a pure function) because angular/joint velocity need the previous
    sample to differentiate, and the LPF needs its previous output. reset() on re-arm.

    Produces the 20D proprio block only. The 3D integral and 46D history are owned by
    the policy runtime (see module docstring) -- this surface is purely sensor->proprio.
    """

    proprio_dim = PROPRIO_DIM

    def __init__(self):
        self.reset()

    # ------------------------------------------------------------------ lifecycle
    def reset(self):
        self._prev_euler = None         # (3,) last roll,pitch,yaw
        self._prev_jpos = None          # (2,) last joint angles
        self._angvel_lpf = np.zeros(3, dtype=np.float32)
        self._jvel_lpf = np.zeros(2, dtype=np.float32)
        self._last_action = np.zeros(8, dtype=np.float32)
        # thruster first-order-lag state (obs 14:20). Mirrors ThrusterModel._state,
        # which _setup_buffers zeros at construction/reset. Zero at re-arm too.
        self._thr_state = np.zeros(6, dtype=np.float32)

    def set_last_action(self, action8):
        """RL node calls this each step with the 8D action it just published.

        Thruster feedback (obs 14:20) = the 6 thruster channels of the last action.
        """
        self._last_action = np.asarray(action8, dtype=np.float32).reshape(8)

    # ------------------------------------------------------------------ main map
    def build(self, s):
        """Map a sensor dict to the 20D proprio vector. See module docstring for keys.

        Required keys: cmd_att(2), cmd_yawrate, euler(3), joint_pos(2).
        Optional: joint_vel(2) -- if present, used verbatim (driver-differentiated).
        Optional: thruster(6) -- if absent, echoes set_last_action()[2:8] (the
        runtime path; explicit injection is for bench tests / golden replays).
        """
        euler = np.asarray(s["euler"], dtype=np.float32).reshape(3)
        jpos = np.asarray(s["joint_pos"], dtype=np.float32).reshape(2)

        # angular velocity (obs 6:9): prefer the firmware raw gyro (sim root_ang_vel_b
        # truth, already rotate_gyro'd by the node). Fall back to euler-differentiation
        # only when no gyro is published (pre-gyro firmware / bench tests).
        if "gyro" in s:
            angvel = np.asarray(s["gyro"], dtype=np.float32).reshape(3)
            # keep euler-diff state continuous so a later fallback isn't discontinuous
            self._prev_euler = euler
        else:
            angvel = self._estimate_rate(euler, self._prev_euler, self._angvel_lpf, wrap=True)
            self._prev_euler = euler

        # joint velocity: prefer the driver-supplied value (differentiated at the true
        # 10 Hz read rate, no 10->50 Hz staircase). Fall back to self-differentiation
        # only when no driver is publishing it (bench tests / degraded mode).
        if "joint_vel" in s:
            jvel = np.asarray(s["joint_vel"], dtype=np.float32).reshape(2)
            self._prev_jpos = jpos  # keep prev in sync so a later fallback is continuous
        else:
            jvel = self._estimate_rate(jpos, self._prev_jpos, self._jvel_lpf, wrap=False)
            self._prev_jpos = jpos

        manip = manipulability(jpos[1])
        if "thruster" in s:
            # explicit injection (bench tests / golden replays): use verbatim, do NOT
            # run the lag -- the golden fixtures already carry the sim's filtered state.
            thr = np.asarray(s["thruster"], dtype=np.float32).reshape(6)
        else:
            # runtime: obs[14:20] is the FILTERED thruster state (sim parity), advanced
            # by one env step from the last published thruster command. Matches the sim's
            # apply_dynamics(action[2:], physics_dt) exactly (see _advance_thruster_lag).
            thr = self._advance_thruster_lag(self._last_action[2:8])

        cmd_att = np.asarray(s["cmd_att"], dtype=np.float32).reshape(2)
        cmd_yawrate = np.float32(s["cmd_yawrate"])

        out = np.concatenate([
            cmd_att,                             # 0:2  roll_att, pitch_att command
            np.array([cmd_yawrate], np.float32), # 2    yaw_rate command
            euler,                               # 3:6  euler roll,pitch,yaw
            angvel,                              # 6:9  body angular velocity p,q,r
            jpos,                                # 9:11 joint positions
            jvel,                                # 11:13 joint velocities
            np.array([manip], np.float32),       # 13   manipulability
            thr,                                 # 14:20 thruster echo
        ]).astype(np.float32)
        assert out.shape[0] == PROPRIO_DIM, (out.shape[0], PROPRIO_DIM)
        return out

    # ------------------------------------------------------------------ internals
    def _advance_thruster_lag(self, cmd6):
        """Advance the thruster first-order lag one env step; return the new state (6,).

        Byte-for-byte port of marinelab ThrusterModel.apply_dynamics(). The sim calls
        it once per env step with the raw thruster command and physics_dt, then reads
        _thruster.state into obs[14:20]. Recurrence (per channel):

            target = clip(cmd, -1, 1)
            tau    = tau_up  if target > state else tau_down     # direction-dependent
            state  = state + (dt/tau) * (target - state)

        State is mutated in place so the next step sees the advanced value. The board
        used to echo `cmd` raw here, which the policy never saw in training (the sim
        always filters) -> OOD -> thruster saturation. This makes the board report what
        the policy trained on.
        """
        target = np.clip(np.asarray(cmd6, dtype=np.float32).reshape(6), -1.0, 1.0)
        tau = np.where(target > self._thr_state, THR_TAU_UP, THR_TAU_DOWN)
        # keep the accumulator float32 so it byte-matches the sim's torch state -- np.where
        # yields float64 branch values, which would otherwise silently promote _thr_state.
        self._thr_state = (self._thr_state + (THR_FILTER_DT / tau) * (target - self._thr_state)).astype(np.float32)
        return self._thr_state.copy()

    def _estimate_rate(self, cur, prev, lpf_state, wrap):
        """d/dt + LPF(alpha). First call (prev is None) -> zero, no state update.

        lpf_state is updated IN PLACE so the next call sees the filtered history.
        """
        if prev is None:
            return np.zeros_like(cur)
        delta = _wrap(cur - prev) if wrap else (cur - prev)
        raw = delta / CONTROL_DT
        lpf_state[:] = (1.0 - LPF_ALPHA) * lpf_state + LPF_ALPHA * raw
        return lpf_state.copy()
