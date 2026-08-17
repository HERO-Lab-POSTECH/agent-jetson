#!/usr/bin/env python
"""ROS node: real-time student-policy inference for the agent-jetson UUV (50 Hz).

Subscribes to the board sensors, applies the board-frame IMU correction
(build_proprio.rotate_imu, oracle: albc_control imu_rotation.h), assembles the
20D proprio block (build_proprio.py) for the 69D attitude-only policy, runs the
torch-free numpy policy (../numpy_port/np_policy.py), and publishes the arm
joint TARGET plus the 6 thruster channels. Pure numpy + rospy -- no torch on
the board (ROS lunar rosrun => Python 2.7.12, numpy 1.11).

DATA FLOW (one 50 Hz tick)
--------------------------
    /hero_agent/sensors (RAW imu frame) -> rotate_imu ->| ProprioBuilder.build()
    /albc/joint_states  (pos+vel, 10 Hz driver)       ->|  -> 20D proprio
    [att cmd / yaw-rate cmd] setpoints                ->|  + 3D command
                                                         v
                     NumpyStudentPolicy.act(proprio_20, command_3) -> action(8)
                                                         v
    policy.joint_target (ABSOLUTE rad)  -> /hero_agent/active_joint{1,2}_position_controller/command
    action[2:8] * thruster_scale        -> /albc/thruster_cmd   [wire to your thruster mixer]

JOINT COMMAND CONTRACT (never publish the raw action)
-----------------------------------------------------
The driver (joint_angle_command.cpp updateJoint) interprets the command topic
as an ABSOLUTE angle and unwrap-follows it (the legacy publisher on this topic,
status_publisher.h, sent mapTo2Pi absolutes). The policy's arm action is a
DELTA that the sim PD accumulated into q_des -- np_policy carries that
accumulator (_joint_target, DELTA_SCALE=0.10, nominal [0, pi/2]) and this node
publishes policy.joint_target, NOT action[0:2].

WHAT THIS NODE OWNS vs DELEGATES
--------------------------------
  * IMU board-frame correction ............... build_proprio.rotate_imu (on receive;
        /hero_agent/sensors is RAW -- every consumer corrects it itself, and the
        usual corrector albc_controller is NOT running in the RL configuration)
  * proprio 20D layout, frames, signs ........ build_proprio.ProprioBuilder
  * joint velocity ........................... driver JointState.velocity verbatim
        (driver differentiates at its true 10 Hz; re-diffing here would staircase)
  * thruster echo (obs 14:20) ................ builder echoes set_last_action()
  * history(20:66) + integral(66:69)
    + bias_ema(69:72) ........................ NumpyStudentPolicy (state buffers)
  * command setpoint (obs 0:3) ............... this node (_command, via /rl/command)
  * staleness / non-finite gates ............. this node (_tick gates; holds = no publish)

The 3D integral is the policy runtime's responsibility (leak=0.99, clamp=2.0,
gated, sigma=0.10, mirroring attitude_only/albc_env.py). The 3D bias_ema
(alpha=0.99, ungated) is likewise the runtime's. This node passes the 3D command
straight through; it does NOT reorder or recompute either buffer.

ROSPARAMS (all defaults are field-safe)
---------------------------------------
  ~encoder_type       : "tcn" | "gru"             (default gru)
  ~weights_dir        : dir with weights_*.npz    (default ../numpy_port)
  ~control_hz         : 50
  ~use_board_rates    : false  (true => trust /albc_status angular vel [8:11];
                                that topic is std_msgs/Float64MultiArray)
  ~imu_yaw_offset_deg : 102.0  IMU mounting yaw offset (albc_controller.yaml).
                                CONFIRMED 2026-08-12. Was -78.0 (2026-08-11),
                                itself 180 deg out; before that 45.0.
  ~sensor_timeout_s   : 0.2    IMU staleness gate: hold (no publish) + warn
  ~joint_timeout_s    : 0.5    /albc/joint_states staleness gate (driver = 10 Hz)
  ~joint1_start_max_rad : pi   refuse to start if |joint1| exceeds this. <= 0
                               disables. See _gate_start_pose.
  ~home_on_start      : false  FAIL-SAFE default. Homing commands the arm and
                               cannot unwind it, so it stays opt-in until the
                               restart round-trip regression test passes.
  ~thruster_scale     : 0.0    gain on the 6 thruster channels. FAIL-SAFE default:
                               0.0 = surface test (thrusters held at 0). Live
                               thrusters must be requested EXPLICITLY (1.0). The
                               arm action is never scaled. Secondary safety only --
                               the board has no /albc/thruster_cmd subscriber yet.
  ~thruster_max_s     : 0.0    if > 0, thrusters live for this many seconds after
                               the first tick, then latch to 0 (joints keep going).

NOT RUNNABLE ON A DEV MAC (no ROS). Build + run on the board.
"""
import hashlib
import os
import sys
import time
import traceback

import numpy as np
import rospy
from std_msgs.msg import Float64, Float32MultiArray, Float64MultiArray
from sensor_msgs.msg import JointState

from hero_msgs.msg import hero_agent_sensor

# local modules (numpy_port is a sibling of ros_node/)
_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _HERE)                                   # build_proprio
sys.path.insert(0, os.path.join(_HERE, "..", "numpy_port")) # np_policy, npforward

from build_proprio import ProprioBuilder, rotate_imu, rotate_gyro  # noqa: E402
from np_policy import NumpyStudentPolicy, DELTA_SCALE  # noqa: E402
from dynamic_reconfigure.server import Server  # noqa: E402
from albc_rl.cfg import GyroOffsetConfig  # noqa: E402


def _md5_8(path):
    """First 8 hex chars of a file md5 -- printed at startup so the operator can
    confirm WHICH weights are loaded (deploy packs swap the .npz files)."""
    h = hashlib.md5()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()[:8]


class RLInferenceNode(object):
    def __init__(self):
        # gru: the shipped 72D pack is GRU (pack_inc9998_gru). The 69D TCN pack in
        # numpy_port/ predates the bias_ema obs and would fail the contract check.
        self.encoder_type = rospy.get_param("~encoder_type", "gru")
        weights_dir = rospy.get_param(
            "~weights_dir", os.path.join(_HERE, "..", "numpy_port"))
        self.hz = float(rospy.get_param("~control_hz", 50.0))
        self.use_board_rates = rospy.get_param("~use_board_rates", False)
        self.imu_yaw_offset = float(np.deg2rad(
            float(rospy.get_param("~imu_yaw_offset_deg", 102.0))))
        self._dyn_srv = Server(GyroOffsetConfig, self._on_reconfigure)
        self.sensor_timeout = float(rospy.get_param("~sensor_timeout_s", 0.2))
        self.joint_timeout = float(rospy.get_param("~joint_timeout_s", 0.5))
        # surface-test safety on the 6 thruster channels (arm action never scaled).
        # FAIL-SAFE: code default is 0.0 -- live thrusters are an explicit opt-in.
        self.thruster_scale = float(rospy.get_param("~thruster_scale", 0.0))
        self.thruster_max_s = float(rospy.get_param("~thruster_max_s", 0.0))
        # No homing / HOLDING phase: the policy runs from the first valid tick.
        # joint1 multi-turn excursions are NOT bounded here, and no longer bounded
        # in the policy runtime either (the np_policy +-6*pi clamp was removed
        # 2026-08-17). The cable ceiling is a DRIVER-layer abort --
        # joint_angle_command ~joint1_abort_rad, default 6*pi = 3 turns -- because
        # a policy-layer rail never reaches the hardware: on 2026-08-13 the arm
        # was driven to -35.54 rad while the policy's own targets stayed inside
        # +-18.85. Excursions past the 4*pi training constraint are COUNTED on
        # /albc/joint_guard rather than clipped, so the number stays comparable
        # across TDC, classic PID and RL.
        self._first_tick_t = None   # set on the first published tick (thruster_max_s)

        student_npz = os.path.join(weights_dir, "weights_%s.npz" % self.encoder_type)
        teacher_npz = os.path.join(weights_dir, "weights_teacher.npz")
        self.policy = NumpyStudentPolicy(student_npz, teacher_npz, self.encoder_type)

        # Per-TICK joint-target gain. Exposed because it and control_hz both scale
        # the per-SECOND wind rate, so changing control_hz alone cannot tell you
        # whether an instability is observation delay or gain. Hold this at
        # DELTA_SCALE/5 while running at the full 50 Hz to isolate delay.
        # DIAGNOSTIC KNOB: any value other than the default is a deliberate
        # departure from what the checkpoint trained with. Do not ship one without
        # a decision recorded next to deployed_tam.json.
        self.policy.delta_scale = float(
            rospy.get_param("~joint_delta_scale", DELTA_SCALE))
        self.policy.reset()
        self.builder = ProprioBuilder()
        self.builder.reset()

        # latest sensor snapshot (filled by callbacks, read by the timer thread)
        self._euler = np.zeros(3, dtype=np.float32)     # board-frame corrected
        self._gyro = None        # board-frame corrected raw gyro (p,q,r); None until first GYRO sample
        self._joint_pos = np.zeros(2, dtype=np.float32)
        self._joint_vel = None            # driver-differentiated, None until seen
        self._board_rates = None          # set if use_board_rates and /albc_status seen
        self._last_sensor_t = None        # rospy time of last GOOD imu sample
        self._last_joints_t = None        # rospy time of last GOOD joint state

        # command setpoint (obs 0:3). Default = hold (zero). Wire /rl/command to change.
        # [roll_att_cmd, pitch_att_cmd, yaw_rate_cmd]  (attitude-only)
        self._command = np.zeros(3, dtype=np.float32)

        # terminal status bookkeeping (1 Hz line so the operator can SEE it run)
        self._tick_count = 0
        self._pub_count = 0
        self._tick_ms_max = 0.0
        self._tick_ms_ema = 0.0

        # --- subscribers ---
        rospy.Subscriber("/hero_agent/sensors", hero_agent_sensor,
                         self._on_sensor, queue_size=1)
        rospy.Subscriber("/albc/joint_states", JointState,
                         self._on_joints, queue_size=1)
        if self.use_board_rates:
            # /albc_status is advertised as Float64MultiArray (status_publisher.h)
            rospy.Subscriber("/albc/status", Float64MultiArray,
                             self._on_albc_status, queue_size=1)
        # optional external setpoint: [roll_att, pitch_att, yaw_rate]
        rospy.Subscriber("/rl/command", Float32MultiArray,
                         self._on_command, queue_size=1)

        # --- publishers ---
        self._pub_j1 = rospy.Publisher(
            "/hero_agent/active_joint1_position_controller/command", Float64, queue_size=1)
        self._pub_j2 = rospy.Publisher(
            "/hero_agent/active_joint2_position_controller/command", Float64, queue_size=1)
        self._pub_thr = rospy.Publisher("/albc/thruster_cmd", Float32MultiArray, queue_size=1)

        self._log_startup_banner(weights_dir, student_npz, teacher_npz)

        # RL-DEPLOY 2026-08-17: refuse to start from an already-wound joint1.
        # This runs BEFORE homing on purpose -- homing commands the arm and cannot
        # unwind it (it stops at the nearest 0-equivalent), so a wound arm must not
        # reach it. Do not move this below _home_arm or the Timer.
        if not self._gate_start_pose():
            rospy.signal_shutdown("joint1 start-pose gate")
            return

        # Home the arm BEFORE the policy starts, so every run begins from the same
        # pose. Without this the operator had to launch-albc, press 3 (FIXED ramps
        # to FK(90,90)), then Ctrl-C -- and that Ctrl-C kills joint_angle_command,
        # whose exit handler disables torque, so the arm drifted again before the
        # RL driver picked it up. Doing it here removes the round trip AND the drift.
        # It matters beyond convenience: np_policy seeds _joint_target from the
        # MEASURED angles on the first act(), so the starting pose is an initial
        # condition of the experiment, not a detail. On 2026-08-13 a run started at
        # joint2 = 2.985 rad, 0.157 from the pi abort threshold and at 8 % of the
        # available moment arm.
        if rospy.get_param("~home_on_start", False):
            self._home_arm(
                float(rospy.get_param("~home_joint1", 0.0)),
                float(rospy.get_param("~home_joint2", np.pi / 2.0)),
                float(rospy.get_param("~home_tol", 0.05)),
                float(rospy.get_param("~home_timeout_s", 25.0)))

        self._timer = rospy.Timer(rospy.Duration(1.0 / self.hz), self._tick)

    def _gate_start_pose(self):
        """Refuse to start the policy from an already-wound joint1.

        WHY. The 2026-08-13 cable break was two-stage: the policy itself wound J1
        past 2*pi (measured cmd_trav up to -31.89 rad in 13.3 s), and only then did
        the driver's restart ratchet push the arm beyond the policy's own rail. The
        trained constraint was never breached -- Constraint/margin/joint1_pos =
        0.99875 -- because sim writes theta1 to uniform(-pi, pi) at EVERY episode
        reset, so "+-2 turns" is measured from a start that is always near zero.
        The real arm has no such reset. |theta1| <= pi at launch is therefore the
        precondition under which the trained limit means anything here.

        WHY HERE AND NOT IN THE DRIVER. joint_angle_command (`run-joint`) is the
        tool the operator uses to unwind a wound arm, and it is the only publisher
        of /albc/joint_states. A driver-side refusal would block the recovery
        procedure and blind the operator to the very angle being gated. The driver
        keeps the guard that matters there -- the 3-turn abort on commanded AND
        measured angle -- and this node keeps the one about how a run may begin.

        ~joint1_start_max_rad is a knob, not a wall: raise it to start deliberately
        from a wound pose. <= 0 disables the check, matching the driver's
        overGuard(limit > 0) convention.
        """
        limit = float(rospy.get_param("~joint1_start_max_rad", np.pi))
        if limit <= 0.0:
            rospy.logwarn("start-pose gate DISABLED (~joint1_start_max_rad = %.3f)", limit)
            return True
        try:
            msg = rospy.wait_for_message("/albc/joint_states", JointState, timeout=5.0)
        except rospy.ROSException:
            # Not a reason to abort: the driver being down is a different failure,
            # and _tick already HOLDs while joint_states is stale.
            rospy.logwarn("start-pose gate: no /albc/joint_states in 5 s -- is "
                          "joint_angle_command running? Check SKIPPED.")
            return True
        if len(msg.position) < 1 or not np.isfinite(msg.position[0]):
            rospy.logwarn("start-pose gate: joint1 unreadable -- check SKIPPED.")
            return True

        theta1 = float(msg.position[0])
        if abs(theta1) > limit:
            rospy.logerr("START REFUSED: joint1 is at %.3f rad (%.2f turns), past the "
                         "%.3f rad start limit. The arm is still wound -- unwind it "
                         "before running the policy (`run-joint`, or by hand with "
                         "torque off). Starting here would spend the trained +-2-turn "
                         "budget from an offset sim never showed the policy. Override "
                         "with _joint1_start_max_rad:=<rad> if that is deliberate.",
                         theta1, theta1 / (2.0 * np.pi), limit)
            return False
        rospy.loginfo("start-pose gate OK: joint1 %.3f rad (%.2f turns) within %.3f",
                      theta1, theta1 / (2.0 * np.pi), limit)
        return True

    def _home_arm(self, q1, q2, tol, timeout_s):
        """Drive the arm to (q1, q2) and block until it arrives or times out.

        Safe to block: rospy dispatches subscriber callbacks on their own threads,
        so _on_joints keeps updating self._joint_pos while this loop runs. The
        driver applies its own 5 s startup ramp to the first command it receives
        (joint_angle_command STARTUP_TICKS), so this is a slow move, not a snap.

        A timeout WARNS and continues rather than aborting: refusing to start would
        leave the operator with a dead node and a wet robot, which is worse than a
        known-bad initial condition that the log records.
        """
        rospy.loginfo("homing arm -> joint1 %.3f  joint2 %.3f rad (tol %.3f) ...",
                      q1, q2, tol)
        rate = rospy.Rate(5.0)
        t0 = rospy.get_time()
        while not rospy.is_shutdown():
            # gate on _last_joints_t, NOT on _joint_pos: the latter is initialised to
            # zeros(2) (:151), so "is not None" is true before any sample arrives and
            # a home target of (0, 0) would report success against stale zeros.
            if self._last_joints_t is not None:
                d1 = abs(float(self._joint_pos[0]) - q1)
                d2 = abs(float(self._joint_pos[1]) - q2)
                if d1 < tol and d2 < tol:
                    rospy.loginfo("homing done: joint1 %.3f  joint2 %.3f rad",
                                  float(self._joint_pos[0]), float(self._joint_pos[1]))
                    return True
                rospy.loginfo_throttle(
                    2.0, "  homing: at (%.3f, %.3f) err (%.3f, %.3f)",
                    float(self._joint_pos[0]), float(self._joint_pos[1]), d1, d2)
            elapsed = rospy.get_time() - t0
            if elapsed > timeout_s:
                where = ("no /albc/joint_states yet -- is joint_angle_command running?"
                         if self._last_joints_t is None
                         else "at (%.3f, %.3f)" % (float(self._joint_pos[0]),
                                                   float(self._joint_pos[1])))
                rospy.logwarn("homing TIMEOUT after %.1f s (%s) -- starting anyway. "
                              "The policy will seed _joint_target from wherever the "
                              "arm actually is; treat this run's initial condition "
                              "as unknown.", elapsed, where)
                return False
            self._pub_j1.publish(Float64(q1))
            self._pub_j2.publish(Float64(q2))
            rate.sleep()
        return False

    # ------------------------------------------------------------- startup
    def _log_startup_banner(self, weights_dir, student_npz, teacher_npz):
        rospy.loginfo("=============================================")
        rospy.loginfo(" RL inference node  (69D attitude-only)")
        rospy.loginfo("  encoder      : %s   control: %.0f Hz", self.encoder_type, self.hz)
        rospy.loginfo("  joint gain   : delta_scale %.4f/tick -> %.3f rad/s at full "
                      "action%s", self.policy.delta_scale,
                      self.policy.delta_scale * self.hz,
                      "" if abs(self.policy.delta_scale - DELTA_SCALE) < 1e-9
                      else "   *** NOT the trained default %.4f ***" % DELTA_SCALE)
        rospy.loginfo("  weights_dir  : %s", os.path.abspath(weights_dir))
        rospy.loginfo("  student npz  : %s  md5 %s", os.path.basename(student_npz),
                      _md5_8(student_npz))
        rospy.loginfo("  teacher npz  : %s  md5 %s", os.path.basename(teacher_npz),
                      _md5_8(teacher_npz))
        rospy.loginfo("  imu_yaw_offset: %.1f deg (rotate_imu board-frame correction ON)",
                      float(np.rad2deg(self.imu_yaw_offset)))
        rospy.loginfo("  board_rates  : %s   timeouts: imu %.2fs / joints %.2fs",
                      self.use_board_rates, self.sensor_timeout, self.joint_timeout)
        if self.thruster_scale == 0.0:
            rospy.logwarn("THRUSTER SAFE MODE: scale=0.0 -> thrusters held at 0 (joint-only test)")
        elif self.thruster_max_s > 0.0:
            rospy.logwarn("THRUSTER TIME LIMIT: scale=%.2f for %.1fs, then latched to 0",
                          self.thruster_scale, self.thruster_max_s)
        else:
            rospy.logwarn("THRUSTER LIVE: scale=%.2f, no time limit", self.thruster_scale)
        rospy.loginfo("  waiting for /hero_agent/sensors + /albc/joint_states ...")
        rospy.loginfo("=============================================")

    # ------------------------------------------------------------- callbacks
    def _on_reconfigure(self, config, level):
        # live-tune the IMU mounting yaw offset (euler + gyro share this).
        self.imu_yaw_offset = float(np.deg2rad(config.imu_yaw_offset_deg))
        # policy setpoint (obs 0:3). The /rl/command topic writes the same field;
        # last writer wins, both log, and the log line says which. See GyroOffset.cfg
        # for why there is no hidden precedence rule.
        self._command = np.array([
            np.deg2rad(config.cmd_roll_deg),
            np.deg2rad(config.cmd_pitch_deg),
            config.cmd_yaw_rate,
        ], dtype=np.float32)
        rospy.loginfo("reconfigure: imu_yaw_offset %.2f deg | cmd roll %.1f deg "
                      "pitch %.1f deg yawrate %.3f rad/s",
                      config.imu_yaw_offset_deg, config.cmd_roll_deg,
                      config.cmd_pitch_deg, config.cmd_yaw_rate)
        return config

    def _on_sensor(self, msg):
        # hero_agent_sensor: ROLL,PITCH,YAW,DEPTH + GYRO_X/Y/Z -- RAW imu frame. Apply
        # the board-frame correction here (oracle: imu_rotation.h; albc_controller,
        # which normally does it, is NOT running in the RL configuration).
        raw = (msg.ROLL, msg.PITCH, msg.YAW)
        if not np.all(np.isfinite(raw)):
            rospy.logwarn_throttle(
                1.0, "IMU sample dropped: non-finite values %s" % (raw,))
            return  # keep previous sample; staleness gate trips if this persists
        self._euler = rotate_imu(raw[0], raw[1], raw[2], self.imu_yaw_offset)
        # raw firmware gyro (sim root_ang_vel_b truth). A gyro-capable firmware is
        # flashed and publishes live GYRO_X/Y/Z, so this path is normally active.
        # The all-zero guard below stays only as a fallback: a pre-gyro firmware
        # would publish 0.0 defaults, and treating all-zero as "no gyro" keeps the
        # euler-diff path available if such a firmware is ever re-flashed.
        gyro_raw = (msg.GYRO_X, msg.GYRO_Y, msg.GYRO_Z)
        if np.all(np.isfinite(gyro_raw)) and any(g != 0.0 for g in gyro_raw):
            self._gyro = rotate_gyro(gyro_raw[0], gyro_raw[1], gyro_raw[2],
                                     self.imu_yaw_offset)
        self._last_sensor_t = rospy.get_time()

    def _on_joints(self, msg):
        if len(msg.position) < 2:
            return
        pos = np.array(msg.position[:2], dtype=np.float32)
        vel = None
        if len(msg.velocity) >= 2:
            # driver-differentiated at its true loop rate -- use verbatim (no re-diff)
            vel = np.array(msg.velocity[:2], dtype=np.float32)
        if not np.all(np.isfinite(pos)) or (vel is not None and not np.all(np.isfinite(vel))):
            rospy.logwarn_throttle(1.0, "joint_states sample dropped: non-finite values")
            return
        self._joint_pos = pos
        self._joint_vel = vel
        self._last_joints_t = rospy.get_time()

    def _on_albc_status(self, msg):
        # /albc_status layout (status_publisher.h): indices [8,9,10] = ang vel p,q,r.
        if len(msg.data) >= 11:
            self._board_rates = np.array(msg.data[8:11], dtype=np.float32)

    def _on_command(self, msg):
        if len(msg.data) >= 3:
            self._command = np.array(msg.data[:3], dtype=np.float32)

    # ------------------------------------------------------------- control loop
    def _tick(self, _evt):
        # The Timer thread dies on an uncaught exception while the node stays
        # alive-but-silent -- so everything is guarded and logged loudly.
        try:
            t0 = time.time()
            self._tick_count += 1
            if self._tick_impl():
                self._pub_count += 1
            ms = (time.time() - t0) * 1000.0
            self._tick_ms_ema = 0.95 * self._tick_ms_ema + 0.05 * ms
            if ms > self._tick_ms_max:
                self._tick_ms_max = ms
            budget_ms = 1000.0 / self.hz
            if ms > budget_ms:
                rospy.logwarn_throttle(
                    5.0, "tick overrun: %.1f ms > %.1f ms budget" % (ms, budget_ms))
        except Exception:
            rospy.logerr_throttle(
                5.0, "tick exception (control loop still alive):\n%s"
                % traceback.format_exc())

    def _tick_impl(self):
        """One control tick. Returns True if a command was published."""
        now = rospy.get_time()

        # --- gates: never run the policy on missing/stale state -----------------
        if self._last_sensor_t is None:
            rospy.logwarn_throttle(
                2.0, "waiting for first IMU sample on /hero_agent/sensors "
                     "(rosserial up? /dev/ttyUSB1?)")
            return False
        if self._last_joints_t is None:
            rospy.logwarn_throttle(
                2.0, "waiting for first /albc/joint_states "
                     "(joint_angle_command driver running?)")
            return False
        sensor_age = now - self._last_sensor_t
        if sensor_age > self.sensor_timeout:
            rospy.logwarn_throttle(
                1.0, "IMU STALE %.2fs (> %.2fs) -- HOLDING, no commands published"
                % (sensor_age, self.sensor_timeout))
            return False
        joints_age = now - self._last_joints_t
        if joints_age > self.joint_timeout:
            rospy.logwarn_throttle(
                1.0, "joint_states STALE %.2fs (> %.2fs) -- HOLDING, no commands published"
                % (joints_age, self.joint_timeout))
            return False

        # --- assemble obs and run the policy ------------------------------------
        sensors = {
            "cmd_att": self._command[0:2],           # roll_att, pitch_att
            "cmd_yawrate": float(self._command[2]),  # yaw_rate
            "euler": self._euler,
            "joint_pos": self._joint_pos,
            # no "thruster" key: builder echoes set_last_action()[2:8] itself
        }
        if self._gyro is not None:
            sensors["gyro"] = self._gyro             # firmware raw gyro (rotate_gyro'd)
        if self._joint_vel is not None:
            sensors["joint_vel"] = self._joint_vel   # driver value, verbatim
        proprio = self.builder.build(sensors)
        if self.use_board_rates and self._board_rates is not None:
            # overwrite the differentiated angvel (6:9) with the board estimate
            if self._gyro is not None:
                rospy.logwarn_throttle(
                    10.0, "use_board_rates=true overrides firmware gyro -- "
                          "gyro passthrough ignored (set use_board_rates=false)")
            proprio[6:9] = self._board_rates

        # 69D attitude-only: command [roll_att, pitch_att, yaw_rate] passes straight
        # through; the policy runtime does cmd - measured internally and carries the
        # leaky integral as a state buffer (the 87D reorder step no longer exists).
        action = self.policy.act(proprio, self._command)
        if not np.all(np.isfinite(action)):
            rospy.logerr_throttle(
                1.0, "non-finite action -- resetting policy+builder state, no publish")
            self.policy.reset()
            self.builder.reset()
            return False
        self.builder.set_last_action(action)

        # --- publish -------------------------------------------------------------
        # arm joints: ABSOLUTE accumulated PD target (driver contract), never scaled
        target = self.policy.joint_target
        self._pub_j1.publish(Float64(float(target[0])))
        self._pub_j2.publish(Float64(float(target[1])))

        # thrusters: surface-test safety (scale + optional time latch)
        scale = self.thruster_scale
        if self.thruster_max_s > 0.0:
            if self._first_tick_t is None:
                self._first_tick_t = now
            elif now - self._first_tick_t >= self.thruster_max_s:
                if scale != 0.0:
                    rospy.logwarn_throttle(
                        30.0, "thruster time limit %.1fs reached -- latched to 0"
                        % self.thruster_max_s)
                scale = 0.0  # latch thrusters off; joints keep running
        thr_msg = Float32MultiArray()
        thr_msg.data = [float(x) * scale for x in action[2:8]]
        self._pub_thr.publish(thr_msg)

        # --- 1 Hz status line so the operator can SEE the loop run ---------------
        eul_deg = np.rad2deg(self._euler)
        rospy.loginfo_throttle(
            1.0,
            "#%d pub=%d | eul(deg) r%+.1f p%+.1f y%+.1f | jnt(%.2f,%.2f)->tgt(%.2f,%.2f) "
            "| thr %s | age imu %dms jnt %dms | tick %.1f/%.1fms"
            % (self._tick_count, self._pub_count,
               eul_deg[0], eul_deg[1], eul_deg[2],
               self._joint_pos[0], self._joint_pos[1], target[0], target[1],
               ("SAFE0" if scale == 0.0 else "x%.2f" % scale),
               int(sensor_age * 1000), int(joints_age * 1000),
               self._tick_ms_ema, self._tick_ms_max))
        return True


def main():
    rospy.init_node("rl_inference_node")
    RLInferenceNode()
    rospy.spin()


if __name__ == "__main__":
    main()
