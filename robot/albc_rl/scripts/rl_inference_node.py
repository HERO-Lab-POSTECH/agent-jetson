#!/usr/bin/env python
"""ROS node: real-time student-policy inference for the agent-jetson UUV (50 Hz).

Subscribes to the board sensors, assembles the 20D proprio block (build_proprio.py)
for the 69D attitude-only policy, runs the torch-free numpy policy
(../numpy_port/np_policy.py), and publishes the 8D action. Pure numpy + rospy --
no torch on the board (Python 3.5, numpy 1.11, ROS lunar).

DATA FLOW (one 50 Hz tick)
--------------------------
    /hero_agent/sensors (ROLL,PITCH,YAW,DEPTH) ->| ProprioBuilder.build()
    /albc/joint_states  (j0,j1 pos)            ->|  -> 20D proprio
    [att cmd / yaw-rate cmd] setpoints         ->|  + 3D command
                                                  v
                          NumpyStudentPolicy.act(proprio_20, command_3) -> action(8)
                                                  v
    action[0:2] -> /hero_agent/active_joint{1,2}_position_controller/command (arm delta)
    action[2:8] -> /albc/thruster_cmd (6 thrusters)   [wire to your thruster mixer]

WHAT THIS NODE OWNS vs DELEGATES
--------------------------------
  * proprio 20D layout, frames, signs ......... build_proprio.ProprioBuilder
  * history(20:66) + integral(66:69) blocks ... NumpyStudentPolicy (internal buffers)
  * thruster echo (obs 14:20) ................. builder.set_last_action(prev action)
  * command setpoint (obs 0:3) ................ this node (_command, settable via topic)
  * angular velocity (obs 6:9) ................ builder differentiates euler (default),
        OR subscribe /albc_status and feed measured rates (set use_board_rates:=true)

The 3D integral is the policy runtime's responsibility -- it carries `_error_integral`
as a state buffer (leak=0.99, clamp=2.0, gated, sigma=0.10), mirroring the sim
(attitude_only/albc_env.py). This node just passes the 3D command through; it does NOT
reorder or recompute the integral (the 87D command_to_integral_order step is gone).

ROSPARAMS (all have safe defaults)
----------------------------------
  ~encoder_type   : "tcn" | "gru"            (default tcn)
  ~weights_dir    : dir holding weights_*.npz (default ../numpy_port)
  ~control_hz     : 50
  ~use_board_rates: false                    (true => trust /albc_status angular vel)
  ~thruster_scale : 1.0   gain on the 6 thruster channels before publish. 0.0 = surface
                          test (joint-only, thrusters held at 0). The arm action is never
                          scaled. SECONDARY safety only -- the board has no /albc/thruster_cmd
                          subscriber yet, so thrusters are physically unwired regardless.
  ~thruster_max_s : 0.0   if > 0, thrusters publish for this many seconds after the first
                          tick, then latch to 0 for the rest of the run (joint keeps going).
                          0.0 = no time limit. Use e.g. 2.0 for a brief surface burst test.

NOT RUNNABLE ON A DEV MAC (no ROS). Build + run on the board -- see README.md.
"""
import os

import numpy as np
import rospy
from std_msgs.msg import Float64, Float32MultiArray
from sensor_msgs.msg import JointState

from hero_msgs.msg import hero_agent_sensor

# local modules (numpy_port is a sibling of ros_node/)
import sys
_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _HERE)                                   # build_proprio
sys.path.insert(0, os.path.join(_HERE, "..", "numpy_port")) # np_policy, npforward

from build_proprio import ProprioBuilder       # noqa: E402
from np_policy import NumpyStudentPolicy         # noqa: E402


class RLInferenceNode(object):
    def __init__(self):
        self.encoder_type = rospy.get_param("~encoder_type", "tcn")
        weights_dir = rospy.get_param(
            "~weights_dir", os.path.join(_HERE, "..", "numpy_port"))
        self.hz = float(rospy.get_param("~control_hz", 50.0))
        self.use_board_rates = rospy.get_param("~use_board_rates", False)
        # surface-test safety on the 6 thruster channels (arm action never scaled)
        self.thruster_scale = float(rospy.get_param("~thruster_scale", 1.0))
        self.thruster_max_s = float(rospy.get_param("~thruster_max_s", 0.0))
        self._first_tick_t = None   # set on the first published tick (for thruster_max_s)

        student_npz = os.path.join(weights_dir, "weights_%s.npz" % self.encoder_type)
        teacher_npz = os.path.join(weights_dir, "weights_teacher.npz")
        self.policy = NumpyStudentPolicy(student_npz, teacher_npz, self.encoder_type)
        self.policy.reset()
        self.builder = ProprioBuilder()
        self.builder.reset()

        # latest sensor snapshot (filled by callbacks, read by the timer)
        self._euler = np.zeros(3, dtype=np.float32)
        self._joint_pos = np.zeros(2, dtype=np.float32)
        self._board_rates = None          # set if use_board_rates and /albc_status seen
        self._have_sensor = False

        # command setpoint (obs 0:3). Default = hold (zero). Wire a topic to change it.
        # [roll_att_cmd, pitch_att_cmd, yaw_rate_cmd]  (attitude-only, no lin-vel command)
        self._command = np.zeros(3, dtype=np.float32)

        # --- subscribers ---
        rospy.Subscriber("/hero_agent/sensors", hero_agent_sensor,
                         self._on_sensor, queue_size=1)
        rospy.Subscriber("/albc/joint_states", JointState,
                         self._on_joints, queue_size=1)
        if self.use_board_rates:
            rospy.Subscriber("/albc_status", Float32MultiArray,
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

        rospy.loginfo("RL node: %s encoder (69D attitude-only), %.0f Hz, board_rates=%s",
                      self.encoder_type, self.hz, self.use_board_rates)
        if self.thruster_scale == 0.0:
            rospy.logwarn("THRUSTER SAFE MODE: scale=0.0 -> thrusters held at 0 (joint-only test)")
        elif self.thruster_max_s > 0.0:
            rospy.logwarn("THRUSTER TIME LIMIT: scale=%.2f for %.1fs, then latched to 0",
                          self.thruster_scale, self.thruster_max_s)
        else:
            rospy.logwarn("THRUSTER LIVE: scale=%.2f, no time limit", self.thruster_scale)
        self._timer = rospy.Timer(rospy.Duration(1.0 / self.hz), self._tick)

    # ------------------------------------------------------------- callbacks
    def _on_sensor(self, msg):
        # hero_agent_sensor: ROLL, PITCH, YAW, DEPTH. Board-frame correction
        # (PITCH sign, yaw offset) happens in the board controller; if the raw
        # topic is NOT yet corrected, apply rotateImu-equivalent here. See README.
        self._euler = np.array([msg.ROLL, msg.PITCH, msg.YAW], dtype=np.float32)
        self._have_sensor = True

    def _on_joints(self, msg):
        if len(msg.position) >= 2:
            self._joint_pos = np.array(msg.position[:2], dtype=np.float32)

    def _on_albc_status(self, msg):
        # /albc_status layout (board): indices [8,9,10] = angular velocity p,q,r.
        if len(msg.data) >= 11:
            self._board_rates = np.array(msg.data[8:11], dtype=np.float32)

    def _on_command(self, msg):
        if len(msg.data) >= 3:
            self._command = np.array(msg.data[:3], dtype=np.float32)

    # ------------------------------------------------------------- control loop
    def _tick(self, _evt):
        if not self._have_sensor:
            return  # wait for first IMU sample
        sensors = {
            "cmd_att": self._command[0:2],          # roll_att, pitch_att
            "cmd_yawrate": float(self._command[2]),  # yaw_rate
            "euler": self._euler,
            "joint_pos": self._joint_pos,
            "thruster": np.zeros(6, dtype=np.float32),  # filled below via last action echo
        }
        # thruster obs = previous action's 6 thruster channels (builder tracks it)
        proprio = self.builder.build(sensors)
        if self.use_board_rates and self._board_rates is not None:
            # overwrite the differentiated angvel (6:9) with the board estimate
            proprio[6:9] = self._board_rates

        # 69D attitude-only: command is [roll_att, pitch_att, yaw_rate], which is exactly
        # the integral-error channel order [roll, pitch, yaw_rate]. The policy runtime does
        # cmd - measured internally (measured = euler[roll,pitch], ang_vel_b[2]) and carries
        # the leaky integral as a state buffer -- so the node passes the 3D command straight
        # through, no reorder. (The 87D command_to_integral_order step no longer exists.)
        action = self.policy.act(proprio, self._command)
        self.builder.set_last_action(action)

        # arm joints: always published unscaled
        self._pub_j1.publish(Float64(action[0]))
        self._pub_j2.publish(Float64(action[1]))

        # thrusters: apply surface-test safety (scale + optional time latch)
        scale = self.thruster_scale
        if self.thruster_max_s > 0.0:
            now = rospy.get_time()
            if self._first_tick_t is None:
                self._first_tick_t = now
            elif now - self._first_tick_t >= self.thruster_max_s:
                scale = 0.0  # latch thrusters off; joints keep running
        thr_msg = Float32MultiArray()
        thr_msg.data = [float(x) * scale for x in action[2:8]]
        self._pub_thr.publish(thr_msg)


def main():
    rospy.init_node("rl_inference_node")
    RLInferenceNode()
    rospy.spin()


if __name__ == "__main__":
    main()
