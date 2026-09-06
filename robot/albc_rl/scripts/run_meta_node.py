#!/usr/bin/env python
"""Publish /albc/run_meta for runs whose controller cannot publish it itself.

WHO NEEDS THIS. rl_inference_node publishes its own manifest because only it
knows which weights it loaded. Every OTHER path -- the TDC controller
(albc_controller, C++), the firmware yaw/depth PID reached through teleop, an
open-loop instrumentation sweep -- has no python node that knows the run's
identity, and so produced bags with no provenance at all. This node fills that
in from the parameter server and the first sample of each state topic.

WHY IT SPINS. The manifest is LATCHED, and a latched topic exists only while
its publisher does. Exiting after one publish would leave the recorder holding
a message whose publisher is gone -- which works for a recorder that already
connected and silently fails for one that starts a second later. So it stays up
for the life of the run and shuts down with the launch.

NOT A CONTROLLER. It publishes one manifest and nothing else. It takes one
sample of each state topic through wait_for_message and holds no subscription
afterwards -- a node keeping callbacks alive on a 100 Hz topic for no reason is
load the control loop has to share.

Run:  rosrun albc_rl run_meta_node.py _controller:=tdc _scenario:=s1_attitude
"""
import numpy as np
import rospy
import rospkg
from sensor_msgs.msg import JointState

from hero_msgs.msg import hero_agent_sensor

from albc_rl import contract
from albc_rl.build_proprio import rotate_imu
from albc_rl.contract import TOPICS
from albc_rl.run_log import RunLogger, git_provenance, pack_provenance

# How long to wait for one sample of each state topic before giving up on that
# field of the initial condition. Short on purpose: a missing initial attitude
# is a thin manifest, and a thin manifest must not delay a run.
FIRST_SAMPLE_TIMEOUT_S = 5.0


def _param_tree(ns, default=None):
    """Whole parameter namespace as a dict, or `default` when it is absent.

    Grabbing the namespace rather than a hand-listed set of names is the point:
    a knob added to albc_controller.yaml next month lands in the manifest
    without anyone remembering to add it here. A hand-listed set would be a
    second place to keep in sync, which is the drift this repo keeps paying for.
    """
    try:
        return rospy.get_param(ns)
    except KeyError:
        return default if default is not None else {}
    except Exception as exc:                           # noqa: BLE001
        rospy.logwarn("could not read params under %s (%s)", ns, exc)
        return {}


def _first(topic, msg_type, timeout=FIRST_SAMPLE_TIMEOUT_S):
    try:
        return rospy.wait_for_message(topic, msg_type, timeout=timeout)
    except rospy.ROSException:
        rospy.logwarn("no %s within %.1f s -- initial condition will be partial",
                      topic, timeout)
        return None


def collect_initial(imu_yaw_offset_deg):
    """The run's initial condition, in the frame the controllers actually use.

    Attitude is passed through rotate_imu. /hero_agent/sensors is the RAW imu
    frame and reading it straight is the 2026-08-11 error; a manifest that
    recorded raw roll/pitch would seed every later analysis with it.
    """
    out = {}
    js = _first(TOPICS["joint_states"], JointState)
    if js is not None and len(js.position) >= 2:
        out["joint1_rad"] = float(js.position[0])
        out["joint2_rad"] = float(js.position[1])
        out["joint2_deg"] = float(np.degrees(js.position[1]))

    sen = _first(TOPICS["sensors"], hero_agent_sensor)
    if sen is not None:
        off = float(np.deg2rad(imu_yaw_offset_deg))
        eul = rotate_imu(sen.ROLL, sen.PITCH, sen.YAW, off)
        out["roll_deg"] = float(np.degrees(eul[0]))
        out["pitch_deg"] = float(np.degrees(eul[1]))
        out["yaw_deg"] = float(np.degrees(eul[2]))
        # sqrt(r^2+p^2) is invariant under rotate_imu (a rotation about z), so
        # this number means the same thing in either frame -- the property the
        # start-attitude gate relies on.
        out["tilt_deg"] = float(np.degrees(np.hypot(eul[0], eul[1])))
        out["imu_yaw_offset_deg"] = float(imu_yaw_offset_deg)
        out["frame"] = "body (rotate_imu applied)"
        # DEPTH on this message is NOT depth -- it carries loop_speed. Real
        # depth is hero_agent_state.Depth, which the recorder now bags. Saying
        # so here stops the next reader from reaching for the wrong field.
        out["sensors_depth_field"] = \
            "loop_speed, NOT depth (real depth: /hero_agent/state.Depth)"
    return out


def main():
    rospy.init_node("run_meta_node")

    controller = rospy.get_param("~controller", "none")
    scenario = rospy.get_param("~scenario", "") or None
    notes = rospy.get_param("~notes", "")

    # The TDC controller's knobs live in its own namespace (launch_albc.sh does
    # a rosparam load into /albc_controller), so they are NOT under this node's
    # private namespace and have to be read from there explicitly.
    ctrl_params = _param_tree("/albc_controller")
    mixer_params = _param_tree("/thruster_mixer")
    driver_params = _param_tree("/joint_angle_command")

    # imu_yaw_offset: the controller's live value when it is up, else the
    # pinned board default. Recorded either way, because the initial attitude
    # below is only interpretable together with the offset that produced it.
    imu_off = float(ctrl_params.get("imu_yaw_offset", 102.0))

    pkg = rospkg.RosPack().get_path("albc_rl")
    logger = RunLogger("run_meta_node", controller, scenario=scenario)

    # A weights_dir is only meaningful for an RL run, and an RL run publishes
    # its own manifest -- so this stays empty except when someone points this
    # node at a pack deliberately (a replay, an offline check).
    weights_dir = rospy.get_param("~weights_dir", "")
    pack = pack_provenance(weights_dir) if weights_dir else {}

    logger.publish_meta(
        params={
            "self": _param_tree("~"),
            "albc_controller": ctrl_params,
            "thruster_mixer": mixer_params,
            "joint_angle_command": driver_params,
        },
        pack=pack,
        git=git_provenance(pkg),
        mixer=mixer_params,
        contract={
            "POLICY_OBS_DIM": contract.POLICY_OBS_DIM,
            "ACTION_DIM": contract.ACTION_DIM,
            "DELTA_SCALE": contract.DELTA_SCALE,
            "CONTROL_DT": contract.CONTROL_DT,
            "L_LINK": contract.L_LINK,
            "topics": dict(TOPICS),
        },
        initial=collect_initial(imu_off),
        notes=notes,
    )
    rospy.loginfo("run_meta_node up -- holding the latched manifest for this run")
    rospy.spin()


if __name__ == "__main__":
    main()
