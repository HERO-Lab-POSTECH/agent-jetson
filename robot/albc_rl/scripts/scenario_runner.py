#!/usr/bin/env python
"""Drive and MARK the three validation scenarios. Board python 2.7, ROS lunar.

    rosrun albc_rl scenario_runner.py --scenario s1 --controller rl
    rosrun albc_rl scenario_runner.py --scenario s2 --controller tdc
    rosrun albc_rl scenario_runner.py --scenario s3 --objects objects.json
    python  scenario_runner.py --self-test        # offline, no ROS

WHAT THIS IS FOR. A validation run has to be repeatable and it has to be
segmentable afterwards. Until now both were done by hand: the operator typed
dynparam commands and wrote times in a notebook, and the analysis recovered the
segments by regexing /rosout_agg (t4_step_analyze.py's RECONF pattern). That
loses the run the moment a log line is reworded, and it cannot produce the
repeats the statistics need.

So this script does two things and no more:
  * ISSUE the attitude setpoint sequence, where the sequence is programmatic
  * MARK every phase boundary on /albc/run_event, as data

It does NOT drive translation, depth or the gripper. On this robot those are
the operator's: horizontal motion is open-loop teleop (move_speed), the depth
setpoint moves only with the firmware keys o and l, and there is no gripper
channel in the 8D action space. S2 and S3 are therefore operator-driven with
scripted MARKS, which is an honest division of labour rather than a limitation
papered over.

ONE FILE, THREE SCENARIOS, on purpose. They share the setpoint plumbing, the
safety preconditions and the marking; only the sequence differs, and a sequence
is data. Three files would be three places to fix the next safety rule.

WHY THE SEQUENCES LOOK LIKE THIS
--------------------------------
  * A BASELINE segment comes first, always. E1 (2026-08-25) set a 0.2 deg
    success threshold and then measured an in-water noise floor of 0.09 to 0.23
    deg, which made the verdict meaningless. A threshold is interpretable only
    against a floor measured in the same session, so every scenario measures
    one and marks it.
  * REPEATS, with the repeat index in the event. Lynnerup et al. 2019
    (arXiv:1909.03772 section 4.3) run 10 repetitions per condition on real
    hardware to establish statistical significance, and section 3.1 grants an
    explicit exemption: state why, if the cost makes repetition impossible.
    Three is the default here because the tank session is the binding cost; the
    number is a knob and the run records which one was used.
  * OPTIONAL SHUFFLE with a recorded seed. A fixed ladder confounds amplitude
    with session drift, and this vehicle drifts (depth +0.0023 m/min, gyro
    2.2 deg/min).
  * The return to zero between steps is not padding: settling time is defined
    only from a known starting point.

SAFETY
------
  * S1 REFUSES to start if any thruster scale is non-zero. "Thrusters off" is
    the definition of the scenario, not a preference.
  * Every scenario watches /albc/run_event for kind == "guard" and stops. The
    RL node's arm guard latches INSIDE that node, so this event is the only way
    an outside process learns the arm has stopped being commanded.
  * Ctrl-C marks scenario_abort before exiting, so a partial run is still a
    segmented run.
"""
import argparse
import json
import random
import sys
import threading

import numpy as np

# ROS is imported softly so --self-test runs on a dev machine. The sequence
# builders and the safety tables below are the parts worth checking, and a
# check that only runs on the board is a check that runs the day after the tank
# session -- the same argument run_log.py makes for its pure functions.
try:
    import rospy
    from std_msgs.msg import Float32MultiArray, String
    HAVE_ROS = True
except ImportError:                                    # dev machine
    rospy = None
    Float32MultiArray = String = None
    HAVE_ROS = False

if __package__ or HAVE_ROS:
    from albc_rl.contract import TOPICS
    from albc_rl.run_log import RunLogger, EVENT_TOPIC
else:                                                  # running the file directly
    import os as _os
    sys.path.insert(0, _os.path.join(
        _os.path.dirname(_os.path.dirname(_os.path.abspath(__file__))), "src"))
    from albc_rl.contract import TOPICS
    from albc_rl.run_log import RunLogger, EVENT_TOPIC

# The project's own settling band: 0.087 rad = 5 deg, already the
# settling_threshold of the sim evaluation (RL-ALBC - Experiments.md). Reused
# rather than reinvented, so sim and tank report the same quantity.
SETTLE_BAND_DEG = 5.0


# ------------------------------------------------------------------ setpoints
class RLSetpoint(object):
    """Attitude setpoint for the learned controller, over /albc/rl_command.

    The topic, not the dynparam, because the topic is RECORDED while a dynparam
    write survives only as a log line. rl_inference_node emits a `setpoint`
    event on change either way, but a scenario should not depend on a
    reconfigure server being up.
    """
    name = "rl"

    def __init__(self):
        self._pub = rospy.Publisher(TOPICS["rl_command"], Float32MultiArray,
                                    queue_size=1)

    def wait_ready(self, timeout=10.0):
        # rospy's first publish goes out before the subscriber connection is
        # established and is simply dropped, so a one-shot setpoint vanishes.
        t0 = rospy.get_time()
        while (self._pub.get_num_connections() < 1
               and rospy.get_time() - t0 < timeout and not rospy.is_shutdown()):
            rospy.sleep(0.1)
        return self._pub.get_num_connections() > 0

    def send(self, roll_deg, pitch_deg, yaw_rate=0.0):
        msg = Float32MultiArray()
        msg.data = [float(np.deg2rad(roll_deg)), float(np.deg2rad(pitch_deg)),
                    float(yaw_rate)]
        self._pub.publish(msg)


class TDCSetpoint(object):
    """Attitude setpoint for the TDC baseline, over dynamic_reconfigure.

    target_roll / target_pitch in DEGREES, clamped to +-45 by the .cfg
    (albc_control/cfg/ALBCController.cfg lines 18-19). There is no topic for
    this one, so the dynparam client is not a choice.
    """
    name = "tdc"

    def __init__(self):
        import dynamic_reconfigure.client              # noqa: PLC0415
        self._cli = dynamic_reconfigure.client.Client(
            "/albc_controller", timeout=10.0)

    def wait_ready(self, timeout=10.0):
        return self._cli is not None

    def send(self, roll_deg, pitch_deg, yaw_rate=0.0):
        # yaw_rate has no counterpart on this controller. Callers are told once,
        # at scenario start, rather than having it silently dropped per step.
        self._cli.update_configuration({"target_roll": float(roll_deg),
                                        "target_pitch": float(pitch_deg)})


def make_setpoint(kind):
    if kind == "rl":
        return RLSetpoint()
    if kind == "tdc":
        return TDCSetpoint()
    raise ValueError(
        "no setpoint interface for controller %r. rl uses %s, tdc uses the "
        "/albc_controller dynparam; a new controller needs its own class here "
        "AND its topics added to run_record.launch, or its runs cannot be "
        "analysed." % (kind, TOPICS["rl_command"]))


# ------------------------------------------------------------------ sequences
def s1_steps(amplitudes, axes, hold_s, repeats, shuffle, seed):
    """Step ladder as data: [(name, roll_deg, pitch_deg, hold_s, meta), ...]

    Each step is followed by a return to zero, because settling time is defined
    only from a known starting point.
    """
    base = []
    for axis in axes:
        for amp in amplitudes:
            for sign in (+1, -1):
                base.append((axis, sign * amp))
    out = []
    rng = random.Random(seed)
    for rep in range(repeats):
        order = list(base)
        if shuffle:
            rng.shuffle(order)
        for axis, amp in order:
            roll = amp if axis == "roll" else 0.0
            pitch = amp if axis == "pitch" else 0.0
            out.append(("step_%s_%+d" % (axis, amp), roll, pitch, hold_s,
                        {"axis": axis, "amp_deg": amp, "repeat": rep}))
            out.append(("return_zero", 0.0, 0.0, hold_s,
                        {"axis": axis, "amp_deg": 0, "repeat": rep}))
    return out


# S2: the operator drives; this list is the marking schedule. `disturbance` is
# carried into the event because the analysis has to know which phases are NOT
# quiet holds -- averaging a disturbed phase into a hold statistic is how a
# controller comparison becomes unfair without anyone noticing.
S2_PHASES = [
    ("surface_hold", "hold at the surface, target (0,0). BASELINE floor.", None),
    ("translate", "operator teleop translation (OPEN LOOP: move_speed, wasd)",
     "open_loop_horizontal_thrust"),
    ("descend", "depth PID descent via the firmware keys o and l",
     # pid.cpp lines 107-108 give m0 and m3 the SAME depth command, and m3 is
     # DEAD. The descent thrust is therefore single sided at 3 o'clock and
     # injects a pitch moment. Labelling it is what keeps the comparison fair,
     # and it is also the condition the fault-tolerant-control literature has
     # never actually run on hardware.
     "single_sided_heave_m0_only_m3_dead"),
    ("approach", "final approach to the object", None),
    ("grasp", "gripper closes on the object (operator)", "payload_attach"),
    ("lift", "object lifted clear", "payload_attached"),
    ("ascend", "depth PID ascent", "single_sided_heave_m0_only_m3_dead"),
    ("return", "teleop translation back to start", "open_loop_horizontal_thrust"),
    ("surface_hold_end", "hold at the surface with payload, target (0,0)",
     "payload_attached"),
]

# S3: one block per object. The hold phases are what the metrics are computed
# over; the marks around them are what makes the blocks comparable.
S3_PHASES = [
    ("baseline_hold", "hold with NO payload, target (0,0)", None),
    ("grasp", "gripper closes (operator)", "payload_attach"),
    ("payload_hold", "hold WITH payload, target (0,0)", "payload_attached"),
    ("release", "gripper opens (operator)", "payload_release"),
    ("recovery_hold", "hold after release, target (0,0)", None),
]


# ------------------------------------------------------------------ runner
class ScenarioRunner(object):
    def __init__(self, args):
        self.args = args
        self.setpoint = make_setpoint(args.controller)
        self.log = RunLogger("scenario_runner", args.controller,
                             scenario=args.scenario)
        self._stop = threading.Event()
        self._stop_reason = ""
        # EVENT_TOPIC, not a literal: the 2026-09-05 rename left 13 diag tools
        # subscribing to names nobody published, and ROS reports that as an
        # empty topic rather than an error. A wrong constant fails to import.
        rospy.Subscriber(EVENT_TOPIC, String, self._on_event, queue_size=20)

    # -- safety -------------------------------------------------------
    def _on_event(self, msg):
        """Stop on any guard event.

        The RL arm guard latches INSIDE the node: it stops commanding the arm
        and says so only in its own log. Without this subscription a scenario
        keeps issuing setpoints at a robot that stopped listening, and the bag
        then looks like a controller that ignored its commands.
        """
        try:
            evt = json.loads(msg.data)
        except ValueError:
            return
        if evt.get("kind") == "guard":
            self._stop_reason = "guard event: %s %s" % (
                evt.get("name"), (evt.get("data") or {}).get("reason", ""))
            self._stop.set()

    def preflight(self):
        """Refuse the run rather than produce a mislabelled one."""
        problems = []

        if self.args.scenario == "s1":
            # "thrusters off" is the DEFINITION of S1. A run with thrust on is
            # not a degraded S1, it is a different experiment wearing S1's
            # label, and a mislabelled row in a comparison table is worse than
            # a missing one.
            for ns in ("/rl_inference_node/thruster_scale",
                       "/albc_rl/rl_inference_node/thruster_scale"):
                scale = rospy.get_param(ns, None)
                if scale is not None and float(scale) != 0.0:
                    problems.append("%s = %s, but S1 is defined as thrusters "
                                    "OFF" % (ns, scale))

        if not self.setpoint.wait_ready():
            problems.append(
                "no subscriber on the %s setpoint interface after 10 s. Is the "
                "controller running? The first publish on a fresh topic is "
                "dropped before a connection exists, so a setpoint sent now "
                "would silently vanish." % self.setpoint.name)

        if problems:
            for p in problems:
                rospy.logerr("PREFLIGHT: %s", p)
            return False
        return True

    # -- phases -------------------------------------------------------
    def _hold(self, seconds, label):
        """Sleep, but wake on a guard stop. rospy.sleep would ride through it."""
        rate = rospy.Rate(10.0)
        t_end = rospy.get_time() + seconds
        while rospy.get_time() < t_end and not rospy.is_shutdown():
            if self._stop.is_set():
                rospy.logerr("STOPPING during %s: %s", label, self._stop_reason)
                return False
            rate.sleep()
        return True

    def run_s1(self):
        a = self.args
        steps = s1_steps(a.amplitudes, a.axes, a.hold_s, a.repeats,
                         a.shuffle, a.seed)
        self.log.phase("scenario_start", scenario="s1",
                       controller=a.controller, n_steps=len(steps),
                       amplitudes_deg=a.amplitudes, axes=a.axes,
                       hold_s=a.hold_s, repeats=a.repeats,
                       shuffle=a.shuffle, seed=a.seed,
                       settle_band_deg=SETTLE_BAND_DEG,
                       thrusters="off (S1 definition)")

        # Baseline FIRST and always. A settling threshold is meaningful only
        # against a noise floor measured in the same session and the same
        # water: E1 set 0.2 deg against a floor that turned out to be 0.09 to
        # 0.23 deg, and the verdict had to be thrown away.
        self.setpoint.send(0.0, 0.0)
        self.log.phase("baseline_floor", purpose="in-session noise floor",
                       duration_s=a.baseline_s)
        if not self._hold(a.baseline_s, "baseline_floor"):
            return False

        for name, roll, pitch, hold_s, meta in steps:
            if self._stop.is_set():
                return False
            self.setpoint.send(roll, pitch)
            self.log.phase(name, roll_deg=roll, pitch_deg=pitch,
                           duration_s=hold_s, **meta)
            rospy.loginfo("S1 %s -> roll %+.1f pitch %+.1f, hold %.0f s",
                          name, roll, pitch, hold_s)
            if not self._hold(hold_s, name):
                return False

        self.setpoint.send(0.0, 0.0)
        return True

    def run_marked(self, phases, block_label=None, block_meta=None):
        """Operator-driven phases. The script marks; the human drives.

        Blocks on stdin between phases deliberately. A timed schedule would
        mark a boundary at a moment the operator had not reached, and a wrong
        boundary is worse than a late one because the analysis cannot see that
        it is wrong.
        """
        if block_label:
            self.log.phase("block_start", block=block_label, **(block_meta or {}))
        for name, description, disturbance in phases:
            if self._stop.is_set():
                rospy.logerr("STOPPING: %s", self._stop_reason)
                return False
            # target attitude is held at zero for the whole mission: that is
            # the constraint S2 and S3 are defined by
            self.setpoint.send(0.0, 0.0)
            sys.stdout.write("\n  NEXT PHASE: %s\n    %s\n"
                             "    press ENTER when it BEGINS (q to abort): "
                             % (name, description))
            sys.stdout.flush()
            answer = sys.stdin.readline().strip().lower()
            if answer == "q":
                self._stop_reason = "operator aborted at %s" % name
                self._stop.set()
                return False
            meta = {"description": description}
            if disturbance:
                meta["disturbance"] = disturbance
            if block_label:
                meta["block"] = block_label
            self.log.phase(name, **meta)
            rospy.loginfo("marked phase %s", name)
        if block_label:
            self.log.phase("block_end", block=block_label)
        return True

    def run_s2(self):
        self.log.phase("scenario_start", scenario="s2",
                       controller=self.args.controller,
                       note="closed loop: yaw and depth only. Horizontal "
                            "translation is OPEN LOOP teleop; this vehicle has "
                            "no xyz position controller "
                            "(firmware/agent/dvl_position.h, open circuit since "
                            "2026-09-04). The gripper is operator driven; the "
                            "8D action space has no gripper channel.",
                       target_attitude_deg=[0.0, 0.0])
        return self.run_marked(S2_PHASES)

    def run_s3(self):
        objects = load_objects(self.args.objects)
        self.log.phase("scenario_start", scenario="s3",
                       controller=self.args.controller,
                       n_objects=len(objects),
                       hold_s=self.args.hold_s,
                       target_attitude_deg=[0.0, 0.0])
        for obj in objects:
            if self._stop.is_set():
                return False
            # The payload's physical description goes in as its own event, so
            # the analysis can axis the comparison on mass / volume / CoG
            # offset without a side table. YCB (Calli et al. 2015) motivates
            # choosing objects that span shape, size, weight and rigidity, but
            # that set is axed for GRASP difficulty; no benchmark found in the
            # survey measures vehicle attitude while grasping, so the
            # disturbance axes are re-derived here.
            self.log.event("payload", obj.get("id", "?"), **obj)
            rospy.loginfo("=== object %s ===", obj.get("id"))
            if not self.run_marked(S3_PHASES, block_label=obj.get("id"),
                                   block_meta={"object": obj}):
                return False
        return True

    def run(self):
        if not self.preflight():
            self.log.phase("scenario_refused", reason="preflight")
            return 2
        try:
            ok = {"s1": self.run_s1, "s2": self.run_s2,
                  "s3": self.run_s3}[self.args.scenario]()
        except KeyboardInterrupt:
            self.log.phase("scenario_abort", reason="KeyboardInterrupt")
            rospy.logwarn("aborted by operator. The partial run is still "
                          "segmented; the marks are in the bag.")
            return 130
        self.log.phase("scenario_end", completed=bool(ok),
                       stop_reason=self._stop_reason)
        return 0 if ok else 1


def load_objects(path):
    """Payload descriptions for S3. See objects.example.json beside this file."""
    if not path:
        raise SystemExit(
            "s3 needs --objects <file.json>. The payload's mass, volume and "
            "CoG offset ARE the independent variable; running without them "
            "produces holds that cannot be compared to each other.")
    with open(path) as f:
        objects = json.load(f)
    required = ("id", "mass_g", "volume_ml")
    for obj in objects:
        missing = [k for k in required if k not in obj]
        if missing:
            raise SystemExit("object %r is missing %s"
                             % (obj.get("id", "?"), ", ".join(missing)))
    return objects


def parse_args(argv):
    p = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    p.add_argument("--scenario", required=True, choices=("s1", "s2", "s3"))
    p.add_argument("--controller", default="rl", choices=("rl", "tdc"),
                   help="which attitude controller is under test")
    p.add_argument("--hold-s", type=float, default=30.0, dest="hold_s",
                   help="seconds per step / hold. 30 s is about 19 periods of "
                        "the 0.6233 Hz vehicle attitude mode (finding/145), so "
                        "a settled mean is not one swing of it")
    p.add_argument("--baseline-s", type=float, default=30.0, dest="baseline_s",
                   help="in-session noise floor duration")
    p.add_argument("--amplitudes", type=float, nargs="+",
                   default=[10.0, 20.0, 30.0],
                   help="step amplitudes in deg. 30 is the edge of the trained "
                        "envelope; the start gate refuses a START above 45")
    p.add_argument("--axes", nargs="+", default=["roll", "pitch"],
                   choices=["roll", "pitch"])
    p.add_argument("--repeats", type=int, default=3,
                   help="repetitions of the whole ladder. Lynnerup 2019 uses "
                        "10 per condition on real hardware; if the tank "
                        "session cannot afford that, SAY SO in the paper -- "
                        "that paper's section 3.1 grants the exemption")
    p.add_argument("--shuffle", action="store_true",
                   help="randomise step order within each repeat, so amplitude "
                        "is not confounded with session drift")
    p.add_argument("--seed", type=int, default=0,
                   help="shuffle seed, recorded in the run event")
    p.add_argument("--objects", default=None, help="s3 payload description JSON")
    return p.parse_args(argv)


def main():
    args = parse_args(rospy.myargv(argv=sys.argv)[1:])
    rospy.init_node("scenario_runner")
    sys.exit(ScenarioRunner(args).run())


def demo():
    """Offline check of the sequence builders, no ROS:

        python scenario_runner.py --self-test
    """
    steps = s1_steps([10.0, 20.0], ["roll"], 5.0, 2, False, 0)
    # 2 amplitudes x 2 signs x 2 repeats = 8 steps, each followed by a return
    assert len(steps) == 16, len(steps)
    assert steps[0][0] == "step_roll_+10" and steps[1][0] == "return_zero", steps[:2]
    assert steps[0][1] == 10.0 and steps[0][2] == 0.0, steps[0]
    assert steps[-1][4]["repeat"] == 1, steps[-1]

    pitch = s1_steps([30.0], ["pitch"], 5.0, 1, False, 0)
    assert pitch[0][1] == 0.0 and pitch[0][2] == 30.0, pitch[0]

    # shuffle must reorder WITHOUT losing or inventing steps
    a = s1_steps([10.0, 20.0, 30.0], ["roll", "pitch"], 5.0, 1, False, 0)
    b = s1_steps([10.0, 20.0, 30.0], ["roll", "pitch"], 5.0, 1, True, 7)
    assert sorted(s[0] for s in a) == sorted(s[0] for s in b)
    assert [s[0] for s in a] != [s[0] for s in b], "seed 7 did not reorder"

    # phase names must be unique inside a list, or two segments merge silently
    for phases in (S2_PHASES, S3_PHASES):
        names = [p[0] for p in phases]
        assert len(names) == len(set(names)), names

    # the descend/ascend disturbance label must survive: it is what keeps the
    # controller comparison fair when one vertical thruster is dead
    dis = dict((p[0], p[2]) for p in S2_PHASES)
    assert dis["descend"] == "single_sided_heave_m0_only_m3_dead", dis
    assert dis["ascend"] == dis["descend"]
    assert dis["surface_hold"] is None, "the baseline phase must be undisturbed"

    try:
        make_setpoint("nope")
    except ValueError:
        pass
    else:
        raise AssertionError("unknown controller was accepted")

    print("scenario_runner: OK")


if __name__ == "__main__":
    if "--self-test" in sys.argv:
        demo()
    else:
        main()
