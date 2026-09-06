#!/usr/bin/env python
"""Guard the recording contract. stdlib + albc_rl only, so it runs anywhere.

    python test_run_record.py

WHAT THIS CATCHES, and why each check exists rather than being obvious:

  1. A TOPIC THAT NOBODY BAGS. Every topic in contract.TOPICS must be either in
     run_record.launch's recorder list or named in EXEMPT with a reason. That
     is precisely the defect this change fixes: /albc/status and
     /hero_agent/state were published, subscribed, and never recorded, so the
     baseline controller's setpoint and the entire depth axis existed only in
     RAM. Nothing failed. The bags just quietly had no answer in them.

  2. A DOUBLE HYPHEN IN AN XML COMMENT. XML forbids it and roslaunch parses
     strictly, so one stray pair makes every launch that includes the file fail
     to parse. Hit while authoring run_record.launch; kept here because its
     comments are long and prose-like, which is the shape that grows one.

  3. A LAUNCH THAT PRODUCES DATA WITHOUT RECORDING IT. Each launch below must
     include run_record.launch. albc.launch went the whole project without one.

  4. SCHEMA DRIFT. run_log's validators, exercised on a manifest shaped the way
     the nodes actually build one.

Deliberately STATIC: it parses files instead of running ROS, so the dev machine
can run it. A recording guard that only works on the board is a guard that runs
the day after the data was lost.
"""
import os
import re
import sys
import xml.dom.minidom

_HERE = os.path.dirname(os.path.abspath(__file__))
_PKG = os.path.dirname(_HERE)
sys.path.insert(0, os.path.join(_PKG, "src"))

from albc_rl import contract                                    # noqa: E402
from albc_rl import run_log                                     # noqa: E402

LAUNCH_DIR = os.path.join(_PKG, "launch")
RECORD_LAUNCH = os.path.join(LAUNCH_DIR, "run_record.launch")

# Topics that are legitimately NOT recorded, each with its reason. Adding a
# name here is a decision; leaving one out by accident is the bug.
EXEMPT = {
    "command": "/hero_agent/command is a firmware INPUT; what the operator "
               "actually sent is on key_input, which is recorded",
}


def _recorded_topics(path):
    """Topic names out of the rosbag record node's args attribute."""
    dom = xml.dom.minidom.parse(path)
    for node in dom.getElementsByTagName("node"):
        if node.getAttribute("type") == "record":
            args = node.getAttribute("args")
            # -o's value is a path, not a topic; drop anything that is not a
            # bare ROS name (no dots, no directory that is not a topic).
            found = re.findall(r"(?<![\w.])/[\w/]+", args)
            return set(t for t in found if "albc_bags" not in t)
    raise AssertionError("no rosbag record node in %s" % path)


def check_every_topic_is_recorded():
    recorded = _recorded_topics(RECORD_LAUNCH)
    missing = []
    for key, topic in sorted(contract.TOPICS.items()):
        if topic in recorded or key in EXEMPT:
            continue
        missing.append("%s (contract.TOPICS[%r])" % (topic, key))
    assert not missing, (
        "these contract.TOPICS are published but NOT recorded, so a bag cannot "
        "answer questions about them: %s. Add each to run_record.launch, or to "
        "EXEMPT with a reason." % ", ".join(missing))

    # The two topics the whole run record rests on. If either drops out, every
    # bag silently loses its provenance and its segment marks, and looks fine.
    for topic in (run_log.META_TOPIC, run_log.EVENT_TOPIC):
        assert topic in recorded, "%s is not in the recorder list" % topic
    return recorded


def check_no_double_hyphen_in_comments():
    for name in sorted(os.listdir(LAUNCH_DIR)):
        if not name.endswith(".launch"):
            continue
        path = os.path.join(LAUNCH_DIR, name)
        with open(path) as f:
            text = f.read()
        for comment in re.findall(r"<!--(.*?)-->", text, re.S):
            assert "--" not in comment, (
                "%s has a double hyphen inside an XML comment; roslaunch's "
                "parser rejects the whole file. Use a colon or a single dash."
                % name)
        xml.dom.minidom.parse(path)          # and it must actually parse


def check_data_launches_record():
    """Every launch that brings a controller up must bring the recorder up."""
    robot_dir = os.path.dirname(_PKG)
    expect = [
        (os.path.join(LAUNCH_DIR, "albc_rl_fieldtest.launch"), "RL"),
        (os.path.join(robot_dir, "albc_control", "launch", "albc.launch"), "TDC"),
    ]
    for path, controller in expect:
        if not os.path.isfile(path):
            continue                        # a partial checkout is not a failure
        with open(path) as f:
            text = f.read()
        assert "run_record.launch" in text, (
            "%s starts a controller but never includes run_record.launch, so "
            "its runs leave no bag. That is how the %s path went the whole "
            "project unrecorded." % (os.path.basename(path), controller))

    # The operator's real TDC entry point is the shell script, not the launch
    # file, so checking only the launch would pass while every actual run went
    # unrecorded.
    sh = os.path.join(robot_dir, "albc_control", "scripts", "launch_albc.sh")
    if os.path.isfile(sh):
        with open(sh) as f:
            assert "run_record.launch" in f.read(), (
                "launch_albc.sh is what launch-albc actually runs, and it does "
                "not start the recorder")


def _resolve_includes(path, seen=None):
    """Every launch file reachable from `path`, following $(find pkg) includes."""
    seen = seen if seen is not None else []
    if path in seen or not os.path.isfile(path):
        return seen
    seen.append(path)
    robot_dir = os.path.dirname(_PKG)
    dom = xml.dom.minidom.parse(path)
    for inc in dom.getElementsByTagName("include"):
        ref = inc.getAttribute("file")
        # only $(find <pkg>)/... is used in this tree; anything else is skipped
        if not ref.startswith("$(find "):
            continue
        pkg = ref[len("$(find "):ref.index(")")]
        rest = ref[ref.index(")") + 1:].lstrip("/")
        _resolve_includes(os.path.join(robot_dir, pkg, rest), seen)
    return seen


def check_one_recorder_per_graph():
    """At most ONE rosbag record node in any launch graph an operator starts.

    ROS kills the older node of a duplicate name without an error, and a killed
    recorder leaves a .bag.active with no index -- unreadable until someone
    reindexes it, and nothing announces that it happened. run_record.launch's
    own comment cites thruster_mixer as "same hazard, same idiom", but the
    mixer has an arg gate AND test_exactly_one_thruster_mixer_per_launch, and
    the recorder had neither. There are now three paths that can start one
    (albc.launch, launch_albc.sh, albc_rl_fieldtest.launch), so this is the
    half of that idiom that was missing.
    """
    robot_dir = os.path.dirname(_PKG)
    entry_points = [
        os.path.join(LAUNCH_DIR, "albc_rl_fieldtest.launch"),
        os.path.join(LAUNCH_DIR, "albc_rl.launch"),
        os.path.join(robot_dir, "albc_control", "launch", "albc.launch"),
    ]
    for top in entry_points:
        if not os.path.isfile(top):
            continue
        n = 0
        where = []
        for f in _resolve_includes(top):
            dom = xml.dom.minidom.parse(f)
            for node in dom.getElementsByTagName("node"):
                if node.getAttribute("type") == "record":
                    n += 1
                    where.append(os.path.basename(f))
        assert n <= 1, (
            "%s resolves to %d rosbag record nodes (%s). ROS kills the older "
            "one silently and its bag is left unindexed."
            % (os.path.basename(top), n, ", ".join(where)))


def check_schema():
    run_log.demo()

    # a manifest shaped the way rl_inference_node actually builds one
    meta = run_log.build_meta(
        run_log.make_run_id("rl"), "rl_inference_node", "rl",
        scenario="s1_attitude",
        pack={"tag": "pack_r3a_p3b7500_gru_260906_145553",
              "digests": {"weights_gru.npz": "4cfc7154" + "0" * 56}},
        git={"head": "c7c9f26", "branch": "deploy/72d-inc9998-gru", "dirty": False},
        contract={"POLICY_OBS_DIM": contract.POLICY_OBS_DIM,
                  "ACTION_DIM": contract.ACTION_DIM,
                  "DELTA_SCALE": contract.DELTA_SCALE},
        initial={"joint1_rad": 0.0, "joint2_rad": 2.618, "tilt_deg": 3.2,
                 "frame": "body (rotate_imu applied)"})
    problems = run_log.validate_meta(meta)
    assert problems == [], problems
    assert meta["contract"]["POLICY_OBS_DIM"] == 72, meta["contract"]

    # every event kind the nodes emit must be legal
    for kind, name in (("phase", "policy_start"), ("setpoint", "dynparam"),
                       ("guard", "arm_guard_trip"), ("mark", "grasp"),
                       ("payload", "object_B"), ("note", "operator")):
        evt = run_log.build_event(1, kind, name, t=0.0)
        assert run_log.validate_event(evt) == [], (kind, name)

    # the controller the OTHER session is building must already be legal here,
    # or its runs land in the comparison table as "unknown controller"
    assert "thruster_only" in run_log.CONTROLLERS


def main():
    check_no_double_hyphen_in_comments()
    recorded = check_every_topic_is_recorded()
    check_data_launches_record()
    check_one_recorder_per_graph()
    check_schema()
    print("test_run_record: OK (%d topics recorded)" % len(recorded))


if __name__ == "__main__":
    main()
