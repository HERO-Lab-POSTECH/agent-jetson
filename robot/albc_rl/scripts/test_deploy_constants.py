"""Deployment-constant guard: derive the constants, do not trust them.

WHY THIS EXISTS
---------------
Two constants on this robot are shared across the whole stack and were each
wrong for weeks without anything complaining:

  imu_yaw_offset   45.0 -> -78.0 (2026-08-11) -> +102.0 (2026-08-12).
                   Lives in TEN places; the yaml is the runtime SSOT and the rest
                   are fallbacks that silently keep whatever they were last set to.
  thruster_order   four wrong values in a row before [3,2,4,0,5,1] stuck.

Every one of those failures is the SAME failure: somebody read a constant out of
a source tree and assumed it described the deployed system. A source tree cannot
tell you what a checkpoint learned, and a fallback cannot tell you what the yaml
says. So this file never trusts a constant -- it re-derives each one from an
artifact and asserts the constants agree.

  thruster_order : derived from deployed_tam.json (the deployed checkpoint's own
                   allocation matrix) by inverting M = r x F, then matched to the
                   measured firmware channel map BY PHYSICAL POSITION.
  imu_yaw_offset : the yaml is authoritative; every other site must equal it.

Pure stdlib, no ROS, no numpy -- runs on the board and on a dev machine.
Run: pytest test_deploy_constants.py   (or: python test_deploy_constants.py)
"""
import ast
import json
import math
import os
import re
import xml.etree.ElementTree as ET

HERE = os.path.dirname(os.path.abspath(__file__))
ROBOT = os.path.normpath(os.path.join(HERE, "..", ".."))   # .../robot
TAM_JSON = os.path.join(HERE, "deployed_tam.json")
MIXER = os.path.join(HERE, "thruster_mixer.py")

# Every site that carries an imu_yaw_offset number, and how to pull it out.
# path relative to robot/, regex whose group(1) is the number.
OFFSET_SITES = [
    ("albc_control/config/albc_controller.yaml", r"^imu_yaw_offset:\s*([-+0-9.]+)"),
    ("albc_control/cfg/ALBCController.cfg", r'gen\.add\("imu_yaw_offset".*?,\s*([-+0-9.]+),\s*-180'),
    ("albc_rl/cfg/GyroOffset.cfg", r"^\s*([-+0-9.]+),\s*-180\.0,\s*180\.0\)"),
    ("albc_rl/scripts/rl_inference_node.py", r'get_param\("~imu_yaw_offset_deg",\s*([-+0-9.]+)\)'),
    ("albc_control/src/albc_controller.cpp", r'param<double>\("imu_yaw_offset",\s*\w+,\s*([-+0-9.]+)\)'),
    ("hero_agent/src/agent.cpp", r'param<double>\("/albc_controller/imu_yaw_offset",\s*\w+,\s*([-+0-9.]+)\)'),
    ("hero_agent/include/hero_agent/state_monitor.h", r"imu_yaw_offset_rad\s*=\s*([-+0-9.]+)\s*\*\s*M_PI"),
    ("albc_rl/launch/albc_rl.launch", r'name="imu_yaw_offset_deg"\s+default="([-+0-9.]+)"'),
    ("albc_rl/launch/albc_rl_fieldtest.launch", r'name="imu_yaw_offset_deg"\s+default="([-+0-9.]+)"'),
    ("albc_control/scripts/measurement/tilt_azimuth.py", r'"--offset-deg",\s*type=float,\s*default=([-+0-9.]+)'),
]
YAML_SITE = OFFSET_SITES[0][0]


def _read(rel):
    with open(os.path.join(ROBOT, rel)) as f:
        return f.read()


def _tam():
    with open(TAM_JSON) as f:
        return json.load(f)


def _mixer_default_order():
    """DEFAULT_ORDER out of thruster_mixer.py without importing rospy."""
    tree = ast.parse(open(MIXER).read())
    for node in tree.body:
        if isinstance(node, ast.Assign):
            for t in node.targets:
                if isinstance(t, ast.Name) and t.id == "DEFAULT_ORDER":
                    return ast.literal_eval(node.value)
    raise AssertionError("DEFAULT_ORDER not found in thruster_mixer.py")


def _hour(x, y):
    """Clock position of (x, y). +x = 3 o'clock, +y = 12 o'clock, clockwise."""
    a = math.degrees(math.atan2(y, x))
    h = (3.0 - a / 30.0) % 12.0
    return 12.0 if abs(h) < 1e-6 else round(h, 1)


def _column_positions(tam):
    """Invert M = r x F for every column. Returns {col: (hour, axis)}.

    Mz = x*Fy - y*Fx  pins a horizontal thruster onto the +-arm diagonal grid;
    My = -x*Fz        pins a vertical one onto the +-arm x-axis.
    The solution on the grid is unique -- that uniqueness is asserted below, so a
    matrix that does not correspond to this geometry fails loudly instead of
    silently picking one.
    """
    A = tam["allocation_matrix"]
    ah = tam["geometry"]["horizontal_arm_m"]
    out = {}
    for j in range(6):
        Fx, Fy, Fz, Mz, My = A["Fx"][j], A["Fy"][j], A["Fz"][j], A["Mz"][j], A["My"][j]
        if abs(Fz) > 0.5:
            out[j] = (_hour(-My / Fz, 0.0), "vertical")
            continue
        hits = [(sx * ah, sy * ah) for sx in (1, -1) for sy in (1, -1)
                if abs((sx * ah) * Fy - (sy * ah) * Fx - Mz) < 1e-3]
        assert len(hits) == 1, "column %d has %d grid solutions, not 1" % (j, len(hits))
        out[j] = (_hour(*hits[0]), "horizontal")
    return out


def _derive_order(tam):
    """fw channel j -> sim column, matched by physical position."""
    cols = _column_positions(tam)
    fw = tam["measured_channel_map"]
    order = [None] * 6
    for j in range(6):
        hour, axis = fw["m%d" % j]["hour"], fw["m%d" % j]["axis"]
        match = [c for c, (h, a) in cols.items() if a == axis and abs(h - hour) < 0.2]
        assert len(match) == 1, \
            "m%d (%s %.1fh) matched %d sim columns, not 1" % (j, axis, hour, len(match))
        order[j] = match[0]
    assert sorted(order) == list(range(6)), "derived order is not a permutation: %s" % order
    return order


# --------------------------------------------------------------------------
# thruster_order
# --------------------------------------------------------------------------

def test_order_derived_from_checkpoint_matches_the_mixer():
    """The one that would have caught all four wrong values."""
    tam = _tam()
    derived = _derive_order(tam)
    assert derived == tam["expected_thruster_order"], (
        "deployed_tam.json's expected_thruster_order (%s) disagrees with what its own "
        "matrix + channel map imply (%s). One of the three was edited alone."
        % (tam["expected_thruster_order"], derived))
    assert derived == _mixer_default_order(), (
        "thruster_mixer.DEFAULT_ORDER (%s) is not what the deployed checkpoint's matrix "
        "implies (%s). Do NOT 'fix' this by editing the JSON to match the mixer -- the "
        "JSON is the artifact, the mixer is the claim." % (_mixer_default_order(), derived))


def test_vertical_commands_cannot_reach_horizontal_motors():
    """The mis-edit that dives the boat, independent of which permutation is right."""
    tam = _tam()
    cols = _column_positions(tam)
    order = _derive_order(tam)
    for j in range(6):
        assert cols[order[j]][1] == tam["measured_channel_map"]["m%d" % j]["axis"], \
            "fw m%d would be driven by a %s sim column" % (j, cols[order[j]][1])


def test_channel_map_declares_its_frame():
    """A bearing without a stated frame is unusable; 2026-07-05 proved it."""
    frame = _tam()["measured_channel_map"].get("_frame", "")
    assert "GRIPPER" in frame.upper() and "12" in frame, \
        "measured_channel_map._frame must name the reference explicitly"


def test_deployed_tam_carries_its_provenance():
    """So 'which checkpoint is this?' is never answered from memory."""
    p = _tam()["provenance"]
    for k in ("run_path", "checkpoint", "checkpoint_sha256", "source_file_sha256", "trained"):
        assert p.get(k), "provenance.%s is missing" % k
    assert len(p["checkpoint_sha256"]) == 64


# --------------------------------------------------------------------------
# imu_yaw_offset
# --------------------------------------------------------------------------

def _offset_at(rel, pattern):
    m = re.search(pattern, _read(rel), re.M | re.S)
    assert m, "no imu_yaw_offset match in %s" % rel
    return float(m.group(1))


def test_every_imu_yaw_offset_site_equals_the_yaml():
    """Ten sites, one value. A fallback that drifts is invisible until it is used."""
    truth = _offset_at(*OFFSET_SITES[0])
    bad = []
    for rel, pat in OFFSET_SITES[1:]:
        v = _offset_at(rel, pat)
        if abs(v - truth) > 1e-9:
            bad.append("%s = %s" % (rel, v))
    assert not bad, (
        "imu_yaw_offset drifted from %s (%s):\n  %s" % (YAML_SITE, truth, "\n  ".join(bad)))


def test_imu_yaw_offset_is_in_range():
    v = _offset_at(*OFFSET_SITES[0])
    assert -180.0 <= v <= 180.0, "outside the dynamic_reconfigure range: %s" % v


LAUNCH_DIR = os.path.join(ROBOT, "albc_rl", "launch")


def test_exactly_one_thruster_mixer_per_launch():
    """No launch may start two nodes named thruster_mixer.

    ROS does not error on a duplicate node name -- the later registration silently
    unregisters the earlier one, so one mixer dies with nothing in the log that reads
    like a fault. albc_rl.launch runs a mixer of its own (so running it standalone is
    not the silent no-op it used to be) and albc_rl_fieldtest.launch includes that file
    AND runs its own parameterised mixer, which is exactly the collision. The child's
    mixer is therefore gated on launch_mixer, and any parent that owns a mixer must
    pass launch_mixer=false. This test is the thing that notices when a new parent
    forgets, since the symptom at runtime is a node quietly missing.
    """
    def mixers(root):
        return [n for n in root.iter("node") if n.get("name") == "thruster_mixer"]

    # the child's own mixer must stay gated, or the arg becomes decorative
    child = ET.parse(os.path.join(LAUNCH_DIR, "albc_rl.launch")).getroot()
    gated = [n for g in child.iter("group")
             if "launch_mixer" in (g.get("if") or "") for n in mixers(g)]
    assert len(mixers(child)) == len(gated) == 1, (
        "albc_rl.launch's thruster_mixer must sit inside <group if=\"$(arg launch_mixer)\">")

    bad = []
    for fn in sorted(os.listdir(LAUNCH_DIR)):
        if not fn.endswith(".launch") or fn == "albc_rl.launch":
            continue
        root = ET.parse(os.path.join(LAUNCH_DIR, fn)).getroot()
        includes = [i for i in root.iter("include")
                    if "albc_rl.launch" in (i.get("file") or "")]
        if not includes or not mixers(root):
            continue
        for inc in includes:
            passed = {a.get("name"): (a.get("value") or "").strip().lower()
                      for a in inc.iter("arg")}
            if passed.get("launch_mixer") != "false":
                bad.append(fn)
    assert not bad, (
        "these launches own a thruster_mixer AND include albc_rl.launch without "
        "launch_mixer:=false -> duplicate node name: %s" % ", ".join(bad))


# --------------------------------------------------------------- start-pose gate
# RL-DEPLOY 2026-08-17. The 2026-08-13 cable break began with the policy winding J1
# past 2*pi from a start sim never shows it: randomize_joint_positions writes theta1
# to uniform(-pi, pi) at EVERY episode reset, so the trained +-2-turn budget is
# measured from near zero. On the real arm nothing resets it. Two defences ship
# together and both are easy to un-ship by editing one default, which is what these
# tests watch.

NODE_REL = "albc_rl/scripts/rl_inference_node.py"
RL_LAUNCHES = ("albc_rl.launch", "albc_rl_fieldtest.launch")


def _launch_arg(fn, name):
    root = ET.parse(os.path.join(LAUNCH_DIR, fn)).getroot()
    for a in root.findall("arg"):
        if a.get("name") == name:
            return (a.get("default") or "").strip()
    raise AssertionError("%s declares no <arg name=%r>" % (fn, name))


def test_home_on_start_defaults_to_false_everywhere():
    """Homing COMMANDS the arm and cannot unwind a wound one.

    _home_arm drives toward joint1 0, which the driver reaches at the NEAREST
    0-equivalent -- so from -11 rad it does not unwind, it just stops somewhere and,
    on timeout, warns and proceeds anyway. Until the restart round-trip regression
    test passes, arm motion before the policy is opt-in. Three sites carry the
    default and a run picks up whichever one is nearest, so all three must agree.
    """
    bad = [fn for fn in RL_LAUNCHES if _launch_arg(fn, "home_on_start").lower() != "false"]
    assert not bad, "home_on_start must default to false in: %s" % ", ".join(bad)
    m = re.search(r'get_param\("~home_on_start",\s*(\w+)\)', _read(NODE_REL))
    assert m and m.group(1) == "False", (
        "rl_inference_node's own default must be False too -- a bare rosrun does not "
        "read the launch file (found %s)" % (m.group(1) if m else "no call"))


def test_joint1_start_gate_default_is_pi():
    """pi, because that is the band sim actually trained over.

    Not a safety margin picked by feel: uniform(-pi, pi) is literally the reset
    distribution. Widening this is a deliberate experiment, so it must be a per-run
    override, never a committed default.
    """
    m = re.search(r'get_param\("~joint1_start_max_rad",\s*([^)]+)\)', _read(NODE_REL))
    assert m, "rl_inference_node no longer reads ~joint1_start_max_rad"
    assert "np.pi" in m.group(1), "node default must be np.pi, found %s" % m.group(1)
    for fn in RL_LAUNCHES:
        v = float(_launch_arg(fn, "joint1_start_max_rad"))
        assert abs(v - math.pi) < 1e-9, "%s defaults to %s, not pi" % (fn, v)


def test_start_pose_gate_is_wired_not_decorative():
    """An <arg> nobody passes down is a comment with XML syntax.

    albc_rl.launch must turn the arg into a <param> on the node; the fieldtest parent
    must forward it through the <include>. Either half missing and the gate silently
    runs on its code default while the launch file advertises a knob.
    """
    child = ET.parse(os.path.join(LAUNCH_DIR, "albc_rl.launch")).getroot()
    params = {p.get("name") for n in child.iter("node") for p in n.iter("param")}
    assert "joint1_start_max_rad" in params, (
        "albc_rl.launch declares the arg but never sets it as a <param> on the node")

    parent = ET.parse(os.path.join(LAUNCH_DIR, "albc_rl_fieldtest.launch")).getroot()
    fwd = [a for i in parent.iter("include")
           if "albc_rl.launch" in (i.get("file") or "")
           for a in i.iter("arg") if a.get("name") == "joint1_start_max_rad"]
    assert fwd, ("albc_rl_fieldtest.launch does not forward joint1_start_max_rad "
                 "through its <include> -- launch-rl would ignore the arg")


def test_gate_runs_before_homing_and_before_the_timer():
    """Order is the whole defence, and nothing at runtime would complain about it.

    Homing below the gate would command a wound arm; the Timer below neither would
    let the policy tick from a wound start. Both reorderings launch cleanly and look
    correct in the log, which is why this is pinned in a test rather than a comment.
    """
    src = _read(NODE_REL)
    body = src[src.index("def __init__"):src.index("def _gate_start_pose")]
    for marker in ("self._gate_start_pose()", '"~home_on_start"', "rospy.Timer("):
        assert marker in body, "%s left __init__ -- this test no longer guards it" % marker
    gate = body.index("self._gate_start_pose()")
    assert gate < body.index('"~home_on_start"'), "start-pose gate must run BEFORE homing"
    assert gate < body.index("rospy.Timer("), "start-pose gate must run BEFORE the timer"
    assert "rospy.signal_shutdown" in body, (
        "a failed gate must stop the node, not just log")


if __name__ == "__main__":
    fails = 0
    for name, fn in sorted(globals().items()):
        if name.startswith("test_") and callable(fn):
            try:
                fn()
                print("PASS %s" % name)
            except AssertionError as e:
                fails += 1
                print("FAIL %s\n     %s" % (name, e))
    raise SystemExit(1 if fails else 0)
