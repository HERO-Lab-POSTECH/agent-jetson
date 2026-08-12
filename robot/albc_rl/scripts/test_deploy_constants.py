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
