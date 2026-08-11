"""Regression guard for thruster_mixer's channel-axis invariant.

WHY THIS EXISTS
---------------
2026-08-11: the mixer's default `thruster_order` was [4,0,1,5,2,3] -- the SAME
permutation the sim already applies to its TAM columns
(`_reorder_columns(_BASE_ALLOCATION_MATRIX, _ESC_CHANNEL_ORDER)`, envs/main/
config.py). Applying it twice routed the m4/m5 HORIZONTAL commands into the
m0/m3 VERTICAL motors -- an uncommanded dive.

The mixer's own axis assert could not catch it: SIM_VERT was frozenset((4,5)),
which are `_BASE_ALLOCATION_MATRIX` indices the board never sees. Against the
LIVE matrix the vertical columns are 0 and 3 (the deployed teacher run's
params/env.yaml records Fz = (1,0,0,1,0,0)). So the wrong order passed AND the
correct identity order raised ROSInitException. Nothing errored for months only
because thruster_scale defaulted to 0.0 and no recorded run ever raised it.

This test pins both directions so the pairing cannot silently drift again.

Imports the constants with `ast` rather than `import thruster_mixer`, because
the module imports rospy at load time and this must run on a dev machine.
Run: pytest test_thruster_mixer_axes.py   (or: python test_thruster_mixer_axes.py)
"""
import ast
import os

MIXER = os.path.join(os.path.dirname(os.path.abspath(__file__)), "thruster_mixer.py")

IDENTITY = [0, 1, 2, 3, 4, 5]
DOUBLE_PERMUTED = [4, 0, 1, 5, 2, 3]  # the pre-2026-08-11 dive default


def _constants():
    """Module-level literals from thruster_mixer.py, without importing rospy."""
    out = {}
    for node in ast.parse(open(MIXER).read()).body:
        if not (isinstance(node, ast.Assign) and isinstance(node.targets[0], ast.Name)):
            continue
        name, value = node.targets[0].id, node.value
        try:
            out[name] = ast.literal_eval(value)
        except ValueError:
            # frozenset((...)) is a Call, not a literal
            if isinstance(value, ast.Call) and getattr(value.func, "id", "") == "frozenset":
                out[name] = frozenset(ast.literal_eval(value.args[0]))
    return out


def _accepts(c, order):
    """Mirror of ThrusterMixer._validate_order's axis predicate."""
    if sorted(order) != list(range(c["NUM_THR"])):
        return False
    vert_src = set(order[ch] for ch in c["FW_VERT_CH"])
    horz_src = set(order[ch] for ch in c["FW_HORZ_CH"])
    return vert_src <= c["SIM_VERT"] and horz_src == c["SIM_HORZ"]


def test_sim_axis_sets_are_live_tam_indices():
    """SIM_VERT/SIM_HORZ must index the LIVE TAM, i.e. equal the firmware wiring.

    If these ever go back to {4,5}/{0,1,2,3} the assert is comparing against
    _BASE_ALLOCATION_MATRIX again and stops protecting anything.
    """
    c = _constants()
    assert c["SIM_VERT"] == set(c["FW_VERT_CH"]), "vertical sim indices must match fw wiring"
    assert c["SIM_HORZ"] == set(c["FW_HORZ_CH"]), "horizontal sim indices must match fw wiring"
    assert c["SIM_VERT"] | c["SIM_HORZ"] == set(range(c["NUM_THR"]))
    assert not (c["SIM_VERT"] & c["SIM_HORZ"])


def test_identity_is_the_default_and_is_accepted():
    """Sim already reordered to firmware channels, so the mixer must pass through."""
    c = _constants()
    assert c["DEFAULT_ORDER"] == IDENTITY
    assert _accepts(c, IDENTITY)


def test_double_permutation_is_refused():
    """The old default sent m4/m5 horizontal commands to the m0/m3 vertical motors."""
    c = _constants()
    assert not _accepts(c, DOUBLE_PERMUTED)
    # spell out the failure it caused, so a future reader sees the consequence
    assert DOUBLE_PERMUTED[0] in c["SIM_HORZ"], "m0 (vertical) would have sourced a horizontal cmd"
    assert DOUBLE_PERMUTED[3] in c["SIM_HORZ"], "m3 (vertical) would have sourced a horizontal cmd"


def test_axis_crossing_orders_are_refused():
    c = _constants()
    assert not _accepts(c, [1, 0, 2, 3, 4, 5]), "m0 sourcing a horizontal channel must be refused"
    assert not _accepts(c, [0, 1, 2, 4, 3, 5]), "m3 sourcing a horizontal channel must be refused"
    assert not _accepts(c, [0, 1, 2, 3, 4]), "wrong length must be refused"
    assert not _accepts(c, [0, 0, 2, 3, 4, 5]), "non-permutation must be refused"


if __name__ == "__main__":
    for _name, _fn in sorted(globals().items()):
        if _name.startswith("test_"):
            _fn()
            print("PASS %s" % _name)
