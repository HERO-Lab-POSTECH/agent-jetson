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
# Physical positions from the dry channel probe (2026-08-11 evening) matched to the
# live sim columns, which come from _ESC_CHANNEL_ORDER = (4,0,1,5,2,3) -- in the repo
# since 2026-07-03 and unchanged, so it is what the deployed teacher trained on.
# The 2026-08-11 value [3,2,4,0,5,1] used a mis-recalled tuple (4,1,3,5,2,0).
GEOMETRIC = [3, 5, 4, 0, 1, 2]
MISRECALLED = [3, 2, 4, 0, 5, 1]  # superseded 2026-08-12; axis-legal but wrong sources


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


def test_geometric_order_is_the_default_and_is_accepted():
    """The default must be the geometry-derived order, not identity.

    Identity assumed the sim-side _ESC_CHANNEL_ORDER already matched the wiring.
    Driving each firmware channel alone and locating the spinning propeller showed
    all four horizontals 90 deg out and the two verticals swapped, so the mixer has
    real work to do. Identity stays axis-legal (it never crosses the split) -- it is
    just wrong, which is exactly why it needs pinning here rather than in the assert.
    """
    c = _constants()
    assert c["DEFAULT_ORDER"] == GEOMETRIC
    assert _accepts(c, GEOMETRIC)
    assert _accepts(c, IDENTITY), "identity is axis-legal; only the default changed"


def test_misrecalled_order_is_axis_legal_but_is_not_the_default():
    """[3,2,4,0,5,1] passes the axis assert yet routes three horizontals wrongly.

    This is the whole reason the default needs pinning by a test: the startup
    assertion only guards the vertical/horizontal split, and this order keeps that
    split intact (m0<-3, m3<-0). It differs from the correct one by a 3-cycle among
    the horizontal sources, which in the water shows up as translation in the wrong
    direction -- and NOT as a yaw error, because Mz is identical for all four
    horizontals. Nothing downstream would have complained.
    """
    c = _constants()
    assert _accepts(c, MISRECALLED), "it was axis-legal, which is why it survived"
    assert c["DEFAULT_ORDER"] != MISRECALLED
    moved = [j for j in range(6) if MISRECALLED[j] != GEOMETRIC[j]]
    assert moved == [1, 4, 5], "only the three horizontal sources moved"


def _undeadband_fn():
    """Exec the shipped undeadband() alone, so the math under test is the real one.

    thruster_mixer.py imports rospy at module scope, so it cannot be imported here.
    Pulling just this one pure function out of the AST keeps the test honest: a
    re-implementation would pass even if the shipped formula were wrong.
    """
    tree = ast.parse(open(MIXER).read())
    for node in tree.body:
        if isinstance(node, ast.FunctionDef) and node.name == "undeadband":
            ns = {}
            try:                       # py3.8+ requires type_ignores; py2.7 rejects it
                mod = ast.Module(body=[node], type_ignores=[])
            except TypeError:
                mod = ast.Module(body=[node])
            exec(compile(mod, MIXER, "exec"), ns)
            return ns["undeadband"]
    raise AssertionError("undeadband() not found in thruster_mixer.py")


def test_deadband_inverse_preserves_zero_and_full_scale():
    """a=0 must stay exactly neutral and a=+-1 must keep full authority.

    Zero is the load-bearing case: the whole reason for this compensation is that
    the old firmware parked the verticals off-neutral and the ESC crept. A
    compensation that pushed a=0 off zero would reintroduce exactly that.
    """
    f, D = _undeadband_fn(), 0.15
    assert f(0.0, D) == 0.0
    assert abs(f(1.0, D) - 1.0) < 1e-9
    assert abs(f(-1.0, D) + 1.0) < 1e-9


def test_deadband_inverse_lifts_small_commands_clear_of_the_dead_zone():
    """Any non-trivial command must land strictly outside the measured dead zone."""
    f, D = _undeadband_fn(), 0.15
    for a in (0.01, 0.05, 0.1, 0.5, -0.01, -0.2):
        out = f(a, D)
        assert abs(out) > D, "|%.3f| -> |%.3f| still inside the dead zone" % (a, out)
        assert (out > 0) == (a > 0), "sign must survive the inverse"


def test_deadband_zero_is_a_passthrough():
    """0.0 disables compensation -- the escape hatch for pre-reflash firmware."""
    f = _undeadband_fn()
    for a in (0.05, 0.5, -0.7, 1.0):
        assert f(a, 0.0) == a


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
