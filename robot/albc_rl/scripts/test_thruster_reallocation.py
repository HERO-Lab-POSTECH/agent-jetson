"""Guard for the RL fault-tolerant reallocation (thruster_mixer.reallocate).

WHY THIS EXISTS
---------------
Before 2026-08-24 the mixer could only MUTE a channel (~thruster_sign[j] = 0).
Muting is not fault tolerance: the policy asked for a wrench and got that wrench
minus one column, so the response does not merely weaken -- it points somewhere
else. The classic firmware path had already been fixed (pid.cpp 3771674 drops one
channel per mode and the remaining two reproduce the heading exactly); the RL path
had no such layer.

What this file pins is the property that makes reallocation worth having:

    with m3 and m4 dead, the wrench the live channels REALISE equals the wrench
    the policy INTENDED, exactly, on every axis that still has rank.

and the two honest limits:

    * the vertical axis has rank 1, so My is abandoned and every heave command
      carries a pitch disturbance. That is not a bug to be tested away.
    * magnitude is capped; when the solve exceeds +-1 the whole GROUP scales down
      so direction survives. A per-channel clamp would rotate the wrench, which
      is the exact failure reallocation exists to prevent.

Plus the one that keeps the feature safe to ship: with nothing broken it must be
a bit-for-bit no-op, or turning the flag on becomes its own regression.

Imports the constants and pure functions with `ast` rather than
`import thruster_mixer`, because the module imports rospy at load time and this
must run on a dev machine. Same technique as test_thruster_mixer_axes.py, widened
to carry the module constants too (reallocate() reads NUM_THR / FW_*_CH).

Pure stdlib, no ROS, no numpy -- runs on the board's python2.7 and on a dev machine.
Run: pytest test_thruster_reallocation.py   (or: python test_thruster_reallocation.py)
"""
import ast
import json
import os

HERE = os.path.dirname(os.path.abspath(__file__))
MIXER = os.path.join(HERE, "thruster_mixer.py")
TAM_JSON = os.path.join(HERE, "deployed_tam.json")

PURE_FNS = ("solve3x3", "reallocate", "normalize_sign", "undeadband")
SAFE_IMPORTS = ("math", "json", "os")   # everything except rospy / ROS messages
AXES = ("Fx", "Fy", "Fz", "Mz")

ALL_LIVE = [1, 1, 1, 1, 1, 1]
M4_DEAD = [1, 1, 1, 1, 0, 1]           # the 2026-08-24 operator decision
M3_M4_DEAD = [1, 1, 1, 0, 0, 1]        # what deployed_tam.json's health map implies


def _ns():
    """The mixer's module constants + pure functions, without importing rospy."""
    tree = ast.parse(open(MIXER).read())
    body = []
    for node in tree.body:
        if isinstance(node, ast.Import):
            if all(a.name in SAFE_IMPORTS for a in node.names):
                body.append(node)
        elif isinstance(node, ast.Assign) and isinstance(node.targets[0], ast.Name):
            body.append(node)
        elif isinstance(node, ast.FunctionDef) and node.name in PURE_FNS:
            body.append(node)
    ns = {}
    try:                       # py3.8+ requires type_ignores; py2.7 rejects it
        mod = ast.Module(body=body, type_ignores=[])
    except TypeError:
        mod = ast.Module(body=body)
    exec(compile(mod, MIXER, "exec"), ns)
    return ns


def _alloc():
    return json.load(open(TAM_JSON))["allocation_matrix"]


def _intended(alloc, action):
    """The wrench the policy meant, over all six sim columns."""
    return dict((ax, sum(alloc[ax][i] * action[i] for i in range(6))) for ax in AXES)


def _realised(alloc, out, order, sign):
    """The wrench the live channels actually produce from a fw-indexed command."""
    return dict((ax, sum(alloc[ax][order[j]] * out[j]
                         for j in range(6) if sign[j] != 0.0)) for ax in AXES)


def _close(a, b, tol=1e-9):
    return abs(a - b) <= tol


def test_no_fault_is_a_bit_for_bit_no_op():
    """With nothing broken, reallocation must equal the plain permutation.

    This is the safety property that lets the flag be flipped at all: if turning
    it on changed behaviour on a healthy robot, every tank result before and
    after would be incomparable.
    """
    ns, alloc = _ns(), _alloc()
    order = ns["DEFAULT_ORDER"]
    sign = ns["normalize_sign"](ALL_LIVE)
    action = [0.3, -0.7, 0.2, -0.1, 0.9, -0.4]
    out = ns["reallocate"](action, order, sign, alloc)
    for j in range(6):
        assert out[j] == action[order[j]], "channel m%d diverged with nothing broken" % j


def test_order_is_not_applied_twice():
    """reallocate() consumes `order` itself; the caller must not permute again.

    Double permutation is this repo's most expensive recurring bug (it routed
    horizontal commands into the vertical motors and would have dived the boat).
    A distinct value per channel makes a second permutation impossible to miss.
    """
    ns, alloc = _ns(), _alloc()
    order = ns["DEFAULT_ORDER"]
    sign = ns["normalize_sign"](ALL_LIVE)
    action = [0.0, 0.1, 0.2, 0.3, 0.4, 0.5]
    out = ns["reallocate"](action, order, sign, alloc)
    twice = [out[order[j]] for j in range(6)]
    assert out == [action[order[j]] for j in range(6)]
    assert twice != out, "the test vector must be able to detect a second permute"


def test_dead_channels_are_pinned_to_exact_zero():
    ns, alloc = _ns(), _alloc()
    order = ns["DEFAULT_ORDER"]
    for table in (M4_DEAD, M3_M4_DEAD):
        sign = ns["normalize_sign"](table)
        out = ns["reallocate"]([0.5] * 6, order, sign, alloc)
        for j in range(6):
            if sign[j] == 0.0:
                assert out[j] == 0.0, "m%d is disabled and must stay at exact zero" % j


def test_horizontal_wrench_is_reproduced_exactly_with_m4_dead():
    """Three live horizontals, three axes -> the heading is recovered EXACTLY.

    This is the answer to "can yaw and xy compensate for a dead thruster": on the
    horizontal axis, yes, and not approximately. What is lost is magnitude, not
    direction -- see test_authority_is_the_cost_not_accuracy, below.

    The requests here are deliberately small. Exactness is only claimable INSIDE
    the reduced reach: with three channels the solve needs roughly twice the per-
    channel command the four-channel one did, so a mid-range request already caps
    out and scales down. Saturation is a separate test, and asserting no
    saturation here keeps the two properties from masking each other.
    """
    ns, alloc = _ns(), _alloc()
    order = ns["DEFAULT_ORDER"]
    sign = ns["normalize_sign"](M3_M4_DEAD)
    for action in ([0.10, -0.05, 0.15, 0.05, 0.08, -0.12],
                   [0.00, 0.05, 0.05, 0.00, 0.05, 0.05],
                   [-0.02, 0.01, -0.01, 0.02, 0.00, 0.03]):
        out = ns["reallocate"](action, order, sign, alloc)
        assert max(abs(v) for v in out) < 1.0, "premise: this request is inside the reach"
        want = _intended(alloc, action)
        got = _realised(alloc, out, order, sign)
        for ax in ("Fx", "Fy", "Mz"):
            assert _close(got[ax], want[ax]), \
                "%s not reproduced: got %.9f want %.9f" % (ax, got[ax], want[ax])


def test_vertical_keeps_Fz_and_openly_loses_My():
    """m3 is dead -> (Fz, My) is rank 1. Fz is kept; My cannot be.

    Pinned as a POSITIVE assertion on the residual rather than left unsaid: a
    future reader must not be able to mistake this for a solved axis. The residual
    is exactly what m0 alone contributes.
    """
    ns, alloc = _ns(), _alloc()
    order = ns["DEFAULT_ORDER"]
    sign = ns["normalize_sign"](M3_M4_DEAD)
    action = [0.5, 0.0, 0.0, 0.3, 0.0, 0.0]        # pure heave request
    out = ns["reallocate"](action, order, sign, alloc)
    want, got = _intended(alloc, action), _realised(alloc, out, order, sign)
    assert _close(got["Fz"], want["Fz"]), "Fz must be reproduced exactly"
    # My is not in AXES; recompute it here to state the loss explicitly.
    my_want = sum(alloc["My"][i] * action[i] for i in range(6))
    my_got = sum(alloc["My"][order[j]] * out[j] for j in range(6) if sign[j] != 0.0)
    assert not _close(my_got, my_want), \
        "My cannot be honoured with one vertical channel -- if this ever passes, " \
        "the rank story changed and the docstring is stale"
    assert _close(my_got, alloc["My"][order[0]] * out[0]), "the residual is m0's own My"


def test_saturation_scales_the_group_and_preserves_direction():
    """Over-range solves scale down as a group; a per-channel clamp would rotate
    the wrench, turning a magnitude problem into a heading error."""
    ns, alloc = _ns(), _alloc()
    order = ns["DEFAULT_ORDER"]
    sign = ns["normalize_sign"](M3_M4_DEAD)
    action = [0.0, 1.0, -1.0, 0.0, 1.0, -1.0]      # hard turn, guaranteed to exceed 1
    out = ns["reallocate"](action, order, sign, alloc)
    assert max(abs(v) for v in out) <= 1.0 + 1e-12, "output must stay in the policy contract"
    want, got = _intended(alloc, action), _realised(alloc, out, order, sign)
    scales = [got[ax] / want[ax] for ax in ("Fx", "Fy", "Mz") if abs(want[ax]) > 1e-9]
    assert scales, "the test vector must produce a non-zero horizontal wrench"
    for s in scales:
        assert 0.0 < s <= 1.0, "scaling must shrink, never flip or grow"
        assert _close(s, scales[0]), "every axis must shrink by the SAME factor"


def test_the_two_groups_scale_independently():
    """A saturated turn must not throttle depth. They no longer share an axis
    (My is already abandoned), so coupling them would be a pure loss."""
    ns, alloc = _ns(), _alloc()
    order = ns["DEFAULT_ORDER"]
    sign = ns["normalize_sign"](M3_M4_DEAD)
    action = [0.2, 1.0, -1.0, 0.2, 1.0, -1.0]      # saturating horizontal, mild heave
    out = ns["reallocate"](action, order, sign, alloc)
    want, got = _intended(alloc, action), _realised(alloc, out, order, sign)
    assert max(abs(out[j]) for j in ns["FW_HORZ_CH"]) > 0.9, "horizontal really is at the cap"
    assert _close(got["Fz"], want["Fz"]), "Fz survived the horizontal saturation intact"


def test_authority_is_the_cost_not_accuracy():
    """Same heading, less push. Quantifies what the fault actually costs.

    A pure-yaw request that the healthy robot delivers at some magnitude comes
    back from the 3-channel solve pointing the same way and weaker -- the 50%
    figure in the analysis is about reach, not about being wrong.
    """
    ns, alloc = _ns(), _alloc()
    order = ns["DEFAULT_ORDER"]
    healthy = ns["normalize_sign"](ALL_LIVE)
    faulted = ns["normalize_sign"](M3_M4_DEAD)
    action = [0.0, 0.8, -0.8, 0.0, 0.8, -0.8]
    w_healthy = _realised(alloc, ns["reallocate"](action, order, healthy, alloc),
                          order, healthy)
    out_f = ns["reallocate"](action, order, faulted, alloc)
    w_faulted = _realised(alloc, out_f, order, faulted)
    assert abs(w_healthy["Mz"]) > 1e-6, "the vector must be a real yaw request"
    ratio = w_faulted["Mz"] / w_healthy["Mz"]
    assert ratio > 0.0, "the recovered yaw must point the SAME way"
    assert ratio <= 1.0 + 1e-12, "it cannot gain authority by losing a thruster"


def test_horizontal_submatrix_is_regular_and_matches_the_analysis():
    """det = 0.288 -- re-derived here from the artifact, not copied from prose.

    If this ever goes to zero the 3-channel solve has no unique answer and every
    claim above collapses, so it is worth asserting on rather than trusting.
    """
    ns, alloc = _ns(), _alloc()
    order = ns["DEFAULT_ORDER"]
    sign = ns["normalize_sign"](M3_M4_DEAD)
    cols = [order[j] for j in ns["FW_HORZ_CH"] if sign[j] != 0.0]
    assert len(cols) == 3
    a = [[alloc[ax][c] for c in cols] for ax in ("Fx", "Fy", "Mz")]
    # solve3x3 returns None exactly when the determinant is degenerate
    assert ns["solve3x3"](a, [1.0, 0.0, 0.0]) is not None, "must be regular"
    d = (a[0][0] * (a[1][1] * a[2][2] - a[1][2] * a[2][1])
         - a[0][1] * (a[1][0] * a[2][2] - a[1][2] * a[2][0])
         + a[0][2] * (a[1][0] * a[2][1] - a[1][1] * a[2][0]))
    assert abs(abs(d) - 0.288) < 0.001, "det %.6f != the analysis's 0.288" % d


def test_solve3x3_refuses_a_singular_system():
    ns = _ns()
    singular = [[1.0, 2.0, 3.0], [2.0, 4.0, 6.0], [1.0, 0.0, 1.0]]
    assert ns["solve3x3"](singular, [1.0, 2.0, 3.0]) is None
    ident = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
    assert ns["solve3x3"](ident, [3.0, -1.0, 2.0]) == [3.0, -1.0, 2.0]


def test_non_finite_input_lands_on_neutral_not_garbage():
    """The plain path already dropped NaN/inf to 0; reallocation must not
    reintroduce it -- one NaN in the wrench sum would poison all three channels."""
    ns, alloc = _ns(), _alloc()
    order = ns["DEFAULT_ORDER"]
    sign = ns["normalize_sign"](M3_M4_DEAD)
    inf = float("inf")
    for bad in (float("nan"), inf, -inf):
        out = ns["reallocate"]([bad, 0.1, 0.1, 0.0, 0.1, 0.1], order, sign, alloc)
        for j, v in enumerate(out):
            assert v == v, "m%d went NaN" % j
            assert abs(v) != inf, "m%d went infinite" % j


def test_too_few_live_horizontals_passes_through_instead_of_inventing_a_solve():
    """Two live horizontals cannot span (Fx,Fy,Mz). Fall back rather than emit a
    least-squares answer that would look authoritative and be wrong."""
    ns, alloc = _ns(), _alloc()
    order = ns["DEFAULT_ORDER"]
    sign = ns["normalize_sign"]([1, 1, 0, 1, 0, 1])   # m2 and m4 out
    action = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6]
    out = ns["reallocate"](action, order, sign, alloc)
    for j in ns["FW_HORZ_CH"]:
        assert out[j] == (action[order[j]] if sign[j] != 0.0 else 0.0)


if __name__ == "__main__":
    for _name, _fn in sorted(globals().items()):
        if _name.startswith("test_"):
            _fn()
            print("PASS %s" % _name)
