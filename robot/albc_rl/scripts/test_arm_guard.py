#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Pin the 2026-08-25 arm-protection guards. stdlib only, runs on the board.

WHY THIS FILE EXISTS. arm2 fractured on 2026-08-25 during E2 run 1. Three guards
that would each have stopped that run did not exist: a joint2 angle window, a
sustained joint-current cap on the RL path, and a start-attitude gate. They exist
now, spread across rl_inference_node.py and two launch files, and a safety limit
that lives in three places is a safety limit that drifts. Full incident record:
.community/posts/finding/047-e2-run1-arm2-fracture.md

WHAT IT CHECKS, and why each check earned its place:

  1. XML well-formedness of both launch files. Not paranoia -- writing these
     guards introduced a literal double-hyphen inside an XML comment, which is
     illegal and would have made `launch-rl` fail outright at the pool. Nothing
     else caught it.
  2. Every guard default agrees across rl_inference_node.py, albc_rl.launch and
     albc_rl_fieldtest.launch, AND the fieldtest include forwards each one. A
     default that disagrees between the two launch files means the guard silently
     differs depending on which entry point the operator used. A knob the include
     forgets to forward is a knob that reads as working and does nothing.
  5. The sustained-current timer, including the injected-zero sequence that stands
     in for the CRITICAL defect. It imports arm_guard.py, which pulls in numpy
     and nothing else -- it used to live in rl_inference_node, and importing that
     drags in rospy, so the most intricate logic in the change could only be tested
     on the board.
  6. The DRIVER half of the current guard, asserted on C++ source text. Step 5 alone
     pins nothing: against a driver that still publishes a fabricated 0 on a failed
     read, the node's stale handling is a no-op and every zero clears the timer. The
     two halves only work as a pair, and C++ offers no other layer here.
  7. That rl_inference_node actually IMPORTS that predicate and keeps no local copy.
     Step 6 validates arm_guard.py; before the split it validated the node itself.
     Without this, stubbing the guard out in the node leaves everything else green.
  8. home_joint2's default (150 deg) agrees across the node and both launch files --
     the same class of check as 2. See decision/061 C.

  RETIRED 2026-08-26 (decision/061 A1/A2, guard rollback): steps 3-4 used to test
  a joint2 HARD WINDOW (derivation from the manipulability threshold, and the
  window predicate on the break-run poses). That window is gone -- it was a
  hand-written clamp bolted onto a constrained-RL system whose singularity
  avoidance is already a TRAINED cost (manipulability_cost), and it latched the
  policy mid an ordinary attitude-lowering move on 2026-08-25 (policy lifetime
  0.255 s -- notes/2026-08-25-guard-session-retraction-handoff.md). See
  decision/061.

EVERY source-text assertion in here has been watched FAIL on a deliberate mutation.
That is not ceremony: such an assertion has two failure modes and only one is loud.
Written after the code, too-strict fails on correct input and costs minutes (step 6
did, on a brace initialiser). Written before, too-loose passes on nothing and ships a
check that asserts a string it never finds. A green run is evidence only if the red
run was also seen.

Usage:  python test_arm_guard.py        (exit 0 = pass)
"""
import math
import os
import re
import sys
import xml.etree.ElementTree as ET

HERE = os.path.dirname(os.path.abspath(__file__))
NODE = os.path.join(HERE, 'rl_inference_node.py')
LAUNCH = os.path.join(HERE, '..', 'launch', 'albc_rl.launch')
FIELD = os.path.join(HERE, '..', 'launch', 'albc_rl_fieldtest.launch')

GUARDS = ['joint_current_max_ma', 'joint_current_max_s', 'start_att_max_deg']

fails = []


def check(cond, msg):
    if cond:
        print('  ok   %s' % msg)
    else:
        print('  FAIL %s' % msg)
        fails.append(msg)


def args_of(path):
    root = ET.parse(path).getroot()
    return dict((a.get('name'), a.get('default')) for a in root.findall('arg'))


def main():
    global fails
    fails = []

    print('1. launch files are well-formed XML')
    launch_args = None
    field_args = None
    for path in (LAUNCH, FIELD):
        try:
            a = args_of(path)
        except ET.ParseError as e:
            check(False, '%s is NOT well-formed XML: %s' % (os.path.basename(path), e))
            continue
        check(True, '%s parses (%d args)' % (os.path.basename(path), len(a)))
        if path == LAUNCH:
            launch_args = a
        else:
            field_args = a

    if launch_args is None or field_args is None:
        print('\ncannot continue without both launch files')
        return 1

    print('\n2. guard defaults agree across all three files, and are forwarded')
    node_src = open(NODE).read()
    inc = ET.parse(FIELD).getroot().find('include')
    forwarded = set(a.get('name') for a in inc.findall('arg'))
    # The <arg> half is only half the wiring. An arg that is declared, agreed and
    # forwarded but never BOUND to a <param> inside the node block is a knob that reads
    # as working and does nothing -- the same class as the XML bug check 1 catches, and
    # the same class only a mechanical check finds. VERIFIED BY MUTATION 2026-08-25:
    # deleting one <param> line left this file reporting 0 failures.
    node_el = None
    for n in ET.parse(LAUNCH).getroot().iter('node'):
        if n.get('name') == 'rl_inference_node':
            node_el = n
    check(node_el is not None, 'albc_rl.launch has an rl_inference_node <node> block')
    bound = ({} if node_el is None
             else dict((p.get('name'), p.get('value')) for p in node_el.findall('param')))
    for name in GUARDS:
        check(bound.get(name) == '$(arg %s)' % name,
              '%s is BOUND as <param value="$(arg %s)"> in the node block (got %r)'
              % (name, name, bound.get(name)))
    for name in GUARDS:
        a = launch_args.get(name)
        b = field_args.get(name)
        check(a is not None, '%s declared in albc_rl.launch' % name)
        check(b is not None, '%s declared in albc_rl_fieldtest.launch' % name)
        if a is not None and b is not None:
            check(abs(float(a) - float(b)) < 1e-9,
                  '%s default matches: %s == %s' % (name, a, b))
        check(name in forwarded, '%s is forwarded by the fieldtest include' % name)
        # The node must read the same default too, or a bare `rosrun` of the node
        # (no launch file) would run a different guard than the pool procedure.
        # stop at the closing paren: the value expression never contains one
        m = re.search(r'get_param\("~%s",\s*([^)]+)\)' % name, node_src)
        check(m is not None, '%s is read by rl_inference_node' % name)
        if m is not None and a is not None:
            raw = m.group(1).strip()
            try:
                literal = float(raw)
            except ValueError:
                # the joint2 bounds are derived expressions in the node; step 3 checks
                # that the derivation itself is still the manipulability threshold
                check('_mnp' in raw,
                      '%s node default is derived from _mnp (%s)' % (name, raw))
            else:
                check(abs(literal - float(a)) < 1e-9,
                      '%s node default %s == launch default %s' % (name, raw, a))

    # steps 3-4 (joint2 hard-window derivation + predicate) RETIRED 2026-08-26 --
    # decision/061 A1/A2, guard rollback. See the module docstring.

    print('\n5. the sustained-current timer, including the injected-zero sequence')
    # over_current_held lives in arm_guard.py, which imports numpy and NOTHING else,
    # precisely so that the tests standing in for the CRITICAL current-read defect run
    # off-board too. It used to sit inside rl_inference_node, and importing that drags
    # in rospy, so this test ran only on the board -- the most intricate logic in the
    # change was gated behind an import it does not need.
    from albc_rl.arm_guard import over_current_held   # noqa: E402
    CAP, HOLD = 900.0, 0.5

    def run(samples, cap=CAP):
        """samples = [(t, mA, stale)] -> max held reached."""
        since, worst = None, 0.0
        for t, ma, st in samples:
            since, held = over_current_held(since, t, ma, cap, st)
            worst = max(worst, held)
        return worst

    steady = [(i * 0.02, 1300.0, False) for i in range(50)]     # 1 s at 1300 mA
    check(run(steady) >= HOLD, 'a steady 1300 mA stall reaches the %.2f s trip' % HOLD)
    # THE 2026-08-25 DEFECT, as a test. The driver used to publish 0 mA on a failed
    # Dynamixel read; one per 0.5 s window reset the timer and the guard never fired.
    # It is fixed upstream (the driver skips the publish), so a gap now arrives as
    # STALE -- and stale must NOT clear an excess that is already accumulating.
    injected = []
    for i in range(50):
        injected.append((i * 0.02, 0.0 if i % 10 == 9 else 1300.0, False))
    check(run(injected) < HOLD,
          'a FABRICATED fresh 0 defeats the timer -- this is WHY '
          'joint_angle_command.cpp must SKIP the publish on a failed read. NOT a '
          'node behaviour to preserve (narrative; the real pin is step 6)')
    gappy = []
    for i in range(50):
        gappy.append((i * 0.02, 1300.0, i % 10 == 9))   # value held, sample STALE
    check(run(gappy) >= HOLD,
          'a STALE gap keeps the timer running (unknown != clear)')
    check(run([(0.0, 500.0, False), (1.0, 500.0, True)]) == 0.0,
          'stale with nothing accumulating stays clear')
    check(run([(0.0, 1300.0, False), (0.02, 100.0, False), (1.0, 1300.0, False)]) < HOLD,
          'a genuine fresh under-cap sample DOES clear it')
    # first over sample must not trip instantly
    since, held = over_current_held(None, 10.0, 1300.0, CAP, False)
    check(since == 10.0 and held == 0.0, 'first over-cap sample starts at held = 0')
    # Clock jump: this board restores its clock from a snapshot at boot and can
    # move DAYS when jetson_clock_sync lands. held >= 0 is only the symptom -- what
    # matters is that `since` RESTARTS. Preserving it would clamp held to 0 until
    # wall time caught up, i.e. a guard inert for the whole jump during a stall.
    since, held = over_current_held(100.0, 50.0, 1300.0, CAP, False)
    check(since == 50.0 and held == 0.0,
          'a BACKWARD clock jump RESTARTS the window (since 100 -> 50), not freezes it')
    check(run([(100.0, 1300.0, False), (50.0, 1300.0, False),
               (50.6, 1300.0, False)]) >= HOLD,
          'and the restarted window still trips %.2f s after the jump' % HOLD)

    print('\n6. the DRIVER half of the current guard (the other half of step 5)')
    # The node's stale handling is a NO-OP against a driver that still publishes a
    # fabricated 0 on a failed read: _last_cur_t refreshes every cycle, `stale` is never
    # true, and every zero clears the accumulator -- the original defect, untouched. So
    # the two halves only work as a pair, and step 5 alone pins nothing. C++ offers no
    # other layer in this repo, so this asserts on source text. Crude, and it fails
    # loudly if anyone un-gates the publish, which is the whole job.
    DRIVER = os.path.join(HERE, '..', '..', 'albc_control', 'src',
                          'joint_angle_command.cpp')
    try:
        cpp_src = open(DRIVER).read()
    except IOError as e:
        check(False, 'cannot read joint_angle_command.cpp (%s)' % e)
    else:
        check('bool readCurrent(' in cpp_src,
              'readCurrent reports success instead of returning a zero-initialised local')
        check(re.search(r'cur_ok\s*=\s*readCurrent\([^;]*&&[^;]*readCurrent\(', cpp_src)
              is not None,
              'both channels are read and BOTH must succeed (max(|.|) consumer)')
        # bounded non-greedy, NOT [^}]* -- the body contains a brace initialiser
        # (current_msg.data = {a, b}) so a "no closing brace" class stops short of the
        # publish and reports a false FAIL. Caught by this check failing on correct code.
        check(re.search(r'if \(cur_ok\)\s*\{.{0,300}?current_pub\.publish', cpp_src, re.S)
              is not None,
              'the /joint_currents publish is GATED on that success')
        # Scoped to readCurrent's OWN body, and ordered within it. The first version of
        # this check asserted `'return false;' in cpp_src` (true of the whole file --
        # readPosition has one) AND an index ordering that is trivially true whenever
        # both strings exist, since `*out = current;` is inside the very function whose
        # signature it was compared against. It read like a pin and pinned nothing: the
        # same shape as old step 5. It also used str.index, which RAISES when the
        # substring is absent -- and check() evaluates its argument before the call, so
        # a mutated signature escaped as a ValueError, killing the summary line and any
        # later step. .find() >= 0 fails closed instead.
        body = cpp_src[cpp_src.find('bool readCurrent('):]
        body = body[:body.find('\n}')] if body else ''
        ret, assign = body.find('return false;'), body.find('*out = current;')
        check(ret >= 0 and assign >= 0 and ret < assign,
              'readCurrent returns false BEFORE assigning *out (ordered inside its own body)')

    print('\n7. the node actually USES arm_guard (the coupling the split broke)')
    # Steps 5-6 validate arm_guard.py. Before the 2026-08-25 split it imported from
    # rl_inference_node, so they validated the NODE. Moving the predicate out bought
    # off-board execution and PAID WITH THIS COUPLING: nothing downstream asserted the
    # node still imports it. MUTATION D, reproduced here before writing the fix --
    # replacing the import with a local stub (`over_current_held -> (None, 0.0)`)
    # neutered the guard in the production file and the suite still reported 0
    # failures. Third instance of the same class as the XML bug and the two blind
    # spots: only a mechanical check finds it.
    for fn in ('over_current_held',):
        check(re.search(r'from albc_rl\.arm_guard import[^\n]*%s' % fn, node_src) is not None,
              'rl_inference_node imports %s from albc_rl.arm_guard' % fn)
    # This is the half that catches Mutation D. An import the node then shadows with
    # its own definition is worse than no import: it reads as wired and is not.
    shadow = re.search(r'^def over_current_held', node_src, re.M)
    check(shadow is None, 'and keeps no local copy of it')

    print('\n8. home_joint2 (150 deg) agrees across the node and both launch files')
    # decision/061 C: home_joint2 was pi/2 (max lever, max load) and was not exposed
    # as a launch arg at all, so an operator could not lower it from the CLI without
    # editing the node source. C moved the default to 150 deg -- a control-response-
    # verified parking pose, NOT a stress measurement, see decision/061 C -- and
    # exposed it, along with the other three home_* knobs, the same way every guard
    # above already is.
    HOME_ARGS = ['home_joint1', 'home_joint2', 'home_tol', 'home_timeout_s']
    for name in HOME_ARGS:
        a = launch_args.get(name)
        b = field_args.get(name)
        check(a is not None, '%s declared in albc_rl.launch' % name)
        check(b is not None, '%s declared in albc_rl_fieldtest.launch' % name)
        if a is not None and b is not None:
            check(abs(float(a) - float(b)) < 1e-9,
                  '%s default matches: %s == %s' % (name, a, b))
        check(name in forwarded, '%s is forwarded by the fieldtest include' % name)
        check(bound.get(name) == '$(arg %s)' % name,
              '%s is BOUND as <param value="$(arg %s)"> in the node block (got %r)'
              % (name, name, bound.get(name)))
        m = re.search(r'get_param\("~%s",\s*([^)]+)\)' % name, node_src)
        check(m is not None, '%s is read by rl_inference_node' % name)
        if m is not None and a is not None:
            raw = m.group(1).strip()
            try:
                literal = float(raw)
            except ValueError:
                check(False, '%s node default %r is not a plain float literal' % (name, raw))
            else:
                check(abs(literal - float(a)) < 1e-9,
                      '%s node default %s == launch default %s' % (name, raw, a))
    if launch_args.get('home_joint2') is not None:
        check(abs(float(launch_args['home_joint2']) - math.radians(150.0)) < 1e-6,
              'home_joint2 default %s rad == 150 deg' % launch_args['home_joint2'])

    print('\n%s  (%d failures)' % ('PASS' if not fails else 'FAIL', len(fails)))
    if not fails:
        print('\nREMINDER: a green run does NOT mean the deployed system is covered.')
        print('The over-current guard is INERT until joint_angle_command is restarted')
        print('with this binary -- and that restart drops arm torque, so it happens OUT')
        print('OF THE WATER. Steps 5-6 test the pair; only a restart deploys it.')
    return 1 if fails else 0


def test_arm_guard_suite():
    assert main() == 0


if __name__ == "__main__":
    sys.exit(main())
