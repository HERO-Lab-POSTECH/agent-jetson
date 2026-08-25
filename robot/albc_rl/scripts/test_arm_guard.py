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
  3. The joint2 window is still DERIVED from the manipulability threshold the
     policy trained with (w = sqrt|sin theta2| >= 0.3), not a number someone typed.
     This repo's recurring failure is constants that drift away from the artifact
     they came from; the window is re-derived here and compared.
  4. The window predicate accepts/rejects the poses actually measured on the break
     run. Skipped with a loud note when rospy is absent (i.e. off-board), because
     importing the node module needs it.

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

# The manipulability cost the policy trained with: manipulability_cost w_threshold.
W_THRESHOLD = 0.3

GUARDS = ['joint2_min_rad', 'joint2_max_rad', 'joint_current_max_ma',
          'joint_current_max_s', 'start_att_max_deg']

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
    sys.exit(1)

print('\n2. guard defaults agree across all three files, and are forwarded')
node_src = open(NODE).read()
inc = ET.parse(FIELD).getroot().find('include')
forwarded = set(a.get('name') for a in inc.findall('arg'))
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

print('\n3. the joint2 window is still derived from w_threshold %.1f' % W_THRESHOLD)
mnp = math.asin(W_THRESHOLD ** 2)
check(abs(float(launch_args['joint2_min_rad']) - mnp) < 1e-4,
      'joint2_min_rad %s == asin(%.2f) = %.5f rad (%.2f deg)'
      % (launch_args['joint2_min_rad'], W_THRESHOLD ** 2, mnp, math.degrees(mnp)))
check(abs(float(launch_args['joint2_max_rad']) - (math.pi - mnp)) < 1e-4,
      'joint2_max_rad %s == pi - asin(%.2f) = %.5f rad (%.2f deg)'
      % (launch_args['joint2_max_rad'], W_THRESHOLD ** 2, math.pi - mnp,
         math.degrees(math.pi - mnp)))
check(re.search(r'np\.arcsin\(_w_thresh \*\* 2\)', node_src) is not None,
      'rl_inference_node derives the bound rather than hardcoding it')

print('\n4. the window predicate on the poses measured during the fracture')
try:
    sys.path.insert(0, HERE)
    from rl_inference_node import j2_in_window          # noqa: E402
except ImportError as e:
    print('  SKIP  cannot import rl_inference_node (%s).' % e)
    print('        This step needs rospy, so run it ON THE BOARD:')
    print('          source ~/catkin_ws/devel/setup.bash && python %s'
          % os.path.basename(__file__))
    print('        Steps 1-3 above ran; those are the ones that catch drift.')
else:
    lo = float(launch_args['joint2_min_rad'])
    hi = float(launch_args['joint2_max_rad'])
    # every value below is a MEASUREMENT off e2_run1_gain010_BREAK.bag
    cases = [
        (math.radians(-10.20), False, 'joint2 the policy started from (bag t=0)'),
        (math.radians(349.80), False, 'the same pose wrapped: -10.20 + 360'),
        (math.radians(166.30), True, 'centre of the 0.623 Hz limit cycle'),
        (math.radians(182.90), False, 'top of the limit cycle at t=90 s'),
        (math.radians(221.04), False, 'peak of the startup slew'),
        (math.radians(149.94), True, 'where park-only left the arm'),
        (-0.061, False, 'the driver-logged command, unwrapped'),
        (-0.061 + 2 * math.pi, False, 'the same command +1 turn: must agree'),
        (2.900, True, 'a normal working pose, 166.2 deg'),
    ]
    for val, want, why in cases:
        got = j2_in_window(val, lo, hi)
        check(got == want, '%8.2f deg -> %-5s expected %-5s (%s)'
              % (math.degrees(val), got, want, why))
    check(j2_in_window(-0.061, lo, hi) == j2_in_window(-0.061 + 2 * math.pi, lo, hi),
          'wrap equivalence: +-2*pi cannot change the verdict')
    check(j2_in_window(99.0, 1.0, 1.0) is True,
          'hi <= lo disables the window (overGuard convention)')

print('\n%s  (%d failures)' % ('PASS' if not fails else 'FAIL', len(fails)))
sys.exit(1 if fails else 0)
