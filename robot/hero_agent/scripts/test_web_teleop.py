#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Self-check for web_teleop's routing and page rendering. Stdlib only.

Runs on py2.7 (the board) and py3 (a dev Mac) alike: rospy / std_msgs /
BaseHTTPServer / urlparse are stubbed, so no ROS master and no py2 needed. What
this pins is the part that decides where a key goes - the allow-list boundary,
the DIRECT_CMD split, and the repeat guard.

    python test_web_teleop.py
"""
import os
import sys
import types


def _stub(name, **attrs):
    m = types.ModuleType(name)
    for k, v in attrs.items():
        setattr(m, k, v)
    sys.modules[name] = m
    return m


_stub("rospy", loginfo=lambda *a: None, get_time=lambda: 0.0)
_std = _stub("std_msgs")
_std.msg = _stub("std_msgs.msg", Int8=lambda **kw: kw)
# Handler subclasses BaseHTTPRequestHandler at import time, so it must be a class.
_stub("BaseHTTPServer", BaseHTTPRequestHandler=object, HTTPServer=object)
_stub("urlparse", urlparse=None, parse_qs=None)

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import web_teleop as W


def main():
    # --- allow-list is derived from key_teleop's KEY_TABLE, not hand-written ---
    assert ord('w') in W.ALLOWED, "jog key missing"
    assert ord('1') in W.ALLOWED, "relay toggle missing"
    assert ord('b') in W.ALLOWED, "gripper close missing"
    assert ord('x') not in W.ALLOWED, "unlisted key leaked into allow-list"

    last = {}

    # --- jog keys go to the agent node verbatim, and repeat freely while held ---
    assert W.route(ord('w'), 0.00, last) == ("key", ord('w'))
    assert W.route(ord('w'), 0.01, last) == ("key", ord('w')), "jog must not be guarded"

    # --- toggles are translated and sent to the firmware command topic instead ---
    assert W.route(ord('1'), 1.0, last) == ("cmd", ord('R')), "relay must map to 'R'"

    # --- and a double-tap inside the guard window is swallowed, not toggled back ---
    half = 1.0 + W.REPEAT_GUARD_SEC / 2.0
    assert W.route(ord('1'), half, last) == ("guarded", ord('1'))
    later = 1.0 + W.REPEAT_GUARD_SEC * 2.0
    assert W.route(ord('1'), later, last) == ("cmd", ord('R')), "guard must expire"

    # --- the trust boundary: an unlisted code never reaches any topic ---
    assert W.route(ord('x'), 2.0, last) == ("ignored", ord('x'))

    # --- every allowed key is reachable from the page ---
    page = W.build_page()
    for c in sorted(W.ALLOWED):
        assert 'data-c="%d"' % c in page, "no button for %r" % chr(c)
    assert 'data-c="%d" data-hold' % ord('w') in page, "jog key must be holdable"
    assert 'data-c="%d">' % ord('1') in page, "toggle key must not be holdable"

    print("test_web_teleop: OK (%d keys allowed)" % len(W.ALLOWED))
    return 0


def test_web_teleop():
    assert main() == 0


if __name__ == '__main__':
    sys.exit(main())
