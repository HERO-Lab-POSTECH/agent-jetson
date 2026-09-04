#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Web teleop node for HERO Agent - phone-friendly buttons over HTTP.

Same wire protocol as key_teleop.py: one Int8 per key press, onto the same two
topics with the same DIRECT_CMD split. KEY_TABLE / DIRECT_CMD / REPEAT_GUARD are
imported from key_teleop rather than copied - keymap.h and key_teleop.py already
carry a lock-step pair, and a third copy of the keymap would rot apart.

Stdlib only (BaseHTTPServer, py2.7). The board has no internet, so rosbridge is
not installable there; the page talks plain GET instead of websockets.

Reach: this binds the board's own interface, which only the Mac can see over the
USB ethernet link. What a phone can reach is decided by the tunnel opened on the
Mac, e.g.

    ssh -N -L 0.0.0.0:8080:192.168.2.100:8080 agent-jetson

There is no auth - the allow-list below bounds *what* can be sent, not *who* may
send it. Keep that tunnel off untrusted networks.

Usage:
    rosrun hero_agent web_teleop.py           # optional: _port:=8080
"""
import rospy
from std_msgs.msg import Int8
from BaseHTTPServer import BaseHTTPRequestHandler, HTTPServer
from urlparse import urlparse, parse_qs

from key_teleop import KEY_TABLE, DIRECT_CMD, REPEAT_GUARD_KEYS, REPEAT_GUARD_SEC
from albc_rl.contract import TOPICS

# Allow-list derived from key_teleop's KEY_TABLE, so the two cannot diverge.
# Entries pack several keys per row ("w/s", "c/v/b"), one char each.
# Same discipline as the terminal teleop: unlisted keys are ignored, never
# forwarded to the firmware. This is the trust boundary - the page is untrusted.
ALLOWED = set()
for _grp, _keys in KEY_TABLE:
    for _k, _desc in _keys:
        for _one in _k.split("/"):
            ALLOWED.add(ord(_one))

_last = {}      # guarded key -> last publish time (mirrors key_teleop's last_sent)
_pub = None
_pub_cmd = None


def route(c, now, last):
    """Decide what a key code becomes. Pure but for the `last` guard bookkeeping.

    Returns (kind, value): ignored / guarded / cmd (firmware direct) / key (agent node).
    Split out from publish() so the logic is testable without a ROS master.
    """
    if c not in ALLOWED:
        return ("ignored", c)
    if c in REPEAT_GUARD_KEYS:
        if now - last.get(c, 0.0) < REPEAT_GUARD_SEC:
            return ("guarded", c)
        last[c] = now
    if c in DIRECT_CMD:
        return ("cmd", DIRECT_CMD[c])
    return ("key", c)


def publish(c):
    kind, val = route(c, rospy.get_time(), _last)
    if kind == "cmd":
        _pub_cmd.publish(Int8(data=val))
        rospy.loginfo("Web: '%s' (%d) -> cmd '%s' (%d) [direct]",
                      chr(c), c, chr(val), val)
    elif kind == "key":
        _pub.publish(Int8(data=val))
        rospy.loginfo("Web: '%s' (%d)", chr(c), c)
    return "%s %d" % (kind, val)


PAGE_TMPL = """<!doctype html>
<html><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1,user-scalable=no">
<title>HERO Teleop</title>
<style>
 body{background:#111;color:#eee;font:16px/1.4 system-ui,-apple-system,sans-serif;
      margin:0;padding:12px 12px 64px}
 h1{font-size:18px;margin:0 0 4px}
 h2{font-size:12px;letter-spacing:.08em;text-transform:uppercase;color:#8ab;margin:18px 0 6px}
 .row{display:flex;flex-wrap:wrap;gap:8px}
 button{min-width:58px;min-height:58px;font-size:22px;background:#2a2f36;color:#eee;
        border:1px solid #444;border-radius:10px;touch-action:none;
        -webkit-user-select:none;user-select:none}
 button:active{background:#3d8f6a}
 button[data-c="113"]{background:#a33;min-width:100px}
 p.legend{color:#888;font-size:12px;margin:6px 0 0}
 #log{position:fixed;left:0;right:0;bottom:0;background:#000;border-top:1px solid #333;
      padding:10px 12px;font-family:ui-monospace,monospace;font-size:13px;color:#6c9}
</style></head><body>
<h1>HERO Teleop</h1>
__ROWS__
<div id="log">ready</div>
<script>
var timer=null, log=document.getElementById('log');
function send(c){
  fetch('/k?c='+c).then(function(r){return r.text();})
   .then(function(s){log.textContent=s;})
   .catch(function(e){log.textContent='ERR '+e;});
}
// ponytail: one shared timer, so two buttons held at once is last-wins.
// Split per-button if diagonal jog (w+d) turns out to matter.
Array.prototype.forEach.call(document.querySelectorAll('button'), function(b){
  var c=b.getAttribute('data-c'), hold=b.getAttribute('data-hold');
  b.addEventListener('pointerdown', function(e){
    e.preventDefault(); send(c);
    if(hold){ timer=setInterval(function(){ send(c); }, 100); }
  });
  ['pointerup','pointercancel','pointerleave'].forEach(function(ev){
    b.addEventListener(ev, function(){
      if(timer){ clearInterval(timer); timer=null; }
    });
  });
});
</script></body></html>
"""


def build_page():
    """Render KEY_TABLE as button rows. The legend line reuses key_teleop's own
    'k=desc' phrasing verbatim - descriptions like 'Yaw +/-0.1' do not survive
    being split per key, so they stay whole under the row."""
    out = []
    for grp, keys in KEY_TABLE:
        btns = []
        for k, desc in keys:
            for one in k.split("/"):
                # Repeat-guarded keys are self-toggles: tap once, never auto-repeat.
                # Jog/throttle/setpoint keys repeat while held, matching the OS
                # key auto-repeat the terminal teleop relies on.
                hold = "" if ord(one) in REPEAT_GUARD_KEYS else ' data-hold="1"'
                btns.append('<button data-c="%d"%s>%s</button>' % (ord(one), hold, one))
        legend = "&nbsp; ".join("%s=%s" % (k, d) for k, d in keys)
        out.append('<section><h2>%s</h2><div class="row">%s</div>'
                   '<p class="legend">%s</p></section>'
                   % (grp, "".join(btns), legend))
    return PAGE_TMPL.replace("__ROWS__", "".join(out))


class Handler(BaseHTTPRequestHandler):
    def _reply(self, code, body, ctype="text/plain"):
        self.send_response(code)
        self.send_header("Content-Type", ctype + "; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        u = urlparse(self.path)
        if u.path == "/":
            self._reply(200, build_page(), "text/html")
        elif u.path == "/k":
            try:
                c = int(parse_qs(u.query).get("c", [""])[0])
            except ValueError:
                self._reply(400, "bad c")
                return
            self._reply(200, publish(c))
        else:
            self._reply(404, "not found")

    def log_message(self, fmt, *args):
        pass    # rospy already logs every accepted key; this would double it


def main():
    global _pub, _pub_cmd
    rospy.init_node("web_teleop", anonymous=True, disable_signals=True)
    _pub = rospy.Publisher(TOPICS["key_input"], Int8, queue_size=10)
    _pub_cmd = rospy.Publisher(TOPICS["command"], Int8, queue_size=10)

    port = int(rospy.get_param("~port", 8080))
    srv = HTTPServer(("0.0.0.0", port), Handler)
    rospy.loginfo("Web teleop on port %d - no auth, trusted link only", port)
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        srv.server_close()
        rospy.loginfo("Web teleop stopped.")


if __name__ == '__main__':
    main()
