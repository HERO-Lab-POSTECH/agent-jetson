"""Self-describing run records: /albc/run_meta (latched) + /albc/run_event.

WHY THIS EXISTS
---------------
Until 2026-09-06 a field-test bag recorded nine signal topics and nothing that
said WHICH system produced them. Provenance lived only as free text in
/rosout_agg and the analyzers regexed it back out (t4_step_analyze.py's RECONF
pattern). Two bills came due for that:

  * "were the four runs the same checkpoint?" could not be answered at all --
    run 1's rosout carried no weights md5 (2026-09-02 resume brief, section 3).
  * a segment boundary IS a log line. Reword one loginfo and every analyzer
    silently returns zero segments -- the same silent-empty failure class as
    the 2026-09-04 topic rename.

So the bag now carries its own manifest and its own segment marks, as DATA.

CONTRACT (schema_version 1)
---------------------------
  /albc/run_meta   std_msgs/String, LATCHED, exactly one per run.
                   Latched because the recorder subscribes a beat after the
                   node starts; an unlatched manifest lands before anyone is
                   listening and the bag then has no manifest at all.
  /albc/run_event  std_msgs/String, one JSON object per event, monotonic seq.

Strings rather than a custom .msg ON PURPOSE. A .msg change needs a board
catkin_make and a generated-header round trip, and this schema will grow every
time a scenario does. The price is that nothing type-checks the payload -- so
validate_meta() / validate_event() below ARE the check, and they run off-board
where the analysis lives.

PY2 ON THE BOARD (ROS lunar, python 2.7.12, numpy 1.11). rospy is imported
lazily inside RunLogger so everything above it imports and self-tests on a dev
machine with no ROS -- the same split arm_guard.py uses, and for the same
reason: a guard you can only exercise on the robot is a guard you do not
exercise.
"""
import binascii
import hashlib
import json
import os
import subprocess
import time

SCHEMA_VERSION = 1

META_TOPIC = "/albc/run_meta"
EVENT_TOPIC = "/albc/run_event"

# Event kinds. Keep this CLOSED: the analyzer switches on it, and a typo that
# invents a new kind is a segment that silently never matches -- which is the
# failure this whole module exists to remove, so it must not be reintroduced
# one layer up.
EVENT_KINDS = ("phase", "setpoint", "guard", "mark", "note", "payload")

# Controllers a comparison table may group by. "none" is a real category, not a
# placeholder: T1's J1 azimuth sweep (2026-09-02) was open-loop with no
# controller at all and still produced the static attitude map.
#
#   rl            72D student policy (rl_inference_node)
#   tdc           time-delay attitude controller (albc_controller, control_mode 1)
#   classic       firmware yaw/depth PID + open-loop teleop translation
#   thruster_only ATTITUDE WITHOUT THE ARM -- the baseline that answers "what does
#                 ALBC buy?". Reserved here so the two are comparable in one table
#                 the day it lands; the session building it owns the name.
#   none          open-loop / instrumentation-only
CONTROLLERS = ("rl", "tdc", "classic", "thruster_only", "none")


# --------------------------------------------------------------- provenance
def file_digest(path, algo="sha256"):
    """Full hex digest of a file, or None if it is not there.

    FULL, not truncated. The startup banner prints md5[:8] because a human
    reads it; a bag is read by a machine, and an 8-hex prefix cannot be matched
    against MANIFEST.json's sha256 without re-hashing the artifact.
    """
    if not path or not os.path.isfile(path):
        return None
    h = hashlib.new(algo)
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def pack_provenance(weights_dir, arch=None):
    """Deploy-pack identity, preferring the pack's OWN manifest to a re-hash.

    A pack ships a manifest carrying `tag`, the training checkpoint paths, and a
    sha256 per file. Its NAME varies by where you look, which cost this function
    its tag on the first live run: the archived packs under 05_deploy/pack_*/
    use MANIFEST.json, but the board's own numpy_port/ holds one manifest PER
    ENCODER -- MANIFEST.gru.json and MANIFEST.tcn.json side by side, because
    both weight sets ship together and only ~encoder_type says which one is
    loaded. Looking only for MANIFEST.json found nothing on the robot and fell
    through to hashing, so the digests were right and the human-readable tag
    (pack_r3a_p3b7500_gru_260906_145553) was silently None.

    So: MANIFEST.json, else MANIFEST.<arch>.json, else the sole MANIFEST.*.json
    if there is exactly one. With several and no `arch` we do NOT guess -- the
    candidates are recorded and the digests still identify the weights.

    Hashing the npz is the fallback for a directory with weights and no
    manifest at all (the pre-2026-08 packs).

    Carries the manifest's sha256 table VERBATIM rather than recomputing it. A
    disagreement between the manifest and the file on disk is itself a finding,
    and silently overwriting one with the other would erase it. The analysis
    side re-hashes when it needs to check.

    Never raises: provenance failing must not stop a run.
    """
    out = {"weights_dir": os.path.abspath(weights_dir) if weights_dir else None,
           "manifest": None, "tag": None, "digests": {}}
    try:
        cands = [n for n in sorted(os.listdir(weights_dir))
                 if n.startswith("MANIFEST") and n.endswith(".json")]
        name = None
        if "MANIFEST.json" in cands:
            name = "MANIFEST.json"
        elif arch and ("MANIFEST.%s.json" % arch) in cands:
            name = "MANIFEST.%s.json" % arch
        elif len(cands) == 1:
            name = cands[0]
        elif len(cands) > 1:
            out["manifest_candidates"] = cands
        man = os.path.join(weights_dir, name) if name else ""
        if name and os.path.isfile(man):
            with open(man) as f:
                m = json.load(f)
            out["manifest"] = name
            out["tag"] = m.get("tag")
            files = m.get("files", {})
            out["digests"] = dict(
                (k, v.get("sha256")) for k, v in files.items()
                if isinstance(v, dict))
            out["checkpoints"] = m.get("checkpoints")
        else:
            for name in sorted(os.listdir(weights_dir)):
                if name.endswith(".npz"):
                    out["digests"][name] = file_digest(
                        os.path.join(weights_dir, name))
    except Exception as exc:            # noqa: BLE001 -- best effort by design
        out["error"] = "%s: %s" % (type(exc).__name__, exc)
    return out


def git_provenance(path):
    """HEAD sha + branch + dirty flag for the repo containing `path`.

    `git describe` is deliberately NOT used: the board has no tags, and a
    describe failure would make this return nothing on the one machine that
    matters. Dirty is reported separately because a dirty tree means the sha
    does not identify what ran -- validate_meta() escalates that.
    """
    out = {"head": None, "branch": None, "dirty": None}
    if not path:
        return out
    cwd = path if os.path.isdir(path) else os.path.dirname(path)
    try:
        def _git(*args):
            p = subprocess.Popen(("git", "-C", cwd) + args,
                                 stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            so, _ = p.communicate()
            if p.returncode != 0:
                return None
            return so.decode("utf-8", "replace").strip()
        out["head"] = _git("rev-parse", "HEAD")
        out["branch"] = _git("rev-parse", "--abbrev-ref", "HEAD")
        status = _git("status", "--porcelain")
        if status is not None:
            out["dirty"] = bool(status)
    except Exception as exc:            # noqa: BLE001
        out["error"] = "%s: %s" % (type(exc).__name__, exc)
    return out


def make_run_id(prefix="run", now=None, nonce=None):
    """Sortable-ish id that does NOT depend on the board clock being right.

    The board runs a fake-clock snapshot and is late by however many days it
    was powered off (resume brief section 6) -- bag FILENAMES already carry
    wrong dates because of it. The timestamp here is a convenience for a human
    scanning a directory; the 4-hex nonce is what makes the id unique.
    Analysis joins on this whole string and never parses the date out of it.
    """
    tm = time.localtime(now) if now is not None else time.localtime()
    if nonce is None:
        nonce = binascii.hexlify(os.urandom(2)).decode("ascii")
    return "%s-%s-%s" % (prefix, time.strftime("%Y%m%d-%H%M%S", tm), nonce)


# --------------------------------------------------------------- payloads
def _text(v):
    """py2: promote a UTF-8 byte string to unicode so json.dumps cannot raise.

    An operator note in Korean (ALBC_NOTES) arrives from the parameter server
    as a py2 `str` when the transport hands back bytes. json.dumps then throws
    UnicodeDecodeError, _emit swallows it, and the run continues with NO
    manifest and one throttled warning -- the silent-empty class this module
    exists to remove. py3 str passes through untouched.
    """
    if isinstance(v, bytes) and bytes is not str:      # py3 bytes: leave alone
        return v
    try:
        if isinstance(v, str):
            return v.decode("utf-8")                   # py2 only; py3 str has no decode
    except (UnicodeDecodeError, AttributeError):
        pass
    return v


def build_meta(run_id, node, controller, scenario=None, params=None,
               pack=None, git=None, contract=None, initial=None,
               mixer=None, notes=None, wall_clock=None):
    """The run manifest. Pure -- no ROS, no I/O. Tested off-board."""
    scenario = _text(scenario)
    notes = _text(notes)
    return {
        "schema_version": SCHEMA_VERSION,
        "run_id": run_id,
        "node": node,
        "controller": controller,
        "scenario": scenario,
        # BOARD wall clock, labelled as such because it is frequently wrong.
        # Every time axis in the analysis comes from the bag, not from here.
        "wall_clock_board": wall_clock or time.strftime("%Y-%m-%dT%H:%M:%S"),
        "params": params or {},
        "pack": pack or {},
        "git": git or {},
        "mixer": mixer or {},
        "contract": contract or {},
        "initial": initial or {},
        "notes": notes or "",
    }


def build_event(seq, kind, name, t=None, data=None):
    """One run event. Pure.

    `name` is the label an analyzer segments on ("step_roll_+15", "descend",
    "grasp"); `data` carries whatever that kind needs.
    """
    if kind not in EVENT_KINDS:
        raise ValueError("unknown event kind %r (allowed: %s)"
                         % (kind, ", ".join(EVENT_KINDS)))
    return {
        "schema_version": SCHEMA_VERSION,
        "seq": int(seq),
        "kind": kind,
        "name": name,
        "t": t,
        "data": data or {},
    }


def validate_meta(meta):
    """Return a list of problems; empty means the manifest is usable.

    This is the type check a String payload does not get. It runs in the
    analysis pipeline on every bag, so a node that drifted from the schema is
    caught when the data is READ -- rather than when a figure looks strange.
    """
    problems = []
    if not isinstance(meta, dict):
        return ["meta is not an object"]
    if meta.get("schema_version") != SCHEMA_VERSION:
        problems.append("schema_version %r != %d"
                        % (meta.get("schema_version"), SCHEMA_VERSION))
    for key in ("run_id", "node", "controller"):
        if not meta.get(key):
            problems.append("missing %s" % key)
    git = meta.get("git") or {}
    if not git.get("head"):
        # No code identity at all. Reachable when git_provenance was pointed at
        # a path outside the repo (a deploy pack, say): every field comes back
        # None and the manifest would otherwise validate clean.
        problems.append("git.head missing -- nothing identifies the code that ran")
    if meta.get("controller") not in CONTROLLERS:
        problems.append("unknown controller %r (allowed: %s)"
                        % (meta.get("controller"), ", ".join(CONTROLLERS)))
    pack = meta.get("pack") or {}
    if meta.get("controller") == "rl" and not (pack.get("tag")
                                               or pack.get("digests")):
        # An RL run whose weights cannot be identified cannot be compared to
        # another RL run. That is exactly the 2026-09-02 "same checkpoint?"
        # defect, made mechanical.
        problems.append("rl run with no pack identity (tag or digests)")
    git = meta.get("git") or {}
    if git.get("dirty"):
        problems.append("git tree was DIRTY -- head %s does not identify what ran"
                        % (git.get("head") or "?"))
    return problems


def validate_event(evt):
    problems = []
    if not isinstance(evt, dict):
        return ["event is not an object"]
    if evt.get("kind") not in EVENT_KINDS:
        problems.append("unknown kind %r" % evt.get("kind"))
    if not evt.get("name"):
        problems.append("missing name")
    if not isinstance(evt.get("seq"), int):
        problems.append("seq is not an int")
    return problems


# --------------------------------------------------------------- ROS glue
class RunLogger(object):
    """Thin rospy wrapper. The rospy import cost is paid only on the board.

    Deliberately forgiving: a logging failure must never take down a control
    node, so every publish is wrapped and a failure warns (throttled) instead
    of raising into the caller's control loop.
    """

    def __init__(self, node, controller, scenario=None, queue_size=10,
                 run_id=None):
        import rospy                                   # noqa: PLC0415
        from std_msgs.msg import String                # noqa: PLC0415
        self._rospy = rospy
        self._String = String
        self.node = node
        self.controller = controller
        self.scenario = scenario
        self.run_id = run_id or make_run_id(prefix=controller or "run")
        self._seq = 0
        # latch=True: the recorder in the same launch connects ~1 s later, and
        # an unlatched manifest published at __init__ reaches nobody.
        self._pub_meta = rospy.Publisher(META_TOPIC, String,
                                         queue_size=1, latch=True)
        self._pub_evt = rospy.Publisher(EVENT_TOPIC, String,
                                        queue_size=queue_size)

    def publish_meta(self, params=None, pack=None, git=None, contract=None,
                     initial=None, mixer=None, notes=None):
        meta = build_meta(self.run_id, self.node, self.controller,
                          scenario=self.scenario, params=params, pack=pack,
                          git=git, contract=contract, initial=initial,
                          mixer=mixer, notes=notes)
        problems = validate_meta(meta)
        if problems:
            # Loud, but not fatal. An imperfect manifest still beats none, and
            # refusing to run over thin provenance would strand the operator
            # with a wet robot -- the same trade _home_arm's timeout makes.
            self._rospy.logwarn("run_meta has %d problem(s): %s",
                                len(problems), "; ".join(problems))
        self._emit(self._pub_meta, meta)
        self._rospy.loginfo("run_id %s  (%s / %s)  -> %s",
                            self.run_id, self.controller,
                            self.scenario or "-", META_TOPIC)
        return meta

    def event(self, kind, name, **data):
        """Emit one event. Returns the dict sent, or None if it was rejected."""
        try:
            t = self._rospy.get_time()
        except Exception:                              # noqa: BLE001
            t = None
        # Build FIRST, then commit the sequence number. The header promises a
        # monotonic seq, so an analyzer checks completeness with it; a number
        # burned by a rejected event leaves a permanent hole that reads exactly
        # like a dropped message.
        try:
            evt = build_event(self._seq + 1, kind, name, t=t, data=data)
        except ValueError as exc:
            self._rospy.logerr("run_event rejected: %s", exc)
            return None
        self._seq += 1
        evt["run_id"] = self.run_id
        self._emit(self._pub_evt, evt)
        return evt

    def wait_recorded(self, timeout_s=3.0, poll_s=0.05):
        """Block until BOTH publishers have a subscriber, or timeout. -> bool.

        Only needed before an early exit. A latched topic is served by the
        LIVE publisher, so a node that publishes its manifest and shuts down
        immediately leaves nothing behind: the recorder connects about a second
        into the launch and finds the publisher already gone. The event
        publisher is worse -- it has no latch at all, so its first message goes
        out before any connection is established and is simply dropped.

        That is exactly the refusal path: publish manifest, mark the guard,
        signal_shutdown. It produced a bag with no record of the refusal and no
        sign that anything was missing -- the silent-empty failure this module
        exists to remove, reintroduced one layer up.

        Returns False on timeout; the caller should exit anyway. A refused run
        that also hangs is worse than one that is not written down.
        """
        try:
            deadline = self._rospy.get_time() + float(timeout_s)
            while self._rospy.get_time() < deadline:
                if (self._pub_meta.get_num_connections() > 0
                        and self._pub_evt.get_num_connections() > 0):
                    # connected: give the messages a tick to actually go out
                    self._rospy.sleep(poll_s)
                    return True
                self._rospy.sleep(poll_s)
        except Exception:                              # noqa: BLE001
            return False
        self._rospy.logwarn(
            "run_meta/run_event have no subscriber after %.1f s -- this run "
            "is not being recorded", timeout_s)
        return False

    # the four names an analyzer segments on
    def phase(self, name, **data):
        return self.event("phase", name, **data)

    def setpoint(self, name, **data):
        return self.event("setpoint", name, **data)

    def guard(self, name, **data):
        return self.event("guard", name, **data)

    def mark(self, name, **data):
        return self.event("mark", name, **data)

    def _emit(self, pub, payload):
        try:
            pub.publish(self._String(json.dumps(payload, sort_keys=True)))
        except Exception as exc:                       # noqa: BLE001
            self._rospy.logwarn_throttle(
                10.0, "run_log publish failed (%s: %s) -- the run continues, "
                      "but this bag is missing records"
                      % (type(exc).__name__, exc))


# --------------------------------------------------------------- self-check
def demo():
    """Runnable anywhere, py2 or py3:  python run_log.py

    Covers the two things that actually break -- a manifest that cannot
    identify an RL checkpoint, and an event-kind typo.
    """
    meta = build_meta("rl-20260906-231500-ab12", "rl_inference_node", "rl",
                      scenario="s1_attitude",
                      pack={"tag": "pack_r3a_p3b7500_gru_260906_145553"},
                      git={"head": "c7c9f26", "dirty": False})
    assert validate_meta(meta) == [], validate_meta(meta)
    assert json.loads(json.dumps(meta)) == meta

    bad = build_meta("x", "n", "rl")                  # rl run, no pack identity
    assert any("pack identity" in p for p in validate_meta(bad)), validate_meta(bad)

    dirty = build_meta("x", "n", "tdc", git={"head": "abc", "dirty": True})
    assert any("DIRTY" in p for p in validate_meta(dirty))

    unknown = build_meta("x", "n", "magic")
    assert any("unknown controller" in p for p in validate_meta(unknown))

    evt = build_event(1, "phase", "step_roll_+15", t=12.5, data={"deg": 15})
    assert validate_event(evt) == []
    assert validate_event({"kind": "typo", "name": "", "seq": "1"})

    try:
        build_event(2, "typo", "x")
    except ValueError:
        pass
    else:
        raise AssertionError("bad event kind was accepted")

    rid = make_run_id("rl")
    parts = rid.split("-")
    assert parts[0] == "rl" and len(parts) == 4 and len(parts[3]) == 4, rid
    assert make_run_id("rl", nonce="dead").endswith("-dead")

    # provenance helpers must never raise on a missing path
    assert file_digest("/no/such/file") is None
    assert isinstance(pack_provenance("/no/such/dir"), dict)
    assert isinstance(git_provenance("/no/such/dir"), dict)

    # the board keeps one manifest PER ENCODER, not MANIFEST.json. Reading the
    # 05_deploy archive's layout instead of the robot's cost the tag on the
    # first live run: digests were right, tag was None, nothing errored.
    import shutil
    import tempfile
    d = tempfile.mkdtemp()
    try:
        for a, tag in (("gru", "pack_gru_X"), ("tcn", "pack_tcn_Y")):
            with open(os.path.join(d, "MANIFEST.%s.json" % a), "w") as f:
                json.dump({"tag": tag, "checkpoints": {"student_%s" % a: {}},
                           "files": {"weights_%s.npz" % a: {"sha256": "de" + a}}}, f)
        got = pack_provenance(d, arch="gru")
        assert got["tag"] == "pack_gru_X", got
        assert got["manifest"] == "MANIFEST.gru.json", got
        assert got["digests"] == {"weights_gru.npz": "degru"}, got
        # two manifests and no arch: do NOT guess, and say what was found
        amb = pack_provenance(d)
        assert amb["tag"] is None, amb
        assert amb["manifest_candidates"] == ["MANIFEST.gru.json",
                                              "MANIFEST.tcn.json"], amb
        # a single manifest under any name is unambiguous
        os.remove(os.path.join(d, "MANIFEST.tcn.json"))
        assert pack_provenance(d)["tag"] == "pack_gru_X"
    finally:
        shutil.rmtree(d)

    print("run_log: OK")


if __name__ == "__main__":
    demo()
