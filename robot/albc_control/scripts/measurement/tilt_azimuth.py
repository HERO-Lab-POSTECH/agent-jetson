#!/usr/bin/env python
"""tilt_azimuth.py -- body-frame azimuth of a tilt. Arm/IMU frame calibration.

WHAT THIS ANSWERS
-----------------
"When the robot is tilted so that physical side X goes up, where does side X sit
in the SIM body frame?"  Answering it four times, 90 degrees apart, pins the
whole mapping between the operator's view of the robot and the frame the policy
actually consumes -- including the sign of theta1 = 0, which is what a wrong
answer would put 180 degrees out.

HOLD IT IN THE AIR. NOTHING MAY TOUCH THE ROBOT BUT YOUR HANDS.
---------------------------------------------------------------
That is the whole procedural requirement, and it is the ONLY one that matters.

A robot still resting on a surface while one side is lifted pivots about whatever
frame edge happens to touch down, so the tilt direction is dragged toward the
chassis geometry rather than toward where you pushed. Measured 2026-08-11: two
tilts 90 degrees apart in J1 produced an azimuth slope of -1.243 instead of
-1.000 -- a 22% error, i.e. +-10..20 degrees on a single reading.

Lift the robot fully clear and the contact constraint does not exist, so the
error does not either. A rigid board the robot is chocked to works for the same
reason (robot tilt == board tilt by construction) but buys nothing over hands and
is not worth building. What ruins either version is PARTIAL contact -- one corner
still on the bench, a cable taking weight.

Hand tremor is not a concern: static sensor spread is 0.0004 deg (measured, 135
msgs over 6 s at 22.5 Hz) and the mean over --n samples absorbs a +-1..2 deg
wobble about a steady mean pose. Just hold it still for the sampling window
(default 60 samples ~ 2.7 s) and keep the tilt in the 15-20 deg band.

FRAME CONVENTION
----------------
World-up expressed in the body frame is
    u_b = (-sin(pitch), sin(roll)*cos(pitch), cos(roll)*cos(pitch))
so its horizontal part points at
    alpha = atan2(sin(roll)*cos(pitch), -sin(pitch))
In this FLU right-handed frame a POSITIVE pitch is nose-DOWN (a rotation about +y
carries +x toward -z), so pitch>0 gives alpha = 180 deg and 180 deg is the side
that went UP -- the tail. **alpha ALREADY points at the raised side.**

FIXED 2026-08-13: this docstring used to say pitch>0 was nose-UP, and the code
therefore added another 180. That inversion is what produced imu_yaw_offset
= -78.0, itself 180 deg from the correct +102.0. Verified against measured data
before the fix: 2026-08-12 dry, 3-o'clock side held DOWN gave cor_pitch = +40.8,
so the raised side is 9 o'clock = 180 deg in the sim body frame (+x = 3 o'clock),
and atan2(0, -sin(40.8)) = 180 deg. Equal -- no extra offset belongs here.

The correction is NOT re-derived here: rotate_imu is imported from build_proprio,
which is itself pinned to the C++ oracle imu_rotation.h. There are already three
copies of that formula in this repo (C++ oracle, imu_processor, build_proprio);
a fourth transcription is exactly how this project has drifted before.

Note the composed map (rotate_imu then alpha) has determinant -1 -- it is a
REFLECTION, alpha_out = 135 deg - alpha_in for a 45 deg offset. The raw frame is
left-handed and the pinned `raw_pitch = -(PITCH)` is what restores handedness.
That is why self-check 2 (ordering) matters as much as self-check 1 (spacing).

TILT SIZE: rotate_imu spins the (roll, pitch) PAIR as if it were a 2-vector,
which is exact only in the small-angle limit -- Euler angles are not a vector.
The total tilt therefore is not preserved: replaying the 2026-08-11 readings,
28.40 deg raw became 28.69 deg corrected (M1) -- 0.3 deg of it at ~30 deg tilt,
growing with tilt. This is a property of the deployed correction, not of this
script (the policy sees exactly this frame), but it is why 15-20 deg is the sweet
spot: big enough to beat sensor noise, small enough that the approximation is
still tight.

This script only SUBSCRIBES. It publishes nothing and moves nothing.

USAGE
-----
    # hold the robot clear of everything, tilt 15-20 deg, hold still, then run.
    # once per raised side, 4 sides, ~90 deg apart.
    rosrun albc_control tilt_azimuth.py measure --label gripper-up
    rosrun albc_control tilt_azimuth.py measure --label left-up
    ...
    rosrun albc_control tilt_azimuth.py check --gripper-j1 135.44

Rows accumulate in ~/albc_diag/tilt_azimuth.csv. `check` re-reads them and runs
the self-checks; it never writes.
"""
from __future__ import print_function

import argparse
import csv
import datetime
import math
import os
import sys

import numpy as np

# rotate_imu lives in the albc_rl package; import it rather than re-transcribe.
_HERE = os.path.dirname(os.path.abspath(__file__))
_RL_SRC = os.path.normpath(os.path.join(_HERE, "..", "..", "..", "albc_rl", "src"))
if _RL_SRC not in sys.path:
    sys.path.insert(0, _RL_SRC)
from albc_rl.build_proprio import rotate_imu  # noqa: E402
from albc_rl.contract import TOPICS  # noqa: E402

# J1 Homing Offset (EEPROM addr 20) the `check` suggestions are measured against.
# CONFIRMED 2026-08-12 by two link1 points 90 deg apart (residual 0.3 deg).
# Dead values, do not resurrect: -1029, -2908, -2021.
J1_HOMING_OFFSET = -1509

# v2 because `high_side_deg` changed meaning on 2026-08-13 (the spurious +180 was
# removed). Rows written before that sit in the v1 file under the OLD convention;
# mixing the two in one file would silently corrupt `check`. v1 is left in place.
CSV_PATH = os.path.expanduser("~/albc_diag/tilt_azimuth_v2.csv")
CSV_PATH_V1 = os.path.expanduser("~/albc_diag/tilt_azimuth.csv")
HEADER = [
    "ts_iso", "label", "n", "offset_deg",
    "raw_roll", "raw_pitch", "cor_roll", "cor_pitch",
    "tilt_deg", "alpha_raw_deg", "alpha_cor_deg", "high_side_deg",
]
MIN_TILT_DEG = 10.0   # below this the azimuth is noise-dominated
MAX_TILT_DEG = 30.0   # above this rotate_imu's small-angle approximation loosens
MAX_SEM_DEG = 1.0     # standard error of the mean pose; hand wobble is fine, drift is not


def azimuth_deg(roll, pitch):
    """Azimuth of world-up's horizontal component, in the frame roll/pitch describe."""
    return math.degrees(math.atan2(math.sin(roll) * math.cos(pitch), -math.sin(pitch)))


def tilt_deg(roll, pitch):
    """Total tilt off vertical: angle between body +z and world up."""
    return math.degrees(math.acos(max(-1.0, min(1.0, math.cos(roll) * math.cos(pitch)))))


def wrap180(d):
    return (d + 180.0) % 360.0 - 180.0


def _measure(args):
    import rospy
    from hero_msgs.msg import hero_agent_sensor

    samples = []

    def cb(msg):
        if len(samples) < args.n:
            samples.append((msg.ROLL, msg.PITCH, msg.YAW))

    rospy.init_node("tilt_azimuth", anonymous=True, disable_signals=True)
    rospy.Subscriber(TOPICS["sensors"], hero_agent_sensor, cb, queue_size=1)

    print("collecting %d samples from /hero_agent/sensors ..." % args.n)
    deadline = rospy.Time.now() + rospy.Duration(args.timeout)
    rate = rospy.Rate(50)
    while len(samples) < args.n and rospy.Time.now() < deadline and not rospy.is_shutdown():
        rate.sleep()
    if len(samples) < args.n:
        print("FAIL: got %d/%d samples in %.0fs -- is the firmware publishing?"
              % (len(samples), args.n, args.timeout))
        return 1

    a = np.array(samples, dtype=np.float64)
    raw_roll, raw_pitch, yaw = a[:, 0].mean(), a[:, 1].mean(), a[:, 2].mean()
    # Hand-held wobble is EXPECTED and harmless -- what matters is how well the
    # mean pose is pinned, i.e. the standard error, not the spread.
    jitter = math.degrees(max(a[:, 0].std(), a[:, 1].std()))
    sem = jitter / math.sqrt(len(samples))

    off = math.radians(args.offset_deg)
    cor = rotate_imu(raw_roll, raw_pitch, yaw, off)
    cor_roll, cor_pitch = float(cor[0]), float(cor[1])

    t = tilt_deg(cor_roll, cor_pitch)
    a_raw = azimuth_deg(raw_roll, raw_pitch)
    a_cor = azimuth_deg(cor_roll, cor_pitch)
    # alpha ALREADY points at the raised side -- see FRAME CONVENTION. The old
    # `+ 180.0` here is the defect that produced imu_yaw_offset = -78.0.
    high = wrap180(a_cor)

    print("  raw   roll=%+.4f pitch=%+.4f   alpha_raw=%+.2f deg" % (raw_roll, raw_pitch, a_raw))
    print("  corr  roll=%+.4f pitch=%+.4f   alpha_cor=%+.2f deg" % (cor_roll, cor_pitch, a_cor))
    print("  tilt  %.2f deg   hold spread %.3f deg   mean pinned to +-%.3f deg"
          % (t, jitter, sem))
    print("  ==> RAISED SIDE ('%s') sits at %+.2f deg in the sim body frame" % (args.label, high))

    if t < MIN_TILT_DEG:
        print("  WARN tilt %.1f deg < %.0f -- azimuth is noise-dominated, tilt more"
              % (t, MIN_TILT_DEG))
    if t > MAX_TILT_DEG:
        print("  WARN tilt %.1f deg > %.0f -- rotate_imu's small-angle approximation"
              % (t, MAX_TILT_DEG))
        print("       is loosening; 15-20 deg is the band to aim for")
    if sem > MAX_SEM_DEG:
        print("  WARN mean only pinned to +-%.2f deg (> %.1f) -- hold steadier or raise --n"
              % (sem, MAX_SEM_DEG))

    row = [datetime.datetime.now().replace(microsecond=0).isoformat(),
           args.label, args.n, "%.1f" % args.offset_deg,
           "%.4f" % raw_roll, "%.4f" % raw_pitch, "%.4f" % cor_roll, "%.4f" % cor_pitch,
           "%.2f" % t, "%.2f" % a_raw, "%.2f" % a_cor, "%.2f" % high]

    d = os.path.dirname(CSV_PATH)
    if not os.path.isdir(d):
        os.makedirs(d)
    exists = os.path.isfile(CSV_PATH)
    with open(CSV_PATH, "a") as f:
        w = csv.writer(f)
        if not exists:
            w.writerow(HEADER)
        w.writerow(row)
    print("  appended to %s" % CSV_PATH)
    return 0


def _check(args):
    if not os.path.isfile(CSV_PATH):
        print("FAIL: %s does not exist -- run `measure` first" % CSV_PATH)
        return 1
    with open(CSV_PATH) as f:
        rows = list(csv.DictReader(f))
    if not rows:
        print("FAIL: %s has no rows" % CSV_PATH)
        return 1

    # last measurement of each of the 4 most recent distinct labels, in measured order
    seen, keep = set(), []
    for r in reversed(rows):
        if r["label"] not in seen:
            seen.add(r["label"])
            keep.append(r)
        if len(keep) == 4:
            break
    keep.reverse()

    print("last %d measurements:" % len(keep))
    for r in keep:
        print("  %-16s raised side at %+8s deg   (tilt %s deg)"
              % (r["label"], r["high_side_deg"], r["tilt_deg"]))

    ok = True
    if len(keep) < 4:
        print("\nSELF-CHECK 1 (90 deg spacing): SKIPPED -- need 4 distinct labels, have %d"
              % len(keep))
        ok = False
    else:
        highs = [float(r["high_side_deg"]) for r in keep]
        gaps = [wrap180(highs[(i + 1) % 4] - highs[i]) for i in range(4)]
        worst = max(abs(abs(g) - 90.0) for g in gaps)
        print("\nSELF-CHECK 1 (90 deg spacing): gaps %s" % ["%+.1f" % g for g in gaps])
        if worst <= args.tol:
            print("  PASS  worst deviation %.1f deg <= %.1f" % (worst, args.tol))
        else:
            ok = False
            print("  FAIL  worst deviation %.1f deg > %.1f -- the robot moved relative to"
                  % (worst, args.tol))
            print("        the board, or the four sides were not actually 90 deg apart")

        # Handedness. In FLU (+x fwd, +y left, +z up) azimuth grows from +x toward
        # +y, i.e. COUNTER-clockwise seen from above. So measuring the raised side
        # in clockwise order must make the azimuth DECREASE. If it increases, the
        # corrected frame is still left-handed -- the pinned raw_pitch = -(PITCH)
        # is wrong and every attitude-derived judgement downstream is suspect.
        want = -1 if args.order == "cw" else +1
        print("SELF-CHECK 2 (handedness, measured %s from above): " % args.order, end="")
        if not (all(g > 0 for g in gaps) or all(g < 0 for g in gaps)):
            ok = False
            print("FAIL  gaps change sign -- not a consistent rotation")
        elif (1 if gaps[0] > 0 else -1) == want:
            print("PASS  azimuth %s as expected -- corrected frame is right-handed"
                  % ("decreases" if want < 0 else "increases"))
        else:
            ok = False
            print("FAIL  azimuth %s but %s order requires the opposite."
                  % ("increases" if gaps[0] > 0 else "decreases", args.order))
            print("      The corrected frame is LEFT-handed: rotate_imu's pinned")
            print("      raw_pitch = -(PITCH) does not restore handedness here. STOP --")
            print("      do not run the policy; every attitude judgement is sign-suspect.")

    grip = [r for r in keep if "grip" in r["label"].lower()]
    print("SELF-CHECK 3 (theta1=0 azimuth): ", end="")
    if not grip:
        print("SKIPPED -- no row whose label contains 'grip'")
    elif args.gripper_j1 is None:
        print("SKIPPED -- pass --gripper-j1 <deg> (J1 at which the arm points at the gripper)")
    else:
        g = float(grip[-1]["high_side_deg"])
        delta = wrap180(g - args.gripper_j1)
        print("gripper at %+.2f deg, arm reaches it at J1=%+.2f" % (g, args.gripper_j1))
        print("  ==> theta1=0 points at %+.2f deg in the sim body frame" % delta)
        if abs(delta) < 90.0:
            print("  ==> +x (FRONT). Matches the sim assumption joint1=0 -> link1 along +x.")
        else:
            print("  ==> -x (REAR). This CONTRADICTS the sim URDF; do NOT run the policy")
            print("      until it is resolved -- the buoy would be commanded to the wrong side.")
        if abs(delta) > args.delta_tol:
            print("  ACTION |delta| = %.1f deg > %.1f: absorb it into J1 Homing Offset."
                  % (abs(delta), args.delta_tol))
            print("         delta_tick = %+.0f  (2048 tick = 180 deg), new offset = %d %+.0f"
                  % (delta / 180.0 * 2048.0, J1_HOMING_OFFSET, delta / 180.0 * 2048.0))
        else:
            print("  ACTION none: |delta| = %.1f deg <= %.1f, J1 offset %d stands."
                  % (abs(delta), args.delta_tol, J1_HOMING_OFFSET))

    print("\n%s" % ("ALL STRUCTURAL CHECKS PASS" if ok else "SOME CHECKS FAILED -- see above"))
    return 0 if ok else 1


def _selftest(args):
    """Pin the azimuth convention against measured data. No robot, no CSV."""
    # 2026-08-12 dry: 3-o'clock side held DOWN gave cor_pitch = +40.8 deg, roll ~ 0.
    # In the sim body frame +x = 3 o'clock, so the side that went UP is 9 o'clock
    # = 180 deg. The raised side must come out at 180, not at 0.
    high = wrap180(azimuth_deg(0.0, math.radians(40.8)))
    assert abs(wrap180(high - 180.0)) < 1.0, \
        "raised side %+.1f deg, expected 180 -- the +180 defect is back" % high

    # Mirror case: 9-o'clock side down (pitch negative) must raise 3 o'clock = 0 deg.
    high = wrap180(azimuth_deg(0.0, math.radians(-40.8)))
    assert abs(wrap180(high)) < 1.0, \
        "raised side %+.1f deg, expected 0" % high

    # Roll-only: positive roll raises +y = 12 o'clock = +90 deg.
    high = wrap180(azimuth_deg(math.radians(20.0), 0.0))
    assert abs(wrap180(high - 90.0)) < 1.0, \
        "raised side %+.1f deg, expected +90" % high

    assert J1_HOMING_OFFSET == -1509, "J1 offset drifted from the confirmed value"
    print("selftest OK -- azimuth points at the RAISED side; J1 offset %d"
          % J1_HOMING_OFFSET)
    return 0


def main():
    p = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    sub = p.add_subparsers(dest="cmd")

    m = sub.add_parser("measure", help="record one raised-side azimuth")
    m.add_argument("--label", required=True,
                   help="which physical side is UP, named against the gripper "
                        "(e.g. gripper-up, gripper-down, left-up, right-up)")
    m.add_argument("--offset-deg", type=float, default=102.0,
                   help="IMU mounting yaw offset (albc_controller.yaml imu_yaw_offset)")
    m.add_argument("--n", type=int, default=60,
                   help="samples to average (~2.7 s at 22.5 Hz -- a comfortable hold)")
    m.add_argument("--timeout", type=float, default=20.0, help="seconds to wait for samples")
    m.set_defaults(func=_measure)

    c = sub.add_parser("check", help="re-read the CSV and run the self-checks")
    c.add_argument("--gripper-j1", type=float, default=None,
                   help="J1 in degrees at which the arm points at the gripper (2026-08-11: 135.44)")
    c.add_argument("--order", choices=("cw", "ccw"), default="cw",
                   help="the order the four sides were measured in, seen FROM ABOVE "
                        "(default cw: gripper, then +90 clockwise each time)")
    c.add_argument("--tol", type=float, default=12.0, help="allowed deviation from 90 deg spacing")
    c.add_argument("--delta-tol", type=float, default=10.0,
                   help="|delta| above which J1 Homing Offset should absorb it")
    c.set_defaults(func=_check)

    s = sub.add_parser("selftest", help="pin the azimuth convention (no robot needed)")
    s.set_defaults(func=_selftest)

    args = p.parse_args()
    if not getattr(args, "cmd", None):
        p.print_help()
        return 2
    return args.func(args)


if __name__ == "__main__":
    sys.exit(main())
