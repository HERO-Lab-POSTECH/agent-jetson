#!/usr/bin/env python
"""tilt_azimuth.py -- body-frame azimuth of a tilt. Arm/IMU frame calibration.

WHAT THIS ANSWERS
-----------------
"When the robot is tilted so that physical side X goes up, where does side X sit
in the SIM body frame?"  Answering it four times, 90 degrees apart, pins the
whole mapping between the operator's view of the robot and the frame the policy
actually consumes -- including the sign of theta1 = 0, which is what a wrong
answer would put 180 degrees out.

WHY A TILT BOARD, NOT A LIFTED EDGE
-----------------------------------
A robot resting on the floor and lifted by hand pivots about whatever frame edge
happens to touch down, so the tilt direction is dragged toward the chassis
geometry rather than toward where you pushed. Measured 2026-08-11: two tilts 90
degrees apart in J1 produced an azimuth slope of -1.243 instead of -1.000, a 22%
error, which is +-10..20 degrees on a single reading.

Put the robot on a rigid flat board, CHOCK IT so it cannot move relative to the
board, and tilt the BOARD. Then robot tilt == board tilt by construction and the
pivot ambiguity is gone. If the robot can rock on the board the measurement is
void -- self-check 1 below is what catches that.

FRAME CONVENTION
----------------
World-up expressed in the body frame is
    u_b = (-sin(pitch), sin(roll)*cos(pitch), cos(roll)*cos(pitch))
so its horizontal part points at
    alpha = atan2(sin(roll)*cos(pitch), -sin(pitch))
For a nose-up pitch (>0) that gives alpha = 180 deg, i.e. alpha points at the LOW
side. The raised side is therefore  alpha + 180.

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
    # once per raised side, 4 sides, ~90 deg apart, tilt 15-25 deg
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
_RL_SCRIPTS = os.path.normpath(os.path.join(_HERE, "..", "..", "..", "albc_rl", "scripts"))
if _RL_SCRIPTS not in sys.path:
    sys.path.insert(0, _RL_SCRIPTS)
from build_proprio import rotate_imu  # noqa: E402

CSV_PATH = os.path.expanduser("~/albc_diag/tilt_azimuth.csv")
HEADER = [
    "ts_iso", "label", "n", "offset_deg",
    "raw_roll", "raw_pitch", "cor_roll", "cor_pitch",
    "tilt_deg", "alpha_raw_deg", "alpha_cor_deg", "high_side_deg",
]
MIN_TILT_DEG = 10.0   # below this the azimuth is noise-dominated
MAX_TILT_DEG = 35.0   # above this, suspect the robot slid on the board


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
    rospy.Subscriber("/hero_agent/sensors", hero_agent_sensor, cb, queue_size=1)

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
    jitter = math.degrees(max(a[:, 0].std(), a[:, 1].std()))

    off = math.radians(args.offset_deg)
    cor = rotate_imu(raw_roll, raw_pitch, yaw, off)
    cor_roll, cor_pitch = float(cor[0]), float(cor[1])

    t = tilt_deg(cor_roll, cor_pitch)
    a_raw = azimuth_deg(raw_roll, raw_pitch)
    a_cor = azimuth_deg(cor_roll, cor_pitch)
    high = wrap180(a_cor + 180.0)

    print("  raw   roll=%+.4f pitch=%+.4f   alpha_raw=%+.2f deg" % (raw_roll, raw_pitch, a_raw))
    print("  corr  roll=%+.4f pitch=%+.4f   alpha_cor=%+.2f deg" % (cor_roll, cor_pitch, a_cor))
    print("  tilt  %.2f deg   sample jitter %.3f deg" % (t, jitter))
    print("  ==> RAISED SIDE ('%s') sits at %+.2f deg in the sim body frame" % (args.label, high))

    if t < MIN_TILT_DEG:
        print("  WARN tilt %.1f deg < %.0f -- azimuth is noise-dominated, tilt more"
              % (t, MIN_TILT_DEG))
    if t > MAX_TILT_DEG:
        print("  WARN tilt %.1f deg > %.0f -- check the robot did not slide on the board"
              % (t, MAX_TILT_DEG))
    if jitter > 0.5:
        print("  WARN jitter %.2f deg -- robot is rocking; chock it to the board" % jitter)

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

        print("SELF-CHECK 2 (handedness): ", end="")
        if all(g > 0 for g in gaps) or all(g < 0 for g in gaps):
            print("PASS  azimuth advances monotonically")
            print("  NOTE compare that direction against the physical order you measured.")
            print("       If you went counter-clockwise (seen from above) and the azimuths")
            print("       DECREASE, the corrected frame is still left-handed -- the pinned")
            print("       raw_pitch = -(PITCH) is wrong and everything downstream is suspect.")
        else:
            ok = False
            print("FAIL  gaps change sign -- readings are not a consistent rotation")

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
            print("         delta_tick = %+.0f  (2048 tick = 180 deg), new offset = -1029 %+.0f"
                  % (delta / 180.0 * 2048.0, delta / 180.0 * 2048.0))
        else:
            print("  ACTION none: |delta| = %.1f deg <= %.1f, J1 offset -1029 stands."
                  % (abs(delta), args.delta_tol))

    print("\n%s" % ("ALL STRUCTURAL CHECKS PASS" if ok else "SOME CHECKS FAILED -- see above"))
    return 0 if ok else 1


def main():
    p = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    sub = p.add_subparsers(dest="cmd")

    m = sub.add_parser("measure", help="record one raised-side azimuth")
    m.add_argument("--label", required=True,
                   help="which physical side is UP, named against the gripper "
                        "(e.g. gripper-up, gripper-down, left-up, right-up)")
    m.add_argument("--offset-deg", type=float, default=45.0,
                   help="IMU mounting yaw offset (albc_controller.yaml imu_yaw_offset)")
    m.add_argument("--n", type=int, default=100, help="samples to average")
    m.add_argument("--timeout", type=float, default=20.0, help="seconds to wait for samples")
    m.set_defaults(func=_measure)

    c = sub.add_parser("check", help="re-read the CSV and run the self-checks")
    c.add_argument("--gripper-j1", type=float, default=None,
                   help="J1 in degrees at which the arm points at the gripper (2026-08-11: 135.44)")
    c.add_argument("--tol", type=float, default=12.0, help="allowed deviation from 90 deg spacing")
    c.add_argument("--delta-tol", type=float, default=10.0,
                   help="|delta| above which J1 Homing Offset should absorb it")
    c.set_defaults(func=_check)

    args = p.parse_args()
    if not getattr(args, "cmd", None):
        p.print_help()
        return 2
    return args.func(args)


if __name__ == "__main__":
    sys.exit(main())
