# ADR 002 — Arm protection: sustained-current cap and start-attitude gate

Status: accepted (2026-08-25), joint2 hard window rescinded 2026-08-26
Code: `robot/albc_rl/scripts/rl_inference_node.py` (`~joint_current_max_ma`,
`~joint_current_max_s`, `_gate_start_pose`, `_arm_guard`)

The text below was moved out of the source on 2026-09-04, verbatim. It is the
reasoning behind the numbers, not the numbers themselves; the code keeps the
one-line statement of what each knob does.

---

--- ARM PROTECTION, added 2026-08-25 after arm2 fractured -------------
The E2 run that broke arm2 had NONE of these. It started from a body
tilt of 50.58 deg (the arm had drifted 160 deg during a 417 s torque-off
driver handover), the policy answered with a 170 deg joint2 slew in 9 s
at up to 47 deg/s, joint2 current peaked at 1323.5 mA, and after 40 load
cycles at 0.623 Hz arm2 broke. Every one of those numbers sat in a region
nothing was watching: JOINT_TARGET_CLAMP is [6*pi, inf] so joint2 is
DELIBERATELY unlimited (np_policy.py:82-86), the 900 mA cap existed only
inside a diagnostic script, and _gate_start_pose watches joint1 winding
while the VEHICLE attitude went unchecked -- it logged "start-pose gate
OK" at pitch -50.4 deg. Evidence and full timeline:
  .community/posts/finding/047-e2-run1-arm2-fracture.md

THE DEFAULTS ARE CALIBRATED ON THAT RUN, NOT GUESSED -- but they are
still DECISIONS, not measurements: arm2's allowable stress and fatigue
limit have never been measured. Each cites the strongest thing that does
exist. Widen deliberately, never silently.

joint2 hard window REMOVED 2026-08-26 (decision/061 A1/A2, guard rollback):
this system is constrained RL (ConstraintTRPO + IPO) and singularity
avoidance is already a TRAINED cost (manipulability_cost,
w = sqrt|sin theta2| >= 0.3); sim never clamps theta2 either. The window
this replaced LATCHED the policy output during an ordinary attitude-
lowering move on 2026-08-25 (policy lifetime 0.255 s, 6 commanded ticks --
notes/2026-08-25-guard-session-retraction-handoff.md) and forbade the
mirror branch [185.16, 354.84] deg on two grounds, of which only one is
refuted: "out of the trained distribution" is wrong (sim resets theta2
uniform over (-pi, pi) -- env.yaml:344-346, finding/059), but "it is the
other elbow solution" is a GEOMETRY claim nobody has ever checked -- no
document measures whether link2 clears the hull past 180 deg
(review/062, MEDIUM). Supervise the first crossing of 180 deg.
Even the hand-written classic controller (TDC) only refuses a
singularity as a START condition, never as a running constraint
(albc_controller.cpp EE-radius gate). See decision/061.
Sustained current, not peak: breakaway from rest legitimately draws
1.1-1.3 A (measured 2026-08-25 by e1_fold_sweep.py) and the 2026-08-22
arm1 break was 1.0 A SUSTAINED. On the break run |I_J2| stayed above
900 mA for a longest run of 1.240 s (first at t = 4.77 s) while 1000 mA
never held longer than 0.44 s -- so 900 mA / 0.5 s fires and 1000 / 0.5
would not. <= 0 on either disables the check.
