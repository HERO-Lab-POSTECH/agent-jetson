# Architecture decision records

Each file here is the reasoning behind one decision that the code implements.
They exist because the sources had grown long narrative comments — incident
timelines, measurement tables, rejected alternatives — that were valuable and
in the wrong place: they buried the code, and they were duplicated wherever two
files touched the same decision.

The split is deliberate. **The code keeps the convention**, in one to three
lines: the sign rule, the order, the boundary, the "do NOT fix without intent"
warning. **The record keeps the evidence**: what was measured, on which run,
what was decided rather than measured, and what is still unjudged.

If you are about to change one of these behaviours, read the record first. Most
of them exist because somebody already tried the obvious thing.

| Record | Decision | Where it is implemented |
|:---|:---|:---|
| [001](001-joint-unwrap-cable-break.md) | Nearest-equivalent unwrap; no clamp on the joint target | `joint_unwrap.h`, `np_policy.py`, `joint_angle_command.cpp` |
| [002](002-arm-guard-current-cap.md) | Sustained-current cap and start-attitude gate; no joint2 angle window | `rl_inference_node.py` |
| [003](003-thruster-order-and-sign.md) | Channel order [3,2,4,0,5,1], sign set, deadband inverse | `thruster_mixer.py` |
| [004](004-driver-rl-deployment-patches.md) | What the RL deployment changed in the arm driver | `joint_angle_command.cpp` |
| [005](005-obs-assembly-phase.md) | Thruster first-order lag in the observation echo | `build_proprio.py`, `contract.py` |
| [006](006-m4-exclusion-three-channel.md) | m4 permanently excluded; three-channel reallocation | `firmware/agent/pid.cpp` |
| [007](007-esc-deadband-ff.md) | Classic yaw ESC deadband feed-forward | `firmware/agent/pid.cpp` |

Records 006 and 007 describe firmware behaviour that is **not yet judged in
closed loop**. Read the last note in each before the next tank session.
