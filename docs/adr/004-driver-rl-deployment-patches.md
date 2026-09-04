# ADR 004 — Dynamixel driver: what the RL deployment changed, and why

Status: accepted; the running record of every deliberate driver change
Code: `robot/albc_control/src/joint_angle_command.cpp`

Moved out of the source verbatim on 2026-09-04. The file header keeps the two
contract statements (measured position, driver-owned velocity); this keeps the
change log and the reasoning behind each item.

---

joint_angle_command.cpp  —  PATCHED for RL deployment
------------------------------------------------------------------------------
Adds a /albc/joint_states publisher to the existing Dynamixel driver so the RL
inference node can read the arm state WITHOUT opening the serial port itself
(the port has a single owner -- this node). Two deliberate choices, both decided
against the sim contract (observations.py robot.data.joint_pos / joint_vel):

  1. joint POSITION published = ACTUAL measured position (readPosition every loop,
     unwrapped+accumulated the same way absolute_angle is), NOT the commanded
     absolute_angle. The sim obs uses robot.data.joint_pos (true state), so the
     deployed obs must use the measured encoder value to match.

  2. joint VELOCITY published = differentiated HERE at the true loop rate (10 Hz),
     shipped in JointState.velocity. The RL node runs at 50 Hz; if it re-diff'd a
     10 Hz position it would see a 4-tick-zero / 1-tick-spike staircase. The driver
     knows its real dt, so it owns the derivative.

DIFF FROM ORIGINAL (search "RL-DEPLOY"):
  + #include <sensor_msgs/JointState.h>
  + readPositionRad() helper + per-joint measured-angle unwrap accumulator
  + joint_state_pub + publishJointStates() in the main loop
  + (2026-06-12 field-test audit fixes)
    - command subscribers queue_size 10 -> 1 (50 Hz cmd vs 10 Hz loop: stale-burst
      of ~5 serial writes per joint per cycle -> exactly one, latest wins)
    - readPosition() returns success; a failed read no longer injects raw=0 as a
      real angle (+-pi fake delta, +-31 rad/s velocity spike into the RL obs).
      Failed cycles skip the JointState publish; failed INIT reads are fatal.
    - velocity differentiation uses the measured loop dt (ros::Time), not 1/LOOP_HZ
    - Dynamixel hardware error byte surfaced (WARN_THROTTLE) on read/write
    - one-shot "First command received" INFO so the operator sees cmd flow
  + (2026-08-17 cable-break fix — see joint_unwrap.h for the derivation)
    - unwrap is now NEAREST-equivalent, not single-step. The old rule leaked
      (k-1) whole turns on the first command whenever the arm started more
      than one turn from zero, which broke the J1->J2 cable on 2026-08-13.
    - joint1 cable guard: ~joint1_abort_rad (default 6pi = 3 turns) ABORTS the
      run — latches, stops applying commands, holds the last goal, leaves
      torque ON. Checked on BOTH the commanded and the MEASURED angle.
    - constraint metering on /albc/joint_guard. Counted, never enforced: a
      silent clamp would destroy the quantity used to compare TDC / classic
      PID / RL, and all four entry points drive the arm through this node.
Everything else is byte-identical to the board original (verified @ 2026-06-07).
------------------------------------------------------------------------------
