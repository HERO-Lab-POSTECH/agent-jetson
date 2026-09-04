# ADR 003 — Thruster order, sign set, and the deadband inverse

Status: accepted; order [3,2,4,0,5,1] settled after four wrong values
Code: `robot/albc_rl/scripts/thruster_mixer.py` (`DEFAULT_ORDER`,
`DEFAULT_DEADBAND`, `reallocate`)

Moved out of the source verbatim on 2026-09-04. The code keeps the sign and
order conventions themselves; this is why they are what they are.

---

Order DERIVED 2026-08-12 from the DEPLOYED CHECKPOINT'S OWN allocation matrix.

METHOD (this is the part that matters -- three earlier derivations went wrong by
reading a constant instead of the artifact):
  Do NOT read _ESC_CHANNEL_ORDER out of the sim working tree. A working tree
  cannot tell you what a checkpoint was trained with, and this constant has in
  fact changed under us. Read the matrix the policy actually trained on, out of
  the run's own params/env.yaml, and invert it geometrically:
      M = r x F   =>   Mz = x*Fy - y*Fx,  My = z*Fx - x*Fz,  Mx = y*Fz - z*Fy
  On the +-0.102 diagonal grid each column's position is a UNIQUE solution, and
  the tiny Mx/My terms cross-check it (they all resolve to z = -+0.0099).

DEPLOYED TEACHER: logs/rsl_rl/albc_trpo_teacher/teacher_iter_budget/
  trpo_iterbudget_s30_260805_012813/model_9998.pt -- named as `teacher` by BOTH
  MANIFEST.tcn.json and MANIFEST.gru.json on this board, so it is the matrix the
  shipped weights learned. Its allocation_matrix (params/env.yaml:303):
      Fx [ 0.000,  0.707, -0.707,  0.000, -0.707,  0.707]
      Fy [ 0.000,  0.707,  0.707,  0.000, -0.707, -0.707]
      Fz [ 1.000,  0.000,  0.000,  1.000,  0.000,  0.000]
      Mz [ 0.000, -0.144,  0.144,  0.000, -0.144,  0.144]
  Inverting gives, per SIM column:
      col0 vertical @ 9h    col1 @ 10.5h   col2 @ 1.5h
      col3 vertical @ 3h    col4 @ 4.5h    col5 @ 7.5h

MEASURED firmware channels (b1_channel_probe, dry, one channel at a time, on the
clock face whose 12 is the GRIPPER):
      m0 = 3h vertical   m1 = 1.5h   m2 = 4.5h
      m3 = 9h vertical   m4 = 7.5h   m5 = 10.5h        (m3's motor is DEAD)
  Three independent records agree on this: the 2026-07-05 B1 verbal record
  (m1=rear-left, m2=front-left, m4=front-right, m5=rear-right -- which lands on
  exactly these four clock positions once you read its "front" as the 6h side,
  i.e. opposite the gripper, with left=3h, a consistent right-handed frame), the
  2026-08-11 probe, and a 2026-08-12 re-probe of m1 and m2.

Matching fw channel to sim column BY PHYSICAL POSITION gives the order below.
Note what that means: the sim's horizontal columns sit 90 deg rotated from the
real robot, and its two vertical columns are swapped. The vertical swap is not
news -- constrained-albc 3bb042b says so itself ("STILL OPEN ... vertical Fz/My
row redesign"). The horizontal 90 deg is: 3bb042b mapped m1<-T1, but T1 is at
10.5h while m1 is measured at 1.5h.

WHAT THE PREVIOUS WRONG VALUES GOT WRONG (four of them, all the same class):
  [4,0,1,5,2,3]   applied the sim's own column reorder a SECOND time.
  identity        assumed the sim-side reorder already matched the wiring.
  [3,5,4,0,1,2]   (2026-08-12 afternoon) was derived from _ESC_CHANNEL_ORDER =
                  (4,0,1,5,2,3). That tuple is REAL but STALE: it was introduced
                  2026-07-03 (238932c) and REPLACED by (4,1,3,5,2,0) on
                  2026-07-14 (3bb042b, "rewrite horizontal TAM rows + ESC
                  permutation to 2026-07-06 B1 measurement"). The teacher trained
                  2026-08-05, i.e. AFTER. The claim that every post-238932c
                  config.py commit was "DR/latency work" was simply false, and it
                  was read off a month-stale container checkout.
  [3,2,4,0,5,1]   (2026-08-11) was RIGHT and was discarded on wrong grounds.
                  It is restored here, now backed by the checkpoint artifact
                  rather than by any constant.

YAW IS NOT PERMUTATION-INVARIANT HERE. An earlier note in this file claimed the
four horizontals all carry Mz = +0.144 so any permutation yaws identically. That
was true of the PRE-3bb042b matrix only. The deployed matrix has a 2-2 split
(cols 1,4 = -0.144; cols 2,5 = +0.144), so a wrong horizontal permutation yaws
wrong as well as translating wrong.

OPEN, and deliberately left to ~thruster_sign: matching by position puts a sim
Mz sign on each corner OPPOSITE to the rotation direction recorded on 2026-07-05
(m1,m4 = CW; m2,m5 = CCW). A permutation cannot flip a sign, but the per-channel
sign table can, and a consistent solution provably exists because each horizontal
thruster is tangential to its own radius -- flipping its sign points the thrust
along the opposite tangent, which fixes position and Mz together. That is the
restrained-tank measurement (Phase 2b-0), not something to guess here.
