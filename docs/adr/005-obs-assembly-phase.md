# ADR 005 — Thruster first-order lag in the observation echo

Status: accepted 2026-08-26 (dt corrected from 0.005 to 0.02)
Code: `robot/albc_rl/src/albc_rl/build_proprio.py`, `albc_rl.contract`
(`THR_TAU_UP`, `THR_TAU_DOWN`, `THR_FILTER_DT`)

Moved out of the source verbatim on 2026-09-04. The constants stay pinned in
`contract.py`; this is the audit trail proving which sim commit they match.

---

---- thruster first-order lag (obs echo 14:20 -- must match sim byte-for-byte) ----
The sim feeds obs[14:20] the FILTERED thruster state, not the raw command:
marinelab ThrusterModel.apply_dynamics() runs a first-order lag each env step
(constrained-albc envs/main/albc_env.py:801 -> apply_dynamics(action[2:], step_dt)).
The board previously echoed the RAW action here (std 1.0, |z| up to 8.7 vs the sim
normalizer) -> OOD input the policy never saw in training -> thruster saturation.
We replicate the EXACT recurrence so the board reports what the policy trained on.

Constants PINNED to the live sim:
  ALBCThrusterCfg.time_constant_up   = 0.1   (rising: target > state)
  ALBCThrusterCfg.time_constant_down = 0.05  (falling)
  filter dt = step_dt = physics_dt * decimation = 0.005 * 4 = 0.02  (2026-08-26
    re-verify, decision/061 D, .community/posts/finding/060: apply_dynamics is
    called from _pre_physics_step, once per ENV step, so the elapsed time is
    step_dt, not physics_dt. Confirmed against the live marinelab-isaaclab
    container: commit 9a2768c9 "fix(sim): thruster lag advances with step_dt,
    not physics_dt" (2026-07-12) changed the call site from physics_dt to
    step_dt; config.py:469 physics_dt=0.005, config.py:436 decimation=4. The
    deployed teacher trpo_iterbudget_s30_260805_012813 trained 2026-08-05 --
    24 days AFTER that fix -- so the policy was trained on dt=0.02 while this
    file fed the board dt=0.005 (4x too slow) until this change. The prior
    "verified 2026-07-02" pin here predates 9a2768c9 and was never re-checked
    against it.
  target = raw command clamped to [-1, 1]; recurrence: s += (dt/tau)*(target - s).
