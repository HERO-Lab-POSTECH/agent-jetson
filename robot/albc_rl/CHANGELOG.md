# Changelog — albc_rl

RL student-policy 추론 패키지(69D attitude-only, torch-free numpy 런타임)의 변경 이력.
가장 안전크리티컬한 추론 경로이므로 동결 계약(frozen contract) 변경은 모두 여기 기록한다.

## [1.0.0] — 2026-06-15 — CHANGELOG 신설 (보드 배포 가능 상태 박제)

agent-jetson(TX2, ROS lunar/py2.7, numpy 1.11) 보드에서 student 정책을 torch 없이
추론하는 노드의 첫 정식 CHANGELOG. 그동안의 패키지 신설~69D 전환~보드 검증 이력을
소급 정리하고, 동결 계약을 명문화한다.

### Added
- `rl_inference_node` — 보드 RL student 정책 추론 노드 (632bfb8: 패키지 신설,
  69D attitude-only). GYRO 진값 읽기 + rotate_gyro + sensors dict 주입 (dd94aa9).
- `numpy_port/` — torch-free 순전파 재구현 (TRT가 Conv1d/GRU/LayerNorm 파싱 불가 →
  numpy 순전파가 유일 배포 경로). `npforward.py`(순전파), `np_policy.py`(69D obs +
  history + integral), 골든 파리티 테스트 2종(`test_npforward.py`, `test_np_policy_api.py`).
- `rotate_gyro` — sensor→base frame 자이로 변환 (593ccf1; r=yaw rate는 z축 회전
  불변이라 통과). euler 미분 fallback과 진값 우선 분기 (2f35df9).
- dynamic_reconfigure로 `imu_yaw_offset` 실시간 튜닝 (f447156).

### Changed
- obs 차원 87D(26D) → **69D attitude-only**(20D proprio + 46D history + 3D integral).
  선속도 추종 전체 제거. marinegym-isaaclab attitude_only sim에서 계약 추출.
- `/albc_status` → `/albc/status` 네임스페이스 통일 (49d4216, 4-site lock-step).
- npforward.py py2.7 호환: `@` matmul → `np.dot` (6662546, ROS lunar rospy=py2.7).
- package.xml: exec_depend → depend (fbf37b0, catkin_package find_package 정합).

### Verification
- numpy_port 69D e2e 보드 1e-5 파리티 검증 (action 2.68e-7, integral 7.28e-12).
- 실기동 1·2차: 통신두절 해소 후 정책 첫 거동 관측. 안전게이트(STALE 감지 시 HOLDING) 정상.

### Frozen contract (변경 시 반드시 기록)
- POLICY_OBS_DIM = 69, ACTION_DIM = 8, DELTA_SCALE = 0.10.
- integral 3D: leaky, gated(|err| < sigma), clamp ±2.0, on [roll, pitch, yaw_rate].
- numpy 1.11.0 호환 밴드 (keepdims 침묵 무시 함정 — 1.11 미만 금지).

### Notes / 보류
- 69D GRU 체크포인트 미존재 → `test_npforward.py::test_gru`는 `[SKIP]` 가드로 통과(TCN-only 배포).
- 동결 계약 load-time assert 부재 (npz shape·numpy version) — enterprise-cleanup Task 3 대상.
