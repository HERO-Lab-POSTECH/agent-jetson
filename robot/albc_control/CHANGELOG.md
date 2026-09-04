# Changelog — albc_control

## [Unreleased] — refactor/cleanup-2026-09 (병합 대기)

브랜치 `refactor/cleanup-2026-09` (base `90b13e4`, 커밋 12개) 의 albc_control 몫.
동작 보존이 1순위였고 제어 수학·게이트 임계값·launch 인자 이름/기본값은 건드리지 않았다.
버전 번호는 `deploy/72d-inc9998-gru` 로 ff-merge 하는 시점에 확정한다.

### Removed
- `albc_controller.cpp` 의 2026-06 분해 때 남긴 "now lives in X" 묘비 주석 74줄 (`466006c`).
- `LEVEL_THRESHOLD 0.01745` 중복 — `control_law.h` 사본을 지우고 `feedback_filters.h` 의
  `ORACLE_LEVEL_THRESHOLD` 하나만 남겼다. C++11 상수식이 되도록 `static const` →
  `static constexpr` (값·경로 불변) (`466006c`).
- `attitude_controller.h` 의 미사용 `<algorithm>`·`<cmath>` include — 그 헤더는 std 심볼을
  직접 쓰지 않는다(실측). `control_law.h` 는 두 include 를 그대로 유지 (`466006c`).
- `joint_angle_command.cpp` 의 공용 루프 dt(`prev_loop_t`) — 관절별 마지막 성공 읽기로 대체
  (`9471176`).
- `test/test_deadband_gate.cpp` — 어느 러너에도 안 물려 있던 고아 테스트를
  `tests/characterization/` 로 옮겨 `run.sh` glob 에 편입 (`8df619f`).

### Added
- 관절 드라이버 `updateJoint()` 진입에 `std::isfinite` 가드. 그 아래의 모든 비교문이 NaN 에
  대해 fail-open 이므로 여기가 와이어 직전 마지막 경계다 (`9471176`).

### Changed
- **토픽 이름 SSOT** (`13c5d0d`, D13 안 A). albc_control 의 C++ 은 이제
  `hero_msgs/include/hero_msgs/topics.h` 하나만 읽는다. 이름 변경 3건: 관절 전류 →
  `/albc/joint_currents`, ros_control 잔재였던 관절 명령 2개 → `/albc/joint1_cmd`·
  `/albc/joint2_cmd`, RL 명령 → `/albc/rl_command`.
  **Jetson 내부 전용 — 펌웨어 구독 미변경, 재flash 불필요.** 옛 이름으로 녹화된 bag 은
  vault `code/albc_diag/topic_aliases.py` 의 alias 로 계속 읽힌다.
- `updateMeasured()` 가 dt 를 **그 관절의** 마지막 성공 읽기에서 낸다. 한쪽 관절 읽기가
  실패하면 다음 델타가 두 주기를 걸치면서 한 주기로 나뉘어 속도를 실제의 2배로 보고하고
  있었다. 양쪽이 성공하는 경로는 값이 동일하므로 클램프(0.005~1.0)는 그대로다 (`9471176`).
- `status_publisher.h` 의 주석이 광고하던 옛 이름 `/albc_status` → `/albc/status` 정정.
  실 발행 이름은 이전부터 `/albc/status` 였고 주석만 낡아 있었다 (`d504f96`).
- `dynamixel_config.h` 주석의 옛 baud 57600 정정 (`d504f96`).
- `scripts/measurement/arm_step_response.py` 가 SIGTERM·SIGHUP 에도 e-stop 을 건다. SIGINT
  핸들러만 있어 프로세스가 kill 되거나 터미널이 닫히면 토크가 남았다 (`9471176`).
- `scripts/measurement/tilt_azimuth.py` 가 albc_rl 의 새 패키지 경로를 import (`66bf43a`).

### Moved to docs/adr (실행 코드 변경 0)
- `joint_unwrap.h`·`joint_angle_command.cpp` 의 사고 경위 서술 → `docs/adr/001`(최근접
  unwrap·관절 목표 무클램프)·`docs/adr/004`(드라이버 RL 배포 패치 이력). 코드에는 규약·
  경계·"의도 없이 고치지 말 것" 경고와 한 줄 포인터만 남겼다. 옮긴 문장은 원문 그대로다.
  주석·빈 줄을 뺀 diff 가 파일마다 0줄임을 실측 (`79397ff`).

### Verification
- `PYTHON=python3.12 bash tests/run_all.sh` → `RUN_ALL: PASS`
  (characterization 13종 533 checks, pytest 83).
- E2E 골든 파리티 PASSED (atol 1e-5, max|action err| 9.54e-07) — 오라클 통합과 드라이버
  수정이 수치 경로를 안 건드렸다는 실측.
- **미실행: 보드 게이트.** clean+full `catkin_make`, py2.7 `run_all.sh`, 건식 launch 3종은
  `tests/board_gate.sh` 로 스크립트화됐지만 보드 미접속(`192.168.2.100:22` timeout,
  2026-09-04 22:07 재확인)으로 한 번도 안 돌았다. **병합 전 필수.**

### Notes
- 적대 검증: codex 인벤토리 + agy 3축 리뷰(BLOCKER 0 / MAJOR 0 / MINOR 3, 전부 종결).
- 드라이버 init 읽기 실패 시 torque ON 유지는 **현행 유지 결정**(D9) — 같은 파일의 설계
  원칙("수중 limp arm 이 더 위험", `tripAbort()`)과 일관.

---

## [2.0.0] — 2026-06-06 — God object 분해 재설계

chain 2(albc_control) 구조적 재설계. albc_controller.cpp의 290줄 main God object를
책임별 6 클래스로 분해. 제어 수학은 oracle 정본 승격으로 거동 byte-identical 보존.

### Removed
- dead `Kd_td` 게인 (TDC D-term은 df46b09에서 제거됨, 미사용 — 게인 구조체·yaml·cfg·
  대시보드에서 제거. 재도입 필요 시 git 이력 참조)
- 전역 상태 12개 (control_mode/gains/state/ik_cfg/imu_yaw_offset/joint_current×2/manual×4
  → 각 클래스 멤버로 흡수)
- blocking stdin 모드 선택 (selectModeInteractive → control_mode param 시작)

### Added
- imu_rotation.h / dls_ik.h / control_law.h / feedback_filters.h — 제어 수학 정본
  (ROS-free, albc_controller.cpp에서 byte-identical 추출 후 정본 승격)
- inverse_kinematics.h — InverseKinematics 클래스 (DLS IK + solveIK 헬퍼, IK 중복 해소)
- imu_processor.h — ImuProcessor 클래스 (IMU 콜백 + yaw offset 회전)
- attitude_controller.h — AttitudeController 클래스 (4모드 제어law + integral/derivative
  gate/LPF + 모드전환 리셋)
- mode_manager.h — ModeManager 클래스 (모드 FSM + 키 핸들링 + 전환 감지)
- dashboard.h — Dashboard 클래스 (ANSI 대시보드 렌더)
- status_publisher.h — StatusPublisher 클래스 (/albc_status 11필드 + joint angle 발행)
- joint_current_monitor.h — JointCurrentMonitor 클래스 (모터 전류 캐시)
- dynamixel_config.h — Dynamixel 모터 설정 상수 SSOT (ID·baud·레지스터·프로파일·gain·변환)
- tests/characterization/ albc 5종 — 제어 수학 박제 골든 테스트 (150 checks, 재설계 안전망)

### Changed
- main()을 290줄 God object → 얇은 조립부 (클래스 인스턴스 생성·연결·루프 오케스트레이션)
- IK 반복 블록 중복(MANUAL POSITION + 피드백) → InverseKinematics::solveIK 단일화
- cfg/yaml gain_mult 디폴트 정합 (cfg 1.5 → 3.0, 코드 fallback 1.5 → 3.0, yaml 3.0 일치)
- launch: blocking stdin 제거로 albc.launch roslaunch 정상 작동 (control_mode param 시작)
- joint_angle_command: Dynamixel 통신 에러처리 보강 (enableTorque/setProfileVelocity/
  readPosition 반환값 체크 + 로그)
- FIXED_ALPHA를 control_law.h 정본으로 (oracle 승격 시 함수내 inline → 헤더 상수)
- package.xml version 3.7.51 → 2.0.0 (의미불명 → 재설계 major)

### Verification
- 특성화 테스트 260 checks (chain1 110 + chain2 albc 150), 0 failures, RESULT PASS
- 각 task 단위: 로컬 run.sh 그린 → agent-jetson catkin_make EXIT 0 (빌드 게이트)
- 9 commits, 매 단위 빌드 게이트 통과

### Notes
- **계약 불변 (chain1↔chain2 보존)**: /albc_status 11필드 순서·단위, 노드명
  albc_controller, param /albc_controller/imu_yaw_offset(45.0), joint command 토픽 2개,
  /hero_agent/sensors 구독. chain1 agent.cpp가 이 param·토픽을 절대경로로 읽으므로 불변.
- **거동 보존 경계**: 제어 수학(TDC/PID/FIXED/MANUAL·DLS IK·IMU 회전·damping gate·
  integral freeze·LPF)은 oracle 정본 승격으로 byte-identical. 타이밍(50Hz dt·IK 반복수·
  spinOnce 위치)은 100% 보존 — 루프 구조 변경 없음.
- **보류 (별도 task)**: 타이밍 개선(AsyncSpinner·IK 수렴판정)은 거동변경이라 향후 실보드
  검증·튜닝과 함께 별도 진행.
