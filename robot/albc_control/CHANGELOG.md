# Changelog — albc_control

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
