# Changelog — hero_agent

## [2.1.0] — 2026-06-15 — key teleop 기업표준 재설계 (allow-list + self-toggle)

`keymap.h` SSOT allow-list + self-toggle + 미등록 키 drop으로 teleop 안전성·명시성 개선.
fw_char 와이어 프로토콜 재배치 + 펌웨어 dead 핸들러 제거. (보드 빌드·flash·실기 검증은 Task 7.)

### Removed
- **패스스루 안전구멍**: 미등록 키가 raw fw_char로 펌웨어에 도달하던 경로 제거 (allow-list drop으로 대체)
- **concon 핸들러**: `cont_direc=8` 분기 + 소비 분기 (firmware + key_translator)
- **dvl cont 핸들러**: `cont_direc=7` 분기 + 소비 분기 (firmware + key_translator)
- **split-toggle char**: relay·yaw·depth·laser 각 on/off 분리 char (e/t·r/f·y/h·p/;) → self-toggle 단일 char으로 통합
- **dead char**: `print_i` 디버그('2'/'3'), yaw micro-adjust('6'/'5'/'8'/'7'), 구 PWM 'g', 구 yaw reset 'n', 구 speed 'z'/'x'
- **HELP 허상 40키**: 광고는 됐으나 실제 keymap에 없던 미등록 키 전체

### Added
- **allow-list SSOT** (`keymap.h` `KEYMAP[]`): 미등록 키는 `key_translator`에서 drop, fw로 전달되지 않음
- **self-toggle** (R/Y/D/L): relay·yaw·depth·laser 각 단일 fw_char, 펌웨어가 `state=!state`로 처리
- **HELP 자동생성**: `KEYMAP[]` 테이블에서 직접 생성 → 광고=실제 일치 보장
- **characterization 회귀 가드**: keymap 57 checks + translation 16 checks (거동 박제 골든 테스트)

### Changed
- **speed 사용자키**: `z`/`x` → `y`/`h` (fw_char는 `+`/`-`)
- **toggle fw_char 재배치**: relay→`R`, yaw→`Y`, depth→`D`, laser→`L` (대문자 self-toggle 통일)
- **PWM Neutral fw_char**: `g` → `P`
- **yaw reset fw_char**: `n` → `Z`
- **debounce 게이트**: `keymap.h`의 `debounce` 플래그 기준으로 통일

### Verification
- characterization 0 failures 확인됨 (keymap 57 checks + translation 16 checks)
- 보드 catkin_make 컴파일 게이트 · 펌웨어 avr-g++ -c 게이트 · flash write+verify · 실기 self-toggle 검증은 **Task 7에서 수행 예정** (미완료)

### Notes
- 비가역 flash 1회 필요: R/Y/D/L self-toggle 단일 char + dead char 제거가 모두 펌웨어 변경을 유발. teleop·자이로 변경을 단일 flash에 묶어 처리(Task 7).
- **self-toggle 리셋 desync**: 펌웨어 리셋 후 `state_Relay`/`state_Laser`가 0으로 재초기화되나 물리 relay는 ON 상태일 수 있음. 첫 토글 1회가 resync에 소비될 수 있음 — 설계 상 의도된 트레이드오프(사용자 명시 선택).
- **'R'(shift) = CSV 로깅**: `csv_logger.toggle()` 호출 (record_flag 토글). rosbag 아님, agent 내부 처리, `KEYMAP[]`에 없음.
- `Th_*/darknet_Th_*` 전역: dvl writer 제거 후 write-only로 남음 (물리 삭제는 별도 작업).

---

## [2.0.0] — 2026-06-06 — 단일 노드 재설계

chain 1(hero_agent) 구조적 재설계. agent_main + agent_command → 단일 `agent` 노드.

### Removed
- lawnmower 측량 기능 전체 (lawnmower_survey.cpp, LawnmowerState, p/o 키, count%10 FSM)
- `/hero_agent/key_translated` 토픽 왕복과 teleop fallback 경로
- 죽은 백업 agent_main_fsm_backup.cpp (FSM 골격은 설계노트로 발췌)
- sensor_msgs 의존 (백업 전용이었음)
- ControlFlags/ctrl 전역 (lawnmower 단일멤버라 함께 소멸)

### Added
- topics.h — 토픽명 SSOT 상수
- keymap.h — 선언적 키맵 테이블(KEYMAP[])
- key_translator.h — V3 키 번역 정본(ROS-free 순수함수)
- teleop_controller.h — teleop target 누적기 클래스
- state_monitor.h / csv_logger.h / rosbag_recorder.h — 책임 분리 클래스
- tests/characterization/ — 거동 박제 골든 테스트(재설계 안전망)
- config 전체 param화(agent.yaml): teleop step, loop_rate, csv_rate, debounce, results_dir

### Changed
- 3 ROS 노드(key_teleop.py + agent_main + agent_command) → key_teleop.py + 단일 agent
- 키 번역 switch → keymap 테이블 룩업
- God-object 전역 13개 → 클래스 멤버 흡수

### Verification
- 특성화 테스트(KeyTranslator/keymap/teleop) byte-identical 그린
- agent-jetson catkin_make 각 단위 통과

### Notes / 보류 (펌웨어 사양 확인 후)
- teleop target clamp 미적용 (dvl 소비자=Arduino 펌웨어, target 단위·범위 .ino 부재로 미확인)
- legacy msg 5종 물리 삭제 보류 (firmware rosserial 의존 가능성)
