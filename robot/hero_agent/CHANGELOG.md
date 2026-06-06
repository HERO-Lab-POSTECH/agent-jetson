# Changelog — hero_agent

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
