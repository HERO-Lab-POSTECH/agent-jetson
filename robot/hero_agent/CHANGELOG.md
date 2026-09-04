# Changelog — hero_agent

## [Unreleased] — refactor/cleanup-2026-09 (병합 대기)

브랜치 `refactor/cleanup-2026-09` (base `90b13e4`, 커밋 12개) 의 hero_agent·hero_msgs·
펌웨어 몫. 버전 번호는 `deploy/72d-inc9998-gru` 로 ff-merge 하는 시점에 확정한다.

### Removed
- **2022 DVL/darknet 유산 (펌웨어)** (`274b192`). 시스템 어디에도 발행자가 없는 구독자
  3종(`/hero_agent/dvl_velocity`·`cont_para`·`cont_xy_darknet`)과 그 콜백, 그것들만 쓰던
  전역 전부 — `Th_0..3`, `darknet_Th_*`, `Kp/Ki/Kd/Mb/KKp/KKv` 위치 게인, `save_acc_*` 링,
  `print_i`. `/hero_agent/result` 의 죽은 publisher 객체도. **2.1.0 Notes 의
  "`Th_*`/`darknet_Th_*` … 물리 삭제는 별도 작업" 이 여기서 닫혔다.**
- **유산 메시지 5종 + srv 1종** — `hero_agent_cont_para`, `hero_agent_cont_xy`,
  `hero_agent_dvl_velocity`, `hero_agent_position_result`, `hero_xy_cont`,
  `hero_command.srv` (`274b192`). **2.0.0 Notes 의 "legacy msg 5종 물리 삭제 보류" 가
  닫혔다.** `hero_agent_vision.msg` 는 유일한 소비자였던 `ros_opencv_manipulation` 과
  같은 커밋에서 삭제 (`654e215`).
- **미참조 vendored 패키지 6종 240파일** (`654e215`, D1): `perception/darknet_ros`(199파일,
  CUDA 툴체인을 끌어온다), `ros_opencv_{manipulation,qr_calibration,save,ipcam_qr}`,
  `drivers/dynamixel_sdk_examples`. launch·CMakeLists·package.xml·소스 어디에서도 자기 트리
  밖 참조가 **0건**임을 실측하고 지웠다 — 보드 `catkin_make` 가 매번 전부 빌드하고 있었다.
  존치분의 출처·버전·존치 사유는 새 `THIRD_PARTY.md`.
  복구: `git checkout 90b13e4 -- perception/ drivers/dynamixel_sdk_examples`.
- `scripts/web_teleop.py` 와 그 테스트 (`654e215`, D5) — launch·alias·CMakeLists PROGRAMS
  어디서도 안 부른다.
- `TeleopController` 의 w/s/a/d(xy) 분기와 누산기, `teleop/xy_step` 파라미터 (`9471176`).
  `keymap.h` 가 w/s/a/d 를 `translated=0` 으로 두고 fw_char 로 직행시키므로 그 분기는
  **도달 불가**였고 누산기는 늘 0.0 이었다. `hero_agent_dvl` 의 `TARGET_X/Y` 는 리터럴
  `0.0` 이 됐는데 **늘 실려 있던 그 값**이다.
- `StateMonitor::sensor_depth`(write-only), `CsvLogger::open` 의 무시되던 인자 (`9471176`).
  ⚠️ 설계 문서가 "미호출 게터 4종" 이라 한 것은 **틀렸다** — depth/attitude 게터 다섯 개는
  `csv_logger.h` 가 전부 호출한다. 삭제하지 않았다.
- `state_monitor.h` 의 인라인 IMU 회전 사본 — `albc_control/imu_rotation.h` 의 `rotateImu()`
  호출로 대체. 삭제된 사본의 식과 문자 단위로 같음을 실측 (`466006c`).
- `package.xml` 의 `xterm` exec_depend — launch 주석이 "no separate xterm" 이라 명시한다
  (`d504f96`).

### Added
- **`hero_msgs/include/hero_msgs/topics.h`** — C++ 토픽 이름 SSOT. `hero_agent/topics.h` 는
  이것을 re-export 하는 얇은 별칭으로 남는다. python 은 `albc_rl.contract.TOPICS`, 펌웨어는
  `firmware/agent/config.h` 의 명명 상수. **펌웨어 wire 이름은 불변 = 펌웨어 ABI,
  재flash 불필요** (`13c5d0d`).
- **펌웨어 호스트 컴파일 테스트** `tests/characterization/test_firmware_dvl.cpp` — shipped
  `dvl_position.cpp` 를 메시지 shim 과 함께 **실제로 컴파일**하고 두 분기를 pin 한다.
  삭제에서 살아남지 못한 extern 은 원래 보드 링크 에러로만 드러났는데 이제 로컬에서 잡힌다
  (작성 중 실제로 타입 불일치 3건을 잡았다) (`274b192`).
- `build_firmware.sh` 가 `set -euo pipefail` 로 fail-fast 하고 hex md5 를 출력한다 —
  결과를 핀으로 박을 수 있다 (`274b192`).
- `package.xml` 에 누락돼 있던 `uvc_camera`·`rosserial_python` exec_depend (`d504f96`).

### Changed (거동 변경 — 엣지 경로 한정, D14)
- `rl_action_to_pwm`: NaN 이 두 범위 비교를 모두 실패하고 임의 count 로 ESC 프레임에
  도달하던 것을 **중립**으로 매핑 (`274b192`, #6).
- `'D'`(depth)가 **상승엣지**에서 `I_depth` 를 0 으로 — yaw 가 이미 갖고 있던 규약. 포화된
  적분이 depth 재개 순간 그대로 다시 걸리고 있었다 (`274b192`, #18).
- **마지막으로 켜져 있던 제어기를 끄면** 그 제어기의 마지막 ESC 프레임이 래치된 채 남았다
  (전송은 제어기가 켜져 있는 동안만 돈다). 이제 두 키 핸들러가 하강엣지에서 NEUTRAL
  1프레임을 흘린다 (`274b192`, #4). 첫 구현의 `!rl_active` 가드는 `312d344` 에서 **제거**
  — `messageThruster()` 가 RL 메시지마다 `cont_yaw_on`·`cont_depth_on` 을 0 으로 강제하므로
  이미 도달 불가였고, 마지막 RL 메시지와 300 ms 워치독 사이 구간에서는 오히려 안전 flush 를
  **지연**시켰다.
- `'R'`(CSV 토글)이 KEYMAP 조회 전에 return 해, 다른 토글이 다 거치는 500 ms 디바운스를
  건너뛰던 것 → `debounce_ok` 경유 (`9471176`).
- HUD 가 루프율(100 Hz)로 전체 재그리기 하던 것을 `log_period`(yaml 기본 0.5 s)로 제한.
  이 파라미터는 정확히 이 용도로 파싱된 뒤 `(void)` 캐스트로 버려지고 있었다 (`9471176`).
- `agent_launch.launch` 의 `$(env ROCON_RTSP_CAMERA_RELAY_URL)` → `$(optenv …)`. 보드
  `.bashrc` 가 export 하고 있어 현장 무증상이었다 (`d504f96`).
- `hero_msgs` package.xml 정합화, 불필요한 `roscpp`/`rospy` 제거 (`d504f96`).

### Verification
- characterization **13종 533 checks** 전부 PASS, `RUN_ALL: PASS`, E2E 골든 파리티 PASSED.
- **미실행: 보드·펌웨어.** `tests/board_gate.sh` 한 줄이 A 클린 재빌드 → B py2.7 `run_all.sh`
  → C 3층 스택 건식 기동 → D 펌웨어 컴파일까지 하도록 스크립트화됐으나, 보드 미접속
  (`192.168.2.100:22` timeout, 2026-09-04 22:07 재확인)으로 한 번도 안 돌았다.
  **펌웨어는 flash 하지 않았다** — 위 D14 거동 변경 3건은 flash 전까지 무효(inert)다.

### Notes
- 🔴 **teleop `r`/`f`(heave) 키는 펌웨어 깊이 목표를 못 움직인다 — 이번 정리 이전부터.**
  `msgCallback_dvl_velocity` 가 `desired_angle_depth = temp_depth + TARGET_Z` 를 하던
  **유일한** 자리였고, 그 콜백이 사라지면서 `TARGET_X/Y/Z` 를 읽는 코드가 0 이 됐다. 즉
  이 열린 회로는 삭제가 만든 게 아니라 삭제가 **드러낸** 것이다. 살아 있는 경로는 펌웨어
  자체 키 `'o'`/`'l'`(`desired_angle_depth ±0.1`). 기존 결함이므로 **고치지 않고**
  `dvl_position.h` 머리말에 명시했다 — 로봇 앞에서 결정할 항목.
- `w/s/a/d` 가 `translated=0` 으로 펌웨어에 직행한다는 사실은 agy 적대 리뷰가 확인해 줬고,
  그래서 위 xy 경로 삭제가 런타임 값을 보존한다는 것이 근거를 갖는다.
- 2.1.0 이 남긴 **self-toggle 리셋 desync** 와 `u/j` throttle 키 정리는 여전히 열려 있다.

---

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
