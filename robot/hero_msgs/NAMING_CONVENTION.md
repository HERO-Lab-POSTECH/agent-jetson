# hero_msgs / albc 토픽·메시지 명명 규칙 (Naming Convention)

> ROS 기업표준 기반 단일 진실 공급원(SSOT). 출처: REP-103(grammar), REP-135(namespace/relative
> name practices), REP-145(IMU/standard messages) + Autoware.Auto naming guidelines,
> leggedrobotics/ros_best_practices. 작성 2026-06-14.
> **이 문서가 토픽·메시지 명명의 단일 진실이다.** 신규 토픽·필드는 여기 규칙을 따른다.

## 적용 상태 범례

- **✅ APPLIED** : 코드에 적용 완료 (ROS쪽, flash 불필요)
- **📋 RULE-ONLY** : 규칙만 정의, 적용은 펌웨어 현대화(요구사항 1, flash 단계)와 묶어 보류

> **왜 일부는 RULE-ONLY인가**: ATmega2560 펌웨어 flash = UUV 유일 컨트롤러 brick 위험.
> 메시지 *필드명* 변경은 펌웨어 `.ino`가 그 필드를 쓰므로 re-flash를 유발한다. 따라서
> 이번 정리는 ROS쪽(.msg 삭제·C++/Py 토픽 문자열·문서)만 적용하고, 필드 rename·타입
> 변경·펌웨어 토픽은 규칙으로 박제만 한다.

---

## 원칙 1 — 토픽 계층: `/<robot>/<subsystem>/<signal>`, producer 기준 그룹화

토픽은 *소비하는 노드*가 아니라 *생산하는 서브시스템의 기능*으로 묶는다 (Autoware.Auto).
멀티로봇·재사용을 위해 robot 이름은 launch에서 주입(`ns=`)하는 것이 표준.

| 서브시스템 | 토픽 | 상태 |
|:---|:---|:---|
| 펌웨어 센서/상태 | `/hero_agent/sensors`, `/hero_agent/state`, `/hero_agent/dvl`, `/hero_agent/command`, `/hero_agent/key_input` | 📋 RULE-ONLY (펌웨어 계약, flash) |
| RL 입출력 | `/albc/joint_states`, `/albc/thruster_cmd` | ✅ (이미 모범) |
| 제어기 상태 | `/albc/status` (구 `/albc_status`) | ✅ APPLIED |
| RL 명령 | `/albc/rl_command` | ✅ APPLIED |
| 관절 전류 | `/albc/joint_currents` | ✅ APPLIED |

**근거**: Autoware.Auto Naming Guidelines — "Topics should be namespaced based on the
function of the node which produces them and not the node(s) which consume them."

---

## 원칙 2 — 글로벌 prefix 최소화, 절대경로 일관성 (REP-135)

ROS 표준은 "노드 코드엔 상대명을 쓰고 연결은 launch `<remap>`으로 노출"이다. 글로벌 `/`
prefix는 멀티로봇·중복 인스턴스를 깨므로 최소화한다. 단 이 코드베이스는 단일 로봇·단일
네임스페이스이므로 현실적 규칙은 **"같은 토픽은 pub·sub가 동일한 절대경로"**로 충분하다.

**금지: 상대 publish vs 절대 subscribe 불일치** (네임스페이스에 따라 연결이 끊김).

- ✅ `/albc/joint_currents` : publisher(`joint_angle_command.cpp`)·subscriber(`albc_controller.cpp`)
  모두 절대경로 `/albc/joint_currents` 통일.

**근거**: REP-135 Driver Namespace Practices; leggedrobotics — "Global names should be
avoided as much as possible."

---

## 원칙 3 — 토픽 상수 SSOT (raw 리터럴 흩뿌림 금지)

모든 토픽명은 `topics.h` 류 상수 헤더에 정의하고, 코드 곳곳에 raw 문자열을 흩뿌리지
않는다. raw 리터럴 중복은 오타 → 한쪽만 바뀜 → 토픽 체인 단절을 유발한다.

- 기존 SSOT: `hero_agent/include/hero_agent/topics.h` (`hero::topics::*` 네임스페이스)
- ✅ `ALBC_STATUS` 상수가 `/albc/status`를 정의하고, publisher가 이 상수를 참조
- 📋 albc_control·albc_rl의 나머지 raw 리터럴(`/albc/joint_states`, `/albc/thruster_cmd`,
  `/albc/rl_command` 등)은 점진적으로 이 SSOT 또는 패키지별 상수로 이관

---

## 원칙 4 — 메시지 필드명: `lower_snake_case` (REP-103 grammar) 📋 RULE-ONLY

모든 메시지 필드는 `lower_snake_case`. 현재 **모든 메시지가 위반**(ALL_CAPS 또는
Mixed_Case). 적용은 flash 단계(.msg 필드 변경이 펌웨어 `.ino` lock-step을 유발).

옳은 형태(목표):

| 메시지 | 현재 (위반) | 표준형 |
|:---|:---|:---|
| hero_agent_sensor | ROLL, PITCH, YAW, DEPTH, GYRO_X/Y/Z | roll, pitch, yaw, depth, gyro_x/y/z |
| hero_agent_state | Yaw, Target_yaw, Throttle, Valid_yaw, Depth, Target_depth, Move_speed, Cont_state, State_addit | yaw, target_yaw, throttle, valid_yaw, depth, target_depth, move_speed, cont_state, state_addit |
| hero_agent_dvl | TARGET_X/Y/Z, command | target_x/y/z, command |
| hero_agent_dvl_velocity | VX, VY, TIME, VALID | vx, vy, time, valid |
| hero_agent_vision | WHITE_VALID, BLACK_VALID, OBJECT_VALID, LASER_VALID, HIGH_LASER, LOW_LASER, FOR_YAW | white_valid, black_valid, object_valid, laser_valid, high_laser, low_laser, for_yaw |
| hero_agent_cont_xy | T0~T3, TARGET_DEPTH | t0~t3, target_depth |
| hero_agent_cont_para | control_T, Kp, Ki, Kd, Mb, KKp, KKv | control_t, kp, ki, kd, mb, kkp, kkv |
| hero_xy_cont | TARGET_X/Y, VALID | target_x/y, valid |
| hero_agent_position_result | TARGET_X/Y/Z, X, Y, Z | target_x/y/z, x, y, z |

**근거**: ROS Names spec / ROS 2 interface spec — field names "must be lowercase
alphanumeric characters with underscores."

---

## 원칙 5 — 표준 메시지 재사용 + 타입 일관성 (REP-145)

custom 메시지 남발 금지. 의미가 맞으면 표준 메시지를 쓴다.

- IMU(자세+각속도) → `sensor_msgs/Imu`, 속도 명령 → `geometry_msgs/Twist` 권장
  📋 RULE-ONLY (현재 custom hero_agent_sensor + 별도 토픽, 표준화는 펌웨어 단계)
- boolean 플래그는 `bool` 타입. 현재 `char VALID`('y'와 비교, hero_agent_dvl_velocity)와
  `int8 VALID`(hero_xy_cont) 혼재 → `bool`로 통일 📋 RULE-ONLY (.msg 타입 변경 = flash)
- ROS엔 공식 메시지 deprecation 기구가 없다(genmsg#67 미해결). de-facto는
  `.msg` 상단 주석 + 런타임 `ROS_WARN_ONCE`.

**근거**: REP-145; leggedrobotics — "Use standard data types whenever possible (try to
prevent .msg proliferation)."

---

## teleop 키맵 SSOT (key_input 와이어 프로토콜)

`keymap.h`의 `KEYMAP[]` 테이블이 **user_key → fw_char 매핑의 단일 진실**이다.
3곳이 fw_char 와이어 프로토콜을 공유하며 반드시 lock-step으로 유지해야 한다:

| SSOT 위치 | 역할 |
|:---|:---|
| `robot/hero_agent/include/hero_agent/keymap.h` `KEYMAP[]` | user_key → fw_char 매핑 정의 (SSOT) |
| `robot/hero_agent/scripts/key_teleop.py` KEY_TABLE | HELP 화면 광고 (KEYMAP[]에서 자동생성) |
| `firmware/agent/agent.ino` `messageCommand` | fw_char → 펌웨어 동작 분기 |

**경계(boundary)**: fw_char 문자 하나. 이 문자가 바뀌면 3곳 모두 동시에 바꿔야 한다.

### 확정 키 레이아웃 (2026-06-15 기준)

| 그룹 | 사용자 키 | 동작 | fw_char | 종류 |
|:---|:---|:---|:---|:---|
| 시스템 | `1` | Relay | `R` | self-toggle |
| | `2` | PWM Neutral (ESC init) | `P` | one-shot |
| | `N` | Yaw reset | `Z` | one-shot |
| | `R`(shift) | CSV 로깅 | (agent 내부, KEYMAP 없음) | toggle |
| 제어 | `3` | Yaw control | `Y` | self-toggle |
| | `4` | Depth control | `D` | self-toggle |
| | `5` | Laser | `L` | self-toggle |
| Jog | `w`/`s`/`a`/`d` | 전진/후진/좌/우 | `w`/`s`/`a`/`d` | press |
| | `q` | STOP | `q` | press |
| Speed | `y`/`h` | move_speed ±10 | `+`/`-` | press |
| Throttle | `u`/`j` | throttle ±10 | `u`/`j` | press |
| Setpoint | `i`/`k` | desired_yaw ±0.1 | `i`/`k` | press |
| | `o`/`l` | desired_depth ±0.1 | `o`/`l` | press |
| Gripper | `c`/`v`/`b` | open/stop/close | `c`/`v`/`b` | press |
| Heave | `r`/`f` | z축 ± (Jetson teleop) | (fw 미전달) | press |
| (제거됨) | — | concon, dvl cont | — | 삭제 |

### 정책

- **allow-list drop**: `KEYMAP[]`에 없는 키는 `key_translator`에서 버려진다. 미등록 키가 fw에 도달하는 패스스루는 없다.
- **self-toggle**: R/Y/D/L은 fw_char 단일 문자; 펌웨어가 `state=!state`로 처리. 별도 on/off char 없음.
- **SSOT 동기화 의무**: `KEYMAP[]` 변경 시 `messageCommand`(firmware)와 KEY_TABLE(key_teleop.py)을 lock-step으로 갱신한다.

> **self-toggle 리셋 desync 주의 (필드 운용)**: 펌웨어 리셋 후 `state_Relay`/`state_Laser`는 0으로 재초기화되나 물리 relay가 ON이면 첫 토글 1회가 resync에 소비된다. 설계 상 의도된 트레이드오프.

---

## 메시지 생애주기 분류

### ACTIVE (6) — live 그래프에 publisher·subscriber 양끝 존재

- `hero_agent_sensor` — 펌웨어 → albc_controller / rl_inference_node (GYRO_X/Y/Z append됨)
- `hero_agent_state` — 펌웨어 → agent.cpp StateMonitor
- `hero_agent_dvl` — agent.cpp → 펌웨어
- `hero_xy_cont` — perception
- `hero_agent_vision` — perception
- `hero_command.srv` — perception manipulation

### DEPRECATED (4) — 펌웨어 stub은 있으나 ROS 그래프 짝 없음 (QR/DVL 미션 dormant)

- `hero_agent_cont_xy`, `hero_agent_cont_para`, `hero_agent_dvl_velocity`,
  `hero_agent_position_result`
- `.msg` 상단에 DEPRECATED 주석 표기. 완전 제거는 펌웨어 subscriber stub 제거(flash,
  요구사항 1)와 함께. QR/DVL 미션 부활 시 재활성 여지를 위해 지금은 보존.

### DELETE (1) — 완전 dead, 활성 참조 0

- `hero_usbl_cont` — 어디서도 활성 사용 안 됨(주석·CMakeLists·README 외 0) → ✅ 삭제됨

---

## 보류 사유 요약 (flash 회피)

ATmega2560 펌웨어 flash = UUV 유일 컨트롤러 brick 위험 → 이번 명명 정리는 ROS쪽만 적용.
**필드 rename(원칙 4)·bool 타입 통일(원칙 5)·`/hero_agent/*` 네임스페이스(원칙 1)·dormant
stub 제거**는 요구사항 1(펌웨어 구조화, re-flash 동반)에서 lock-step으로 처리한다.
