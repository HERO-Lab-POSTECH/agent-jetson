# firmware — agent-jetson UUV 펌웨어 (ATmega2560)

정본: `agent/agent.ino` (보드에 플래시된 바이너리, 2026-06-14 echo로 confirmed).

## 코드 구조 (2026-06-14 구조화)

996줄 단일 `2024_agent_tdc.ino`를 behavior-identical하게 모듈로 분리했다. 산술식·실행순서·핀번호·레지스터값을 글자 단위 보존하고 모듈 경계만 도입(순수 cut-paste). 공유 volatile 전역의 **정의**는 `agent.ino`가 소유하고, 각 모듈 헤더는 `extern` 선언만 둔다.

| 파일 | 책임 |
|:---|:---|
| `agent.ino` | 공유 전역 정의, ISR(USART1_RX_vect), ROS pub/sub 객체, setup()/loop(), messageCommand 디스패처 |
| `config.h` | 매직넘버 named `static const`(핀·ESC/depth PWM·gripper·yaw unwrap·MIP 프로토콜), 값은 원본 리터럴 1:1 보존 |
| `ahrs.{h,cpp}` | MIP 패킷 파싱(ahrs_parse_packet), UART1_write, AHRS init |
| `thrusters.{h,cpp}` | esc_input, UART2_write |
| `pid.{h,cpp}` | PID_control_yaw / PID_control_depth (cont_direc 믹싱 포함) |
| `dvl_position.{h,cpp}` | DVL 콜백 4개(XY 적분·control_T 0/1/3/4 제어) |
| `io.{h,cpp}` | relay/laser GPIO·gripper Timer1 PWM 제어 |

### 의도된 거동 변경 (1건)
- **relay-ON의 `delay(5000)` 제거** (`messageCommand` 'e' 핸들러). 원본은 relay HIGH 후 후속 동작 없이 5초간 `nh.spinOnce()`/PID 루프 전체를 freeze했다. 이 블로킹을 제거(비블로킹화)했다. **이외 모든 거동은 동일** — 산술식 byte-identical(avr-g++ 함수단위 md5 검증), 차이는 주석처리 dead 코드 삭제와 모듈 함수 래핑뿐.

### TODO (별도 flash 조각)
- PID dt(`T`/`T_depth`)의 동적화: 현재는 고정 명목값. 측정 loop_speed 기반 dt로 바꾸면 D/I 게인 재튜닝 필수(거동 변경) → atomic IMU+gyro flash 조각에서 opt-in.

### 검증 = 컴파일 게이트 (flash 아님)
- 보드 avr-g++ 4.9.2(C++98)로 6 TU(`agent_main`+5모듈 .cpp; config.h는 header-only) `-c` 컴파일 EXIT 0.
- 프로토타입 주입 없이 헤더 include만으로 컴파일 통과(구조화 완결 증명).
- baseline(구조화 전 원본) 대비 산술 라인 전수 일치(상수치환 정규화 후).
- **이 트리는 컴파일 검증만 — 보드 flash는 atomic IMU+gyro 조각(비가역)에서.**

## 빌드체인
hero_msgs (.msg) → make_libraries.py → ros_lib → arduino 빌드

ros_lib는 generated라 git에 vendor하지 않는다. 메시지 스키마 변경 시 재생성:

    bash regen_ros_lib.sh

### ⚠️ 빌드·flash 방식은 [`BUILD_AND_FLASH.md`](BUILD_AND_FLASH.md) 필독
보드에 Arduino IDE가 2개 설치돼 있고, **MegaCore `board=2560`(ATmega2560)으로만** 빌드된다. 표준 코어(`board=mega`=1280)는 `multiple definition of '__vector_36'` 링커 에러로 실패한다(펌웨어 결함 아니라 코어 선택 오류, 2026-06-14 ELF 링크 실측 확정). 정확한 IDE 설정·링커 충돌 원인·flash 절차·롤백 자산은 전부 그 문서에 있다.

## 외부 라이브러리
- `libraries/BlueRobotics_MS5837/` — 압력센서 (verbatim vendor)
- ros_lib — regen_ros_lib.sh로 생성 (커밋 안 함)

## GYRO 필드 (2026-06-14 publish 활성화)

hero_agent_sensor.msg의 GYRO_X/Y/Z를 펌웨어가 publish한다 — `acc_roll/acc_pitch/acc_yaw`(MIP 파싱된 자이로 진값 p,q,r, raw sensor frame). receive-side(`albc_rl/scripts/build_proprio.py`의 `rotate_gyro`)가 sensor→base frame 변환(rotate_imu와 동일 z회전; r=yaw rate는 z축 불변이라 진값 통과). 정책 obs[6:9]를 euler 미분 대신 진값으로 채워 sim↔실기 angular velocity frame 정합(2차 실기동 근본결함 해결).

- **offset 실시간 튜닝**: `albc_rl`의 dynamic_reconfigure `imu_yaw_offset_deg`(default 45). euler·gyro 변환이 공유. ⚠️ launch의 `imu_yaw_offset_deg` rosparam보다 dynamic_reconfigure가 이긴다(Server가 시작 시 cfg default로 덮음) — 다른 default를 쓰려면 cfg default도 같이 바꿔야.
- **flash 필요**: GYRO publish는 .ino 변경이라 보드 flash 전엔 효과 없다. flash 전 보드는 GYRO=0.0 → RL이 euler 미분 fallback 자동 사용(`_on_sensor`의 all-zero=no-gyro 판정).
- **use_board_rates 후순위**: 옛 `/albc_status` euler-rate 경로(`use_board_rates=true`)는 gyro 진값을 덮어쓴다 — launch 두 곳 다 `default="false"`. gyro flash 후엔 false 유지(켜면 진값 무효화 경고 로그).

## non-flash 원칙
이 repo의 .ino 변경은 보드 flash와 분리. flash는 명시적·별도 작업.
