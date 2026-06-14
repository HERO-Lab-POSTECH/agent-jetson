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

## 외부 라이브러리
- `libraries/BlueRobotics_MS5837/` — 압력센서 (verbatim vendor)
- ros_lib — regen_ros_lib.sh로 생성 (커밋 안 함)

## ⚠️ GYRO 필드
hero_agent_sensor.msg에 GYRO_X/Y/Z가 있으나 이 펌웨어는 아직 publish하지 않는다.
실제 publish(.ino depth_count==3 블록 3줄 추가)는 별도 조각(atomic imu+gyro flash, 비가역)에서.

## non-flash 원칙
이 repo의 .ino 변경은 보드 flash와 분리. flash는 명시적·별도 작업.
