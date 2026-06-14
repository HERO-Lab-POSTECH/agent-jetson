# firmware — agent-jetson UUV 펌웨어 (ATmega2560)

정본: `agent/agent.ino` (보드에 플래시된 바이너리, 2026-06-14 echo로 confirmed).

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
