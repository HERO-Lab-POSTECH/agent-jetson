# 펌웨어 빌드·flash 방식 (agent-jetson ATmega2560)

> 이 펌웨어를 빌드·flash하려면 **반드시 이 문서를 먼저 읽어라.** 보드에 Arduino IDE가 2개 설치돼 있고, 틀린 쪽으로 빌드하면 링커 에러로 막힌다(아래 §함정). 2026-06-14 보드 SSH 실측·ELF 링크 검증으로 확정한 내용이다.

## 한 줄 결론

**빌드·flash = MegaCore 보드패키지 + `board=2560` (ATmega2560), Arduino IDE의 "Verify/Upload".** 표준 Arduino AVR 코어(`board=mega`=ATmega1280)로 빌드하면 `multiple definition of '__vector_36'` 링커 에러로 실패한다 — 이건 펌웨어 결함이 아니라 코어 선택 오류다.

## 정확한 설정 (보드 `~/.arduino15/preferences.txt` 실측값)

| 항목 | 값 |
|:---|:---|
| 보드 패키지 | MegaCore (MCUdude) — `boardsmanager.additional.urls`에 `https://mcudude.github.io/MegaCore/package_MCUdude_MegaCore_index.json` |
| Board | **ATmega2560** (`board=2560`) |
| Clock | 16 MHz external (`custom_clock=2560_16MHz_external`) |
| Pinout | AVR pinout (`custom_pinout=2560_avr_pinout`, variant `100-pin-avr`) |
| BOD | 2.7V (`custom_BOD=2560_2v7`) |
| Sketchbook | `/home/nvidia/Arduino` (ros_lib·MS5837 라이브러리가 여기 있음) |
| Upload 포트 | `/dev/ttyUSB1` (FT232RL = Arduino 표준, 자동리셋 DTR 지원) — ttyUSB0 아님 |
| Upload protocol | `arduino` (STK500, IDE가 DTR로 자동리셋) |

## 왜 표준 코어로는 안 되나 (링커 충돌의 진짜 원인)

펌웨어는 AHRS UART1 수신을 위해 **자체 `ISR(USART1_RX_vect)`를 정의**한다(`agent.ino`). 동시에 `ros::NodeHandle nh`가 rosserial `ArduinoHardware` 생성자를 통해 코어 **`Serial` 객체(USART0)**를 참조한다. 이 `Serial` 참조가 링커로 하여금 코어의 `HardwareSerial*` 오브젝트를 끌어오게 만든다.

- **표준 Arduino 코어**: `Serial`과 모든 USART ISR(`ISR(USART1_RX_vect)` 포함)이 **단일 `HardwareSerial.cpp`** 한 파일에 들어있다. `Serial`을 끌어오면 USART1 ISR도 같이 딸려온다 → 펌웨어 자체 ISR과 **이중정의** → `multiple definition of '__vector_36'`.
- **MegaCore**: USART별로 `HardwareSerial0/1/2/3.cpp`로 **분리**돼 있다. `Serial`(=USART0=`HardwareSerial0.o`)만 끌려오고, USART1 ISR이 든 `HardwareSerial1.o`는 링크에서 **완전 제외**된다(archive-member semantics + `--gc-sections`) → 충돌 없음.

즉 충돌 여부를 가르는 것은 MCU(둘 다 atmega2560 가능)가 아니라 **코어의 HardwareSerial 파일 분리 구조**다.

## 검증 증거 (2026-06-14, 칩 미접촉 ELF 링크 실측)

보드 `/tmp`에서 양쪽 코어로 ELF 링크까지 빌드(flash 없음, 임시 디렉토리, 소스 트리 무변경). MCU는 `-mmcu=atmega2560`로 고정해 코어 구조만 변수로 둠:

| | MegaCore core | 표준 Arduino core |
|:---|:---|:---|
| 링크 결과 | **exit 0, 충돌 0** (275 KB ELF) | **exit 1** |
| 에러 | 없음 | `multiple definition of '__vector_36'` (`HardwareSerial.cpp:506` vs ros_lib `node_handle.h:128`) |
| 최종 바이너리 `__vector_36` | 펌웨어 ISR 하나뿐 (`agent_main.o`) | 충돌로 바이너리 미생성 |

ros_lib 3복사본(`~/Arduino/libraries`, `~/sketchbook/libraries`, `catkin_ws/.../ros_lib`)은 byte-identical(md5 동일)이라 다중복사 충돌은 원인이 아니다(기각됨).

## ⚠️ 함정 — 보드에 IDE가 2개 있다

| 설치 | 정체 | board 설정 |
|:---|:---|:---|
| `~/.arduino15` + MegaCore 패키지 | **메인 컨트롤러 펌웨어용 (이걸 써라)** | `board=2560` (ATmega2560) |
| `~/.arduino` (Arduino IDE 1.0.5, `/usr/bin/arduino`) | **2019 ESC 보드용 stale 설치** (`.vscode`도 MiniCore atmega8 ESC) | `board=mega` (ATmega**1280**) ← 틀림 |

`~/.arduino`(1.0.5)로 빌드하려 하면 위 링커 충돌에 막힌다. firmware 디렉토리에 한때 있던 `~/fw_gate.sh`는 **구조화 검증용 헬퍼**(`avr-g++ -c` 오브젝트 컴파일만, 링크·flash 없음)지 빌드/flash 도구가 아니다.

## MCU = ATmega2560 (확정)

코어 `ISR(USART1_RX_vect)`는 2560의 vector 36에 해당하고, MegaCore `board=2560`·이 README 헤더와 일치한다. `~/.arduino`의 `board=mega`(1280)는 stale 오설정이다. avrdude CLI 수동 호출은 자동리셋(DTR)이 안 걸려 sync 실패하므로, flash는 IDE Upload(DTR 자동리셋) 경로를 쓴다.

## flash 절차 (비가역 — 신중히)

펌웨어 flash는 UUV 유일 컨트롤러를 덮어쓰는 비가역 작업이다. 순서:

- **사전**: ROS가 ttyUSB1을 점유하면 flash 불가 → `pkill -f rosserial; pkill -f roslaunch`로 포트 해제.
- **롤백 자산**: 보드 `~/chip_known_good_2024_agent_tdc_20260614.ino` (md5 `751df1cc`, 칩에 깔린 직전 펌웨어 소스). read-back .hex는 avrdude CLI sync 실패로 못 뜨므로 이 소스 재flash가 유일 롤백 경로다.
- **빌드/Upload**: 위 §설정대로 MegaCore `board=2560`·16 MHz external·port ttyUSB1로 IDE Verify → Upload.
- **flash 후 검증**: ① `rostopic echo /hero_agent/sensors`가 발행되는지(checksum 일치 = 통신 두절 해소) ② GYRO 필드에 자이로 진값이 흐르는지(축·부호·단위 rad/s vs deg/s) ③ 보드 `catkin_make`(dynamic_reconfigure cfg 헤더 생성).
