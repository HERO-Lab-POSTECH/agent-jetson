# 펌웨어 빌드·flash 방식 (agent-jetson ATmega2560)

> 이 펌웨어를 빌드·flash하려면 **반드시 이 문서를 먼저 읽어라.** 보드에 Arduino IDE가 2개 설치돼 있고, 틀린 쪽으로 빌드하면 링커 에러로 막힌다(아래 §함정). 2026-06-14 보드 SSH 실측으로 확정: 빌드는 ELF 링크 검증(MegaCore exit 0 / 표준코어 exit 1), flash 경로는 avrdude 시그니처 read sync 성공(`0x1e9801`). 둘 다 칩 미접촉 read-only로 검증.

## 한 줄 결론

- **빌드 = MegaCore 코어(`board=2560`, ATmega2560).** 표준 Arduino AVR 코어(`board=mega`=ATmega1280)로 빌드하면 `multiple definition of '__vector_36'` 링커 에러로 실패한다 — 펌웨어 결함이 아니라 코어 선택 오류다.
- **flash = avrdude CLI 직접** (`-c arduino -b 115200 -P /dev/ttyUSB1`). **보드엔 MegaCore를 구동할 IDE가 없다**(IDE 1.0.5만 있고 그건 표준코어=충돌). avrdude 6.2 + MegaCore avrdude.conf는 보드에 있으므로 IDE 없이 보드에서 flash가 완결된다. 2026-06-14 시그니처 read로 이 protocol/speed가 칩과 sync됨을 실측 확인(`0x1e9801`).

## 정확한 설정 (보드 `~/.arduino15/preferences.txt` 실측값)

| 항목 | 값 |
|:---|:---|
| 보드 패키지 | MegaCore (MCUdude) — `boardsmanager.additional.urls`에 `https://mcudude.github.io/MegaCore/package_MCUdude_MegaCore_index.json` |
| Board | **ATmega2560** (`board=2560`) |
| Clock | 16 MHz external (`custom_clock=2560_16MHz_external`) |
| Pinout | AVR pinout (`custom_pinout=2560_avr_pinout`, variant `100-pin-avr`) |
| BOD | 2.7V (`custom_BOD=2560_2v7`) |
| Sketchbook | `/home/nvidia/Arduino` (ros_lib·MS5837 라이브러리가 여기 있음) |
| Upload 포트 | `/dev/ttyUSB1` (FT232R, ID `0403:6001`, DTR 자동리셋 지원) — ttyUSB0 아님 |
| Upload protocol | **`arduino`** (STK500v1) — ⚠️ 표준 Mega2560의 `wiring`(STK500v2) 아님. 칩에 MegaCore optiboot_flash 부트로더가 깔려 있어서다 |
| Upload speed | **115200** (`2560.menu.clock.16MHz_external.upload.speed`) |

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

## MCU = ATmega2560 (칩이 직접 확인 — 2026-06-14)

avrdude 시그니처 read로 칩이 직접 답함: **`Device signature = 0x1e9801` (= atmega2560)**. 1280이면 `0x1e9703`이다. MegaCore `board=2560`·README 헤더·코어 `ISR(USART1_RX_vect)`(2560 vector 36)와 100% 일치. `~/.arduino`의 `board=mega`(1280)는 stale 오설정이다.

## flash 절차 (비가역 — 신중히)

펌웨어 flash는 UUV 유일 컨트롤러를 덮어쓰는 비가역 작업이다. 아래는 보드에서 IDE 없이 avrdude CLI로 완결하는 절차다. `CONF=~/.arduino15/packages/MegaCore/hardware/avr/2.0.1/avrdude.conf` 로 두고 진행한다.

**0. 사전 — 포트 해제**: ROS rosserial이 ttyUSB1을 점유하면 flash 불가 → `pkill -f rosserial; pkill -f roslaunch` 후 `fuser /dev/ttyUSB1`로 free 확인.

**1. 롤백 백업 (필수, read-only)**: 이제 sync가 되므로 현재 칩 펌웨어를 .hex로 떠둔다 — 소스 재flash보다 정확한 바이너리 복원이다.

    avrdude -C "$CONF" -p atmega2560 -c arduino -P /dev/ttyUSB1 -b 115200 -U flash:r:chip_backup_$(date +%Y%m%d).hex:i

추가 소스 롤백 자산: 보드 `~/chip_known_good_2024_agent_tdc_20260614.ino` (md5 `751df1cc`, 직전 펌웨어 소스).

**2. .hex 빌드**: 보드엔 MegaCore 구동 IDE가 없으므로, `platform.txt` recipe를 그대로 재현한 수동 빌드 스크립트 `firmware/build_firmware.sh`로 `.ino`+모듈을 `.hex`까지 빌드한다(이 한 줄이 5세션 미문서화였던 ".ino→.hex" 절차 — 2026-06-15 복원·검증).

    bash ~/catkin_ws/src/firmware/build_firmware.sh ~/catkin_ws/src/firmware/agent agent
    # 결과: ~/fw_full_agent/agent.hex

스크립트가 자동으로 하는 일(직접 빌드 시 똑같이 해야 함): `.ino` 맨 앞에 `#include <Arduino.h>` 삽입(Arduino 변환 모방) → 모듈·라이브러리·MegaCore core 컴파일 → 링크 → objcopy. **빌드 결정사항(복원 과정에서 확정한 함정)**:

| 항목 | 값 / 주의 |
|:---|:---|
| 툴체인 | `/usr/bin/avr-g++` = **GCC 4.9.2** (시스템 설치. MegaCore 패키지엔 avr-gcc 번들 없음) |
| 최적화 | **`-Os`** (recipe 정본). `-O0`로 빌드하면 hex가 **1.94배** 비대해진다(73KB vs 37KB) — 함정 |
| 컴파일 플래그 | `-c -g -Os -w -std=gnu++11 -fno-exceptions -ffunction-sections -fdata-sections -fno-threadsafe-statics` |
| defines | `-mmcu=atmega2560 -DF_CPU=16000000L -DARDUINO=10809 -DARDUINO_AVR_ATmega2560 -DARDUINO_ARCH_AVR` (IDE 1.8.9=10809) |
| include (-I) | core·variant(`100-pin-avr` — 17행 Pinout과 일치, 보드 실제 설정)·`~/Arduino/libraries/ros_lib`·Servo·Wire·Wire/utility·MS5837 (7개) |
| ros_lib | 헤더(-I)뿐 아니라 **`time.cpp`·`duration.cpp`도 컴파일**해야 함 — 안 하면 link에서 `undefined reference to ros::normalizeSecNSec` |
| 라이브러리 | `Servo.cpp`·`Wire.cpp`·`Wire/utility/twi.c`·`MS5837.cpp` |
| core.a | MegaCore `MCUdude_corefiles` 전체 → `avr-ar rcs` (표준 core면 `__vector_36` 충돌, §위 참조) |
| 링크 | `avr-gcc -w -Os -Wl,--gc-sections -mmcu=atmega2560 -o .elf {objs} core.a -lm` |
| objcopy | `avr-objcopy -O ihex -R .eeprom agent.elf agent.hex` (`avr-objcopy`는 `/usr/bin/` 또는 `/usr/share/arduino/hardware/tools/avr/bin/`) |

**빌드 검증**(byte-identical은 컴파일러 함수배치 비결정성으로 불가 — 기능적 동등으로 판정): 직전 정상 flash본과 hex *크기*가 ±수십 바이트(0.13%) 이내인지, `avr-objdump -d`로 새 로직이 기계어에 있는지 확인. 추측 hex는 flash 금지.

**3. flash (비가역)**:

    avrdude -C "$CONF" -p atmega2560 -c arduino -P /dev/ttyUSB1 -b 115200 -D -U flash:w:agent.hex:i

(`-D`=칩 자동 erase 생략; optiboot가 페이지 단위로 처리. 부트로더 보존.)

**4. flash 후 검증**: ① `rostopic echo /hero_agent/sensors`가 발행되는지(checksum 일치 = 통신 두절 해소) ② GYRO 필드에 자이로 진값이 흐르는지(축·부호·단위 rad/s vs deg/s) ③ 보드 `catkin_make`(dynamic_reconfigure cfg 헤더 생성).

### avrdude sync 실측 (2026-06-14, RESUME가 못 푼 sync 실패 종결)
정답 조합 `-c arduino -b 115200`로 시그니처 read 성공(`0x1e9801`, exit 0). RESUME 2차 세션의 sync 실패(4종 전부 `not in sync`)는 **틀린 프로토콜**이 원인이었다 — 시도한 `wiring`(STK500v2)·`arduino/57600`은 칩의 optiboot_flash(STK500v1/115200)와 불일치. `arduino/115200`은 안 시도했었다. DTR 자동리셋도 정상(FT232R) — 자동리셋 문제가 아니라 프로토콜 미스매치였다.
