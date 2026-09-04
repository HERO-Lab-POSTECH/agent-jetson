#!/bin/bash
# agent-jetson 메인 컨트롤러 펌웨어 빌드 (.ino + 모듈 -> .hex)
# ───────────────────────────────────────────────────────────────────────────
# BUILD_AND_FLASH.md §2 의 실제 빌드 명령. 보드(agent-jetson)에서 실행한다.
# Arduino IDE 없이 platform.txt recipe 를 그대로 재현한 수동 빌드 (보드에 MegaCore
# 구동 IDE 가 없으므로). 2026-06-15 복원·검증: gyro 소스 재빌드 hex 가 직전 정상
# flash 본과 크기 0.13% 일치(37716B vs 37764B; byte 차이는 컴파일러 함수배치 순서뿐,
# 동작 불변), teleop hex flash write+verify 통과(36754/36754 verified).
#
# 사용법:  bash build_firmware.sh <소스디렉토리> <출력이름>
#   소스디렉토리: agent.ino + 모듈 .cpp/.h 가 있는 곳 (예: ~/catkin_ws/src/firmware/agent)
#   출력이름:     결과 hex 이름 (예: agent)
#   결과:         ~/fw_full_<출력이름>/<출력이름>.hex
#
# ⚠️ 빌드 결정사항 (복원 과정에서 확정된 함정들):
#   - 최적화 = -Os (platform.txt recipe 정본). -O0 로 빌드하면 hex 가 1.94배 비대해진다.
#   - .ino 는 Arduino 변환을 모방: 맨 앞에 #include <Arduino.h> 를 붙여 .cpp 로 만든다.
#   - ros_lib 는 헤더(-I)뿐 아니라 time.cpp/duration.cpp 도 컴파일해야 한다
#     (안 하면 link 에서 undefined reference to ros::normalizeSecNSec).
#   - core 는 MegaCore(MCUdude_corefiles). 표준 Arduino core 로 링크하면
#     __vector_36 다중정의 충돌(상세 = BUILD_AND_FLASH.md §왜 표준 코어로는 안 되나).
set -euo pipefail   # a failed compile must not leave a stale .hex looking fresh
SRCDIR="${1:?소스디렉토리 필요}"
NAME="${2:?출력이름 필요}"

MEGA=~/.arduino15/packages/MegaCore/hardware/avr/2.0.1
CORE=$MEGA/cores/MCUdude_corefiles
# variant 는 보드 실제 설정과 반드시 일치해야 한다: ~/.arduino15/preferences.txt 의
# custom_pinout=2560_avr_pinout → boards.txt:158 → build.variant=100-pin-avr.
# ⚠️ 100-pin-arduino-mega 로 빌드하면 digital pin 번호→포트 매핑이 어긋난다:
# pin 15(PIN_RELAY) 가 avr=PB0(SS) 인데 arduino-mega=PJ0(USART3_RX) 로 매핑돼
# digitalWrite(15,HIGH) 가 엉뚱한 핀을 켠다. State bit4 는 토글되나 물리 relay 는
# 죽는다(2026-07-03 실기 확정: avr variant 로 재빌드하니 dynamixel PING_OK, relay 작동).
VARIANT=$MEGA/variants/100-pin-avr
ROSLIB=~/Arduino/libraries/ros_lib
SERVO=/usr/share/arduino/libraries/Servo
WIRE=/usr/share/arduino/libraries/Wire
MS5837=~/Arduino/libraries/BlueRobotics_MS5837_Library-master

# platform.txt recipe 정본 플래그 (MegaCore 2.0.1, avr-g++ 4.9.2, IDE 1.8.9=10809)
CPPFLAGS="-c -g -Os -w -std=gnu++11 -fno-exceptions -ffunction-sections -fdata-sections -fno-threadsafe-statics -MMD"
CFLAGS="-c -g -Os -w -std=gnu11 -ffunction-sections -fdata-sections -MMD"
SFLAGS="-c -g -x assembler-with-cpp"
DEFS="-mmcu=atmega2560 -DF_CPU=16000000L -DARDUINO=10809 -DARDUINO_AVR_ATmega2560 -DARDUINO_ARCH_AVR"
INCS="-I$CORE -I$VARIANT -I$ROSLIB -I$SERVO -I$WIRE -I$WIRE/utility -I$MS5837"
ELFFLAGS="-w -Os -Wl,--gc-sections -mmcu=atmega2560"

OUT=~/fw_full_$NAME
rm -rf "$OUT" && mkdir -p "$OUT/obj" "$OUT/src"
LOG=$OUT/build.log; : > "$LOG"

# --- .ino -> agent_main.cpp 변환 (Arduino 빌드 모방: 맨 앞 #include <Arduino.h>) ---
echo "=== [0] 소스 준비 ($SRCDIR) ===" | tee -a "$LOG"
cp "$SRCDIR"/*.cpp "$SRCDIR"/*.h "$OUT/src/" 2>/dev/null
INO=$(ls "$SRCDIR"/*.ino 2>/dev/null | head -1)
if [ -n "$INO" ]; then
  echo "#include <Arduino.h>" > "$OUT/src/agent_main.cpp"
  cat "$INO" >> "$OUT/src/agent_main.cpp"
  echo "  $(basename "$INO") -> agent_main.cpp (Arduino.h 삽입)" | tee -a "$LOG"
fi
BSRC=$OUT/src

echo "=== [1] 펌웨어 모듈 컴파일 ===" | tee -a "$LOG"
FW_OBJS=""
for cpp in "$BSRC"/*.cpp; do
  base=$(basename "$cpp" .cpp)
  avr-g++ $CPPFLAGS $DEFS $INCS "$cpp" -o "$OUT/obj/$base.o" 2>>"$LOG" \
    || { echo "FAIL compile $base"; tail -5 "$LOG"; exit 1; }
  FW_OBJS="$FW_OBJS $OUT/obj/$base.o"
  echo "  compiled $base.o" | tee -a "$LOG"
done

echo "=== [2] 라이브러리 컴파일 (Servo/Wire/MS5837/ros_lib) ===" | tee -a "$LOG"
LIB_OBJS=""
for cpp in $SERVO/Servo.cpp $WIRE/Wire.cpp $MS5837/MS5837.cpp $ROSLIB/time.cpp $ROSLIB/duration.cpp; do
  [ -f "$cpp" ] || { echo "  MISSING $cpp" | tee -a "$LOG"; continue; }
  base=$(basename "$cpp" .cpp)
  avr-g++ $CPPFLAGS $DEFS $INCS "$cpp" -o "$OUT/obj/lib_$base.o" 2>>"$LOG" \
    && { LIB_OBJS="$LIB_OBJS $OUT/obj/lib_$base.o"; echo "  compiled lib_$base.o" | tee -a "$LOG"; }
done
if [ -f "$WIRE/utility/twi.c" ]; then
  avr-gcc $CFLAGS $DEFS $INCS "$WIRE/utility/twi.c" -o "$OUT/obj/lib_twi.o" 2>>"$LOG" \
    && { LIB_OBJS="$LIB_OBJS $OUT/obj/lib_twi.o"; echo "  compiled lib_twi.o" | tee -a "$LOG"; }
fi

echo "=== [3] core.a 빌드 (MegaCore core 전체) ===" | tee -a "$LOG"
CORE_A=$OUT/core.a
for f in $CORE/*.cpp $CORE/*.c $CORE/*.S; do
  [ -f "$f" ] || continue
  base=$(basename "$f"); obj="$OUT/obj/core_$base.o"
  case "$f" in
    *.cpp) avr-g++ $CPPFLAGS $DEFS $INCS "$f" -o "$obj" 2>>"$LOG" ;;
    *.c)   avr-gcc $CFLAGS  $DEFS $INCS "$f" -o "$obj" 2>>"$LOG" ;;
    *.S)   avr-gcc $SFLAGS  $DEFS $INCS "$f" -o "$obj" 2>>"$LOG" ;;
  esac
  [ -f "$obj" ] && avr-ar rcs "$CORE_A" "$obj" 2>>"$LOG"
done
echo "  core.a = $(wc -c < "$CORE_A") bytes" | tee -a "$LOG"

echo "=== [4] 링크 -> .elf ===" | tee -a "$LOG"
avr-gcc $ELFFLAGS -o "$OUT/$NAME.elf" $FW_OBJS $LIB_OBJS "$CORE_A" -lm 2>>"$LOG" \
  || { echo "LINK FAIL"; tail -20 "$LOG"; exit 1; }
echo "  elf ok" | tee -a "$LOG"

echo "=== [5] .elf -> .hex (objcopy) ===" | tee -a "$LOG"
avr-objcopy -O ihex -R .eeprom "$OUT/$NAME.elf" "$OUT/$NAME.hex" 2>>"$LOG"

echo "=== [6] 크기 ===" | tee -a "$LOG"
avr-size "$OUT/$NAME.elf" | tee -a "$LOG"
echo
echo "RESULT_HEX=$OUT/$NAME.hex"
_fwbin="$(mktemp)"
trap 'rm -f "$_fwbin"' EXIT
avr-objcopy -I ihex -O binary "$OUT/$NAME.hex" "$_fwbin"
echo "flash bytes: $(wc -c < "$_fwbin")"
echo "hex md5: $(md5sum "$OUT/$NAME.hex" 2>/dev/null || md5 -q "$OUT/$NAME.hex")"
