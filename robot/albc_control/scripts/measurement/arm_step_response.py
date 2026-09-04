#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
arm_step_response.py — ALBC UUV 팔 관절 계단응답 계측 (측정1, 주력)

이 스크립트는 **배포 드라이버(joint_angle_command.cpp)와 별개인 독립 계측 도구**다.
현 배포 노드는 (a) PresentVelocity(addr128)를 안 읽고, (b) ProfileVelocity를
0으로 못 두며(첫 command 후 5초 20->100), (c) 10Hz current-only 루프라
계획서의 "즉시 step + >=200Hz Pos/Vel/Current 로깅"을 만족 못한다.
그래서 배포 노드를 완전히 종료한 뒤 이 스크립트가 bus를 단독 점유해 측정한다.
(SSOT: notes/2026-07-02-onboard-measurement-audit.md B1/B2/B3/M4)

동작:
  1) relay ON + ping으로 관절 ID 11/12 응답 확인 (무응답이면 측정 거부).
  2) 측정 시점 레지스터 실측값(P/I/D/ProfileVel/Mode) read 후 기록.
  3) ProfileVelocity=0(속도 프로파일 비활성=즉시 이동) write → 진짜 step.
     주의: vel=0 은 "정지"가 아니라 "프로파일 무제한". 여전히 Position Control Mode.
  4) GoalPosition 계단 입력, step 순간부터 GroupSyncRead로
     PresentCurrent(126)+PresentVelocity(128)+PresentPosition(132)를
     addr126~135 연속 10바이트 한 번에 읽어 고속 로깅.
  5) 조건별 CSV + README(레지스터 실측·달성 rate) 저장.

안전:
  - joint1 ±6π 하드클램프(케이블 3바퀴 보호). joint2 무제한(배포 규칙과 동일).
  - SIGINT(Ctrl-C)/폭주 감지 시 즉시 torque disable.
  - 배포 노드가 살아있으면 bus 충돌 → 반드시 먼저 rosnode kill.

실행 (보드 agent-jetson에서, py2.7 또는 py3 모두 동작):
  python arm_step_response.py --joint 1 --condition air_nobuoy --steps 0,30,60,0,-30 --repeats 3
  python arm_step_response.py --joint 2 --condition water_buoy --start-deg 90 --steps 90,120,90,60
사용 전 반드시: 배포 노드 종료 확인, torque/전원 e-stop 대기, 수조/케이블 클리어런스 확보.
"""

from __future__ import print_function

import argparse
import csv
import os
import signal
import sys
import time

# ---- DynamixelSDK import (배포 패키지의 SDK 재사용) ---------------------------
try:
    from dynamixel_sdk import PortHandler, PacketHandler, GroupSyncRead
    from dynamixel_sdk import COMM_SUCCESS
except ImportError:
    sys.stderr.write(
        "[FATAL] dynamixel_sdk import 실패. ROS 환경 source 후 실행하거나 "
        "PYTHONPATH에 dynamixel_sdk/src 추가.\n")
    sys.exit(1)

import math

# ==============================================================================
# 상수 — 배포 드라이버(joint_angle_command.cpp)와 1:1 대조해 맞춤
# ==============================================================================
JOINT_IDS = {1: 11, 2: 12}          # 논리 관절 -> Dynamixel ID
DEFAULT_PORT = "/dev/ttyDynamixel"
DEFAULT_BAUD = 1000000               # 배포와 동일(2026-07-03 모터 EEPROM 57600->1M). --baud 로 조절
PROTOCOL = 2.0

# 레지스터 주소 (XW540-T260, Protocol 2.0 Control Table)
ADDR_OPERATING_MODE   = 11
ADDR_TORQUE_ENABLE    = 64
ADDR_POSITION_D_GAIN  = 80
ADDR_POSITION_I_GAIN  = 82
ADDR_POSITION_P_GAIN  = 84
ADDR_PROFILE_ACCEL    = 108
ADDR_PROFILE_VELOCITY = 112
ADDR_GOAL_POSITION    = 116
ADDR_PRESENT_CURRENT  = 126          # 2 byte (int16)
ADDR_PRESENT_VELOCITY = 128          # 4 byte (int32). ⚠️ 표준 X-series는 128/4B/0.229rpm이나
                                     # XW540-T260 데이터시트로 주소·단위 재확인 필요(부호 to_signed는
                                     # 무관하게 안전, 크기·단위만 영향). 틀리면 SyncRead가 10B는 그대로
                                     # 읽되 vel 값이 조용히 잘못됨(크래시 아님).
ADDR_PRESENT_POSITION = 132          # 4 byte (int32)

# SyncRead 연속 블록: 126~135 = current(2)+velocity(4)+position(4) = 10 byte
SYNC_START = ADDR_PRESENT_CURRENT
SYNC_LEN   = 10

TORQUE_ON  = 1
TORQUE_OFF = 0

# 배포 드라이버가 enableTorque 시 write 하는 값 (측정 셋업 재현)
GAIN_P = 800
GAIN_I = 1
GAIN_D = 40
PROFILE_ACCEL_VAL = 40               # step 측정 시엔 profile 자체를 끄지만, 안전 기본값

# 단위 변환: 배포 드라이버 RAD_TO_DXL = rad/pi*2048  (2048 tick = pi)
TICK_PER_PI = 2048.0

def rad_to_dxl(rad):
    return int(round(rad / math.pi * TICK_PER_PI))

def dxl_to_rad(tick):
    return float(tick) / TICK_PER_PI * math.pi

def deg_to_rad(deg):
    return deg * math.pi / 180.0

# int32/int16 부호 복원 (SyncRead는 unsigned로 옴)
def to_signed(value, bits):
    if value >= (1 << (bits - 1)):
        value -= (1 << bits)
    return value

# 현재 mA 변환계수 (배포 드라이버 하드코딩 2.69f). XW540 current unit ~2.69mA/LSB.
CURRENT_LSB_MA = 2.69

# joint1 안전 클램프 = ±6π (케이블 3바퀴). 배포 np_policy 규칙과 동일.
JOINT1_CLAMP_RAD = 6.0 * math.pi

# nominal 자세 (배포/sim 정합): joint1=0, joint2=π/2
NOMINAL_RAD = {1: 0.0, 2: math.pi / 2.0}


# ==============================================================================
# 계측기 본체
# ==============================================================================
class StepResponseMeter(object):
    def __init__(self, port_name, baud, joint, other_hold=True):
        self.port_name = port_name
        self.baud = baud
        self.joint = joint
        self.dxl_id = JOINT_IDS[joint]
        self.other_joint = 2 if joint == 1 else 1
        self.other_id = JOINT_IDS[self.other_joint]
        self.other_hold = other_hold

        self.port = PortHandler(port_name)
        self.packet = PacketHandler(PROTOCOL)
        self.sync = GroupSyncRead(self.port, self.packet, SYNC_START, SYNC_LEN)

        self._torque_enabled_ids = []
        self.registers = {}          # 측정 시점 레지스터 실측값

    # ---- 저수준 헬퍼 ---------------------------------------------------------
    def _check(self, result, error, ctx):
        if result != COMM_SUCCESS:
            raise RuntimeError("%s: comm fail (%s)" % (ctx, self.packet.getTxRxResult(result)))
        if error != 0:
            sys.stderr.write("[WARN] %s: packet error %d (%s)\n"
                             % (ctx, error, self.packet.getRxPacketError(error)))

    def _read2(self, dxl_id, addr, ctx):
        val, result, error = self.packet.read2ByteTxRx(self.port, dxl_id, addr)
        self._check(result, error, ctx)
        return val

    def _read4(self, dxl_id, addr, ctx):
        val, result, error = self.packet.read4ByteTxRx(self.port, dxl_id, addr)
        self._check(result, error, ctx)
        return val

    def _read1(self, dxl_id, addr, ctx):
        val, result, error = self.packet.read1ByteTxRx(self.port, dxl_id, addr)
        self._check(result, error, ctx)
        return val

    def _write2(self, dxl_id, addr, data, ctx):
        result, error = self.packet.write2ByteTxRx(self.port, dxl_id, addr, data)
        self._check(result, error, ctx)

    def _write4(self, dxl_id, addr, data, ctx):
        result, error = self.packet.write4ByteTxRx(self.port, dxl_id, addr, data)
        self._check(result, error, ctx)

    def _write1(self, dxl_id, addr, data, ctx):
        result, error = self.packet.write1ByteTxRx(self.port, dxl_id, addr, data)
        self._check(result, error, ctx)

    # ---- 연결/안전 ------------------------------------------------------------
    def open(self):
        if not self.port.openPort():
            raise RuntimeError("포트 열기 실패: %s (배포 노드가 bus 점유 중이면 먼저 rosnode kill)"
                               % self.port_name)
        if not self.port.setBaudRate(self.baud):
            raise RuntimeError("baud 설정 실패: %d" % self.baud)
        print("[OK] port=%s baud=%d 열림" % (self.port_name, self.baud))

    def ping_or_die(self):
        """관절 ID 응답 확인. 무응답이면 측정 거부(안전 사전확인)."""
        ids = [self.dxl_id]
        if self.other_hold:
            ids.append(self.other_id)
        for dxl_id in ids:
            model, result, error = self.packet.ping(self.port, dxl_id)
            if result != COMM_SUCCESS:
                raise RuntimeError(
                    "ping 실패 ID=%d (%s). relay ON·baud·배선 확인 후 재시도. 측정 중단."
                    % (dxl_id, self.packet.getTxRxResult(result)))
            print("[OK] ping ID=%d model=%d" % (dxl_id, model))

    def snapshot_registers(self):
        """측정 시점 레지스터 실측값 기록 (드라이버 write값과 다르면 그 자체가 발견)."""
        reg = {}
        reg["operating_mode"] = self._read1(self.dxl_id, ADDR_OPERATING_MODE, "read mode")
        reg["p_gain"]         = self._read2(self.dxl_id, ADDR_POSITION_P_GAIN, "read P")
        reg["i_gain"]         = self._read2(self.dxl_id, ADDR_POSITION_I_GAIN, "read I")
        reg["d_gain"]         = self._read2(self.dxl_id, ADDR_POSITION_D_GAIN, "read D")
        reg["profile_velocity"]     = self._read4(self.dxl_id, ADDR_PROFILE_VELOCITY, "read profVel")
        reg["profile_acceleration"] = self._read4(self.dxl_id, ADDR_PROFILE_ACCEL, "read profAcc")
        self.registers = reg
        print("[REGISTERS @measure] mode=%d P=%d I=%d D=%d profVel=%d profAcc=%d"
              % (reg["operating_mode"], reg["p_gain"], reg["i_gain"], reg["d_gain"],
                 reg["profile_velocity"], reg["profile_acceleration"]))
        # 드라이버 write값(P800/I1/D40)과 다르면 경고 — 계획서 §안전 3번의 핵심 발견 포인트
        if (reg["p_gain"], reg["i_gain"], reg["d_gain"]) != (GAIN_P, GAIN_I, GAIN_D):
            print("[NOTE] 실측 gain (P=%d,I=%d,D=%d)이 드라이버 write값 (P=%d,I=%d,D=%d)과 다름 "
                  "→ README에 반드시 기록 (중요 발견)."
                  % (reg["p_gain"], reg["i_gain"], reg["d_gain"], GAIN_P, GAIN_I, GAIN_D))

    def setup_for_step(self):
        """진짜 계단응답을 위해 ProfileVelocity=0(프로파일 비활성) 설정."""
        # 측정 대상 관절 torque enable + gain 재현 write
        self._write1(self.dxl_id, ADDR_TORQUE_ENABLE, TORQUE_ON, "torque on")
        self._torque_enabled_ids.append(self.dxl_id)
        self._write2(self.dxl_id, ADDR_POSITION_P_GAIN, GAIN_P, "write P")
        self._write2(self.dxl_id, ADDR_POSITION_I_GAIN, GAIN_I, "write I")
        self._write2(self.dxl_id, ADDR_POSITION_D_GAIN, GAIN_D, "write D")
        # ProfileVelocity=0 => velocity profile 비활성 => 즉시(step) 이동
        self._write4(self.dxl_id, ADDR_PROFILE_VELOCITY, 0, "write profVel=0")
        print("[OK] joint%d(ID%d): torque ON, gain P%d/I%d/D%d, ProfileVelocity=0 (step mode)"
              % (self.joint, self.dxl_id, GAIN_P, GAIN_I, GAIN_D))

        # 다른 관절은 nominal에 torque-hold (roll 관성 posture 의존 통제)
        if self.other_hold:
            self._write1(self.other_id, ADDR_TORQUE_ENABLE, TORQUE_ON, "other torque on")
            self._torque_enabled_ids.append(self.other_id)
            hold_tick = rad_to_dxl(NOMINAL_RAD[self.other_joint])
            self._write4(self.other_id, ADDR_GOAL_POSITION, hold_tick, "other hold")
            print("[OK] joint%d(ID%d): nominal hold @ %.3f rad (tick %d)"
                  % (self.other_joint, self.other_id,
                     NOMINAL_RAD[self.other_joint], hold_tick))

        # SyncRead 파라미터 등록 (측정 관절만 고속 로깅)
        if not self.sync.addParam(self.dxl_id):
            raise RuntimeError("GroupSyncRead addParam 실패 ID=%d" % self.dxl_id)

    # ---- step 이동 + 로깅 ----------------------------------------------------
    def _clamp_goal_rad(self, goal_rad):
        if self.joint == 1 and abs(goal_rad) > JOINT1_CLAMP_RAD:
            clamped = math.copysign(JOINT1_CLAMP_RAD, goal_rad)
            print("[CLAMP] joint1 goal %.3f -> %.3f rad (±6π 안전클램프)" % (goal_rad, clamped))
            return clamped
        return goal_rad

    def move_to(self, goal_rad, settle_sec, log_rows, goal_tag):
        """goal_rad로 step, settle_sec 동안 SyncRead 로깅. log_rows에 append."""
        goal_rad = self._clamp_goal_rad(goal_rad)
        goal_tick = rad_to_dxl(goal_rad)

        t_step = time.time()
        self._write4(self.dxl_id, ADDR_GOAL_POSITION, goal_tick, "goal step")

        t_end = t_step + settle_sec
        n = 0
        n_fail = 0
        while time.time() < t_end:
            t = time.time()
            result = self.sync.txRxPacket()
            if result != COMM_SUCCESS:
                # 실패 샘플: bus 문제 시 100% CPU busy-spin 방지용 미세 sleep + 카운트
                n_fail += 1
                time.sleep(0.0005)
                continue
            if not self.sync.isAvailable(self.dxl_id, ADDR_PRESENT_CURRENT, 2):
                n_fail += 1
                time.sleep(0.0005)
                continue
            cur_raw = self.sync.getData(self.dxl_id, ADDR_PRESENT_CURRENT, 2)
            vel_raw = self.sync.getData(self.dxl_id, ADDR_PRESENT_VELOCITY, 4)
            pos_raw = self.sync.getData(self.dxl_id, ADDR_PRESENT_POSITION, 4)

            cur_ma = to_signed(cur_raw, 16) * CURRENT_LSB_MA
            vel = to_signed(vel_raw, 32)             # 원단위(0.229rpm) 그대로 기록
            pos_rad = dxl_to_rad(to_signed(pos_raw, 32))

            log_rows.append([
                "%.6f" % (t - t_step),               # step 순간=0 기준 상대시각
                goal_tag,
                "%.6f" % goal_rad,
                "%.6f" % pos_rad,
                vel,
                "%.3f" % cur_ma,
            ])
            n += 1

        rate = n / float(settle_sec) if settle_sec > 0 else 0.0
        print("  step->%s (goal %.1f deg): %d samples in %.2fs = %.1f Hz (fail=%d)"
              % (goal_tag, goal_rad * 180.0 / math.pi, n, settle_sec, rate, n_fail))
        return rate

    # ---- 정리 ---------------------------------------------------------------
    def emergency_stop(self):
        """폭주/인터럽트 시 즉시 torque disable."""
        for dxl_id in self._torque_enabled_ids:
            try:
                self._write1(dxl_id, ADDR_TORQUE_ENABLE, TORQUE_OFF, "estop")
                print("[ESTOP] ID=%d torque OFF" % dxl_id)
            except Exception as exc:
                sys.stderr.write("[ESTOP-ERR] ID=%d: %s\n" % (dxl_id, exc))

    def close(self):
        try:
            self.sync.clearParam()
        except Exception:
            pass
        # 포트가 열리기 전(SIGINT-before-open)엔 self.port.ser=None → closePort()가
        # AttributeError. open() 이전에 Ctrl-C 눌러도 클린 종료되도록 가드.
        try:
            if getattr(self.port, "ser", None) is not None:
                self.port.closePort()
                print("[OK] 포트 닫음")
        except Exception:
            pass


# ==============================================================================
# CSV / README 저장
# ==============================================================================
def resolve_data_dir(base, location, purpose):
    """data/#_location/YYYYMMDD_purpose/. location 모르면 data/0_etc/YYYYMMDD_purpose/."""
    day = time.strftime("%Y%m%d")
    loc = location if location else "0_etc"
    out = os.path.join(base, loc, "%s_%s" % (day, purpose))
    if not os.path.isdir(out):
        os.makedirs(out)
    return out


def write_csv(out_dir, condition, joint, log_rows):
    fname = os.path.join(out_dir, "step_%s_j%d.csv" % (condition, joint))
    f = open(fname, "w")
    try:
        writer = csv.writer(f)
        writer.writerow(["t", "goal_tag", "goal_pos_rad",
                         "present_pos_rad", "present_vel_raw", "present_current_mA"])
        writer.writerows(log_rows)
    finally:
        f.close()
    print("[SAVED] %s (%d rows)" % (fname, len(log_rows)))
    return fname


def write_readme(out_dir, meter, args, achieved_rates):
    fname = os.path.join(out_dir, "README.md")
    lines = []
    lines.append("# Arm Step-Response 측정 — %s" % time.strftime("%Y-%m-%d %H:%M:%S"))
    lines.append("")
    lines.append("- **로봇 상태**: (수동 기입) 수조/공기중, buoy 부착 여부, 수온 등")
    lines.append("- **측정 관절**: joint%d (Dynamixel ID %d)" % (args.joint, meter.dxl_id))
    lines.append("- **조건 라벨**: %s" % args.condition)
    lines.append("- **port / baud**: %s / %d" % (args.port, args.baud))
    lines.append("- **step 시퀀스(deg)**: %s" % args.steps)
    lines.append("- **시작각(deg)**: %s, 반복: %d" % (args.start_deg, args.repeats))
    lines.append("- **다른 관절 hold**: %s (nominal J1=0, J2=π/2)"
                 % ("ON" if meter.other_hold else "OFF"))
    lines.append("")
    lines.append("## 측정 시점 레지스터 실측값")
    reg = meter.registers
    lines.append("| register | 실측 | 드라이버 write값 |")
    lines.append("|:--|:--|:--|")
    lines.append("| Operating Mode (addr11) | %d | 3(Position) 가정 — 드라이버가 mode 미기재, 실측 확인 |" % reg.get("operating_mode", -1))
    lines.append("| Position P (addr84) | %d | %d |" % (reg.get("p_gain", -1), GAIN_P))
    lines.append("| Position I (addr82) | %d | %d |" % (reg.get("i_gain", -1), GAIN_I))
    lines.append("| Position D (addr80) | %d | %d |" % (reg.get("d_gain", -1), GAIN_D))
    lines.append("| Profile Velocity (addr112) | %d | 측정중 0(프로파일 비활성) |" % reg.get("profile_velocity", -1))
    lines.append("| Profile Accel (addr108) | %d | %d |" % (reg.get("profile_acceleration", -1), PROFILE_ACCEL_VAL))
    lines.append("")
    lines.append("## 달성 로깅 rate")
    lines.append("- 목표: >=200Hz (57600 baud면 SyncRead ~90Hz가 물리 상한 — baud 상향 시 200Hz+)")
    if achieved_rates:
        lines.append("- 실측 평균: %.1f Hz (%d step)"
                     % (sum(achieved_rates) / len(achieved_rates), len(achieved_rates)))
        lines.append("- step별: %s" % ", ".join("%.1f" % r for r in achieved_rates))
    lines.append("")
    lines.append("## 필수 수동 기입 (재현성)")
    lines.append("- 압력센서 rate/해상도, buoy 질량/부착위치, 수온")
    lines.append("- 모터 모델/펌웨어 버전, current 변환계수(2.69 mA/LSB 사용)")
    lines.append("- IMU body 각속도 동시 로깅 여부(수중 step base 오염 판정용)")
    lines.append("")
    lines.append("## CSV 컬럼")
    lines.append("`t`(step순간=0 상대초), `goal_tag`, `goal_pos_rad`, "
                 "`present_pos_rad`, `present_vel_raw`(0.229rpm 단위), `present_current_mA`")
    f = open(fname, "w")
    try:
        f.write("\n".join(lines) + "\n")
    finally:
        f.close()
    print("[SAVED] %s" % fname)


# ==============================================================================
# main
# ==============================================================================
def parse_args():
    p = argparse.ArgumentParser(description="ALBC arm 관절 계단응답 계측 (측정1)")
    p.add_argument("--joint", type=int, choices=[1, 2], required=True,
                   help="측정 관절 (1=ID11, 2=ID12)")
    p.add_argument("--condition", required=True,
                   help="조건 라벨 (예: air_nobuoy, water_buoy) — CSV 파일명·분리에 사용")
    p.add_argument("--steps", default="0,30,60,0,-30",
                   help="step 각도열(deg), 시작각 기준 절대 목표. 예: 0,30,60,0,-30")
    p.add_argument("--start-deg", type=float, default=None,
                   help="시작 각도(deg). 미지정시 joint nominal(J1=0,J2=90).")
    p.add_argument("--repeats", type=int, default=3, help="전체 step 시퀀스 반복 횟수")
    p.add_argument("--settle", type=float, default=1.5, help="각 step 정착 대기·로깅 시간(초)")
    p.add_argument("--port", default=DEFAULT_PORT)
    p.add_argument("--baud", type=int, default=DEFAULT_BAUD,
                   help="baud. 기본 1000000(배포동일, 200Hz+). 57600이면 ~90Hz")
    p.add_argument("--no-hold", action="store_true",
                   help="다른 관절 nominal hold 비활성(기본은 hold ON)")
    p.add_argument("--data-base", default=None,
                   help="데이터 루트. 기본 = 이 스크립트 기준 albc/data/")
    p.add_argument("--location", default=None, help="data/#_location/ 의 location (모르면 0_etc)")
    p.add_argument("--purpose", default="armstep", help="폴더 접미사 YYYYMMDD_purpose")
    return p.parse_args()


def main():
    args = parse_args()

    # 데이터 루트: albc/data/ (albc/code/agent-jetson/robot/albc_control/scripts/measurement/ 기준 상위)
    if args.data_base is None:
        here = os.path.dirname(os.path.abspath(__file__))
        args.data_base = os.path.abspath(os.path.join(here, "..", "..", "..", "..", "..", "..", "data"))

    start_deg = args.start_deg
    if start_deg is None:
        start_deg = NOMINAL_RAD[args.joint] * 180.0 / math.pi
    step_degs = [float(x) for x in args.steps.split(",") if x.strip() != ""]

    print("=" * 60)
    print(" ALBC ARM STEP-RESPONSE 계측 (측정1)")
    print("=" * 60)
    print(" joint=%d condition=%s start=%.1fdeg steps=%s repeats=%d settle=%.1fs"
          % (args.joint, args.condition, start_deg, step_degs, args.repeats, args.settle))
    print(" ⚠️ 배포 노드가 종료됐는지 확인했는가? (bus 단독 점유 필수)")
    print("=" * 60)

    meter = StepResponseMeter(args.port, args.baud, args.joint, other_hold=not args.no_hold)

    # SIGINT -> 즉시 e-stop
    def _sigint(signum, frame):
        print("\n[INT] 인터럽트 — e-stop 실행")
        meter.emergency_stop()
        meter.close()
        sys.exit(130)
    signal.signal(signal.SIGINT, _sigint)

    log_rows = []
    achieved_rates = []
    try:
        meter.open()
        meter.ping_or_die()
        meter.snapshot_registers()
        meter.setup_for_step()

        # 시작각으로 먼저 이동(첫 로깅 없이 정착)
        print("[MOVE] 시작각 %.1f deg 로 이동, 정착 대기..." % start_deg)
        meter.move_to(deg_to_rad(start_deg), args.settle, [], "start")
        time.sleep(0.3)

        for rep in range(args.repeats):
            print("--- repeat %d/%d ---" % (rep + 1, args.repeats))
            for i, tgt in enumerate(step_degs):
                tag = "r%d_s%d_%+.0f" % (rep, i, tgt)
                rate = meter.move_to(deg_to_rad(tgt), args.settle, log_rows, tag)
                achieved_rates.append(rate)

    except Exception as exc:
        sys.stderr.write("[FATAL] %s\n" % exc)
        meter.emergency_stop()
        meter.close()
        sys.exit(1)

    # 정상 종료: torque off + 저장
    meter.emergency_stop()

    out_dir = resolve_data_dir(args.data_base, args.location, args.purpose)
    write_csv(out_dir, args.condition, args.joint, log_rows)
    write_readme(out_dir, meter, args, achieved_rates)
    meter.close()

    if achieved_rates:
        avg = sum(achieved_rates) / len(achieved_rates)
        print("\n[DONE] 평균 로깅 rate %.1f Hz. 목표 200Hz 대비 %.0f%%."
              % (avg, avg / 200.0 * 100.0))
        if avg < 100 and args.baud <= 57600:
            print("[HINT] rate가 낮으면 --baud 1000000 으로 올려 재측정 (EEPROM baud 변경 필요).")


if __name__ == "__main__":
    main()
