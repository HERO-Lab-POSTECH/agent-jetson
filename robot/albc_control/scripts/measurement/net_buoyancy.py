#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
net_buoyancy.py — ALBC UUV 순 부력 계측 (측정2, 부수)

목적: thruster OFF·관절 고정 상태에서 중립 깊이에 놓고 손을 뗀 뒤
depth z(t)를 로깅해 **순 수직력의 부호(뜨나/가라앉나)와 오더**를 판정.
(SSOT: notes/2026-07-02-onboard-measurement-audit.md 측정2 / minor)

물리 한계(반드시 인지):
  - a0 = F_net / (m + m_A_heave). 단일 z(t)로는 F_net과 added-mass 분리 불가
    → 얻는 것은 **z̈(가속도)이지 F_net 절대크기가 아님**. 부호는 신뢰, 크기는 오더만.
  - drag는 초기창(≤0.15s, 변위<3mm)에서만 무시가능. 그 창은 압력센서
    해상도(MS5837류 ~2mm)·rate 병목이라, **부호는 수초 상승/하강 방향으로,
    오더는 후반 종단속도로** 얻는 게 실무적으로 안전.
  - SNR 낮으므로 최소 5회, 서로 다른 초기깊이 2~3, 시간정렬 평균 권장.

⚠️ 이 depth 센서 토픽(/hero_agent/sensors, hero_agent_sensor.msg)에는
   ROLL/PITCH/YAW/DEPTH만 있고 **가속도(IMU accel) 필드가 없다** → z축 가속도
   직접 교차검증 불가. z(t)만 로깅한다.

데이터 소스: /hero_agent/sensors (hero_agent_sensor) 의 DEPTH 필드 (raw 센서값).
  ⚠️ DEPTH 단위는 펌웨어 raw — README에 단위(m/mbar 등)·부호(아래가 +인지) 반드시 기입.

실행 (보드 agent-jetson에서, roscore·센서 노드 살아있어야 함):
  rosrun albc_control net_buoyancy.py --condition neutral_buoy --duration 8 --trials 5
  또는: python net_buoyancy.py --condition neutral --duration 8

사용 전: thruster 전부 OFF, 관절 torque hold(또는 off 고정), 손 뗀 시점 표시.
"""

from __future__ import print_function

import argparse
import csv
import os
import sys
import time

try:
    import rospy
    from hero_msgs.msg import hero_agent_sensor
except ImportError as exc:
    sys.stderr.write(
        "[FATAL] rospy/hero_msgs import 실패 (%s). ROS 환경 source 후 "
        "rosrun 으로 실행. roscore·센서 노드가 떠 있어야 함.\n" % exc)
    sys.exit(1)


class DepthLogger(object):
    def __init__(self):
        self.rows = []          # (t_rel, depth_raw)
        self.t0 = None
        self.last_depth = None
        self.count = 0
        self.recording = False

    def start(self):
        self.rows = []
        self.t0 = time.time()
        self.count = 0
        self.recording = True

    def stop(self):
        self.recording = False

    def callback(self, msg):
        if not self.recording:
            self.last_depth = msg.DEPTH
            return
        t = time.time() - self.t0
        self.rows.append(("%.6f" % t, "%.6f" % msg.DEPTH))
        self.last_depth = msg.DEPTH
        self.count += 1


def resolve_data_dir(base, location, purpose):
    day = time.strftime("%Y%m%d")
    loc = location if location else "0_etc"
    out = os.path.join(base, loc, "%s_%s" % (day, purpose))
    if not os.path.isdir(out):
        os.makedirs(out)
    return out


def write_csv(out_dir, condition, trial, rows):
    fname = os.path.join(out_dir, "buoyancy_%s_t%d.csv" % (condition, trial))
    f = open(fname, "w")
    try:
        w = csv.writer(f)
        w.writerow(["t", "depth_raw"])
        w.writerows(rows)
    finally:
        f.close()
    print("[SAVED] %s (%d rows)" % (fname, len(rows)))
    return fname


def write_readme(out_dir, args, trial_stats):
    fname = os.path.join(out_dir, "README.md")
    lines = []
    lines.append("# Net Buoyancy 측정 — %s" % time.strftime("%Y-%m-%d %H:%M:%S"))
    lines.append("")
    lines.append("- **조건**: %s" % args.condition)
    lines.append("- **trials**: %d, 각 %.1fs 로깅" % (args.trials, args.duration))
    lines.append("- **토픽**: /hero_agent/sensors (hero_agent_sensor.DEPTH)")
    lines.append("")
    lines.append("## ⚠️ 필수 수동 기입")
    lines.append("- **DEPTH 단위·부호**: raw 센서값 단위(m? mbar?), 아래로 갈수록 +인지 -인지")
    lines.append("- **압력센서 실효 rate·해상도** (MS5837류 ~2mm, 고속 악화)")
    lines.append("- 로봇 총 무게(저울 실측 — a0에서 F_net 하한 잡는 데 필요)")
    lines.append("- 부착물(buoy) 질량/부피/부착위치")
    lines.append("- 각 trial 초기 깊이, 손 뗀 시점(가능하면 CSV t=0과 정렬)")
    lines.append("- thruster OFF·관절 고정 확인")
    lines.append("")
    lines.append("## trial별 요약 (부호 판정)")
    lines.append("| trial | samples | achieved Hz | z_start | z_end | Δz | 부호(추정) |")
    lines.append("|:--|:--|:--|:--|:--|:--|:--|")
    for st in trial_stats:
        lines.append("| %d | %d | %.1f | %.4f | %.4f | %+.4f | %s |"
                     % (st["trial"], st["n"], st["rate"], st["z0"], st["z1"],
                        st["dz"], st["sign"]))
    lines.append("")
    lines.append("## 해석 주의")
    lines.append("- **부호**(뜨나/가라앉나)는 Δz 방향으로 신뢰 가능.")
    lines.append("- **크기**는 z̈ 오더만 — F_net = (m+m_A)·z̈ 에서 added-mass 미분리라 절대크기 아님.")
    lines.append("- 초기 포물선 피팅(z=z0+v0·t+½a·t²)으로 a 추정 시 drag-free 창(≤0.15s) 짧음에 주의.")
    lines.append("- 사용처: 측정 net buoyancy → DORAEMON volume/body_mass/water_density DR의 부력 중심 재캘리브레이션.")
    f = open(fname, "w")
    try:
        f.write("\n".join(lines) + "\n")
    finally:
        f.close()
    print("[SAVED] %s" % fname)


def parse_args(argv):
    p = argparse.ArgumentParser(description="ALBC net buoyancy 계측 (측정2)")
    p.add_argument("--condition", required=True, help="조건 라벨 (예: neutral_buoy)")
    p.add_argument("--duration", type=float, default=8.0, help="trial당 로깅 시간(초)")
    p.add_argument("--trials", type=int, default=5, help="반복 trial 수 (SNR 낮으니 >=5 권장)")
    p.add_argument("--data-base", default=None)
    p.add_argument("--location", default=None)
    p.add_argument("--purpose", default="buoyancy")
    # rosrun이 붙이는 __name/__log 인자 무시
    args, _ = p.parse_known_args(argv)
    return args


def trial_stat(trial, rows, duration):
    n = len(rows)
    rate = n / duration if duration > 0 else 0.0
    if n >= 2:
        z0 = float(rows[0][1])
        z1 = float(rows[-1][1])
        dz = z1 - z0
        sign = "+ (깊어짐/가라앉음?)" if dz > 0 else "- (얕아짐/뜸?)"
    else:
        z0 = z1 = dz = 0.0
        sign = "N/A"
    return {"trial": trial, "n": n, "rate": rate, "z0": z0, "z1": z1, "dz": dz, "sign": sign}


def main():
    args = parse_args(rospy.myargv(argv=sys.argv)[1:])

    if args.data_base is None:
        here = os.path.dirname(os.path.abspath(__file__))
        args.data_base = os.path.abspath(
            os.path.join(here, "..", "..", "..", "..", "..", "..", "data"))

    rospy.init_node("net_buoyancy_meter", anonymous=True)
    logger = DepthLogger()
    rospy.Subscriber("/hero_agent/sensors", hero_agent_sensor, logger.callback, queue_size=200)

    print("=" * 60)
    print(" ALBC NET BUOYANCY 계측 (측정2)")
    print("=" * 60)
    print(" condition=%s trials=%d duration=%.1fs" % (args.condition, args.trials, args.duration))
    print(" ⚠️ thruster 전부 OFF·관절 고정 확인했는가?")
    print("=" * 60)

    # 센서 수신 대기
    print("[WAIT] /hero_agent/sensors 수신 대기...")
    t_wait = time.time()
    while logger.last_depth is None and not rospy.is_shutdown():
        if time.time() - t_wait > 10.0:
            sys.stderr.write("[FATAL] 10초간 depth 미수신. 센서 노드·토픽 확인. 중단.\n")
            sys.exit(1)
        time.sleep(0.1)
    print("[OK] depth 수신 (raw=%.4f)" % logger.last_depth)

    out_dir = resolve_data_dir(args.data_base, args.location, args.purpose)
    trial_stats = []

    for trial in range(1, args.trials + 1):
        try:
            raw_input_fn = raw_input  # py2
        except NameError:
            raw_input_fn = input      # py3
        raw_input_fn("[TRIAL %d/%d] 중립 깊이에 놓고 손 뗄 준비되면 Enter → 로깅 시작 (%s)..."
                     % (trial, args.trials, args.condition))

        logger.start()
        print("  로깅 %.1fs..." % args.duration)
        t_end = time.time() + args.duration
        while time.time() < t_end and not rospy.is_shutdown():
            time.sleep(0.02)
        logger.stop()

        rows = list(logger.rows)
        write_csv(out_dir, args.condition, trial, rows)
        st = trial_stat(trial, rows, args.duration)
        trial_stats.append(st)
        print("  trial %d: %d samples %.1fHz  Δz=%+.4f %s"
              % (trial, st["n"], st["rate"], st["dz"], st["sign"]))

    write_readme(out_dir, args, trial_stats)
    print("\n[DONE] %d trials 저장. README에 DEPTH 단위·부호·센서 해상도 반드시 기입." % args.trials)


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
