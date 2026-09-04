# ADR 007 — Classic yaw: ESC deadband feed-forward

Status: accepted 2026-08-24; closed-loop stability UNJUDGED (see the last note)
Code: `firmware/agent/pid.cpp` (`PID_control_yaw`)
Test: `tests/characterization/test_yaw_deadband_ff.cpp`

Moved out of the source verbatim on 2026-09-04.

---

---- 2026-08-24: ESC 불감대 역보정 (feedforward) ---------------------------
아래 채널식이 PID_yaw 를 그대로 PWM 오프셋으로 쓰는데, ESC 는 중립 ±45 counts
안에서는 아예 돌지 않는다. 그래서 |PID_yaw| < 45 인 명령은 **전부 버려졌다** —
2026-08-24 수조 정지 830 s / 16,340 샘플에서 탈출 0회, 최대 44.22.

이건 이번 m4 재배분이 만든 결함이 아니다. 옛 4채널 믹싱도 각 채널에 |PID_yaw| 를
그대로 얹었으므로 문턱 45 는 동일했다. 재배분이 바꾼 것은 문턱 위의 권한(0.576 ->
0.288)뿐이고, 문턱 자체는 원래 있던 것이 m4 를 빼면서 드러난 것이다.

형태는 RL 믹서 thruster_mixer.undeadband() 와 동형이다(D = 45/300 = 0.15):
    out = sign(u) * (D_counts + (1-D)*|u|),   |u| < EPS -> 0
즉 클래식이 RL 과 **같은 보정**을 쓰게 되는 것이지 새 설계가 아니다.

게인 상향(P x14)은 오답이다 — 문턱을 2.8° 로 내리는 대신 명령이 613 counts 로
span 300 을 두 배 넘겨 시간의 2.8% 를 포화로 보낸다(사실상 뱅뱅). FF 는 같은 구간
replay 에서 최대 82.6 counts, 포화 0%. 근거 표: 분석 문서 §3-5.

폐루프 안정성은 미판정이다 — 개루프 replay 는 "무엇을 명령하게 되는지"까지만
말한다. 첫 수조에서 작은 스텝(10°)으로 한계주기부터 볼 것.
fabs/copysign 대신 맨 산술을 쓴다 — 보드 avr-g++ 는 C++98 이고 이 파일은
<math.h> 를 직접 include 하지 않는다. 거동은 동일하다.
