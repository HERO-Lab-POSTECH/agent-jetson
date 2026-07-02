# ALBC 온보드 실측 계측 스크립트

sim-to-real gap 축소용 온보드 실측 도구. **배포 드라이버(`joint_angle_command.cpp`)와 별개인 독립 계측 도구**다. 배포 노드가 현 상태로는 계획서 절차(즉시 step + 고속 Pos/Vel/Current 로깅)를 만족 못하므로, 배포 노드를 완전히 종료한 뒤 이 스크립트가 bus를 단독 점유해 측정한다.

> 설계 근거·검증 분석: `notes/2026-07-02-onboard-measurement-audit.md` (blocker B1/B2/B3, major M1~M5)

## 파일

| 파일 | 측정 | 설명 |
|:--|:--|:--|
| `arm_step_response.py` | 측정1 (주력) | 팔 관절 계단응답. DynamixelSDK 직접, GroupSyncRead로 Current+Velocity+Position 고속 로깅 |
| `net_buoyancy.py` | 측정2 (부수) | 순 부력. `/hero_agent/sensors`의 DEPTH 구독, z(t) 로깅 |

## ⚠️ 측정 전 필수 (안전 — M4 blocker)

`/dev/ttyDynamixel`은 단일 half-duplex bus다. 배포/RL 노드와 계측 스크립트가 동시에 열면 Protocol 2.0 TxRx가 인터리브되어 CRC 깨짐·데이터 오염이 발생한다. **SDK PortHandler는 O_EXCL을 안 걸어 두 번째 openPort가 대개 성공하므로, OS가 안 막고 조용히 오염된다 → 더 위험.**

```bash
# 1. 배포/RL 노드 완전 종료 (bus writer 전부)
rosnode kill /joint_angle_command   # 또는 해당 launch 종료
rosnode list | grep -iE "joint|rl_inference|albc"   # 남은 bus writer 없는지 확인

# 2. relay ON 확인 (관절 전원)
# 3. 수조/케이블/부력재 클리어런스 확보, 수동 e-stop(전원 차단) 대기
```

## 측정1 — arm step-response

```bash
# 보드에서 (py2.7/py3 모두 동작). ROS 환경 source 후:
source /opt/ros/lunar/setup.bash
source /home/nvidia/catkin_ws/devel/setup.bash

# joint1, 공기중·buoy 미부착, 시작 0deg → ±30/60 계단, 3회 반복
python arm_step_response.py --joint 1 --condition air_nobuoy --steps 0,30,60,0,-30 --repeats 3

# joint2는 nominal(π/2=90deg) 근처에서 step (M3: nominal 자세 응답이 튜닝에 더 유의미)
python arm_step_response.py --joint 2 --condition water_buoy --start-deg 90 --steps 90,120,90,60 --repeats 3

# 200Hz 필요하면 baud 상향 (EEPROM baud 변경 필요 — 측정 후 배포용 57600 복구 잊지 말 것)
python arm_step_response.py --joint 1 --condition water_buoy --baud 1000000 --repeats 5
```

**동작**: ping으로 관절 응답 확인(무응답이면 거부) → 레지스터 실측(P/I/D/ProfileVel/Mode) → ProfileVelocity=0(프로파일 비활성=즉시 step) → GoalPosition 계단 + SyncRead 고속 로깅 → 종료 시 torque off. Ctrl-C·예외 시 즉시 e-stop.

**주요 인자**: `--joint {1,2}`, `--condition <라벨>`, `--steps <deg열>`, `--start-deg`, `--repeats`, `--settle`(정착 로깅 초), `--baud`, `--no-hold`(다른 관절 hold 끄기).

**출력**: `data/<location>/YYYYMMDD_armstep/step_<condition>_j<n>.csv` + `README.md`(레지스터 실측·달성 rate 자동 기록).

## 측정2 — net buoyancy

```bash
# roscore·센서 노드 살아있어야 함. thruster OFF·관절 고정 확인 후:
rosrun albc_control net_buoyancy.py --condition neutral_buoy --duration 8 --trials 5
```

각 trial마다 Enter 대기 → 중립 깊이에 놓고 손 떼고 Enter → z(t) 로깅. SNR 낮으니 ≥5회.

**출력**: `data/<location>/YYYYMMDD_buoyancy/buoyancy_<condition>_t<n>.csv` + `README.md`(trial별 Δz 부호 요약).

## 조건 매트릭스 (계획서 §측정1)

| 조건 라벨 예시 | 매질 | buoy | 용도 |
|:--|:--|:--|:--|
| `air_nobuoy` | 공기중 | 미부착 | **actuator fit 기준** (M2: 유체·base 오염 없는 순수 모터+PID 응답) |
| `air_buoy` | 공기중 | 부착 | 부하 대조 |
| `water_nobuoy` | 수중 | 미부착 | 유체효과 대조 |
| `water_buoy` | 수중 | 부착 | **배포 조건** (검증용 hold-out, fit 아님) |

> actuator fit은 `air_nobuoy` + base 지그 클램프에서만. 수중/buoy는 sim 전체 검증용. (M2)

## 알려진 한계 (README에 반드시 반영)

- **57600 baud면 SyncRead ~90Hz가 물리 상한**. 200Hz는 baud 상향 필수. 저rate에선 rise/overshoot 신뢰 낮으니 settling·ss-error 위주. (B2)
- **측정1의 30° 대계단은 배포 동작점(50Hz delta 명령열, action ±1→±5.7°)과 물리가 다름.** 보조 sanity check로만 쓰고, 배포 동작점 재현은 별도 delta 명령열 주입 필요. (B3 — 향후 `arm_delta_sysid.py`로 추가 예정)
- **측정2는 z̈ 오더·부호만.** F_net 절대크기는 added-mass 미분리라 못 얻음. depth 센서 토픽에 IMU accel 없어 z축 가속도 교차검증 불가. (측정2 물리)
- **step 좌표는 기동시점 상대**일 수 있음(드라이버 delta 누적). 이 스크립트는 절대 tick으로 GoalPosition을 주지만, 다른 관절 hold는 nominal(J1=0, J2=π/2) 기준. README에 물리 시작각 기입. (M3)

## 향후 (미구현)

- `arm_delta_sysid.py` — 배포와 동일한 50Hz로 0.10×action delta 명령열 주입 + PresentPosition 로깅 (B3의 operating-regime system-ID). 30° 계단보다 배포 동작점에 충실.
