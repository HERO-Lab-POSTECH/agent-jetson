# Characterization tests — hero_agent (chain 1) + albc_control (chain 2)

현 거동을 **박제(pin)** 하는 골든 테스트. hero_agent 패키지의 공격적 재설계 전에
"지금 이 코드가 *하는 일*"을 코드로 고정해, 재설계가 거동을 의도치 않게 바꾸면
즉시 실패하게 만드는 **회귀 안전망**이다.

> 일반 TDD("해야 할 일 → 테스트 → 구현")가 아니라 특성화 테스트
> ("지금 하는 일 → 박제 → 재설계 후에도 동일")다. 버그조차 일단 고정한다 —
> 목적은 *옳음*이 아니라 *불변(invariance)* 검증.

## 실행

두 가지 경로가 있다. 둘 다 같은 standalone `.cpp`를 빌드한다 (gtest 아님).

**로컬/보드 (ROS·catkin 불필요)** — 모든 안전망을 한 번에:

```bash
bash tests/run_all.sh                # 전부 빌드+실행, 하나라도 실패 시 non-zero
```

**catkin (보드 워크스페이스)** — `characterization_tests` 패키지로 통합됨:

```bash
catkin_make run_tests                 # 13종 일괄 실행 (CTest 요약에 표시)
# 빠지면 fallback: catkin_make tests && (cd build/characterization_tests && ctest)
```

`package.xml`이 `hero_agent`·`albc_control`을 build/test depend로 가져와 두 패키지의
exported include를 상속한다. 테스트 `.cpp`는 package-style include(`hero_agent/...`,
`albc_control/...`)를 써서 run.sh(`-I` 플래그)와 catkin(`include_directories`) 양쪽에서
같은 헤더를 찾는다.

재설계 **전후로 둘 다** 돌려 출력이 동일해야 한다.

## 무엇을 박제하는가

| 테스트 | 대상 (원본) | 박제 내용 |
|:---|:---|:---|
| `test_key_translation.cpp` | `agent_main.cpp:207-298` (V3 키 번역 switch) | 입력키 → `/hero_agent/command` char + `/hero_agent/key_translated` char. 토글 ON/OFF 양분기, blocked 키, pass-through 25종 |
| `test_keymap_table.cpp` | `keymap.h` (선언형 KEYMAP 테이블) | self-toggle/one-shot/translated 키 정의 + allow-list `lookup_key` (미등록 키 → nullptr) |
| `test_processkey.cpp` | `teleop.cpp` (processKey) | 키 → target.x/y/z 변위 + 부호 규약(r=z−, f=z+) |

### chain 2 (albc_control — 제어 수학)

제어 알고리즘이라 chain 1보다 거동 보존이 훨씬 민감하다. 아래는 `albc_controller.cpp`의 제어 수학을 ROS-free 로 박제한 것 (Phase 1, 2026-06-06).

| 테스트 | 대상 (원본) | 박제 내용 |
|:---|:---|:---|
| `test_kinematics.cpp` | `albc_kinematics.h` (순수 헤더, 직접 include) | FK·Jacobian·mapTo2Pi·mapTo360. closed-form 독립검산 병행 |
| `test_imu_rotation.cpp` | `albc_controller.cpp:205-214` (imuCallback) | IMU yaw offset 회전 (PITCH 반전·회전행렬 부호·yaw pass-through) |
| `test_dls_ik.cpp` | `albc_controller.cpp:155-199` (updateJointAngles) | DLS 역기구학 1스텝 (λ 가변댐핑·JᵀJ+λ²I·pseudo-inverse·특이점 클램프) |
| `test_control_law.cpp` | `albc_controller.cpp:385-433` (computeControlOutput) | 4모드 제어식 (TDC common_factor·PID·FIXED·MANUAL no-op·equilibrium hold) |
| `test_damping_integral.cpp` | `albc_controller.cpp:674-702` (피드백 파이프라인) | asymmetric damping gate(sign-only, RAW)·integral freeze+clamp·derivative LPF |
| `test_firmware_dvl.cpp` | `firmware/agent/dvl_position.cpp` | 살아남은 `/hero_agent/dvl` 콜백을 호스트에서 **실제 컴파일**해 두 분기(리셋·목표 저장)를 pin. 2026-09-04 에 발행자 없는 콜백 3개를 지운 뒤 남은 extern 이 전부 맞는지 확인하는 것이 주목적 — 빠지면 보드에서만 링크 에러가 난다 |
| `test_deadband_gate.cpp` | `control_law.h`·`feedback_filters.h` | target 0에서 기존 measured-angle gate와 bit-identical, nonzero target에서 error gate가 명령을 내는지 |
| `test_ee_seed_guard.cpp` | `albc_controller.cpp:main()` 시드 경로 | raw-cumulative 관절값의 wrap 시드와 접힌 특이점 EE 반경 guard |
| `test_joint_unwrap.cpp` | `joint_unwrap.h` | 현장 replay의 첫 명령 unwrap·3-turn guard·기존 single-step 결함 재현 |
| `test_yaw_deadband_ff.cpp` | `firmware/agent/pid.cpp`·`agent.ino` | yaw deadband feed-forward·I clamp·모터 채널 적용 |

## oracle 의 정직성

각 `*_oracle.h` 는 원본 함수의 **ROS 의존을 제거한 1:1 추출본**이며, 줄번호
주석으로 원본과 추적 가능하다. oracle 은 코드의 *복사본*이므로:

- ✅ 보장: 재설계 코드 == oracle (테스트가 PASS면 거동 불변)
- ⚠️ 전제: oracle == 원본 (추출 시점에 1:1로 박제 — 줄번호 주석으로 검증 가능)

## 박제하지 않은 것 (의도적 제외)

- **debounce 타이밍** (`agent_main.cpp:73`, 500ms 게이트): 어느 char 를 보내느냐가
  아니라 *반복 입력을 떨구느냐*만 결정 → 키 매핑 불변식과 무관. oracle 은 게이트
  OPEN 가정.
- **ROS 발행 자체** (`pub_*.publish`): 부수효과라 순수 로직에서 분리.
- **Arduino 측 액추에이터 동작**: 펌웨어가 repo 에 없어 박제 불가 (Phase 0 참조).

### chain 2 (albc_control)

- **타이밍 의존 거동**: 50Hz 고정 dt, 매 루프 IK 반복수(최대 3000), `spinOnce` 위치.
  순수 함수 1스텝은 박제하나, "몇 번 반복하면 수렴하나"·"콜백 지연" 같은 루프 타이밍은
  순수 단위로 분리 불가 → 재설계 시 루프 구조 변경은 빌드 게이트(실보드 catkin_make)로만 검증.
- **Dynamixel I/O** (`joint_angle_command.cpp`): 모터 통신·각도 변환은 하드웨어 의존이라
  ROS-free 박제 불가. `RAD_TO_DXL`/`DXL_TO_RAD`/`updateJoint` unwrap 로직은 향후 필요 시 추가.
- **dynamic_reconfigure 콜백**: 런타임 게인 주입은 ROS 의존 → oracle 은 게인을 인자로 받음.
- **dead `Kd_td` 게인**: TDC 식에서 D-term 은 df46b09 에서 제거됨(미사용). oracle(`control_law.h`)
  은 `Kd_td` 를 입력으로 받지 않으므로 박제에 영향 0. Phase 4 Task 7 에서 게인 구조체·yaml·cfg
  에 잔존하던 `Kd_td` 도 거동 무변으로 제거 (재도입 필요 시 git 이력 참조).
