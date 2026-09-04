# Changelog — albc_rl

RL student-policy 추론 패키지(72D attitude-only, torch-free numpy 런타임)의 변경 이력.
가장 안전크리티컬한 추론 경로이므로 동결 계약(frozen contract) 변경은 모두 여기 기록한다.

## [Unreleased] — refactor/cleanup-2026-09 (병합 대기)

브랜치 `refactor/cleanup-2026-09` (base `90b13e4`, 커밋 12개) 의 albc_rl 몫.
동결 계약(obs 배치·integral·DELTA_SCALE·액션 차원)은 하나도 안 바뀌었다.
버전 번호는 `deploy/72d-inc9998-gru` 로 ff-merge 하는 시점에 확정한다.

### Added
- **`src/albc_rl/` catkin python 패키지** (`66bf43a`, D3). `np_policy`·`npforward`·
  `build_proprio`·`arm_guard` 가 `scripts/`·`numpy_port/` 에서 이리로 옮겨 왔고
  `setup.py` + `catkin_python_setup()` 으로 설치된다. `rl_inference_node.py` 의
  `sys.path` 해킹이 사라졌다. **배포 팩과 보드 코드가 분리됐다** — `numpy_port/` 는 이제
  `weights_*.npz`·`golden/`·`MANIFEST.*.json` 만 담으므로 팩을 통째 교체해도 정책 런타임이
  같이 날아가지 않는다.
- **`src/albc_rl/contract.py`** — 상수 SSOT. `POLICY_OBS_DIM`·`ACTION_DIM`·`CONTROL_DT`·
  `DELTA_SCALE`·`_wrap_angle`·`TOPICS` 등 두 파일에 흩어져 있던 21종을 한 모듈로 모았다.
  `90b13e4` 의 값·dtype·표현과 전수 대조 완료 (`66bf43a`, `13c5d0d`).
- **토픽 파리티 테스트 4종** (`test_deploy_constants.py`): C++/python SSOT 문자열 동일,
  펌웨어는 교집합에서 일치, launch·sh 의 모든 토픽이 SSOT 출신, 옛 철자 잔존 0건.
  변이 검증 완료 — C++ 상수 하나를 일부러 틀리면 2개가 FAIL 한다 (`13c5d0d`).
  스캐너가 `.md` 까지 훑는다 (`312d344`, 아래 Notes 참조).
- C++↔python 교차 pin (`L1/L2`, LPF, `LEVEL_THRESHOLD`) 과 `install` 규칙 보강
  (launch·`numpy_port`·`deployed_tam.json`) (`d504f96`).

### Changed
- **토픽 이름** (`13c5d0d`, D13 안 A): RL 명령 입력 `/albc/rl_command`, 관절 명령 출력
  `/albc/joint1_cmd`·`/albc/joint2_cmd`, 관절 전류 입력 `/albc/joint_currents`.
  Jetson 내부 전용이라 **재flash 불필요**.
- `_command` 를 dynamic_reconfigure `Server(...)` **앞에서** 생성. 서버가 초기 config 로
  `_on_reconfigure` 를 동기 호출하는데 그 시점에 `_command` 가 아직 없었다 (`9471176`).
- 틱당 명령을 한 번만 스냅샷(`cmd = self._command.copy()`) — 관측과 액션이 서로 다른
  설정점에서 조립될 수 없다 (`9471176`).
- **문서 정정 69D → 72D** (`d504f96`). 코드는 이미 72D 였고 서술만 낡아 있었다. 같은
  커밋에서 `/albc_status` 옛 이름 서술도 정정.
- **`npforward.py` 입력 계약 docstring 정정** — "must ALREADY BE NORMALIZED" 는 틀렸다.
  torch 배포 오라클(`DeployedStudentPolicy`)도 raw obs 를 인코더에 준다는 것을 골든 재실행
  9.54e-07 일치로 확인했다 (`d504f96`). ⚠️ codex 적대 리뷰가 이것을 CRITICAL **코드** 결함
  으로 올렸으나 **기각**됐다 — 결함은 코드가 아니라 docstring 이었다.
- 테스트 docstring 의 실행법을 `PYTHONPATH=../src` 로 갱신. `src/` 이동 후 광고돼 있던
  `python3 <file>` 이 안 돌고 있었다. **적기 전에 실제로 돌려서** 확인(39 passed / 3 passed)
  (`ced45d6`).
- `package.xml` 정합: `rostopic` 잔류 제거, `python-numpy` 추가 (`d504f96`).

### Removed
- `INTEGRAL_GATED` else 분기 — 72D 계약에서 항상 True 라 도달 불가. 의미는 사용처 주석으로
  이전 (`9471176`).
- `JOINT1_TRAIN_LIMIT` (참조 0). 이 상수가 들고 있던 2026-08 클램프 제거 근거는 누산이
  실제로 일어나는 자리에 복원했다 (`9471176`).
- `b1_channel_probe.py` 의 도달 불가 분기, `_arm_guard()` 의 미사용 `target` 인자 (`9471176`).

### Moved to docs/adr (실행 코드 변경 0)
- `np_policy.py`·`rl_inference_node.py`·`thruster_mixer.py`·`build_proprio.py` 의 사고 경위
  서술 → `docs/adr/001`(unwrap·무클램프)·`002`(arm guard 전류 캡)·`003`(thruster order/sign)·
  `005`(thruster 1차 지연 dt)·`006`(m4 배제·3채널 재배분)·`007`(ESC 불감대 FF).
  옮긴 문장은 원문 그대로. 코드에는 규약과 포인터만 (`79397ff`).
  🔴 006·007 이 끝에 달고 있는 **"폐루프 미판정"** 경고는 이전엔 주석 한가운데 묻혀 있었는데
  이제 각 ADR 의 마지막 줄이자 색인의 마지막 문단이다.

### Verification
- `PYTHON=python3.12 bash tests/run_all.sh` → `RUN_ALL: PASS` (characterization 13종
  533 checks, pytest 83).
- **E2E 골든 파리티 PASSED** (atol 1e-5, 24 steps; obs 3.576e-07, actions 3.576e-07,
  joint_targets 8.941e-08, integrals·bias_emas 0) — 패키지 이동이 수치 경로를 바꾸지
  않았음의 실측.
- **미실행: 보드 게이트** (py2.7 + numpy 1.11 에서의 `run_all.sh`, 건식 launch 3종).
  `tests/board_gate.sh` 로 스크립트화만 됐고 보드 미접속으로 한 번도 안 돌았다. 병합 전 필수.

### Frozen contract (현재 값)
- `POLICY_OBS_DIM` = **72**, `ACTION_DIM` = 8, `DELTA_SCALE` = 0.10 — 정본은
  `src/albc_rl/contract.py`. 위 1.0.0 항목의 69 는 그 시점 값이다.
- integral 3D: leaky, gated(|err| < sigma), clamp ±2.0, on [roll, pitch, yaw_rate] — 불변.
- numpy 1.11.0 호환 밴드 — 불변.

### Notes / 여전히 열린 것
- 1.0.0 이 남긴 **"동결 계약 load-time assert 부재(npz shape·numpy version)"** 는 아직
  열려 있다. 이번 정리 범위 밖.
- 파리티 스캐너가 `.md` 를 안 훑고 있던 것이 옛 토픽 이름이 `docs/ARCHITECTURE.md` 7곳에
  살아남은 원인이었다 — 리네임 커밋이 그 파일을 세 번 만졌는데도 게이트가 초록이었다.
  `.md` 를 추가하고 옛 이름 재주입 → FAIL, 되돌림 → PASS 로 변별력을 확인했다 (`312d344`).
  검출이 일인 두 파일(이 스캐너 자신, `tests/board_gate.sh`)만 이름으로 면제된다 (`a4a91c5`).

---

## [1.0.0] — 2026-06-15 — CHANGELOG 신설 (보드 배포 가능 상태 박제)

agent-jetson(TX2, ROS lunar/py2.7, numpy 1.11) 보드에서 student 정책을 torch 없이
추론하는 노드의 첫 정식 CHANGELOG. 그동안의 패키지 신설~69D 전환~보드 검증 이력을
소급 정리하고, 동결 계약을 명문화한다.

### Added
- `rl_inference_node` — 보드 RL student 정책 추론 노드 (632bfb8: 패키지 신설,
  69D attitude-only). GYRO 진값 읽기 + rotate_gyro + sensors dict 주입 (dd94aa9).
- `numpy_port/` — torch-free 순전파 재구현 (TRT가 Conv1d/GRU/LayerNorm 파싱 불가 →
  numpy 순전파가 유일 배포 경로). `npforward.py`(순전파), `np_policy.py`(69D obs +
  history + integral), 골든 파리티 테스트 2종(`test_npforward.py`, `test_np_policy_api.py`).
- `rotate_gyro` — sensor→base frame 자이로 변환 (593ccf1; r=yaw rate는 z축 회전
  불변이라 통과). euler 미분 fallback과 진값 우선 분기 (2f35df9).
- dynamic_reconfigure로 `imu_yaw_offset` 실시간 튜닝 (f447156).

### Changed
- obs 차원 87D(26D) → **69D attitude-only**(20D proprio + 46D history + 3D integral).
  선속도 추종 전체 제거. marinegym-isaaclab attitude_only sim에서 계약 추출.
- `/albc_status` → `/albc/status` 네임스페이스 통일 (49d4216, 4-site lock-step).
- npforward.py py2.7 호환: `@` matmul → `np.dot` (6662546, ROS lunar rospy=py2.7).
- package.xml: exec_depend → depend (fbf37b0, catkin_package find_package 정합).

### Verification
- numpy_port 69D e2e 보드 1e-5 파리티 검증 (action 2.68e-7, integral 7.28e-12).
- 실기동 1·2차: 통신두절 해소 후 정책 첫 거동 관측. 안전게이트(STALE 감지 시 HOLDING) 정상.

### Frozen contract (변경 시 반드시 기록)
- POLICY_OBS_DIM = 69, ACTION_DIM = 8, DELTA_SCALE = 0.10.
- integral 3D: leaky, gated(|err| < sigma), clamp ±2.0, on [roll, pitch, yaw_rate].
- numpy 1.11.0 호환 밴드 (keepdims 침묵 무시 함정 — 1.11 미만 금지).

### Notes / 보류
- 69D GRU 체크포인트 미존재 → `test_npforward.py::test_gru`는 `[SKIP]` 가드로 통과(TCN-only 배포).
- 동결 계약 load-time assert 부재 (npz shape·numpy version) — enterprise-cleanup Task 3 대상.
