# Characterization tests — hero_agent (chain 1)

현 거동을 **박제(pin)** 하는 골든 테스트. hero_agent 패키지의 공격적 재설계 전에
"지금 이 코드가 *하는 일*"을 코드로 고정해, 재설계가 거동을 의도치 않게 바꾸면
즉시 실패하게 만드는 **회귀 안전망**이다.

> 일반 TDD("해야 할 일 → 테스트 → 구현")가 아니라 특성화 테스트
> ("지금 하는 일 → 박제 → 재설계 후에도 동일")다. 버그조차 일단 고정한다 —
> 목적은 *옳음*이 아니라 *불변(invariance)* 검증.

## 실행

ROS·catkin 불필요. 로컬 C++ 컴파일러만 있으면 된다:

```bash
tests/characterization/run.sh        # 전부 빌드+실행, 하나라도 실패 시 non-zero
```

재설계 **전후로 둘 다** 돌려 출력이 동일해야 한다.

## 무엇을 박제하는가

| 테스트 | 대상 (원본) | 박제 내용 |
|:---|:---|:---|
| `test_key_translation.cpp` | `agent_main.cpp:207-298` (V3 키 번역 switch) | 입력키 → `/hero_agent/command` char + `/hero_agent/key_translated` char. 토글 ON/OFF 양분기, blocked 키, pass-through 25종 |
| `test_keymap_table.cpp` | `keymap.h` (선언형 KEYMAP 테이블) | 토글/고정/translated 키 정의 + `is_blocked_key`/`lookup_key` 분기 |
| `test_processkey.cpp` | `teleop.cpp` (processKey) | 키 → target.x/y/z 변위 + 부호 규약(r=z−, f=z+) |

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
