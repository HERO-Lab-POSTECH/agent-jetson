# ADR 006 — m4 permanently excluded, three-channel reallocation

Status: accepted 2026-08-24 (hardware fault, operator-confirmed)
Code: `firmware/agent/pid.cpp` (`PID_control_yaw` and the cont_direc mixing)

Moved out of the source verbatim on 2026-09-04.

---

---- 2026-08-24: m4 영구 배제 + 3채널 재배분 -------------------------------
m4(7.5시)는 3상 중 한 상이 개방·간헐이고 커넥터에 물리적으로 접근할 수 없다
(운영자 확인 2026-08-24). 그래서 이 함수는 m1·m2·m5 세 채널만 쓴다.

세 채널만으로도 (Fx, Fy, Mz) 랭크는 3 이라 방위는 정확히 복원된다. 각 모드가
살아 있는 셋 중 둘만 쓰면 되고, 대가는 방위 오차가 아니라 권한 50% 다:

  전/후진 (cont_direc 1·2)  m2 를 뺀다  -> m1·m5    (6시/0시 정확)
  좌/우   (cont_direc 3·4)  m5 를 뺀다  -> m1·m2    (3시/9시 정확)
  yaw                       m1 을 뺀다  -> m2·m5    (Fx=Fy=0, 순수 yaw)

어느 채널을 빼는지는 배치 기하라서 실물 부호계 가설과 **무관하다** — 부호
가설 8가지 전부에서 같은 채널이 빠진다. 바뀌는 것은 남는 두 계수의 부호뿐이고
여기서는 기존 규약을 그대로 뒀다. 수조에서 어느 축이 반대로 가면 그 분기의
두 계수 부호만 뒤집어라 -- 축당 시험 한 번으로 갈린다.
유도: code/classic_allocation_analysis.py --m4-dead (배포 TAM 에서 재유도)

throttle 이 사라진 것도 같은 이유다. 네 채널일 때 throttle 은 순 렌치가 정확히
0 인 널스페이스 항이었는데, m4 가 빠지면 널스페이스가 사라져(3x3 정칙)
4.5시로 |F|=1.0 을 미는 순수 외란이 된다. 3채널에서는 0 말고 실현이 없다.
(기본값 40 이 ESC 불감대 +-45 안쪽이라 그동안 증상이 가려져 있었다.)
