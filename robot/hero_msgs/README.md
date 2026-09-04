# hero_msgs

Custom ROS message definitions for the HERO underwater robot agent.

## Naming Convention

토픽·필드 명명 규칙은 [NAMING_CONVENTION.md](NAMING_CONVENTION.md)가 단일 진실(SSOT)이다
(ROS REP-103/135/145 기반). 요약:

- Prefix: `hero_agent_` for agent-related messages
- Format: `hero_agent_<subsystem>.msg`
- 필드명 표준은 `lower_snake_case`. 현재 메시지는 legacy `UPPER_CASE`/`Mixed_Case`이며,
  NAMING_CONVENTION.md에 **RULE-ONLY**로 기록됨(필드 rename은 펌웨어 현대화/re-flash와 묶음).

## Messages

메시지 생애주기 분류는 NAMING_CONVENTION.md 참조. DEPRECATED 메시지는 펌웨어(.ino)에
subscriber/publisher stub은 있으나 live ROS 그래프에 짝이 없다(QR/DVL 미션 dormant).
완전 제거는 펌웨어 stub 제거(re-flash)와 함께.

| Message | Purpose | 상태 |
|:--------|:--------|:-----|
| `hero_agent_state.msg` | Full robot state (yaw, depth, throttle, etc.) | ACTIVE |
| `hero_agent_sensor.msg` | IMU sensor data (roll, pitch, yaw, depth, gyro) | ACTIVE |
| `hero_agent_dvl.msg` | DVL target commands | ACTIVE |
| `hero_agent_thruster_cmd.msg` | 6-channel RL thruster command | ACTIVE |

## Services

| Service | Purpose |
|:--------|:--------|
