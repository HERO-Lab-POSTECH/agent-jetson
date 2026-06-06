# hero_msgs

Custom ROS message definitions for the HERO underwater robot agent.

## Message Naming Convention

- Prefix: `hero_agent_` for agent-related messages
- Format: `hero_agent_<subsystem>.msg`
- Field names: `UPPER_CASE` (legacy convention)

## Messages

legacy 메시지는 현재 repo 코드에서 직접 쓰이지 않으나, Arduino 펌웨어가 rosserial로 발행/구독할 수 있어 물리 삭제하지 않고 보존한다(.ino 부재로 검증 불가).

| Message | Purpose | 상태 |
|:--------|:--------|:-----|
| `hero_agent_state.msg` | Full robot state (yaw, depth, throttle, etc.) | active |
| `hero_agent_sensor.msg` | IMU sensor data (roll, pitch) | active |
| `hero_agent_dvl.msg` | DVL target commands | active |
| `hero_agent_dvl_velocity.msg` | DVL velocity measurements | legacy (firmware-only, 코드 미사용·보존) |
| `hero_agent_cont_xy.msg` | XY thruster control commands (T0-T3) | legacy (firmware-only, 코드 미사용·보존) |
| `hero_agent_cont_para.msg` | QR controller parameters (Kp, Ki, Kd, etc.) | legacy (firmware-only, 코드 미사용·보존) |
| `hero_agent_position_result.msg` | Position result (X, Y, Z) | legacy (firmware-only, 코드 미사용·보존) |
| `hero_agent_vision.msg` | Vision detection results | active (perception) |
| `hero_usbl_cont.msg` | USBL control data | legacy (firmware-only, 코드 미사용·보존) |
| `hero_xy_cont.msg` | XY control data | active (perception) |

## Services

| Service | Purpose |
|:--------|:--------|
| `hero_command.srv` | Single command request/response |
