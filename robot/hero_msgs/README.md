# hero_msgs

Custom ROS message definitions for the HERO underwater robot agent.

## Message Naming Convention

- Prefix: `hero_agent_` for agent-related messages
- Format: `hero_agent_<subsystem>.msg`
- Field names: `UPPER_CASE` (legacy convention)

## Messages

| Message | Purpose |
|:--------|:--------|
| `hero_agent_state.msg` | Full robot state (yaw, depth, throttle, etc.) |
| `hero_agent_sensor.msg` | IMU sensor data (roll, pitch) |
| `hero_agent_dvl.msg` | DVL target commands |
| `hero_agent_dvl_velocity.msg` | DVL velocity measurements |
| `hero_agent_cont_xy.msg` | XY thruster control commands (T0-T3) |
| `hero_agent_cont_para.msg` | QR controller parameters (Kp, Ki, Kd, etc.) |
| `hero_agent_position_result.msg` | Position result (X, Y, Z) |
| `hero_agent_vision.msg` | Vision detection results |
| `hero_usbl_cont.msg` | USBL control data |
| `hero_xy_cont.msg` | XY control data |

## Services

| Service | Purpose |
|:--------|:--------|
| `hero_command.srv` | Single command request/response |
