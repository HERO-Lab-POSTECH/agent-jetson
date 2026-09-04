# agent-jetson

ROS 1 (lunar) workspace for the HERO underwater vehicle's Jetson TX2. This
repository **is** `~/catkin_ws/src` on the board — clone it there, do not nest it.

Python on the board is 2.7.12 with numpy 1.11.0, and the C++ is C++11. Nothing
here may assume newer.

## Packages we wrote

| Package | What it owns |
|:---|:---|
| `robot/hero_agent` | The board node: keyboard teleop, HUD, rosbag recording, and the serial bridge to the Arduino firmware. |
| `robot/albc_control` | The arm. Dynamixel driver (`joint_angle_command`), the classic TDC attitude controller (`albc_controller`), and the measurement scripts under `scripts/measurement/`. |
| `robot/albc_rl` | The RL student policy: a torch-free numpy runtime (`src/albc_rl/`), the 72D observation builder, the thruster mixer, and `rl_inference_node.py`. |
| `robot/hero_msgs` | Message definitions and the topic-name SSOT (`include/hero_msgs/topics.h`). |

`firmware/agent/` is the Arduino sketch that drives the ESCs and reads the
sensors. Flashing it is a separate, deliberate procedure — see
`firmware/BUILD_AND_FLASH.md`.

Vendored third-party packages are inventoried in `THIRD_PARTY.md`.

## Build

```bash
source /opt/ros/lunar/setup.bash
cd ~/catkin_ws && catkin_make          # always full; never --pkg after a clean
```

A partial build after `catkin_make clean` fails, because clean removes every
generated message header and `hero_agent` depends on them transitively.

## Run

Operator aliases (defined in the board's `.bashrc`):

| Alias | Brings up |
|:---|:---|
| `launch-agent` | `hero_agent` node + serial bridge |
| `launch-albc` | classic TDC attitude control |
| `launch-rl` | RL policy inference |
| `run-joint` | arm driver alone |
| `run-albc` | attitude controller alone |
| `teleop` | keyboard teleop |

## Test

```bash
bash tests/run_all.sh                    # dev machine (python3 + numpy + pytest)
PYTHON=python2.7 bash tests/run_all.sh   # board
```

That runs the ROS-free C++ characterization suite under `tests/characterization/`
plus every pytest suite. The characterization tests pin current behaviour of the
control law, kinematics, IMU rotation, joint unwrap, and the teleop keymap; they
exist so a refactor that changes the robot's numbers fails loudly.
