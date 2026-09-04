# Third-party code in this workspace

Everything under `drivers/`, `perception/`, and `firmware/libraries/` is vendored
upstream code. It entered the tree in a single import commit, `8ecc709`
(2026-02-14, "catkin_ws/src 전체 구조 재정리"), and nobody here has modified it.
Treat it as read-only: fix a bug upstream, or carry a patch that this file names.

## Kept

| Path | Package | Version | License | Why it is here |
|:---|:---|:---|:---|:---|
| `drivers/dynamixel_sdk` | dynamixel_sdk | 3.7.51 | Apache 2.0 | The arm's two Dynamixel joints. `albc_control` links against it. |
| `drivers/rosserial`, `rosserial_arduino`, `rosserial_msgs` | rosserial | 0.8.0 | BSD | The Jetson-Arduino serial bridge. `serial_node.py` is the process that carries every firmware topic. |
| `perception/camera_umd` (`uvc_camera`) | camera_umd | 0.2.7 | GPLv2 | USB camera capture, launched from `agent_launch.launch` when `camera:=true`. |
| `perception/rocon_rtsp_camera_relay` | rocon_rtsp_camera_relay | 0.0.7 | BSD | RTSP relay for the IP camera, launched when `rtsp:=true`. |
| `firmware/libraries/BlueRobotics_MS5837` | BlueRobotics MS5837 | vendored | MIT | Depth sensor driver compiled into the Arduino firmware. |

## Removed 2026-09-04

Six vendored packages had **zero references** from this repository's own code:
no launch file started them, no `CMakeLists.txt` or `package.xml` depended on
them, and nothing included their headers. The board rebuilt all of them on every
`catkin_make`, and `darknet_ros` dragged in a CUDA toolchain for it.

| Path | Note |
|:---|:---|
| `perception/darknet_ros` (199 files) | YOLO detection, CUDA. The largest single cost in a clean build. |
| `perception/ros_opencv_manipulation` | Sole consumer of `hero_msgs/hero_agent_vision.msg`, which was removed with it. |
| `perception/ros_opencv_qr_calibration` | |
| `perception/ros_opencv_save` | |
| `perception/ros_opencv_ipcam_qr` | `firmware/agent/agent.ino` still included its header without using a single symbol from it; that include went too. |
| `drivers/dynamixel_sdk_examples` | Upstream SDK samples, never built into anything we run. |

They are recoverable in full: `git checkout 90b13e4 -- perception/ drivers/dynamixel_sdk_examples`.
