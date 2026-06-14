# DEPRECATED (2026-06-14): QR/DVL mission dormant. Firmware (.ino) has a
# subscriber/publisher stub but NO matching ROS publisher/subscriber in the
# live graph. Do not use for new code. Full removal is bundled with firmware
# modernization (requirement #1, requires re-flash). See NAMING_CONVENTION.md.
int8 control_T
float64 Kp
float64 Ki
float64 Kd
float64 Mb
float64 KKp
float64 KKv
