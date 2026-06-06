// Golden oracle: teleop processKey (ROS-free extraction)
//
// 1:1 transcription of teleop.cpp:82-107 (processKey). The ROS plumbing
// (ros::Rate& loop_rate parameter) is dropped because processKey never uses
// it. State (target.x/y/z, ctrl.lawnmower, lawnmower.sway/surge_count) is
// passed explicitly. The redesign must keep process_key() byte-identical.
//
// Sign conventions pinned here (do NOT "fix" without intent):
//   w/s -> x +/-     d/a -> y +/-     r -> z- , f -> z+   (heave)

#ifndef PROCESSKEY_ORACLE_H
#define PROCESSKEY_ORACLE_H

struct PkState {
    double x;
    double y;
    double z;
    int lawnmower;
    int sway_count;
    int surge_count;
};

// Pure transcription of teleop.cpp:82-107.
inline void process_key(int ch, double xy_step, double z_step, PkState& s)
{
    if (ch < 0) return;   // teleop.cpp:84

    switch (ch) {
    // XYZ teleop (teleop.cpp:89-94)
    case 'w': s.x += xy_step; break;
    case 's': s.x -= xy_step; break;
    case 'd': s.y += xy_step; break;
    case 'a': s.y -= xy_step; break;
    case 'r': s.z -= z_step;  break;
    case 'f': s.z += z_step;  break;

    // Lawnmower control (teleop.cpp:97-102)
    case 'o':
        s.lawnmower = 0;
        break;
    case 'p':
        s.lawnmower = 1;
        s.sway_count = 0;
        s.surge_count = 0;
        break;

    default:
        break;
    }
}

#endif // PROCESSKEY_ORACLE_H
