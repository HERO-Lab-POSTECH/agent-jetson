// Golden oracle: lawnmower survey FSM (ROS-free extraction)
//
// 1:1 transcription of lawnmower_survey.cpp:7-49 (executeLawnmowerSurvey).
// The redesign must keep lawnmower_step() producing the identical state
// sequence and target deltas, or the characterization test fails.
//
// Real code uses file-scope globals (ctrl.lawnmower, lawnmower.sway_count,
// target.y, ...). Here they are explicit parameters so the FSM is a pure
// function of (state, count, lm, target). `state` aliases ctrl.lawnmower.

#ifndef LAWNMOWER_ORACLE_H
#define LAWNMOWER_ORACLE_H

struct LawnmowerState {
    int sway_count = 0;
    int surge_count = 0;
    int sway_num = 300;
    int surge_num = 50;
    float move_dis = 0.01f;
};

struct Target {
    double x;
    double y;
    double z;
};

// Pure transcription of lawnmower_survey.cpp:7-49.
// Mutates `state` (= ctrl.lawnmower), `lm` (sway_count/surge_count) and `tgt`.
inline void lawnmower_step(int& state, int count, LawnmowerState& lm, Target& tgt)
{
    if (state == 1) {
        // Sway positive
        if (count == 0) {
            lm.sway_count++;
            tgt.y += lm.move_dis;
        } else if (count == 9 && lm.sway_count > lm.sway_num) {
            state = 2;
            lm.sway_count = 0;
        }
    }
    else if (state == 2) {
        // Surge forward
        if (count == 0) {
            lm.surge_count++;
            tgt.x += lm.move_dis;
        } else if (count == 9 && lm.surge_count > lm.surge_num) {
            state = 3;
            lm.surge_count = 0;
        }
    }
    else if (state == 3) {
        // Sway negative
        if (count == 0) {
            lm.sway_count++;
            tgt.y -= lm.move_dis;
        } else if (count == 9 && lm.sway_count > lm.sway_num) {
            state = 4;
            lm.sway_count = 0;
        }
    }
    else if (state == 4) {
        // Surge forward (return leg)
        if (count == 0) {
            lm.surge_count++;
            tgt.x += lm.move_dis;
        } else if (count == 9 && lm.surge_count > lm.surge_num) {
            state = 0;
            lm.surge_count = 0;
        }
    }
}

#endif // LAWNMOWER_ORACLE_H
