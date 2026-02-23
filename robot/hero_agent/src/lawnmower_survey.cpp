#include "hero_agent/hero_agent_types.h"

// ==============================
// Lawnmower survey pattern control
// ==============================

void executeLawnmowerSurvey(int count)
{
    if (ctrl.lawnmower == 1) {
        // Sway positive direction
        if (count == 0) {
            lawnmower.sway_count++;
            target.y += lawnmower.move_dis;
        } else if (count == 9 && lawnmower.sway_count > lawnmower.sway_num) {
            ctrl.lawnmower = 2;
            lawnmower.sway_count = 0;
        }
    }
    else if (ctrl.lawnmower == 2) {
        // Surge forward
        if (count == 0) {
            lawnmower.surge_count++;
            target.x += lawnmower.move_dis;
        } else if (count == 9 && lawnmower.surge_count > lawnmower.surge_num) {
            ctrl.lawnmower = 3;
            lawnmower.surge_count = 0;
        }
    }
    else if (ctrl.lawnmower == 3) {
        // Sway negative direction
        if (count == 0) {
            lawnmower.sway_count++;
            target.y -= lawnmower.move_dis;
        } else if (count == 9 && lawnmower.sway_count > lawnmower.sway_num) {
            ctrl.lawnmower = 4;
            lawnmower.sway_count = 0;
        }
    }
    else if (ctrl.lawnmower == 4) {
        // Surge forward (return leg)
        if (count == 0) {
            lawnmower.surge_count++;
            target.x += lawnmower.move_dis;
        } else if (count == 9 && lawnmower.surge_count > lawnmower.surge_num) {
            ctrl.lawnmower = 0;
            lawnmower.surge_count = 0;
        }
    }
}
