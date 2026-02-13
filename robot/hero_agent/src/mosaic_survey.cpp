#include "hero_agent/agent_command_types.h"

// ==============================
// Mosaic survey pattern control
// ==============================

void executeMosaicSurvey(int count)
{
    if (cont_mosaic == 1) {
        // Sway positive direction
        if (count == 0) {
            sway_count++;
            target_y += move_dis;
        } else if (count == 9 && sway_count > sway_num) {
            cont_mosaic = 2;
            sway_count = 0;
        }
    }
    else if (cont_mosaic == 2) {
        // Surge forward
        if (count == 0) {
            surge_count++;
            target_x += move_dis;
        } else if (count == 9 && surge_count > surge_num) {
            cont_mosaic = 3;
            surge_count = 0;
        }
    }
    else if (cont_mosaic == 3) {
        // Sway negative direction
        if (count == 0) {
            sway_count++;
            target_y -= move_dis;
        } else if (count == 9 && sway_count > sway_num) {
            cont_mosaic = 4;
            sway_count = 0;
        }
    }
    else if (cont_mosaic == 4) {
        // Surge forward (return leg)
        if (count == 0) {
            surge_count++;
            target_x += move_dis;
        } else if (count == 9 && surge_count > surge_num) {
            cont_mosaic = 0;
            surge_count = 0;
        }
    }
}
