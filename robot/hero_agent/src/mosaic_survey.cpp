#include "hero_agent/hero_agent_types.h"

// ==============================
// Mosaic survey pattern control
// ==============================

void executeMosaicSurvey(int count)
{
    if (ctrl.mosaic == 1) {
        // Sway positive direction
        if (count == 0) {
            mosaic.sway_count++;
            target.y += mosaic.move_dis;
        } else if (count == 9 && mosaic.sway_count > mosaic.sway_num) {
            ctrl.mosaic = 2;
            mosaic.sway_count = 0;
        }
    }
    else if (ctrl.mosaic == 2) {
        // Surge forward
        if (count == 0) {
            mosaic.surge_count++;
            target.x += mosaic.move_dis;
        } else if (count == 9 && mosaic.surge_count > mosaic.surge_num) {
            ctrl.mosaic = 3;
            mosaic.surge_count = 0;
        }
    }
    else if (ctrl.mosaic == 3) {
        // Sway negative direction
        if (count == 0) {
            mosaic.sway_count++;
            target.y -= mosaic.move_dis;
        } else if (count == 9 && mosaic.sway_count > mosaic.sway_num) {
            ctrl.mosaic = 4;
            mosaic.sway_count = 0;
        }
    }
    else if (ctrl.mosaic == 4) {
        // Surge forward (return leg)
        if (count == 0) {
            mosaic.surge_count++;
            target.x += mosaic.move_dis;
        } else if (count == 9 && mosaic.surge_count > mosaic.surge_num) {
            ctrl.mosaic = 0;
            mosaic.surge_count = 0;
        }
    }
}
