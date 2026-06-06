#ifndef HERO_AGENT_KEY_TRANSLATOR_H
#define HERO_AGENT_KEY_TRANSLATOR_H

#include "hero_agent/keymap.h"

namespace hero {

struct KeyXlate { char cmd; char translated; };  // 0 = not published

// keymap 테이블 룩업 기반 V3 번역. 거동은 기존 oracle과 byte-identical.
// lawnmower_on_prev: 'p'(lawnmower) 특수처리에만 사용. (Task 4에서 'p' 제거 예정)
inline KeyXlate translate_key(int ch, const ToggleState& st,
                              bool lawnmower_on_prev = false) {
    KeyXlate out = {0, 0};

    // lawnmower 'p' 특수처리 (테이블에 없음; Task 4에서 제거)
    if (ch == 'p') {
        out.translated = (!lawnmower_on_prev) ? 'p' : 'o';
        return out;
    }

    const KeyDef* k = lookup_key(ch);
    if (k) {
        if (k->toggle) {
            int enabled = 0;
            switch (k->toggle_bit) {
                case ToggleBit::RELAY: enabled = st.relay_enabled; break;
                case ToggleBit::YAW:   enabled = st.control_yaw_enabled; break;
                case ToggleBit::DEPTH: enabled = st.control_depth_enabled; break;
                case ToggleBit::LASER: enabled = st.laser_enabled; break;
                case ToggleBit::NONE:  break;
            }
            out.cmd = enabled ? k->cmd_on : k->cmd_off;
        } else {
            out.cmd = k->cmd_off;       // 고정 cmd (translated키는 cmd_off=0)
            out.translated = k->translated;
        }
        return out;
    }

    if (is_blocked_key(ch)) return out;  // {0,0}

    // pass-through
    out.cmd = (char)ch;
    out.translated = (char)ch;
    return out;
}

}  // namespace hero

#endif  // HERO_AGENT_KEY_TRANSLATOR_H
