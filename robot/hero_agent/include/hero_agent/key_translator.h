#ifndef HERO_AGENT_KEY_TRANSLATOR_H
#define HERO_AGENT_KEY_TRANSLATOR_H
#include "hero_agent/keymap.h"
namespace hero {

struct KeyXlate { char cmd; char translated; };  // 0=발행 안 함

// allow-list 번역. self-toggle이라 state 인자 불필요.
// KEYMAP에 있으면 (fw_char 발행, translated 있으면 teleop), 없으면 {0,0} drop.
inline KeyXlate translate_key(int ch) {
    KeyXlate out = {0, 0};
    const KeyDef* k = lookup_key(ch);
    if (!k) return out;          // 미등록 → drop
    out.cmd = k->fw_char;        // 0이면 펌웨어 안 감(heave)
    out.translated = k->translated;
    return out;
}
}  // namespace hero
#endif
