#ifndef HERO_AGENT_KEYMAP_H
#define HERO_AGENT_KEYMAP_H
namespace hero {

enum KeyGroup { GRP_SYSTEM, GRP_CONTROL, GRP_JOG, GRP_SPEED, GRP_THROTTLE, GRP_SETPOINT, GRP_GRIPPER, GRP_HEAVE };

struct KeyDef {
    int         key;          // 사용자 입력 키
    char        fw_char;      // 펌웨어로 보낼 char (0=펌웨어 안 감: heave 등)
    char        translated;   // teleop 액션 char (0=없음). heave 'r'/'f'만.
    bool        is_toggle;    // self-toggle 키(debounce 권장). 펌웨어가 state=!state.
    bool        debounce;     // 500ms 게이트
    KeyGroup    group;        // HELP 자동생성 분류
    const char* label;        // HELP 라벨
};

// SSOT. allow-list: 여기 있는 키만 처리. 없으면 translator가 drop.
// fw_char는 firmware f569da4 messageCommand(self-toggle)에 정합. 재flash 시 이 표를
// 새 messageCommand char와 대조할 것(과거 칩↔노드 규약 drift로 teleop 전면 사망한 이력).
static const KeyDef KEYMAP[] = {
    // System
    // 토글류는 firmware f569da4가 self-toggle(한 char로 firmware가 state=!state)이라
    // 노드는 상태추적 없이 fw_char 하나만 발행한다. (구 칩 2f6725d의 'e'/'t' 분리식
    // + agent.cpp 가로채기 로직은 제거됨.) is_toggle=true는 debounce 권장 표식.
    {'1', 'R', 0, true,  true,  GRP_SYSTEM,  "Relay"},
    {'2', 'P', 0, false, true,  GRP_SYSTEM,  "PWM Neutral"},
    {'N', 'Z', 0, false, true,  GRP_SYSTEM,  "Yaw Reset"},
    // Control toggles — firmware f569da4 self-toggle: yaw 'Y', depth 'D', laser 'L'
    // (구 분리식 'y'/'h', 'p'/';', 'r'/'f' + 노드 상태읽기 발행은 제거).
    {'3', 'Y', 0, true,  true,  GRP_CONTROL, "Yaw Ctrl"},
    {'4', 'D', 0, true,  true,  GRP_CONTROL, "Depth Ctrl"},
    {'5', 'L', 0, true,  true,  GRP_CONTROL, "Laser"},
    // Thruster jog
    {'w', 'w', 0, false, false, GRP_JOG,     "Forward"},
    {'s', 's', 0, false, false, GRP_JOG,     "Backward"},
    {'a', 'a', 0, false, false, GRP_JOG,     "Left"},
    {'d', 'd', 0, false, false, GRP_JOG,     "Right"},
    {'q', 'q', 0, false, false, GRP_JOG,     "Stop"},
    // Speed (user y/h -> firmware f569da4 move_speed +/-)
    {'y', '+', 0, false, false, GRP_SPEED,   "Speed+"},
    {'h', '-', 0, false, false, GRP_SPEED,   "Speed-"},
    // Throttle
    {'u', 'u', 0, false, false, GRP_THROTTLE,"Throttle+"},
    {'j', 'j', 0, false, false, GRP_THROTTLE,"Throttle-"},
    // Setpoint
    {'i', 'i', 0, false, false, GRP_SETPOINT,"Yaw+0.1"},
    {'k', 'k', 0, false, false, GRP_SETPOINT,"Yaw-0.1"},
    {'o', 'o', 0, false, false, GRP_SETPOINT,"Depth+0.1"},
    {'l', 'l', 0, false, false, GRP_SETPOINT,"Depth-0.1"},
    // Gripper
    {'c', 'c', 0, false, false, GRP_GRIPPER, "Grip Open"},
    {'v', 'v', 0, false, false, GRP_GRIPPER, "Grip Stop"},
    {'b', 'b', 0, false, false, GRP_GRIPPER, "Grip Close"},
    // Heave (teleop only, no firmware)
    {'r', 0, 'r', false, false, GRP_HEAVE,   "Heave Up"},
    {'f', 0, 'f', false, false, GRP_HEAVE,   "Heave Down"},
};
static const int KEYMAP_SIZE = sizeof(KEYMAP)/sizeof(KEYMAP[0]);

inline const KeyDef* lookup_key(int ch) {
    for (int i=0;i<KEYMAP_SIZE;++i) if (KEYMAP[i].key==ch) return &KEYMAP[i];
    return 0;
}
}  // namespace hero
#endif
