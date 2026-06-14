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
static const KeyDef KEYMAP[] = {
    // System
    // 키1은 key_input_callback이 가로채 relay 상태에 따라 'e'/'t'를 직접 발행한다
    // (칩 2f6725d는 self-toggle 안 함). 따라서 fw_char는 KEYMAP을 통해 발행되지 않는
    // 죽은 값 → 0으로 표기(heave 행과 동일 컨벤션). 행은 HELP 라벨/그룹 보존용으로 유지.
    {'1', 0,   0, true,  true,  GRP_SYSTEM,  "Relay"},
    {'2', 'g', 0, false, true,  GRP_SYSTEM,  "PWM Neutral"},
    {'N', 'n', 0, false, false, GRP_SYSTEM,  "Yaw Reset"},
    // Control toggles — 칩(2f6725d)이 ON/OFF 별도 char라 key_input_callback이 가로채
    // 현재 상태 보고 발행한다(yaw 'y'/'h', depth 'p'/';', laser 'r'/'f'). fw_char는
    // KEYMAP 경유 발행 안 되는 죽은값 → 0(키1 relay 컨벤션 동일). 행은 HELP/그룹 보존용.
    {'3', 0,   0, true,  true,  GRP_CONTROL, "Yaw Ctrl"},
    {'4', 0,   0, true,  true,  GRP_CONTROL, "Depth Ctrl"},
    {'5', 0,   0, true,  true,  GRP_CONTROL, "Laser"},
    // Thruster jog
    {'w', 'w', 0, false, false, GRP_JOG,     "Forward"},
    {'s', 's', 0, false, false, GRP_JOG,     "Backward"},
    {'a', 'a', 0, false, false, GRP_JOG,     "Left"},
    {'d', 'd', 0, false, false, GRP_JOG,     "Right"},
    {'q', 'q', 0, false, false, GRP_JOG,     "Stop"},
    // Speed (user y/h -> fw +/-)
    {'y', 'z', 0, false, false, GRP_SPEED,   "Speed+"},
    {'h', 'x', 0, false, false, GRP_SPEED,   "Speed-"},
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
