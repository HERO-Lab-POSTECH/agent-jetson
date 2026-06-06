#ifndef HERO_AGENT_KEYMAP_H
#define HERO_AGENT_KEYMAP_H

namespace hero {

// 어느 ToggleState 비트를 보는지
enum class ToggleBit { NONE, RELAY, YAW, DEPTH, LASER };

// Arduino가 /hero_agent/state control_flags로 보고한 하드웨어 토글 상태
struct ToggleState {
    int relay_enabled;          // bit 2
    int control_yaw_enabled;    // bit 0
    int control_depth_enabled;  // bit 1
    int laser_enabled;          // bit 3
};

struct KeyDef {
    int         key;          // 입력 키
    bool        toggle;       // true=토글키(cmd_off/cmd_on+toggle_bit 사용)
    char        cmd_off;      // toggle=false면 고정 cmd; toggle=true면 OFF 상태 cmd
    char        cmd_on;       // toggle=true일 때 ON 상태 cmd (아니면 0)
    char        translated;   // jetson 액션 char (0=없음)
    bool        debounce;     // 500ms 게이트
    ToggleBit   toggle_bit;   // toggle=true일 때 볼 비트
    const char* label;        // 모니터/HELP 자동생성용
};

// 명시 키만 테이블에. 차단키/pass-through는 테이블에 없음(lookup_key가 nullptr 반환).
constexpr KeyDef KEYMAP[] = {
    // 토글키
    {'1', true,  'e', 't', 0,   true,  ToggleBit::RELAY, "Relay"},
    {'2', true,  'y', 'h', 0,   true,  ToggleBit::YAW,   "Yaw"},
    {'3', true,  'p', ';', 0,   true,  ToggleBit::DEPTH, "Depth"},
    {'5', true,  'r', 'f', 0,   true,  ToggleBit::LASER, "Laser"},
    // 고정 cmd키
    {'4', false, 'g',  0,  0,   true,  ToggleBit::NONE,  "PWM Init"},
    {'N', false, 'n',  0,  0,   false, ToggleBit::NONE,  "Yaw Reset"},
    {'o', false, 'o',  0,  0,   false, ToggleBit::NONE,  "Depth-0.1"},
    // 고정 translated키
    {'r', false,  0,   0, 'r',  false, ToggleBit::NONE,  "Heave Up"},
    {'f', false,  0,   0, 'f',  false, ToggleBit::NONE,  "Heave Down"},
};

constexpr int KEYMAP_SIZE = sizeof(KEYMAP) / sizeof(KEYMAP[0]);

// 테이블 룩업: 명시 키면 그 KeyDef*, 아니면 nullptr(=차단 또는 pass-through는 호출자 판단)
inline const KeyDef* lookup_key(int ch) {
    for (int i = 0; i < KEYMAP_SIZE; ++i)
        if (KEYMAP[i].key == ch) return &KEYMAP[i];
    return nullptr;
}

// 차단키 집합(테이블엔 없지만 pass-through도 아닌, 명시적 no-op 키)
inline bool is_blocked_key(int ch) {
    switch (ch) {
    case 'p':
    case ';': case 'n': case 'm': case '.': case ',':
    case 't': case 'g': case 'y': case 'h': case 'e': case 'q':
    case '6': case '7': case '8': case '9': case '0':
    case 'R':
        return true;
    default:
        return false;
    }
}

}  // namespace hero

#endif  // HERO_AGENT_KEYMAP_H
