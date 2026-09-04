#ifndef HERO_AGENT_TELEOP_CONTROLLER_H
#define HERO_AGENT_TELEOP_CONTROLLER_H

namespace hero {

// teleop target 누적기. teleop.cpp processKey 부호 규약을 그대로 흡수.
// 전역 target 대신 멤버로 보관 — 단일 노드 통합에서 토픽 왕복 없이 직접 호출된다.
//
// heave(z)만 남아 있다. KEYMAP은 w/s/a/d를 translated=0으로 두고 fw_char로만
// 내보내므로(제자리 jog는 펌웨어가 직접 처리) apply()에는 'r'/'f' 외의 액션이
// 도달할 수 없었다. 그 xy 누적분은 항상 0인 채로 hero_agent_dvl.TARGET_X/Y에
// 실려 나갔다 — 그래서 발행부는 리터럴 0.0을 쓴다(값 동일).
class TeleopController {
public:
    explicit TeleopController(double z_step) : z_step_(z_step) {}

    // translated 액션 char('r'/'f') → target 갱신. 매핑 안 되는 char는 무시.
    // 반환: target이 바뀌었으면 true (dvl 발행 트리거용)
    bool apply(char action) {
        switch (action) {
            case 'r': z_ -= z_step_; return true;   // heave up (부호 규약 보존)
            case 'f': z_ += z_step_; return true;   // heave down
            default:  return false;
        }
    }

    // step 크기 재설정 (main에서 param 로드 후 전역 인스턴스 갱신용).
    // 생성자 인자가 const라 재설정이 불가하므로 setter로 노출.
    void setSteps(double z) { z_step_ = z; }

    double z() const { return z_; }

private:
    double z_step_;
    double z_ = 0;
};

}  // namespace hero

#endif  // HERO_AGENT_TELEOP_CONTROLLER_H
