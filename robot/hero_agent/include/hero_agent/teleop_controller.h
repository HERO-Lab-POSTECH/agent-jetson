#ifndef HERO_AGENT_TELEOP_CONTROLLER_H
#define HERO_AGENT_TELEOP_CONTROLLER_H

namespace hero {

// teleop target 누적기. teleop.cpp processKey 부호 규약을 그대로 흡수.
// 전역 target 대신 멤버로 보관 — 단일 노드 통합에서 토픽 왕복 없이 직접 호출된다.
class TeleopController {
public:
    TeleopController(double xy_step, double z_step)
        : xy_step_(xy_step), z_step_(z_step) {}

    // translated 액션 char(w/s/a/d/r/f) → target 갱신. 매핑 안 되는 char는 무시.
    // 반환: target이 바뀌었으면 true (dvl 발행 트리거용)
    bool apply(char action) {
        switch (action) {
            case 'w': x_ += xy_step_; return true;
            case 's': x_ -= xy_step_; return true;
            case 'd': y_ += xy_step_; return true;
            case 'a': y_ -= xy_step_; return true;
            case 'r': z_ -= z_step_;  return true;  // heave up (부호 규약 보존)
            case 'f': z_ += z_step_;  return true;  // heave down
            default:  return false;
        }
    }

    double x() const { return x_; }
    double y() const { return y_; }
    double z() const { return z_; }

private:
    double xy_step_, z_step_;
    double x_ = 0, y_ = 0, z_ = 0;
};

}  // namespace hero

#endif  // HERO_AGENT_TELEOP_CONTROLLER_H
