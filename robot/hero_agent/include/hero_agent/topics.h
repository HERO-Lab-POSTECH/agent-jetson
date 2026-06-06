#ifndef HERO_AGENT_TOPICS_H
#define HERO_AGENT_TOPICS_H

// 토픽명 단일 진실 공급원(SSOT). raw 리터럴 중복으로 인한 오타-체인단절 방지.
// KEY_TRANSLATED 는 단일 노드 통합에서 제거되므로 의도적으로 넣지 않는다.
namespace hero {
namespace topics {

constexpr char COMMAND[]     = "/hero_agent/command";
constexpr char KEY_INPUT[]   = "/hero_agent/key_input";
constexpr char DVL[]         = "/hero_agent/dvl";
constexpr char STATE[]        = "/hero_agent/state";
constexpr char SENSORS[]     = "/hero_agent/sensors";
constexpr char ALBC_STATUS[] = "/albc_status";

}  // namespace topics
}  // namespace hero

#endif  // HERO_AGENT_TOPICS_H
