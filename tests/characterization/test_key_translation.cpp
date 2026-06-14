// 안전 속성 고정: 미등록 키는 firmware에 안 감(drop), 등록키만 allow-list 번역. self-toggle이라 state 불요.
#include "../../robot/hero_agent/include/hero_agent/key_translator.h"
#include <cstdio>
using namespace hero;
static int failures=0,checks=0;
static void eq(int g,int w,const char*d){checks++;if(g!=w){failures++;std::printf("FAIL[%s]:got %d want %d\n",d,g,w);} }
int main(){
    KeyXlate o;
    // 토글: fw_char 그대로 발행 (state 안 봄)
    o=translate_key('1'); eq(o.cmd,'R',"'1'->cmd R"); eq(o.translated,0,"'1' no teleop");
    o=translate_key('3'); eq(o.cmd,'Y',"'3'->Y"); eq(o.translated,0,"'3' no teleop");
    // jog (이전 pass-through, 이제 allow-list 등록 — 동일 char)
    o=translate_key('w'); eq(o.cmd,'w',"'w'->w"); eq(o.translated,0,"'w' no teleop");
    o=translate_key('a'); eq(o.cmd,'a',"'a'->a");
    // speed remap
    o=translate_key('y'); eq(o.cmd,'+',"'y'->+");
    o=translate_key('h'); eq(o.cmd,'-',"'h'->-");
    // heave: teleop only, no fw cmd
    o=translate_key('r'); eq(o.cmd,0,"'r' no fw"); eq(o.translated,'r',"'r' teleop");
    o=translate_key('f'); eq(o.cmd,0,"'f' no fw"); eq(o.translated,'f',"'f' teleop");
    // 미등록 키 = drop (cmd=0, translated=0)
    o=translate_key('z'); eq(o.cmd,0,"'z' drop cmd"); eq(o.translated,0,"'z' drop tel");
    o=translate_key('5'+100); eq(o.cmd,0,"oob drop");
    std::printf("\n%d checks, %d failures\n",checks,failures);
    return failures?1:0;
}
