#include "../../robot/hero_agent/include/hero_agent/keymap.h"
#include <cstdio>
using namespace hero;
static int failures=0, checks=0;
static void eq(int g,int w,const char*d){checks++;if(g!=w){failures++;std::printf("FAIL[%s]:got %d want %d\n",d,g,w);} }
static void tt(bool g,const char*d){checks++;if(!g){failures++;std::printf("FAIL[%s]\n",d);} }

int main(){
    // 토글 4종: 단일 fw_char (self-toggle), is_toggle=true
    const KeyDef* k;
    k=lookup_key('1'); tt(k!=nullptr,"'1' present"); if(k){eq(k->fw_char,'R',"'1'->R"); tt(k->is_toggle,"'1' toggle"); tt(k->debounce,"'1' debounce");}
    k=lookup_key('3'); tt(k!=nullptr,"'3' present"); if(k){eq(k->fw_char,'Y',"'3'->Y"); tt(k->is_toggle,"'3' toggle");}
    k=lookup_key('4'); tt(k!=nullptr,"'4' present"); if(k){eq(k->fw_char,'D',"'4'->D"); tt(k->is_toggle,"'4' toggle");}
    k=lookup_key('5'); tt(k!=nullptr,"'5' present"); if(k){eq(k->fw_char,'L',"'5'->L"); tt(k->is_toggle,"'5' toggle");}
    // one-shot
    k=lookup_key('2'); tt(k!=nullptr,"'2' present"); if(k){eq(k->fw_char,'P',"'2'->P"); tt(!k->is_toggle,"'2' not toggle");}
    k=lookup_key('N'); tt(k!=nullptr,"'N' present"); if(k){eq(k->fw_char,'Z',"'N'->Z");}
    // jog (allow-list 등록 — 이전엔 pass-through라 테이블에 없었음)
    k=lookup_key('w'); tt(k!=nullptr,"'w' present(allow-list)"); if(k){eq(k->fw_char,'w',"'w'->w");}
    k=lookup_key('s'); tt(k!=nullptr,"'s' present"); if(k){eq(k->fw_char,'s',"'s'->s");}
    k=lookup_key('a'); tt(k!=nullptr,"'a' present"); if(k){eq(k->fw_char,'a',"'a'->a");}
    k=lookup_key('d'); tt(k!=nullptr,"'d' present"); if(k){eq(k->fw_char,'d',"'d'->d");}
    k=lookup_key('q'); tt(k!=nullptr,"'q' present"); if(k){eq(k->fw_char,'q',"'q'->q");}
    // speed: user y/h -> fw +/-
    k=lookup_key('y'); tt(k!=nullptr,"'y' present"); if(k){eq(k->fw_char,'+',"'y'->+");}
    k=lookup_key('h'); tt(k!=nullptr,"'h' present"); if(k){eq(k->fw_char,'-',"'h'->-");}
    // throttle/setpoint/gripper
    k=lookup_key('u'); tt(k!=nullptr,"'u' present"); if(k){eq(k->fw_char,'u',"'u'->u");}
    k=lookup_key('j'); tt(k!=nullptr,"'j' present"); if(k){eq(k->fw_char,'j',"'j'->j");}
    k=lookup_key('i'); tt(k!=nullptr,"'i' present"); if(k){eq(k->fw_char,'i',"'i'->i");}
    k=lookup_key('k'); tt(k!=nullptr,"'k' present"); if(k){eq(k->fw_char,'k',"'k'->k");}
    k=lookup_key('o'); tt(k!=nullptr,"'o' present"); if(k){eq(k->fw_char,'o',"'o'->o");}
    k=lookup_key('l'); tt(k!=nullptr,"'l' present"); if(k){eq(k->fw_char,'l',"'l'->l");}
    k=lookup_key('c'); tt(k!=nullptr,"'c' present"); if(k){eq(k->fw_char,'c',"'c'->c");}
    k=lookup_key('v'); tt(k!=nullptr,"'v' present"); if(k){eq(k->fw_char,'v',"'v'->v");}
    k=lookup_key('b'); tt(k!=nullptr,"'b' present"); if(k){eq(k->fw_char,'b',"'b'->b");}
    // heave: teleop char (fw_char==0, translated=='r'/'f')
    k=lookup_key('r'); tt(k!=nullptr,"'r' present"); if(k){eq(k->fw_char,0,"'r' no fw"); eq(k->translated,'r',"'r' heave");}
    k=lookup_key('f'); tt(k!=nullptr,"'f' present"); if(k){eq(k->translated,'f',"'f' heave");}
    // 미등록 키는 nullptr (allow-list drop은 translator가 처리)
    tt(lookup_key('z')==nullptr,"'z' unregistered");
    tt(lookup_key('1'+200)==nullptr,"oob unregistered");
    std::printf("\n%d checks, %d failures\n",checks,failures);
    return failures?1:0;
}
