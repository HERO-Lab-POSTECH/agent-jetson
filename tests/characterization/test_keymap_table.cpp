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
    k=lookup_key('3'); if(k){eq(k->fw_char,'Y',"'3'->Y"); tt(k->is_toggle,"'3' toggle");}
    k=lookup_key('4'); if(k){eq(k->fw_char,'D',"'4'->D"); tt(k->is_toggle,"'4' toggle");}
    k=lookup_key('5'); if(k){eq(k->fw_char,'L',"'5'->L"); tt(k->is_toggle,"'5' toggle");}
    // one-shot
    k=lookup_key('2'); if(k){eq(k->fw_char,'P',"'2'->P"); tt(!k->is_toggle,"'2' not toggle");}
    k=lookup_key('N'); if(k){eq(k->fw_char,'Z',"'N'->Z");}
    // jog (allow-list 등록 — 이전엔 pass-through라 테이블에 없었음)
    k=lookup_key('w'); tt(k!=nullptr,"'w' present(allow-list)"); if(k){eq(k->fw_char,'w',"'w'->w");}
    k=lookup_key('s'); if(k){eq(k->fw_char,'s',"'s'->s");}
    k=lookup_key('a'); if(k){eq(k->fw_char,'a',"'a'->a");}
    k=lookup_key('d'); if(k){eq(k->fw_char,'d',"'d'->d");}
    k=lookup_key('q'); if(k){eq(k->fw_char,'q',"'q'->q");}
    // speed: user y/h -> fw +/-
    k=lookup_key('y'); if(k){eq(k->fw_char,'+',"'y'->+");}
    k=lookup_key('h'); if(k){eq(k->fw_char,'-',"'h'->-");}
    // throttle/setpoint/gripper
    k=lookup_key('u'); if(k){eq(k->fw_char,'u',"'u'->u");}
    k=lookup_key('j'); if(k){eq(k->fw_char,'j',"'j'->j");}
    k=lookup_key('i'); if(k){eq(k->fw_char,'i',"'i'->i");}
    k=lookup_key('k'); if(k){eq(k->fw_char,'k',"'k'->k");}
    k=lookup_key('o'); if(k){eq(k->fw_char,'o',"'o'->o");}
    k=lookup_key('l'); if(k){eq(k->fw_char,'l',"'l'->l");}
    k=lookup_key('c'); if(k){eq(k->fw_char,'c',"'c'->c");}
    k=lookup_key('v'); if(k){eq(k->fw_char,'v',"'v'->v");}
    k=lookup_key('b'); if(k){eq(k->fw_char,'b',"'b'->b");}
    // heave: teleop char (fw_char==0, translated=='r'/'f')
    k=lookup_key('r'); if(k){eq(k->fw_char,0,"'r' no fw"); eq(k->translated,'r',"'r' heave");}
    k=lookup_key('f'); if(k){eq(k->translated,'f',"'f' heave");}
    // 미등록 키는 nullptr (allow-list drop은 translator가 처리)
    tt(lookup_key('z')==nullptr,"'z' unregistered");
    tt(lookup_key('1'+200)==nullptr,"oob unregistered");
    std::printf("\n%d checks, %d failures\n",checks,failures);
    return failures?1:0;
}
