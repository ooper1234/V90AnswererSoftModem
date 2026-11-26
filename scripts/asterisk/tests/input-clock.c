#include "asterisk.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "asterisk/frame.h"
#include "asterisk/utils.h"
#include "asterisk/format_cache.h"
#include "asterisk/res_audiosocket.h"
struct ast_format *ast_format_slin;
static int total, zeros, original, sends, fail_send;
const int ast_audiosocket_send_frame(const int svc,const struct ast_frame *f) {
 (void)svc; sends++;
 if(fail_send)return -1;
 assert(f->datalen==f->samples*2 && f->data.ptr);
 total+=f->samples;
 const int16_t *p=f->data.ptr;
 if(p[0]==0) {for(int i=0;i<f->samples;i++)assert(p[i]==0);zeros+=f->samples;}
 else original+=f->samples;
 return 0;
}
#include "audiosocket-modem-input-clock.h"
int main(void) {
 int16_t audio[160];for(int i=0;i<160;i++)audio[i]=1234;
 struct ast_frame f={0};f.frametype=AST_FRAME_VOICE;f.samples=160;f.datalen=320;f.data.ptr=audio;
 ast_set_flag(&f,AST_FRFLAG_HAS_TIMING_INFO);
 struct modem_input_clock c={0};
 f.ts=100;assert(!modem_send_input(1,&f,&c));
 f.ts=120;assert(!modem_send_input(1,&f,&c));
 f.ts=160;assert(!modem_send_input(1,&f,&c));
 assert(total==640 && zeros==160 && original==480);
 f.ts=220;assert(!modem_send_input(1,&f,&c));
 assert(total==1120 && zeros==480 && original==640);
 /* Empty translated interpolation preserves its sample count exactly once. */
 ast_clear_flag(&f,AST_FRFLAG_HAS_TIMING_INFO);f.datalen=0;f.data.ptr=NULL;
 assert(!modem_send_input(1,&f,&c));assert(c.next_ms==260);
 ast_set_flag(&f,AST_FRFLAG_HAS_TIMING_INFO);f.ts=260;f.datalen=320;f.data.ptr=audio;
 assert(!modem_send_input(1,&f,&c));assert(total==1440 && zeros==640);
 /* New timestamp origin / large jump and backwards time are bounded. */
 f.ts=10000;assert(!modem_send_input(1,&f,&c));assert(total==1600);
 f.ts=500;assert(!modem_send_input(1,&f,&c));assert(total==1760);
 fail_send=1;f.ts=540;assert(modem_send_input(1,&f,&c)==-1);
 fail_send=0;f.samples=9000;assert(modem_send_input(1,&f,&c)==-1);
 puts("INPUT_SAMPLE_CLOCK_REGRESSIONS_PASSED: contiguous, one/two lost packets, empty interpolation, resync, backwards timestamp, send failure, bounds");
}
