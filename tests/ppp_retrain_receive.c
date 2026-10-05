#define SM_HAVE_BM
#include "../src/call/sm_call.c"
#include <assert.h>
#include <sys/socket.h>
static int lapm_connected;
int bm_error_control(bm_answerer *a) { (void)a; return lapm_connected; }
static void byte(sm_call_t *c, unsigned char value) {
    call_put_bit(c, 0);
    for (int i=0; i<8; i++) call_put_bit(c, (value>>i)&1);
    call_put_bit(c, 1);
}
int main(void) {
    sm_call_t *c=calloc(1,sizeof(*c)); unsigned char got[2]; int fd[2];
    assert(c); c->bm=(bm_answerer *)1; c->ppp_started=1;
    sm_deframer_init(&c->deframer); c->phase=SM_CALL_DATA;
    lapm_connected=1; byte(c,0x35);
    c->phase=SM_CALL_HANDSHAKE; reset_unvalidated_rx(c); byte(c,0xa6);
    reset_unvalidated_rx(c);
    assert(sm_deframer_take(&c->deframer,got,2)==2);
    assert(got[0]==0x35 && got[1]==0xa6);
    lapm_connected=0; byte(c,0x99);
    assert(sm_deframer_take(&c->deframer,got,2)==0);
    lapm_connected=1; c->ppp_started=0; byte(c,0x77);
    assert(sm_deframer_take(&c->deframer,got,2)==0);
    /* Delivery must continue during the physical handshake, not just buffer
       already acknowledged bytes until a later data-mode transition. */
    assert(socketpair(AF_UNIX,SOCK_STREAM,0,fd)==0);
    assert(fcntl(fd[0],F_SETFL,O_NONBLOCK)==0);
    assert(fcntl(fd[1],F_SETFL,O_NONBLOCK)==0);
    c->ppp_started=1; c->ppp_fd=fd[0];
    byte(c,0x52); byte(c,0xe1);
    pump_ppp(c);
    assert(read(fd[1],got,2)==2);
    assert(got[0]==0x52 && got[1]==0xe1);
    assert(c->deframer.out_len==0 && c->ppy_out_len==0);
    /* A blocked relay keeps validated bytes queued for a later drain. */
    unsigned char fill[1024]={0};
    while(write(fd[0],fill,sizeof(fill))>0) {}
    assert(errno==EAGAIN || errno==EWOULDBLOCK);
    byte(c,0x68); pump_ppp(c);
    assert(c->ppy_out_len==1 && c->ppy_out[0]==0x68);
    while(read(fd[1],fill,sizeof(fill))>0) {}
    pump_ppp(c);
    assert(read(fd[1],got,2)==1 && got[0]==0x68);
    assert(c->ppy_out_len==0);
    /* A full relay queue must leave newly validated bytes in the deframer. */
    while(write(fd[0],fill,sizeof(fill))>0) {}
    memset(c->ppy_out,0x4d,sizeof(c->ppy_out));
    c->ppy_out_len=sizeof(c->ppy_out);
    byte(c,0x7a); pump_ppp(c);
    assert(c->ppy_out_len==sizeof(c->ppy_out));
    assert(c->deframer.out_len==1 && c->deframer.out[0]==0x7a);
    while(read(fd[1],fill,sizeof(fill))>0) {}
    pump_ppp(c);
    unsigned char drained[4097]; int total=0; ssize_t count;
    while((count=read(fd[1],drained+total,sizeof(drained)-total))>0) total+=count;
    assert(total==sizeof(drained));
    for(int i=0;i<4096;i++) assert(drained[i]==0x4d);
    assert(drained[4096]==0x7a);
    assert(c->deframer.out_len==0 && c->ppy_out_len==0);
    close(fd[0]); close(fd[1]);
    free(c); puts("PASS: validated LAPM bytes survive retrain; raw and pre-PPP bits are gated");
}
