#include <assert.h>
#include "call/sm_call.c"

static int classify(double frequency, double noise, int length)
{
    sm_legacy_detect_t d;sm_legacy_detect_init(&d);
    unsigned rng=42;int result=0;
    for (int i=0;i<length;i++) {
        rng=rng*1664525u+1013904223u;
        double x=.12*sin(2*SM_PI*frequency*i/8000)+noise*((rng>>16)/32768.0-1);
        int r=sm_legacy_detect_step(&d,x);if(r) result=r;
    }
    return result;
}
static void init_legacy(sm_call_t *c)
{
    sm_call_config_t cfg={0};cfg.rate=2400;cfg.use_binmodem=1;cfg.use_v90=1;cfg.echo_data=1;
    sm_call_init(c,&cfg,1);
    bm_destroy(c->bm);c->bm=NULL;c->phase=SM_CALL_HANDSHAKE;
    v22bis_restart(&c->modem);c->legacy_probe=1;
    sm_legacy_detect_init(&c->legacy_detector);
}
static void feed_silence(sm_call_t *c,int chunks)
{
    int16_t in[160]={0};
    for(int i=0;i<chunks;i++) process_audio(c,in,160);
}
/* 0: V.23, 1: Bell 103, 2: V.21. */
static void duplex(int mode)
{
    int bell=mode==1, fsk300=mode!=0;
    sm_call_t c;init_legacy(&c);
    if(mode==1) {
        feed_silence(&c,250);assert(c.legacy_probe==2);
    } else if(mode==2) {
        feed_silence(&c,410);assert(c.legacy_probe==3);
    } else {
        feed_silence(&c,560);assert(c.bm && strstr(bm_phase(c.bm),"V.23"));
    }
    bm_answerer *peer=mode==1 ? bm_create_bell103(0,NULL,NULL,NULL,NULL)
                    : mode==2 ? bm_create_v21(0,NULL,NULL,NULL,NULL)
                    : bm_create_v23(0,0,NULL,NULL,NULL,NULL);
    int16_t in[160]={0};
    for(int i=0;i<200;i++) {
        process_audio(&c,in,160);
        bm_service(peer);
        for(int k=0;k<160;k++) in[k]=bm_step(peer,c.txbuf[k]);
        if(c.phase==SM_CALL_DATA && bm_status(peer)==BM_CONNECTED) break;
    }
    assert(c.bm && c.phase==SM_CALL_DATA && bm_status(peer)==BM_CONNECTED);
    assert(bm_rate_tx(c.bm)==(fsk300?300:1200));
    assert(bm_rate_rx(c.bm)==(fsk300?300:75));
    assert(strstr(bm_phase(c.bm),mode==1?"Bell 103":mode==2?"V.21":"V.23"));
    (void)bell;
    assert(!bm_error_control(c.bm));
    bm_destroy(peer);bm_destroy(c.bm);
}
/* An HST caller from off hook: it hears ANSam out as its answer tone, starts
   its back channel when the tone ends, and the legacy probe takes it. */
static void hst_call(void)
{
    sm_call_t c;sm_call_config_t cfg={0};cfg.rate=2400;cfg.use_binmodem=1;cfg.use_v90=1;cfg.echo_data=1;
    sm_call_init(&c,&cfg,1);
    bm_answerer *peer=bm_create_hst(0,NULL,NULL,NULL,NULL);
    int16_t in[160]={0};
    int i;
    for(i=0;i<1500;i++) {
        process_audio(&c,in,160);
        bm_service(peer);
        for(int k=0;k<160;k++) in[k]=bm_step(peer,c.txbuf[k]);
        if(c.phase==SM_CALL_DATA && bm_status(peer)==BM_CONNECTED) break;
    }
    assert(c.bm && strstr(bm_phase(c.bm),"HST"));
    assert(c.phase==SM_CALL_DATA && bm_status(peer)==BM_CONNECTED);
    printf("HST: connected after %.2f s at %d/%d\n",i*0.02,bm_rate_tx(c.bm),bm_rate_rx(c.bm));
    assert(bm_rate_tx(c.bm)>=7200 && (bm_rate_rx(c.bm)==450 || bm_rate_rx(c.bm)==300));
    bm_destroy(peer);bm_destroy(c.bm);
}
static void preserve_v22(void)
{
    sm_call_t c;init_legacy(&c);
    int16_t in[160];
    for(int i=0;i<600;i++) {
        for(int k=0;k<160;k++) in[k]=(int16_t)(4000*sin(2*SM_PI*1200*(i*160+k)/8000));
        process_audio(&c,in,160);
    }
    assert(c.bm==NULL && c.legacy_v22_seen && c.legacy_probe!=2);
}
static void real_v22_startup(int rate)
{
    sm_call_t c;init_legacy(&c);
    v22bis_state_t peer;
    v22bis_init(&peer,true,rate,NULL,NULL,NULL,NULL,NULL,NULL);
    int16_t in[160]={0};
    for(int i=0;i<500;i++) {
        process_audio(&c,in,160);
        v22bis_rx(&peer,c.txbuf,160);v22bis_tx(&peer,in,160);
        if(c.phase==SM_CALL_DATA) break;
    }
    assert(c.phase==SM_CALL_DATA && !c.bm && !c.legacy_probe);
    assert(c.negotiated_rate==rate);
}
static void full_fallback_without_caller(void)
{
    sm_call_t c;sm_call_config_t cfg={0};cfg.rate=2400;cfg.use_binmodem=1;cfg.use_v90=1;cfg.echo_data=1;
    sm_call_init(&c,&cfg,1);
    feed_silence(&c,1500);
    assert(c.bm && strstr(bm_phase(c.bm),"V.23"));
    assert(c.phase!=SM_CALL_DATA && bm_status(c.bm)!=BM_CONNECTED);
    bm_destroy(c.bm);
}
int main(void)
{
    assert(classify(1270,.025,8000)==1);
    assert(classify(1262,.015,8000)==1);
    assert(classify(1278,.015,8000)==1);
    assert(classify(1200,.025,8000)==2);
    assert(classify(980,.025,8000)==3);
    assert(classify(972,.015,8000)==3);
    assert(classify(988,.015,8000)==3);
    double rejected[]={0,300,360,375,420,450,1070,1180,1300,1650,1800,1850,2100,2225,2250,2400};
    for(unsigned j=0;j<sizeof(rejected)/sizeof(*rejected);j++) assert(classify(rejected[j],.02,8000)==0);
    assert(classify(1270,0,1280)==0); /* A short tone is not enough. */
    preserve_v22();real_v22_startup(1200);real_v22_startup(2400);
    full_fallback_without_caller();duplex(1);duplex(2);duplex(0);hst_call();
    puts("legacy startup: tone rejection, V.22 retention, Bell 103, V.21, HST and V.23 handoffs passed");
}
