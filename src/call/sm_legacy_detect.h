#ifndef SM_LEGACY_DETECT_H
#define SM_LEGACY_DETECT_H
#include <math.h>
#include <string.h>

/* Caller-channel startup detection at 8 kHz. Never classify the similar
   2225/2250 Hz answering signals as a caller response. Forty-ms windows
   separate Bell's 1270 Hz from V.22's 1200 Hz and V.23's 1300 Hz. Require
   200 ms of dominant mark, not a single FFT bin or a broadband carrier.

   Bell's mark is judged against the caller's band only (bins across
   900-1500 Hz), not the whole line: from about 0.9 s on the line also
   carries the echo of our own V.22bis answer signal, which is often louder
   than the caller and used to hide the mark until the V.22bis receiver had
   mistaken it for a V.22 caller.

   V.21's caller idles at its channel 1 mark, 980 Hz (space 1180 Hz), once it
   hears our 1650 Hz answer mark; it is told from Bell's 1270 and from
   V.22's 1200 the same way, by a steady tone clear of the others.

   HST's caller answers the tone on its back channel: 8-PSK at 150 baud on
   375 Hz, which spreads across 300-450 Hz. Most of the energy there, spread
   over several bins with none standing alone, so mains hum's harmonics are
   not taken for it. */
#define SM_LD_BINS 15
#define SM_LD_HST0 8 /* 300, 325, ... 450 Hz */
typedef struct {
    double coeff[SM_LD_BINS], q1[SM_LD_BINS], q2[SM_LD_BINS], energy;
    int count, bell_held, v22_held, v21_held, hst_held;
} sm_legacy_detect_t;

static inline void sm_legacy_detect_init(sm_legacy_detect_t *d)
{
    static const double f[SM_LD_BINS] = {1270, 1200, 1300, 1070, 900, 1500, 980, 1180,
                                        300, 325, 350, 375, 400, 425, 450};
    memset(d, 0, sizeof(*d));
    for (int j=0;j<SM_LD_BINS;j++) d->coeff[j]=2*cos(2*3.14159265358979323846*f[j]/8000);
}

/* 1: Bell 103 idle mark; 2: V.22 low-channel startup; 3: V.21 channel 1
   idle mark; 4: HST back channel; 0: unqualified. */
static inline int sm_legacy_detect_step(sm_legacy_detect_t *d, double x)
{
    d->energy+=x*x;
    for (int j=0;j<SM_LD_BINS;j++) {
        double q=x+d->coeff[j]*d->q1[j]-d->q2[j];
        d->q2[j]=d->q1[j];d->q1[j]=q;
    }
    if (++d->count < 320) return 0;
    double p[SM_LD_BINS];
    for (int j=0;j<SM_LD_BINS;j++)
        p[j]=2*(d->q1[j]*d->q1[j]+d->q2[j]*d->q2[j]-d->coeff[j]*d->q1[j]*d->q2[j])/320;
    int audible=d->energy>320*0.000016;
    /* A tone at 1270 Hz standing well clear of everything else in the
       caller's band; a V.22 caller's PSK spreads across all of these bins. */
    int bell=p[0]>320*0.000016*.65 && p[0]>4*p[1] && p[0]>4*p[2]
             && p[0]>8*p[3] && p[0]>8*p[4] && p[0]>8*p[5];
    int v22=audible && p[1]>.65*d->energy && p[1]>4*p[0] && p[1]>4*p[2];
    int v21=p[6]>320*0.000016*.65 && p[6]>4*p[7] && p[6]>4*p[3]
            && p[6]>8*p[0] && p[6]>8*p[1] && p[6]>8*p[2] && p[6]>8*p[4] && p[6]>8*p[5];
    d->bell_held=bell ? d->bell_held+1 : 0;
    d->v22_held=v22 ? d->v22_held+1 : 0;
    double low=0,top=0;int wide=0;
    for (int j=SM_LD_HST0;j<SM_LD_BINS;j++) {
        low+=p[j]; if (p[j]>top) top=p[j]; if (p[j]>=.08*d->energy) wide++;
    }
    int hst=audible && low>.5*d->energy && top<.6*d->energy && wide>=3;
    d->v21_held=v21 ? d->v21_held+1 : 0;
    /* One window in a few reads narrower: let it slip back, not restart. */
    d->hst_held=hst ? d->hst_held+1 : d->hst_held>0 ? d->hst_held-1 : 0;
    int result=d->bell_held>=5 ? 1 : d->v22_held>=5 ? 2 : d->v21_held>=5 ? 3
              : d->hst_held>=6 ? 4 : 0;
    memset(d->q1,0,sizeof(d->q1));memset(d->q2,0,sizeof(d->q2));
    d->count=0;d->energy=0;
    return result;
}
#endif
