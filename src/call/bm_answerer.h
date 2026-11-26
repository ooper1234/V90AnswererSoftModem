#ifndef BM_ANSWERER_H
#define BM_ANSWERER_H

/* The BinModem answerer (Rust, third_party/BinModem/crates/ffi), as the
   daemon sees it: one line sample at a time in and out at 8 kHz, plus a bit
   interface identical in shape to spanDSP's get_bit/put_bit -- the C side
   frames DTE bytes itself (sm_bitq) and deframes recovered bits itself
   (sm_deframer), so this layer never sees a byte.
   Status codes BM_* are shared with crates/ffi/src/lib.rs. */

#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct bm_answerer bm_answerer;

#define BM_RUNNING      0   /* negotiation or start-up still going */
#define BM_CONNECTED    1   /* in data mode */
#define BM_FAILED       2   /* terminal failure; bm_failure says why */
#define BM_AGREED_V22   3   /* V.8 chose V.22bis: caller takes over */
#define BM_AGREED_OTHER 4   /* no V.8 on the line: caller takes over */
#define BM_RETRAINING   5   /* back from data mode for a retrain */

/* Next bit to transmit (idle line is ones); called only while the pump
   accepts bits. Recovered RX bits go to put_bit from data mode onward. */
typedef int (*bm_get_bit_fn)(void *user);
typedef void (*bm_put_bit_fn)(void *user, int bit);

/* answer: nonzero for the answering side. want_v34: nonzero to offer V.34
   in the V.8 menu, alongside V.32bis and V.22bis. Legacy answering calls
   select V.32 from the opening tone or hand back to V.22bis.
   Returns NULL only on allocation failure. */
bm_answerer *bm_create(int answer, int want_v34,
                       bm_get_bit_fn get_bit, void *get_ud,
                       bm_put_bit_fn put_bit, void *put_ud);

/* The digital end of a V.90 call instead: V.8 offering the digital PCM
   category, then V.90's start-up at the line's 8 kHz. answer should be
   nonzero (V.8 pairs the answering end digital). One object carries the
   fallback ladder: digital pairing -> V.90; V.34 agreed without it, or
   negotiated V.32bis -> the 16 kHz V.32 stage; legacy V.32 opening -> V.32;
   V.22bis -> BM_AGREED_V22 for the caller to take over. On the V.90 path
   it negotiates V.42 LAPM and V.42bis compression by default; a peer without
   LAPM falls back to transparent async data. V90_COMPRESSION=0 keeps LAPM
   without compression. V90_ERROR_CONTROL=0 explicitly
   selects raw bits. Like every other entry point, one sample per bm_step. */
bm_answerer *bm_create_v90(int answer,
                           bm_get_bit_fn get_bit, void *get_ud,
                           bm_put_bit_fn put_bit, void *put_ud);

/* V.23 transparent split-speed FSK. reversed=1 swaps the 1200/75 channels. */
bm_answerer *bm_create_v23(int answer, int reversed,
                         bm_get_bit_fn get_bit, void *get_ud,
                         bm_put_bit_fn put_bit, void *put_ud);
/* Transparent Bell 103: 300 bit/s in both directions. */
bm_answerer *bm_create_bell103(int answer,
                         bm_get_bit_fn get_bit, void *get_ud,
                         bm_put_bit_fn put_bit, void *put_ud);
/* Transparent V.21: 300 bit/s in both directions on V.21's channels. */
bm_answerer *bm_create_v21(int answer,
                         bm_get_bit_fn get_bit, void *get_ud,
                         bm_put_bit_fn put_bit, void *put_ud);
/* U.S. Robotics HST (MNP inside), async characters both ways. The answerer
   holds the forward channel from the start. */
bm_answerer *bm_create_hst(int answer,
                         bm_get_bit_fn get_bit, void *get_ud,
                         bm_put_bit_fn put_bit, void *put_ud);
/* Bell 103 / V.23: hand line bits up at the recovered bit clock rather than
   as re-framed characters, for a V.42 link (bm_ec_link) run on top. */
void bm_set_raw_line_bits(bm_answerer *a, int on);
/* Dedicated V.32 (max_rate=9600) or V.32bis (max_rate=14400), with V.25
   answering tone, V.42 detection and negotiated V.42bis compression. */
bm_answerer *bm_create_v32(int answer, int max_rate,
                         bm_get_bit_fn get_bit, void *get_ud,
                         bm_put_bit_fn put_bit, void *put_ud);

/* One 8 kHz line sample in, the corresponding outgoing sample out (s16).
   Deterministic: exactly one out per in, after a few milliseconds of
   resampler priming at the start. */
int bm_step(bm_answerer *a, int sample);

/* Move data bits: top the transmitter up from get_bit and hand recovered
   bits to put_bit. Call once per audio chunk, before bm_step. */
void bm_service(bm_answerer *a);

/* Throw away what the receiver made of the handshake. Call once, at the
   moment BM_CONNECTED is first observed, before bm_service runs again. */
void bm_flush_rx(bm_answerer *a);

int bm_status(bm_answerer *a);
int bm_pending(bm_answerer *a);     /* bits queued in the engine's transmitter */
int bm_tx_delivered(bm_answerer *a); /* 1: far modem has acknowledged all data (always 1 without V.42) */
unsigned long long bm_rx_total(bm_answerer *a); /* bits the receiver handed up */
int bm_found_again(bm_answerer *a);  /* decoder re-acquisitions */
int bm_slips(bm_answerer *a);        /* sample slips seen by the receiver */
unsigned long long bm_underruns(bm_answerer *a); /* dry transmit queue */
unsigned long long bm_clips(bm_answerer *a);     /* engine output past full scale */
double bm_peak(bm_answerer *a);                  /* largest |output| seen */
/* Boundary echo canceller: locked delay in samples (0 = never locked),
   correlation peak of the lock, filter energy (0 = nothing learned). */
void bm_echo(bm_answerer *a, int *delay, double *peak, double *energy);
unsigned long long bm_up_odd(bm_answerer *a, unsigned char *last);
int bm_rate_tx(bm_answerer *a);     /* bit/s, valid once BM_CONNECTED */
int bm_rate_rx(bm_answerer *a);
int bm_carrier(bm_answerer *a);     /* far end's data signal present */
int bm_error_control(bm_answerer *a); /* V.42 LAPM established */
int bm_compression(bm_answerer *a);   /* 0 none, 1 V.42bis, 2 V.44 */
unsigned long long bm_damaged_frames(bm_answerer *a);
/* plain sent, wire sent, wire received, plain received; 0 if none */
int bm_compression_counts(bm_answerer *a, unsigned long long out[4]);
int bm_ec_peer_released(bm_answerer *a);   /* far modem sent DISC (ATH) */
int bm_ec_phase(bm_answerer *a);
int bm_lapm_declared(bm_answerer *a);
int bm_ec_observed(bm_answerer *a);

/* V.42 for a pump outside the engine (the C V.22bis): the pump trades line
   bits with the link, and the link trades an async DTE bit stream through
   get_bit/put_bit as the pump did. Detection runs unless lapm_declared, so a
   caller without V.42 falls back to a transparent async link. */
typedef struct EcLink bm_ec_link;
bm_ec_link *bm_ec_create(int answer, int lapm_declared, int line_bps, int round_trip_ms,
                         bm_get_bit_fn get_bit, void *get_user,
                         bm_put_bit_fn put_bit, void *put_user);
void bm_ec_rx_bit(bm_ec_link *l, int bit);   /* a bit from the line */
int bm_ec_tx_bit(bm_ec_link *l);             /* the next bit for the line */
void bm_ec_tick(bm_ec_link *l, int ms);
int bm_ec_link_peer_released(bm_ec_link *l); /* far modem sent DISC (ATH) */
int bm_ec_link_phase(bm_ec_link *l);         /* 0 detect, 1 XID, 2 LAPM, 3 transparent */
int bm_ec_link_compression(bm_ec_link *l);   /* 0 none, 1 V.42bis, 2 V.44 */
unsigned long long bm_ec_link_damaged_frames(bm_ec_link *l);
void bm_ec_destroy(bm_ec_link *l);

/* Human-readable start-up phase / failure reason. Pointers are into `a`
   and valid until the next call on it. */
const char *bm_phase(bm_answerer *a);
const char *bm_failure(bm_answerer *a);

void bm_destroy(bm_answerer *a);

#ifdef __cplusplus
}
#endif

#endif /* BM_ANSWERER_H */
