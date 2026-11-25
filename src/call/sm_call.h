#ifndef SM_CALL_H
#define SM_CALL_H

#include "sm_common.h"
#include "sm_log.h"
#include "modem/v22bis/v22bis.h"
#include "serial/sm_async.h"
#include "ast_socket/sm_ast_socket.h"
#include "sm_legacy_detect.h"

#include <sys/types.h>
#include <stdio.h>

#ifdef SM_HAVE_V8
#include "modem/v8/sm_v8.h"
#endif

/* One dial-up call: AudioSocket PCM <-> V.22bis answerer <-> async serial
   <-> pty <-> pppd. */
typedef enum {
    SM_CALL_MODE_V22 = 0,
    SM_CALL_MODE_V34
} sm_call_mode_t;

typedef enum {
    SM_CALL_IDLE = 0,
    SM_CALL_ANSWER_TONE,      /* sending 2100 Hz ANS */
    SM_CALL_V8,               /* V.8 negotiation */
    SM_CALL_HANDSHAKE,        /* V.22bis training running */
    SM_CALL_DATA,             /* both directions in NORMAL_OPERATION */
    SM_CALL_HANGUP
} sm_call_phase_t;

typedef struct {
    /* configuration */
    int rate;                       /* 1200 or 2400 */
    int answer_tone_ms;             /* 2100 Hz answer tone duration (0=off) */
    int pre_tone_silence_ms;        /* silence before answer tone */
    const char *pppd_path;
    const char *shim_exe;
    const char *local_ip;
    const char *peer_ip;
    const char *dns1;
    const char *dns2;
    const char *log_dir;
    const char *ip_up_script;
    const char *ip_down_script;
    const char *bbs_db;
    int auth;
    int enable_ppp;
    int echo_data;                  /* loop data back instead of using pppd */
    int use_v8;                     /* run V.8 negotiation first */
    int v23_mode;                  /* 0=off, 1=1200/75, 2=75/1200 */
    int v32_max_rate;               /* dedicated V.32: 9600; V.32bis: 14400 */
    int use_v34;                    /* offer V.34 in V.8 (opt-in; the V.34
                                       receive data path is not finished yet) */
    int use_binmodem;               /* answer with the vendored BinModem
                                       engine (V.8 + V.34 in one object) */
    int use_v90;                    /* BinModem V.90 mode: V.8 offering the
                                       digital PCM category, V.90 when the
                                       far end pairs, V.34 fallback below
                                       that (implies use_binmodem) */
    int v34_baud;                   /* V.34 symbol rate for the engine */
    int v34_rate;                   /* V.34 maximum bit rate for the engine */
    sm_log_level_t log_level;
} sm_call_config_t;

typedef struct {
    sm_call_config_t cfg;
    int call_id;

    sm_ast_socket_t as;
    FILE *cap_in;                   /* raw PCM capture of the far end (V34_CAPTURE) */
    FILE *cap_out;                  /* raw PCM capture of our TX */
    long cap_count;                 /* samples captured per direction */
    sm_call_mode_t mode;
    v22bis_state_t modem;           /* used when mode is SM_CALL_MODE_V22 */
    void *v34;                      /* v34_state_t *, when mode is SM_CALL_MODE_V34 */
    int v34_baud_rate;              /* negotiated V.34 symbol rate (for logs) */

    /* BinModem answerer (bm_answerer_t *), when cfg.use_binmodem. NULL at
       other times and once the engine has handed a V.22bis call back. */
    void *bm;
    /* V.42 over the C V.22bis (bm_ec_link *), from V.22bis data mode on;
       what V.8 said about LAPM and the echo's round trip are kept from the
       engine for it when it hands the call back. */
    void *v22_ec;
    int v22_lapm_declared;
    int v22_round_trip_ms;
    int v22_rx_guard;               /* samples of V.22bis RX squelch after a V.8 handover */
    int v22_ec_seen;                /* last V.42 phase logged, -1 none */
    int v22_tick_rem;               /* samples not yet counted as a ms */
    void *bbs;                     /* bbs_session_t while selecting/BBS */
    char bm_phase_seen[96];         /* last phase string logged */
    int bm_ec_seen;                 /* last V.42 phase logged */
    long bm_nocarrier;              /* samples since the far end's carrier */
    sm_legacy_detect_t legacy_detector;
    int legacy_probe;              /* 1: V.22, 2: Bell mark, 3: V.21 mark, 0: disabled */
    long legacy_elapsed;
    int legacy_v22_seen;

    sm_bitq_t txbits;               /* pppd bytes -> modem TX bits */
    int tx_lead_mark;               /* idle mark bits still owed before TX data */
    sm_deframer_t deframer;         /* modem RX bits -> pppd bytes */

    /* answer tone generator */
    int tone_samples_left;
    int silence_samples_left;
    double tone_phase;
    double tone_phase_inc;
    double tone_amplitude;

    /* pppd (started only after the login menu picks PPP) */
    int ppp_fd;
    pid_t pppd_pid;
    int ppp_started;
    /* The server side (pppd, or the BBS behind the login screen) closed the
       pty: nothing more can flow, so the call is hung up once its last
       bytes are down the line (sm_call_server_closed_step). */
    int ppp_closed;
    long long ppp_closed_samples;   /* since the close */
    long long ppp_drained_samples;  /* since everything queued was sent */

#ifdef SM_HAVE_V8
    sm_v8_t *v8;
    int v8_done;
    int v8_ok;
    int rx_guard;                   /* samples to ignore after V.8 (FSK tail) */
#endif

    /* state */
    sm_call_phase_t phase;
    int rx_normal;
    int tx_normal;
    int negotiated_rate;
    long long samples_in;
    long long samples_out;
    long long data_bytes_rx;
    long long data_bytes_tx;
    sm_log_t log;

    /* RX byte assembly for the pty */
    uint8_t ppy_out[4096];
    int ppy_out_len;

    /* per-frame TX scratch (max AudioSocket payload / 2 samples) */
    int16_t txbuf[SM_AS_MAX_PAYLOAD / 2];
    int16_t rxsquelch[SM_AS_MAX_PAYLOAD / 2];
} sm_call_t;

/* After the server side closed: count `n` more samples, `drained` when no
   bytes are left to send. Returns 1 when it is time to hang up: 2 s after
   the last bytes went (time for the final V.42 frames to be acknowledged
   and the goodbye text to show), or 20 s after the close at the latest. */
int sm_call_server_closed_step(long long *since_close, long long *since_drained,
                               int n, int drained);

/* Initialise a call with the given configuration. */
void sm_call_init(sm_call_t *c, const sm_call_config_t *cfg, int call_id);

/* Run one call to completion on the accepted AudioSocket fd. Returns 0 on a
   clean hangup. Blocks until the call ends. */
int sm_call_run(sm_call_t *c, int socket_fd);

#endif
