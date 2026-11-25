#include "sm_call.h"

#include "ppp/sm_pppd.h"
#include "bbs/bbs.h"

#include <stdlib.h>

/* Bytes each SM_PPP_*_DUMP keeps, SM_PPP_DUMP_MAX overriding. A megabyte by
   default: a call in data mode decodes kilobytes a second, so a cap sized for
   a sweep fills in seconds and then records nothing for the rest of the call.
   Off unless the variable is set. */
static size_t dump_max(void)
{
    static size_t max;
    if (max == 0)
        max = (size_t)strtoul(getenv("SM_PPP_DUMP_MAX") ? getenv("SM_PPP_DUMP_MAX") : "1048576", NULL, 10);
    return max;
}

#ifdef SM_HAVE_BM
#include "bm_answerer.h"
#endif

#ifdef SM_HAVE_V34
/* spandsp/v34.h assumes the base spanDSP environment has been included. */
#include "spandsp/telephony.h"
#include "spandsp/logging.h"
#include "spandsp/complex.h"
#include "spandsp/async.h"
#include "spandsp/dds.h"
#include "spandsp/v29rx.h"
#include "spandsp/v8.h"
#include "spandsp/v34.h"
#endif

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <signal.h>
#include <math.h>
#include <sys/wait.h>
#include <sys/stat.h>
#include <stdio.h>
#include <time.h>

/* ------------------------------------------------------------------ */
/* Modem callbacks                                                     */
/* ------------------------------------------------------------------ */

static int start_pppd(sm_call_t *c);

static void bbs_write(void *ud, const uint8_t *data, size_t len)
{
    sm_call_t *c = ud;
    size_t i;
    for (i = 0; i < len; i++)
    {
        if (sm_bitq_push_byte(&c->txbits, data[i]) < 0)
        {
            sm_log_message(&c->log, SM_LOG_WARNING, "BBS output queue full");
            break;
        }
    }
    c->data_bytes_tx += i;
}

static void start_bbs_selector(sm_call_t *c)
{
    bbs_connection_meta_t m;
    if (c->bbs || c->cfg.echo_data)
        return;
    memset(&m, 0, sizeof(m));
    m.protocol = c->mode == SM_CALL_MODE_V34 ? "V.34" : "V.22bis";
    m.tx_bps = c->negotiated_rate;
    m.rx_bps = c->negotiated_rate;
#ifdef SM_HAVE_BM
    if (c->bm)
    {
        m.protocol = "V.34";
        m.tx_bps = bm_rate_tx(c->bm);
        m.rx_bps = bm_rate_rx(c->bm);
    }
#endif
    m.connected = 1;
    m.connected_at = time(NULL);
    m.carrier_state = 1;
    /* RTP statistics live in sm_sip and are not carried by AudioSocket yet. */
    m.packets_rx = m.packets_tx = m.packets_lost = -1;
    m.jitter_ms = -1.0;
    c->bbs = bbs_session_create(c->cfg.bbs_db, &m, bbs_write, c);
    if (c->bbs)
        bbs_session_start(c->bbs);
}

static int call_get_bit(void *ud)
{
    sm_call_t *c = ud;
    int b;

    if (c->tx_lead_mark > 0)
    {
        c->tx_lead_mark--;
        return 1;
    }
    b = sm_bitq_pop(&c->txbits);

    /* Idle line is all ones when there is nothing to send. */
    return (b < 0) ? 1 : b;
}

static void call_put_bit(void *ud, int bit);

/* The C V.22bis pump's own line bits. Once V.22bis is in data mode they go
   through V.42 (c->v22_ec), which hands the DTE side to call_get_bit and
   call_put_bit; before that, and with V22_V42=0, straight there as always. */
static int v22_get_bit(void *ud)
{
    sm_call_t *c = ud;
#ifdef SM_HAVE_BM
    if (c->v22_ec)
        return bm_ec_tx_bit((bm_ec_link *) c->v22_ec);
#endif
    return call_get_bit(ud);
}

static void v22_put_bit(void *ud, int bit)
{
    sm_call_t *c = ud;
#ifdef SM_HAVE_BM
    if (c->v22_ec)
    {
        if (bit >= 0)
            bm_ec_rx_bit((bm_ec_link *) c->v22_ec, bit);
        return;
    }
#endif
    call_put_bit(ud, bit);
}

#ifdef SM_HAVE_BM
/* Timers and phase logging for a V.42 link outside the engine (V.22bis in
   C, or Bell 103 in the engine's transparent FSK service). */
static void ec_link_service(sm_call_t *c, int n, const char *label)
{
    bm_ec_link *ec = (bm_ec_link *) c->v22_ec;
    int ms;

    c->v22_tick_rem += n * 1000;
    ms = c->v22_tick_rem / SM_SAMPLE_RATE;
    c->v22_tick_rem -= ms * SM_SAMPLE_RATE;
    bm_ec_tick(ec, ms);
    if (bm_ec_link_peer_released(ec) && c->phase != SM_CALL_HANGUP)
    {
        sm_log_message(&c->log, SM_LOG_FLOW,
                       "call ended: %s caller released V.42 (DISC, ATH), hanging up", label);
        c->phase = SM_CALL_HANGUP;
    }
    /*endif*/
    if (bm_ec_link_phase(ec) != c->v22_ec_seen)
    {
        static const char *const names[] = {"detection", "XID", "LAPM", "transparent"};
        int ep = bm_ec_link_phase(ec);

        c->v22_ec_seen = ep;
        if (ep >= 0 && ep <= 3)
            sm_log_message(&c->log, SM_LOG_FLOW,
                           "%s: V.42 %s; error control=%s compression=%s",
                           label, names[ep], ep == 2 ? "V.42 LAPM" : "none",
                           bm_ec_link_compression(ec) == 2 ? "V.44" :
                           bm_ec_link_compression(ec) == 1 ? "V.42bis" : "none");
    }
}

/* V.42 over Bell 103 unless B103_V42=0. */
static int b103_v42(void)
{
    const char *e = getenv("B103_V42");
    return !(e && strcmp(e, "0") == 0);
}
#endif

/* LAPM has already acknowledged these bytes. A physical retrain must not
   discard them at the DTE boundary or reset their async framing. */
static int retains_validated_rx(sm_call_t *c)
{
#ifdef SM_HAVE_BM
    return c->ppp_started
           && ((c->bm && bm_error_control(c->bm))
               || (c->v22_ec && bm_ec_link_phase((bm_ec_link *) c->v22_ec) == 2));
#else
    (void)c;
    return 0;
#endif
}

static void reset_unvalidated_rx(sm_call_t *c)
{
    if (!retains_validated_rx(c))
        sm_deframer_init(&c->deframer);
}

static void call_put_bit(void *ud, int bit)
{
    sm_call_t *c = ud;

    if (bit < 0)
        return;
#ifdef SM_HAVE_V34
    if (c->mode == SM_CALL_MODE_V34)
    {
        /* The V.34 receiver only delivers payload bits once the E has been
           seen and the primary demapper is running, which is exactly data
           mode. Entering here loses no bits. */
        if (c->phase != SM_CALL_DATA)
        {
            c->phase = SM_CALL_DATA;
            c->negotiated_rate = v34_get_current_bit_rate((v34_state_t *) c->v34);
            sm_log_message(&c->log, SM_LOG_FLOW,
                           "==> DATA MODE at %d bps (V.34)", c->negotiated_rate);
        }
        /*endif*/
        sm_deframer_bit(&c->deframer, bit);
        return;
    }
    /*endif*/
#endif
    if (c->phase != SM_CALL_DATA && !retains_validated_rx(c))
        return;
    sm_deframer_bit(&c->deframer, bit);
}

static const char *status_name(v22bis_status_t st)
{
    switch (st)
    {
    case V22BIS_STATUS_TRAINING_SUCCEEDED: return "TRAINING_SUCCEEDED";
    case V22BIS_STATUS_TRAINING_FAILED:    return "TRAINING_FAILED";
    case V22BIS_STATUS_CARRIER_UP:         return "CARRIER_UP";
    case V22BIS_STATUS_CARRIER_DOWN:       return "CARRIER_DOWN";
    case V22BIS_STATUS_RETRAIN_OCCURRED:   return "RETRAIN";
    default:                               return "?";
    }
}

static void call_status(void *ud, v22bis_status_t st)
{
    sm_call_t *c = ud;

    sm_log_message(&c->log, SM_LOG_FLOW, "modem status: %s", status_name(st));
    if (st == V22BIS_STATUS_RETRAIN_OCCURRED)
    {
        /* A retrain resets the link; drop partial async state. */
        sm_deframer_init(&c->deframer);
        c->phase = SM_CALL_HANDSHAKE;
    }
}

/* ------------------------------------------------------------------ */
/* Init                                                                */
/* ------------------------------------------------------------------ */

#ifdef SM_HAVE_V8
static void call_v8_result(void *user_data, int status,
                           int modulations, int call_function, int protocols)
{
    sm_call_t *c = user_data;

    sm_log_message(&c->log, SM_LOG_FLOW,
                   "V.8 status=%d mods=0x%x cf=%d proto=%d",
                   status, modulations, call_function, protocols);
    switch (status)
    {
    case 0:  /* V8_STATUS_IN_PROGRESS */
        break;
    case 2:  /* V8_STATUS_V8_CALL */
        c->v8_done = 1;
#ifdef SM_HAVE_V34
        if (c->cfg.use_v34  &&  (modulations & V8_MOD_V34))
            c->v8_ok = 2;   /* V.34 selected */
        else
#endif
            c->v8_ok = (modulations & 0x04) != 0;   /* V8_MOD_V22 */
        /*endif*/
        break;
    case 3:  /* V8_STATUS_NON_V8_CALL */
        c->v8_done = 1;
        c->v8_ok = 0;
        break;
    case 4:  /* V8_STATUS_FAILED */
        c->v8_done = 1;
        c->v8_ok = 0;
        break;
    default:
        /* offered / calling tone / cng - keep going */
        break;
    }
}
#endif

void sm_call_init(sm_call_t *c, const sm_call_config_t *cfg, int call_id)
{
    memset(c, 0, sizeof(*c));
    c->cfg = *cfg;
    c->call_id = call_id;
    c->ppp_fd = -1;
    c->pppd_pid = -1;
    c->bm_ec_seen = -2;
    sm_log_init(&c->log, cfg->log_level, "CALL", call_id);

#ifdef SM_HAVE_BM
    if (cfg->use_binmodem)
    {
        /* The vendored BinModem answerer owns the whole start-up: its own
           V.8 (ANSam, CM/JM) and then V.34 through phase 4 into data mode.
           --v90 switches it to the V.90 mode: V.8 offering the digital PCM
           category, V.90's start-up when the far end pairs as the analogue
           half, and -- because one object carries the whole ladder -- the
           same 16 kHz V.34 stage as the fallback when it does not. sm_v8,
           the answer tone and the spanDSP V.34 engine stay out of it
           entirely -- until the engine hands a V.22bis call back, which
           bm_poll does. The V.22bis engine is still initialised here: the
           hand-back runs it. */
        c->bm = cfg->v23_mode
                    ? bm_create_v23(1, cfg->v23_mode == 2,
                                    call_get_bit, c, call_put_bit, c)
                    : cfg->v32_max_rate
                    ? bm_create_v32(1, cfg->v32_max_rate,
                                    call_get_bit, c, call_put_bit, c)
                    : cfg->use_v90
                    ? bm_create_v90(1 /* answer */,
                                    call_get_bit, c, call_put_bit, c)
                    : bm_create(1 /* answer */, cfg->use_v34,
                                call_get_bit, c, call_put_bit, c);
        if (c->bm)
        {
            c->phase = SM_CALL_V8;
            sm_log_message(&c->log, SM_LOG_FLOW, cfg->v23_mode
                           ? "BinModem V.23 answerer started (transparent split-speed FSK)"
                           : cfg->v32_max_rate
                           ? "BinModem V.32/V.32bis answerer started (V.42/V.42bis)"
                           : cfg->use_v90
                           ? "BinModem answerer started (V.90 + V.34 fallback, 8 kHz line path)"
                           : "BinModem answerer started (V.8 + V.34, 16 kHz engine)");
            v22bis_init(&c->modem, false /* answerer */, cfg->rate,
                        v22_get_bit, c, v22_put_bit, c, call_status, c);
            c->modem.log.call_id = call_id;
            sm_log_set_level(&c->modem.log, cfg->log_level);
            sm_bitq_init(&c->txbits);
            sm_deframer_init(&c->deframer);
            goto capture_setup;
        }
        sm_log_message(&c->log, SM_LOG_WARNING,
                       "BinModem init failed; falling back to the built-in path");
    }
#endif
    v22bis_init(&c->modem, false /* answerer */, cfg->rate,
                v22_get_bit, c, v22_put_bit, c, call_status, c);
    c->modem.log.call_id = call_id;
    sm_log_set_level(&c->modem.log, cfg->log_level);
    sm_bitq_init(&c->txbits);
    sm_deframer_init(&c->deframer);

capture_setup:
    {
        /* SM_CAPTURE=<base> writes the call's inbound and outbound PCM as raw
           little endian 16 bit samples at 8 kHz, for offline analysis of a
           real call. Bounded to 120 s per direction. */
        const char *base = getenv("SM_CAPTURE");
        if (base  &&  base[0])
        {
            char path[512];
            snprintf(path, sizeof(path), "%s.in.pcm", base);
            c->cap_in = fopen(path, "wb");
            snprintf(path, sizeof(path), "%s.out.pcm", base);
            c->cap_out = fopen(path, "wb");
            sm_log_message(&c->log, SM_LOG_FLOW, "capture enabled: %s.in/out.pcm", base);
        }
        /*endif*/
    }

    c->tone_phase = 0.0;
    c->tone_phase_inc = SM_TWO_PI * 2100.0 / (double) SM_SAMPLE_RATE;
    c->tone_amplitude = 6000.0;

#ifdef SM_HAVE_BM
    /* The BinModem engine runs its own V.8 and speaks for itself; the
       sm_v8 negotiation and the plain answer sequence below would double
       every signal it sends. */
    if (c->bm)
        return;
    /*endif*/
#endif

#ifdef SM_HAVE_V8
    if (cfg->use_v8)
    {
        int allowed = 0x04 /* V8_MOD_V22 */;
#ifdef SM_HAVE_V34
        if (cfg->use_v34)
            allowed |= V8_MOD_V34;
        /*endif*/
#endif
        c->v8 = sm_v8_create(false /* answerer */, allowed,
                             call_v8_result, c, cfg->log_level);
        if (c->v8)
        {
            c->phase = SM_CALL_V8;
            return;
        }
        sm_log_message(&c->log, SM_LOG_WARNING,
                       "V.8 init failed; falling back to classic answer sequence");
    }
#endif

    if (cfg->answer_tone_ms > 0 || cfg->pre_tone_silence_ms > 0)
    {
        c->phase = SM_CALL_ANSWER_TONE;
        c->silence_samples_left = cfg->pre_tone_silence_ms * SM_SAMPLE_RATE / 1000;
        c->tone_samples_left = cfg->answer_tone_ms * SM_SAMPLE_RATE / 1000;
    }
    else
    {
        c->phase = SM_CALL_HANDSHAKE;
    }
}

/* ------------------------------------------------------------------ */
/* PPP plumbing                                                        */
/* ------------------------------------------------------------------ */

static void ppp_flush(sm_call_t *c)
{
    /* The 2026-09-26 calls: the modem decoded 966 bytes of the far end's LCP
     * and the relay reported taking none of them, with no drop and no write
     * error logged anywhere. Say which of the two ways out of here is being
     * taken, and which is not, once a call. */
    static int said_full, said_none;

    if (c->ppy_out_len > 0 && c->ppp_fd < 0 && !said_none)
    {
        said_none = 1;
        sm_log_message(&c->log, SM_LOG_WARNING,
                       "ppp: %d bytes decoded but no pppd to give them to",
                       c->ppy_out_len);
    }
    while (c->ppy_out_len > 0 && c->ppp_fd >= 0)
    {
        ssize_t w = write(c->ppp_fd, c->ppy_out, (size_t) c->ppy_out_len);
        if (w < 0)
        {
            if (errno == EINTR)
                continue;
            if (errno == EAGAIN || errno == EWOULDBLOCK)
            {
                static int said_busy;
                if (!said_busy)
                {
                    said_busy = 1;
                    sm_log_message(&c->log, SM_LOG_WARNING,
                                   "ppp: the relay is not taking bytes: %s",
                                   strerror(errno));
                }
                return;
            }
            sm_log_message(&c->log, SM_LOG_WARNING, "ppp write failed: %s", strerror(errno));
            return;
        }
        if (!said_full)
        {
            said_full = 1;
            sm_log_message(&c->log, SM_LOG_FLOW, "ppp: relay took %d bytes", (int) w);
        }
        memmove(c->ppy_out, c->ppy_out + w, (size_t) (c->ppy_out_len - w));
        c->ppy_out_len -= (int) w;
    }
}

static void pump_ppp(sm_call_t *c)
{
    if (c->phase != SM_CALL_DATA && c->ppp_fd < 0 && !c->cfg.echo_data)
        return;

    /* Straight to PPP: no selection banner, pppd starts the moment data
       mode is up. Bytes typed before that are staged in ppy_out and flushed
       by ppp_flush as soon as the pty exists. */
    if (c->phase == SM_CALL_DATA && !c->cfg.echo_data
        && c->cfg.enable_ppp && !c->ppp_started)
    {
        if (start_pppd(c) < 0)
        {
            c->phase = SM_CALL_HANGUP;
            return;
        }
    }

    /* Modem RX -> selector/BBS, pppd, or echo. */
    if (c->phase == SM_CALL_DATA || retains_validated_rx(c))
    {
        uint8_t tmp[256];
        int limit = (int) sizeof(tmp);
        /* LAPM has already acknowledged these bytes. Leave them in the
           deframer until the PPP relay has capacity instead of taking and
           discarding them when a blocked relay fills ppy_out. */
        if (c->ppp_started && c->ppp_fd >= 0 && !c->bbs && !c->cfg.echo_data)
        {
            int room;
            ppp_flush(c);
            room = (int) sizeof(c->ppy_out) - c->ppy_out_len;
            if (limit > room)
                limit = room;
        }
        int n = sm_deframer_take(&c->deframer, tmp, limit);
        int off = 0;

        if (n > 0 && c->cfg.echo_data)
        {
            int i;
            for (i = 0; i < n; i++)
                sm_bitq_push_byte(&c->txbits, tmp[i]);
            c->data_bytes_tx += n;
            c->data_bytes_rx += n;
            return;
        }

        if (c->bbs)
        {
            int route;
            if (n > 0)
                bbs_session_feed((bbs_session_t *) c->bbs, tmp, (size_t) n);
            route = bbs_session_route((bbs_session_t *) c->bbs);
            c->data_bytes_rx += n;
            if (route == BBS_ROUTE_HANGUP)
            {
                c->phase = SM_CALL_HANGUP;
                return;
            }
            if (route != BBS_ROUTE_PPP)
                return;
            if (c->cfg.enable_ppp && c->ppp_fd < 0 && start_pppd(c) < 0)
            {
                c->phase = SM_CALL_HANGUP;
                return;
            }
            if (c->ppp_fd >= 0)
            {
                uint8_t early[1024];
                size_t en = bbs_session_take_ppp((bbs_session_t *) c->bbs,
                                                  early, sizeof(early));
                if (en > sizeof(c->ppy_out)) en = sizeof(c->ppy_out);
                memcpy(c->ppy_out, early, en);
                c->ppy_out_len = (int) en;
            }
            bbs_session_destroy((bbs_session_t *) c->bbs);
            c->bbs = NULL;
            ppp_flush(c);
            return;
        }

        /* SM_PPP_RX_DUMP: the first bytes the line hands over in data mode, as
           hex, so what the far end's data mode really carries can be read
           rather than inferred from pppd's silence. Off unless it is set. */
        {
            static FILE *dump;
            static size_t dumped;
            const char *path = getenv("SM_PPP_RX_DUMP");

            if (n > 0 && path && path[0])
            {
                if (!dump)
                    dump = fopen(path, "w");
                if (dump && dumped < dump_max())
                {
                    size_t room = dump_max() - dumped;
                    int k, upto = (int)(room < (size_t)n ? room : (size_t)n);

                    for (k = 0; k < upto; k++)
                        fprintf(dump, "%02x%s", tmp[k], (k % 32 == 31) ? "\n" : " ");
                    if (upto % 32)
                        fputc('\n', dump);
                    fflush(dump);
                    dumped += (size_t)upto;
                }
            }
        }

        while (off < n)
        {
            if (c->ppy_out_len >= (int) sizeof(c->ppy_out))
            {
                sm_log_message(&c->log, SM_LOG_WARNING,
                               "ppp rx buffer full, dropped %d bytes", n - off);
                break;
            }
            {
                int space = (int) sizeof(c->ppy_out) - c->ppy_out_len;
                int chunk = (n - off < space) ? n - off : space;
                memcpy(c->ppy_out + c->ppy_out_len, tmp + off, (size_t) chunk);
                c->ppy_out_len += chunk;
                off += chunk;
            }
        }
        c->data_bytes_rx += n;
        ppp_flush(c);
    }

    /* pppd -> modem TX. Stage bytes during training, but leave excess bytes
       in the relay until the modem makes room. Consuming a fixed-size read
       during a long retrain used to discard its tail when txbits filled. */
    {
        uint8_t tmp[256];
        ssize_t r;
        size_t queued, room;
        if (c->ppp_fd < 0)
            return;
        queued = sm_bitq_count(&c->txbits);
        room = queued < SM_BITQ_SIZE ? (SM_BITQ_SIZE - queued) / 10 : 0;
        if (room == 0)
            return;
        if (room > sizeof(tmp))
            room = sizeof(tmp);
        r = read(c->ppp_fd, tmp, room);
        if (r > 0)
        {
            int i;
            for (i = 0; i < r; i++)
            {
                if (sm_bitq_push_byte(&c->txbits, tmp[i]) < 0)
                {
                    sm_log_message(&c->log, SM_LOG_WARNING, "modem tx queue full");
                    break;
                }
            }
            c->data_bytes_tx += i;

            /* SM_PPP_TX_DUMP: the bytes pppd hands the line, as hex, the
               counterpart to SM_PPP_RX_DUMP. Off unless it is set. */
            {
                static FILE *dump;
                static size_t dumped;
                const char *path = getenv("SM_PPP_TX_DUMP");

                if (path && path[0])
                {
                    if (!dump)
                        dump = fopen(path, "w");
                    if (dump && dumped < dump_max())
                    {
                        size_t room = dump_max() - dumped;
                        int k, upto = (int)(room < (size_t)r ? room : (size_t)r);

                        for (k = 0; k < upto; k++)
                            fprintf(dump, "%02x%s", tmp[k], (k % 32 == 31) ? "\n" : " ");
                        if (upto % 32)
                            fputc('\n', dump);
                        fflush(dump);
                        dumped += (size_t)upto;
                    }
                }
            }
        }
        else if (r == 0)
        {
            sm_log_message(&c->log, SM_LOG_INFO, "pppd closed the pty");
            c->ppp_fd = -1;
            c->ppp_closed = 1;
        }
    }
}

/* ------------------------------------------------------------------ */
/* Audio processing                                                    */
/* ------------------------------------------------------------------ */

#ifdef SM_HAVE_BM
/* Observe the BinModem engine once per audio chunk: log phase changes, and
   turn its status into the call state machine's phases. Runs before the
   chunk's bits are moved, so anything the transition flushes cannot reach
   the byte path. */
static void bm_poll(sm_call_t *c, int n)
{
    int st = bm_status(c->bm);
    const char *ph = bm_phase(c->bm);
    int ep = bm_ec_phase(c->bm);

    if (c->v22_ec)
        ec_link_service(c, n, "Bell 103");

    if (strcmp(ph, c->bm_phase_seen) != 0)
    {
        int d = 0;
        double pk = 0.0, en = 0.0;
        snprintf(c->bm_phase_seen, sizeof(c->bm_phase_seen), "%s", ph);
        bm_echo(c->bm, &d, &pk, &en);
        if (ph[0])
            sm_log_message(&c->log, SM_LOG_FLOW,
                           "binmodem: %s (echo delay=%d peak=%.2f energy=%.4f)",
                           ph, d, pk, en);
        /*endif*/
    }

    if (ep != c->bm_ec_seen)
    {
        static const char *const names[] = {"detection", "XID", "LAPM", "transparent"};
        c->bm_ec_seen = ep;
        if (ep >= 0 && ep <= 3)
            sm_log_message(&c->log, SM_LOG_FLOW,
                           "binmodem: V.42 %s (V.8 LAPM=%d observed=0x%x damaged=%llu)",
                           names[ep], bm_lapm_declared(c->bm),
                           bm_ec_observed(c->bm), bm_damaged_frames(c->bm));
    }

    if (bm_ec_peer_released(c->bm))
    {
        sm_log_message(&c->log, SM_LOG_FLOW,
                       "call ended: caller released V.42 (DISC, ATH), hanging up");
        c->phase = SM_CALL_HANGUP;
        return;
    }
    /*endif*/

    if (st == BM_FAILED)
    {
        sm_log_message(&c->log, SM_LOG_ERROR, "binmodem failed: %s",
                       bm_failure(c->bm));
        c->phase = SM_CALL_HANGUP;
        return;
    }

    if (st == BM_AGREED_V22 || st == BM_AGREED_OTHER)
    {
        /* V.8 settled on something this daemon's own engines carry. Hand
           back exactly the transition the built-in V.8 path performs: the
           V.22bis receiver restarts, squelched for 500 ms while the answer
           sequence it is about to hear stops ringing in its own band. */
        sm_log_message(&c->log, SM_LOG_FLOW,
                       st == BM_AGREED_V22
                           ? "V.8 negotiated V.22bis; starting V.22bis"
                           : "V.8 not negotiated; starting V.22bis");
        c->mode = SM_CALL_MODE_V22;
        /* Restart both directions: the fallback must begin with the 75 ms
           gap and high-channel unscrambled ones, not stale TX training. */
        v22bis_restart(&c->modem);
#ifdef SM_HAVE_V8
        c->rx_guard = SM_SAMPLE_RATE * 500 / 1000;
#endif
        c->phase = SM_CALL_HANDSHAKE;
        if (st == BM_AGREED_OTHER) {
            c->legacy_probe=1;c->legacy_elapsed=0;c->legacy_v22_seen=0;
            sm_legacy_detect_init(&c->legacy_detector);
            sm_log_message(&c->log, SM_LOG_FLOW, "legacy auto selection: V.22bis / Bell 103 / V.21 / HST, then V.23");
        }
        {
            int delay = 0;
            double pk = 0.0, en = 0.0;
            bm_echo(c->bm, &delay, &pk, &en);
            c->v22_round_trip_ms = delay * 1000 / SM_SAMPLE_RATE;
            c->v22_lapm_declared = st == BM_AGREED_V22 && bm_lapm_declared(c->bm);
            if (st == BM_AGREED_V22)
            {
                /* Our JM is 1650 Hz FSK, inside the V.22bis receive band,
                   and its echo from the far end keeps arriving for one
                   round trip after we stop sending it (about a second on
                   long VoIP routes). Heard as the caller's carrier it runs
                   our sequence ahead of a caller that has not started. The
                   caller cannot answer U11 sooner than a round trip plus
                   its own 155 + 456 ms, so squelching for the round trip
                   plus a margin loses nothing. (rx_guard above only exists
                   in builds with the spandsp V.8 library.) */
                int guard_ms = c->v22_round_trip_ms + 100;
                if (guard_ms < 500)
                    guard_ms = 500;
                if (guard_ms > 2500)
                    guard_ms = 2500;
                c->v22_rx_guard = SM_SAMPLE_RATE / 1000 * guard_ms;
                sm_log_message(&c->log, SM_LOG_FLOW,
                               "V.22bis: receiver squelched %d ms (round trip %d ms)",
                               guard_ms, c->v22_round_trip_ms);
            }
        }
        bm_destroy(c->bm);
        c->bm = NULL;
        return;
    }

    if (st == BM_RETRAINING  &&  c->phase == SM_CALL_DATA)
    {
        /* Raw demodulator bits are training noise; validated LAPM bytes
           remain part of the established PPP stream through a retrain. */
        reset_unvalidated_rx(c);
        c->phase = SM_CALL_HANDSHAKE;
        c->bm_nocarrier = 0;
        sm_log_message(&c->log, SM_LOG_FLOW, "binmodem: retrain");
    }

    if (st == BM_CONNECTED)
    {
        int rate = bm_rate_rx(c->bm);

        if (c->phase != SM_CALL_DATA)
        {
            /* Everything the receiver made of the handshake is noise; the
               engine's own integrator says the same. */
            bm_flush_rx(c->bm);
            reset_unvalidated_rx(c);
            c->phase = SM_CALL_DATA;
            c->negotiated_rate = rate;
            /* Text goes out back-to-back; a V.23 caller whose receiver
               joined it on the wrong bit stays misframed for the whole
               screen (letters bit-rotated, spaces and CR/LF lost) unless
               there is idle mark between characters to resync on. */
            if (ph && strncmp(ph, "Bell 103", 8) == 0 && b103_v42() && !c->v22_ec)
            {
                c->v22_ec = bm_ec_create(1 /* answer */, 0, 300, c->v22_round_trip_ms,
                                         call_get_bit, c, call_put_bit, c);
                c->v22_ec_seen = -1;
                c->v22_tick_rem = 0;
                sm_log_message(&c->log, SM_LOG_FLOW,
                               "Bell 103: V.42 detection (round trip %d ms)",
                               c->v22_round_trip_ms);
            }
            if (ph && (strncmp(ph, "V.23", 4) == 0 || strncmp(ph, "Bell 103", 8) == 0))
            {
                c->txbits.stop_bits = 2;
                /* And a second of mark first: the login screen used to start
                   ~1.4 s after our carrier, before the caller's receiver had
                   settled, so it joined mid-character. Idle mark gives it a
                   clean first start bit to lock on. */
                c->tx_lead_mark = bm_rate_tx(c->bm);
            }
            sm_log_message(&c->log, SM_LOG_FLOW,
                           "==> DATA MODE at %d bps rx, %d bps tx (%s)",
                           rate, bm_rate_tx(c->bm), ph);
            sm_log_message(&c->log, SM_LOG_FLOW,
                           "binmodem: error control=%s compression=%s damaged_frames=%llu",
                           bm_error_control(c->bm) ? "V.42 LAPM" : "none",
                           bm_compression(c->bm) == 2 ? "V.44" :
                           bm_compression(c->bm) == 1 ? "V.42bis" : "none",
                           bm_damaged_frames(c->bm));

        }
        else if (rate != c->negotiated_rate)
        {
            c->negotiated_rate = rate;
            sm_log_message(&c->log, SM_LOG_FLOW,
                           "binmodem: rate renegotiated to %d bps", rate);
        }

        if (bm_carrier(c->bm))
            c->bm_nocarrier = 0;
        else if ((c->bm_nocarrier += n) >= SM_SAMPLE_RATE / 2)
        {
            /* Half a second without the far end's signal: the call is over
               whether the engine says so or not. */
            sm_log_message(&c->log, SM_LOG_FLOW, "binmodem: carrier lost");
            c->phase = SM_CALL_HANGUP;
        }
    }
}
#endif

int sm_call_server_closed_step(long long *since_close, long long *since_drained,
                               int n, int drained)
{
    *since_close += n;
    *since_drained = drained ? *since_drained + n : 0;
    return *since_drained >= 2LL * SM_SAMPLE_RATE
        || *since_close >= 20LL * SM_SAMPLE_RATE;
}

/* The BBS logged the caller off, or PPP ended: hang up rather than leave
   the caller on a line that can never carry anything again. */
static void server_closed_service(sm_call_t *c, int n)
{
    int drained;

    if (!c->ppp_closed || c->phase == SM_CALL_HANGUP)
        return;
    drained = sm_bitq_count(&c->txbits) == 0;
#ifdef SM_HAVE_BM
    /* Not bm_pending: V.42 keeps the engine's transmitter topped up with
       flags all call, so it never reads empty (2026-11-20 call 8 waited for
       the 20 s backstop and the caller hung up first). The far end's
       acknowledgement of every frame is what says the last bytes are out. */
    if (c->bm)
        drained = drained && bm_tx_delivered((bm_answerer *) c->bm);
#endif
    if (sm_call_server_closed_step(&c->ppp_closed_samples, &c->ppp_drained_samples, n, drained))
    {
        sm_log_message(&c->log, SM_LOG_FLOW,
                       "call ended: the server side closed (BBS logoff or PPP ended) %.1f s ago%s, hanging up",
                       (double) c->ppp_closed_samples / SM_SAMPLE_RATE,
                       drained ? "" : " and the last bytes would not drain");
        c->phase = SM_CALL_HANGUP;
    }
}

static void process_audio(sm_call_t *c, const int16_t *in, int n)
{
    int i = 0;

    server_closed_service(c, n);

    if ((c->cap_in  ||  c->cap_out)  &&  c->cap_count < 120L*SM_SAMPLE_RATE)
    {
        if (c->cap_in)
            fwrite(in, sizeof(int16_t), (size_t) n, c->cap_in);
        /*endif*/
        if (c->cap_out)
            fwrite(c->txbuf, sizeof(int16_t), (size_t) n, c->cap_out);
        /*endif*/
        c->cap_count += n;
    }
    /*endif*/

#ifdef SM_HAVE_BM
    if (!c->bm && c->legacy_probe && c->phase == SM_CALL_HANDSHAKE) {
        int detected=0;
        for (int k=0;k<n;k++) {
            int d=sm_legacy_detect_step(&c->legacy_detector,in[k]/32768.0);
            if (d==1) detected=1;
            if (d==3 && !detected) detected=3;
            if (d==4 && !detected) detected=4;
            if (d==2) c->legacy_v22_seen=1;
        }
        c->legacy_elapsed+=n;
        if (c->legacy_v22_seen && c->legacy_probe>=2) {
            c->legacy_probe=1;
            v22bis_restart(&c->modem);
            sm_log_message(&c->log, SM_LOG_FLOW, "legacy caller: qualified V.22 low channel; resuming V.22bis");
        }
        if (detected==1 && !c->legacy_v22_seen) {
            /* Through the V.42 hooks: they pass straight to call_get_bit /
               call_put_bit until data mode starts the link. */
            c->bm=bm_create_bell103(1,v22_get_bit,c,v22_put_bit,c);
            if (c->bm && b103_v42())
                bm_set_raw_line_bits(c->bm,1);
            c->legacy_probe=0;
            sm_deframer_init(&c->deframer);
            sm_log_message(&c->log, SM_LOG_FLOW, "legacy caller: qualified 1270 Hz mark -> Bell 103 (300/300)");
        } else if (detected==4 && !c->legacy_v22_seen) {
            /* The caller is on HST's back channel already, having heard our
               answer tone end: hold the forward channel at once. */
            c->bm=bm_create_hst(1,call_get_bit,c,call_put_bit,c);
            c->legacy_probe=0;
            sm_deframer_init(&c->deframer);
            sm_log_message(&c->log, SM_LOG_FLOW, "legacy caller: qualified HST back channel -> HST (MNP)");
        } else if (detected==3 && !c->legacy_v22_seen) {
            c->bm=bm_create_v21(1,call_get_bit,c,call_put_bit,c);
            c->legacy_probe=0;
            sm_deframer_init(&c->deframer);
            sm_log_message(&c->log, SM_LOG_FLOW, "legacy caller: qualified 980 Hz mark -> V.21 (300/300)");
        } else if (!c->legacy_v22_seen && c->legacy_probe==1 && c->legacy_elapsed>=5*SM_SAMPLE_RATE) {
            c->legacy_probe=2;c->legacy_elapsed=0;c->tone_phase=0;
            sm_log_message(&c->log, SM_LOG_FLOW, "no V.22bis response; probing Bell 103 with 2225 Hz mark");
        } else if (!c->legacy_v22_seen && c->legacy_probe==2 && c->legacy_elapsed>=3*SM_SAMPLE_RATE) {
            /* V.21's caller waits for V.21's own answer mark, 1650 Hz:
               neither V.22bis's USB1 nor Bell's 2225 Hz starts it. */
            c->legacy_probe=3;c->legacy_elapsed=0;c->tone_phase=0;
            sm_log_message(&c->log, SM_LOG_FLOW, "no V.22bis/Bell 103 response; probing V.21 with 1650 Hz mark");
        } else if (!c->legacy_v22_seen && c->legacy_probe==3 && c->legacy_elapsed>=3*SM_SAMPLE_RATE) {
            c->bm=bm_create_v23(1,0,call_get_bit,c,call_put_bit,c);
            c->legacy_probe=0;
            sm_deframer_init(&c->deframer);
            sm_log_message(&c->log, SM_LOG_FLOW, "no V.22bis/Bell 103/V.21 response; probing V.23 with 1300 Hz mark");
        }
        if (c->legacy_probe>=2) {
            double hz=c->legacy_probe==2 ? 2225 : 1650;
            for (int k=0;k<n;k++) {
                c->txbuf[k]=sm_sat16((float)(9000*sin(c->tone_phase)));
                c->tone_phase+=2*3.14159265358979323846*hz/SM_SAMPLE_RATE;
                if (c->tone_phase>2*3.14159265358979323846) c->tone_phase-=2*3.14159265358979323846;
            }
            c->samples_in+=n;c->samples_out+=n;
            return;
        }
    }
    if (c->bm)
    {
        int k;

        bm_poll(c, n);
        if (c->bm)
        {
            /* Order matters: poll first (a transition flushes the noise the
               receiver made before data mode), then move this chunk's bits,
               then run the engine over the chunk. On the hand-back to
               V.22bis, c->bm is NULL and the fall-through below starts the
               built-in engine with the guard bm_poll armed. */
            bm_service(c->bm);
            for (k = 0; k < n; k++)
                c->txbuf[k] = bm_step(c->bm, in[k]);
            i = n;
        }
    }
#endif

    while (i < n)
    {
#ifdef SM_HAVE_V8
        if (c->phase == SM_CALL_V8)
        {
            int m = n - i;
            int got;

            got = c->v8 ? sm_v8_tx(c->v8, c->txbuf + i, m) : 0;
            if (got < m)
                memset(c->txbuf + i + got, 0, (size_t) (m - got) * sizeof(int16_t));
            if (c->v8)
                sm_v8_rx(c->v8, in + i, m);
            i = n;
            break;
        }
#endif
        if (c->phase == SM_CALL_ANSWER_TONE)
        {
            /* Tone phase: one sample at a time so the switch to modem TX is
               sample-exact. */
            if (c->silence_samples_left > 0)
            {
                c->txbuf[i] = 0;
                c->silence_samples_left--;
            }
            else if (c->tone_samples_left > 0)
            {
                c->txbuf[i] = sm_sat16((float) (c->tone_amplitude * sin(c->tone_phase)));
                c->tone_phase += c->tone_phase_inc;
                c->tone_samples_left--;
            }
            if (c->silence_samples_left <= 0 && c->tone_samples_left <= 0)
            {
                c->phase = SM_CALL_HANDSHAKE;
                sm_log_message(&c->log, SM_LOG_FLOW,
                               "answer sequence done -> V.22bis handshake (rate %d)", c->cfg.rate);
            }
            i++;
        }
        else
        {
            int m = n - i;
            if (c->v22_rx_guard > 0  &&  c->mode == SM_CALL_MODE_V22)
            {
                int skip = (m < c->v22_rx_guard) ? m : c->v22_rx_guard;
                static const int16_t quiet[SM_AS_MAX_PAYLOAD / 2];
                if (skip > (int) (sizeof(quiet) / sizeof(quiet[0])))
                    skip = (int) (sizeof(quiet) / sizeof(quiet[0]));
                v22bis_tx(&c->modem, c->txbuf + i, skip);
                v22bis_rx(&c->modem, quiet, skip);
                c->v22_rx_guard -= skip;
                i += skip;
                m -= skip;
            }
#ifdef SM_HAVE_V8
            if (c->rx_guard > 0)
            {
                int skip = (m < c->rx_guard) ? m : c->rx_guard;
                /* The TX must run for these samples; the RX sees silence. */
                v22bis_tx(&c->modem, c->txbuf + i, skip);
                memset(c->rxsquelch, 0, (size_t) skip * sizeof(int16_t));
                v22bis_rx(&c->modem, c->rxsquelch, skip);
                c->rx_guard -= skip;
                i += skip;
                m -= skip;
            }
#endif
            if (m > 0)
            {
#ifdef SM_HAVE_V34
                if (c->mode == SM_CALL_MODE_V34)
                {
                    v34_tx((v34_state_t *) c->v34, c->txbuf + i, m);
                    v34_rx((v34_state_t *) c->v34, in + i, m);
                }
                else
#endif
                {
                    v22bis_tx(&c->modem, c->txbuf + i, m);
                    v22bis_rx(&c->modem, in + i, m);
                }
                /*endif*/
                i = n;
            }
        }
    }

#ifdef SM_HAVE_V8
    /* V.8 -> V.22bis transition. Reset the V.22bis receiver and squelch its
       input for 500 ms. Rationale: right after V.8 the answerer transmits
       U11 on the 2400 Hz carrier, and the calling modem answers with an
       unmodulated 1200 Hz carrier while its 155+456 ms timer runs. Echo of
       our own U11 leaks into the 1200 Hz RX band and can trip the carrier
       detector early, making the receiver commit to 1200 bps before the
       calling modem's S1 (sent 155+456 ms after U11 starts, lasting 100 ms).
       Squelching until ~500 ms puts symbol acquisition on the calling
       modem's own carrier, so the error-tolerant S1 search sees the whole
       S1 burst. TX is unaffected. */
    if (c->phase == SM_CALL_V8 && c->v8_done)
    {
        int started_v34 = 0;

#ifdef SM_HAVE_V34
        if (c->v8_ok == 2)
        {
            /* The common Conexant/PAP2 path rejects 3429 baud in INFO1c but
               offers 3200 baud through 31200 bit/s. Use that interoperable
               ceiling until INFO1-driven dynamic selection is complete. */
            int baud = c->cfg.v34_baud  ?  c->cfg.v34_baud  :  2400;
            int rate = c->cfg.v34_rate  ?  c->cfg.v34_rate  :  2400;

            sm_log_message(&c->log, SM_LOG_FLOW,
                           "V.34 connection speeds: 33600, 31200, 28800, 26400, 24000, 21600, 19200, 16800, 14400, 12000, 9600, 7200, 4800, 2400 bps");
            sm_log_message(&c->log, SM_LOG_FLOW, ">>> v34_init ENTRY baud=%d rate=%d", baud, rate);
            c->v34 = v34_init(NULL, baud, rate,
                              false /* answerer */, true /* duplex */,
                              call_get_bit, c, call_put_bit, c);
            sm_log_message(&c->log, SM_LOG_FLOW, "<<< v34_init EXIT v34=%p", c->v34);
            if (c->v34)
            {
                v34_tx_power((v34_state_t *) c->v34, -13.0f);
                /* V34_TRACE=1 logs the engine's state machine (PP/TRN/J,
                   MP/E, data mode) into the call log. */
                {
                    const char *t = getenv("V34_TRACE");
                    if (t  &&  atoi(t))
                    {
                        logging_state_t *vlog = v34_get_logging_state((v34_state_t *) c->v34);
                        span_log_set_level(vlog, SPAN_LOG_SHOW_SEVERITY | SPAN_LOG_SHOW_TAG | SPAN_LOG_FLOW);
                        span_log_set_tag(vlog, "v34");
                    }
                    /*endif*/
                }
                c->mode = SM_CALL_MODE_V34;
                c->v34_baud_rate = baud;
                started_v34 = 1;
                sm_log_message(&c->log, SM_LOG_FLOW,
                               "V.8 negotiated V.34; starting V.34 (%d baud, up to %d bps)",
                               baud, rate);
                c->phase = SM_CALL_HANDSHAKE;
            }
            else
            {
                sm_log_message(&c->log, SM_LOG_WARNING,
                               "V.34 init failed; starting V.22bis instead");
                c->v8_ok = 1;
            }
            /*endif*/
        }
        /*endif*/
#endif

        if (!started_v34)
        {
#ifdef SM_HAVE_V34
            /* When V.8 fails but --v34 is set, try V.34 directly anyway.
               The PAP2T audio bridge often corrupts the V.8 byte exchange
               (CM/JM), but both endpoints support V.34.  Start V.34 as if
               V.8 had selected it; if the far end is truly V.22bis the
               training will fail and we'll catch it in the handshake. */
            if (c->cfg.use_v34  &&  !c->v8_ok)
            {
                int baud = c->cfg.v34_baud  ?  c->cfg.v34_baud  :  2400;
                int rate = c->cfg.v34_rate  ?  c->cfg.v34_rate  :  2400;

                sm_log_message(&c->log, SM_LOG_FLOW,
                               "V.8 failed; forcing V.34 anyway (%d baud, up to %d bps)",
                               baud, rate);
                c->v34 = v34_init(NULL, baud, rate,
                                  false /* answerer */, true /* duplex */,
                                  call_get_bit, c, call_put_bit, c);
                if (c->v34)
                {
                    v34_tx_power((v34_state_t *) c->v34, -13.0f);
                    {
                        const char *t = getenv("V34_TRACE");
                        if (t  &&  atoi(t))
                        {
                            logging_state_t *vlog = v34_get_logging_state((v34_state_t *) c->v34);
                            span_log_set_level(vlog, SPAN_LOG_SHOW_SEVERITY | SPAN_LOG_SHOW_TAG | SPAN_LOG_FLOW);
                            span_log_set_tag(vlog, "v34");
                        }
                        /*endif*/
                    }
                    c->mode = SM_CALL_MODE_V34;
                    c->v34_baud_rate = baud;
                    started_v34 = 1;
                    c->phase = SM_CALL_HANDSHAKE;
                }
                else
                {
                    sm_log_message(&c->log, SM_LOG_WARNING,
                                   "V.34 init failed; falling back to V.22bis");
                }
            }
            else
#endif
            {
                sm_log_message(&c->log, SM_LOG_FLOW,
                               c->v8_ok ? "V.8 negotiated V.22bis; starting V.22bis"
                                        : "V.8 not negotiated (non-V.8 call); starting V.22bis");
                c->mode = SM_CALL_MODE_V22;
                v22bis_rx_restart(&c->modem);
                c->rx_guard = SM_SAMPLE_RATE * 500 / 1000;
                c->phase = SM_CALL_HANDSHAKE;
            }
        /*endif*/
    }
    }
#endif

    /* Detect data mode: both directions in NORMAL_OPERATION. (V.34 enters
       data mode from its own put_bit callback, on the first payload bit.)
       The gate on c->bm keeps this off while the BinModem engine owns the
       call: a V.22bis engine that has never been stepped sits at
       NORMAL_OPERATION in both directions, which would read as connected. */
    if (c->mode == SM_CALL_MODE_V22
        && c->phase == SM_CALL_HANDSHAKE
        && c->bm == NULL
        && c->modem.tx.training == V22BIS_TX_TRAINING_NORMAL_OPERATION
        && c->modem.rx.training == V22BIS_RX_TRAINING_NORMAL_OPERATION)
    {
        c->phase = SM_CALL_DATA;
        c->negotiated_rate = c->modem.negotiated_bit_rate;
        c->legacy_probe = 0;
        c->modem.negotiated_bit_rate = c->negotiated_rate;
        sm_deframer_init(&c->deframer);
        sm_log_message(&c->log, SM_LOG_FLOW,
                       "==> DATA MODE at %d bps (%s)",
                       c->negotiated_rate,
                       c->modem.rx.sixteen_way_decisions ? "16-way" : "4-way");
#ifdef SM_HAVE_BM
        if (!c->v22_ec && !(getenv("V22_V42") && strcmp(getenv("V22_V42"), "0") == 0))
        {
            c->v22_ec = bm_ec_create(1 /* answer */, c->v22_lapm_declared, c->negotiated_rate,
                                     c->v22_round_trip_ms, call_get_bit, c, call_put_bit, c);
            c->v22_ec_seen = -1;
            c->v22_tick_rem = 0;
            sm_log_message(&c->log, SM_LOG_FLOW,
                           "V.22bis: V.42 %s (round trip %d ms)",
                           c->v22_lapm_declared ? "agreed in V.8: XID" : "detection",
                           c->v22_round_trip_ms);
        }
#endif
    }

#ifdef SM_HAVE_BM
    /* With the engine running (Bell 103), bm_poll services the link. */
    if (c->v22_ec && !c->bm)
        ec_link_service(c, n, "V.22bis");
#endif

    c->samples_in += n;
    c->samples_out += n;
    if (c->bbs)
        bbs_session_tick((bbs_session_t *) c->bbs,
                         (unsigned) ((long long) n * 1000 / SM_SAMPLE_RATE));
}

/* ------------------------------------------------------------------ */
/* pppd management                                                     */
/* ------------------------------------------------------------------ */

static int start_pppd(sm_call_t *c)
{
    sm_pppd_config_t pc;
    char log_path[512];

    memset(&pc, 0, sizeof(pc));
    pc.pppd_path = c->cfg.pppd_path;
    pc.shim_exe = c->cfg.shim_exe;
    pc.local_ip = c->cfg.local_ip;
    pc.peer_ip = c->cfg.peer_ip;
    pc.dns1 = c->cfg.dns1;
    pc.dns2 = c->cfg.dns2;
    pc.auth = c->cfg.auth;
    pc.ip_up_script = c->cfg.ip_up_script;
    pc.ip_down_script = c->cfg.ip_down_script;
    if (c->cfg.log_dir && c->cfg.log_dir[0])
        snprintf(log_path, sizeof(log_path), "%s/pppd-call%d.log", c->cfg.log_dir, c->call_id);
    else
        log_path[0] = 0;
    pc.log_path = log_path[0] ? log_path : NULL;

    c->pppd_pid = sm_pppd_spawn(&pc, &c->ppp_fd);
    if (c->pppd_pid < 0)
    {
        sm_log_message(&c->log, SM_LOG_ERROR, "failed to spawn pppd: %s", strerror(errno));
        return -1;
    }
    fcntl(c->ppp_fd, F_SETFL, O_NONBLOCK);
    c->ppp_started = 1;
    sm_log_message(&c->log, SM_LOG_FLOW, "pppd started pid=%d %s:%s (log %s)",
                   (int) c->pppd_pid, c->cfg.local_ip, c->cfg.peer_ip,
                   log_path[0] ? log_path : "-");
    return 0;
}

static void stop_pppd(sm_call_t *c)
{
    if (c->ppp_fd >= 0)
    {
        close(c->ppp_fd);
        c->ppp_fd = -1;
    }
    if (c->pppd_pid > 0)
    {
        int st;
        kill(c->pppd_pid, SIGTERM);
        /* Give it a moment, then be firm. */
        if (waitpid(c->pppd_pid, &st, WNOHANG) == 0)
        {
            usleep(200000);
            kill(c->pppd_pid, SIGKILL);
            waitpid(c->pppd_pid, &st, 0);
        }
        c->pppd_pid = -1;
    }
}

/* ------------------------------------------------------------------ */
/* Main call loop                                                      */
/* ------------------------------------------------------------------ */

int sm_call_run(sm_call_t *c, int socket_fd)
{
    uint8_t payload[SM_AS_MAX_PAYLOAD];
    uint8_t kind;
    size_t plen;
    int r;
    int timeout_count = 0;
    int rc = 0;
    /* A call that has not reached data mode after this many seconds is hung
       up, so a modem stuck in training (or a dead line Asterisk never ends)
       does not hold a trunk channel and a caller slot. SM_HANDSHAKE_TIMEOUT
       overrides it; 0 disables. Once data mode is reached it no longer
       applies, including to later retrains. */
    double handshake_limit = 120.0;
    int reached_data = 0;
    struct timespec call_started;
    {
        const char *e = getenv("SM_HANDSHAKE_TIMEOUT");
        if (e && *e)
            handshake_limit = atof(e);
    }

    sm_log_message(&c->log, SM_LOG_FLOW, ">>> sm_call_run ENTRY socket_fd=%d call_id=%d phase=%d", socket_fd, c->call_id, c->phase);

    sm_ast_init(&c->as, socket_fd);

    /* First message from Asterisk carries the 16-byte call UUID. */
    for (;;)
    {
        r = sm_ast_read(&c->as, &kind, payload, sizeof(payload), &plen, 5000);
        if (r < 0)
        {
            sm_log_message(&c->log, SM_LOG_WARNING, "socket closed before UUID");
            return -1;
        }
        if (r == 0)
        {
            sm_log_message(&c->log, SM_LOG_WARNING, "timeout waiting for UUID");
            return -1;
        }
        if (kind == SM_AS_KIND_UUID && plen >= 16)
        {
            char u[37];
            sm_ast_uuid_string(payload, u);
            sm_log_message(&c->log, SM_LOG_FLOW, "call start, UUID %s", u);
#ifdef SM_HAVE_BM
            /* UUID ...0091: the dialplan's mark for a caller whose modem
               stalls in V.90 phase 3 after V.92 Quick Connect (a USR
               Sportster: TRN for 3 s, then tone A, never Ja), where the same
               modem trains when Quick Connect is not used. The engine reads
               the offer when it is made, so it is made again here, before
               any audio, without V.92. Each call is its own process, so
               the setting goes no further than this call. */
            if (strcmp(u, "00000000-0000-0000-0000-000000000091") == 0
                && c->bm && c->cfg.use_v90 && !c->cfg.v23_mode && !c->cfg.v32_max_rate)
            {
                setenv("V92_QUICK_CONNECT", "0", 1);
                bm_destroy(c->bm);
                c->bm = bm_create_v90(1 /* answer */, call_get_bit, c, call_put_bit, c);
                sm_log_message(&c->log, SM_LOG_FLOW, c->bm
                               ? "V.92 and its Quick Connect not offered on this call (UUID 0091)"
                               : "engine re-create without V.92 failed");
                if (!c->bm)
                    return -1;
            }
#endif
            break;
        }
        if (kind == SM_AS_KIND_AUDIO)
        {
            sm_log_message(&c->log, SM_LOG_FLOW, "audio before UUID, starting anyway");
            process_audio(c, (const int16_t *) payload, (int) plen / 2);
            break;
        }
        sm_log_message(&c->log, SM_LOG_DEBUG, "pre-call msg kind=0x%02x len=%d", kind, (int) plen);
    }

    /* pppd is started only after the caller selects PPP (or sends an LCP
       frame directly). Starting it here used to queue binary PPP across the
       modem before the post-CONNECT BBS/PPP choice could be shown. */

    clock_gettime(CLOCK_MONOTONIC, &call_started);
    while (c->phase != SM_CALL_HANGUP)
    {
        if (c->phase == SM_CALL_DATA)
            reached_data = 1;
        /*endif*/
        if (!reached_data && handshake_limit > 0.0)
        {
            struct timespec now;
            clock_gettime(CLOCK_MONOTONIC, &now);
            double waited = (double) (now.tv_sec - call_started.tv_sec)
                          + (double) (now.tv_nsec - call_started.tv_nsec) / 1e9;
            if (waited >= handshake_limit)
            {
                sm_log_message(&c->log, SM_LOG_FLOW,
                               "call ended: no data mode after %.0f s of handshake, hanging up",
                               waited);
                break;
            }
            /*endif*/
        }
        /*endif*/
        r = sm_ast_read(&c->as, &kind, payload, sizeof(payload), &plen, 2000);
        if (r < 0)
        {
            sm_log_message(&c->log, SM_LOG_FLOW, "call ended: socket closed");
            break;
        }
        if (r == 0)
        {
            if (++timeout_count >= 5)
            {
                sm_log_message(&c->log, SM_LOG_FLOW, "call ended: no audio for 10s");
                break;
            }
            pump_ppp(c);
            continue;
        }
        timeout_count = 0;

        switch (kind)
        {
        case SM_AS_KIND_AUDIO:
            if (plen & 1)
            {
                sm_log_message(&c->log, SM_LOG_WARNING, "odd audio payload %d", (int) plen);
                plen--;
            }
            if (plen > (size_t) (sizeof(c->txbuf) / sizeof(c->txbuf[0])) * 2)
                plen = (size_t) (sizeof(c->txbuf) / sizeof(c->txbuf[0])) * 2;
            process_audio(c, (const int16_t *) payload, (int) plen / 2);
            if (sm_ast_write_audio(&c->as, c->txbuf, plen / 2) < 0)
            {
                sm_log_message(&c->log, SM_LOG_WARNING, "audio write failed");
                goto out;
            }
            break;

        case SM_AS_KIND_HANGUP:
            sm_log_message(&c->log, SM_LOG_FLOW, "call ended: hangup from Asterisk");
            goto out;

        case SM_AS_KIND_DTMF:
            if (plen >= 1)
                sm_log_message(&c->log, SM_LOG_FLOW, "DTMF digit '%c'", payload[0]);
            break;

        case SM_AS_KIND_ERROR:
            sm_log_message(&c->log, SM_LOG_WARNING, "AudioSocket error frame");
            goto out;

        default:
            sm_log_message(&c->log, SM_LOG_DEBUG, "ignoring msg kind=0x%02x len=%d", kind, (int) plen);
            break;
        }

        pump_ppp(c);

        /* If pppd died, the data path is gone: end the call. */
        if (c->ppp_started && c->pppd_pid > 0)
        {
            int st;
            if (waitpid(c->pppd_pid, &st, WNOHANG) == c->pppd_pid)
            {
                sm_log_message(&c->log, SM_LOG_FLOW, "pppd exited (status %d)", st);
                c->pppd_pid = -1;
                goto out;
            }
        }
    }

out:
    {
        int d = 0;
        double pk = 0.0, en = 0.0;
        if (c->bm)
            bm_echo(c->bm, &d, &pk, &en);
        /*endif*/
        sm_log_message(&c->log, SM_LOG_FLOW,
                       "call summary: samples_in=%lld samples_out=%lld tx_bytes=%lld rx_bytes=%lld rate=%d state=%s echo(delay=%d peak=%.2f energy=%.4f)",
                       c->samples_in, c->samples_out, c->data_bytes_tx, c->data_bytes_rx,
                       c->negotiated_rate ? c->negotiated_rate : c->modem.negotiated_bit_rate,
                       c->phase == SM_CALL_DATA ? "DATA" : "HANDSHAKE",
                       d, pk, en);
        unsigned long long cc[4];
        if (c->bm  &&  bm_compression_counts(c->bm, cc))
        {
            /* Saved = the share of the plain bytes that never went on the line. */
            sm_log_message(&c->log, SM_LOG_FLOW,
                           "compression summary: down %llu -> %llu bytes (%.0f%% saved), up %llu -> %llu bytes (%.0f%% saved)",
                           cc[0], cc[1], cc[0] ? 100.0*((double) cc[0] - (double) cc[1])/(double) cc[0] : 0.0,
                           cc[3], cc[2], cc[3] ? 100.0*((double) cc[3] - (double) cc[2])/(double) cc[3] : 0.0);
        }
        /*endif*/
    }
    if (c->cap_in)
        fclose(c->cap_in);
    /*endif*/
    if (c->cap_out)
        fclose(c->cap_out);
    /*endif*/
    c->cap_in = NULL;
    c->cap_out = NULL;
#ifdef SM_HAVE_V8
    if (c->v8)
    {
        sm_v8_destroy(c->v8);
        c->v8 = NULL;
    }
    /*endif*/
#endif
#ifdef SM_HAVE_BM
    if (c->bm)
    {
        bm_destroy(c->bm);
        c->bm = NULL;
    }
    /*endif*/
    if (c->v22_ec)
    {
        sm_log_message(&c->log, SM_LOG_FLOW, "V.42 link: damaged_frames=%llu",
                       bm_ec_link_damaged_frames((bm_ec_link *) c->v22_ec));
        bm_ec_destroy((bm_ec_link *) c->v22_ec);
        c->v22_ec = NULL;
    }
#endif
#ifdef SM_HAVE_V34
    if (c->v34)
    {
        v34_free((v34_state_t *) c->v34);
        c->v34 = NULL;
    }
    /*endif*/
#endif
    if (c->bbs)
    {
        bbs_session_destroy((bbs_session_t *) c->bbs);
        c->bbs = NULL;
    }
    /* Explicit AudioSocket termination also releases receivers that do not
       promptly notice TCP EOF. Best effort: the peer may already be gone. */
    (void) sm_ast_write(&c->as, SM_AS_KIND_HANGUP, NULL, 0);
    stop_pppd(c);
    close(socket_fd);
    sm_log_message(&c->log, SM_LOG_FLOW, "<<< sm_call_run EXIT rc=%d", rc);
    return rc;
}
