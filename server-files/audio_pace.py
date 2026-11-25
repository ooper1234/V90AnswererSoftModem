#!/usr/bin/python3
"""AudioSocket relay that keeps the modem clocked when VoIP input stalls.

sm_daemon answers every received audio frame with exactly one transmitted
frame, so its transmitter is slaved to inbound RTP. When the provider's
audio pauses (media path not yet latched at call start, or a loss burst),
the modem falls silent too: ANSam/V.8 come out late or never, and the
silence keeps the far media relay from latching onto us.

This relay sits between Asterisk and sm_daemon:
  * real audio frames are forwarded immediately (no added delay);
  * if no input frame has arrived for GAP_MS, a silence frame is injected
    every 20 ms, so the modem keeps transmitting on time;
  * frames that arrive in a burst right after a gap (late, not lost) are
    dropped against the silence already injected, so the modem's sample
    count stays aligned with real time.

With PACE_EXACT=1 the count of frames handed to the modem is kept exact
across a gap: when a real frame arrives after one, silence is injected for
every 20 ms of the gap not already covered, so a burst of lost packets costs
the modem their content but not its place in the stream.

With PACE_TX_LEAD=K (frames; 0, the default, leaves this off) the modem's
output is steadied as well: its audio frames are queued and exactly one is
released to Asterisk for each frame handed to the modem, K frames of silence
going out first. The modem can then be up to K*20 ms late with a frame (it
is, when a training solve runs) without the line hearing a gap and then a
burst, which a telephone adapter's playout buffer answers by slipping whole
frames. If the modem is later still, silence stands in for the frame and the
late frame is dropped when it comes, so the stream keeps its alignment.

With PACE_STEADY=1 (and a PACE_TX_LEAD) the two directions are taken apart.
Every frame from Asterisk goes to the modem as it comes, late or in a burst,
and none is made up or dropped: the leg is TCP, so nothing on it is lost, and
on the calls that failed on 2026-11-12 Asterisk held frames back 60-125 ms and
then handed them over together, which the rules above answered with silence in
the modem's input and real frames thrown away. The modem's frames go out to
Asterisk on this relay's own 20 ms clock instead of one for each frame that
came in, so the line does not hear Asterisk's pauses, or the modem's (about
110 ms every six or seven seconds), as a gap and a burst. The clock is steered
slowly so that the queue stays PACE_TX_LEAD frames deep, which keeps it on
the line's own rate. Silence is still injected into the modem's input when
nothing has come for PACE_GAP_MS, which should then be long.

AudioSocket framing: kind (1 byte), length (2 bytes BE), payload.
Kinds: 0x00 hangup, 0x01 UUID, 0x10 audio (signed linear 8 kHz), 0xff error.
"""
import os
import select
import socket
import struct
import sys
import threading
import time

LISTEN = ('127.0.0.1', int(os.environ.get('PACE_LISTEN_PORT', '9104')))
TARGET = ('127.0.0.1', int(os.environ.get('PACE_TARGET_PORT', '9105')))
FRAME_MS = 20
FRAME_BYTES = 320            # 160 samples of 16-bit signed linear
GAP_MS = float(os.environ.get('PACE_GAP_MS', '60'))
TX_LEAD = int(os.environ.get('PACE_TX_LEAD', '0'))
EXACT = os.environ.get('PACE_EXACT', '0') == '1'
STEADY = os.environ.get('PACE_STEADY', '0') == '1' and TX_LEAD > 0
LOSS_MS = float(os.environ.get('PACE_LOSS_MS', '70'))   # lateness that is loss and not delivery (PACE_EXACT)
EVENT_MS = float(os.environ.get('PACE_EVENT_MS', '60'))  # a silence this long from either side is logged
STEER_GAIN = 1e-3            # clock correction per frame of queue error (PACE_STEADY)
SETTLE_TICKS = 250           # ticks after the lead (5 s) whose mean depth is the steering mark
BURST_MS = 8.0               # frames closer than this after a gap are late copies
DEBT_MAX = 250               # most silence (5 s) late frames may be dropped against (PACE_STEADY)
KIND_HANGUP, KIND_UUID, KIND_AUDIO = 0x00, 0x01, 0x10
SILENCE = struct.pack('!BH', KIND_AUDIO, FRAME_BYTES) + bytes(FRAME_BYTES)


def log(*a):
    print(time.strftime('%H:%M:%S'), *a, flush=True)


def read_exact(sock, n):
    buf = b''
    while len(buf) < n:
        chunk = sock.recv(n - len(buf))
        if not chunk:
            return None
        buf += chunk
    return buf


class Call:
    def __init__(self, ast, modem, tag):
        self.ast, self.modem, self.tag = ast, modem, tag
        self.lock = threading.Lock()          # serialises writes to the modem
        self.last_in = None                   # monotonic time of last real input frame
        self.started = False                  # UUID forwarded: modem expects audio now
        self.debt = 0                         # silence frames injected and not yet repaid
        self.done = threading.Event()
        self.injected = self.dropped = self.forwarded = 0
        self.last_real = None                 # monotonic time of the last real frame forwarded
        self.gap_injected = 0                 # silence injected since then
        self.expect = None                    # when the next real frame is due (PACE_EXACT)
        self.seen_in = self.seen_modem = 0.0   # when each side last sent audio (event log)
        self.odd = 0                          # audio frames that were not 20 ms long
        self.made_up = 0                      # silence injected on a frame's arrival, for its gap
        self.ast_lock = threading.Lock()      # serialises writes to Asterisk
        self.gap_lock = threading.Lock()      # one thread at a time decides what the modem is handed
        self.txq = []                         # modem audio frames waiting for their turn
        self.lead_left = TX_LEAD              # silence frames still to go out first
        self.tx_debt = 0                      # late modem frames to drop when they come
        self.tx_late = self.tx_dropped = 0
        self.tx_low = TX_LEAD                 # fewest frames ever waiting at a release
        self.steer = 0.0                      # the clock's current correction (PACE_STEADY)
        self.steer_sum, self.steer_n = 0.0, 0 # for the call's mean correction
        self.mark = None                      # the queue depth steered to, once settled
        self.modem_at = None                  # when the modem's last audio frame was queued
        self.dry = 0                          # silence released for an empty queue since the line last sent (PACE_STEADY)

    def to_ast(self, data):
        with self.ast_lock:
            self.ast.sendall(data)

    def release(self):
        """One frame to Asterisk for one frame handed to the modem."""
        if not TX_LEAD or STEADY:
            return
        with self.lock:
            if self.lead_left:
                self.lead_left -= 1
                frame = SILENCE
            elif self.txq:
                self.tx_low = min(self.tx_low, len(self.txq) - 1)
                frame = self.txq.pop(0)
            else:
                self.tx_low = -1
                self.tx_late += 1
                self.tx_debt += 1
                frame = SILENCE
        self.to_ast(frame)

    def to_modem(self, data):
        with self.lock:
            self.modem.sendall(data)
        if data[0] == KIND_AUDIO:
            self.release()

    def ast_reader(self):
        """Asterisk -> modem."""
        prev = 0.0
        try:
            while not self.done.is_set():
                hdr = read_exact(self.ast, 3)
                if hdr is None:
                    break
                kind, ln = struct.unpack('!BH', hdr)
                payload = read_exact(self.ast, ln) if ln else b''
                if payload is None:
                    break
                # One frame's accounting at a time: the pacer thread injects too.
                with self.gap_lock:
                    now = time.monotonic()
                    if kind == KIND_AUDIO:
                        if self.seen_in and (now - self.seen_in) * 1000 >= EVENT_MS:
                            log(f'{self.tag} event: {(now - self.seen_in) * 1000:.0f} ms without a frame from Asterisk, '
                                f'{(now - self.seen_modem) * 1000:.0f} ms since the modem last sent, txq {len(self.txq)}')
                        self.seen_in = now
                    if kind == KIND_AUDIO and ln != FRAME_BYTES:
                        self.odd += 1
                    if kind == KIND_AUDIO and EXACT and not STEADY:
                        # A frame after a gap: every 20 ms of the gap is a frame
                        # the modem must have had by now, real or silence, or
                        # its sample count falls behind the line's and with it
                        # its framing and its echo canceller's delay. Silence
                        # for the ones not injected yet; if the missing frames
                        # were only late they follow in a burst and are dropped
                        # against the debt below.
                        #
                        # The gap is judged against when this frame was due on
                        # the line's own cadence (self.expect, which follows
                        # the arrivals slowly), not against the frame before:
                        # frames 10 and 30 ms apart in turn are jitter, with
                        # nothing missing between them.
                        if self.expect is None:
                            self.expect = now
                        late_ms = (now - self.expect) * 1000
                        # Lateness under LOSS_MS is delivery, not
                        # loss: Asterisk hands some calls' frames over two at a
                        # time, 38 and 2 ms apart, with every packet on the
                        # wire on time (2026-11-12, eight calls of twenty), and
                        # the first of each pair is then most of a frame late.
                        # Counted as loss, each put 20 ms of silence into the
                        # modem's input and 20 ms onto the line's delay.
                        # (A frame and a half was not enough: the first of a
                        # pair came 30 ms late now and then, two silences went
                        # in, and five calls of seventeen broke on them.)
                        missing = int((late_ms + FRAME_MS / 2) // FRAME_MS) if late_ms >= LOSS_MS else 0
                        for _ in range(min(missing, 50)):
                            self.to_modem(SILENCE)
                            self.injected += 1
                            self.made_up += 1
                            self.debt = min(self.debt + 1, 50)
                        self.expect += missing * FRAME_MS / 1000.0
                    if kind == KIND_AUDIO and STEADY:
                        # Silence stood in for every slot of a gap (pacer()),
                        # so frames that come after it in one burst are the
                        # gap's own, late: dropped against that silence, or
                        # the modem has more samples than the line and its
                        # framing slips (2026-11-25 call 18: 31 frames over,
                        # PCM upstream never decoded a frame again). A frame
                        # on the line's cadence ends the burst: the rest of
                        # the gap was lost, and the silence was right.
                        if self.debt and not self.gap_injected and (now - prev) * 1000 < BURST_MS:
                            self.debt -= 1
                            self.dropped += 1
                            prev = now
                            continue
                        if not self.gap_injected:
                            self.debt = 0
                        self.dry = 0
                        prev = now
                        self.last_in = now
                        self.last_real = now
                        self.gap_injected = 0
                        self.forwarded += 1
                    elif kind == KIND_AUDIO:
                        if self.debt and (now - prev) * 1000 < BURST_MS:
                            self.debt -= 1        # a late frame: silence already stood in for it
                            self.dropped += 1
                            prev = now
                            continue
                        if self.debt and EXACT and self.gap_injected:
                            pass                  # first frame after a gap: a burst may follow it
                        elif self.debt and self.last_in and (now - self.last_in) * 1000 > 2 * FRAME_MS:
                            pass                  # still gappy; keep the debt
                        elif self.debt:
                            self.debt = max(0, self.debt - 1)   # normal cadence: the gap was real loss
                        prev = now
                        self.last_in = now
                        self.last_real = now
                        self.gap_injected = 0
                        if EXACT:
                            # Due again in 20 ms, a twentieth of this frame's
                            # earliness or lateness taken into the cadence.
                            self.expect += FRAME_MS / 1000.0 + 0.05 * (now - self.expect)
                        self.forwarded += 1
                    self.to_modem(hdr + payload)
                if kind == KIND_UUID:
                    self.started = True
                    self.last_in = now
                if kind == KIND_HANGUP:
                    break
        except OSError:
            pass
        self.done.set()

    def release_steady(self):
        """One frame to Asterisk on this relay's clock (PACE_STEADY)."""
        with self.lock:
            if self.lead_left:
                self.lead_left -= 1
                frame = SILENCE
            elif self.txq:
                self.tx_low = min(self.tx_low, len(self.txq) - 1)
                frame = self.txq.pop(0)
            else:
                # The modem is later than the whole lead: silence, and its
                # frame goes out when it comes, the queue one deeper until
                # the clock has taken that back.
                self.tx_low = -1
                self.tx_late += 1
                frame = SILENCE
                if self.gap_injected:
                    self.tx_debt += 1     # the modem is answering the gap's silence: its frame is this slot's
                else:
                    self.dry += 1
            depth = len(self.txq) + self.lead_left
            if self.modem_at is not None:
                # Counted in frames waiting, the depth steps by a whole
                # frame when the modem's frames come at our tick, by a
                # hair before it or after; the time since the last one, in
                # frames, makes it continuous, and keeps it level through a
                # stall too (a frame fewer waiting each tick, a frame longer
                # since the last) until the late frames come together.
                depth += (time.monotonic() - self.modem_at) / (FRAME_MS / 1000.0)
        self.to_ast(frame)
        return depth

    def modem_reader(self):
        """modem -> Asterisk: unchanged, or queued for release() with a lead."""
        try:
            while not self.done.is_set():
                if not TX_LEAD:
                    data = self.modem.recv(65536)
                    if not data:
                        break
                    self.to_ast(data)
                    continue
                hdr = read_exact(self.modem, 3)
                if hdr is None:
                    break
                kind, ln = struct.unpack('!BH', hdr)
                payload = read_exact(self.modem, ln) if ln else b''
                if payload is None:
                    break
                if kind != KIND_AUDIO:
                    self.to_ast(hdr + payload)
                    continue
                t = time.monotonic()
                if self.seen_modem and (t - self.seen_modem) * 1000 >= EVENT_MS:
                    log(f'{self.tag} event: {(t - self.seen_modem) * 1000:.0f} ms without a frame from the modem, '
                        f'{(t - self.seen_in) * 1000:.0f} ms since Asterisk last sent, txq {len(self.txq)}')
                self.seen_modem = t
                with self.lock:
                    if self.tx_debt:
                        self.tx_debt -= 1     # silence already went out in its place
                        self.tx_dropped += 1
                    else:
                        self.txq.append(hdr + payload)
                        self.modem_at = t
        except OSError:
            pass
        self.done.set()

    def pacer(self):
        """Inject silence while input is missing, on a 20 ms schedule."""
        next_tick = time.monotonic()
        depth_avg, steer = float(TX_LEAD), 0.0
        # What the queue reads when the modem is on our clock: counted after
        # the release, and wherever the modem's frames fall between our
        # ticks, it is TX_LEAD less one or so, not TX_LEAD. Steering to
        # TX_LEAD itself took that frame as the line's rate and held the
        # clock up to 50 ppm slow for a whole call (2026-11-16: +40 to +60
        # ppm on the line against a modem within 5 of ours), which the far
        # end's buffer takes up by slipping. So the depth the first five seconds
        # after the lead settles at is the mark.
        mark, settle = None, []
        held_mark = float(TX_LEAD)          # the mark before the line last went quiet
        while not self.done.is_set():
            next_tick += FRAME_MS / 1000.0 * (1.0 - steer)
            delay = next_tick - time.monotonic()
            if delay > 0:
                time.sleep(delay)
            elif delay < -0.2 or not STEADY:
                next_tick = time.monotonic()  # fell behind (scheduler hiccup): resync
            # (STEADY, a little late: send now and keep the schedule, so the
            # frame times on the line stay on one 20 ms grid.)
            if not self.started or self.last_in is None:
                continue
            if STEADY:
                try:
                    depth = self.release_steady()
                except OSError:
                    break
                # The far end's jitter buffer reads our frames against their
                # RTP timestamps, which step 20 ms each whatever the send
                # time: a clock that wanders hundreds of parts in a million
                # makes it resize or conceal (2026-11-13, outside route: +-700
                # ppm, 43 ms of skew in 3 min, a 70 ms jump and a glitch every
                # 10 s). So the clock is the nominal 20 ms, nudged at most 300
                # ppm by ten seconds' average of the queue: the modem's
                # stalls come and go inside the lead and do not reach it, and
                # what remains is the line's clock against ours, a few tens of
                # ppm, which this takes up a frame at a time.
                if self.gap_injected or (self.last_in is not None and (time.monotonic() - self.last_in) * 1000 >= GAP_MS):
                    # Nothing from the line (a hold, or an adapter that has
                    # stopped sending): the modem gets nothing, or our own
                    # silence, so it owes us the frames of the gap for good
                    # and the queue is shallower by them. That is not the
                    # line's rate: leave the clock where it was and settle
                    # the mark again once the line is back (2026-11-16 call
                    # 10: 53 s of hold steered to -300 ppm and the modem
                    # could not stay trained after it).
                    # While the line is not listening, silence makes up
                    # the lead the gap took, so the modem's stalls are
                    # taken up in it again afterwards.
                    if mark is not None:
                        held_mark = mark
                    if self.gap_injected and depth < held_mark - 0.5:
                        with self.lock:
                            self.lead_left += 1
                    mark, settle = None, []
                elif mark is None:
                    if not self.lead_left:
                        settle.append(depth)
                    if len(settle) >= SETTLE_TICKS:
                        # The mean, for a queue that flickers between two
                        # depths when the modem's frames come at our tick;
                        # less the stall dips, more than a frame under the
                        # median.
                        mid = sorted(settle)[len(settle) // 2]
                        kept = [d for d in settle if d >= mid - 1]
                        mark = round(sum(kept) / len(kept), 2)
                        depth_avg = mark
                        self.mark = mark
                else:
                    depth_avg += 0.002 * (depth - depth_avg)
                    err = depth_avg - mark
                    # 1e-3 a frame: the line's rate is taken up in about
                    # 20 s (50 ticks/s x 1e-3), the ten seconds' average in
                    # front of it leaving the loop near critically damped.
                    # 5e-5 took 400 s, longer than a call, and with no
                    # jitter buffer in Asterisk the answerer runs on the
                    # caller's clock, which is the one to follow
                    # (2026-11-16: 22 ppm fast, the download falling behind
                    # it until the caller's modem lost its frames).
                    steer = max(-3e-4, min(3e-4, STEER_GAIN * err))
                    self.steer = steer
                    self.steer_sum += steer
                    self.steer_n += 1
            with self.gap_lock:
                idle_ms = (time.monotonic() - self.last_in) * 1000
                if STEADY and (idle_ms >= GAP_MS or (self.gap_injected and idle_ms >= FRAME_MS)):
                    # A gap: silence for every slot of it, the first GAP_MS
                    # too, so the modem's sample count stays the line's
                    # (2026-11-25: the first 400 ms of each gap went
                    # unfilled, the modem fell 20 frames behind the line
                    # for good). The modem's frames for that silence fill
                    # slots that already went out as silence, so they are
                    # dropped when they come, or the line's delay grows by
                    # the gap.
                    n = int(idle_ms // FRAME_MS) if not self.gap_injected else 1
                    try:
                        for _ in range(n):
                            self.to_modem(SILENCE)
                    except OSError:
                        break
                    with self.lock:
                        if not self.gap_injected:
                            self.tx_debt += self.dry
                            self.dry = 0
                    self.injected += n
                    self.gap_injected += n
                    self.debt = min(self.debt + n, DEBT_MAX)
                    self.last_in += n * FRAME_MS / 1000.0
                    continue
                if idle_ms >= GAP_MS and not STEADY:
                    try:
                        self.to_modem(SILENCE)
                    except OSError:
                        break
                    self.injected += 1
                    self.gap_injected += 1
                    if self.expect is not None:
                        self.expect += FRAME_MS / 1000.0   # this silence stands in a frame's slot
                    self.debt = min(self.debt + 1, 50)   # never hold more than 1 s of debt
                    # Count the injected frame as input time so the next one comes 20 ms later.
                    self.last_in += FRAME_MS / 1000.0

    def run(self):
        threads = [threading.Thread(target=f, daemon=True)
                   for f in (self.ast_reader, self.modem_reader, self.pacer)]
        for t in threads:
            t.start()
        self.done.wait()
        for s in (self.ast, self.modem):
            try:
                s.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            s.close()
        log(f'{self.tag} ended: forwarded={self.forwarded} injected_silence={self.injected} '
            f'dropped_late={self.dropped}'
            + (f' made_up={self.made_up}' if EXACT else '') + f' odd={self.odd}'
            + (' steady' if STEADY else '')
            + (f' tx_lead={TX_LEAD} tx_low={self.tx_low} tx_late={self.tx_late} tx_dropped={self.tx_dropped}'
               if TX_LEAD else '')
            + (f' mark={self.mark} steer_mean={self.steer_sum / self.steer_n * 1e6:+.1f}ppm'
               f' steer_end={self.steer * 1e6:+.1f}ppm' if STEADY and self.steer_n else ''))


def serve(conn, addr, n):
    try:
        modem = socket.create_connection(TARGET, timeout=5)
    except OSError as e:
        log(f'call {n}: modem daemon unreachable: {e}')
        conn.close()
        return
    for s in (conn, modem):
        s.settimeout(None)
        s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
    log(f'call {n} started')
    Call(conn, modem, f'call {n}').run()


def main():
    srv = socket.socket()
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind(LISTEN)
    srv.listen(16)
    log(f'audio pacer listening on {LISTEN[0]}:{LISTEN[1]} -> {TARGET[0]}:{TARGET[1]} (gap {GAP_MS:.0f} ms'
        + (f', transmit lead {TX_LEAD} frames' if TX_LEAD else '') + ')')
    n = 0
    while True:
        conn, addr = srv.accept()
        n += 1
        threading.Thread(target=serve, args=(conn, addr, n), daemon=True).start()


if __name__ == '__main__':
    main()
