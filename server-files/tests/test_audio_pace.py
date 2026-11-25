"""The pacer's STEADY clock against a modem that keeps exact time.

Run: python3 -m pytest server-files/tests -q   (about 25 s)
"""
import importlib.util, os, socket, struct, threading, time, pathlib

HERE = pathlib.Path(__file__).resolve().parent.parent


def load(**env):
    os.environ.update({k: str(v) for k, v in env.items()})
    spec = importlib.util.spec_from_file_location('audio_pace_t', HERE / 'audio_pace.py')
    m = importlib.util.module_from_spec(spec); spec.loader.exec_module(m)
    return m


def run_call(m, seconds, frame_s, stall_every=0, stall_s=0.0, gap_at=None, gap_s=0.0, gap_burst=False, counts=None):
    """A fake Asterisk sending a frame every frame_s and a fake modem that
    answers each frame with one at once; returns the Call when done."""
    ast_far, ast_near = socket.socketpair()
    modem_near, modem_far = socket.socketpair()
    call = m.Call(ast_near, modem_near, 'test')
    threading.Thread(target=call.run, daemon=True).start()
    frame = struct.pack('!BH', m.KIND_AUDIO, m.FRAME_BYTES) + bytes(m.FRAME_BYTES)

    def modem():
        start = time.monotonic(); held = []; next_stall = stall_every
        try:
            while True:
                hdr = m.read_exact(modem_far, 3)
                if hdr is None: return
                kind, ln = struct.unpack('!BH', hdr)
                body = m.read_exact(modem_far, ln) if ln else b''
                if kind != m.KIND_AUDIO: continue
                if counts is not None: counts['modem_in'] = counts.get('modem_in', 0) + 1
                if stall_every and time.monotonic() - start >= next_stall:
                    # The answerer's training solve: nothing for stall_s,
                    # then every frame it owes at once.
                    next_stall += stall_every
                    time.sleep(stall_s)
                modem_far.sendall(hdr + body)
        except OSError: pass

    def drain():
        try:
            while ast_far.recv(65536): pass
        except OSError: pass

    threading.Thread(target=modem, daemon=True).start()
    threading.Thread(target=drain, daemon=True).start()
    ast_far.sendall(struct.pack('!BH', m.KIND_UUID, 16) + bytes(16))
    t = time.monotonic(); n = 0
    while n * frame_s < seconds:
        n += 1
        d = t + n * frame_s - time.monotonic()
        if d > 0: time.sleep(d)
        if gap_at is not None and gap_at <= n * frame_s < gap_at + gap_s:
            owed = locals().get('owed', 0) + 1
            continue                      # the line is on hold: nothing from Asterisk
        if gap_burst and locals().get('owed'):
            ast_far.sendall(frame * owed)  # held back, now all at once
            owed = 0
        ast_far.sendall(frame)
    steer = call.steer
    if counts is not None:
        counts['slots'] = n
        counts['tx'] = (call.tx_late, call.tx_dropped)   # before the line stops for good
        time.sleep(0.3)                   # the modem's last frames land
    ast_far.sendall(struct.pack('!BH', m.KIND_HANGUP, 0))
    call.done.wait(2)
    return call, steer


def test_a_modem_on_time_leaves_the_clock_at_its_own_rate():
    m = load(PACE_TX_LEAD=6, PACE_STEADY=1, PACE_EXACT=1, PACE_GAP_MS=400)
    # At the real 20 ms: Python's few hundred microseconds of thread jitter
    # is then a small part of a tick, as the answerer's is on the server.
    call, steer = run_call(m, 25, m.FRAME_MS / 1000)
    # The modem is exactly on our clock, so nothing should be steered: a
    # tenth of the old 50 ppm is the most the queue's phase may leave.
    assert abs(steer) < 5e-6, f'steered {steer * 1e6:+.1f} ppm'


def test_the_answerers_stalls_do_not_steer_the_clock():
    m = load(PACE_TX_LEAD=6, PACE_STEADY=1, PACE_EXACT=1, PACE_GAP_MS=400)
    # 90 ms every 3 s, as the server's pacer logged on 2026-11-16.
    call, steer = run_call(m, 25, m.FRAME_MS / 1000, stall_every=3, stall_s=0.09)
    assert abs(steer) < 5e-6, f'steered {steer * 1e6:+.1f} ppm'
    assert call.tx_low >= 0, 'the lead ran dry'


def test_the_clock_follows_a_line_that_runs_fast():
    # Without Asterisk's jitter buffer the answerer runs on the caller's
    # clock (2026-11-16: 22 ppm fast); the pacer must follow it within a
    # call, not over the 400 s the first steering took.
    m = load(PACE_TX_LEAD=6, PACE_STEADY=1, PACE_EXACT=1, PACE_GAP_MS=400)
    call, steer = run_call(m, 45, m.FRAME_MS / 1000 * (1 - 100e-6))
    assert 70e-6 < steer < 130e-6, f'steered {steer * 1e6:+.1f} ppm against a line 100 ppm fast'


def test_a_hold_with_no_audio_from_the_line_does_not_steer_the_clock():
    # 2026-11-16 call 10: the adapter sent nothing for 53 s while the modem was
    # on hold (V.92 MOH, hook flash). The modem got no frames for the first
    # PACE_GAP_MS of it, so it owed us those frames for good, and the steering
    # chased the shallower queue to the -300 ppm limit; after the hold the
    # modem could not stay trained on a clock that far off.
    m = load(PACE_TX_LEAD=6, PACE_STEADY=1, PACE_EXACT=1, PACE_GAP_MS=400)
    call, steer = run_call(m, 26, m.FRAME_MS / 1000, gap_at=8, gap_s=4)
    assert call.injected > 100, 'the hold should have been filled with silence'
    assert abs(steer) < 20e-6, f'steered {steer * 1e6:+.1f} ppm after the hold'
    # And the lead the modem's stalls are taken up in is whole again after it.
    assert call.mark is not None and call.mark >= m.TX_LEAD - 1.5, f'mark {call.mark} after the hold'


def test_asterisk_holding_frames_back_briefly_does_not_reset_the_steering():
    # Asterisk sometimes holds frames back 60-125 ms and hands them over
    # together (2026-11-12); they are late, not lost, and the mark must stand.
    m = load(PACE_TX_LEAD=6, PACE_STEADY=1, PACE_EXACT=1, PACE_GAP_MS=400)
    call, steer = run_call(m, 26, m.FRAME_MS / 1000 * (1 - 100e-6), gap_at=10, gap_s=0.12, gap_burst=True)
    assert call.injected == 0
    assert 50e-6 < steer < 150e-6, f'steered {steer * 1e6:+.1f} ppm against a line 100 ppm fast'


def test_late_audio_after_a_long_gap_leaves_the_modem_on_the_lines_time():
    # 2026-11-25 call 18 (BinModem on a phone): 1 s with nothing from the
    # line, silence injected for the last 620 ms of it, then the late frames
    # all at once. Forwarding them on top of the silence put 31 frames into
    # the modem that the line never had, its PCM upstream framing slipped by
    # 4 samples, and no frame decoded again.
    m = load(PACE_TX_LEAD=6, PACE_STEADY=1, PACE_EXACT=1, PACE_GAP_MS=400)
    counts = {}
    call, _ = run_call(m, 10, m.FRAME_MS / 1000, gap_at=4, gap_s=1.0, gap_burst=True, counts=counts)
    assert call.injected > 20, 'the gap should have been filled with silence'
    assert abs(counts['modem_in'] - counts['slots']) <= 1, \
        f"modem had {counts['modem_in']} frames for {counts['slots']} slots of line"
    # Downstream: every silence sent for a modem frame that was late is
    # matched by that frame dropped, so the line's timeline does not move.
    late, dropped = counts['tx']
    assert abs(late - dropped) <= 1, f'tx_late {late} tx_dropped {dropped}'


def test_audio_lost_in_a_long_gap_leaves_the_modem_on_the_lines_time():
    # The same gap with the frames lost, not late: nothing comes after it but
    # the line's own cadence. The modem must still be handed a frame for
    # every slot, or it is behind the line for the rest of the call.
    m = load(PACE_TX_LEAD=6, PACE_STEADY=1, PACE_EXACT=1, PACE_GAP_MS=400)
    counts = {}
    call, _ = run_call(m, 10, m.FRAME_MS / 1000, gap_at=4, gap_s=1.0, counts=counts)
    assert abs(counts['modem_in'] - counts['slots']) <= 1, \
        f"modem had {counts['modem_in']} frames for {counts['slots']} slots of line"
    late, dropped = counts['tx']
    assert abs(late - dropped) <= 1, f'tx_late {late} tx_dropped {dropped}'
