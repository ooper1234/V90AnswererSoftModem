import os, struct, sys
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import ccp_shim as cs
import mppc


def frames(data):
    return [f for _, f in cs.Deframer().feed(data) if f]


def ccp(code, ident, body=b''):
    return cs.build_frame(cs.PROTO_CCP, cs.control(code, ident, body))


def mppc_req(bits):
    return cs.option_bytes([(cs.MPPC_OPTION, bits.to_bytes(4, 'big'))])


IP = cs.build_frame(0x0021, b'\x45\x00' + b'GET /index.html HTTP/1.0 ' * 20, accm=0)
LCP_REQ = cs.build_frame(cs.PROTO_LCP, cs.control(1, 1, cs.option_bytes([(2, b'\0\0\0\0'), (5, b'\x12\x34\x56\x78')])))
LCP_ACK_FROM_PPPD = cs.build_frame(cs.PROTO_LCP, cs.control(2, 1, cs.option_bytes([(2, b'\0\0\0\0'), (5, b'\x12\x34\x56\x78')])))


def opened(log=None):
    """A shim that a Windows-like caller has agreed MPPC with."""
    s = cs.Shim(log=log or (lambda *a: None), clock=lambda: 0.0)
    s.from_peer(LCP_REQ)
    s.from_pppd(LCP_ACK_FROM_PPPD)
    to_pppd, to_peer = s.from_peer(ccp(cs.CONF_REQ, 7, mppc_req(1)))
    assert to_pppd.strip(b'~') == b'', to_pppd   # a bare flag between frames is nothing
    replies = [cs.Deframer() and f for f in frames(to_peer)]
    codes = [(p, pl[0]) for p, pl in replies]
    assert codes == [(cs.PROTO_CCP, cs.CONF_ACK), (cs.PROTO_CCP, cs.CONF_REQ)], codes
    our_req = replies[1][1]
    assert our_req[4:] == mppc_req(1)
    s.from_peer(ccp(cs.CONF_ACK, our_req[1], our_req[4:]))
    assert s.open
    return s


def test_a_caller_that_never_asks_for_mppc_gets_every_byte_through_unchanged():
    s = cs.Shim(log=lambda *a: None)
    other_ccp = ccp(cs.CONF_REQ, 1, cs.option_bytes([(26, b'\x0f\x08')]))   # deflate
    for chunk in (b'\x7e', LCP_REQ, IP[:30], IP[30:], other_ccp, b'junk before', IP):
        assert s.from_peer(chunk)[1] == b''
    stream = LCP_REQ + IP + other_ccp + b'junk before' + IP
    s2 = cs.Shim(log=lambda *a: None)
    got = b''.join(s2.from_peer(stream[i:i + 7])[0] for i in range(0, len(stream), 7))
    assert got == stream
    s3 = cs.Shim(log=lambda *a: None)
    assert s3.from_pppd(LCP_ACK_FROM_PPPD + IP) == LCP_ACK_FROM_PPPD + IP


def test_windows_asking_for_mppc_gets_it_both_ways():
    s = opened()
    # pppd -> caller: compressed, and the caller's decompressor reads it.
    out = s.from_pppd(IP)
    (proto, payload), = frames(out)
    assert proto == cs.PROTO_MPPC
    d = mppc.Decompressor()
    inner = d.decompress(payload)
    (ip_proto, ip_payload), = frames(IP)
    assert inner == struct.pack('!H', ip_proto) + ip_payload
    assert len(out) < len(IP) * 0.6
    # The caller's ACCM of 0 was taken: no control octet escaped needlessly.
    assert b'\x7d\x20' not in out[1:-1] or 0x00 not in payload
    # caller -> pppd: decompressed back into a plain frame.
    c = mppc.Compressor()
    wire = cs.build_frame(cs.PROTO_MPPC, c.compress(inner))
    to_pppd, _ = s.from_peer(wire)
    assert frames(to_pppd) == [(ip_proto, ip_payload)]
    a, b, w, p = s.counts()
    assert a > b and p == len(inner)


def test_encryption_bits_are_answered_with_mppc_alone():
    s = cs.Shim(log=lambda *a: None, clock=lambda: 0.0)
    _, to_peer = s.from_peer(ccp(cs.CONF_REQ, 3, mppc_req(0x01000061)))
    first = frames(to_peer)[0][1]
    assert first[0] == cs.CONF_NAK and first[4:] == mppc_req(1)


def test_other_options_beside_mppc_are_rejected():
    s = cs.Shim(log=lambda *a: None, clock=lambda: 0.0)
    _, to_peer = s.from_peer(ccp(cs.CONF_REQ, 3, mppc_req(1) + cs.option_bytes([(17, b'\x01\x02')])))
    first = frames(to_peer)[0][1]
    assert first[0] == cs.CONF_REJ and first[4:] == cs.option_bytes([(17, b'\x01\x02')])


def test_a_reset_request_flushes_the_next_packet():
    s = opened()
    s.from_pppd(IP)
    s.from_peer(ccp(cs.RESET_REQ, 9))
    (_, payload), = frames(s.from_pppd(IP))
    assert payload[0] & 0x80, 'no FLUSHED bit after a Reset-Request'


def test_a_damaged_compressed_packet_asks_for_a_reset_and_waits_for_a_flush():
    s = opened()
    c = mppc.Compressor()
    packet = b'\x00\x21' + b'hello hello hello hello'
    s.from_peer(cs.build_frame(cs.PROTO_MPPC, c.compress(packet)))
    c.compress(packet)                                   # lost on the way
    to_pppd, to_peer = s.from_peer(cs.build_frame(cs.PROTO_MPPC, c.compress(packet)))
    assert to_pppd.strip(b'~') == b'', to_pppd   # a bare flag between frames is nothing
    assert [pl[0] for _, pl in frames(to_peer)] == [cs.RESET_REQ]
    c.reset()
    to_pppd, _ = s.from_peer(cs.build_frame(cs.PROTO_MPPC, c.compress(packet)))
    assert frames(to_pppd) == [(0x21, b'hello hello hello hello')]


def test_a_caller_rejecting_mppc_leaves_ccp_to_pppd():
    s = cs.Shim(log=lambda *a: None, clock=lambda: 0.0)
    _, to_peer = s.from_peer(ccp(cs.CONF_REQ, 3, mppc_req(1)))
    req = frames(to_peer)[1][1]
    s.from_peer(ccp(cs.CONF_REJ, req[1], req[4:]))
    again = ccp(cs.CONF_REQ, 4, mppc_req(1))
    to_pppd, to_peer = s.from_peer(again)
    assert to_pppd == again and to_peer == b''
    assert s.from_pppd(IP) == IP


def test_an_unanswered_request_is_sent_again_then_given_up():
    now = [0.0]
    s = cs.Shim(log=lambda *a: None, clock=lambda: now[0])
    s.from_peer(ccp(cs.CONF_REQ, 3, mppc_req(1)))
    sent = 0
    for _ in range(40):
        now[0] += 3.1
        if s.tick():
            sent += 1
    assert sent == cs.RESEND_MAX - 1
    assert s.given_up
