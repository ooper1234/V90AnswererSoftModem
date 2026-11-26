import os, socket, sys, threading, time, types
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.modules.setdefault('telnet_door', types.ModuleType('telnet_door'))
import ccp_shim as cs, mppc, server


def test_the_relay_carries_mppc_both_ways_and_ends_with_pppd():
    caller, stream = socket.socketpair()
    inner, pppd = socket.socketpair()
    done = threading.Event()
    t = threading.Thread(target=server.mppc_relay, args=(stream, inner, done, 'test'), daemon=True)
    t.start()

    def fake_pppd():      # echoes every IP packet, as a route back would
        d = cs.Deframer()
        while True:
            data = pppd.recv(4096)
            if not data:
                return
            for _, f in d.feed(data):
                if f and f[0] == 0x21:
                    pppd.sendall(cs.build_frame(0x21, f[1]))
    threading.Thread(target=fake_pppd, daemon=True).start()

    d = cs.Deframer(); got = []
    def read_until(n, secs=5):
        caller.settimeout(0.2); end = time.time() + secs
        while len(got) < n and time.time() < end:
            try: got.extend(f for _, f in d.feed(caller.recv(65536)) if f)
            except socket.timeout: pass
    caller.sendall(cs.build_frame(cs.PROTO_CCP, cs.control(1, 1, cs.option_bytes([(18, b'\0\0\0\1')]))))
    read_until(2)
    (_, ack), (_, req) = got[:2]
    assert ack[0] == cs.CONF_ACK and req[0] == cs.CONF_REQ
    caller.sendall(cs.build_frame(cs.PROTO_CCP, cs.control(2, req[1], req[4:])))
    comp, decomp = mppc.Compressor(), mppc.Decompressor()
    ip = b'\x45' + b'some page text, some page text, ' * 30
    caller.sendall(cs.build_frame(cs.PROTO_MPPC, comp.compress(b'\x00\x21' + ip)))
    read_until(3)
    proto, payload = got[2]
    assert proto == cs.PROTO_MPPC and decomp.decompress(payload) == b'\x00\x21' + ip
    pppd.shutdown(socket.SHUT_RDWR); pppd.close()   # pppd exits: the relay ends, the caller's stream stays
    t.join(3)
    assert not t.is_alive()
    caller.sendall(b'still open'); assert stream.recv(64) == b'still open'
