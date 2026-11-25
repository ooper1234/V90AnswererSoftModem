"""MPPC between the caller and pppd, which runs with noccp.

The caller's byte stream goes through here on its way to pppd and back.
Nothing is changed until the caller asks for MPPC in a CCP
Configure-Request (Windows does, when "Enable software compression" is
ticked). From then this end answers CCP itself: it agrees MPPC, without
encryption, both ways; compresses what pppd sends the caller and
decompresses what the caller sends, so pppd only ever sees plain packets.
A caller that never asks for MPPC gets every byte through as before, and
pppd's noccp answers its CCP, if any, with a Protocol-Reject as it always
did.

Framing is RFC 1662's asynchronous HDLC. Frames this end builds escape what
the caller's LCP asked for (its ACCM, as pppd acknowledged it), and always
carry the address/control field and a two-octet protocol, which every PPP
receiver accepts.
"""
import struct
import time

import mppc

PROTO_LCP, PROTO_CCP, PROTO_MPPC = 0xc021, 0x80fd, 0x00fd
MPPC_OPTION, MPPC_BIT = 18, 0x00000001
CONF_REQ, CONF_ACK, CONF_NAK, CONF_REJ, TERM_REQ, TERM_ACK, CODE_REJ = 1, 2, 3, 4, 5, 6, 7
RESET_REQ, RESET_ACK = 14, 15
RESEND_S, RESEND_MAX = 3.0, 10


def fcs16(data):
    value = 0xffff
    for byte in data:
        value ^= byte
        for _ in range(8):
            value = (value >> 1) ^ (0x8408 if value & 1 else 0)
    return value


def build_frame(protocol, payload, accm=0xffffffff):
    data = b'\xff\x03' + struct.pack('!H', protocol) + payload
    data += struct.pack('<H', fcs16(data) ^ 0xffff)
    wire = bytearray(b'\x7e')
    for byte in data:
        if byte in (0x7d, 0x7e) or (byte < 32 and accm >> byte & 1):
            wire.extend((0x7d, byte ^ 0x20))
        else:
            wire.append(byte)
    wire.append(0x7e)
    return bytes(wire)


def parse_frame(raw):
    """(protocol, payload) of an unescaped frame body with its FCS, or None."""
    if len(raw) < 4 or fcs16(raw) != 0xf0b8:
        return None
    data = raw[:-2]
    if data[:2] == b'\xff\x03':
        data = data[2:]
    if not data:
        return None
    if data[0] & 1:
        return data[0], data[1:]
    if len(data) < 2:
        return None
    return struct.unpack('!H', data[:2])[0], data[2:]


class Deframer:
    """Splits a byte stream into frames, keeping each one's raw bytes.

    feed() returns a list of (raw, frame): raw is exactly the bytes that came,
    flags included, so a frame passed on unread goes out as it arrived; frame
    is (protocol, payload) or None for anything that is not a good frame
    (bytes before the first flag, a bad FCS)."""

    def __init__(self):
        self.raw = bytearray()
        self.body = bytearray()
        self.escaped = False

    def feed(self, chunk):
        out = []
        for byte in chunk:
            if byte == 0x7e:
                if self.body or self.raw:
                    self.raw.append(byte)
                    frame = parse_frame(bytes(self.body)) if not self.escaped else None
                    out.append((bytes(self.raw), frame))
                    self.raw = bytearray()
                else:
                    self.raw.append(byte)   # a lone flag opens the next frame
                    out.append((bytes(self.raw), None))
                    self.raw = bytearray()
                self.body = bytearray()
                self.escaped = False
                continue
            self.raw.append(byte)
            if byte == 0x7d:
                self.escaped = True
            else:
                self.body.append(byte ^ 0x20 if self.escaped else byte)
                self.escaped = False
            if len(self.raw) > 20000:
                out.append((bytes(self.raw), None))
                self.raw, self.body, self.escaped = bytearray(), bytearray(), False
        return out


def options(data):
    """CCP/LCP options as (type, value) pairs; None if malformed."""
    out, i = [], 0
    while i < len(data):
        if i + 2 > len(data) or data[i + 1] < 2 or i + data[i + 1] > len(data):
            return None
        out.append((data[i], data[i + 2:i + data[i + 1]]))
        i += data[i + 1]
    return out


def option_bytes(pairs):
    return b''.join(bytes((t, len(v) + 2)) + v for t, v in pairs)


def control(code, ident, body=b''):
    return struct.pack('!BBH', code, ident, 4 + len(body)) + body


MPPC_ONLY = option_bytes([(MPPC_OPTION, MPPC_BIT.to_bytes(4, 'big'))])


class Shim:
    """The two directions' work. Not thread-safe: the caller holds a lock."""

    def __init__(self, log=print, clock=time.monotonic):
        self.log, self.clock = log, clock
        self.peer_in = Deframer()
        self.pppd_in = Deframer()
        self.peer_accm = 0xffffffff       # what the caller wants escaped
        self.pending_accm = None
        self.reset_state()
        self.given_up = False

    def reset_state(self):
        self.active = False               # this end is answering CCP
        self.acked_theirs = False          # we compress toward the caller
        self.ours_acked = False            # the caller compresses toward us
        self.our_id = 0
        self.our_req_at = None
        self.our_tries = 0
        self.comp = mppc.Compressor()
        self.decomp = mppc.Decompressor()
        self.awaiting_flush = False

    @property
    def open(self):
        return self.active and self.acked_theirs and self.ours_acked

    # caller -> pppd ------------------------------------------------------

    def from_peer(self, chunk):
        """Bytes from the caller: (for pppd, for the caller)."""
        to_pppd, to_peer = bytearray(), bytearray()
        for raw, frame in self.peer_in.feed(chunk):
            if frame is None:
                to_pppd += raw
                continue
            proto, payload = frame
            if proto == PROTO_LCP:
                self._lcp_seen(payload, from_peer=True)
                to_pppd += raw
            elif proto == PROTO_CCP and self._wants_ccp(payload):
                to_peer += self._ccp_from_peer(payload)
            elif proto == PROTO_MPPC and self.active:
                packet = self._decompress(payload, to_peer)
                if packet is not None:
                    to_pppd += self._inner_to_pppd(packet)
            else:
                to_pppd += raw
        return bytes(to_pppd), bytes(to_peer)

    def _wants_ccp(self, payload):
        if self.given_up:
            return False
        if self.active:
            return True
        if len(payload) >= 4 and payload[0] == CONF_REQ:
            opts = options(payload[4:struct.unpack('!H', payload[2:4])[0]])
            if opts and any(t == MPPC_OPTION for t, _ in opts):
                self.active = True
                self.log('mppc: the caller asked for MPPC; answering CCP here')
                return True
        return False

    def _ccp_from_peer(self, payload):
        if len(payload) < 4:
            return b''
        code, ident, length = struct.unpack('!BBH', payload[:4])
        body = payload[4:length]
        out = bytearray()
        if code == CONF_REQ:
            opts = options(body) or []
            others = [(t, v) for t, v in opts if t != MPPC_OPTION]
            mine = [v for t, v in opts if t == MPPC_OPTION]
            if others:
                out += self._ccp(CONF_REJ, ident, option_bytes(others))
            elif not mine or int.from_bytes(mine[0], 'big') != MPPC_BIT:
                # Encryption bits or none at all: MPPC alone is what this end does.
                out += self._ccp(CONF_NAK, ident, MPPC_ONLY)
            else:
                out += self._ccp(CONF_ACK, ident, body)
                if not self.acked_theirs:
                    self.comp = mppc.Compressor()
                self.acked_theirs = True
            if self.our_req_at is None and not self.ours_acked:
                out += self._send_request()
        elif code == CONF_ACK and ident == self.our_id:
            self.ours_acked = True
            self.our_req_at = None
            self.decomp = mppc.Decompressor()
            self.awaiting_flush = False
        elif code == CONF_NAK and ident == self.our_id:
            opts = options(body) or []
            asked = [v for t, v in opts if t == MPPC_OPTION]
            if asked and int.from_bytes(asked[0], 'big') & MPPC_BIT:
                out += self._send_request()   # MPPC alone again
            else:
                self._give_up('the caller will not take MPPC alone')
        elif code in (CONF_REJ, CODE_REJ):
            self._give_up('the caller rejected MPPC')
        elif code == TERM_REQ:
            out += self._ccp(TERM_ACK, ident)
            self.log('mppc: the caller closed CCP')
            self.reset_state()
        elif code == RESET_REQ:
            # 2118: flush, and the next packet carries the FLUSHED bit; no
            # Reset-Ack.
            self.comp.reset()
        if self.open and not getattr(self, '_said_open', False):
            self._said_open = True
            self.log('mppc: CCP open, MPPC both ways')
        return bytes(out)

    def _give_up(self, why):
        self.log(f'mppc: {why}; leaving CCP to pppd')
        self.reset_state()
        self.given_up = True

    def _send_request(self):
        self.our_id = (self.our_id + 1) & 0xff
        self.our_req_at = self.clock()
        self.our_tries += 1
        return self._ccp(CONF_REQ, self.our_id, MPPC_ONLY)

    def _ccp(self, code, ident, body=b''):
        return build_frame(PROTO_CCP, control(code, ident, body), self.peer_accm)

    def _decompress(self, payload, to_peer):
        if self.awaiting_flush and not (len(payload) >= 2 and payload[0] & 0x80):
            return None
        try:
            packet = self.decomp.decompress(payload)
        except mppc.Error as e:
            if not self.awaiting_flush:
                head = int.from_bytes(payload[:2], 'big') if len(payload) >= 2 else 0
                flags = ''.join(f for f, b in (('A', 0x8000), ('B', 0x4000), ('C', 0x2000), ('D', 0x1000)) if head & b)
                self.log(f'mppc: a packet from the caller would not decompress ({e}); Reset-Request '
                         f'[flags {flags or "-"}, count {head & 0xfff}, {len(payload)} bytes, history at {self.decomp.pos}, '
                         f'head {payload[:24].hex()}]')
                self.our_id = (self.our_id + 1) & 0xff
                to_peer += self._ccp(RESET_REQ, self.our_id)
            self.awaiting_flush = True
            return None
        self.awaiting_flush = False
        return packet

    def _inner_to_pppd(self, packet):
        if not packet:
            return b''
        if packet[0] & 1:
            proto, payload = packet[0], packet[1:]
        elif len(packet) >= 2:
            proto, payload = struct.unpack('!H', packet[:2])[0], packet[2:]
        else:
            return b''
        return build_frame(proto, payload)     # every control octet escaped: pppd takes it

    # pppd -> caller ------------------------------------------------------

    def from_pppd(self, chunk):
        """Bytes from pppd, for the caller."""
        out = bytearray()
        for raw, frame in self.pppd_in.feed(chunk):
            if frame is None:
                out += raw
                continue
            proto, payload = frame
            if proto == PROTO_LCP:
                self._lcp_seen(payload, from_peer=False)
                out += raw
            elif self.open and 0x0021 <= proto <= 0x3fff and proto != PROTO_MPPC:
                packet = struct.pack('!H', proto) + payload
                out += build_frame(PROTO_MPPC, self.comp.compress(packet), self.peer_accm)
            else:
                out += raw
        return bytes(out)

    def tick(self):
        """Resend our Configure-Request while it goes unanswered."""
        if not self.active or self.ours_acked or self.our_req_at is None:
            return b''
        if self.clock() - self.our_req_at < RESEND_S:
            return b''
        if self.our_tries >= RESEND_MAX:
            self._give_up('no answer to our CCP request')
            return b''
        return self._send_request()

    # LCP: the ACCM, and a renegotiation closes CCP ------------------------

    def _lcp_seen(self, payload, from_peer):
        if len(payload) < 4:
            return
        code = payload[0]
        length = struct.unpack('!H', payload[2:4])[0]
        if code == CONF_REQ:
            opts = options(payload[4:length]) or []
            if from_peer:
                accm = [v for t, v in opts if t == 2 and len(v) == 4]
                self.pending_accm = int.from_bytes(accm[0], 'big') if accm else 0xffffffff
            if self.active:
                self.log('mppc: LCP renegotiating; CCP closed')
                self.reset_state()
                self.given_up = False
        elif code == CONF_ACK and not from_peer and self.pending_accm is not None:
            self.peer_accm = self.pending_accm

    def counts(self):
        return (self.comp.plain, self.comp.wire, self.decomp.wire, self.decomp.plain)
