"""Microsoft Point-to-Point Compression (RFC 2118), both halves.

An 8192-byte history shared by the compressor at one end and the
decompressor at the other; each packet goes out with a 16-bit header:

    A (0x8000)  the history was reset before this packet
    B (0x4000)  this packet was placed at the front of the history
    C (0x2000)  the packet is compressed
    D (0x1000)  encrypted (never, here)
    low 12 bits: the coherency count, one more each packet

The bit stream, most significant bit first:

    literal < 0x80:     0 + 7 bits
    literal >= 0x80:    10 + 7 bits
    offset 1..63:       1111 + 6 bits
    offset 64..319:     1110 + 8 bits (offset - 64)
    offset 320..8191:   110 + 13 bits (offset - 320)
    length 3:           0
    length 4..7:        10 + 2 bits;  8..15: 110 + 3 bits; ...
                        k ones and a zero, then k+1 bits, for 2^(k+1)..
                        up to 4096..8191: eleven ones, a zero, 12 bits

The packet ends at its last octet; the bits after the last token are zero,
fewer than eight of them, so they never read as a literal.
"""

HISTORY = 8192
FLAG_A, FLAG_B, FLAG_C, FLAG_D = 0x8000, 0x4000, 0x2000, 0x1000
COUNT_MASK = 0x0fff
MIN_MATCH, MAX_MATCH = 3, 8191
CHAIN = 24          # match candidates tried at each position


class Error(Exception):
    """A packet this decompressor cannot read."""


class _Bits:
    def __init__(self):
        self.out = bytearray()
        self.acc = 0
        self.n = 0

    def put(self, value, width):
        self.acc = (self.acc << width) | value
        self.n += width
        while self.n >= 8:
            self.n -= 8
            self.out.append((self.acc >> self.n) & 0xff)
        self.acc &= (1 << self.n) - 1

    def done(self):
        if self.n:
            self.out.append((self.acc << (8 - self.n)) & 0xff)
            self.acc = self.n = 0
        return bytes(self.out)


def _put_literal(bits, byte):
    if byte < 0x80:
        bits.put(byte, 8)
    else:
        bits.put(0b10 << 7 | (byte & 0x7f), 9)


def _put_copy(bits, offset, length):
    if offset < 64:
        bits.put(0b1111 << 6 | offset, 10)
    elif offset < 320:
        bits.put(0b1110 << 8 | (offset - 64), 12)
    else:
        bits.put(0b110 << 13 | (offset - 320), 16)
    if length == 3:
        bits.put(0, 1)
        return
    k = length.bit_length() - 2          # 4..7 -> 1, 8..15 -> 2, ...
    bits.put(((1 << k) - 1) << 1, k + 1)  # k ones, a zero
    bits.put(length - (1 << (k + 1)), k + 1)


class Compressor:
    def __init__(self):
        self.count = 0
        self.reset()
        self.plain = self.wire = 0

    def reset(self):
        """Start the history again; the next packet says so (A)."""
        self.hist = bytearray(HISTORY)
        self.pos = 0
        self.heads = {}
        self.flush_next = True

    def _match(self, start):
        """The longest earlier match for hist[start:], as (offset, length)."""
        h = bytes(self.hist[start:start + 3])
        best_len, best_off = 0, 0
        limit = min(MAX_MATCH, self._end - start)
        tried = 0
        for cand in reversed(self.heads.get(h, ())):
            if tried >= CHAIN:
                break
            tried += 1
            off = start - cand
            if off >= HISTORY:
                break
            n = 3
            while n < limit and self.hist[cand + n] == self.hist[start + n]:
                n += 1
            if n > best_len:
                best_len, best_off = n, off
                if n == limit:
                    break
        return best_off, best_len

    def _remember(self, at):
        if at + 3 <= self._end:
            key = bytes(self.hist[at:at + 3])
            chain = self.heads.setdefault(key, [])
            chain.append(at)
            if len(chain) > 64:
                del chain[:32]

    def compress(self, packet):
        """One packet out: the 2-byte header and what follows it."""
        flags = 0
        if self.flush_next:
            flags |= FLAG_A | FLAG_B
            self.hist = bytearray(HISTORY)
            self.pos = 0
            self.heads = {}
            self.flush_next = False
        if self.pos + len(packet) > HISTORY:
            # 2118: "the history buffer ... is reset to the beginning".
            flags |= FLAG_B
            self.pos = 0
            self.heads = {}
        start = self.pos
        self.hist[start:start + len(packet)] = packet
        self._end = start + len(packet)
        bits = _Bits()
        i = start
        while i < self._end:
            off, n = self._match(i) if self._end - i >= MIN_MATCH else (0, 0)
            if n >= MIN_MATCH:
                _put_copy(bits, off, n)
                for j in range(i, i + n):
                    self._remember(j)
                i += n
            else:
                _put_literal(bits, self.hist[i])
                self._remember(i)
                i += 1
        body = bits.done()
        count = self.count
        self.count = (self.count + 1) & COUNT_MASK
        self.plain += len(packet)
        if len(body) >= len(packet):
            # It grew: send it as it is, and start the history again both
            # here and (A) at the far end, on this packet and the next, so
            # the far end agrees whatever it does with an uncompressed one.
            self.hist = bytearray(HISTORY)
            self.pos = 0
            self.heads = {}
            self.flush_next = True
            out = (FLAG_A | FLAG_B | count).to_bytes(2, 'big') + bytes(packet)
        else:
            self.pos = self._end
            out = (flags | FLAG_C | count).to_bytes(2, 'big') + body
        self.wire += len(out)
        return out


class Decompressor:
    def __init__(self):
        self.hist = bytearray(HISTORY)
        self.pos = 0
        self.expect = None        # the next coherency count, once one is seen
        self.plain = self.wire = 0

    def decompress(self, data):
        """The packet a compressed datagram carried; Error if it cannot be
        read, after which nothing is right until a packet with A."""
        if len(data) < 2:
            raise Error('short')
        head = int.from_bytes(data[:2], 'big')
        body = data[2:]
        count = head & COUNT_MASK
        if head & FLAG_D:
            raise Error('encrypted')
        if head & FLAG_A:
            self.hist = bytearray(HISTORY)
            self.pos = 0
        elif self.expect is not None and count != self.expect:
            self.expect = None
            raise Error(f'coherency count {count}, expected another')
        elif self.expect is None:
            raise Error('waiting for a flushed packet')
        self.expect = (count + 1) & COUNT_MASK
        if head & FLAG_B:
            self.pos = 0
        if not head & FLAG_C:
            out = bytes(body)
            if self.pos + len(out) <= HISTORY:
                self.hist[self.pos:self.pos + len(out)] = out
                self.pos += len(out)
        else:
            out = self._decode(body)
        self.plain += len(out)
        self.wire += len(data)
        return out

    def _decode(self, body):
        start = self.pos
        hist = self.hist
        pos = self.pos
        total = len(body) * 8
        at = 0

        def get(width):
            nonlocal at
            if at + width > total:
                raise Error('ran off the end')
            v = 0
            for _ in range(width):
                v = (v << 1) | (body[at >> 3] >> (7 - (at & 7)) & 1)
                at += 1
            return v

        try:
            while total - at >= 8:
                if not get(1):
                    byte = get(7)
                    lit = True
                elif not get(1):
                    byte = 0x80 | get(7)
                    lit = True
                else:
                    lit = False
                if lit:
                    if pos >= HISTORY:
                        raise Error('history overflow')
                    hist[pos] = byte
                    pos += 1
                    continue
                # "11" read: offset class.
                if get(1):
                    offset = get(6) if get(1) else 64 + get(8)
                else:
                    offset = 320 + get(13)
                k = 0
                while get(1):
                    k += 1
                    if k > 11:
                        raise Error('length too long')
                length = 3 if k == 0 else (1 << (k + 1)) + get(k + 1)
                src = pos - offset
                if offset == 0 or src < 0 or pos + length > HISTORY:
                    raise Error('copy outside the history')
                for j in range(length):
                    hist[pos + j] = hist[src + j]
                pos += length
        except Error:
            self.expect = None
            raise
        self.pos = pos
        return bytes(hist[start:pos])
