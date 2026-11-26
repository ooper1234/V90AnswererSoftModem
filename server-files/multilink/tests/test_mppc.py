import os, random, sys
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import mppc


def roundtrip(packets):
    c, d = mppc.Compressor(), mppc.Decompressor()
    for p in packets:
        assert d.decompress(c.compress(p)) == p
    return c


def test_text_compresses_and_comes_back():
    page = b'<html><head><title>Hello</title></head><body>' + b'<p>the same paragraph again</p>' * 40 + b'</body></html>'
    c = roundtrip([page[i:i + 1400] for i in range(0, len(page), 1400)])
    assert c.wire * 3 < c.plain, (c.plain, c.wire)


def test_every_byte_value_and_both_literal_forms():
    roundtrip([bytes(range(256)), bytes(range(255, -1, -1)) * 3])


def test_random_data_goes_uncompressed_and_the_next_still_reads():
    r = random.Random(1)
    noise = bytes(r.randrange(256) for _ in range(1400))
    c, d = mppc.Compressor(), mppc.Decompressor()
    for p in (b'abc' * 100, noise, b'abc' * 100, noise, b'hello hello hello hello'):
        w = c.compress(p)
        if p is noise:
            assert not int.from_bytes(w[:2], 'big') & mppc.FLAG_C
        assert d.decompress(w) == p


def test_history_wraps_to_the_front():
    r = random.Random(2)
    words = [bytes(r.choice(b'abcdefgh') for _ in range(r.randrange(3, 9))) for _ in range(200)]
    packets = [b' '.join(r.choice(words) for _ in range(250))[:1500] for _ in range(40)]
    c = roundtrip(packets)          # 40 x 1500 is well past 8192
    assert c.wire < c.plain


def test_long_matches_use_every_length_class():
    for n in (3, 4, 7, 8, 15, 16, 31, 63, 64, 127, 128, 255, 511, 1023, 2047, 3990):
        p = b'xyz' + b'Q' * n + b'xyz' + b'Q' * n
        roundtrip([p])


def test_far_offsets_in_each_class():
    r = random.Random(3)
    for gap in (1, 60, 63, 64, 300, 319, 320, 5000, 8000):
        filler = bytes(r.randrange(256) for _ in range(gap))
        roundtrip([b'MARKER!!' + filler + b'MARKER!!'])


def test_a_lost_packet_is_noticed_and_a_flushed_one_recovers():
    c, d = mppc.Compressor(), mppc.Decompressor()
    d.decompress(c.compress(b'first packet first packet'))
    c.compress(b'lost lost lost lost lost lost lost')   # never arrives
    try:
        d.decompress(c.compress(b'third third third third third'))
        assert False, 'a gap in the count went unnoticed'
    except mppc.Error:
        pass
    c.reset()                                       # what a Reset-Request asks
    assert d.decompress(c.compress(b'after the reset')) == b'after the reset'


def test_a_known_stream_decodes():
    # Built by hand from RFC 2118's tables: "ab" literal, then a copy of
    # offset 2 length 4 ("abab"), then 0xff literal; A|B|C, count 0.
    bits = '0' + '1100001' + '0' + '1100010' + '1111' + '000010' + '10' + '00' + '10' + '1111111'
    bits += '0' * (-len(bits) % 8)
    body = bytes(int(bits[i:i + 8], 2) for i in range(0, len(bits), 8))
    head = (mppc.FLAG_A | mppc.FLAG_B | mppc.FLAG_C).to_bytes(2, 'big')
    assert mppc.Decompressor().decompress(head + body) == b'ababab\xff'
