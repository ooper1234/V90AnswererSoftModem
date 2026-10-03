"""Check corruption handling and arbitrary stream boundaries in the uplink."""
import importlib.util
from pathlib import Path
import sys
import unittest

directory = Path(__file__).resolve().parents[1] / 'scripts' / 'multilink'
sys.path.insert(0, str(directory))
from ppp_wire import Decoder, frame
from server import ppp_arguments


class Framing(unittest.TestCase):
    def test_all_bytes_and_stream_boundaries(self):
        payload = bytes(range(256)) * 6
        encoded = frame(0x21, payload) + frame(0xc021, b'\x01\x02\x00\x04')
        for size in (1, 3, 17, 4096):
            decoder = Decoder()
            packets = []
            for start in range(0, len(encoded), size):
                packets += decoder.feed(encoded[start:start + size])
            self.assertEqual(packets, [(0x21, payload), (0xc021, b'\x01\x02\x00\x04')])

    def test_corrupt_oversize_and_aborted_frames_are_discarded(self):
        encoded = bytearray(frame(0x21, b'hello world'))
        encoded[-3] ^= 1
        wire = bytes(encoded) + b'\x7e' + b'a' * 9000 + b'\x7e\x7d\x7e' + frame(0x21, b'intact')
        self.assertEqual(Decoder().feed(wire), [(0x21, b'intact')])

    def test_shared_bundle_and_no_default_route(self):
        args = ppp_arguments()
        self.assertIn('multilink', args)
        self.assertIn('nodefaultroute', args)
        self.assertEqual(args[args.index('bundle') + 1], 'v90-windows')
        self.assertIn('hide-password', args)


if __name__ == '__main__':
    unittest.main()
