"""Check corruption handling and arbitrary stream boundaries in the uplink."""
import importlib.util
from pathlib import Path
import sys
import unittest
from unittest.mock import patch

directory = Path(__file__).resolve().parents[1] / 'scripts' / 'multilink'
sys.path.insert(0, str(directory))
from ppp_wire import Decoder, frame
from server import admission_allowed, ppp_arguments


class Admission(unittest.TestCase):
    def test_vpn_outage_preserves_ppp_only_after_firewall_configuration(self):
        config = {'allowed_clients': ['10.77.90.2']}
        self.assertTrue(admission_allowed('10.77.90.2', config, 0,
                                         'vpngate', False, True))
        self.assertFalse(admission_allowed('10.77.90.2', config, 0,
                                          'vpngate', True, False))
        self.assertFalse(admission_allowed('10.77.90.9', config, 0,
                                          'vpngate', False, True))
        self.assertFalse(admission_allowed('10.77.90.2', config, 4,
                                          'vpngate', False, True))

    def test_wifi_still_requires_ready_uplink(self):
        config = {'allowed_clients': ['10.77.90.2']}
        self.assertFalse(admission_allowed('10.77.90.2', config, 0,
                                          'wifi', False, True))
        self.assertTrue(admission_allowed('10.77.90.2', config, 0,
                                         'wifi', True, False))


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

    def test_vj_protocols_survive_stream_boundaries_and_corruption(self):
        packets = [(0x2d, b'\x80\x7e\x7d\x00'), (0x2f, bytes(range(64)))]
        damaged = bytearray(frame(0x2d, b'bad context'))
        damaged[-3] ^= 1
        wire = bytes(damaged) + b''.join(frame(p, data) for p, data in packets)
        decoder = Decoder()
        recovered = []
        for byte in wire:
            recovered += decoder.feed(bytes([byte]))
        self.assertEqual(recovered, packets)

    def test_vj_negotiation_can_be_disabled(self):
        with patch.dict('os.environ', {'V90_PPP_VJ': '1'}):
            self.assertNotIn('novj', ppp_arguments())
        with patch.dict('os.environ', {'V90_PPP_VJ': '0'}):
            self.assertIn('novj', ppp_arguments())


if __name__ == '__main__':
    unittest.main()
