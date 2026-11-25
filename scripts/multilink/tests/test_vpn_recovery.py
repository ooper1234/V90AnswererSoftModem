import importlib.util
from pathlib import Path
import unittest
from unittest.mock import Mock, patch

spec = importlib.util.spec_from_file_location('vpn_uplink', Path(__file__).parents[1] / 'vpn_uplink.py')
vpn = importlib.util.module_from_spec(spec)
spec.loader.exec_module(vpn)

class RecoveryTests(unittest.TestCase):
    def test_dead_primary_moves_to_backup_without_exiting_supervisor(self):
        attempts = []
        relays = [('192.0.2.1', 1194, 'udp', 'primary'), ('192.0.2.2', 443, 'tcp', 'backup')]
        def connect(profile):
            attempts.append((vpn.RELAY, profile))
            if profile == 'primary':
                raise RuntimeError('VPN handshake timed out')
            raise SystemExit(0)
        with patch.object(vpn, 'READY', Mock()), patch('builtins.print'):
            with self.assertRaises(SystemExit):
                vpn.maintain_relays(relays, connect, lambda _: None)
        self.assertEqual(attempts, [('192.0.2.1', 'primary'), ('192.0.2.2', 'backup')])

    def test_exhausted_relays_retry_without_restarting_ppp(self):
        pauses = []
        attempts = []
        def connect(profile):
            attempts.append(profile)
            raise OSError('unreachable')
        def pause(seconds):
            pauses.append(seconds)
            if seconds == 15:
                raise SystemExit(0)
        ready = Mock()
        with patch.object(vpn, 'READY', ready), patch('builtins.print'):
            with self.assertRaises(SystemExit):
                vpn.maintain_relays([('192.0.2.1', 1194, 'udp', 'first'),
                                     ('192.0.2.2', 1194, 'udp', 'second')], connect, pause)
        self.assertEqual(attempts, ['first', 'second'])
        self.assertEqual(pauses, [3, 3, 15])
        self.assertEqual(ready.unlink.call_count, 2)

    def test_all_transport_exceptions_precede_output_rejection(self):
        calls = []
        def run(*args):
            calls.append(args)
            return 'default via 172.17.0.1 dev eth0' if args[:5] == ('ip', '-4', 'route', 'show', 'default') else ''
        with patch.object(vpn, 'run', run), patch.object(vpn, 'Path'):
            vpn.configure_network([('192.0.2.1', 1194, 'udp', 'first'),
                                   ('192.0.2.2', 443, 'tcp', 'second')])
        rejection = calls.index(('iptables', '-A', 'OUTPUT', '-o', 'eth0', '-j', 'REJECT'))
        for relay, port, proto in [('192.0.2.1', '1194', 'udp'), ('192.0.2.2', '443', 'tcp')]:
            exception = ('iptables', '-A', 'OUTPUT', '-o', 'eth0', '-d', relay + '/32',
                         '-p', proto, '--dport', port, '-j', 'ACCEPT')
            self.assertLess(calls.index(exception), rejection)
        self.assertIn(('iptables', '-A', 'FORWARD', '-j', 'REJECT'), calls)
        self.assertFalse(any(args[:3] == ('ip', 'route', 'replace') and args[3] == 'default' for args in calls))

if __name__ == '__main__':
    unittest.main()