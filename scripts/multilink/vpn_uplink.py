#!/usr/bin/python3
"""VPN Gate egress in the dedicated PPP namespace; never change host routes."""
from pathlib import Path
import signal
import subprocess
import time

READY = Path('/run/v90-uplink-ready')
RELAY = '27.130.93.85'


def run(*args):
    return subprocess.check_output(args, text=True).strip()


def main():
    READY.unlink(missing_ok=True)
    routes = run('ip', '-4', 'route', 'show', 'default').split()
    gateway = routes[routes.index('via') + 1]
    device = routes[routes.index('dev') + 1]
    # Preserve only the VPN transport and Docker control network outside VPN.
    run('ip', 'route', 'replace', RELAY + '/32', 'via', gateway, 'dev', device)
    run('iptables', '-A', 'OUTPUT', '-o', device, '-d', RELAY + '/32',
        '-p', 'udp', '--dport', '1233', '-j', 'ACCEPT')
    run('iptables', '-A', 'OUTPUT', '-o', device, '-d', '10.77.90.0/24', '-j', 'ACCEPT')
    run('iptables', '-A', 'OUTPUT', '-o', device, '-j', 'REJECT')
    run('iptables', '-A', 'FORWARD', '-s', '10.0.2.0/24', '-o', 'tun0', '-j', 'ACCEPT')
    run('iptables', '-A', 'FORWARD', '-i', 'tun0', '-d', '10.0.2.0/24',
        '-m', 'conntrack', '--ctstate', 'ESTABLISHED,RELATED', '-j', 'ACCEPT')
    run('iptables', '-A', 'FORWARD', '-j', 'REJECT')
    run('iptables', '-t', 'nat', '-A', 'POSTROUTING', '-s', '10.0.2.0/24',
        '-o', 'tun0', '-j', 'MASQUERADE')
    # Avoid Docker's DNS forwarder using the home uplink for proxy requests.
    Path('/etc/resolv.conf').write_text('nameserver 1.1.1.1\n')
    log = Path('/var/log/v90-multilink/vpn.log')
    log.parent.mkdir(parents=True, exist_ok=True)
    with log.open('w') as output:
        child = subprocess.Popen(['openvpn', '--config', '/opt/v90/vpn/vpngate.ovpn',
            '--dev', 'tun0', '--route-nopull', '--script-security', '0',
            '--verb', '3', '--auth-nocache', '--connect-retry-max', '1', '--tls-exit'],
            stdout=output, stderr=subprocess.STDOUT)
        def stop(*_):
            child.terminate()
            raise SystemExit(0)
        signal.signal(signal.SIGTERM, stop)
        signal.signal(signal.SIGINT, stop)
        try:
            deadline = time.monotonic() + 35
            while time.monotonic() < deadline:
                if child.poll() is not None:
                    raise RuntimeError('VPN exited before readiness')
                if 'Initialization Sequence Completed' in log.read_text():
                    break
                time.sleep(0.25)
            else:
                raise RuntimeError('VPN handshake timed out')
            run('ip', 'route', 'replace', 'default', 'dev', 'tun0')
            # Old Wi-Fi table must not override the VPN default.
            subprocess.run(['ip', 'rule', 'del', 'priority', '190'],
                           stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            trace = run('curl', '--interface', 'tun0', '--noproxy', '*', '--fail',
                        '--silent', '--show-error', '--max-time', '12',
                        'https://1.1.1.1/cdn-cgi/trace')
            if 'ip=' + RELAY not in trace.splitlines():
                raise RuntimeError('VPN exit verification failed')
            READY.touch()
            print('VPN egress verified: ' + RELAY, flush=True)
            next_probe = time.monotonic() + 30
            failures = 0
            reconnect_after = 0.0
            missing_since = None
            while child.poll() is None:
                now = time.monotonic()
                if not Path('/sys/class/net/tun0').exists():
                    READY.unlink(missing_ok=True)
                    if missing_since is None:
                        missing_since = now
                    if now - missing_since >= 45:
                        raise RuntimeError('VPN interface did not recover')
                    time.sleep(1)
                    continue
                missing_since = None
                # A pushed keepalive restart may recreate tun0 and remove
                # its route even when the reconnect happens between polls.
                route = subprocess.run(['ip', 'route', 'replace', 'default', 'dev', 'tun0'],
                                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                if route.returncode:
                    READY.unlink(missing_ok=True)
                    time.sleep(1)
                    continue
                if now >= next_probe:
                    try:
                        trace = run('curl', '--interface', 'tun0', '--noproxy', '*',
                                    '--fail', '--silent', '--show-error', '--max-time', '6',
                                    'https://1.1.1.1/cdn-cgi/trace')
                        healthy = 'ip=' + RELAY in trace.splitlines()
                    except subprocess.CalledProcessError:
                        healthy = False
                    next_probe = time.monotonic() + 30
                    if healthy:
                        failures = 0
                        READY.touch()
                    else:
                        READY.unlink(missing_ok=True)
                        failures += 1
                        if failures >= 2 and now >= reconnect_after:
                            # Restart only the VPN transport; existing PPP
                            # processes stay alive while its interface recovers.
                            child.send_signal(signal.SIGUSR1)
                            reconnect_after = now + 120
                            failures = 0
                            print('VPN egress stalled: reconnecting transport', flush=True)
                time.sleep(1)
            raise RuntimeError('VPN stopped')
        finally:
            READY.unlink(missing_ok=True)
            if child.poll() is None:
                child.terminate()
            try:
                child.wait(timeout=5)
            except subprocess.TimeoutExpired:
                child.kill()
                child.wait()


if __name__ == '__main__':
    main()
