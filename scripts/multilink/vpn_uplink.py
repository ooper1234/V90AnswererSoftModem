#!/usr/bin/python3
"""VPN Gate egress in the dedicated PPP namespace; never change host routes."""
from pathlib import Path
import signal
import subprocess
import time
import os
import ipaddress
import json

READY = Path('/run/v90-uplink-ready')
RELAY = str(ipaddress.IPv4Address(os.environ.get('V90_VPN_RELAY', '27.130.93.85')))
RELAY_PORT = int(os.environ.get('V90_VPN_PORT', '1233'))
RELAY_PROTO = os.environ.get('V90_VPN_PROTO', 'udp')
if not 1 <= RELAY_PORT <= 65535 or RELAY_PROTO not in ('udp', 'tcp'):
    raise ValueError('Invalid VPN transport configuration')


def run(*args):
    return subprocess.check_output(args, text=True).strip()


def load_relays():
    root = Path('/opt/v90/vpn')
    listing = root / 'relays.json'
    if not listing.exists():
        return [(RELAY, RELAY_PORT, RELAY_PROTO, root / 'vpngate.ovpn')]
    relays = []
    for item in json.loads(listing.read_text()):
        relay = str(ipaddress.IPv4Address(item['ip']))
        port = int(item['port'])
        proto = item['proto']
        profile = (root / item['profile']).resolve()
        if not 1 <= port <= 65535 or proto not in ('udp', 'tcp'):
            raise ValueError('Invalid VPN transport configuration')
        if profile.parent != root.resolve() or not profile.is_file():
            raise ValueError('VPN profile must exist inside the VPN directory')
        relays.append((relay, port, proto, profile))
    if not relays:
        raise ValueError('No VPN relays configured')
    return relays


def configure_network(relays):
    routes = run('ip', '-4', 'route', 'show', 'default').split()
    gateway = routes[routes.index('via') + 1]
    device = routes[routes.index('dev') + 1]
    # Configure all transport exceptions before rejecting non-VPN egress.
    for relay, port, proto, _ in relays:
        run('ip', 'route', 'replace', relay + '/32', 'via', gateway, 'dev', device)
        run('iptables', '-A', 'OUTPUT', '-o', device, '-d', relay + '/32',
            '-p', proto, '--dport', str(port), '-j', 'ACCEPT')
    run('iptables', '-A', 'OUTPUT', '-o', device, '-d', '10.77.90.0/24', '-j', 'ACCEPT')
    run('iptables', '-A', 'OUTPUT', '-o', device, '-j', 'REJECT')
    run('iptables', '-A', 'FORWARD', '-s', '10.0.2.0/24', '-o', 'tun0', '-j', 'ACCEPT')
    run('iptables', '-A', 'FORWARD', '-i', 'tun0', '-d', '10.0.2.0/24',
        '-m', 'conntrack', '--ctstate', 'ESTABLISHED,RELATED', '-j', 'ACCEPT')
    run('iptables', '-A', 'FORWARD', '-j', 'REJECT')
    run('iptables', '-t', 'nat', '-A', 'POSTROUTING', '-s', '10.0.2.0/24',
        '-o', 'tun0', '-j', 'MASQUERADE')
    Path('/etc/resolv.conf').write_text('nameserver 1.1.1.1\n')
    Path('/run/v90-uplink-configured').touch()


def connect_once(profile):
    READY.unlink(missing_ok=True)
    log = Path('/var/log/v90-multilink/vpn.log')
    log.parent.mkdir(parents=True, exist_ok=True)
    with log.open('w') as output:
        child = subprocess.Popen(['openvpn', '--config', str(profile),
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
                        if failures >= 2:
                            # Restart only the VPN transport; existing PPP
                            # processes stay alive while its interface recovers.
                            raise RuntimeError('VPN egress stalled; trying the next relay')
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


def maintain_relays(relays, connect=connect_once, pause=time.sleep):
    global RELAY, RELAY_PORT, RELAY_PROTO
    while True:
        for RELAY, RELAY_PORT, RELAY_PROTO, profile in relays:
            try:
                connect(profile)
            except (RuntimeError, subprocess.CalledProcessError, OSError) as error:
                READY.unlink(missing_ok=True)
                print(f'VPN relay {RELAY} unavailable: {error}', flush=True)
                pause(3)
        pause(15)


def main():
    READY.unlink(missing_ok=True)
    relays = load_relays()
    configure_network(relays)
    maintain_relays(relays)


if __name__ == '__main__':
    main()
