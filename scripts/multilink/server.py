#!/usr/bin/python3
"""Accept modem byte streams and let Linux pppd manage a shared MP bundle."""
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import threading
import time


def ppp_arguments(peer_ip='10.0.2.15'):
    return ['/usr/sbin/pppd', 'notty', 'nodetach', 'local', 'noauth',
            'noipdefault', 'nodefaultroute', 'noipv6', 'noccp', 'novj',
            'multilink', 'mrru', '1600', 'mru', '1500', 'mtu', '1500',
            'bundle', 'v90-windows', 'endpoint', 'local:56.39.30.4d.50',
            '10.0.2.2:' + peer_ip, 'ms-dns', os.environ.get('V90_PPP_DNS', '10.0.2.3'),
            'lcp-max-configure', '120', 'ipcp-max-configure', '120',
            'lcp-restart', '10', 'ipcp-restart', '10',
            'lcp-echo-interval', '15', 'lcp-echo-failure', '6',
            'debug', 'hide-password', 'logfd', '2']


def serve_link(stream, stopping, children, lock, peer_ip='10.0.2.15'):
    """Keep the byte transport alive for a bounded PPP-unit recovery.

    Some pppd/kernel combinations fail to reattach a non-multilink unit
    after LCP reopens on a link that originally offered multilink. A fresh
    pppd can renegotiate over the same modem connection; closing the TCP
    transport instead strands the caller despite a healthy LAPM link.
    """
    try:
        for attempt in range(3):
            if stopping.is_set():
                break
            logfile = Path('/var/log/v90-multilink') / f'link-{time.time_ns()}.log'
            with logfile.open('wb') as log:
                child = subprocess.Popen(ppp_arguments(peer_ip), stdin=stream, stdout=stream, stderr=log)
            with lock:
                children.add(child)
                if stopping.is_set():
                    child.terminate()
            print(f'PPP link started pid={child.pid} peer={peer_ip} log={logfile.name}', flush=True)
            status = child.wait()
            with lock:
                children.discard(child)
            tail = logfile.read_text(errors='replace')[-8192:]
            if (status == 0 or stopping.is_set() or attempt == 2
                    or "Couldn't attach to PPP unit" not in tail
                    or 'Invalid argument' not in tail):
                break
            print('PPP unit reattach failed; renegotiating on the existing modem link', flush=True)
    finally:
        stream.close()


def main():
    config = json.loads(Path('/opt/v90/multilink/server.json').read_text())
    children = set()
    lock = threading.Lock()
    sessions = {}
    stopping = threading.Event()
    os.makedirs('/var/log/v90-multilink', exist_ok=True)
    listener = socket.socket()
    listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    listener.bind(('0.0.0.0', 9300))
    listener.listen(4)
    listener.settimeout(1)

    def stop(*_):
        stopping.set()

    signal.signal(signal.SIGTERM, stop)
    signal.signal(signal.SIGINT, stop)
    try:
        while not stopping.is_set():
            sessions = {t:ip for t,ip in sessions.items() if t.is_alive()}
            try:
                stream, address = listener.accept()
            except socket.timeout:
                continue
            if address[0] not in config['allowed_clients'] or len(sessions) >= 4 or not Path('/run/v90-uplink-ready').exists():
                stream.close()
                continue
            stream.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            # Keep a lease for the entire modem transport, including PPP
            # recovery. Independently routed callers must not overwrite one
            # another's host route. Joined MP members use their bundle's IPCP.
            peer_ip = '10.0.2.15'
            if os.environ.get('V90_UPLINK', 'wifi') == 'vpngate':
                peer_ip = next(ip for ip in (f'10.0.2.{i}' for i in range(15,19))
                               if ip not in sessions.values())
            thread = threading.Thread(target=serve_link, args=(stream, stopping, children, lock, peer_ip), daemon=True)
            sessions[thread] = peer_ip
            thread.start()
    finally:
        listener.close()
        stopping.set()
        with lock:
            active_children = list(children)
        for child in active_children:
            if child.poll() is None:
                child.terminate()
        for child in active_children:
            try:
                child.wait(timeout=5)
            except subprocess.TimeoutExpired:
                child.kill()
                child.wait()
        for thread in sessions:
            thread.join(timeout=6)


if __name__ == '__main__':
    main()
