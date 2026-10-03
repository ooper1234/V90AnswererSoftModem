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


def ppp_arguments():
    return ['/usr/sbin/pppd', 'notty', 'nodetach', 'local', 'noauth',
            'noipdefault', 'nodefaultroute', 'noipv6', 'noccp', 'novj',
            'multilink', 'mrru', '1600', 'mru', '1500', 'mtu', '1500',
            'bundle', 'v90-windows', 'endpoint', 'local:56.39.30.4d.50',
            '10.0.2.2:10.0.2.15', 'ms-dns', '10.0.2.3',
            'lcp-max-configure', '120', 'ipcp-max-configure', '120',
            'lcp-echo-interval', '15', 'lcp-echo-failure', '6',
            'debug', 'hide-password', 'logfd', '2']


def main():
    config = json.loads(Path('/opt/v90/multilink/server.json').read_text())
    children = set()
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
            children = {p for p in children if p.poll() is None}
            try:
                stream, address = listener.accept()
            except socket.timeout:
                continue
            if address[0] not in config['allowed_clients'] or len(children) >= 4 or not Path('/run/v90-uplink-ready').exists():
                stream.close()
                continue
            stream.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            logfile = Path('/var/log/v90-multilink') / f'link-{time.time_ns()}.log'
            with logfile.open('wb') as log:
                child = subprocess.Popen(ppp_arguments(), stdin=stream, stdout=stream, stderr=log)
            stream.close()
            children.add(child)
            print(f'PPP link started pid={child.pid} log={logfile.name}', flush=True)
    finally:
        listener.close()
        for child in children:
            if child.poll() is None:
                child.terminate()
        for child in children:
            try:
                child.wait(timeout=5)
            except subprocess.TimeoutExpired:
                child.kill()
                child.wait()


if __name__ == '__main__':
    main()
