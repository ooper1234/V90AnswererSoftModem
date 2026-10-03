#!/usr/bin/python3
"""The modem's --pppd adapter: relay its FD 45 to the shared PPP service."""
import json
from pathlib import Path
import select
import socket
import sys


def main():
    if 'auth' in sys.argv[1:]:
        raise SystemExit('Local multilink adapter does not support --auth')
    config = json.loads(Path(__file__).with_name('client.json').read_text())
    local = socket.socket(fileno=45)
    remote = socket.create_connection((config['server'], 9300), timeout=10)
    remote.settimeout(None)
    remote.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
    try:
        while True:
            ready, _, _ = select.select([local, remote], [], [])
            for source in ready:
                data = source.recv(16384)
                if not data:
                    return
                (remote if source is local else local).sendall(data)
    finally:
        local.close()
        remote.close()


if __name__ == '__main__':
    main()
