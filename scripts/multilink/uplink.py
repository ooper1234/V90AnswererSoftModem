#!/usr/bin/python3
"""Route native PPP IPv4 through the existing Wi-Fi-only SLiRP adapter.

The TUN interface lives in the dedicated backend container. A source policy
routes only 10.0.2.15 through it; container control traffic retains its route.
"""
import fcntl
import os
from pathlib import Path
import select
import socket
import struct
import subprocess
import time
from ppp_wire import Decoder, control, frame


def negotiate(stream, decoder, protocol, options, ident):
    deadline = time.monotonic() + 30
    acknowledged = peer_acknowledged = False
    last = 0
    while time.monotonic() < deadline:
        if not acknowledged and time.monotonic() - last > 2:
            stream.sendall(frame(protocol, control(1, ident, options)))
            last = time.monotonic()
        if not select.select([stream], [], [], 1)[0]:
            continue
        chunk = stream.recv(16384)
        if not chunk:
            raise RuntimeError('SLiRP closed during negotiation')
        for proto, data in decoder.feed(chunk):
            if len(data) < 4:
                continue
            code, reply_id, length = struct.unpack('!BBH', data[:4])
            if length < 4 or length > len(data):
                continue
            opts = data[4:length]
            if proto == protocol and code == 1:
                stream.sendall(frame(proto, control(2, reply_id, opts)))
                peer_acknowledged = True
            elif proto == protocol and code == 2 and reply_id == ident and opts == options:
                acknowledged = True
            elif proto == protocol and code == 3 and reply_id == ident:
                options = opts
                ident = (ident + 1) & 255
                last = 0
            elif proto == protocol and code == 4:
                raise RuntimeError('SLiRP rejected uplink options')
        if acknowledged and peer_acknowledged:
            return
    raise RuntimeError('SLiRP negotiation timed out')


def main():
    tun = os.open('/dev/net/tun', os.O_RDWR)
    fcntl.ioctl(tun, 0x400454ca, struct.pack('16sH', b'wifi0', 0x0001 | 0x1000))
    left, right = socket.socketpair()
    os.dup2(right.fileno(), 45)
    os.set_inheritable(45, True)
    child = subprocess.Popen(['/opt/v90/wifi/ppp-slirp.py'], pass_fds=(45,), stderr=None)
    os.close(45)
    right.close()
    decoder = Decoder()
    try:
        negotiate(left, decoder, 0xc021, b'', 10)
        negotiate(left, decoder, 0x8021, b'\x03\x06' + socket.inet_aton('10.0.2.15'), 20)
        subprocess.run(['ip', 'link', 'set', 'wifi0', 'mtu', '1500', 'up'], check=True)
        subprocess.run(['ip', 'route', 'replace', 'default', 'dev', 'wifi0', 'table', '90'], check=True)
        subprocess.run(['ip', 'rule', 'add', 'priority', '190', 'from', '10.0.2.15/32', 'table', '90'], check=True)
        Path('/run/v90-uplink-ready').touch()
        print('Wi-Fi uplink ready; only PPP client traffic uses wifi0', flush=True)
        while True:
            ready, _, _ = select.select([tun, left], [], [], 10)
            if child.poll() is not None:
                raise RuntimeError('SLiRP uplink stopped')
            if tun in ready:
                packet = os.read(tun, 65535)
                if packet and packet[0] >> 4 == 4:
                    left.sendall(frame(0x21, packet))
            if left in ready:
                chunk = left.recv(16384)
                if not chunk:
                    raise RuntimeError('SLiRP uplink disconnected')
                for proto, data in decoder.feed(chunk):
                    if proto == 0x21 and data and data[0] >> 4 == 4:
                        os.write(tun, data)
                    elif proto == 0xc021 and len(data) >= 4 and data[0] == 9:
                        left.sendall(frame(proto, control(10, data[1], data[4:])))
    finally:
        Path('/run/v90-uplink-ready').unlink(missing_ok=True)
        left.close()
        os.close(tun)
        child.terminate()
        child.wait(timeout=5)


if __name__ == '__main__':
    main()
