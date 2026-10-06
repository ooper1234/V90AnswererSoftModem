"""Loopback-only HTTP/CONNECT proxy reachable through SLiRP's 10.0.2.2."""
import http.server
import json
import os
import pathlib
import select
import socket
import struct
import urllib.parse

def connect_wifi(host, port):
    if port not in (80, 443):
        raise ValueError('Only website ports 80 and 443 are allowed')
    if os.environ.get('V90_UPLINK', 'wifi') == 'vpngate':
        # The VPN namespace owns the default route and the egress kill switch.
        # Going through the Wi-Fi relay here bypasses that default entirely.
        stream = socket.create_connection((host, port), 15)
        stream.settimeout(None)
        return stream
    config = json.loads(pathlib.Path('/opt/v90/wifi/wifi-uplink.json').read_text())
    error = None
    for _, _, _, _, address in socket.getaddrinfo(host, port, socket.AF_INET, socket.SOCK_STREAM):
        stream = socket.create_connection((config['relay_ip'], 9081), 15)
        try:
            stream.settimeout(20)
            stream.sendall(b'V90W' + bytes.fromhex(config['token']) + socket.inet_aton(address[0]) + struct.pack('!H', port))
            if stream.recv(1) != b'\x00':
                raise OSError('Wi-Fi relay could not connect')
            stream.settimeout(None)
            return stream
        except OSError as caught:
            error = caught
            stream.close()
    raise error or OSError('No IPv4 website address')

def bridge(client, remote):
    endpoints = [client, remote]
    while endpoints:
        readable, _, _ = select.select(endpoints, [], [], 90)
        if not readable:
            return
        for source in readable:
            target = remote if source is client else client
            data = source.recv(16384)
            if not data:
                endpoints.remove(source)
                try:
                    target.shutdown(socket.SHUT_WR)
                except OSError:
                    pass
            else:
                target.sendall(data)

class Proxy(http.server.BaseHTTPRequestHandler):
    protocol_version = 'HTTP/1.1'

    def do_GET(self):
        try:
            target = urllib.parse.urlsplit(self.path)
            if target.scheme != 'http' or not target.hostname or target.username:
                raise ValueError('Expected an absolute HTTP website URL')
            with connect_wifi(target.hostname, target.port or 80) as remote:
                path = urllib.parse.urlunsplit(('', '', target.path or '/', target.query, ''))
                headers = [f'GET {path} HTTP/1.1', f'Host: {target.netloc}', 'Connection: close']
                for name in ('User-Agent', 'Accept', 'Accept-Encoding'):
                    if self.headers.get(name):
                        headers.append(f'{name}: {self.headers[name]}')
                remote.sendall(('\r\n'.join(headers) + '\r\n\r\n').encode('iso-8859-1'))
                bridge(self.connection, remote)
            self.close_connection = True
        except (OSError, ValueError) as error:
            self.send_error(502, str(error))

    def do_CONNECT(self):
        try:
            host, port = self.path.rsplit(':', 1)
            if int(port) != 443:
                raise ValueError('HTTPS CONNECT must use port 443')
            with connect_wifi(host, 443) as remote:
                self.send_response(200, 'Connection established')
                self.end_headers()
                self.wfile.flush()
                bridge(self.connection, remote)
            self.close_connection = True
        except (OSError, ValueError) as error:
            self.send_error(502, str(error))

if __name__ == '__main__':
    http.server.ThreadingHTTPServer(('0.0.0.0', 9082), Proxy).serve_forever()
