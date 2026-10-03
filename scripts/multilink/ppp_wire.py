"""Asynchronous PPP framing for the SLiRP IPv4 uplink (RFC 1662)."""
import struct


def fcs(data):
    value = 0xffff
    for byte in data:
        value ^= byte
        for _ in range(8):
            value = (value >> 1) ^ (0x8408 if value & 1 else 0)
    return value


def frame(protocol, payload):
    data = b'\xff\x03' + struct.pack('!H', protocol) + payload
    data += struct.pack('<H', fcs(data) ^ 0xffff)
    wire = bytearray(b'\x7e')
    for byte in data:
        if byte < 32 or byte in (0x7d, 0x7e):
            wire.extend((0x7d, byte ^ 32))
        else:
            wire.append(byte)
    return bytes(wire) + b'\x7e'


class Decoder:
    def __init__(self):
        self.buffer = bytearray()
        self.escaped = False
        self.overflow = False

    def feed(self, chunk):
        packets = []
        for byte in chunk:
            if byte == 0x7e:
                data = bytes(self.buffer)
                if not self.overflow and not self.escaped and len(data) >= 4 and fcs(data) == 0xf0b8:
                    data = data[:-2]
                    if data.startswith(b'\xff\x03'):
                        data = data[2:]
                    if data and data[0] & 1:
                        packets.append((data[0], data[1:]))
                    elif len(data) >= 2:
                        packets.append((struct.unpack('!H', data[:2])[0], data[2:]))
                self.buffer.clear()
                self.escaped = self.overflow = False
            elif self.overflow:
                continue
            elif byte == 0x7d:
                self.escaped = True
            else:
                self.buffer.append(byte ^ 32 if self.escaped else byte)
                self.escaped = False
                if len(self.buffer) > 8192:
                    self.overflow = True
                    self.buffer.clear()
        return packets


def control(code, ident, options=b''):
    return struct.pack('!BBH', code, ident, 4 + len(options)) + options
