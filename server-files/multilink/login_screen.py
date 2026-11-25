#!/usr/bin/python3
"""Cooper Dialup Services terminal login shown to a caller right after CONNECT.

Screen (24-row terminal): cleared, the POP name on row 20, "Login:" on
row 24, then "Password:" and the PPP session details:

    Cooper Dialup Services



    Login: cooper
    Password:
        Entering PPP Session.
        IP address is 10.0.2.15
        MTU is 1500.

PPP starts only after a successful login against `accounts`
({username: hash_password(...)}; usernames are not case sensitive,
passwords are). A wrong login shows "Login incorrect" and a new Login:
prompt; after `attempts` failures the caller is refused. A caller that
types nothing for `timeout` seconds is refused too. Bytes sent after the
password line stay queued for pppd.

A dialer that starts PPP straight away (Windows Dial-Up Networking without
"bring up terminal window") never types a login: it sends its username and
password inside PPP, with PAP. When a PPP frame arrives at an empty Login:
prompt the screen returns 'ppp' and leaves the frame queued; the caller
must then pass PAP against the same accounts before any IP traffic (see
pap_check.py). PPP frames arriving part-way through a typed login are
still discarded whole, so none of their bytes end up in the login.
"""
import hashlib
import hmac
import os
import select
import socket
import time

TITLE = 'Cooper Dialup Services'
CLEAR = b'\x1b[H\x1b[2J'
# Seconds a caller may sit at Login:/Password: without typing. 60 s was too
# short: callers who read the screen first found a dead line (2026-11-15).
LOGIN_TIMEOUT = float(os.environ.get('LOGIN_TIMEOUT', '300'))
TIMED_OUT = b'\r\nLogin timed out\r\n'
TITLE_ROW, LOGIN_ROW = 20, 24
INDENT = b'    '
# Starts of an HDLC-framed PPP packet: flag + address, flag + compressed
# LCP protocol, or address + escaped control byte.
PPP_MARKERS = (b'\x7e\xff', b'\x7e\xc0\x21', b'\xff\x7d\x23')
FLAG = 0x7e
BACKSPACE = (0x08, 0x7f)


def screen():
    return (CLEAR + b'\r\n' * (TITLE_ROW - 1) + TITLE.encode()
            + b'\r\n' * (LOGIN_ROW - TITLE_ROW) + b'Login:')


def session_banner(peer_ip, mtu):
    return (INDENT + b'Entering PPP Session.\r\n'
            + INDENT + f'IP address is {peer_ip}\r\n'.encode()
            + INDENT + f'MTU is {mtu}.\r\n'.encode())


PBKDF2_ROUNDS = 200_000


def hash_password(password, salt=None):
    """Stored form of a password: pbkdf2_sha256$rounds$salt$hash (hex)."""
    salt = salt if salt is not None else os.urandom(16)
    digest = hashlib.pbkdf2_hmac('sha256', password.encode(), salt, PBKDF2_ROUNDS)
    return f'pbkdf2_sha256${PBKDF2_ROUNDS}${salt.hex()}${digest.hex()}'


def check_login(accounts, username, password):
    stored = {k.lower(): v for k, v in accounts.items()}.get(username.lower())
    # Unknown users cost the same as known ones, so timing says nothing.
    algo, rounds, salt, want = (stored or hash_password('', b'\0' * 16)).split('$')
    got = hashlib.pbkdf2_hmac('sha256', password.encode(), bytes.fromhex(salt), int(rounds))
    return stored is not None and algo == 'pbkdf2_sha256' and hmac.compare_digest(got.hex(), want)


class _Terminal:
    def __init__(self, stream, timeout):
        self.stream = stream
        self.timeout = timeout
        self.skip_lf = False
        self.in_frame = False     # inside a PPP frame that is being discarded
        self.ppp_ok = False       # a PPP frame on an empty line ends the screen
        self.ppp_seen = False

    def read_line(self, echo, ppp_ok=False):
        """Returns (status, text): status is 'line', 'timeout', 'closed', or
        'ppp' (only with ppp_ok) when the caller started PPP instead of typing."""
        self.ppp_ok = ppp_ok
        self.ppp_seen = False
        line = bytearray()
        deadline = time.monotonic() + self.timeout
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                return 'timeout', None
            if not select.select([self.stream], [], [], min(remaining, 0.5))[0]:
                continue
            try:
                data = self.stream.recv(4096, socket.MSG_PEEK)
            except OSError:
                return 'closed', None
            if not data:
                return 'closed', None
            taken, reply, done, typed = self._scan(data, line, echo)
            if taken:
                self.stream.recv(taken)   # consume only what was handled
            else:
                time.sleep(0.05)          # a possible frame start: wait for more
            if reply:
                self.stream.sendall(reply)
            if self.ppp_seen:
                return 'ppp', None
            if typed:
                deadline = time.monotonic() + self.timeout   # typing resets the timer
            if done:
                return 'line', line.decode('ascii')

    def _scan(self, data, line, echo):
        """Handle bytes up to the end of a line. Returns (taken, reply, done, typed)."""
        i = 0
        reply = bytearray()
        typed = False
        while i < len(data):
            byte = data[i]
            if self.in_frame:
                i += 1
                if byte == FLAG:
                    self.in_frame = False
                continue
            if byte in (FLAG, 0xff):
                rest = data[i:]
                if any(rest.startswith(m) for m in PPP_MARKERS):
                    if self.ppp_ok and not line:
                        # Left queued, unconsumed, for pppd.
                        self.ppp_seen = True
                        return i, reply, False, typed
                    self.in_frame = True
                    i += 1
                    continue
                if len(rest) < 3 and any(m.startswith(rest) for m in PPP_MARKERS):
                    break                 # cannot tell yet: leave it queued
            i += 1
            if self.skip_lf:
                self.skip_lf = False
                if byte == 0x0a:
                    continue
            if byte in (0x0d, 0x0a):
                self.skip_lf = byte == 0x0d
                return i, reply, True, True
            if byte in BACKSPACE:
                typed = True
                if line:
                    line.pop()
                    if echo:
                        reply += b'\b \b'
            elif 0x20 <= byte < 0x7f:
                typed = True
                line.append(byte)
                if echo:
                    reply.append(byte)
        return i, reply, False, typed


def run_login(stream, peer_ip, mtu, accounts, timeout=None, attempts=3, doors=None,
              allow_ppp=True):
    """Show the login screen. Returns 'login', 'ppp', 'rejected', 'timeout'
    or 'closed', or ('door', username) when a `doors` account logged in.

    Only on 'login' has the session banner been sent and may PPP start; a
    door login gets no PPP banner (the caller goes to that door instead).
    'ppp' (only with allow_ppp) means the caller started PPP at the Login:
    prompt: PPP may start only with PAP required. Otherwise the caller
    should be hung up on.
    """
    doors = {k.lower() for k in (doors or ())}
    try:
        if timeout is None:
            timeout = LOGIN_TIMEOUT
        stream.sendall(screen())
        term = _Terminal(stream, timeout)
        for attempt in range(attempts):
            if attempt:
                stream.sendall(b'\r\nLogin incorrect\r\nLogin:')
            status, username = term.read_line(echo=True, ppp_ok=allow_ppp)
            if status == 'ppp':
                return 'ppp'
            if status == 'timeout':
                stream.sendall(TIMED_OUT)
            if status != 'line':
                return status
            stream.sendall(b'\r\nPassword:')
            status, password = term.read_line(echo=False)
            if status == 'timeout':
                stream.sendall(TIMED_OUT)
            if status != 'line':
                return status
            if check_login(accounts, username, password):
                if username.lower() in doors:
                    return ('door', username.lower())
                stream.sendall(b'\r\n' + session_banner(peer_ip, mtu))
                return 'login'
        stream.sendall(b'\r\nLogin incorrect\r\n')
        return 'rejected'
    except OSError:
        return 'closed'
