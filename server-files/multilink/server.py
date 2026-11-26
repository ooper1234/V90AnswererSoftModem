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

import ccp_shim
import login_screen
import telnet_door

PPP_MTU = 1500
LOGINS = Path('/opt/v90/multilink/logins.json')
# Accounts that go to a telnet BBS instead of PPP: username -> (name, host, port).
DOORS = {'tuxedocatlounge': ('Tuxedo Cat Lounge BBS', 'bbs.tuxedocatbbs.com', 2323),
         'fozztexx': ('FozzTexx BBS', 'bbs.fozztexx.com', 23)}


def load_accounts():
    """{username: password hash} for the login screen; empty (nobody gets
    in) if the file is missing or unreadable. Read per call, so edits apply
    without a restart."""
    try:
        accounts = json.loads(LOGINS.read_text())
        return {str(k): str(v) for k, v in accounts.items()}
    except (OSError, ValueError, AttributeError) as e:
        print(f'login accounts unavailable ({e}); refusing logins', flush=True)
        return {}


PAP_NAME = 'cooper-dialup'   # pppd's own name; matches its pap-secrets line


def auth_arguments(require_pap):
    """'noauth' after a typed login; otherwise the caller must pass PAP,
    which pppd checks through PAM ('login') with pap_check.py."""
    if not require_pap:
        return ['noauth']
    return ['auth', 'require-pap', 'login', 'name', PAP_NAME]


def ppp_arguments(peer_ip='10.0.2.15', restart=30, require_pap=False, silent=True):
    # silent: wait for the caller's LCP. A pppd started again after a failed
    # reattach must open LCP itself: the caller is already in IPCP and sends
    # no LCP, so a silent one waits until the caller gives up (2026-11-16).
    args = ['/usr/sbin/pppd', 'notty', 'nodetach', 'local', *auth_arguments(require_pap), *(['silent'] if silent else []),
            'noipdefault', 'nodefaultroute', 'noipv6', 'noccp',
            'multilink', 'mrru', '1600', 'mru', str(PPP_MTU), 'mtu', str(PPP_MTU),
            'bundle', 'v90-windows', 'endpoint', 'local:56.39.30.4d.50',
            '10.0.2.2:' + peer_ip, 'ms-dns', os.environ.get('V90_PPP_DNS', '10.0.2.3'),
            'lcp-max-configure', '120', 'ipcp-max-configure', '120',
            'lcp-restart', str(restart), 'ipcp-restart', str(restart),
            'lcp-echo-interval', '30', 'lcp-echo-failure', '6',
            'debug', 'hide-password', 'logfd', '2']
    if os.environ.get('V90_PPP_VJ', '0') != '1':
        args.append('novj')
    return args


def restart_for_rate(bps):
    """pppd retransmit timer that fits one negotiation round trip.

    A ~60-byte LCP/IPCP frame costs 600/bps seconds each way. A fixed 30 s
    timer turns every lost frame on a link without error correction into a
    30 s stall (callers give up first); a 3 s timer floods a 75 bit/s V.23
    upstream with retransmissions that can never be answered in time.
    """
    if not bps or bps <= 0:
        return 10
    frame = 600.0 / bps
    return max(3, min(30, int(2 * frame + 1.5 + 0.999)))


def measure_first_frame(stream, wait=40.0):
    """Line rate (bit/s) from how fast the caller's first PPP frame arrives.

    Only peeks: the bytes stay queued for pppd, which still answers the
    caller's first LCP (it runs 'silent'). Leading non-PPP bytes such as a
    V.42 detection pattern are skipped. Returns None if unknown.
    """
    import select as _select
    deadline = time.monotonic() + wait
    first_at = None
    start_idx = None
    while time.monotonic() < deadline:
        if not _select.select([stream], [], [], 0.05)[0]:
            continue
        try:
            data = stream.recv(4096, socket.MSG_PEEK)
        except OSError:
            return None
        if not data:
            return None
        if start_idx is None:
            for marker in (b'\x7e\xff', b'\x7e\xc0\x21', b'\xff\x7d\x23'):
                i = data.find(marker)
                if i >= 0 and (start_idx is None or i < start_idx):
                    start_idx = i
            if start_idx is not None:
                first_at = time.monotonic()
        if start_idx is not None:
            end = data.find(b'\x7e', start_idx + 2)
            if end > start_idx + 8:
                elapsed = time.monotonic() - first_at
                nbytes = end - start_idx + 1
                if elapsed < 0.15:
                    return 9600   # arrived as a burst: an error-corrected or fast link
                return int(nbytes * 10 / elapsed)
        if len(data) >= 4096:
            return None
        time.sleep(0.03)
    return None


def admission_allowed(address, config, session_count, uplink, ready, configured):
    # The VPN kill switch, rather than a working remote relay, is required
    # for safe PPP admission. Keep IPCP and local gateway traffic available
    # while the VPN reconnects; its firewall still blocks home egress.
    protected = configured if uplink == 'vpngate' else ready
    return address in config['allowed_clients'] and session_count < 4 and protected


def mppc_relay(stream, inner, done, tag):
    """Carry the caller's stream to pppd (inner) and back through ccp_shim,
    until pppd's side closes. The caller's stream is left open: serve_link
    may start another pppd on it."""
    import select as _select
    shim = ccp_shim.Shim(log=lambda m: print(f'{m} ({tag})', flush=True))
    lock = threading.Lock()

    def to_caller(data):
        if data:
            stream.sendall(data)

    def from_pppd():
        try:
            while True:
                data = inner.recv(16384)
                if not data:
                    break
                with lock:
                    to_caller(shim.from_pppd(data))
        except OSError:
            pass
        done.set()

    reader = threading.Thread(target=from_pppd, daemon=True)
    reader.start()
    try:
        while not done.is_set():
            ready = _select.select([stream], [], [], 0.5)[0]
            with lock:
                to_caller(shim.tick())
            if not ready or done.is_set():
                continue
            data = stream.recv(16384)
            if not data:
                break
            with lock:
                to_pppd, back = shim.from_peer(data)
                to_caller(back)
            if to_pppd:
                inner.sendall(to_pppd)
    except OSError:
        pass
    finally:
        try:
            inner.shutdown(socket.SHUT_RDWR)
        except OSError:
            pass
        reader.join(timeout=5)
        inner.close()
        a, b, c, d = shim.counts()
        if shim.active or a or c:
            saved = lambda plain, wire: f'{100 * (plain - wire) / plain:.0f}%' if plain else '-'
            print(f'mppc summary: down {a} -> {b} bytes ({saved(a, b)} saved), '
                  f'up {d} -> {c} bytes ({saved(d, c)} saved) ({tag})', flush=True)


def serve_link(stream, stopping, children, lock, peer_ip='10.0.2.15'):
    """Keep the byte transport alive for a bounded PPP-unit recovery.

    Some pppd/kernel combinations fail to reattach a non-multilink unit
    after LCP reopens on a link that originally offered multilink. A fresh
    pppd can renegotiate over the same modem connection; closing the TCP
    transport instead strands the caller despite a healthy LAPM link.
    """
    try:
        shown = login_screen.run_login(stream, peer_ip, PPP_MTU, load_accounts(), doors=DOORS)
        print(f'login screen: {shown} (peer {peer_ip})', flush=True)
        if isinstance(shown, tuple):
            name, host, port = DOORS[shown[1]]
            print(f'Door {shown[1]}: telnet {host}:{port} (peer {peer_ip})', flush=True)
            ended = telnet_door.bridge(stream, host, port, name)
            print(f'Door {shown[1]} ended: {ended} (peer {peer_ip})', flush=True)
            return
        require_pap = shown == 'ppp'
        if require_pap:
            print(f'Caller started PPP at the login screen; requiring PAP (peer {peer_ip})', flush=True)
        elif shown != 'login':
            # No login, no PPP: closing the transport hangs the call up.
            print(f'No login ({shown}); hanging up peer {peer_ip}', flush=True)
            return
        bps = measure_first_frame(stream)
        restart = restart_for_rate(bps)
        print(f'PPP first frame at ~{bps or "unknown"} bit/s; LCP/IPCP restart {restart}s', flush=True)
        for attempt in range(3):
            if stopping.is_set():
                break
            logfile = Path('/var/log/v90-multilink') / f'link-{time.time_ns()}.log'
            relay = None
            with logfile.open('wb') as log:
                if os.environ.get('V90_MPPC', '1') == '1':
                    # MPPC for callers that ask for it (Windows' "software
                    # compression"); everyone else's bytes pass unchanged.
                    outer, inner = socket.socketpair()
                    child = subprocess.Popen(ppp_arguments(peer_ip, restart, require_pap, silent=attempt == 0), stdin=outer, stdout=outer, stderr=log)
                    outer.close()
                    relay = threading.Thread(target=mppc_relay, args=(stream, inner, threading.Event(), f'peer {peer_ip}'), daemon=True)
                    relay.start()
                else:
                    child = subprocess.Popen(ppp_arguments(peer_ip, restart, require_pap, silent=attempt == 0), stdin=stream, stdout=stream, stderr=log)
            with lock:
                children.add(child)
                if stopping.is_set():
                    child.terminate()
            print(f'PPP link started pid={child.pid} peer={peer_ip} log={logfile.name}', flush=True)
            status = child.wait()
            if relay is not None:
                relay.join(timeout=10)
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
            if not admission_allowed(address[0], config, len(sessions),
                    os.environ.get('V90_UPLINK', 'wifi'),
                    Path('/run/v90-uplink-ready').exists(),
                    Path('/run/v90-uplink-configured').exists()):
                stream.close()
                continue
            stream.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            # Keep a lease for the entire modem transport, including PPP
            # recovery. Independently routed callers must not overwrite one
            # another's host route. Joined MP members use their bundle's IPCP.
            peer_ip = '10.0.2.15'
            if os.environ.get('V90_UPLINK', 'wifi') in ('vpngate', 'direct'):
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
