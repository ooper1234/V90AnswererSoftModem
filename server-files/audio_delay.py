#!/usr/bin/env python3
"""AudioSocket relay that delays every frame by DELAY_MS in each direction.

Test tool: makes a short local call look like a long-distance one (the
round trip grows by twice DELAY_MS). Frames keep their spacing and order;
only when they arrive moves.

  DELAY_LISTEN_PORT (9120)  where Asterisk connects
  DELAY_TARGET_PORT (9104)  the pacer/answerer it relays to
  DELAY_MS          (450)   one-way delay added
"""
import os
import socket
import threading
import time
from collections import deque

LISTEN = ('127.0.0.1', int(os.environ.get('DELAY_LISTEN_PORT', '9120')))
TARGET = ('127.0.0.1', int(os.environ.get('DELAY_TARGET_PORT', '9104')))
DELAY = float(os.environ.get('DELAY_MS', '450')) / 1000.0


def read_frame(sock):
    head = b''
    while len(head) < 3:
        chunk = sock.recv(3 - len(head))
        if not chunk:
            return None
        head += chunk
    n = int.from_bytes(head[1:3], 'big')
    body = b''
    while len(body) < n:
        chunk = sock.recv(n - len(body))
        if not chunk:
            return None
        body += chunk
    return head + body


def pipe(src, dst, delay, stop):
    """Read frames from src; write each to dst `delay` seconds after it came."""
    queue, cv = deque(), threading.Condition()

    def writer():
        while True:
            with cv:
                while not queue and not stop.is_set():
                    cv.wait(0.1)
                if not queue:
                    return
                due, frame = queue[0]
            wait = due - time.monotonic()
            if wait > 0:
                time.sleep(wait)
            with cv:
                queue.popleft()
            if frame is None:
                stop.set()
                return
            try:
                dst.sendall(frame)
            except OSError:
                stop.set()
                return

    t = threading.Thread(target=writer, daemon=True)
    t.start()
    try:
        while not stop.is_set():
            frame = read_frame(src)
            with cv:
                queue.append((time.monotonic() + delay, frame))
                cv.notify()
            if frame is None:
                break
    except OSError:
        with cv:
            queue.append((time.monotonic() + delay, None))
            cv.notify()
    t.join()
    for s in (src, dst):
        try:
            s.shutdown(socket.SHUT_RDWR)
        except OSError:
            pass


def serve(conn):
    try:
        far = socket.create_connection(TARGET, timeout=5)
    except OSError as e:
        print(f'delay: cannot reach {TARGET}: {e}', flush=True)
        conn.close()
        return
    far.settimeout(None)
    for s in (conn, far):
        s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
    # The first frame (the call's UUID) goes straight through.
    first = read_frame(conn)
    if first is None:
        conn.close(); far.close(); return
    far.sendall(first)
    stop = threading.Event()
    up = threading.Thread(target=pipe, args=(conn, far, DELAY, stop), daemon=True)
    up.start()
    pipe(far, conn, DELAY, stop)
    up.join()
    conn.close(); far.close()
    print('delay: call ended', flush=True)


def main():
    srv = socket.socket()
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind(LISTEN)
    srv.listen(4)
    print(f'delay: {LISTEN[1]} -> {TARGET[1]}, {DELAY * 1000:.0f} ms each way', flush=True)
    while True:
        conn, _ = srv.accept()
        print('delay: call', flush=True)
        threading.Thread(target=serve, args=(conn,), daemon=True).start()


if __name__ == '__main__':
    main()
