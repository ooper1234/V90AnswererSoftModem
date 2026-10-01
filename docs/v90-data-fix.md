# V.90 byte transport

## Current hardware result (October 1, 2026)

The physical upstream issue described in the historical notes below is
resolved by the final echo-canceller repairs: unbiased least-squares path
identification, correct transmit-ring alignment, and preserving an identified
path instead of adapting it to the quieter remote modem signal.

The Conexant CX93010 connected in V.90 with LAPM/V42Bis at 45,333–46,667
bit/s downstream and 28,800 bit/s upstream. Both 512-byte and 4,096-byte
echo tests returned byte-exact data; no physical 56 kbit/s claim is made.
PPP LCP/IPCP negotiation and an external DNS response were then verified
over the real modem using the SLiRP backend. See the root README for setup.

Earlier failure reports below are retained as diagnostic history.

The digital answerer now offers V.42 LAPM in V.8 and starts its error-control
stack by default. Previously `--v90` always used raw line bits unless the
bench variable `V90_ERROR_CONTROL` was set, and even setting it did not put
LAPM in the V.8 menu. That could leave the peer and the answerer using
different data framing after physical training.

When V.8 agrees LAPM, the stack proceeds to XID. Otherwise it runs V.42
detection and can fall back to transparent async data. V.42bis compression is
offered in both directions by default and enabled only when the peer agrees.
`V90_COMPRESSION=0` keeps V.42 LAPM but disables compression.
`V90_ERROR_CONTROL=0` selects raw bits and bypasses both protocols.

V.90 also now advances the error-control timers at its actual 8000 Hz line
rate. They previously counted 16 samples per millisecond, the V.34 engine's
rate, so detection and retransmission timers ran at half speed.

## Rebuild and run on the Linux modem host

```sh
cmake --build build
./build/sm_daemon --listen 127.0.0.1 --port 9092 \
    --local-ip 10.67.0.1 --peer-ip 10.67.0.2 \
    --v8 --v34 --binmodem --v90 --debug
```

Use the existing SIP/ATA bridge configuration. Remove any old
`V90_ERROR_CONTROL=0` setting to use the default. The daemon should report
`error control=V.42 LAPM compression=V.42bis` when the peer supports both, or
`error control=none` after transparent fallback.

## Verification

```sh
cd third_party/BinModem
cargo test -p binmodemffi --release --lib error_control_clock_tests
cargo test -p binmodemffi --release --test v90_ffi \
    --test v90_lapm_ffi --test v90_transparent_ffi --test v90_fallback_ffi
```

The LAPM test sends independent 1024-byte patterns in both directions, with
8N1 DTE framing and a service call every 160 line samples (20 ms), matching
the daemon. Both patterns must arrive byte for byte. It also requires the
far end to observe the LAPM offer and a downstream rate of at least 48000
bit/s. The raw and transparent tests check independent bit patterns both
ways. A separate test checks V.90-to-V.34 fallback, and the clock test checks
the detection deadline at both 8000 and 16000 Hz.

These tests passed over a simulated delayed, noisy G.711 mu-law network.
The original LAPM regression failed before the fix because the digital
answerer did not advertise its enabled LAPM.

## Real modem test, 2026-10-01 (Asia/Bangkok)

Tested a Conexant USB CX93010 on Windows COM30 through PAP2T Line 2 at
192.168.137.111 and the existing local Asterisk 20.6.0 server. Dedicated
temporary extensions routed to the fixed server's echo mode; no provider
call was needed.

The first call exposed an additional AudioSocket interoperability problem:
the server sent the header and body in separate writes, and Asterisk closed
the nonblocking socket while waiting for the body. The writer now queues
the complete frame together, with its existing partial-write recovery.
`sm_ast_write_test` checks both the complete-frame send and forced partial
writes. The corrected live call remained up through modem training.

| Mode | Actual result |
|---|---|
| Forced V.90 | Failed before data mode. Server ended training with `no E from the answer modem`; client returned `NO CARRIER`. Zero user bytes transferred. |
| Forced V.34 control | Connected at the server's reported 4800 bit/s. 512 bytes sent, 512 returned, zero mismatches. |

`CONNECT 115200` from the Windows modem describes the DTE port speed; it
does not establish a 115200 bit/s line rate. The server's negotiation log
establishes the V.34 control's actual rate.

V.90 remains unverified on this physical path and is not working in this
test. The successful V.34 control demonstrates real byte transport through
the modem, ATA, Asterisk, and fixed answerer. The remaining V.90 problem is
in startup before LAPM or user data, and its captured waveform is needed
for further diagnosis. Both call captures and logs were preserved. The
modem's original V92 auto settings were verified after the test, the
temporary dialplan extensions were removed, and both test servers stopped.

## Historical full-speed follow-up (before final echo repairs)

The 3429-symbol V.90 upstream capability is enabled and the longer LIVE_SERVER training profile is restored. The LAPM regression now requires 33,600 bit/s upstream as well as at least 48,000 bit/s downstream with byte-exact traffic in both directions.

Real hardware reached V.90 negotiation at 46,666 down / 28,800 up, but LAPM frames were corrupted and the modem never reported CONNECT. Diagnostic upstream caps of 9,600 and 4,800 did not fix physical transfer. No permanent cap is included. Physical V.90 send/receive remains unresolved. Modem readback showed that the V.34 control actually received 33,600 and sent 4,800 bit/s.

## V.42 / V.42bis controls

| Setting before starting the daemon | Behavior |
|---|---|
| Variables unset (default) | Negotiate V.42 LAPM and bidirectional V.42bis. |
| V90_COMPRESSION=0 | Negotiate V.42 LAPM without V.42bis. |
| V90_ERROR_CONTROL=0 | Diagnostic raw-bit mode; no V.42 or V.42bis. |

Compression requires an established error-corrected link and peer agreement.
A peer that supports LAPM alone stays uncompressed. A peer without LAPM
can use transparent fallback. The daemon reports the negotiated result.

The V.42bis regression transfers 4,096 bytes in each direction, combining
repetitive data and varied byte values, at the daemon's 20-ms service cadence.
It verifies V.42bis negotiation, byte-exact decompression, and full simulated
V.90 upstream rate. Separate tests verify a peer without compression and
turning compression off while retaining LAPM.

At this earlier stage the protocol changes alone had not resolved the physical
upstream decoding failure. The final echo repairs and successful hardware
results are documented at the top of this file.
