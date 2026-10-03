# V90AnswererSoftModem

A Linux software **answerer** for an analog V.90 modem calling through a
G.711 ATA and Asterisk AudioSocket. It uses the vendored BinModem engine,
offers V.42 LAPM error correction and V.42bis compression, and can provide
PPP networking. It is an experimental modem implementation.

## What was tested

On October 1, 2026, a Conexant CX93010 USB modem on Windows COM30 called
through a PAP2T Line 2 into Asterisk 20.6.0. Real V.90 calls reported
45,333–46,667 bit/s downstream and 28,800 bit/s upstream, with LAPM and
V42Bis. A 4,096-byte mixed payload returned with zero mismatches. That
echo test used paced writes and does not establish maximum throughput.

PPP was also tested through the real modem: LCP and IPCP negotiated and
an external DNS answer returned through the userspace networking backend.
These results do not promise 56 kbit/s on every ATA/line, or establish
Windows Dial-up/browser compatibility on every client. Automatic V.90-to-V.34
fallback was separately verified on October 2 as described below.

V.34 fallback follows the analogue modem's negotiated selection. The
answerer cannot force that selection by setting a caller-only flag. Failed
initial V.90 starts, including deadline-driven retrains, stop after three
attempts if the peer continues selecting V.90. V.34 data mode uses its own
echo handling rather than the V.90 data tracker. Regression coverage checks
bounded startup retries and bidirectional V.34 data with LAPM/compression.

V.34 now searches for boundary echoes up to 750 ms, including after V.90
fallback; the previous search stopped at 384 ms. Separate simulations verify
400/500 ms echo cancellation and V.34 training with exact bidirectional
6,000-bit payloads at 400/500 ms round-trip delay. These are software tests,
not verification of a particular SIP provider route. Round-trip delay is
different from one-way delay; jitter, packet loss, and voice processing can
still prevent modem operation.

The V.8 answerer offers V.32bis alongside its other supported modes. If V.8
does not complete and the caller is transmitting a sustained 1800 Hz V.32
opening tone, the answerer starts V.32/V.32bis without repeating the answer
tone. Otherwise it hands control to the legacy V.22bis engine. It restarts both the
transmitter and receiver, sends the initial 75 ms gap, then high-channel
unscrambled binary ones. The measured steady startup signal is 2250 Hz;
previously this path incorrectly started V.34 INFO0/Tone A. Negotiated V.90
and V.34 calls retain their own startup paths. The calling engine also keeps
its explicitly selected non-V.8 V.34 mode.

Dedicated `--v32` and `--v32bis` modes start with the V.25 answering sequence
and support V.42 LAPM and negotiated V.42bis compression. Rate exchange is
restricted to rates supported by the peer. V.32 line buffering is limited to
about 50 ms so queued idle bits do not delay error-control responses.
Caller-initiated and answerer-initiated retrains are handled by the V.32
startup engine. Repeated retrains pass simulated transfer checks. On October 2,
a real CX93010 caller connected at 14,400 bit/s with LAPM/V.42bis, requested
a retrain with `ATO1`, reconnected at the same rate, and returned 512 bytes
without mismatches both before and after retraining. The answerer retains its
working echo filter during retraining. Five Windows PPP calls using one modem
completed four website downloads; the fifth failed with no dial tone before
connecting. Physical legacy V.32 at 9,600 bit/s remains unverified.

For an unstable analogue upload path, set `V90_UP_RATE=28800` in the
answerer's environment. This caps upload negotiation at 28.8 kbit/s without
limiting V.90 download negotiation, and never raises a lower rate selected
for the line. Data-mode echo fitting stops during retraining and keeps its
coefficients paired with the delay used for validation. On the tested home
PAP2, three connected calls each completed three verified page downloads
with these settings; two other calls still failed during startup.

## Requirements and build

The server runs on Linux. Windows can run the server through Docker Desktop
and use a physical USB modem as the calling client. You need Asterisk with
`app_audiosocket`, a G.711 ATA, and a modem connected to its phone port.

For a native build, install a current Rust toolchain (edition 2024), GCC,
CMake, Python 3 and either `ppp` or `slirp`. On Ubuntu/Debian:

```sh
sudo apt-get update
sudo apt-get install build-essential cmake python3 ppp slirp
git clone https://github.com/ooper1234/V90AnswererSoftModem.git
cd V90AnswererSoftModem
sh scripts/build-v90.sh
```

Alternatively, build the included container from the repository directory:

```sh
docker build -t v90-answerer .
```

## Asterisk and ATA setup

Configure the ATA modem port to use G.711 **u-law**, with silence suppression,
echo cancellation and echo suppression disabled. Use a fixed/low jitter
buffer. Register that port with your Asterisk server; merge the examples in
`asterisk/` into your configuration and replace the example credentials.
Keep the AudioSocket listener reachable only by Asterisk.

Merge this exact local test extension into the context used by your ATA:

```ini
[from-ata]
exten => 5551000,1,Answer()
 same => n,AudioSocket(00000000-0000-0000-0000-000000000090,127.0.0.1:9093)
 same => n,Hangup()
```

`127.0.0.1` works when Asterisk and the answerer share a machine or network
namespace. For Docker, use the existing Asterisk container's namespace:

```sh
docker run -d --name v90-answerer --network container:cx93010-asterisk v90-answerer
```

Replace `cx93010-asterisk` with your Asterisk container name. This command
starts PPP on port 9093. No privileged container or kernel PPP device is
required for the default userspace backend. Reapply/reload your saved
dialplan after restarting Asterisk; runtime CLI additions are temporary.

## Multiple simultaneous calls

Run one answerer listener. It accepts the next AudioSocket connection while
the current call runs in its own child process. Each child has its own modem
training, V.42/V.42bis state and PPP backend; hanging up one call does not
hang up the other. There is no need to start another listener manually.

For a two-port PAP2, register each phone port with a different authenticated
Asterisk endpoint, for example Line 1 as `201` and Line 2 as `200`. Route
both endpoints to the same `from-ata` extension and AudioSocket listener.
Match these local endpoints by their SIP username: an IP-only identify rule
cannot distinguish two accounts on the same ATA. Keep a separate password
for each account and retain authentication on both endpoints.

Apply the modem audio settings above to **both** ATA lines. With Docker
Desktop, set both Proxy and Outbound Proxy to the laptop's LAN address and
SIP port, enable Use Outbound Proxy and Use OB Proxy In Dialog, and keep
each ATA line's own SIP port distinct. Wait for both accounts to register
after saving settings before dialing. Either modem can dial `*995551000`.

SLiRP runs separately for each call, so clients on different computers may
both receive `10.0.2.15` without sharing their network stack. Two simultaneous
Windows Dial-up interfaces on one computer with that same address need
additional routing/address configuration; the default setup does not
provide distinct addresses for those interfaces.

## PPP with SLiRP (works without kernel PPP support)

For a native build:

```sh
./build/sm_daemon --listen 127.0.0.1 --port 9093 --v90 --v34 \
  --pppd "$PWD/scripts/ppp-slirp.py" \
  --local-ip 10.0.2.2 --peer-ip 10.0.2.15 --dns1 10.0.2.3 --debug
```

SLiRP negotiates client address `10.0.2.15` and forwards outgoing TCP/UDP
through the server's existing internet connection. **Set the client's DNS
manually to `10.0.2.3`.** The launcher has no login authentication and is
intended for a local testing endpoint. It rejects `--auth`; native pppd
address, DNS and ip-up/ip-down options are not implemented by this backend.
The `--local-ip` argument above is a diagnostic label; SLiRP chooses its
own IPCP server address. Do not combine PPP with `--echo` or `--no-ppp`.

SLiRP 1.0.17 has a 64-bit timeout sentinel bug that can make it exit before
negotiation. `slirp-select-fix.c` corrects that sentinel; the build script
places the resulting shared library beside the launcher. Install SLiRP
separately using your package manager; it is not vendored here.

## Dial from Windows

For two modems in one Windows connection, use the
[shared Multilink PPP backend](scripts/multilink/README.md). Independent SLiRP
instances do not support Multilink PPP. The shared backend uses native Linux
PPP for the bundle and a separate SLiRP adapter for the Wi-Fi-only IPv4 uplink;
it requires kernel PPP multilink support. Both callers use the same Asterisk
number. Each physical modem retains its own V.42/V.42bis negotiation.

1. Close PuTTY and other applications using the modem's COM port.
2. Create a **Dial-up** connection in Windows Network and Sharing Center,
   selecting your physical modem.
3. Dial `*995551000` for the tested PAP2T setup: `*99` enables the ATA's
   modem mode and `5551000` is the local Asterisk extension. On an ATA
   without that prefix, use its modem-mode setting and dial `5551000`.
4. The local SLiRP server does not require a username or password.
5. In that connection's IPv4 properties, obtain the IP automatically and
   set DNS to `10.0.2.3`. Enable its default gateway if you want application
   traffic to use the dial-up link. Disable IPv6 for this IPv4-only backend.

To force the tested modem's V.90 mode, use this extra initialization command
in its Windows modem properties if the driver supports it:

```text
AT+MS=V90,0,300,33600,300,56000
```

A terminal can dial `ATDT*995551000` and show CONNECT, but it does not create
a Windows PPP network connection. `CONNECT 115200` reports the serial port
rate, not the negotiated V.90 line speed. On the tested Conexant modem,
disconnect and query `AT&V1` to see line speed, LAPM and V42Bis.

## Native kernel PPP alternative

On a Linux host with `/dev/ppp` and PPP kernel support, run with native pppd:

```sh
sudo ./build/sm_daemon --listen 127.0.0.1 --port 9093 --v90 --v34 \
  --local-ip 10.90.0.1 --peer-ip 10.90.0.2 \
  --dns1 1.1.1.1 --dns2 8.8.8.8 --debug
```

Enable IPv4 forwarding and configure your firewall/NAT for that peer if
internet access is needed. Unlike SLiRP, native pppd requires kernel PPP
support and networking privileges. `--auth` requires PAP secrets configured
for pppd. The current Windows/WSL kernel lacked PPP support, so hardware
PPP validation used SLiRP instead.

## Echo test and error control

For a byte-for-byte echo endpoint instead of PPP, start on another port and
point a separate Asterisk extension at it:

```sh
./build/sm_daemon --listen 127.0.0.1 --port 9092 --v90 --v34 --echo --no-ppp --debug
```

V.42 and V.42bis are enabled by default when the peer agrees. Use
`V90_COMPRESSION=0` to disable V.42bis while retaining LAPM, or
`V90_ERROR_CONTROL=0` for raw diagnostic mode. Read `docs/v90-data-fix.md`
for the transport changes and focused test commands.

On Linux with SLiRP installed, `python3 tests/ppp_slirp.py` checks LCP/IPCP
negotiation and sends a DNS query through the backend without a physical
modem. It requires internet access and the built `slirp-select-fix.so`.

If the modem connects but receives no data, check the server's per-call
logs (`/tmp/v90` in the container), its PPP child log, and the AudioSocket
address/port. Ensure that the daemon is running in PPP mode. If DNS fails,
check the client's manual DNS setting and the server's internet access.

## Live audio processing

V.90 data-mode echo fitting runs on one background worker, so it cannot
block delivery of the next AudioSocket audio frame. Completed fits from an
earlier data interval are discarded after a retrain. Extra A/B/C replay
receivers are disabled unless `V90_ABC_POINTS` or `V90_ABC_PATH` explicitly
requests those diagnostics.

Stopping an idle daemon interrupts its listening socket immediately, so a
replacement can bind the same port without waiting for another caller.

Real Windows dial-up testing improved from 0/10 to 9/10 verified webpage
downloads with these changes, at 48,000–49,333 bit/s downstream and
28,800 bit/s upstream. One call still timed out; the short-page test does
not establish long-call reliability. V.42 and V.42bis remain enabled.

## V.34 fallback

Keep both `--v90 --v34` enabled. A caller that selects V.34 in V.8 uses
the V.34 engine directly; a V.90 caller can also select V.34 in the next
phase-2 INFO1a after unsuccessful V.90 training. The server waits for that
selection after its initial retry budget is exhausted, rather than hanging
up before the caller can request V.34. If the caller selects V.90 again,
the server ends the failed startup instead of retrying indefinitely.
Retrains of an established connection do not consume this initial budget.

The October 2 Windows repeat test completed 8/10 verified short-page
downloads, with 9/10 PPP connections. A forced V.34 hardware check negotiated
33,600 bit/s, V.42 and V.42bis and echoed 512 bytes without corruption.
A subsequent 4 KB check failed during training before sending data. After
moving the ATA to the laptop's wired network, the updated build negotiated
31,200 bit/s, V.42 and V.42bis and echoed all 4,096 bytes without a mismatch.
An additional automatic-fallback check disrupted only initial V.90 training
through a diagnostic AudioSocket relay. The Conexant on Windows COM36
selected V.34 in INFO1a, connected at 33,600 bit/s downstream and 12,000 bit/s
upstream with V.42/V.42bis, and echoed all 4,096 bytes without a mismatch.
A Windows Dial-up call through the same training fault negotiated V.34 and
PPP, downloaded the complete 713-byte Example Domain page, then hung up.
These are controlled functional checks, not a measured connection-success
rate or maximum throughput claim. Earlier tests with the disturbance lasting
into V.34 training returned partial data and remain failed results.

Fallback retains V.90 phase-2 negotiation and retrain roles, while V.34
training/data use the same 16 kHz internal engine as standalone V.34. The
phase display reports the modulation actually negotiated, including retrains.
LAPM hands frames to HDLC one at a time so acknowledgements do not wait
behind a whole window of bulk data. Re-establishment discards compressed
octets queued under the old dictionary before resetting compression.

Regression tests exercise repeated initial retrains followed by automatic
V.34 selection and exact bytes with V.42/V.42bis through the C interface.
The negotiation/retrain roles follow
[ITU-T V.90, sections 9.2.1.1.8 and 9.2.2.1.9](https://www.itu.int/rec/dologin_pub.asp?id=T-REC-V.90-199809-I%21%21PDF-E&lang=e&type=items).

## License and credits

This repository uses GNU GPL v3; see `LICENSE`. Vendored BinModem is
GPL-3.0-or-later and its original license and notices remain in
`third_party/BinModem/`. BinModem upstream:
https://github.com/CasualArclamp/BinModem

The optional external SLiRP backend has its own license. This product
includes software developed by Danny Gasparovski. SLiRP's documentation:
https://slirp.sourceforge.net/

## October 3 reliability update

V.42 loss recovery now preserves its retry deadline across duplicate acknowledgements and polls a busy peer after a lost ready notification. The V.90 answerer also requests its existing full retrain after five continuous seconds of lost receive lock during data mode; brief disturbances reset the interval when lock returns.

The error-control and modem simulations pass. Real Windows PPP tests before the receive-lock change still showed packet stalls despite successful connections and some successful downloads. Real-modem validation demonstrated receive-lock retrain recovery. The later rate-headroom update is described below.

For the tested PAP2 link, `V90_UP_MARGIN_DB=6` reserves six dB below the
phase-3 training SNR when selecting the analogue-to-digital upload rate.
This addresses the observed training/data SNR difference (about 35–36 dB
versus 30 dB). It can select 24,000 bit/s on this line while allowing higher
rates on cleaner lines. It does not cap the V.90 download rate, and V.42
and V.42bis remain enabled. The default margin is zero for compatibility;
set `V90_UP_MARGIN_DB=6` in the daemon environment to opt in. Values are
limited to 0–12 dB. `V90_UP_RATE` remains an independent maximum upload cap.

This is a local reliability policy, not a protocol-mandated margin. A lower
upload rate is preferable to repeated receive-lock loss, but short successful
tests do not establish long-call stability or reliability on other routes.


On October 3, the installed six-dB margin selected 24,000 bit/s upload and
46,666 bit/s download in two isolated modem calls. Each received all 30
pings within the original 1,500 ms timeout and downloaded two verified
example.com pages. A further saved-profile call attached both modem links
to the same PPP bundle, received 20/20 pings, and downloaded both pages.
Total: 80/80 ping replies and 6/6 verified downloads in three dial-up sessions.
These short calls do not establish long-call reliability.

The dense upstream receiver now accounts for carrier rotation across its
recovery window and applies the fitted phase at the next symbol. A targeted
regression fails before this correction and passes afterward; receiver,
V.90 transfer, compression and fallback checks pass. Real testing at 28,800
upstream still showed stalls, so the live six-dB safety margin remains in
place. The corrected receiver with that margin completed 30/30 pings and
both website downloads in its final short validation call. This correction
does not establish reliable 28,800 or 33,600 upstream on the tested ATA.
