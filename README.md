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
Windows Dial-up/browser compatibility on every client. V.34 fallback is
available but was not validated by these V.90 hardware results.

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
To hear a handshake, Asterisk must record/monitor the local modem extension;
a Listen script cannot hear calls that bypass its recording route.

## License and credits

This repository uses GNU GPL v3; see `LICENSE`. Vendored BinModem is
GPL-3.0-or-later and its original license and notices remain in
`third_party/BinModem/`. BinModem upstream:
https://github.com/CasualArclamp/BinModem

The optional external SLiRP backend has its own license. This product
includes software developed by Danny Gasparovski. SLiRP's documentation:
https://slirp.sourceforge.net/
