# V90AnswererSoftModem

A Linux software modem server. Dial-up modems call it over a G.711 phone
line (an ATA or a SIP trunk) into Asterisk, and it answers like an ISP's
modem bank. It supports V.90 and V.92 56k, the older standards down to 300
bit/s, V.42 error correction with V.42bis/V.44 compression, and gives each
caller a PPP internet connection.

It uses the vendored [BinModem](https://github.com/CasualArclamp/BinModem)
engine. It is experimental: it works on live calls, but long-call
reliability depends heavily on the phone line.

## Modulations

Speeds are the most each mode carries; what a call reaches depends on the
line.

| Modulation | Down / up (bit/s) | How it is selected | Status |
|---|---|---|---|
| V.90 | up to 56 000 / 33 600 | V.8, caller offers V.90 | Main mode, live-tested |
| V.92 PCM upstream | up to 56 000 / 48 000 | V.8, caller offers V.92; needs `V92_PCM_UPSTREAM=1` | Experimental |
| V.92 Quick Connect, modem-on-hold | as V.90 | V.8 / V.8bis | Experimental |
| U.S. Robotics x2 | up to 56 000 / 33 600 | caller's x2 marker in V.34 phase 2; needs `V90_X2=1`, mu-law only | Experimental |
| V.34 | up to 33 600 | V.8, or fallback from failed V.90 training | Live-tested |
| V.32bis / V.32 | 14 400 / 9 600 | V.8, or a legacy caller's AA tone | Live-tested |
| V.22bis / V.22 | 2 400 / 1 200 | V.8, or the legacy probe | Live-tested |
| V.23 | 1 200 / 75 (or reversed) | V.8, `--v23`, or last in the legacy probe | Connects; PPP untested |
| Bell 103 | 300 / 300 | legacy probe (2225 Hz mark) | Connects |
| V.21 | 300 / 300 | legacy probe (1650 Hz mark) | Simulation-tested |
| U.S. Robotics HST | 14 400 / 450 (or 300) | caller's back channel heard during the legacy probe | Simulation-tested; see below |

**Error correction and compression:** V.42 LAPM with V.42bis or V.44 on
V.22bis and faster modes (V.42 on Bell 103 with `B103_V42=1`). HST carries
MNP error correction, with 256-octet frames that shrink on a noisy line.

**Legacy probe.** A caller that does not answer V.8 still hears the answer
tone. After it the server tries, in turn: V.32 (if the caller sent AA),
V.22bis (5 s), a Bell 103 mark (3 s), a V.21 mark (3 s), then V.23. A
caller's own tone at any point (Bell 103's 1270 Hz, V.21's 980 Hz, V.22's
1200 Hz or HST's back channel near 375 Hz) selects its mode straight away.

**HST** comes from BinModem, which rebuilt it from a Courier V.Everything's
firmware. Its training and pulse shapes are not the Courier's, so a real
U.S. Robotics Courier may not connect; BinModem dialling with `AT+MS=HST`
does. HST 16 800 and 4 800 are not implemented.

**Not supported:** K56flex (never published), V.32terbo, V.FC.

## What you need

- A Linux machine (or Docker).
- Asterisk with `app_audiosocket`.
- A phone line into Asterisk that stays G.711 end to end: an ATA (such as a
  Linksys PAP2) with a modem on its phone port, or a SIP trunk that does
  not re-encode calls. 56k modes need an exact u-law path.

## Quick setup (all in one)

On a Linux machine with Docker (the script offers to install it on Debian
or Ubuntu):

```sh
git clone https://github.com/ooper1234/V90AnswererSoftModem.git
cd V90AnswererSoftModem
./setup.sh
```

It asks for everything it needs: this machine's address, whether modems
call through an ATA or a SIP provider (or both), the ATA's account or the
provider's details, ports, and which optional modes to turn on. Then it
writes the configuration, builds and starts three containers (Asterisk with
the patched AudioSocket, the audio pacer and the modem answerer), checks
them, and prints the settings to type into your ATA.

Afterwards:

| Command | Does |
|---|---|
| `./setup.sh` | Change settings (your previous answers are the defaults) |
| `./setup.sh status` | What is running, registrations, recent calls |
| `./setup.sh status --show-password` | The same, plus the ATA's password |
| `./setup.sh logs` | Follow the logs |
| `./setup.sh restart` / `stop` | Restart or stop it |
| `./setup.sh uninstall` | Remove the containers and images (settings stay) |

Settings and passwords are kept in `deploy/generated/`, readable only by
you. The first build takes several minutes.

For your own Asterisk instead, or no Docker, install by hand:

## Install

### Native build

Install a current Rust toolchain (edition 2024, via https://rustup.rs),
GCC, CMake, Python 3 and either `slirp` or `ppp`. The BBS loads SQLite at
run time if it is there. On Ubuntu or Debian:

```sh
sudo apt-get update
sudo apt-get install build-essential cmake python3 libsqlite3-0 slirp ppp
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh
git clone https://github.com/ooper1234/V90AnswererSoftModem.git
cd V90AnswererSoftModem
sh scripts/build-v90.sh
```

That builds `build/sm_daemon`. Start it with PPP through userspace SLiRP
(no root needed):

```sh
./build/sm_daemon --listen 127.0.0.1 --port 9093 --v90 --v34 \
    --pppd "$PWD/scripts/ppp-slirp.py" \
    --local-ip 10.0.2.2 --peer-ip 10.0.2.15 --dns1 10.0.2.3 \
    --log-dir /tmp/v90 --debug
```

`./build/sm_daemon --help` lists every option.

### Docker

```sh
docker build -t v90-answerer .
docker run -d --name v90-answerer --network container:<your-asterisk-container> v90-answerer
```

The image starts the same command as above. Sharing Asterisk's network
namespace lets AudioSocket reach `127.0.0.1:9093`.

### Optional modes

Environment variables, off unless set (with Docker, pass them with `-e`):

| Variable | Effect |
|---|---|
| `V90_X2=1` | Answer U.S. Robotics x2 callers (mu-law lines only) |
| `V92_PCM_UPSTREAM=1` | Offer V.92 PCM upstream. PCM is never offered without it |
| `V92_PCM_ECHO=1` | Echo canceller for PCM upstream (use with the above) |
| `B103_V42=1` | V.42 error control on Bell 103 |
| `V22_V42=0` | Turn V.42 off on V.22bis |
| `V8BIS_ENABLE=1` | Answer with V.8bis signalling |
| `V90_COMPRESSION=0` | Keep V.42 but turn compression off |

## Asterisk setup

Send the modem number to the answerer with AudioSocket:

```ini
[from-ata]
exten => 5551000,1,Answer()
 same => n,AudioSocket(00000000-0000-0000-0000-000000000090,127.0.0.1:9093)
 same => n,Hangup()
```

`127.0.0.1` works when Asterisk and the answerer share a machine or network
namespace. Example endpoint and dialplan files are in `asterisk/`; replace
their example credentials. Keep the AudioSocket port reachable only by
Asterisk.

Allow only `ulaw` on the modem's endpoint and turn off direct media. For
calls over the internet, a steady audio clock matters: the patched
`app_audiosocket` in [scripts/asterisk](scripts/asterisk/README.md) keeps
the modem's sample timing through lost packets.

### ATA settings

On the ATA's modem port:

- Codec G.711 **u-law** only.
- Silence suppression, echo cancellation and echo suppression off.
- A small fixed jitter buffer.
- Modem or fax mode on if the ATA has one (on a PAP2, dial `*99` before
  the number, for example `*995551000`).

## Dialling in

From Windows, create a **Dial-up** connection on the modem and dial the
modem number (`*995551000` on a PAP2). No username or password is needed
with the default SLiRP backend. Set the connection's DNS to `10.0.2.3`.

To make a modem prefer V.90, add an extra initialisation command such as:

```text
AT+MS=V90,0,300,33600,300,56000
```

A terminal program (`ATDT5551000`) shows CONNECT, but only a PPP dialler
gets a network connection. `CONNECT 115200` is the serial port's speed, not
the line's.

Several modems can call at once: each call runs in its own process with its
own training, error correction and PPP. Two modems can share one Windows
connection with [Multilink PPP](scripts/multilink/README.md).

## PPP options

- **SLiRP (default):** userspace, no root or kernel PPP needed. Callers get
  `10.0.2.15` and DNS `10.0.2.3`; traffic leaves through the server's own
  internet connection. No login.
- **Native pppd:** on a host with `/dev/ppp`, leave out `--pppd` and run as
  root with your own addresses (`--local-ip`, `--peer-ip`, `--dns1`).
  `--auth` requires PAP secrets. Set up IP forwarding and NAT yourself.

## Testing and troubleshooting

Run the tests:

```sh
cargo test --release --manifest-path third_party/BinModem/Cargo.toml -p binmodemffi -p datapump -p ec
cmake --build build --target legacy_detect_test && ./build/legacy_detect_test
```

For a byte-for-byte echo endpoint instead of PPP, run a second daemon on
another port with `--echo --no-ppp` and point a separate extension at it.

If a modem connects but no data flows, check the per-call logs in the
`--log-dir` folder, the PPP child log there, and the AudioSocket address
and port. If DNS fails, check the client's DNS setting.

More detail on each mode is in [docs](docs/): V.8, V.22bis, V.23, V.32,
V.34, V.42, V.90/V.92.

## License and credits

GNU GPL v3; see `LICENSE`. The vendored BinModem is GPL-3.0-or-later; its
license and notices are in `third_party/BinModem/`. BinModem upstream:
https://github.com/CasualArclamp/BinModem

The optional SLiRP backend has its own license. This product includes
software developed by Danny Gasparovski (https://slirp.sourceforge.net/).
The Asterisk patches in `scripts/asterisk/` keep Asterisk's GPL version 2.
