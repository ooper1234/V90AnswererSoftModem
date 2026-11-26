# AudioSocket sample clock for modem calls

AudioSocket carries 8 kHz signed-linear audio without timestamps. Asterisk
20.6.0 normally timestamps these frames from wall-clock arrival time. A large
host clock correction can therefore insert an RTP timestamp gap even though
successive packets contain contiguous modem samples. On the tested WSL host,
wall time advanced by 650–795 ms while monotonic time advanced about 10 ms.

`app-audiosocket-sample-clock.patch` gives outgoing AudioSocket voice frames
a per-call delivery timeline. It captures the initial epoch once, then advances
by each frame's sample count. It changes neither call pacing nor received audio,
modem training, codecs, SIP routing, or system time. All AudioSocket calls using
this patched application get this behavior; other applications are unaffected.

## Build

Use the exact Asterisk version and development headers matching the server.
The tested package is Ubuntu 24.04
`1:20.6.0~dfsg+~cs6.13.40431414-2build5`.

Download the official Asterisk 20.6.0 source, then apply from its root:

```sh
patch -p1 < /path/to/app-audiosocket-sample-clock.patch
cc -fPIC -shared -O2 -Wall -Werror -I/usr/include \
  -DAST_MODULE_SELF_SYM=__internal_app_audiosocket_self \
  -o app_audiosocket.so apps/app_audiosocket.c
```

Required packages in a separate Ubuntu build environment:
`build-essential asterisk-dev uuid-dev`. Do not replace an active module.
Back up the installed `app_audiosocket.so`, wait for zero active calls, unload
that application, copy the matching candidate into the module directory, and
load it again. Check that Asterisk reports it Running. Restore the backup if
loading fails or hardware validation regresses. The original Asterisk source
and this patch retain its GPL version 2 licensing.

## Validation

Record RTP sequence numbers and timestamps together with monotonic arrival
times. Consecutive G.711 packets containing 160 samples should advance RTP
by 160 even across host wall-clock steps. Distinguish actual monotonic arrival
gaps from clock corrections. Test real V.90 PPP pings, checked uploads and
downloads, and prolonged calls; a correct timestamp trace alone does not prove
end-to-end modem stability.

## Receive jitter buffer

Apply the receive buffer on the actual modem call path, before `AudioSocket`:

```asterisk
same => n,Set(JITTERBUFFER(fixed)=200,1000)
```

This fixed 200 ms buffer reorders and holds incoming RTP; it adds receive
latency. An unused subroutine containing this setting does not protect calls.
Keep it fixed during modem training and data: adaptive changes can insert or
remove waveform samples. It cannot restore genuinely missing packets or
correct delay changes introduced by the remote gateway. Check every provider
route reaches the buffered entry point, then verify repeated PPP transfers and
a prolonged real modem call before claiming stability.

Reference: https://docs.asterisk.org/Asterisk_20_Documentation/API_Documentation/Dialplan_Functions/JITTERBUFFER/

## Missing receive packets

After the sample-clock patch, apply `app-audiosocket-input-clock.patch` and copy
`audiosocket-modem-input-clock.h` beside `apps/app_audiosocket.c` (or add its
folder to the compiler include paths). The helper preserves up to 200 ms of
missing receive time using zero-valued PCM. It detects holes in the original
frame timestamps and also handles empty interpolation frames without filling
an interval twice. Large timestamp resets start a new origin rather than
creating an unbounded silence burst. Ordinary audio passes through unchanged.

Asterisk's jitter buffer and G.711-to-linear translator can discard empty
interpolation frames. AudioSocket has no timestamp field, so omitting those
samples compresses the modem's receive timeline. Preserving time does not
recover lost bits; V.42 must retransmit damaged frames, and larger disturbances
may still require a retrain. The module logs the number of missing input
samples it preserved. Validate the actual modem and PPP connection after
installation; offline sample-accounting checks alone do not prove stability.

Run the frame-clock regression harness with matching development headers:

```sh
sh scripts/asterisk/test-input-clock.sh
```

Set `ASTERISK_INCLUDE` if the headers were extracted to a separate directory.

### Delay and Quick Connect testing

For the Conexant USB caller, Bell 103 and V.23 currently use transparent data
without LAPM/MNP. Select buffered transparent operation (`AT&Q6`) for these
tests; disabling compression alone (`AT%C0`) does not select that operating
mode. The client `scripts/dial-ppp.sh` accepts `MODEM_DATA_MODE=transparent`.
Its default `auto` selects `AT&Q5` for modes with error correction. Transparent
mode uses 576-byte PPP packets: the 128-byte V.23 test generated frequent TCP
acknowledgements on the 75-bps return channel and timed out after 413 bytes;
the 576-byte retest downloaded the full 577-byte page within the same timeout.
These are short single-call results, not a long-stability guarantee. Use RTS/CTS
on both the modem and PPP client. Do not select direct, unbuffered operation
at a 115200-bps terminal speed for a 300-bps line.

A larger fixed buffer trades additional delay for tolerance of late packets. On the tested Callcentric path, reducing the server buffer to 20 ms allowed short V.32 and V.34 PPP/HTTPS tests to pass. This is route-specific evidence, not a universal default or a long-stability guarantee. Keep the original dialplan backed up and change buffer settings only while calls are idle.

The 20 ms experiment was reverted to the original buffers after subsequent Bell 103 PPP startup failures. It did not make every mode reliable, and reducing buffering alone did not complete Quick Connect.

V.92 QC2a has a one-second caller acknowledgement deadline (ITU-T V.92, 9.2.1.2). The measured echo delay plus the QCA2d frame's transmission time can exceed it. Extending only the answerer's TONEq timeout cannot extend the caller's deadline. Check actual caller protocol and the server's QC phase-one completion; a V.32bis fallback connection must not be counted as Quick Connect.
