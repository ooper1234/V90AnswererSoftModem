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
