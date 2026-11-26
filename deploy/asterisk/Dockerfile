# Asterisk for the modem server: Ubuntu 24.04's Asterisk 20.6.0 with
# app_audiosocket rebuilt from the patches in scripts/asterisk (a per-call
# sample clock out, and missing receive time kept as silence), which keep the
# modem's waveform on time across lost and late packets.
# Build from the repository root: docker build -f deploy/asterisk/Dockerfile .

FROM ubuntu:24.04 AS build
RUN apt-get update \
 && apt-get install -y --no-install-recommends asterisk-dev build-essential uuid-dev curl ca-certificates patch \
 && rm -rf /var/lib/apt/lists/*
WORKDIR /src
COPY scripts/asterisk/app-audiosocket-sample-clock.patch scripts/asterisk/app-audiosocket-input-clock.patch ./
COPY scripts/asterisk/audiosocket-modem-input-clock.h apps/
# The module's source for exactly the installed version (20.6.0 on 24.04).
RUN version=$(dpkg-query -W -f='${Version}' asterisk-dev | sed -E 's/^[0-9]+://; s/[~+].*//; s/-[^-]*$//') \
 && echo "Asterisk $version" \
 && curl -fsSL "https://raw.githubusercontent.com/asterisk/asterisk/$version/apps/app_audiosocket.c" -o apps/app_audiosocket.c \
 && patch -p1 < app-audiosocket-sample-clock.patch \
 && patch -p1 < app-audiosocket-input-clock.patch \
 && cc -fPIC -shared -O2 -Wall -I/usr/include -Iapps \
      -DAST_MODULE_SELF_SYM=__internal_app_audiosocket_self \
      -o app_audiosocket.so apps/app_audiosocket.c

FROM ubuntu:24.04
RUN apt-get update \
 && apt-get install -y --no-install-recommends asterisk \
 && rm -rf /var/lib/apt/lists/*
COPY --from=build /src/app_audiosocket.so /tmp/app_audiosocket.so
RUN dir=$(dirname "$(find /usr/lib -name app_audiosocket.so -path '*asterisk/modules*' | head -1)") \
 && cp "$dir/app_audiosocket.so" "$dir/app_audiosocket.so.stock" \
 && mv /tmp/app_audiosocket.so "$dir/app_audiosocket.so"
CMD ["asterisk", "-f", "-U", "asterisk", "-G", "asterisk", "-vvv"]
