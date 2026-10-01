FROM rust:1-bookworm AS build
RUN apt-get update && apt-get install -y --no-install-recommends cmake && rm -rf /var/lib/apt/lists/*
WORKDIR /src
COPY . .
RUN sh scripts/build-v90.sh && ./build/sm_ast_write_test

FROM ubuntu:24.04
RUN apt-get update && apt-get install -y --no-install-recommends python3 slirp ppp libsqlite3-0 && rm -rf /var/lib/apt/lists/*
WORKDIR /opt/v90
COPY --from=build /src/build/sm_daemon ./sm_daemon
COPY --from=build /src/scripts/ppp-slirp.py ./ppp-slirp.py
COPY --from=build /src/scripts/slirp-select-fix.so ./slirp-select-fix.so
COPY LICENSE ./LICENSE
COPY third_party/BinModem/LICENSE ./BinModem-LICENSE
ENTRYPOINT ["/opt/v90/sm_daemon"]
CMD ["--listen", "127.0.0.1", "--port", "9093", "--v90", "--v34", "--pppd", "/opt/v90/ppp-slirp.py", "--local-ip", "10.0.2.2", "--peer-ip", "10.0.2.15", "--dns1", "10.0.2.3", "--log-dir", "/tmp/v90", "--debug"]
