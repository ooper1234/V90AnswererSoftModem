#!/bin/sh
set -eu
cd "$(dirname "$0")/.."
cargo build --release -p binmodemffi --manifest-path third_party/BinModem/Cargo.toml
cmake -S . -B build
cmake --build build --target sm_daemon sm_ast_write_test -j 4
gcc -shared -fPIC -Wall -Wextra scripts/slirp-select-fix.c -o scripts/slirp-select-fix.so -ldl
chmod +x scripts/ppp-slirp.py
