#!/bin/sh
set -eu
here=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
build=$(mktemp -d)
trap 'rm -rf "$build"' EXIT HUP INT TERM
cc -O2 -Wall -Werror -DAST_MODULE_SELF_SYM=__internal_input_clock_test \
 -I"${ASTERISK_INCLUDE:-/usr/include}" -I"$here" \
 "$here/tests/input-clock.c" -o "$build/input-clock-test"
"$build/input-clock-test"
