#!/usr/bin/python3
"""Local, unauthenticated PPP networking without kernel PPP support.

Use --pppd /absolute/path/ppp-slirp.py. The daemon passes a duplex socket
as FD 45. Client IP/DNS are 10.0.2.15/10.0.2.3. Native pppd address,
DNS and hook arguments do not apply to this backend.
"""
import os
from pathlib import Path
import sys

if "auth" in sys.argv[1:]:
    raise SystemExit("SLiRP backend does not support --auth; use native pppd")
fix = Path(__file__).resolve().with_name("slirp-select-fix.so")
if not fix.is_file():
    raise SystemExit("Build scripts/slirp-select-fix.so before starting SLiRP")
os.environ["LD_PRELOAD"] = str(fix)
os.dup2(45, 0)
os.close(45)
os.execv("/usr/bin/slirp", ["slirp", "-P", "-b", "115200", "dns 1.1.1.1"])
