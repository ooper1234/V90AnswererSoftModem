#!/bin/sh
set -eu
rm -f /run/v90-uplink-ready
chmod 600 /opt/v90/wifi/wifi-uplink.json
chmod +x /opt/v90/wifi/ppp-slirp.py
test -c /dev/ppp || mknod /dev/ppp c 108 0
mkdir -p /dev/net
test -c /dev/net/tun || mknod /dev/net/tun c 10 200
mkdir -p /run/ppp
if [ "${V90_UPLINK:-wifi}" = vpngate ]; then
 python3 /opt/v90/multilink/vpn_uplink.py &
else
 python3 /opt/v90/multilink/uplink.py &
fi
uplink=$!
cleanup() { kill "$uplink" ${server:-} ${proxy:-} 2>/dev/null || true; }
trap cleanup EXIT TERM INT
for attempt in $(seq 1 40); do
 test -f /run/v90-uplink-ready && break
 kill -0 "$uplink"
 sleep 1
done
test -f /run/v90-uplink-ready
python3 /opt/v90/multilink/web_proxy.py &
proxy=$!
python3 /opt/v90/multilink/server.py &
server=$!
while kill -0 "$uplink" && kill -0 "$server" && kill -0 "$proxy"; do sleep 2; done
exit 1
