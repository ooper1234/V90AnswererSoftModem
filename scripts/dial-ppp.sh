#!/bin/bash
# Dial the softmodem ISP with real PPP over the USB modem (client side).
#
#   sudo MODEM_DEVICE=/dev/ttyACM0 DIAL_NUMBER=5551000 scripts/dial-ppp.sh
#
# Runs the client pppd inside a network namespace so 10.67.0.2 is not a
# local address of the main namespace: replies must come back across the
# modem link instead of being shortcut-delivered locally.  Every byte,
# upload and download, then really crosses the V.34 link.
#
# Ctrl-C stops pppd; dropping DTR hangs up the modem.
# Requires: sudo scripts/enable-nat.sh having been run once.
set -e
NS=dialup
MODEM_DEVICE="${MODEM_DEVICE:-/dev/ttyACM0}"
DIAL_NUMBER="${DIAL_NUMBER:-5551000}"
# Transparent legacy modes need speed buffering without trying LAPM/MNP.
# Keep auto-reliable operation for the modes that support error correction.
case "${MODEM_DATA_MODE:-auto}" in
    auto) MODEM_LINK_INIT='AT&Q5'; PPP_FRAME_SIZE=1500 ;;
    transparent) MODEM_LINK_INIT='AT&Q6'; PPP_FRAME_SIZE=576 ;;
    *) echo 'MODEM_DATA_MODE must be auto or transparent' >&2; exit 1 ;;
esac

if [ "$(id -u)" != 0 ]; then
    echo "must run via sudo (creates a network namespace)" >&2
    exit 1
fi
if [ ! -c "$MODEM_DEVICE" ]; then
    echo "no such modem device: $MODEM_DEVICE" >&2
    exit 1
fi
if fuser "$MODEM_DEVICE" >/dev/null 2>&1; then
    echo "modem is busy (close minicom/other pppd first):" >&2
    fuser -v "$MODEM_DEVICE" >&2 || true
    exit 1
fi

ip netns del "$NS" 2>/dev/null || true
ip netns add "$NS"
ip netns exec "$NS" ip link set lo up
mkdir -p /etc/netns/"$NS"
printf 'nameserver 1.1.1.1\nnameserver 8.8.8.8\n' > /etc/netns/"$NS"/resolv.conf

mkdir -p /tmp/softmodem
echo "dialing $DIAL_NUMBER on $MODEM_DEVICE (netns $NS)..."
exec ip netns exec "$NS" /usr/sbin/pppd "$MODEM_DEVICE" 115200 \
    connect "/usr/sbin/chat -v ABORT BUSY ABORT 'NO CARRIER' ABORT ERROR '' AT OK 'ATX3&D2&C1&K3' OK '$MODEM_LINK_INIT' OK ATDT$DIAL_NUMBER CONNECT ''" \
    noauth modem crtscts lock \
    mru "$PPP_FRAME_SIZE" mtu "$PPP_FRAME_SIZE" \
    10.67.0.2:10.67.0.1 \
    defaultroute \
    debug \
    logfile /tmp/softmodem/pppd-client.log
