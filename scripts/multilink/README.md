# Shared Multilink PPP backend

## Optional VPN Gate uplink

Set `V90_UPLINK=vpngate` and `V90_PPP_DNS=1.1.1.1` on the dedicated
PPP container, and mount a sanitized OpenVPN profile at
`/opt/v90/vpn/vpngate.ovpn` read-only. The VPN launcher currently pins the
tested relay `27.130.93.85:1233/udp`; changing relays requires updating
the profile, transport firewall allowance and expected exit verification.
Keep profile keys private and outside version control.

The VPN-enabled image needs OpenVPN, CA certificates, curl and iptables.
Only the PPP container's default route changes. Its Docker control network
remains reachable; Asterisk and host management keep their own routes.
PPP IPv4 forwarding is NATed into `tun0`, and the HTTP/HTTPS proxy also uses
the VPN. DNS is offered as `1.1.1.1`, replacing SLiRP's DNS address for VPN
sessions. Reconnect existing clients to receive the new DNS setting.

The launcher verifies an HTTPS exit check before accepting PPP sessions.
When VPN routing fails, firewall rules reject home-internet fallback. A VPN
process/interface failure stops the backend, allowing its container restart
policy to retry; this may disconnect callers. A volunteer relay's continued
availability and unchanged IP are not guaranteed.

In VPN mode, up to four active physical modem transports reserve distinct
private addresses from `10.0.2.15` through `10.0.2.18`. Independent callers
therefore do not overwrite each other's PPP routes. Multilink members join
their existing bundle and retain its IPCP address, including when the first
physical member drops. All callers still share the VPN's public exit IP.
Do not switch backends while calls are active. Back up
the compose file and backend scripts before changing a live installation.

This backend terminates each modem's PPP link with native Linux `pppd` and
joins links with the same peer endpoint discriminator into one kernel PPP
bundle. RFC 1990 packet fragmentation, ordering and link removal are handled
by Linux, rather than implemented inside the modem signal processor.

The Wi-Fi/SLiRP configuration serves one Windows computer at a time:
server `10.0.2.2`, client `10.0.2.15`, DNS `10.0.2.3`. It also accepts a
single-link caller. Two unrelated computers require separate address pools
and authentication configuration, which this local test configuration does
not provide. The VPN address pool separates private routes, but does not
add caller authentication or separate public IPs.

Each physical transport permits at most two PPP process retries for the
specific Linux `Couldn't attach to PPP unit ... Invalid argument` failure.
The modem transport stays open during those retries. Ordinary hangups and
other daemon failures do not automatically restart PPP. LCP and IPCP retry
intervals are ten seconds to accommodate modem latency.

## Prerequisites

- Linux kernel with `CONFIG_PPP=y`, `CONFIG_PPP_ASYNC=y`,
  `CONFIG_PPP_MULTILINK=y`, and TUN support.
- `pppd` built with multilink support, Python 3, `iproute2`, and SLiRP.
- A dedicated container with `NET_ADMIN`, device access to character devices
  `108:0` (PPP) and `10:200` (TUN). No host ports need to be published.
- The existing Wi-Fi-only SLiRP launcher and its libraries/configuration in
  `/opt/v90/wifi/`: `ppp-slirp.py`, `wifi-sockets.so`,
  `slirp-select-fix.so`, `wifi-uplink.json`. Keep relay credentials private;
  they are supplied at deployment and are not part of the image.

On an ordinary Linux host, a plain SLiRP launcher can also supply the IPv4
uplink, but that does not pin traffic to a Windows Wi-Fi interface.

## Deployment

Build the main repository image as `codex-v90-answerer:latest`, then:

```sh
docker build -t codex-v90-multilink:latest scripts/multilink
```

In the backend, supply `/opt/v90/multilink/server.json` with the modem
container's private address:

```json
{"allowed_clients": ["172.17.0.2"]}
```

Set these container sysctls: `net.ipv4.ip_forward=1`,
`net.ipv4.conf.all.rp_filter=0`, `net.ipv4.conf.default.rp_filter=0`.
The backend does not replace its main default route. A source-policy rule
routes only packets from the dial-up client through `wifi0` and the Wi-Fi
SLiRP adapter. Return packets route back to the shared PPP interface.

Copy `client.py` into the modem container, make it executable, and create
`client.json` beside it with the backend's private address:

```json
{"server": "172.17.0.3"}
```

Start the modem with `--pppd /opt/v90/multilink/client.py`. Other modem,
V.42/V.42bis, and Asterisk AudioSocket settings stay as usual. If the shared
uplink fails, the backend stops accepting modem streams instead of silently
using another internet interface.

The HTTP/HTTPS website proxy listens on backend port `9082`, reachable by
the PPP client as `http://10.0.2.2:9082`. No Docker host-port mapping is
required. The loopback proxy used by independent SLiRP sessions can remain
running in the modem container.

## Windows

Select both USB modems in one Dial-up connection, enable dialing all selected
devices, and dial `*995551000` on both ports for the tested PAP2 setup. Obtain
IPv4 automatically; use DNS `10.0.2.3`. This local endpoint does not require
authentication or PPP encryption. V.42 and V.42bis remain modem-level features
on each physical link and operate independently of Multilink PPP.

Server logs in `/var/log/v90-multilink/link-*.log` must show a newly created
bundle and another link attached to the same `pppN` interface. Merely seeing
two answered calls does not prove multilink. Dropping one link must leave
the bundle usable until the last link disconnects.

The tested desktop includes `outputs/Multilink/Enable-Multilink.ps1`,
`Disable-Multilink.ps1`, and `Test-Multilink.ps1` to select the backend,
restore single-link SLiRP, and verify both Windows modem subentries.

References: [pppd multilink documentation](https://ppp.samba.org/pppd.html),
[RFC 1990](https://www.rfc-editor.org/rfc/rfc1990).
