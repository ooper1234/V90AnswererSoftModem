#!/usr/bin/env bash
# All-in-one setup for the V.90 modem server: Asterisk (with the patched
# AudioSocket), the audio pacer and the modem answerer, in Docker.
#
#   ./setup.sh             ask everything, write the configs, build and start
#   ./setup.sh status      what is running, SIP registrations, recent calls
#   ./setup.sh logs        follow the logs (Ctrl-C to stop)
#   ./setup.sh restart     restart the stack
#   ./setup.sh stop        stop the stack
#   ./setup.sh uninstall   stop and remove the containers and images
#
# Answers are saved in deploy/generated/settings.env (readable only by you),
# so running it again offers them as defaults.

set -euo pipefail
cd "$(dirname "$0")"
ROOT=$PWD
DEPLOY=$ROOT/deploy
GEN=$DEPLOY/generated
SETTINGS=$GEN/settings.env
UUID=00000000-0000-0000-0000-000000000090

bold() { printf '\033[1m%s\033[0m\n' "$*"; }
info() { printf '  %s\n' "$*"; }
warn() { printf '\033[33m! %s\033[0m\n' "$*"; }
die()  { printf '\033[31mError: %s\033[0m\n' "$*" >&2; exit 1; }

# ask VAR "Question" default
ask() {
    local var=$1 q=$2 def=${3-} ans
    if [ -n "$def" ]; then read -r -p "  $q [$def]: " ans || true; else read -r -p "  $q: " ans || true; fi
    printf -v "$var" '%s' "${ans:-$def}"
}

# ask_yes VAR "Question" y|n
ask_yes() {
    local var=$1 q=$2 def=$3 ans
    while true; do
        read -r -p "  $q [$( [ "$def" = y ] && echo Y/n || echo y/N )]: " ans || true
        ans=${ans:-$def}
        case $ans in
            [Yy]*) printf -v "$var" y; return ;;
            [Nn]*) printf -v "$var" n; return ;;
        esac
    done
}

# ask_secret VAR "Question" (blank keeps the saved one, or makes one up)
ask_secret() {
    local var=$1 q=$2 cur=${!1-} ans
    if [ -n "$cur" ]; then read -r -s -p "  $q [Enter keeps the saved one]: " ans || true
    else read -r -s -p "  $q [Enter makes one up]: " ans || true; fi
    echo
    if [ -n "$ans" ]; then printf -v "$var" '%s' "$ans"
    elif [ -z "$cur" ]; then printf -v "$var" '%s' "$(head -c 24 /dev/urandom | base64 | tr -dc 'A-Za-z0-9' | cut -c1-16)"
    fi
}

valid_port() { [[ $1 =~ ^[0-9]+$ ]] && [ "$1" -ge 1024 ] && [ "$1" -le 65535 ]; }

port_busy() { # proto port
    local flag; [ "$1" = udp ] && flag=-lun || flag=-ltn
    ss $flag 2>/dev/null | awk '{print $4}' | grep -qE "[:.]$2\$"
}

find_docker() {
    if ! command -v docker >/dev/null 2>&1; then
        warn "Docker is not installed."
        if command -v apt-get >/dev/null 2>&1; then
            local yn; ask_yes yn "Install Docker now with apt (needs sudo)?" y
            [ "$yn" = y ] || die "Docker is needed. Install it and run ./setup.sh again."
            sudo apt-get update
            sudo apt-get install -y docker.io docker-compose-v2 2>/dev/null \
                || sudo apt-get install -y docker.io docker-compose
        else
            die "Install Docker and the compose plugin, then run ./setup.sh again."
        fi
    fi
    if docker info >/dev/null 2>&1; then DOCKER=(docker)
    elif sudo docker info >/dev/null 2>&1; then DOCKER=(sudo docker)
    else die "Docker is installed but not running (try: sudo systemctl start docker)."; fi
    if "${DOCKER[@]}" compose version >/dev/null 2>&1; then COMPOSE=("${DOCKER[@]}" compose)
    elif command -v docker-compose >/dev/null 2>&1; then
        if [ "${DOCKER[0]}" = sudo ]; then COMPOSE=(sudo docker-compose); else COMPOSE=(docker-compose); fi
    else die "Docker Compose is missing (install docker-compose-v2 or the compose plugin)."; fi
}

compose() { "${COMPOSE[@]}" -f "$DEPLOY/compose.yaml" --env-file "$GEN/stack.env" "$@"; }

need_setup() { [ -f "$GEN/stack.env" ] || die "Not set up yet. Run ./setup.sh first."; }

# --- the questions ----------------------------------------------------------

questions() {
    local lan_ip lan_net
    lan_ip=$(ip -4 route get 1.1.1.1 2>/dev/null | awk '{for(i=1;i<NF;i++) if($i=="src") print $(i+1)}' | head -1)
    lan_net=$(ip -4 -o addr show 2>/dev/null | awk -v ip="${lan_ip:-x}" '$4 ~ "^"ip"/" {print $4}' | head -1)
    if [ -n "$lan_net" ] && command -v python3 >/dev/null; then
        lan_net=$(python3 -c "import ipaddress,sys; print(ipaddress.ip_interface(sys.argv[1]).network)" "$lan_net")
    fi

    bold "1. This server"
    ask LAN_IP "This server's address on your network" "${LAN_IP:-$lan_ip}"
    ask LAN_NET "Your local network (devices allowed to register as an ATA)" "${LAN_NET:-${lan_net:-192.168.1.0/24}}"
    ask PUBLIC_IP "Public internet address, if callers come from a SIP provider through NAT (blank if not)" "${PUBLIC_IP-}"

    echo; bold "2. How modems call in"
    info "a) an ATA on your network (a modem plugged into its phone port)"
    info "b) a SIP provider (a phone number people dial)"
    ask_yes USE_ATA "Use an ATA?" "${USE_ATA:-y}"
    ask_yes USE_TRUNK "Use a SIP provider?" "${USE_TRUNK:-n}"
    [ "$USE_ATA" = y ] || [ "$USE_TRUNK" = y ] || die "Choose an ATA, a SIP provider, or both."

    if [ "$USE_ATA" = y ]; then
        echo; bold "3. The ATA's SIP account (you will type these into the ATA)"
        ask ATA_USER "SIP username for the ATA" "${ATA_USER:-modem1}"
        [[ $ATA_USER =~ ^[A-Za-z0-9_.-]+$ ]] || die "Use letters, digits, '.', '_' or '-' in the username."
        ask_secret ATA_PASS "SIP password for the ATA"
        ask MODEM_NUMBER "Number the modem dials" "${MODEM_NUMBER:-5551000}"
        [[ $MODEM_NUMBER =~ ^[0-9]+$ ]] || die "The number must be digits."
    fi

    if [ "$USE_TRUNK" = y ]; then
        echo; bold "4. The SIP provider (from your provider's account page)"
        ask TRUNK_HOST "Provider's SIP server (host or host:port)" "${TRUNK_HOST-}"
        [ -n "$TRUNK_HOST" ] || die "The provider's SIP server is needed."
        ask TRUNK_USER "Account username / number" "${TRUNK_USER-}"
        ask TRUNK_AUTH_USER "Authentication username" "${TRUNK_AUTH_USER:-$TRUNK_USER}"
        ask_secret TRUNK_PASS "Account password"
        ask_yes TRUNK_REGISTER "Register with the provider (needed for most accounts)?" "${TRUNK_REGISTER:-y}"
        info "Every call the provider sends is answered by the modem."
    fi

    echo; bold "5. Ports"
    ask SIP_PORT "SIP port (UDP)" "${SIP_PORT:-5060}"
    ask RTP_START "First RTP audio port (UDP; 100 are used)" "${RTP_START:-10000}"
    ask PACER_PORT "Internal AudioSocket port" "${PACER_PORT:-9310}"
    for p in "$SIP_PORT" "$RTP_START" "$PACER_PORT"; do valid_port "$p" || die "$p is not a usable port (1024-65535)."; done
    RTP_END=$((RTP_START + 99)); ANSWERER_PORT=$((PACER_PORT + 1))

    echo; bold "6. Modem modes"
    info "V.90, V.34, V.32bis, V.22bis, V.23, Bell 103, V.21 and HST are always on."
    ask_yes X2 "U.S. Robotics x2 (56k; needs a u-law line)" "${X2:-y}"
    ask_yes PCM_UP "V.92 PCM upstream (faster uploads; experimental)" "${PCM_UP:-y}"
    ask_yes QC "V.92 Quick Connect (faster reconnects)" "${QC:-y}"
}

summary() {
    echo; bold "Summary"
    info "Server address      $LAN_IP (local network $LAN_NET)"
    [ -n "$PUBLIC_IP" ] && info "Public address      $PUBLIC_IP"
    [ "$USE_ATA" = y ] && info "ATA account         $ATA_USER, modem dials $MODEM_NUMBER"
    [ "$USE_TRUNK" = y ] && info "SIP provider        $TRUNK_USER @ $TRUNK_HOST (register: $TRUNK_REGISTER)"
    info "Ports               SIP $SIP_PORT/udp, RTP $RTP_START-$RTP_END/udp"
    info "Modes               x2 $X2, V.92 PCM upstream $PCM_UP, Quick Connect $QC"
}

# --- writing the configs ----------------------------------------------------

write_configs() {
    umask 077
    mkdir -p "$GEN" "$DEPLOY/logs"
    chmod 700 "$GEN"

    # Saved answers, for the next run.
    {
        for v in LAN_IP LAN_NET PUBLIC_IP USE_ATA ATA_USER ATA_PASS MODEM_NUMBER USE_TRUNK TRUNK_HOST \
                 TRUNK_USER TRUNK_AUTH_USER TRUNK_PASS TRUNK_REGISTER SIP_PORT RTP_START PACER_PORT X2 PCM_UP QC; do
            printf '%s=%q\n' "$v" "${!v-}"
        done
    } > "$SETTINGS"

    printf 'PACER_PORT=%s\nANSWERER_PORT=%s\n' "$PACER_PORT" "$ANSWERER_PORT" > "$GEN/stack.env"

    yes1() { [ "$1" = y ] && echo 1 || echo 0; }
    cat > "$GEN/answerer.env" <<EOF
# The live server's settings for the V.92 build.
ECHO_TAPS=256
V90_ECHO_TRACKING=0
V8BIS_ENABLE=0
V90_PRIME_HANDOVER=1
V90_TRN1D=2.0
BM_EC_HEALTH=1
V92_RENEG_TRN2D=0
V92_QUICK_CONNECT=$(yes1 "$QC")
V92_QC_ANSWER=$(yes1 "$QC")
V92_QC_V8BIS=0
V92_QC_DELAY_COMPENSATION=1
V90_X2=$(yes1 "$X2")
V92_PCM_UPSTREAM=$(yes1 "$PCM_UP")
V92_PCM_ECHO=$(yes1 "$PCM_UP")
EOF

    cat > "$GEN/rtp.conf" <<EOF
[general]
rtpstart=$RTP_START
rtpend=$RTP_END
strictrtp=yes
EOF

    {
        cat <<EOF
; Written by setup.sh. Run ./setup.sh again to change it.
[global]
type=global
user_agent=V90 modem server

[transport-udp]
type=transport
protocol=udp
bind=0.0.0.0:$SIP_PORT
local_net=$LAN_NET
local_net=127.0.0.0/8
EOF
        if [ -n "$PUBLIC_IP" ]; then
            printf 'external_media_address=%s\nexternal_signaling_address=%s\nexternal_signaling_port=%s\n' "$PUBLIC_IP" "$PUBLIC_IP" "$SIP_PORT"
        fi
        if [ "$USE_ATA" = y ]; then
            cat <<EOF

; The ATA: u-law only, from your local network only.
[$ATA_USER-auth]
type=auth
auth_type=userpass
username=$ATA_USER
password=$ATA_PASS

[$ATA_USER]
type=aor
max_contacts=1
remove_existing=yes
qualify_frequency=30

[$ATA_USER]
type=endpoint
transport=transport-udp
context=from-ata
disallow=all
allow=ulaw
auth=$ATA_USER-auth
aors=$ATA_USER
direct_media=no
rtp_symmetric=yes
force_rport=yes
rewrite_contact=yes
ice_support=no
timers=no
deny=0.0.0.0/0.0.0.0
permit=$LAN_NET
EOF
        fi
        if [ "$USE_TRUNK" = y ]; then
            local host=${TRUNK_HOST%%:*}
            cat <<EOF

; The SIP provider.
[trunk-auth]
type=auth
auth_type=userpass
username=$TRUNK_AUTH_USER
password=$TRUNK_PASS

[trunk]
type=aor
contact=sip:$TRUNK_HOST

[trunk]
type=endpoint
transport=transport-udp
context=from-trunk
disallow=all
allow=ulaw
outbound_auth=trunk-auth
aors=trunk
from_user=$TRUNK_USER
from_domain=$host
direct_media=no
rtp_symmetric=yes
force_rport=yes
rewrite_contact=yes
ice_support=no
timers=no

[trunk-identify]
type=identify
endpoint=trunk
match=$host
EOF
            if [ "$TRUNK_REGISTER" = y ]; then
                cat <<EOF

[trunk-registration]
type=registration
transport=transport-udp
outbound_auth=trunk-auth
server_uri=sip:$TRUNK_HOST
client_uri=sip:$TRUNK_USER@$host
contact_user=$TRUNK_USER
retry_interval=60
forbidden_retry_interval=600
expiration=300
EOF
            fi
        fi
    } > "$GEN/pjsip.conf"

    {
        cat <<EOF
; Written by setup.sh. Run ./setup.sh again to change it.
[general]
static=yes
writeprotect=yes

; Every call ends up here: answered by the modem through the pacer.
[modem]
exten => s,1,Answer()
 same => n,AudioSocket($UUID,127.0.0.1:$PACER_PORT)
 same => n,Hangup()
EOF
        if [ "$USE_ATA" = y ]; then
            cat <<EOF

[from-ata]
exten => $MODEM_NUMBER,1,Goto(modem,s,1)
; A PAP2's *99 modem-mode prefix.
exten => *99$MODEM_NUMBER,1,Goto(modem,s,1)
EOF
        fi
        if [ "$USE_TRUNK" = y ]; then
            cat <<EOF

[from-trunk]
exten => _X.,1,Goto(modem,s,1)
exten => _+X.,1,Goto(modem,s,1)
exten => s,1,Goto(modem,s,1)
EOF
        fi
    } > "$GEN/extensions.conf"

    # Asterisk reads these inside its container as its own user; the folder
    # around them stays yours alone.
    chmod 644 "$GEN/pjsip.conf" "$GEN/extensions.conf" "$GEN/rtp.conf"
}

# --- after starting ---------------------------------------------------------

wait_ready() {
    local i
    for i in $(seq 1 30); do
        if "${DOCKER[@]}" exec v90modem-asterisk asterisk -rx "core show version" >/dev/null 2>&1; then return 0; fi
        sleep 2
    done
    return 1
}

check() {
    local ok=1
    echo; bold "Checks"
    if "${DOCKER[@]}" exec v90modem-asterisk asterisk -rx "module show like app_audiosocket" 2>/dev/null | grep -q Running; then
        info "Asterisk is running with the patched AudioSocket."
    else warn "Asterisk or its AudioSocket module is not running (./setup.sh logs)."; ok=0; fi
    if "${DOCKER[@]}" exec v90modem-answerer sh -c "cat /proc/net/tcp" 2>/dev/null | awk '{print $2, $4}' \
        | grep -qi ":$(printf '%04X' "$ANSWERER_PORT") 0A"; then
        info "The modem answerer is listening."
    else warn "The modem answerer is not listening yet (./setup.sh logs)."; ok=0; fi
    if [ "${USE_TRUNK:-n}" = y ] && [ "${TRUNK_REGISTER:-n}" = y ]; then
        local reg=""
        for _ in $(seq 1 10); do
            reg=$("${DOCKER[@]}" exec v90modem-asterisk asterisk -rx "pjsip show registrations" 2>/dev/null | grep -E 'trunk-registration' || true)
            echo "$reg" | grep -q Registered && break
            sleep 2
        done
        if echo "$reg" | grep -q Registered; then info "Registered with the SIP provider."
        else warn "Not registered with the SIP provider yet: check its server, username and password."; ok=0; fi
    fi
    [ $ok = 1 ]
}

instructions() {
    echo; bold "Next steps"
    if [ "${USE_ATA:-n}" = y ]; then
        info "On the ATA's line settings:"
        info "  Proxy / SIP server:   $LAN_IP:$SIP_PORT"
        info "  User ID / username:   $ATA_USER"
        info "  Password:             (the one you chose; ./setup.sh status --show-password shows it)"
        info "  Codec:                G.711 u-law only"
        info "  Silence suppression, echo cancellation, echo suppression: off"
        info "  Jitter buffer:        small and fixed"
        info "Then the modem dials $MODEM_NUMBER (on a PAP2: *99$MODEM_NUMBER)."
    fi
    if [ "${USE_TRUNK:-n}" = y ]; then
        info "Callers dial your provider's number. Behind a router, forward UDP $SIP_PORT and"
        info "$RTP_START-$RTP_END to $LAN_IP, and turn off the router's SIP ALG."
    fi
    info "Open the same UDP ports in this machine's firewall if it has one."
    info "Dial-up clients: no username or password; set DNS to 10.0.2.3."
    info "./setup.sh status shows what is running; ./setup.sh logs follows the calls."
}

# --- commands ---------------------------------------------------------------

load_settings() { if [ -f "$SETTINGS" ]; then set -a; . "$SETTINGS"; set +a; fi; }

cmd_setup() {
    bold "V.90 modem server setup"
    info "This builds and runs Asterisk, the audio pacer and the modem answerer in Docker."
    echo
    find_docker
    load_settings
    questions
    summary
    echo
    local go; ask_yes go "Write the configuration and start?" y
    [ "$go" = y ] || { info "Nothing changed."; exit 0; }

    # Ports this stack will use must be free, unless it is our own stack holding them.
    local ours; ours=$("${DOCKER[@]}" ps -q --filter name=^v90modem-asterisk$ 2>/dev/null || true)
    if [ -z "$ours" ]; then
        port_busy udp "$SIP_PORT" && die "UDP port $SIP_PORT is already in use (another SIP server?). Choose another SIP port."
        port_busy tcp "$PACER_PORT" && die "TCP port $PACER_PORT is in use. Choose another AudioSocket port."
        port_busy tcp "$ANSWERER_PORT" && die "TCP port $ANSWERER_PORT is in use. Choose another AudioSocket port."
    fi

    write_configs
    echo; bold "Building (the first build takes several minutes)"
    compose build
    echo; bold "Starting"
    compose up -d --force-recreate
    wait_ready || warn "Asterisk is slow to start."
    sleep 3
    check || true
    instructions
}

cmd_status() {
    find_docker; need_setup; load_settings
    bold "Containers"; compose ps
    echo; bold "SIP"
    "${DOCKER[@]}" exec v90modem-asterisk asterisk -rx "pjsip show contacts" 2>/dev/null | grep -E 'Contact:' || info "(no devices registered)"
    "${DOCKER[@]}" exec v90modem-asterisk asterisk -rx "pjsip show registrations" 2>/dev/null | grep -E 'Registered|Rejected|Unregistered|Failed' || true
    "${DOCKER[@]}" exec v90modem-asterisk asterisk -rx "core show channels count" 2>/dev/null | grep -i 'active call' || true
    echo; bold "Recent calls"
    "${DOCKER[@]}" logs --since 24h v90modem-answerer 2>&1 | grep -E 'DATA MODE|call summary' | tail -10 || true
    if [ "${1-}" = --show-password ] && [ "${USE_ATA:-n}" = y ]; then echo; info "ATA password: $ATA_PASS"; fi
}

main() {
    case ${1-setup} in
        setup|"") cmd_setup ;;
        status) shift; cmd_status "$@" ;;
        logs) find_docker; need_setup; compose logs -f --tail 50 ;;
        restart) find_docker; need_setup; compose restart ;;
        stop) find_docker; need_setup; compose stop ;;
        uninstall)
            find_docker; need_setup
            local yn; ask_yes yn "Stop and remove the modem server's containers and images (configs stay)?" n
            [ "$yn" = y ] && compose down --rmi local ;;
        -h|--help|help) sed -n '2,13p' "$0" | sed 's/^# \{0,1\}//' ;;
        *) die "Unknown command '$1' (try ./setup.sh help)." ;;
    esac
}

main "$@"
