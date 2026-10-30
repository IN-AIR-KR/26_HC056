#!/usr/bin/env bash
# Run in the Pi's logged-in desktop session, without sudo.
set -euo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")"

fail() { printf 'ERROR: %s\n' "$*" >&2; exit 1; }
if [[ ${1:-} == --help || ${1:-} == -h ]]; then
    printf '%s\n' 'Usage: ./gui.sh camera|hailo [command ...]' \
        'Run from the Pi desktop terminal. Start the host camera publisher first.' \
        'Defaults: 30-second preview; hailo reads /models/model.hef.'
    exit 0
fi
[[ ${1:-} == camera || ${1:-} == hailo ]] || fail 'Choose camera or hailo; see ./gui.sh --help'
command -v docker >/dev/null || fail 'Docker is not installed'
command -v xauth >/dev/null || fail 'Install host xauth: sudo apt install xauth'
docker compose version >/dev/null
[[ ${DISPLAY:-} =~ ^:([0-9]+)(\.[0-9]+)?$ ]] || \
    fail 'Use the Pi desktop terminal with a local DISPLAY (e.g. :0), not SSH X forwarding'
display_number=${BASH_REMATCH[1]}
[[ -S /tmp/.X11-unix/X${display_number} ]] || \
    fail "Local X11/Xwayland socket X${display_number} not found"

auth_dir=$(mktemp -d /tmp/raspi5-gui.XXXXXXXX)
cleanup() { rm -rf -- "$auth_dir"; }
trap cleanup EXIT
export GUI_XAUTHORITY="$auth_dir/session.xauth"
touch "$GUI_XAUTHORITY"
chmod 600 "$GUI_XAUTHORITY"
# FamilyWild matches the container hostname, which differs from the host.
# The cookie remains private and is never printed or committed.
cookies=$(xauth nlist "$DISPLAY")
[[ -n $cookies ]] || fail 'No Xauthority cookie found. Run from a terminal in the Pi desktop session; check XAUTHORITY'
printf '%s\n' "$cookies" | sed 's/^..../ffff/' | xauth -f "$GUI_XAUTHORITY" nmerge -
unset cookies
[[ -s $GUI_XAUTHORITY ]] || fail 'Could not create Xauthority file'

docker compose -f compose.yaml -f compose.gui.yaml run --rm "$@"
