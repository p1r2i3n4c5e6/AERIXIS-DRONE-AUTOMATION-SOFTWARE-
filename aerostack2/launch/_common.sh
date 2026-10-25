# Shared helpers for launch/*.sh (sourced, not run). Each component runs in the background in
# its own process group; logs and PIDs live in $AERIXIS_STATE (default ~/.local/share/aerixis).
REPO="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
GCS_DIR="${AERIXIS_GCS_DIR:-$HOME/custom_gcs}"
STATE="${AERIXIS_STATE:-$HOME/.local/share/aerixis}"
mkdir -p "$STATE/logs" "$STATE/run"

say()  { printf '\033[1m%s\033[0m\n' "$*"; }
warn() { printf '\033[33m%s\033[0m\n' "$*" >&2; }

running() {   # running NAME -> true if its process group is alive
  local pid; pid=$(cat "$STATE/run/$1.pid" 2>/dev/null) || return 1
  kill -0 -- "-$pid" 2>/dev/null
}

start_bg() {  # start_bg NAME COMMAND... -> background process group, output to logs/NAME.log
  local name=$1; shift
  if running "$name"; then warn "$name is already running (pid $(cat "$STATE/run/$name.pid"))"; return 0; fi
  setsid "$@" > "$STATE/logs/$name.log" 2>&1 < /dev/null &
  echo $! > "$STATE/run/$name.pid"
  echo "  $name started (log: $STATE/logs/$name.log)"
}

stop_bg() {   # stop_bg NAME [seconds] -> Ctrl-C the group, then TERM, then kill it if it does not exit
  local name=$1 wait=${2:-15} pid
  pid=$(cat "$STATE/run/$name.pid" 2>/dev/null) || { echo "  $name: not running"; return 0; }
  if kill -0 -- "-$pid" 2>/dev/null; then
    kill -INT -- "-$pid" 2>/dev/null || true
    # A background bash script ignores SIGINT (and cannot trap it), so its cleanup trap only
    # runs on TERM — that is how sitl_onboard.sh / 3_launch_simulation.sh stop their own sessions.
    for _ in 1 2 3; do kill -0 -- "-$pid" 2>/dev/null || break; sleep 1; done
    kill -TERM -- "-$pid" 2>/dev/null || true
    for _ in $(seq 1 "$wait"); do kill -0 -- "-$pid" 2>/dev/null || break; sleep 1; done
    kill -0 -- "-$pid" 2>/dev/null && { kill -KILL -- "-$pid" 2>/dev/null || true; }
    echo "  $name stopped"
  else
    echo "  $name: not running"
  fi
  rm -f "$STATE/run/$name.pid"
}

status_bg() { # status_bg NAME...
  for name in "$@"; do
    if running "$name"; then printf '  %-12s \033[32mrunning\033[0m (pid %s)\n' "$name" "$(cat "$STATE/run/$name.pid")"
    else printf '  %-12s \033[2mstopped\033[0m\n' "$name"; fi
  done
}

logs_bg() { tail -n 60 -F "$STATE/logs/$1.log"; }

wait_http() { # wait_http URL SECONDS
  for _ in $(seq 1 "$2"); do curl -s -o /dev/null --max-time 1 "$1" && return 0; sleep 1; done; return 1
}
