#!/usr/bin/env bash
# Nano-only launcher. No startup service; run action requires an explicit user command.
set -eo pipefail
action="${1:-check}"
route="${2:-1}"
case "$action" in
  check|run|vision|status|stop|stop-vision|smoke-check|smoke-run) ;;
  *) echo 'Use: competitionctl.sh check|run|vision|status|stop|stop-vision|smoke-check|smoke-run [1..4]' >&2; exit 2;;
esac
case "$action" in
  smoke-check|smoke-run|vision|status|stop|stop-vision) ;;
  *)
    case "$route" in 1|2|3|4) ;; *) echo 'Route must be 1..4' >&2; exit 2;; esac
    ;;
esac
ws="$HOME/simple_flight_ws"
visual="$HOME/visual_recognition_ws"
log="$ws/log"
smoke_cfg="$ws/config/route_smoke.yaml"
mkdir -p "$log"
pidfile="$log/competition.pid"
smokepid="$log/smoke.pid"
visionpid="$log/competition_vision.pid"

owned_pid() {
  local file="$1" pattern="$2" pid
  [ -f "$file" ] || return 1
  read -r pid < "$file"
  [[ "$pid" =~ ^[0-9]+$ ]] || return 1
  [ -r "/proc/$pid/cmdline" ] || return 1
  tr '\0' ' ' < "/proc/$pid/cmdline" | grep -Fq "$pattern" || return 1
  printf '%s' "$pid"
}

if [ "$action" = status ]; then
  cat "$log/competition_status.json" 2>/dev/null || true
  exit 0
fi
if [ "$action" = stop ]; then
  if pid="$(owned_pid "$pidfile" 'simple_flight_mission.competition_node')"; then
    kill -INT "$pid"
    echo 'Stop requested: normal handover to pilot, not automatic landing.'
  elif pid="$(owned_pid "$smokepid" 'simple_flight_mission.route_node')"; then
    kill -INT "$pid"
    echo 'Smoke stop requested: normal handover to pilot, not automatic landing.'
  else echo 'No owned competition/smoke process running.'; fi
  exit 0
fi
if [ "$action" = stop-vision ]; then
  if owned_pid "$pidfile" 'simple_flight_mission.competition_node' >/dev/null; then
    echo 'Refusing to stop vision during a running mission.' >&2; exit 3
  fi
  if owned_pid "$smokepid" 'simple_flight_mission.route_node' >/dev/null; then
    echo 'Refusing to stop vision during a running smoke mission.' >&2; exit 3
  fi
  if pid="$(owned_pid "$visionpid" 'tools/serve.py')"; then
    kill -INT "$pid"
    for ((i=0; i<50; i++)); do
      if ! kill -0 "$pid" 2>/dev/null; then echo 'Recording closed; video is ready to copy.'; exit 0; fi
      sleep .2
    done
    echo 'Vision is still closing; refusing to export an unfinished AVI.' >&2; exit 3
  elif curl --fail --silent --max-time 1 http://127.0.0.1:8088/api/status >/dev/null; then
    echo 'Vision owner PID is unavailable; cannot confirm recording is closed.' >&2; exit 3
  fi
  exit 0
fi

ensure_vision() {
  if curl --fail --silent --max-time 1 http://127.0.0.1:8088/api/status >/dev/null; then
    :
  else
    name="$(python3 "$ws/src/simple_flight_payload/scripts/recording_name.py")"
    nohup bash "$visual/start_vision.sh" --source camera --record "$log/$name.avi" \
      --mission-status "$log/competition_status.json" 8>&- 9>&- > "$log/$name.vision.log" 2>&1 < /dev/null &
    printf '%s\n' "$!" > "$visionpid"
    printf '%s\n' "$name.avi" > "$log/latest_recording.txt"
  fi
  for ((i=0; i<30; i++)); do
    if python3 - <<'PY'
import json,sys,urllib.request
try:
    with urllib.request.urlopen('http://127.0.0.1:8088/api/status',timeout=.5) as response:
        data=json.load(response)
    okay=data.get('source')=='camera' and data.get('valid') is True and (data.get('recording') or {}).get('active') is True
    sys.exit(0 if okay else 1)
except Exception:
    sys.exit(1)
PY
    then return 0; fi
    sleep .5
  done
  echo 'Camera/recording not ready. Check vision log; do not start flight.' >&2
  return 2
}

if [ "$action" = vision ]; then ensure_vision; echo 'Camera and recording ready: http://10.42.0.1:8088'; exit 0; fi

# Empty short hop: takeoff → route1 first segment 2.87m forward → return home.
# Uses route_node (no vision, no door drop). SWA low→high starts execute.
if [ "$action" = smoke-check ] || [ "$action" = smoke-run ]; then
  [ -f "$smoke_cfg" ] || { echo "Missing $smoke_cfg" >&2; exit 2; }
  exec 9>"$log/smoke.lock"
  if ! flock -n 9; then echo 'Another smoke mission is running.' >&2; exit 3; fi
  exec 8>"$log/competition.lock"
  if ! flock -n 8; then echo 'Competition preparation/mission is running; refuse smoke.' >&2; exit 3; fi
  if owned_pid "$pidfile" 'simple_flight_mission.competition_node' >/dev/null; then
    echo 'Competition mission is running; refuse smoke.' >&2; exit 3
  fi
  source /opt/ros/humble/setup.bash
  source "$HOME/livox_ws/install/setup.bash"
  source "$HOME/fastlio_ws/install/setup.bash"
  source "$ws/install/setup.bash"
  export ROS_DOMAIN_ID=0
  unset ROS_LOCALHOST_ONLY
  export PYTHONUNBUFFERED=1
  printf '%s\n' "$$" > "$smokepid"
  mode=--check
  extra=(--duration 8)
  if [ "$action" = smoke-run ]; then
    mode=--execute
    extra=()
    echo 'Smoke execute: Position档、SWA先低后高触发；拨低SWA或切手动可接管。无视觉投放、不驱动比赛舱门逻辑。'
  fi
  exec python3 -m simple_flight_mission.route_node --config "$smoke_cfg" "$mode" "${extra[@]}"
fi

exec 9>"$log/competition.lock"
if ! flock -n 9; then echo 'Another competition mission is running.' >&2; exit 3; fi
exec 8>"$log/smoke.lock"
if ! flock -n 8; then echo 'Another smoke mission is running.' >&2; exit 3; fi
if [ "$action" = run ] && [ ! -t 0 ]; then
  echo 'run 需要交互终端确认：请在 SSH 终端直接运行，准备完成后按 Enter。' >&2
  exit 2
fi
source /opt/ros/humble/setup.bash
source "$HOME/livox_ws/install/setup.bash"
source "$HOME/fastlio_ws/install/setup.bash"
source "$ws/install/setup.bash"
export ROS_DOMAIN_ID=0
unset ROS_LOCALHOST_ONLY
export PYTHONUNBUFFERED=1
ensure_vision
printf '%s\n' "$$" > "$pidfile"
mode=--check
extra=()
if [ "$action" = run ]; then
  mode=--execute
  extra=(--wait-enter)
  echo '正在完成关门、静止检查和目标预发送；准备完成后等待Enter，确认前不会请求OFFBOARD或解锁。'
fi
exec python3 -m simple_flight_mission.competition_node --route "$route" "$mode" "${extra[@]}" \
  --start-trigger notebook --door-config "$ws/config/doors.json" \
  --status-file "$log/competition_status.json" --report-dir "$log/competition"
