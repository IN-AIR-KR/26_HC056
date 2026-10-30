#!/usr/bin/env bash
# 격리된 각도 발사 비충돌 안정성 시험 1회.
#   다른 SITL 세션(기본 ROS 도메인 / GZ 파티션 / PX4 인스턴스 0)과 겹치지 않게
#   ROS_DOMAIN_ID, GZ_PARTITION, PX4 인스턴스를 따로 쓰고, 끝나면 '자기 런치'에만 SIGINT 를 보낸다.
#
#   사용: SW/scripts/sitl_diag/run_isolated_stability.sh <앙각> [결과폴더] [phases]
#   예  : SW/scripts/sitl_diag/run_isolated_stability.sh 50 ~/arms_diag_logs/iso P1,P2,P3
set -euo pipefail
# 작업 제어(-m): 백그라운드 런치를 자기 프로세스 그룹에서 돌린다. 비대화형 bash 는 '&' 작업의
#   SIGINT 를 무시하게 띄우므로, 이게 없으면 종료 SIGINT 가 런치에 안 먹고 노드가 고아로 남는다.
set -m
ELEV="$(printf '%.1f' "${1:?앙각(deg) 필요}")"
OUT="${2:-$HOME/arms_diag_logs/isolated}"
PHASES="${3:-P1,P2,P3}"
MIN_FREE_MB="${MIN_FREE_MB:-2500}"
WS="$(cd "$(dirname "$0")/../../arms_ws" && pwd)"
mkdir -p "$OUT"

avail=$(awk '/MemAvailable/ {print int($2/1024)}' /proc/meminfo)
if [ "$avail" -lt "$MIN_FREE_MB" ]; then
  echo "가용 메모리 ${avail}MB < ${MIN_FREE_MB}MB — 다른 세션을 보호하려고 시작하지 않음" >&2
  exit 3
fi

set +u; source /opt/ros/humble/setup.bash; source "$WS/install/setup.bash"; set -u
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID_ISO:-77}" ROS_LOCALHOST_ONLY=1
export GZ_PARTITION="arms_iso_$$" HEADLESS=1
STAMP=$(date +%Y%m%d_%H%M%S)
LOG="$OUT/${STAMP}_elev${ELEV}_launch.log"
echo "격리: ROS_DOMAIN_ID=$ROS_DOMAIN_ID GZ_PARTITION=$GZ_PARTITION PX4 instance=1  log=$LOG"

ros2 launch arms_bringup arms_sitl.launch.py px4_instance:=1 gui:=false rviz:=false \
  auto_save:=false diag:=true diag_jsonl:="$OUT/${STAMP}_elev${ELEV}_monitor.jsonl" \
  launch_elev:="$ELEV" > "$LOG" 2>&1 &
LAUNCH_PID=$!
cleanup() {
  # 자기 런치의 프로세스 그룹에만 보낸다 (다른 세션은 다른 그룹이다).
  kill -INT -- "-$LAUNCH_PID" 2>/dev/null || true
  for _ in $(seq 1 60); do kill -0 "$LAUNCH_PID" 2>/dev/null || break; sleep 0.5; done
  kill -TERM -- "-$LAUNCH_PID" 2>/dev/null || true
  sleep 2
  # 혹시 남은 것: 이 시험의 GZ_PARTITION 을 가진 프로세스만 정리
  for p in $(pgrep -f "arms_|px4|gz sim|parameter_bridge" || true); do
    grep -qz "GZ_PARTITION=$GZ_PARTITION" "/proc/$p/environ" 2>/dev/null && kill -KILL "$p" 2>/dev/null || true
  done
}
trap cleanup EXIT

# 시험 중 다른 세션이 시작돼 메모리가 부족해지면 내 시험을 먼저 접는다(남의 Gazebo 가 OOM 으로 죽지 않게).
(
  while kill -0 "$LAUNCH_PID" 2>/dev/null; do
    a=$(awk '/MemAvailable/ {print int($2/1024)}' /proc/meminfo)
    if [ "$a" -lt "${ABORT_FREE_MB:-700}" ]; then
      echo "[watchdog] 가용 메모리 ${a}MB — 격리 시험 중단" | tee -a "$LOG"
      kill -INT -- "-$LAUNCH_PID" 2>/dev/null; pkill -INT -f "angled_stability_test.*$STAMP" || true
      break
    fi
    sleep 2
  done
) &

ros2 run arms_sim angled_stability_test --ros-args -p elev:="$ELEV" -p phases:="$PHASES" \
  -p drone_model:=arms_drone_1 -p out:="$OUT/${STAMP}_elev${ELEV}_test.jsonl" \
  2>&1 | tee "$OUT/${STAMP}_elev${ELEV}_test.log"
