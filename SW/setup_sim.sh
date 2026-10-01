#!/usr/bin/env bash
# =============================================================================
#  A.R.M.S. SITL 셋업 — clone 후 한 번만 실행.
#  모델 링크 → airframe 등록 → PX4 빌드 → ROS2 워크스페이스 빌드 까지 전부 처리.
#  이거 끝나면 ros2 launch arms_bringup arms_sitl.launch.py 로 바로 실행.
#
#  전제: ROS2 Humble / Gazebo Harmonic / MAVSDK 설치됨,
#        PX4-Autopilot 이 ~/PX4-Autopilot 에 clone 돼 있음.
#        (PX4 위치 다르면:  PX4_DIR=<경로> ./setup_sim.sh)
# =============================================================================

# source 로 실행하면 아래 set -e / exit 가 현재 셸에 걸려 실패 순간 터미널이 닫힌다.
#   ($0 도 셸 이름이 되어 경로 계산부터 틀어진다.) → 반드시 ./setup_sim.sh 로 실행.
if [ -n "${BASH_SOURCE[0]:-}" ] && [ "${BASH_SOURCE[0]}" != "$0" ]; then
  echo "!! source 하지 말고 실행하세요:  ./setup_sim.sh   (또는 bash setup_sim.sh)"
  return 1
fi

set -e

PX4_DIR="${PX4_DIR:-$HOME/PX4-Autopilot}"
SW_DIR="$(cd "$(dirname "$(readlink -f "$0")")" && pwd)"
SIM_SRC="$SW_DIR/arms_ws/src/arms_sim/assets"   # 시뮬 자산(models/worlds/px4) 은 arms_sim/assets
AIRFRAME_SRC="$SIM_SRC/px4/airframes/4022_gz_arms_drone"
PX4_AIRFRAME_DIR="$PX4_DIR/ROMFS/px4fmu_common/init.d-posix/airframes"
PX4_MODELS_DIR="$PX4_DIR/Tools/simulation/gz/models"
PX4_WORLDS_DIR="$PX4_DIR/Tools/simulation/gz/worlds"
CMAKE="$PX4_AIRFRAME_DIR/CMakeLists.txt"

# --- PX4 폴더 있는지 확인 ---
if [ ! -d "$PX4_DIR" ]; then
  echo "!! PX4-Autopilot 이 $PX4_DIR 에 없음."
  echo "   git clone https://github.com/PX4/PX4-Autopilot.git --recursive 먼저 하고,"
  echo "   다른 위치면 PX4_DIR=<경로> ./setup_sim.sh 로 실행."
  exit 1
fi

echo "[1/5] airframe 복사 (4022_gz_arms_drone)"
# 구버전 4021 잔재 제거 (x500_flow와 번호 충돌 방지)
#   파일뿐 아니라 CMake 등록 줄도 지워야 함. 등록 줄만 남으면 CMake 가
#   존재하지 않는 4021 파일을 참조하다 설정 단계에서 실패한다.
rm -f "$PX4_AIRFRAME_DIR/4021_gz_arms_drone"
sed -i '/^\t4021_gz_arms_drone$/d' "$CMAKE"
cp "$AIRFRAME_SRC" "$PX4_AIRFRAME_DIR/"

echo "[2/5] airframe CMake 등록"
if ! grep -q "4022_gz_arms_drone" "$CMAKE"; then
  if grep -q "4001_gz_x500" "$CMAKE"; then
    sed -i 's/\(4001_gz_x500\)/\1\n\t4022_gz_arms_drone/' "$CMAKE"
    echo "      추가됨"
  else
    echo "      !! 경고: CMakeLists 에 4001_gz_x500 패턴 없음 → 수동 등록 필요"
  fi
else
  echo "      이미 등록됨"
fi

echo "[3/5] 드론 모델 / 월드 심볼릭 링크 (레포 → PX4 트리)"
#   PX4 가 'gz_arms_drone' 스폰 시 모델을, PX4_GZ_WORLD 로 월드를 PX4 트리에서 찾으므로
#   레포의 폴더/파일을 PX4 경로에 링크해 둔다. (ln -sfn = 있으면 갱신)
#   px4_sitl.launch.py 도 기동 때마다 같은 링크를 갱신하므로, 여기 건 수동 실행
#   (make px4_sitl gz_arms_drone) 용 겸 최초 셋업.
mkdir -p "$PX4_MODELS_DIR" "$PX4_WORLDS_DIR"
# arms_drone 는 x500 구조를 인라인하고 자체 메시를 쓰므로 x500/x500_base 불필요.
# 이전 셋업에서 걸어둔 stale 링크가 있으면 정리.
for m in x500 x500_base; do
  [ -L "$PX4_MODELS_DIR/$m" ] && rm -f "$PX4_MODELS_DIR/$m" && echo "      정리: stale 링크 $m 제거"
done
for m in arms_drone; do
  if [ -d "$SIM_SRC/models/$m" ]; then
    ln -sfn "$SIM_SRC/models/$m" "$PX4_MODELS_DIR/$m"
    echo "      링크: $PX4_MODELS_DIR/$m -> arms_sim/assets/models/$m"
  fi
done
for w in "$SIM_SRC"/worlds/*.sdf; do
  ln -sfn "$w" "$PX4_WORLDS_DIR/$(basename "$w")"
  echo "      링크: $PX4_WORLDS_DIR/$(basename "$w") -> arms_sim/assets/worlds/$(basename "$w")"
done

echo "[4/5] PX4 SITL 빌드 (수 분 소요)"
( cd "$PX4_DIR" && make px4_sitl )

echo "      저장된 PX4 params 초기화 (airframe 기본값 적용을 위해)"
# PX4 SITL params 저장 파일 삭제 → 다음 시작 시 airframe 기본값으로 fresh 시작
# (= 실행 중 param reset_all 과 동일 효과)
# 실제 저장 경로는 rootfs/parameters*.bson (rootfs/eeprom/parameters 아님 — 예전 경로라
# 여기서 지워지지 않고 넘어가면, 이전 세션에서 저장된 값이 airframe 기본값을 계속
# 덮어써서 param set-default 수정이 반영 안 된 것처럼 보이는 문제가 있었음).
PARAMS_DIR="$PX4_DIR/build/px4_sitl_default/rootfs"
removed=0
for f in "$PARAMS_DIR/parameters.bson" "$PARAMS_DIR/parameters_backup.bson" "$PARAMS_DIR/eeprom/parameters"; do
  if [ -f "$f" ]; then
    rm -f "$f"
    echo "      삭제됨: $f"
    removed=1
  fi
done
if [ "$removed" -eq 0 ]; then
  echo "      (params 파일 없음 — 빌드 후 자동 생성됨)"
fi

echo "[5/5] ROS2 워크스페이스 빌드"
source /opt/ros/humble/setup.bash
( cd "$SW_DIR/arms_ws" && colcon build --symlink-install )

echo ""
echo "============================================"
echo " 셋업 완료! 실행:"
echo "   source arms_ws/install/setup.bash"
echo "   ros2 launch arms_bringup arms_sitl.launch.py"
echo "============================================"
