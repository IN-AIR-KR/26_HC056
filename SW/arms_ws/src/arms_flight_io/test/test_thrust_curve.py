"""PX4 Stabilized 추력 곡선 역변환 검증 (ROS/PX4 불필요).

PX4 mc_att_control::throttle_curve (MPC_THR_CURVE=2):
  stick -1/0/+1 → MPC_MANTHR_MIN / MPC_THR_HOVER / MPC_THR_MAX (구간 선형).
sitl_bridge 는 자동 유도의 물리 추력을 이 곡선의 역함수로 스틱에 싣는다.
"""
import pytest

from arms_flight_io.sitl_bridge_node import thrust_to_stabilized_stick

MIN, HOVER, MAX = 0.08, 0.32, 1.0


def px4_throttle_curve(stick, thr_min=MIN, hover=HOVER, thr_max=MAX):
    if stick <= 0.0:
        return thr_min + (stick + 1.0) * (hover - thr_min)
    return hover + stick * (thr_max - hover)


@pytest.mark.parametrize("thrust", [0.10, 0.20, 0.32, 0.45, 0.60, 0.95])
def test_roundtrip_through_px4_curve(thrust):
    stick = thrust_to_stabilized_stick(thrust, MIN, HOVER, MAX)
    assert px4_throttle_curve(stick) == pytest.approx(thrust, abs=1e-9)


def test_hover_maps_to_center_stick():
    assert thrust_to_stabilized_stick(HOVER, MIN, HOVER, MAX) == pytest.approx(0.0)


def test_limits():
    assert thrust_to_stabilized_stick(0.0, MIN, HOVER, MAX) == -1.0
    assert thrust_to_stabilized_stick(1.5, MIN, HOVER, MAX) == 1.0


def test_old_linear_mapping_was_wrong():
    # 예전: RC = 추력 그대로(0.32 → stick -0.36). PX4 곡선을 거치면 실제 추력 ≈0.234 (호버의 73%).
    old_stick = 0.32 * 2.0 - 1.0
    assert px4_throttle_curve(old_stick) == pytest.approx(0.2336, abs=1e-4)
