"""launcher_geometry 의 카메라/자세 기하 검증 (ROS 불필요)."""
import math

import pytest

from arms_sim import launcher_geometry as G


def test_vertical_attitude_matches_px4_convention():
    # 수직 장착: 기수(body +X)가 월드 +X(East) → PX4 yaw 90°, roll/pitch 0.
    r, p, y = G.expected_px4_euler_deg(90.0, 0.0)
    assert r == pytest.approx(0.0, abs=1e-6)
    assert p == pytest.approx(0.0, abs=1e-6)
    assert y == pytest.approx(90.0, abs=1e-6)


@pytest.mark.parametrize("elev", [60.0, 50.0, 40.0])
@pytest.mark.parametrize("az", [0.0, 45.0, -90.0])
def test_tilted_attitude_is_pure_pitch(elev, az):
    r, p, y = G.expected_px4_euler_deg(elev, az)
    assert r == pytest.approx(0.0, abs=1e-6)
    assert p == pytest.approx(-(90.0 - elev), abs=1e-6)
    # ENU 방위 az(반시계, +X 기준) → NED yaw = 90° - az
    assert ((y - (90.0 - az) + 180.0) % 360.0) - 180.0 == pytest.approx(0.0, abs=1e-6)


@pytest.mark.parametrize("elev", [90.0, 60.0, 50.0, 40.0])
def test_boresight_is_tube_axis(elev):
    bz = G.body_axes_enu(elev, 30.0)[2]
    d = G.axis(elev, 30.0)
    assert all(a == pytest.approx(b, abs=1e-9) for a, b in zip(bz, d))
    # 광축 앙각 = 발사 앙각
    assert math.degrees(math.asin(bz[2])) == pytest.approx(elev, abs=1e-9)


def test_fov_values_match_sdf():
    assert math.degrees(G.CAM_HFOV_RAD) == pytest.approx(43.6, abs=0.05)
    assert math.degrees(G.camera_vfov_rad()) == pytest.approx(29.86, abs=0.05)


@pytest.mark.parametrize("elev", [90.0, 50.0])
def test_target_on_boresight_projects_to_center(elev):
    tgt = G.boresight_point(elev, 0.0, 25.0)
    pr = G.project_target(elev, 0.0, tgt)
    assert pr["in_fov"]
    assert pr["off_axis_deg"] == pytest.approx(0.0, abs=1e-6)
    assert pr["x_center"] == pytest.approx(0.5, abs=1e-9)
    assert pr["y_center"] == pytest.approx(0.5, abs=1e-9)
    assert pr["range_m"] == pytest.approx(25.0, abs=1e-9)


def test_image_axes_orientation():
    # 수직 장착에서 월드 +X(=body +X)로 비낀 표적은 화면 아래, 월드 -Y(=body -Y)는 화면 오른쪽.
    c = G.camera_position(90.0, 0.0)
    down = G.project_target(90.0, 0.0, (c[0] + 2.0, c[1], c[2] + 20.0))
    right = G.project_target(90.0, 0.0, (c[0], c[1] - 2.0, c[2] + 20.0))
    assert down["y_center"] > 0.5 and down["x_center"] == pytest.approx(0.5)
    assert right["x_center"] > 0.5 and right["y_center"] == pytest.approx(0.5)


def test_default_park_target_leaves_fov_when_tilted():
    # 기본 표적 (5, -5, 30) 은 수직 발사에선 화면 안, 50°/40° 로 기울이면 화면 밖이다.
    assert G.project_target(90.0, 0.0, (5.0, -5.0, 30.0))["in_fov"]
    assert not G.project_target(50.0, 0.0, (5.0, -5.0, 30.0))["in_fov"]
    assert not G.project_target(40.0, 0.0, (5.0, -5.0, 30.0))["in_fov"]


def test_target_behind_camera_is_not_in_fov():
    c = G.camera_position(50.0, 0.0)
    pr = G.project_target(50.0, 0.0, (c[0] - 10.0, c[1], c[2] - 10.0))
    assert not pr["in_fov"]


def test_enu_to_ned_conversion_matches_px4_conventions():
    # 단위 쿼터니언(기체 FLU = 월드 ENU): 기수가 East → PX4 yaw 90°, 수평.
    r, p, y = G.enu_flu_quat_to_px4_euler_deg((0.0, 0.0, 0.0, 1.0))
    assert (r, p) == pytest.approx((0.0, 0.0), abs=1e-9) and y == pytest.approx(90.0)
    # ENU yaw +90°(기수 North) → PX4 yaw 0°.
    s = math.sin(math.pi / 4)
    assert G.enu_flu_quat_to_px4_euler_deg((0.0, 0.0, s, s))[2] == pytest.approx(0.0, abs=1e-9)
    # FLU 기준 +Y 축 회전(기수 아래로) → FRD pitch 음수(기수 하강).
    th = math.radians(20.0)
    q = (0.0, math.sin(th / 2), 0.0, math.cos(th / 2))
    assert G.enu_flu_quat_to_px4_euler_deg(q)[1] == pytest.approx(-20.0, abs=1e-9)
    # FLU +X 축 회전(왼쪽 날개 올라감 = 오른쪽으로 기움) → FRD roll 양수.
    q = (math.sin(th / 2), 0.0, 0.0, math.cos(th / 2))
    assert G.enu_flu_quat_to_px4_euler_deg(q)[0] == pytest.approx(20.0, abs=1e-9)


def test_angle_difference_wraps():
    assert G.angle_difference_deg(179.0, -179.0) == pytest.approx(-2.0)
    assert G.angle_difference_deg(-179.0, 179.0) == pytest.approx(2.0)


def test_validate_px4_attitude_reports_observed_50deg_mismatch():
    # 이전 SITL 50° 실측값: 기대 (0, -40, 90) 대비 최대 7.7° 차이 → 3° 허용치에서 불합격.
    r = G.validate_px4_attitude(50.0, 0.0, -6.4, -32.3, 93.6)
    assert not r["valid"]
    assert r["max_error_deg"] == pytest.approx(7.7, abs=1e-6)
    assert G.validate_px4_attitude(50.0, 0.0, 0.5, -39.0, 90.4)["valid"]
