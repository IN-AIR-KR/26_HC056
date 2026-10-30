"""
launcher_geometry.py — 견착 발사기 SITL 의 기하 상수 (단일 출처)  [arms_sim]

이 값들을 쓰는 곳:
  - launcher.py            : 장전 위치·자세, kinematic 발사관 pose, rviz 마커
  - px4_sitl.launch.py     : 최초 스폰 위치 (PX4_GZ_MODEL_POSE)
  - assets/worlds/arms_sitl.sdf 의 arms_launcher 모델
      ★ SDF 는 파이썬을 못 읽으므로 PIVOT_Z, CG_DIST 를 바꾸면 SDF 의
        <pose> 높이와 튜브 visual 길이도 같이 바꿀 것 (주석에 표시해둠).

좌표 약속
  - 피벗 = 어깨 받침점(회전 중심). 월드 (PIVOT_X, PIVOT_Y, PIVOT_Z).
  - 앙각 E: 지면 기준 [deg]. 90 = 수직(지금까지의 이륙 자세), 0 = 수평.
  - 방위 ψ: 월드 +X 기준, 반시계(+Z) 방향 [deg].
  - 튜브 축 방향 d = (cosE·cosψ, cosE·sinψ, sinE). 드론 기수(body +Z)가 d 를 향한다.
  - 드론 무게중심 = 피벗 + CG_DIST · d.
  - 드론 자세 = Rz(ψ) · Ry(90°−E)   (E=90, ψ=0 이면 지금처럼 똑바로 선 자세)
"""

import math

# ── 실측/결정값 ───────────────────────────────────────────────────────────
PIVOT_X = 0.0
PIVOT_Y = 0.0
PIVOT_Z = 1.20          # 어깨 받침점 지면 높이 [m] (사용자 지정 1.2m)

# 피벗 → 드론 무게중심, 튜브 축을 따라 잰 거리 [m]
#   = (어깨 받침점 → 드론 꼬리 끝, 튜브 축 방향)  +  (꼬리 끝 → 무게중심 0.191)
#   ★ 앞부분(어깨→꼬리)은 아직 실측 전 → 사진 기준 추정 0.61m 를 임시로 쓴다.
#     실측하면 TAIL_FROM_PIVOT 만 바꾸고 SDF 튜브 길이 주석 따라 같이 수정.
TAIL_FROM_PIVOT = 0.61
CG_FROM_TAIL = 0.191    # model.sdf: 꼬리 z=-0.191, base_link 원점 = 무게중심 (실측 일치)
CG_DIST = TAIL_FROM_PIVOT + CG_FROM_TAIL   # = 0.801

ELEV_MIN_DEG = 0.0
ELEV_MAX_DEG = 90.0
AZ_MIN_DEG = -180.0
AZ_MAX_DEG = 180.0
ELEV_SPAWN_DEG = 90.0   # 최초 스폰은 수직(지금과 동일). 장착된 뒤 패널로 기울인다.
AZ_SPAWN_DEG = 0.0


def axis(elev_deg, az_deg):
    """튜브 축(=드론 기수) 단위벡터."""
    e = math.radians(elev_deg)
    a = math.radians(az_deg)
    return (math.cos(e) * math.cos(a), math.cos(e) * math.sin(a), math.sin(e))


def drone_position(elev_deg, az_deg):
    """튜브에 장착된 드론 무게중심의 월드 좌표."""
    d = axis(elev_deg, az_deg)
    return (PIVOT_X + CG_DIST * d[0],
            PIVOT_Y + CG_DIST * d[1],
            PIVOT_Z + CG_DIST * d[2])


def _quat_mul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz)


def drone_orientation(elev_deg, az_deg):
    """장착 자세 쿼터니언 (x, y, z, w) = Rz(ψ)·Ry(90°−E)."""
    tilt = math.radians(90.0 - elev_deg)
    yaw = math.radians(az_deg)
    qy = (0.0, math.sin(tilt / 2), 0.0, math.cos(tilt / 2))
    qz = (0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2))
    return _quat_mul(qz, qy)


def spawn_pose_str():
    """PX4_GZ_MODEL_POSE 형식 'x,y,z,roll,pitch,yaw' (최초 스폰 = 수직 장착)."""
    x, y, z = drone_position(ELEV_SPAWN_DEG, AZ_SPAWN_DEG)
    pitch = math.radians(90.0 - ELEV_SPAWN_DEG)
    yaw = math.radians(AZ_SPAWN_DEG)
    return f"{x:.4f},{y:.4f},{z:.4f},0,{pitch:.6f},{yaw:.6f}"


# ── 상방 카메라 (assets/models/arms_drone/model.sdf 의 upward_camera 와 같아야 함) ──
#   <pose>0 0 0.15 0 -1.5708 0</pose> → 광축 = body +Z (FLU), 무게중심에서 +Z 0.15 m.
#   이미지 오른쪽 = body -Y, 이미지 아래 = body +X  (control_params.yaml 의 r_bc 와 동일:
#   OpenCV optical(right, down, forward) → FRD(+y, +x, -z)).
CAM_OFFSET_Z = 0.15
CAM_HFOV_RAD = 0.7610   # 43.6°
CAM_WIDTH = 720
CAM_HEIGHT = 480


def camera_vfov_rad():
    """정사각 픽셀 핀홀 가정의 수직 화각 [rad] (= 29.9°)."""
    return 2.0 * math.atan(math.tan(CAM_HFOV_RAD / 2.0) * CAM_HEIGHT / CAM_WIDTH)


def _quat_to_mat(q):
    x, y, z, w = q
    return ((1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)),
            (2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)),
            (2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)))


def body_axes_enu(elev_deg, az_deg):
    """장착 자세에서 body(FLU) X/Y/Z 축의 월드(ENU) 단위벡터 (열벡터 3개)."""
    r = _quat_to_mat(drone_orientation(elev_deg, az_deg))
    return tuple(tuple(r[i][j] for i in range(3)) for j in range(3))


def expected_px4_euler_deg(elev_deg, az_deg):
    """장착 자세를 PX4 ATTITUDE 규약(NED/FRD, ZYX Euler)으로 바꾼 (roll, pitch, yaw) [deg].

    PX4 gz 브리지는 Gazebo ENU/FLU 를 NED/FRD 로 바꿔 쓴다. 앙각 90°·방위 0° 이면
    (0, 0, 90): 기수(body +X)가 월드 +X(=East) 를 본다. 앙각 E 로 기울이면 추력축이
    기수 방향으로 넘어가므로 pitch = -(90°-E).
    """
    r = _quat_to_mat(drone_orientation(elev_deg, az_deg))
    t_ne = ((0, 1, 0), (1, 0, 0), (0, 0, -1))   # ENU → NED
    t_bf = (1, -1, -1)                          # FRD → FLU (대각)
    m = [[sum(t_ne[i][k] * r[k][j] for k in range(3)) * t_bf[j] for j in range(3)]
         for i in range(3)]
    pitch = math.asin(max(-1.0, min(1.0, -m[2][0])))
    roll = math.atan2(m[2][1], m[2][2])
    yaw = math.atan2(m[1][0], m[0][0])
    return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)


def camera_position(elev_deg, az_deg):
    """장착 상태의 카메라 광학 중심 월드 좌표."""
    c = drone_position(elev_deg, az_deg)
    d = axis(elev_deg, az_deg)
    return tuple(c[i] + CAM_OFFSET_Z * d[i] for i in range(3))


def boresight_point(elev_deg, az_deg, range_m):
    """카메라 광축 위, 카메라로부터 range_m 떨어진 월드 점."""
    c = camera_position(elev_deg, az_deg)
    d = axis(elev_deg, az_deg)
    return tuple(c[i] + range_m * d[i] for i in range(3))


def project_target(elev_deg, az_deg, target_xyz):
    """장착 자세에서 표적이 카메라 화면 어디에 보이는지 예측한다.

    반환 dict:
      off_axis_deg : 광축과 시선의 사잇각
      x_center/y_center : 예측 bbox 중심 (DetectionArray 와 같은 0~1 정규화, 화면 밖이면 범위 밖)
      in_fov : 화면 안(앞쪽이고 가로·세로 모두 반화각 이내)
      range_m : 카메라~표적 거리
    """
    cam = camera_position(elev_deg, az_deg)
    rel = tuple(target_xyz[i] - cam[i] for i in range(3))
    rng = math.sqrt(sum(v * v for v in rel))
    if rng < 1e-9:
        return {"off_axis_deg": 0.0, "x_center": 0.5, "y_center": 0.5,
                "in_fov": False, "range_m": 0.0}
    bx, by, bz = body_axes_enu(elev_deg, az_deg)
    fwd = sum(rel[i] * bz[i] for i in range(3))
    right = -sum(rel[i] * by[i] for i in range(3))
    down = sum(rel[i] * bx[i] for i in range(3))
    off = math.degrees(math.acos(max(-1.0, min(1.0, fwd / rng))))
    th = math.tan(CAM_HFOV_RAD / 2.0)
    tv = math.tan(camera_vfov_rad() / 2.0)
    if fwd <= 1e-9:
        return {"off_axis_deg": off, "x_center": float("nan"), "y_center": float("nan"),
                "in_fov": False, "range_m": rng}
    u, v = right / fwd, down / fwd
    return {"off_axis_deg": off,
            "x_center": 0.5 + u / (2.0 * th),
            "y_center": 0.5 + v / (2.0 * tv),
            "in_fov": abs(u) <= th and abs(v) <= tv,
            "range_m": rng}
