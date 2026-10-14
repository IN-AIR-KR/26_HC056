"""
launcher_geometry.py — 견착 발사기 SITL 의 기하 상수 (단일 출처)  [arms_sim]

이 값들을 쓰는 곳:
  - launcher.py            : 장전 위치·자세, 시각용 발사관 pose, rviz 마커
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
