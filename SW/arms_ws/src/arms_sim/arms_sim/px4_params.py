"""
px4_params.py — SITL PX4 런타임 파라미터 (단일 출처)  [arms_sim]

px4_sitl.launch.py 가 부팅 15 s 뒤 mavlink_shell 로 전부 넣고, sitl_bridge 는 그중
자기 추력 변환이 의존하는 값(ENFORCED_BY_BRIDGE)을 연결 때마다 PARAM_SET·확인한다
(셸이 실패해도 Stabilized 추력 곡선이 어긋나지 않게).
"""

# 자동 disarm / 페일세이프 끄기 (GCS/RC 없는 SITL 자율비행 셋업).
#   COM_DISARM_PRFLT 0  = arm 후 이륙 지연 자동 disarm 해제 (가만히 둬도 안 멈춤)
#   NAV_DLL_ACT      0  = GCS(데이터링크) 연결 끊김 페일세이프 해제
#   NAV_RCL_ACT      0  = RC 연결 끊김 페일세이프 해제
#   MC_ACRO_R_MAX / P_MAX = ACRO 모드 풀스틱 각속도[deg/s]. control의
#     flight command 정규화 기준(400 deg/s)과 일치해야 한다.
PX4_PARAMS = {
    "COM_DISARM_PRFLT": 0,
    "NAV_DLL_ACT": 0,
    "NAV_RCL_ACT": 0,
    # CH7은 control의 RTL 요청용이다. 기존 SITL 파라미터에 남은 kill mapping을
    # 부팅마다 지워야 RC override의 CH7 high가 `Kill switch engaged`가 되지 않는다.
    "RC_MAP_KILL_SW": 0,
    "MC_ACRO_R_MAX": 400,
    "MC_ACRO_P_MAX": 400,
    "MPC_MAN_TILT_MAX": 65,
    # Stabilized 추력 곡선: 스틱 중앙 = MPC_THR_HOVER '파라미터'(2). 기본(0)은 호버 추력
    #   추정값을 써서 시점마다 중앙값이 달라진다. sitl_bridge 가 자동 유도 추력을 이 곡선의
    #   역함수로 스틱에 싣는다 (arms_flight_io/sitl_bridge_node.py thrust_to_stabilized_stick).
    "MPC_THR_CURVE": 2,
    # 호버 추력: arms_drone(1.04 kg, motorConstant 1.4715e-5, ω 150~1000, THR_MDL_FAC 0.3)의
    #   물리 호버는 0.25(모델 계산). SITL 실측: 스틱 중앙 0.32 → +3.38 m/s², 0.24 → -0.37 m/s²
    #   (a = g(T/Th - 1) → Th ≈ 0.25). 예전 0.32 는 추력 곡선 왜곡(0.32→0.234)과 겹쳐 대충 맞았다.
    "MPC_THR_HOVER": 0.25,
    "MPC_TILTMAX_AIR": 65,
    # 견착 발사기: 앙각 30° 이하(수직 기준 60° 이상 기울기)에서도 ARM/발사돼야 한다.
    #   PX4 고장감지기(FD)는 기본 60° 넘게 기울면 'Attitude failure' 로 ARM 을 거부하고
    #   비행 중엔 비상정지까지 걸 수 있다. 0 = 해당 축 검사 끔.
    #   ★ 실기체 Pixhawk 에도 같은 결정이 필요하다 (낮은 앙각 발사를 할 거라면).
    "FD_FAIL_P": 0,
    "FD_FAIL_R": 0,
}


# sitl_bridge 가 직접 보장하는 파라미터: (이름, MAVLink 타입 'i'|'f')
ENFORCED_BY_BRIDGE = (("MPC_THR_CURVE", "i"), ("MPC_THR_HOVER", "f"))


def bridge_enforce_list():
    """sitl_bridge 'enforce_params' 파라미터 형식: ["NAME:i:2", "NAME:f:0.24"]."""
    return [f"{n}:{t}:{PX4_PARAMS[n]}" for n, t in ENFORCED_BY_BRIDGE]
