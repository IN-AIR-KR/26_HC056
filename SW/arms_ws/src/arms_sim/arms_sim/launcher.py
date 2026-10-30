#!/usr/bin/env python3
"""
launcher.py — 견착 발사기 SITL 노드  [arms_sim]

하는 일
  1) 앙각/방위 조절: ROS 파라미터 elevation_deg / azimuth_deg (패널 슬라이더가 씀).
     각도 변경 때 PID 없는 kinematic 발사관을 set_pose 한다. DetachableJoint에 붙은
     드론은 발사관을 즉시 따라가며, 물리 관절·PID는 사용하지 않는다.
  2) 발사(분리): 미션이 TRACK 으로 들어오는 순간 조인트를 끊는다.
       - 자동: 미션 상태가 TRACK 으로 '들어오는 순간'
       - 수동: ARM + 수동모드 + 스로틀을 호버(스틱 중앙) 이상으로 잠깐 유지
     분리 후엔 PX4 ACRO 가 '회전명령 0 = 현재 자세 유지' 라서 발사각 그대로 튀어나가고
     IBVS 가 표적 쪽으로 보정한다.
  3) 재장전(서비스 sim/launcher/reload, Trigger): 드론을 현재 각도의 발사관 위치·자세로
     되돌리고 다시 물린 뒤, '실제로 물렸는지'까지 확인하고 성공/실패를 돌려준다.
     실험 RESET(sitl_reset_manager)이 PX4 를 내린 사이에 부른다.
     패널 [재장전] 버튼은 sim/launcher/reload_manual(토픽)로 오며 DISARM 일 때만 받는다.
  4) 표시: sim/launcher/status (String) + rviz 마커(sim/viz/markers, ns=launcher).

설계 원칙 (예전 시도에서 실패한 것들)
  - 발사기 모델에는 collision 이 하나도 없다 → 물리적으로 드론 비행을 절대 방해 못 함.
  - 드론을 붙잡는 건 DetachableJoint(고정 조인트) 하나뿐 → 기울여도 안 떨어짐.
  - 발사관은 kinematic 모델이며, DetachableJoint의 부모다. 발사관 pose를 바꿔
    기체를 이동시키되 관절 PID·토크·지속 각속도를 사용하지 않는다.
  - 드론이 실제로 물리기 전에는 스폰 자세(수직)를 유지한다.

재장전 순서가 중요한 이유 (gz-sim 8 소스 확인)
  set_pose 서비스(비차단)는 명령을 '큐에 넣고' 바로 true 를 돌려준다. 같은 스텝에 attach 가
  들어오면 Physics 가 조인트를 먼저 만들고(순간이동 전 위치로 용접) 나중에 위치 명령을
  처리하는데, 그땐 이미 용접돼서 순간이동이 조용히 무시된다 → 드론이 추락한 자리에 용접됨.
  그래서: (분리 확인) → set_pose/blocking → 드론이 실제로 목표 위치에 온 걸 포즈 토픽으로 확인
  → attach → 플러그인이 "attached" 라고 답하는 걸 확인 → 위치 재확인.

Gazebo 쪽 이름 (assets/worlds/arms_sitl.sdf 의 arms_launcher 와 일치해야 함)
  /arms_sim/launcher/detach      gz.msgs.Empty    분리
  /arms_sim/launcher/attach      gz.msgs.Empty    재장착
  /arms_sim/launcher/state       gz.msgs.StringMsg "attached"/"detached" (플러그인 출력)
"""

import math
import os
import threading
import time

# referee.py / sitl_viz.py 와 같은 이유: gz.msgs10 의 pb2 가 protobuf 7.x 기본(upb)
# 구현에서 import 가 터진다. google.protobuf 첫 import 전에 순수 파이썬으로 내린다.
os.environ.setdefault("PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION", "python")

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from geometry_msgs.msg import Point
from std_msgs.msg import Empty, String
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker, MarkerArray
from arms_msgs.msg import AttitudeTelemetry, MissionState, OperatorInput

from arms_sim import launcher_geometry as G

_GZ_IMPORT_ERR = ""
try:
    from gz.transport13 import Node as _GzNode
    from gz.msgs10.empty_pb2 import Empty as _GzEmpty
    from gz.msgs10.stringmsg_pb2 import StringMsg as _GzString
    from gz.msgs10.pose_pb2 import Pose as _GzPose
    from gz.msgs10.boolean_pb2 import Boolean as _GzBool
    from gz.msgs10.pose_v_pb2 import Pose_V as _GzPoseV
    _gz_node = _GzNode()
    _GZ_OK = True
except Exception as _e:
    _GZ_IMPORT_ERR = f"{type(_e).__name__}: {_e}"
    _gz_node = None
    _GZ_OK = False

WORLD = "arms_sitl"
DRONE = "arms_drone_0"   # 기본값. 실제 이름은 drone_model 파라미터(PX4 인스턴스 N → arms_drone_N)
T_DETACH = "/arms_sim/launcher/detach"
T_ATTACH = "/arms_sim/launcher/attach"
T_STATE = "/arms_sim/launcher/state"
T_POSES = f"/world/{WORLD}/dynamic_pose/info"
TUBE_MODEL = "arms_launcher_tube"

POSE_TOL_M = 0.03   # 재장전 위치 확인 허용오차 [m]


class Launcher(Node):
    def __init__(self):
        super().__init__("arms_launcher")
        self.declare_parameter("elevation_deg", G.ELEV_SPAWN_DEG)
        self.declare_parameter("azimuth_deg", G.AZ_SPAWN_DEG)
        self.declare_parameter("rate_hz", 50.0)
        # 수동 모드 발사 조건. 스로틀 축 -1(최저) … 0(스틱 중앙 = Stabilized 호버) … +1(최대).
        #   호버 미만에서 놓으면 2m 높이에서 그냥 떨어진다 → 호버 이상을 잠깐 유지해야 분리.
        self.declare_parameter("manual_release_throttle", 0.05)
        self.declare_parameter("manual_release_hold_sec", 0.2)
        # 플러그인 상태 토픽이 안 오는 경우의 폴백: 드론이 보인 뒤 이 시간이 지나면 장착 간주.
        self.declare_parameter("attach_fallback_sec", 1.5)
        # 비충돌 시험용: false 면 TRACK 진입·수동 스로틀에도 분리하지 않는다(드론은 발사관에 남음).
        self.declare_parameter("release_enabled", True)
        self.declare_parameter("drone_model", DRONE)
        # 초기 자세 정렬: set_pose(순간이동)로 기울이면 자이로가 회전을 못 봐서 PX4 EKF 자세가
        #   수직 쪽에 남는다(50° 실측: 기대 0/-40/90 vs PX4 -6.4/-32.3/93.6). 각도 적용·재장전
        #   뒤 DISARM 상태면 sitl_bridge 에 EKF 재정렬을 요청하고, 정렬 전에는 분리하지 않는다.
        self.declare_parameter("realign_estimator", True)
        self.declare_parameter("release_requires_aligned_attitude", True)
        self.declare_parameter("attitude_align_tol_deg", 8.0)
        self._drone = str(self.get_parameter("drone_model").value)

        self._lock = threading.Lock()
        self._reload_lock = threading.Lock()
        # 런치에서 준 초기 각도(elevation_deg/azimuth_deg)를 쓴다. 스폰은 항상 수직이고,
        #   최초 장착이 확인되면 이 각도와 스폰 각도가 다를 때 _start_angle_worker 가 기울인다.
        self._el = min(max(float(self.get_parameter("elevation_deg").value),
                           G.ELEV_MIN_DEG), G.ELEV_MAX_DEG)
        self._az = min(max(float(self.get_parameter("azimuth_deg").value),
                           G.AZ_MIN_DEG), G.AZ_MAX_DEG)
        self._attached = False            # 플러그인이 확인해준(또는 폴백) 장착 여부
        self._rig_ready = False           # 최초 장착 완료(이후부터 사용자 각도 적용)
        self._plugin_state = None         # 플러그인이 알려준 마지막 "attached"/"detached"
        self._state_evt = threading.Event()
        self._drone_seen_t = None
        self._drone_pose = None           # (x, y, z)
        self._drone_pose_t = 0.0          # 마지막 포즈 수신 시각(monotonic)
        self._poses_subscribed = False
        self._prev_state = ""
        self._armed = False
        self._manual = False
        self._throttle = -1.0
        self._throttle_hi_since = None
        self._launched_at = None          # (elev, az) 발사 순간 각도
        self._angle_generation = 0
        self._angle_worker_running = False
        self._att = None                  # (roll, pitch, yaw) PX4 ATTITUDE [deg]
        self._att_t = 0.0                 # 수신 시각(monotonic)
        self._align_since = None          # 재정렬 요청 시각 (정렬 완료 로그용)

        self.add_on_set_parameters_callback(self._on_set_params)

        self._pub_status = self.create_publisher(String, "sim/launcher/status", 10)
        self._pub_markers = self.create_publisher(MarkerArray, "sim/viz/markers", 1)
        self.create_subscription(MissionState, "mission/state", self._on_mission, 10)
        self.create_subscription(OperatorInput, "operator/input", self._on_operator, 10)
        self.create_subscription(Empty, "sim/launcher/reload_manual", self._on_reload_manual, 10)
        self.create_subscription(AttitudeTelemetry, "vehicle/attitude", self._on_attitude, 10)
        self._reinit_cli = self.create_client(Trigger, "sitl/reinit_estimator")

        if not _GZ_OK:
            self.get_logger().error(
                f"gz 파이썬 바인딩 없음 → 발사기 비활성(재장전 서비스도 안 만든다). "
                f"원인: {_GZ_IMPORT_ERR}")
        else:
            self._gz_detach = _gz_node.advertise(T_DETACH, _GzEmpty)
            self._gz_attach = _gz_node.advertise(T_ATTACH, _GzEmpty)
            _gz_node.subscribe(_GzString, T_STATE, self._on_plugin_state)
            self._poses_on()
            # 서비스는 gz 가 될 때만 만든다 → RESET 쪽이 '서비스 없음'으로 정확히 실패한다.
            self.create_service(Trigger, "sim/launcher/reload", self._srv_reload)

        if not self.get_parameter("release_enabled").value:
            self.get_logger().warn("release_enabled=false — 비충돌 시험: 드론을 분리(발사)하지 않는다")
        rate = float(self.get_parameter("rate_hz").value)
        self._dt = 1.0 / max(rate, 1.0)
        self.create_timer(self._dt, self._tick)
        self.create_timer(0.25, self._publish_status)
        self.get_logger().info(
            f"launcher ready: 피벗 높이 {G.PIVOT_Z:.2f}m, 피벗→무게중심 {G.CG_DIST:.3f}m. "
            f"초기 각도 앙각 {self._el:.1f}° 방위 {self._az:.1f}° (장착 확인 후 적용). "
            f"각도 변경은 pose 재배치로 즉시 적용한다.")

    # ── 드론 포즈 구독 (필요할 때만 켠다: 순수 파이썬 protobuf 파싱이 무겁다) ──
    def _poses_on(self):
        if self._poses_subscribed:
            return
        try:
            _gz_node.subscribe(_GzPoseV, T_POSES, self._on_poses)
            self._poses_subscribed = True
        except Exception as e:
            self.get_logger().error(f"드론 포즈 구독 실패: {e}")

    def _poses_off(self):
        if not self._poses_subscribed:
            return
        try:
            _gz_node.unsubscribe(T_POSES)
            self._poses_subscribed = False
        except Exception:
            pass   # 바인딩에 unsubscribe 가 없으면 그냥 켜둔다

    # ── 파라미터 ──────────────────────────────────────────────────────────
    def _on_set_params(self, params):
        target_el, target_az = self._el, self._az
        angle_changed = False
        for p in params:
            if p.name == "elevation_deg":
                v = float(p.value)
                if not (G.ELEV_MIN_DEG <= v <= G.ELEV_MAX_DEG):
                    return SetParametersResult(
                        successful=False,
                        reason=f"앙각은 {G.ELEV_MIN_DEG:.0f}~{G.ELEV_MAX_DEG:.0f}° (받은 값 {v})")
                target_el, angle_changed = v, True
            elif p.name == "azimuth_deg":
                v = float(p.value)
                if not (G.AZ_MIN_DEG <= v <= G.AZ_MAX_DEG):
                    return SetParametersResult(
                        successful=False,
                        reason=f"방위는 {G.AZ_MIN_DEG:.0f}~{G.AZ_MAX_DEG:.0f}° (받은 값 {v})")
                target_az, angle_changed = v, True
        if angle_changed:
            with self._lock:
                if self._armed:
                    return SetParametersResult(
                        successful=False,
                        reason="ARM 상태에서는 발사기 각도를 바꿀 수 없다")
                self._el, self._az = target_el, target_az
                self._angle_generation += 1
            self._start_angle_worker()
        return SetParametersResult(successful=True)

    # ── 입력 ──────────────────────────────────────────────────────────────
    def _on_operator(self, msg):
        self._throttle = float(msg.throttle)

    def _on_mission(self, msg):
        state = msg.state or ""
        self._armed = bool(msg.armed)
        self._manual = bool(msg.manual_mode)
        prev, self._prev_state = self._prev_state, state
        # TRACK 으로 '들어오는 순간'만 본다(보통 LOCK→TRACK). 이미 TRACK 인 상태로
        # 재장전되는 경우(RESET 직후 상태가 아직 안 바뀜)에 다시 쏘지 않게 전이로만 판단.
        if self._attached and state == "TRACK" and prev != "TRACK":
            self._release(f"자동: {prev or '?'}→TRACK (실기체 걸쇠 OPEN 과 같은 순간)")

    def _on_plugin_state(self, msg):
        s = msg.data.strip().lower()
        apply_angles = False
        with self._lock:
            self._plugin_state = s
            if s == "attached":
                self._attached = True
                if not self._rig_ready:
                    self._rig_ready = True
                    self.get_logger().info("드론 장착 확인(플러그인) → 패널 각도 적용 가능")
                    apply_angles = (self._el, self._az) != (
                        G.ELEV_SPAWN_DEG, G.AZ_SPAWN_DEG)
            elif s == "detached":
                self._attached = False
        self._state_evt.set()
        if apply_angles:
            self._start_angle_worker()

    def _on_poses(self, msg):
        for p in msg.pose:
            if p.name == self._drone:
                with self._lock:
                    if self._drone_seen_t is None:
                        self._drone_seen_t = time.monotonic()
                    self._drone_pose = (p.position.x, p.position.y, p.position.z)
                    self._drone_pose_t = time.monotonic()
                return

    # ── 주기 처리 ─────────────────────────────────────────────────────────
    def _tick(self):
        if not _GZ_OK:
            return
        apply_angles = False
        with self._lock:
            if (not self._rig_ready and self._plugin_state is None
                    and self._drone_seen_t is not None
                    and time.monotonic() - self._drone_seen_t
                    > float(self.get_parameter("attach_fallback_sec").value)):
                self._rig_ready = True
                self._attached = True
                self.get_logger().warn(
                    "플러그인 장착 상태 토픽이 안 옴 → 드론 출현 후 시간 경과로 장착 간주")
                apply_angles = (self._el, self._az) != (
                    G.ELEV_SPAWN_DEG, G.AZ_SPAWN_DEG)
            ready = self._rig_ready
            attached = self._attached
        if ready and not self._reload_lock.locked():
            self._poses_off()   # 장착 끝 — 재장전 때만 다시 켠다
        if apply_angles:
            self._start_angle_worker()

        # 수동 발사: ARM + 수동모드 + 스로틀 호버 이상을 hold_sec 동안 유지
        hi = (self._armed and self._manual
              and self._throttle > float(self.get_parameter("manual_release_throttle").value))
        now = time.monotonic()
        if not hi:
            self._throttle_hi_since = None
        elif self._throttle_hi_since is None:
            self._throttle_hi_since = now
        elif (attached and now - self._throttle_hi_since
              >= float(self.get_parameter("manual_release_hold_sec").value)):
            self._release("수동: ARM + 스로틀 호버 이상 유지")

        self._publish_markers()

    def _start_angle_worker(self):
        """패널 드래그 중 들어온 여러 각도 요청은 최신 값만 재장전한다."""
        with self._lock:
            if self._angle_worker_running:
                return
            self._angle_worker_running = True
        threading.Thread(target=self._apply_angle_updates, daemon=True).start()

    def _apply_angle_updates(self):
        try:
            first_update = True
            while rclpy.ok():
                # 처음만 짧게 기다려 elevation/azimuth가 연달아 오는 것을 합친다.
                # 이후 드래그 중에는 최신 pose를 곧바로 적용해 발사관과 드론이 함께 보인다.
                if first_update:
                    time.sleep(0.03)
                with self._lock:
                    generation = self._angle_generation
                    ready, armed = self._rig_ready, self._armed
                if not ready:
                    return
                if armed:
                    return
                ok, msg = self._do_reload()
                if ok:
                    self.get_logger().info(f"발사기 각도 적용: {msg}")
                else:
                    self.get_logger().error(f"발사기 각도 적용: {msg}")
                with self._lock:
                    done = generation == self._angle_generation
                if done:
                    if ok:
                        self._request_realign(f"각도 적용 {self._el:.0f}°/{self._az:.0f}°")
                    return
                first_update = False
        finally:
            with self._lock:
                self._angle_worker_running = False

    # ── 발사 ──────────────────────────────────────────────────────────────
    def _release(self, why):
        if not _GZ_OK:
            return
        if not self.get_parameter("release_enabled").value:
            self.get_logger().warn(
                f"발사 차단 — {why} (release_enabled=false, 비충돌 시험 구성)",
                throttle_duration_sec=2.0)
            return
        err = self._attitude_error()
        tol = float(self.get_parameter("attitude_align_tol_deg").value)
        if self.get_parameter("release_requires_aligned_attitude").value:
            if err is None:
                self.get_logger().warn(
                    "PX4 자세 수신 없음 — 자세 정렬을 확인하지 못한 채 분리한다",
                    throttle_duration_sec=2.0)
            elif err > tol:
                self.get_logger().error(
                    f"발사 차단 — {why}: PX4 자세가 발사관 기하와 {err:.1f}° 다르다(허용 {tol:.0f}°). "
                    f"EKF 정렬 전 분리하면 자세 제어가 틀린 기준으로 돈다. 재장전/정렬 대기",
                    throttle_duration_sec=2.0)
                return
        self._gz_detach.publish(_GzEmpty())
        with self._lock:
            self._attached = False          # 플러그인이 "detached" 로 곧 확인해준다
            self._launched_at = (self._el, self._az)
        # 초기 자세 기록: 실제 PX4 자세와 비교할 기대값(launcher_geometry)도 같이 남긴다.
        #   광축(body +Z)은 튜브 축과 같으므로 카메라 앙각 = 발사 앙각.
        roll, pitch, yaw = G.expected_px4_euler_deg(self._el, self._az)
        self.get_logger().info(
            f"🚀 발사 — {why}. 앙각 {self._el:.1f}°, 방위 {self._az:.1f}° | "
            f"기대 PX4 자세 roll={roll:.1f} pitch={pitch:.1f} yaw={yaw:.1f} deg, "
            f"카메라 광축 앙각 {self._el:.1f}° (HFOV {math.degrees(G.CAM_HFOV_RAD):.1f}°, "
            f"VFOV {math.degrees(G.camera_vfov_rad()):.1f}°)")

    # ── 재장전 ────────────────────────────────────────────────────────────
    def _on_attitude(self, msg):
        self._att = (float(msg.roll_deg), float(msg.pitch_deg), float(msg.yaw_deg))
        self._att_t = time.monotonic()

    def _attitude_error(self):
        """PX4 자세와 현재 발사관 기하(기대 PX4 Euler)의 최대 축 오차[deg]. 자세 미수신이면 None."""
        if self._att is None or time.monotonic() - self._att_t > 1.0:
            return None
        exp = G.expected_px4_euler_deg(self._el, self._az)
        return max(abs(G.angle_difference_deg(m, e)) for m, e in zip(self._att, exp))

    def _request_realign(self, why):
        if not self.get_parameter("realign_estimator").value or self._armed:
            return
        if not self._reinit_cli.service_is_ready():
            self.get_logger().warn(f"EKF 재정렬 서비스 없음(sitl_bridge?) — {why} 후 정렬 생략")
            return
        err = self._attitude_error()
        self._align_since = time.monotonic()
        self.get_logger().info(
            f"EKF 재정렬 요청 ({why}) — 요청 전 자세 오차 "
            f"{'미수신' if err is None else f'{err:.1f}°'}")
        self._reinit_cli.call_async(Trigger.Request()).add_done_callback(
            lambda f: self.get_logger().info(
                f"EKF 재정렬 응답: {f.result().message if f.result() else f.exception()}"))

    def _srv_reload(self, _req, res):
        ok, msg = self._do_reload()
        res.success, res.message = ok, msg
        if ok:
            self._request_realign("재장전")
        return res

    def _on_reload_manual(self, _msg):
        """패널 [재장전] 버튼. 시동 걸린 기체를 튜브로 순간이동시키면 위험하니 DISARM 일 때만."""
        if self._armed:
            self.get_logger().warn("재장전 거부: ARM 상태다. 먼저 DISARM 해라 (또는 실험 RESET).")
            return
        ok, msg = self._do_reload()
        if ok:
            self.get_logger().info(f"재장전: {msg}")
            self._request_realign("수동 재장전")
        else:
            self.get_logger().error(f"재장전: {msg}")

    def _wait_plugin(self, want, timeout):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            with self._lock:
                if self._plugin_state == want:
                    return True
            self._state_evt.clear()
            self._state_evt.wait(0.05)
        with self._lock:
            return self._plugin_state == want

    def _pose_check(self, target, since, timeout):
        """since 이후 받은 포즈로 판정: "near" / "far" / "none"(그 사이 포즈가 안 옴)."""
        deadline = time.monotonic() + timeout
        seen = False
        while time.monotonic() < deadline:
            with self._lock:
                p, t = self._drone_pose, self._drone_pose_t
            if p is not None and t >= since:
                seen = True
                if math.dist(p, target) < POSE_TOL_M:
                    return "near"
            time.sleep(0.003)
        return "far" if seen else "none"

    def _pose_near(self, target, since, timeout):
        return self._pose_check(target, since, timeout) == "near"

    def _set_pose(self, name, target, quat):
        req = _GzPose()
        req.name = name
        req.position.x, req.position.y, req.position.z = target
        req.orientation.x, req.orientation.y, req.orientation.z, req.orientation.w = quat
        # blocking 버전: 실제로 적용된 뒤 돌려준다. 없으면(구버전) 비차단으로 폴백.
        #   호스트 검출(YOLO)이 CPU를 다 쓰면 Gazebo RTF가 0.3 근처까지 떨어지고, 이때
        #   blocking 응답이 1 s를 넘겨 재장전·RESET이 실패했다(PX4만 죽은 채 남음).
        #   한 스텝이 늦어도 기다릴 수 있게 timeout을 넉넉히 두고 한 번 더 시도한다.
        for _ in range(2):
            for srv in (f"/world/{WORLD}/set_pose/blocking", f"/world/{WORLD}/set_pose"):
                try:
                    ok, rep = _gz_node.request(srv, req, _GzPose, _GzBool, 3000)
                    if ok and getattr(rep, "data", True):
                        return True
                except Exception:
                    continue
        return False

    def _do_reload(self):
        """드론을 현재 각도의 튜브 위치로 되돌리고, 물렸음을 확인한다. (ok, 설명)"""
        if not _GZ_OK:
            return False, "gz 바인딩 없음"
        if not self._reload_lock.acquire(blocking=False):
            return False, "이미 재장전 중"
        try:
            self._poses_on()
            el, az = self._el, self._az
            target = G.drone_position(el, az)
            quat = G.drone_orientation(el, az)
            feedback = self._plugin_state is not None

            # 발사관은 DetachableJoint의 kinematic 부모다. 이 pose 변경으로 장전된
            # 드론도 함께 움직인다. 수직 상태에서 방위만 바뀌어도 호출이 필요하다.
            pivot = (G.PIVOT_X, G.PIVOT_Y, G.PIVOT_Z)
            if not self._set_pose(TUBE_MODEL, pivot, quat):
                return False, "발사관 visual set_pose 서비스 실패"

            # 0) 장전된 기체가 발사관 pose를 따라왔다면 재장전은 불필요하다.
            if self._attached and self._pose_near(target, time.monotonic(), 0.5):
                return True, f"이미 장착 상태 (앙각 {el:.0f}°, 방위 {az:.0f}°)"

            # 1) 항상 먼저 분리한다 (용접된 채로는 set_pose 가 조용히 무시된다).
            #    이미 분리 상태면 플러그인이 그냥 무시하므로 손해가 없다.
            was_attached = self._attached or self._plugin_state == "attached"
            self._gz_detach.publish(_GzEmpty())
            if was_attached and feedback:
                if not self._wait_plugin("detached", 2.0):
                    return False, "분리 확인 실패 (플러그인 응답 없음)"
            else:
                time.sleep(0.1)

            # 2) 발사 후 분리된 드론만 같은 발사 자세로 즉시 되돌린다.
            t0 = time.monotonic()
            if not self._set_pose(self._drone, target, quat):
                return False, "set_pose 서비스 실패"
            if not self._pose_near(target, t0, 3.0):
                return False, "set_pose 후 드론이 목표 위치에 안 옴"

            # 3) 물리기 → 플러그인 확인 → 위치 재확인
            self._gz_attach.publish(_GzEmpty())
            if feedback:
                if not self._wait_plugin("attached", 2.0):
                    return False, "재장착 확인 실패 (플러그인이 attached 라고 안 함)"
            else:
                time.sleep(0.3)
                with self._lock:
                    self._attached = True
            time.sleep(0.2)
            after = self._pose_check(target, time.monotonic(), 0.5)
            if after == "far":
                return False, "물린 뒤 위치가 목표와 다름 (엉뚱한 자리에 용접됐을 수 있음)"
            if after == "none":
                # 용접된 기체가 dynamic_pose 에 안 나오는 gz 버전일 수 있다. 플러그인 확인으로 갈음.
                self.get_logger().warn("재장착 후 드론 포즈가 안 와서 위치 재확인은 생략")
            with self._lock:
                self._launched_at = None
                self._rig_ready = True
            note = "" if feedback else " (플러그인 상태 토픽 없음 — 장착은 추정)"
            return True, f"재장전 완료 — 앙각 {el:.0f}°, 방위 {az:.0f}°{note}"
        finally:
            self._reload_lock.release()

    # ── 표시 ──────────────────────────────────────────────────────────────
    def _publish_status(self):
        with self._lock:
            attached, ready, launched = self._attached, self._rig_ready, self._launched_at
        if self._reload_lock.locked():
            text = "재장전 중…"
        elif not ready:
            text = "대기: 드론 장착 전"
        elif attached:
            text = f"장전됨 · 앙각 {self._el:.0f}° 방위 {self._az:.0f}°"
            err = self._attitude_error()
            tol = float(self.get_parameter("attitude_align_tol_deg").value)
            if err is None:
                text += " · PX4 자세 미수신"
            elif err <= tol:
                text += f" · 자세정렬 OK({err:.1f}°)"
                if self._align_since is not None:
                    self.get_logger().info(
                        f"EKF 자세 정렬 완료: 오차 {err:.1f}° "
                        f"({time.monotonic() - self._align_since:.1f}s 소요)")
                    self._align_since = None
            else:
                text += f" · 자세 불일치 {err:.1f}°"
                # 최초 기울임은 PX4 연결 전에 일어나 재정렬 요청이 실패할 수 있다.
                #   장착·DISARM 상태에서 불일치가 남아 있으면 20 s 간격으로 다시 요청한다.
                if (not self._armed and self._reload_lock.locked() is False
                        and (self._align_since is None
                             or time.monotonic() - self._align_since > 20.0)):
                    self._request_realign(f"장착 상태 자세 불일치 {err:.1f}°")
        elif launched:
            text = f"발사됨 · 앙각 {launched[0]:.0f}° 방위 {launched[1]:.0f}°"
        else:
            text = "분리됨"
        self._pub_status.publish(String(data=text))

    def _publish_markers(self):
        now = self.get_clock().now().to_msg()
        d = G.axis(self._el, self._az)
        p = (G.PIVOT_X, G.PIVOT_Y, G.PIVOT_Z)

        def pt(v):
            q = Point()
            q.x, q.y, q.z = v
            return q

        tube = Marker()
        tube.header.frame_id = "map"
        tube.header.stamp = now
        tube.ns = "launcher"
        tube.id = 0
        tube.type = Marker.LINE_LIST
        tube.action = Marker.ADD
        tube.pose.orientation.w = 1.0
        tube.scale.x = 0.10
        tube.color.r, tube.color.g, tube.color.b, tube.color.a = 0.33, 0.42, 0.18, 0.9
        back = tuple(p[i] - 0.45 * d[i] for i in range(3))
        mouth = tuple(p[i] + G.TAIL_FROM_PIVOT * d[i] for i in range(3))
        tube.points = [pt(back), pt(mouth)]

        post = Marker()
        post.header = tube.header
        post.ns = "launcher"
        post.id = 1
        post.type = Marker.LINE_LIST
        post.action = Marker.ADD
        post.pose.orientation.w = 1.0
        post.scale.x = 0.04
        post.color.r, post.color.g, post.color.b, post.color.a = 0.6, 0.6, 0.6, 0.8
        post.points = [pt((p[0], p[1], 0.0)), pt(p)]

        arr = MarkerArray()
        arr.markers = [tube, post]
        self._pub_markers.publish(arr)


def main(args=None):
    rclpy.init(args=args)
    node = Launcher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
