#!/usr/bin/env python3
"""
launcher.py — 견착 발사기 SITL 노드  [arms_sim]

하는 일
  1) 앙각/방위 조절: ROS 파라미터 elevation_deg / azimuth_deg (패널 슬라이더가 씀)
     → 일정 속도(slew_dps)로 부드럽게 따라가며 Gazebo 관절 명령으로 보낸다.
     드론은 튜브에 고정 조인트로 물려 있으므로 같이 돌아가고, 드론 카메라 화면
     (하늘·표적 위치)도 그대로 따라 움직인다. 중력이 있어도 떨어지지 않는다.
  2) 발사(분리): 실기체 발사 걸쇠 서보가 여는 순간과 같은 조건에서 조인트를 끊는다.
       - 자동: 미션 상태가 TRACK 으로 '들어오는 순간' (실기체 servo_lock 이 OPEN 하는 전이)
       - 수동: ARM + 수동모드 + 스로틀을 호버(스틱 중앙) 이상으로 잠깐 유지
     분리 후엔 PX4 ACRO 가 '회전명령 0 = 현재 자세 유지' 라서 발사각 그대로 튀어나가고
     IBVS 가 표적 쪽으로 보정한다.
  3) 재장전(서비스 sim/launcher/reload, Trigger): 드론을 현재 각도의 튜브 위치·자세로
     되돌리고 다시 물린 뒤, '실제로 물렸는지'까지 확인하고 성공/실패를 돌려준다.
     실험 RESET(sitl_reset_manager)이 PX4 를 내린 사이에 부른다.
     패널 [재장전] 버튼은 sim/launcher/reload_manual(토픽)로 오며 DISARM 일 때만 받는다.
  4) 표시: sim/launcher/status (String) + rviz 마커(sim/viz/markers, ns=launcher).

설계 원칙 (예전 시도에서 실패한 것들)
  - 발사기 모델에는 collision 이 하나도 없다 → 물리적으로 드론 비행을 절대 방해 못 함.
  - 드론을 붙잡는 건 DetachableJoint(고정 조인트) 하나뿐 → 기울여도 안 떨어짐.
  - 각도는 관절 '속도 명령' 방식 위치제어 → 드론 무게(1.05kg)에 밀리지 않음.
  - 드론이 실제로 물리기 전에는 스폰 자세(수직)를 유지한다. 물리기 전에 튜브를
    기울이면 드론이 엉뚱한 상대자세로 물리기 때문.

재장전 순서가 중요한 이유 (gz-sim 8 소스 확인)
  set_pose 서비스(비차단)는 명령을 '큐에 넣고' 바로 true 를 돌려준다. 같은 스텝에 attach 가
  들어오면 Physics 가 조인트를 먼저 만들고(순간이동 전 위치로 용접) 나중에 위치 명령을
  처리하는데, 그땐 이미 용접돼서 순간이동이 조용히 무시된다 → 드론이 추락한 자리에 용접됨.
  그래서: (분리 확인) → set_pose/blocking → 드론이 실제로 목표 위치에 온 걸 포즈 토픽으로 확인
  → attach → 플러그인이 "attached" 라고 답하는 걸 확인 → 위치 재확인.

Gazebo 쪽 이름 (assets/worlds/arms_sitl.sdf 의 arms_launcher 와 일치해야 함)
  /arms_sim/launcher/az_cmd      gz.msgs.Double   방위 관절 목표 [rad]
  /arms_sim/launcher/elev_cmd    gz.msgs.Double   앙각 관절 목표 [rad] (= 90°−E)
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
from arms_msgs.msg import MissionState, OperatorInput

from arms_sim import launcher_geometry as G

_GZ_IMPORT_ERR = ""
try:
    from gz.transport13 import Node as _GzNode
    from gz.msgs10.double_pb2 import Double as _GzDouble
    from gz.msgs10.empty_pb2 import Empty as _GzEmpty
    from gz.msgs10.stringmsg_pb2 import StringMsg as _GzString
    from gz.msgs10.pose_pb2 import Pose as _GzPose
    from gz.msgs10.boolean_pb2 import Boolean as _GzBool
    from gz.msgs10.pose_v_pb2 import Pose_V as _GzPoseV
    from gz.msgs10.model_pb2 import Model as _GzModel
    _gz_node = _GzNode()
    _GZ_OK = True
except Exception as _e:
    _GZ_IMPORT_ERR = f"{type(_e).__name__}: {_e}"
    _gz_node = None
    _GZ_OK = False

WORLD = "arms_sitl"
DRONE = "arms_drone_0"
T_AZ = "/arms_sim/launcher/az_cmd"
T_EL = "/arms_sim/launcher/elev_cmd"
T_DETACH = "/arms_sim/launcher/detach"
T_ATTACH = "/arms_sim/launcher/attach"
T_STATE = "/arms_sim/launcher/state"
T_JOINTS = "/arms_sim/launcher/joint_state"   # gz.msgs.Model (JointStatePublisher)
T_POSES = f"/world/{WORLD}/dynamic_pose/info"

POSE_TOL_M = 0.03   # 재장전 위치 확인 허용오차 [m]


def _step_toward(cur, target, max_step):
    d = target - cur
    if abs(d) <= max_step:
        return target
    return cur + math.copysign(max_step, d)


class Launcher(Node):
    def __init__(self):
        super().__init__("arms_launcher")
        self.declare_parameter("elevation_deg", G.ELEV_SPAWN_DEG)
        self.declare_parameter("azimuth_deg", G.AZ_SPAWN_DEG)
        self.declare_parameter("slew_dps", 60.0)          # 조준 회전 속도 [deg/s]
        self.declare_parameter("rate_hz", 50.0)
        # 수동 모드 발사 조건. 스로틀 축 -1(최저) … 0(스틱 중앙 = Stabilized 호버) … +1(최대).
        #   호버 미만에서 놓으면 2m 높이에서 그냥 떨어진다 → 호버 이상을 잠깐 유지해야 분리.
        self.declare_parameter("manual_release_throttle", 0.05)
        self.declare_parameter("manual_release_hold_sec", 0.2)
        # 플러그인 상태 토픽이 안 오는 경우의 폴백: 드론이 보인 뒤 이 시간이 지나면 장착 간주.
        self.declare_parameter("attach_fallback_sec", 1.5)

        self._lock = threading.Lock()
        self._reload_lock = threading.Lock()
        self._el = G.ELEV_SPAWN_DEG       # 현재 명령 중인(슬루 적용된) 각도
        self._az = G.AZ_SPAWN_DEG
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
        self._actual = None               # (elev, az) 관절에서 읽은 실제 각도 [deg]
        self._sag_since = None            # 목표와 실제가 크게 벌어진 시점 (처짐 경고용)
        self._sag_warned = False

        self.add_on_set_parameters_callback(self._on_set_params)

        self._pub_status = self.create_publisher(String, "sim/launcher/status", 10)
        self._pub_markers = self.create_publisher(MarkerArray, "sim/viz/markers", 1)
        self.create_subscription(MissionState, "mission/state", self._on_mission, 10)
        self.create_subscription(OperatorInput, "operator/input", self._on_operator, 10)
        self.create_subscription(Empty, "sim/launcher/reload_manual", self._on_reload_manual, 10)

        if not _GZ_OK:
            self.get_logger().error(
                f"gz 파이썬 바인딩 없음 → 발사기 비활성(재장전 서비스도 안 만든다). "
                f"원인: {_GZ_IMPORT_ERR}")
        else:
            self._gz_az = _gz_node.advertise(T_AZ, _GzDouble)
            self._gz_el = _gz_node.advertise(T_EL, _GzDouble)
            self._gz_detach = _gz_node.advertise(T_DETACH, _GzEmpty)
            self._gz_attach = _gz_node.advertise(T_ATTACH, _GzEmpty)
            _gz_node.subscribe(_GzString, T_STATE, self._on_plugin_state)
            _gz_node.subscribe(_GzModel, T_JOINTS, self._on_joints)
            self._poses_on()
            # 서비스는 gz 가 될 때만 만든다 → RESET 쪽이 '서비스 없음'으로 정확히 실패한다.
            self.create_service(Trigger, "sim/launcher/reload", self._srv_reload)

        rate = float(self.get_parameter("rate_hz").value)
        self._dt = 1.0 / max(rate, 1.0)
        self.create_timer(self._dt, self._tick)
        self.create_timer(0.25, self._publish_status)
        self.get_logger().info(
            f"launcher ready: 피벗 높이 {G.PIVOT_Z:.2f}m, 피벗→무게중심 {G.CG_DIST:.3f}m. "
            f"드론이 튜브에 물리면 패널 각도를 적용한다.")

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
        for p in params:
            if p.name == "elevation_deg":
                v = float(p.value)
                if not (G.ELEV_MIN_DEG <= v <= G.ELEV_MAX_DEG):
                    return SetParametersResult(
                        successful=False,
                        reason=f"앙각은 {G.ELEV_MIN_DEG:.0f}~{G.ELEV_MAX_DEG:.0f}° (받은 값 {v})")
            elif p.name == "azimuth_deg":
                v = float(p.value)
                if not (G.AZ_MIN_DEG <= v <= G.AZ_MAX_DEG):
                    return SetParametersResult(
                        successful=False,
                        reason=f"방위는 {G.AZ_MIN_DEG:.0f}~{G.AZ_MAX_DEG:.0f}° (받은 값 {v})")
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
        with self._lock:
            self._plugin_state = s
            if s == "attached":
                self._attached = True
                if not self._rig_ready:
                    self._rig_ready = True
                    self.get_logger().info("드론 장착 확인(플러그인) → 패널 각도 적용 시작")
            elif s == "detached":
                self._attached = False
        self._state_evt.set()

    def _on_joints(self, msg):
        q = {j.name: j.axis1.position for j in msg.joint}
        if "elev_joint" in q and "az_joint" in q:
            with self._lock:
                self._actual = (90.0 - math.degrees(q["elev_joint"]), math.degrees(q["az_joint"]))

    def _on_poses(self, msg):
        for p in msg.pose:
            if p.name == DRONE:
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
        with self._lock:
            if (not self._rig_ready and self._plugin_state is None
                    and self._drone_seen_t is not None
                    and time.monotonic() - self._drone_seen_t
                    > float(self.get_parameter("attach_fallback_sec").value)):
                self._rig_ready = True
                self._attached = True
                self.get_logger().warn(
                    "플러그인 장착 상태 토픽이 안 옴 → 드론 출현 후 시간 경과로 장착 간주")
            ready = self._rig_ready
            attached = self._attached
        if ready and not self._reload_lock.locked():
            self._poses_off()   # 장착 끝 — 재장전 때만 다시 켠다

        if ready:
            tgt_el = float(self.get_parameter("elevation_deg").value)
            tgt_az = float(self.get_parameter("azimuth_deg").value)
        else:
            tgt_el, tgt_az = G.ELEV_SPAWN_DEG, G.AZ_SPAWN_DEG   # 물리기 전엔 스폰 자세 유지

        step = float(self.get_parameter("slew_dps").value) * self._dt
        self._el = _step_toward(self._el, tgt_el, step)
        self._az = _step_toward(self._az, tgt_az, step)

        q_az, q_el = G.joint_targets(self._el, self._az)
        m = _GzDouble()
        m.data = q_az
        self._gz_az.publish(m)
        m = _GzDouble()
        m.data = q_el
        self._gz_el.publish(m)

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

        self._check_sag()
        self._publish_markers()

    def _check_sag(self):
        """명령 각도와 실제 관절 각도가 5° 넘게 1초 이상 벌어지면 한 번 경고 (처짐 진단)."""
        with self._lock:
            act = self._actual
        if act is None:
            return
        gap = max(abs(act[0] - self._el), abs(act[1] - self._az))
        now = time.monotonic()
        if gap < 5.0:
            self._sag_since = None
            self._sag_warned = False
            return
        if self._sag_since is None:
            self._sag_since = now
        elif now - self._sag_since > 1.0 and not self._sag_warned:
            self._sag_warned = True
            self.get_logger().warn(
                f"발사기 각도가 명령을 못 따라감: 명령 앙각 {self._el:.1f}° / 실제 {act[0]:.1f}°, "
                f"명령 방위 {self._az:.1f}° / 실제 {act[1]:.1f}° — 관절 PID(SDF) 확인")

    # ── 발사 ──────────────────────────────────────────────────────────────
    def _release(self, why):
        if not _GZ_OK:
            return
        self._gz_detach.publish(_GzEmpty())
        with self._lock:
            self._attached = False          # 플러그인이 "detached" 로 곧 확인해준다
            self._launched_at = (self._el, self._az)
        self.get_logger().info(
            f"🚀 발사 — {why}. 앙각 {self._el:.1f}°, 방위 {self._az:.1f}°")

    # ── 재장전 ────────────────────────────────────────────────────────────
    def _srv_reload(self, _req, res):
        ok, msg = self._do_reload()
        res.success, res.message = ok, msg
        return res

    def _on_reload_manual(self, _msg):
        """패널 [재장전] 버튼. 시동 걸린 기체를 튜브로 순간이동시키면 위험하니 DISARM 일 때만."""
        if self._armed:
            self.get_logger().warn("재장전 거부: ARM 상태다. 먼저 DISARM 해라 (또는 실험 RESET).")
            return
        ok, msg = self._do_reload()
        (self.get_logger().info if ok else self.get_logger().error)(f"재장전: {msg}")

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

    def _set_pose(self, target, quat):
        req = _GzPose()
        req.name = DRONE
        req.position.x, req.position.y, req.position.z = target
        req.orientation.x, req.orientation.y, req.orientation.z, req.orientation.w = quat
        # blocking 버전: 실제로 적용된 뒤 돌려준다. 없으면(구버전) 비차단으로 폴백.
        for srv in (f"/world/{WORLD}/set_pose/blocking", f"/world/{WORLD}/set_pose"):
            try:
                ok, rep = _gz_node.request(srv, req, _GzPose, _GzBool, 1000)
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

            # 0) 이미 제자리에 물려 있으면 끝
            if self._attached and self._pose_near(target, time.monotonic(), 0.5):
                return True, f"이미 장착 상태 (앙각 {el:.0f}°, 방위 {az:.0f}°)"

            # 1) 항상 먼저 분리한다 (용접된 채로는 set_pose 가 조용히 무시된다).
            #    이미 분리 상태면 플러그인이 그냥 무시하므로 손해가 없다.
            was_attached = self._attached or self._plugin_state == "attached"
            self._gz_detach.publish(_GzEmpty())
            if was_attached and feedback:
                if not self._wait_plugin("detached", 1.0):
                    return False, "분리 확인 실패 (플러그인 응답 없음)"
            else:
                time.sleep(0.1)

            # 2) 순간이동 → 실제로 그 자리에 왔는지 포즈로 확인
            t0 = time.monotonic()
            if not self._set_pose(target, quat):
                return False, "set_pose 서비스 실패"
            if not self._pose_near(target, t0, 1.5):
                return False, "set_pose 후 드론이 목표 위치에 안 옴"

            # 3) 물리기 → 플러그인 확인 → 위치 재확인
            self._gz_attach.publish(_GzEmpty())
            if feedback:
                if not self._wait_plugin("attached", 1.0):
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
            act = self._actual
        real = f" (실제 {act[0]:.1f}°)" if act is not None else ""
        if self._reload_lock.locked():
            text = "재장전 중…"
        elif not ready:
            text = "대기: 드론 장착 전"
        elif attached:
            text = f"장전됨 · 앙각 {self._el:.0f}°{real} 방위 {self._az:.0f}°"
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
