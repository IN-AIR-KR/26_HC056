#!/usr/bin/env python3
"""
sitl_viz.py — SITL 진실값(gz) → rviz 시각화  [arms_sim]

Gazebo 가 아는 '진짜' 위치를 그대로 rviz 로 내보낸다. 검출·추정을 거치지 않은
ground truth 라서 SITL 전용이다 — 그래서 arms_ui(실기체·시뮬 공용 카메라 오버레이)가
아니라 여기(arms_sim)에 있다.

  발행 : /tf                        map → 드론 / 표적
  발행 : /arms/viz/drone_path       nav_msgs/Path   드론 궤적
  발행 : /arms/viz/target_path      nav_msgs/Path   표적 궤적
  발행 : /arms/viz/markers          visualization_msgs/MarkerArray  기체 메쉬 + 표적

  구독 : gz /world/<world>/dynamic_pose/info   (in-process gz transport)
  구독 : /arms/reset                에피소드 경계 → 궤적 비움

rviz Fixed Frame 은 map 으로 둔다.

시간: 이 스택은 use_sim_time 을 아무 데서도 안 쓴다(전부 벽시계). 그래서 /clock
브리지도, use_sim_time 설정도 필요 없다 — rviz 도 기본값(벽시계) 그대로 두면 된다.

실행: ros2 run arms_sim sitl_viz
"""

import math
import os
import threading

# referee.py 와 같은 이유로 protobuf 순수 파이썬 구현을 강제한다. gz.msgs10 의 _pb2 는
# protoc 3 세대 산출물인데 시스템 protobuf 가 7.x 라, 기본(upb) 구현에서는 import 자체가
# "Descriptors cannot be created directly" 로 터진다.
#   ★ google.protobuf 가 처음 import 되기 전 = 파일 최상단이어야 한다.
os.environ.setdefault("PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION", "python")

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Point, PoseStamped, Quaternion, TransformStamped
from nav_msgs.msg import Path
from std_msgs.msg import Empty
from visualization_msgs.msg import Marker, MarkerArray
from tf2_ros import TransformBroadcaster

MODEL_BALLOON = "target_ball"
MODEL_DRONE = "target_uav"
# 이 z 아래로 내려간 모델은 '숨긴 것'으로 본다. referee 가 안 쓰는 표적을
# HIDE_Z(-100) 지하로 치우는데, 그걸 그리면 지하로 내려가는 궤적이 생긴다.
HIDDEN_Z = -50.0

# 기체 메쉬는 model.sdf 의 visual 과 같은 보정을 줘야 자세가 맞는다:
#   <pose>0 0 0 -1.5708 0 0.7854</pose>, <scale>0.001</scale>
#   (STL 이 mm·Y-up 이라 -90°(X)로 Z-up, yaw +45°로 X-쿼드 대각 정렬)
MESH_SCALE = 0.001
MESH_RPY = (-math.pi / 2.0, 0.0, math.pi / 4.0)

_GZ_IMPORT_ERR = ""
try:
    from gz.transport13 import Node as _GzNode
    from gz.msgs10.pose_v_pb2 import Pose_V as _GzPoseV
    _gz_node = _GzNode()
    _GZ_OK = True
except Exception as _e:
    _GZ_IMPORT_ERR = f"{type(_e).__name__}: {_e}"
    _gz_node = None
    _GzPoseV = None
    _GZ_OK = False


def _quat_mul(a, b):
    """쿼터니언 곱 a⊗b. (x, y, z, w) 순서."""
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def _quat_from_rpy(roll, pitch, yaw):
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


class SitlViz(Node):
    def require_parameter(self, name, parameter_type):
        self.declare_parameter(name, parameter_type)
        value = self.get_parameter(name).value
        if value is None:
            raise RuntimeError(
                f"Required parameter '{name}' is missing for "
                f"{self.get_fully_qualified_name()}")
        return value

    def __init__(self):
        super().__init__("arms_sitl_viz")

        self.require_parameter("world", Parameter.Type.STRING)
        self.require_parameter("frame_id", Parameter.Type.STRING)
        self.require_parameter("rate_hz", Parameter.Type.DOUBLE)
        self.require_parameter("path_max_points", Parameter.Type.INTEGER)
        self.require_parameter("path_min_dist", Parameter.Type.DOUBLE)
        # 조준 보조선: 기체 yaw축(= body +Z = 상방 카메라 보어사이트) 방향으로 뻗는 선.
        # 표적 고도(기본 30m)보다 넉넉히 길어야 '겨냥했는지'가 눈에 보인다. 0 이면 끔.
        self.require_parameter("aim_line_m", Parameter.Type.DOUBLE)

        self._frame = self.get_parameter("frame_id").value
        self._max_pts = int(self.get_parameter("path_max_points").value)
        self._min_dist = float(self.get_parameter("path_min_dist").value)
        self._aim_len = float(self.get_parameter("aim_line_m").value)

        # gz 콜백은 gz transport 스레드에서 온다. 거기서 바로 발행하지 않고 최신 포즈만
        # 담아두고, ROS 타이머가 꺼내 발행한다(=발행률도 여기서 제한된다).
        self._lock = threading.Lock()
        self._drone = None          # (name, (x,y,z), (qx,qy,qz,qw))
        self._target = None
        self._drone_path = []       # [Point]
        self._target_path = []
        self._target_name = None    # 궤적이 어느 표적의 것인지 (바뀌면 비운다)
        self._drone_logged = False

        self._mesh_uri = self._find_mesh()

        self._tf = TransformBroadcaster(self)
        self._pub_drone_path = self.create_publisher(Path, "/arms/viz/drone_path", 1)
        self._pub_target_path = self.create_publisher(Path, "/arms/viz/target_path", 1)
        self._pub_markers = self.create_publisher(MarkerArray, "/arms/viz/markers", 1)
        self.create_subscription(Empty, "/arms/reset", self._on_reset, 10)

        world = self.get_parameter("world").value
        if _GZ_OK:
            topic = f"/world/{world}/dynamic_pose/info"
            try:
                _gz_node.subscribe(_GzPoseV, topic, self._on_poses)
                self.get_logger().info(f"gz 포즈 구독: {topic}")
            except Exception as e:
                self.get_logger().error(f"gz 포즈 구독 실패: {e} — 시각화 비활성")
        else:
            self.get_logger().error(
                f"gz 바인딩 없음 — 시각화 비활성. 원인: {_GZ_IMPORT_ERR or '알 수 없음'}")

        rate = float(self.get_parameter("rate_hz").value)
        self.create_timer(1.0 / max(rate, 1.0), self._tick)
        self.get_logger().info(
            f"sitl_viz ready. frame={self._frame} rate={rate:.0f}Hz "
            f"path<={self._max_pts}점 (>{self._min_dist}m 마다)")

    # ------------------------------------------------------------------
    def _find_mesh(self):
        """기체 메쉬의 file:// URI. 못 찾으면 None → 마커는 단순 도형으로 대체."""
        try:
            share = get_package_share_directory("arms_sim")
        except Exception:
            return None
        p = os.path.join(share, "assets", "models", "arms_drone",
                         "meshes", "arms_model_v2.stl")
        if os.path.isfile(p):
            return f"file://{p}"
        self.get_logger().warn(f"기체 메쉬 없음({p}) → 박스로 대체")
        return None

    def _on_reset(self, _msg):
        """에피소드 경계. 궤적을 비우지 않으면 이전 판의 선이 그대로 이어진다."""
        with self._lock:
            self._drone_path.clear()
            self._target_path.clear()
        self.get_logger().info("실험 RESET: 궤적 비움")

    def _on_poses(self, msg):
        """gz dynamic pose 콜백 — 최신 포즈만 담아둔다 (발행은 타이머가)."""
        drone = target = None
        for p in msg.pose:
            n = p.name
            pos = (p.position.x, p.position.y, p.position.z)
            ori = (p.orientation.x, p.orientation.y,
                   p.orientation.z, p.orientation.w)
            if n in (MODEL_BALLOON, MODEL_DRONE):
                # 지하에 치워둔 표적은 '지금 쓰는 표적'이 아니다.
                if pos[2] > HIDDEN_Z:
                    target = (n, pos, ori)
            elif "drone" in n or "x500" in n:
                # referee 와 같은 판별 규칙 — 표적 이름엔 drone/x500 이 안 들어간다.
                drone = (n, pos, ori)
        with self._lock:
            if drone:
                self._drone = drone
            if target:
                self._target = target

    # ------------------------------------------------------------------
    def _tick(self):
        now = self.get_clock().now().to_msg()
        with self._lock:
            drone, target = self._drone, self._target
            if target and target[0] != self._target_name:
                # 표적 종류가 바뀌었다(풍선↔드론). 다른 물체이므로 궤적을 잇지 않는다.
                self._target_name = target[0]
                self._target_path.clear()
            if drone:
                self._append(self._drone_path, drone[1])
            if target:
                self._append(self._target_path, target[1])
            drone_pts = list(self._drone_path)
            target_pts = list(self._target_path)

        if drone is None and target is None:
            return      # gz 가 아직 아무것도 안 보냈다 (월드 기동 전)

        if drone and not self._drone_logged:
            self._drone_logged = True
            self.get_logger().info(f"드론 추적 시작: '{drone[0]}'")

        markers = MarkerArray()
        if drone:
            self._send_tf(now, drone)
            markers.markers.append(self._drone_marker(now, drone))
            if self._aim_len > 0.0:
                markers.markers.append(self._aim_marker(now, drone))
        if target:
            self._send_tf(now, target)
            markers.markers.append(self._target_marker(now, target))
        self._pub_markers.publish(markers)
        self._pub_drone_path.publish(self._path_msg(now, drone_pts))
        self._pub_target_path.publish(self._path_msg(now, target_pts))

    def _append(self, buf, pos):
        """이동했을 때만 점을 쌓는다. 정지 중에 같은 점을 쌓으면 버퍼만 갉아먹는다."""
        if buf:
            last = buf[-1]
            if math.dist((last.x, last.y, last.z), pos) < self._min_dist:
                return
        p = Point()
        p.x, p.y, p.z = pos
        buf.append(p)
        if len(buf) > self._max_pts:
            del buf[:len(buf) - self._max_pts]

    def _path_msg(self, stamp, pts):
        msg = Path()
        msg.header.stamp = stamp
        msg.header.frame_id = self._frame
        for pt in pts:
            ps = PoseStamped()
            ps.header = msg.header
            ps.pose.position = pt
            ps.pose.orientation.w = 1.0
            msg.poses.append(ps)
        return msg

    def _send_tf(self, stamp, entity):
        name, pos, ori = entity
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = self._frame
        t.child_frame_id = name
        t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = pos
        (t.transform.rotation.x, t.transform.rotation.y,
         t.transform.rotation.z, t.transform.rotation.w) = ori
        self._tf.sendTransform(t)

    def _base_marker(self, stamp, ns, mid):
        m = Marker()
        m.header.stamp = stamp
        m.header.frame_id = self._frame
        m.ns = ns
        m.id = mid
        m.action = Marker.ADD
        m.pose.orientation.w = 1.0
        return m

    def _drone_marker(self, stamp, entity):
        _, pos, ori = entity
        m = self._base_marker(stamp, "drone", 0)
        m.pose.position.x, m.pose.position.y, m.pose.position.z = pos
        if self._mesh_uri:
            m.type = Marker.MESH_RESOURCE
            m.mesh_resource = self._mesh_uri
            # SDF visual 과 같은 회전 보정을 적용해야 자세가 실제와 맞는다.
            q = _quat_mul(ori, _quat_from_rpy(*MESH_RPY))
            m.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
            m.scale.x = m.scale.y = m.scale.z = MESH_SCALE
            m.color.r = m.color.g = m.color.b = 0.25
        else:
            m.type = Marker.CUBE
            m.pose.orientation = Quaternion(x=ori[0], y=ori[1], z=ori[2], w=ori[3])
            m.scale.x = m.scale.y = 0.35
            m.scale.z = 0.12
            m.color.r = m.color.g = m.color.b = 0.3
        m.color.a = 1.0
        return m

    def _aim_marker(self, stamp, entity):
        """조준 보조선 — 기체 yaw축(body +Z) 방향으로 길게 뻗는 선.

        이 축이 곧 상방 카메라가 보는 방향이다(camera_link 가 base_link 기준
        pitch -90° 라 광축이 body +Z 를 향한다). 그래서 이 선이 표적을 꿰뚫고
        있으면 겨냥이 맞은 것이고, 빗나가 있으면 그만큼 조준 오차다.

        마커 pose 에 기체 자세를 그대로 넣고 점은 body 좌표 (0,0,0)→(0,0,L) 로
        준다. 회전은 rviz 가 하므로 여기서 방향 벡터를 계산할 필요가 없다.
        """
        _, pos, ori = entity
        m = self._base_marker(stamp, "aim", 0)
        m.type = Marker.LINE_STRIP
        m.pose.position.x, m.pose.position.y, m.pose.position.z = pos
        m.pose.orientation = Quaternion(x=ori[0], y=ori[1], z=ori[2], w=ori[3])
        p0, p1 = Point(), Point()
        p1.z = self._aim_len
        m.points = [p0, p1]
        m.scale.x = 0.12          # LINE_STRIP 은 scale.x 가 선 두께
        m.color.r, m.color.g, m.color.b, m.color.a = 0.2, 1.0, 0.4, 0.8
        return m

    def _target_marker(self, stamp, entity):
        name, pos, ori = entity
        m = self._base_marker(stamp, "target", 0)
        m.pose.position.x, m.pose.position.y, m.pose.position.z = pos
        if name == MODEL_BALLOON:
            m.type = Marker.SPHERE
            m.scale.x = m.scale.y = m.scale.z = 0.5   # world sdf 의 radius 0.25 → 지름 0.5
            m.color.r, m.color.g, m.color.b = 0.9, 0.1, 0.1
        else:
            m.type = Marker.CUBE
            m.pose.orientation = Quaternion(x=ori[0], y=ori[1], z=ori[2], w=ori[3])
            m.scale.x = m.scale.y = 0.5
            m.scale.z = 0.2
            m.color.r, m.color.g, m.color.b = 0.9, 0.5, 0.1
        m.color.a = 1.0
        return m


def main(args=None):
    rclpy.init(args=args)
    node = SitlViz()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
