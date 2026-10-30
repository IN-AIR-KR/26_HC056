#!/usr/bin/env python3
"""
angled_launch_monitor.py — 각도 발사 SITL 진단 모니터  [arms_sim]

제어·검출 경로에 아무것도 발행하지 않는 '관찰 전용' 노드다. 발사 앙각(90° 기준 vs
40~60°)을 바꿨을 때 카메라·검출·상태머신·GT 가 어떻게 달라지는지 숫자로 남긴다.

집계 (report_period_sec 마다 로그 1줄 + sim/diag/summary JSON)
  - RTF                 : gz WorldStatistics. PX4 lockstep 이라 RTF 가 곧 '벽시계 기준 느려짐'.
  - 카메라              : 벽시계/시뮬시간 FPS, header.stamp 간격으로 본 누락 프레임,
                          수신 시점의 영상 나이(시뮬 시각 추정 - stamp) = 렌더→브리지→DDS 지연.
  - 검출                : 발행 주기, 표적 포함 주기, 검출기가 건너뛴 프레임 비율,
                          처리 지연(같은 stamp 의 영상 수신→검출 수신, 같은 벽시계),
                          표적 포함 검출 사이 최대 간격(control 의 소실/신선도 timeout 과 비교).
  - 상태머신            : 전이마다 즉시 로그. TRACK→IDLE(표적 소실 수동 인계)과 목록 밖 전이 표시.
  - 명령                : TRACK 자동 명령 중 추력 0(=명령 초기화) 구간, 수동 전환 순간.
  - GT                  : control.pn.use_ground_truth 값, GT 토픽 주기/나이, PnDebug.using_ground_truth.
  - 초기 자세           : 발사기 앙각/방위 → 기대 PX4 자세(launcher_geometry) vs 실제 vehicle/attitude,
                          예측 표적 화면 위치 vs 실제 검출 위치(장착 상태에서만).

선택 기능 setup.* (기본 꺼짐): 기동 후 발사기 각도·표적 위치·GT 를 파라미터 서비스로 한 번 맞춘다
  (패널이 하는 것과 같은 서비스 호출). 비충돌 정지 시험(angled_launch_diag.launch.py)이 쓴다.
"""

import json
import math
import os
import threading
import time

os.environ.setdefault("PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION", "python")

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rcl_interfaces.msg import Parameter as ParamMsg, ParameterEvent, ParameterType, ParameterValue
from rcl_interfaces.srv import GetParameters, SetParameters
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray, String
from arms_msgs.msg import (AttitudeTelemetry, DetectionArray, FlightCommand,
                           GuidanceGroundTruth, MissionState, PnDebug)

from arms_sim import launcher_geometry as G
from arms_sim.diag_stats import (RateWindow, SimClock, StampGapTracker, StampMatcher,
                                 TransitionLog, WindowStats)

_GZ_IMPORT_ERR = ""
try:
    from gz.transport13 import Node as _GzNode
    from gz.msgs10.world_stats_pb2 import WorldStatistics as _GzStats
    _gz_node = _GzNode()
    _GZ_OK = True
except Exception as _e:
    _GZ_IMPORT_ERR = f"{type(_e).__name__}: {_e}"
    _gz_node = None
    _GZ_OK = False

CTRL_NODE = "/arms/arms_control_node"
LAUNCHER_NODE = "/arms/arms_launcher"
REFEREE_NODE = "/arms/balloon_referee"


def _stamp_sec(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def _stamp_ns(stamp):
    return stamp.sec * 1_000_000_000 + stamp.nanosec


def _pv(value):
    """rcl ParameterValue → 파이썬 값."""
    t = value.type
    if t == ParameterType.PARAMETER_BOOL:
        return value.bool_value
    if t == ParameterType.PARAMETER_INTEGER:
        return value.integer_value
    if t == ParameterType.PARAMETER_DOUBLE:
        return value.double_value
    if t == ParameterType.PARAMETER_STRING:
        return value.string_value
    return None


def _to_pv(v):
    pv = ParameterValue()
    if isinstance(v, bool):
        pv.type, pv.bool_value = ParameterType.PARAMETER_BOOL, v
    elif isinstance(v, int):
        pv.type, pv.integer_value = ParameterType.PARAMETER_INTEGER, v
    else:
        pv.type, pv.double_value = ParameterType.PARAMETER_DOUBLE, float(v)
    return pv


def _fmt(s, key="mean", scale=1000.0, unit="ms"):
    return "n/a" if not s else f"{s[key] * scale:.0f}{unit}"


class AngledLaunchMonitor(Node):
    # 다른 노드에서 읽어 둘 파라미터 (초기 GetParameters + /parameter_events 갱신)
    WATCH = {
        CTRL_NODE: ["control.pn.use_ground_truth", "control.guidance_mode",
                    "control.pn.max_measurement_age_sec",
                    "mission.track_target_lost_timeout_sec", "mission.detection_timeout_sec"],
        LAUNCHER_NODE: ["elevation_deg", "azimuth_deg"],
        REFEREE_NODE: ["enabled", "pos_x", "pos_y", "alt"],
    }

    def __init__(self):
        super().__init__("arms_angled_launch_monitor")
        self.declare_parameter("world", "arms_sitl")
        self.declare_parameter("camera_topic", "camera/image_raw")
        self.declare_parameter("monitor_camera", True)
        self.declare_parameter("camera_rate_hz", 60.0)
        self.declare_parameter("report_period_sec", 5.0)
        self.declare_parameter("jsonl_path", "")
        # setup.* : 음수/빈값 = 건드리지 않음
        self.declare_parameter("setup.elevation_deg", -1.0)
        self.declare_parameter("setup.azimuth_deg", 0.0)
        self.declare_parameter("setup.target_range_m", 0.0)
        self.declare_parameter("setup.target_offset_deg", 0.0)
        self.declare_parameter("setup.use_ground_truth", -1)

        self._lock = threading.Lock()
        self._sim = SimClock()
        self._period = 1.0 / float(self.get_parameter("camera_rate_hz").value)
        # 주기 창 = 보고 주기. 검출이 ~1 Hz 이하로 떨어져도 0 Hz 로 보이지 않게 한다.
        win = max(2.0, float(self.get_parameter("report_period_sec").value))
        self._cam_rate = RateWindow(win)
        self._cam_rate_sim = RateWindow(win)
        self._cam_gap = StampGapTracker(self._period)
        self._cam_age = WindowStats()
        self._img_rx = StampMatcher()
        self._det_rate = RateWindow(win)
        self._det_tgt_rate = RateWindow(win)
        self._raw_rate = RateWindow(win)
        self._raw_tgt_rate = RateWindow(win)
        self._det_gap = StampGapTracker(self._period)
        self._det_proc = WindowStats()
        self._det_age = WindowStats()
        self._tgt_gap = WindowStats()
        # 검출 노드 자체 계측(perception/status): [5]frame_interval_ms [8]yolo_ms [10]total_ms
        self._det_compute = WindowStats()
        self._det_yolo = WindowStats()
        self._last_tgt_wall = None
        self._last_det = None          # (x, y, w, h, conf)
        self._att_rate = RateWindow(win)
        self._att = None               # (roll, pitch, yaw)
        self._gt_rate = RateWindow(win)
        self._gt_last_wall = None
        self._pn = {"n": 0, "gt": 0, "valid": 0, "active": 0}
        self._transitions = TransitionLog()
        self._state = None
        self._armed = False
        self._manual = False
        self._zero_thr_since = None
        self._zero_thr_events = 0
        self._zero_thr_total = 0.0
        self._last_cmd_manual = None
        self._params = {n: {} for n in self.WATCH}
        self._launch_record = None
        self._setup_done = {}

        jsonl = os.path.expanduser(str(self.get_parameter("jsonl_path").value or ""))
        self._jsonl = None
        if jsonl:
            os.makedirs(os.path.dirname(jsonl) or ".", exist_ok=True)
            self._jsonl = open(jsonl, "a", encoding="utf-8")
            self.get_logger().info(f"진단 기록 파일: {jsonl}")

        sensor_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                                history=HistoryPolicy.KEEP_LAST, depth=5)
        if self.get_parameter("monitor_camera").value:
            self.create_subscription(Image, str(self.get_parameter("camera_topic").value),
                                     self._on_image, sensor_qos)
        # 검출 토픽은 발행자가 RELIABLE depth10, control 은 BEST_EFFORT depth1 로 받는다.
        # 모니터는 BEST_EFFORT depth10 으로 받아 처리 지연 측정이 큐 대기로 부풀지 않게 한다.
        det_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                             history=HistoryPolicy.KEEP_LAST, depth=10)
        self.create_subscription(DetectionArray, "perception/detections", self._on_det, det_qos)
        self.create_subscription(DetectionArray, "perception/detections_raw", self._on_raw, det_qos)
        self.create_subscription(Float32MultiArray, "perception/status", self._on_det_status, 10)
        self.create_subscription(MissionState, "mission/state", self._on_state, 10)
        self.create_subscription(FlightCommand, "control/flight_command", self._on_cmd, 10)
        self.create_subscription(AttitudeTelemetry, "vehicle/attitude", self._on_att,
                                 qos_profile_sensor_data)
        self.create_subscription(GuidanceGroundTruth, "sim/guidance_ground_truth", self._on_gt,
                                 qos_profile_sensor_data)
        self.create_subscription(PnDebug, "control/pn_debug", self._on_pn, 10)
        self.create_subscription(String, "sim/launcher/status", self._on_launcher_status, 10)
        self.create_subscription(ParameterEvent, "/parameter_events", self._on_param_event, 50)
        self._pub_summary = self.create_publisher(String, "sim/diag/summary", 10)
        self._pub_event = self.create_publisher(String, "sim/diag/events", 50)

        self._get_cli = {n: self.create_client(GetParameters, f"{n}/get_parameters")
                         for n in self.WATCH}
        self._set_cli = {n: self.create_client(SetParameters, f"{n}/set_parameters")
                         for n in self.WATCH}
        self._launcher_status = ""

        if _GZ_OK:
            topic = f"/world/{self.get_parameter('world').value}/stats"
            try:
                _gz_node.subscribe(_GzStats, topic, self._on_stats)
            except Exception as e:
                self.get_logger().error(f"gz {topic} 구독 실패: {e} — RTF/영상 나이 측정 불가")
        else:
            self.get_logger().error(f"gz 바인딩 없음 — RTF/영상 나이 측정 불가 ({_GZ_IMPORT_ERR})")

        self.create_timer(float(self.get_parameter("report_period_sec").value), self._report)
        self.create_timer(1.0, self._poll_params_and_setup)
        self.get_logger().info(
            f"angled_launch_monitor ready — 카메라 {1.0 / self._period:.0f} Hz 기준, "
            f"HFOV {math.degrees(G.CAM_HFOV_RAD):.1f}° VFOV {math.degrees(G.camera_vfov_rad()):.1f}°")

    # ── 시간 ─────────────────────────────────────────────────────────────
    @staticmethod
    def _wall():
        return time.time()

    def _on_stats(self, msg):
        sim = msg.sim_time.sec + msg.sim_time.nsec * 1e-9
        with self._lock:
            self._sim.update(sim, self._wall(), msg.real_time_factor)

    def _sim_now(self, wall):
        with self._lock:
            return self._sim.now(wall)

    # ── 카메라 / 검출 ────────────────────────────────────────────────────
    def _on_image(self, msg):
        wall = self._wall()
        st = _stamp_sec(msg.header.stamp)
        self._cam_rate.add(wall)
        self._cam_rate_sim.add(st)
        self._cam_gap.add(st)
        self._img_rx.add(_stamp_ns(msg.header.stamp), wall)
        sim_now = self._sim_now(wall)
        if sim_now is not None:
            self._cam_age.add(sim_now - st)

    def _on_raw(self, msg):
        wall = self._wall()
        self._raw_rate.add(wall)
        if msg.detections:
            self._raw_tgt_rate.add(wall)

    def _on_det(self, msg):
        wall = self._wall()
        st = _stamp_sec(msg.header.stamp)
        self._det_rate.add(wall)
        self._det_gap.add(st)
        rx = self._img_rx.pop(_stamp_ns(msg.header.stamp))
        if rx is not None:
            self._det_proc.add(wall - rx)
        sim_now = self._sim_now(wall)
        if sim_now is not None:
            self._det_age.add(sim_now - st)
        if msg.detections:
            b = max(msg.detections, key=lambda d: d.confidence)
            self._last_det = (b.x_center, b.y_center, b.width, b.height, b.confidence)
            self._det_tgt_rate.add(wall)
            if self._last_tgt_wall is not None:
                self._tgt_gap.add(wall - self._last_tgt_wall)
            self._last_tgt_wall = wall

    def _on_det_status(self, msg):
        if len(msg.data) >= 11:
            self._det_compute.add(msg.data[10] / 1000.0)
            self._det_yolo.add(msg.data[8] / 1000.0)

    # ── 상태 / 명령 ──────────────────────────────────────────────────────
    def _on_state(self, msg):
        wall = self._wall()
        self._armed, self._manual = bool(msg.armed), bool(msg.manual_mode)
        prev, new = self._state, msg.state
        self._state = new
        if prev is None or prev == new:
            return
        since = (f"{wall - self._last_tgt_wall:.2f}s" if self._last_tgt_wall else "없음")
        info = f"armed={self._armed} manual_out={self._manual} 마지막 표적 검출 후 {since}"
        abnormal = self._transitions.add(wall, prev, new, info)
        tag = "⚠ 목록 밖 전이" if abnormal else ""
        if prev == "TRACK" and new == "IDLE":
            tag = "표적 소실 → 유도 중단·수동 인계 경로"
        text = f"[state] {prev} → {new} {tag} ({info})"
        # rclpy 는 한 호출 위치에서 심각도를 바꾸면 예외를 낸다 → 호출을 나눈다.
        if abnormal or tag:
            self.get_logger().warn(text)
        else:
            self.get_logger().info(text)
        self._event("state", prev=prev, new=new, abnormal=abnormal, info=info)
        if new == "TRACK":
            self._record_launch("LOCK→TRACK (발사 명령)")

    def _on_cmd(self, msg):
        wall = self._wall()
        if self._last_cmd_manual is not None and msg.manual_mode != self._last_cmd_manual:
            self.get_logger().warn(
                f"[cmd] {'자동 → 수동' if msg.manual_mode else '수동 → 자동'} 출력 전환 "
                f"(state={self._state}, arm={msg.arm}, rtl={msg.rtl_request})")
            self._event("cmd_mode", manual=bool(msg.manual_mode), arm=bool(msg.arm),
                        state=self._state)
        self._last_cmd_manual = msg.manual_mode
        auto_track = (self._state == "TRACK" and msg.arm and not msg.manual_mode)
        zero = auto_track and msg.throttle <= 1e-3
        if zero and self._zero_thr_since is None:
            self._zero_thr_since = wall
            self._zero_thr_events += 1
            self.get_logger().warn(
                f"[cmd] TRACK 자동 명령 중 추력 0 (stabilized={msg.stabilized_mode}) — "
                f"PN 입력이 신선도 게이트(control.pn.max_measurement_age_sec)를 못 넘었을 가능성")
        elif not zero and self._zero_thr_since is not None:
            dur = wall - self._zero_thr_since
            self._zero_thr_total += dur
            self._zero_thr_since = None
            self._event("zero_thrust", duration_sec=dur)

    def _on_att(self, msg):
        self._att_rate.add(self._wall())
        self._att = (msg.roll_deg, msg.pitch_deg, msg.yaw_deg)

    def _on_gt(self, msg):
        if msg.valid:
            wall = self._wall()
            self._gt_rate.add(wall)
            self._gt_last_wall = wall

    def _on_pn(self, msg):
        self._pn["n"] += 1
        self._pn["gt"] += int(msg.using_ground_truth)
        self._pn["valid"] += int(msg.valid)
        self._pn["active"] += int(msg.command_active)

    def _on_launcher_status(self, msg):
        prev, self._launcher_status = self._launcher_status, msg.data
        if msg.data.startswith("발사됨") and not prev.startswith("발사됨"):
            self._record_launch("발사기 분리 확인")
        elif msg.data.startswith("장전됨") and not prev.startswith("장전됨"):
            self.get_logger().info(f"[launcher] {msg.data}")

    # ── 초기 자세 / 화면 예측 ────────────────────────────────────────────
    def _rail_angles(self):
        p = self._params[LAUNCHER_NODE]
        if "elevation_deg" not in p:
            return None
        return float(p["elevation_deg"]), float(p.get("azimuth_deg", 0.0))

    def _target_park(self):
        p = self._params[REFEREE_NODE]
        if not all(k in p for k in ("pos_x", "pos_y", "alt")) or p.get("enabled"):
            return None   # 비행 중인 표적은 위치를 파라미터로 알 수 없다
        return float(p["pos_x"]), float(p["pos_y"]), float(p["alt"])

    def _attitude_check(self):
        ang = self._rail_angles()
        if ang is None:
            return None
        exp = G.expected_px4_euler_deg(*ang)
        out = {"elev_deg": ang[0], "az_deg": ang[1],
               "expected_rpy_deg": [round(v, 1) for v in exp],
               "boresight_elev_deg": ang[0]}
        if self._att is not None:
            err = [((m - e + 180.0) % 360.0) - 180.0 for m, e in zip(self._att, exp)]
            out["px4_rpy_deg"] = [round(v, 1) for v in self._att]
            out["err_rpy_deg"] = [round(v, 1) for v in err]
        tgt = self._target_park()
        if tgt is not None:
            pr = G.project_target(ang[0], ang[1], tgt)
            out["target_pred"] = {k: (round(v, 3) if isinstance(v, float) else v)
                                  for k, v in pr.items()}
            if self._last_det is not None and self._last_tgt_wall and \
                    self._wall() - self._last_tgt_wall < 1.0 and pr["in_fov"]:
                out["det_minus_pred"] = [round(self._last_det[0] - pr["x_center"], 3),
                                         round(self._last_det[1] - pr["y_center"], 3)]
        return out

    def _record_launch(self, why):
        rec = self._attitude_check() or {}
        rec["why"] = why
        rec["wall"] = self._wall()
        self._launch_record = rec
        self.get_logger().info(f"[launch] 초기 자세 기록 ({why}): {json.dumps(rec, ensure_ascii=False)}")
        self._event("launch_attitude", **rec)

    # ── 파라미터 ─────────────────────────────────────────────────────────
    def _on_param_event(self, ev):
        names = self.WATCH.get(ev.node)
        if not names:
            return
        for p in list(ev.new_parameters) + list(ev.changed_parameters):
            if p.name in names:
                self._set_param_cache(ev.node, p.name, _pv(p.value))

    def _set_param_cache(self, node, name, value):
        old = self._params[node].get(name)
        self._params[node][name] = value
        if old == value:
            return
        if name == "control.pn.use_ground_truth":
            self.get_logger().warn(
                f"[GT] control.pn.use_ground_truth = {value} → PN LOS-rate/접근속도 입력: "
                f"{'Gazebo GT (sim/guidance_ground_truth). 검출은 여전히 PN 갱신 트리거' if value else '영상 추정값'}")
            self._event("gt_flag", value=bool(value))
        elif node == LAUNCHER_NODE:
            self.get_logger().info(f"[launcher] {name} = {value}")

    def _poll_params_and_setup(self):
        for node, names in self.WATCH.items():
            cli = self._get_cli[node]
            if any(n not in self._params[node] for n in names) and cli.service_is_ready():
                req = GetParameters.Request(names=names)
                cli.call_async(req).add_done_callback(
                    lambda f, node=node, names=names: self._on_get(f, node, names))
        self._run_setup()

    def _on_get(self, fut, node, names):
        try:
            values = fut.result().values
        except Exception:
            return
        for n, v in zip(names, values):
            val = _pv(v)
            if val is not None:
                self._set_param_cache(node, n, val)

    def _set_remote(self, node, values, key):
        if self._setup_done.get(key) or not self._set_cli[node].service_is_ready():
            return
        self._setup_done[key] = "pending"
        req = SetParameters.Request(parameters=[ParamMsg(name=k, value=_to_pv(v))
                                                for k, v in values.items()])

        def done(fut):
            try:
                res = fut.result().results
                ok = all(r.successful for r in res)
                why = "; ".join(r.reason for r in res if not r.successful)
            except Exception as e:
                ok, why = False, str(e)
            self._setup_done[key] = ok
            if ok:
                self.get_logger().info(f"[setup] {node} {values} → 적용")
            else:
                self.get_logger().warn(f"[setup] {node} {values} → 실패: {why}")
        self._set_cli[node].call_async(req).add_done_callback(done)

    def _run_setup(self):
        el = float(self.get_parameter("setup.elevation_deg").value)
        az = float(self.get_parameter("setup.azimuth_deg").value)
        if el >= 0.0:
            self._set_remote(LAUNCHER_NODE, {"elevation_deg": el, "azimuth_deg": az}, "launcher")
        rng = float(self.get_parameter("setup.target_range_m").value)
        if rng > 0.0:
            e = el if el >= 0.0 else G.ELEV_SPAWN_DEG
            off = float(self.get_parameter("setup.target_offset_deg").value)
            # 화면 가로(body -Y) 방향으로 off 만큼 비낀 점: 화각 가장자리 시험용.
            bx, by, bz = G.body_axes_enu(e, az)
            d = [math.cos(math.radians(off)) * bz[i] - math.sin(math.radians(off)) * by[i]
                 for i in range(3)]
            c = G.camera_position(e, az)
            p = [c[i] + rng * d[i] for i in range(3)]
            self._set_remote(REFEREE_NODE, {"enabled": False, "pos_x": round(p[0], 2),
                                            "pos_y": round(p[1], 2), "alt": round(p[2], 2)},
                             "referee")
        gt = int(self.get_parameter("setup.use_ground_truth").value)
        if gt in (0, 1):
            self._set_remote(CTRL_NODE, {"control.pn.use_ground_truth": bool(gt)}, "gt")

    # ── 보고 ─────────────────────────────────────────────────────────────
    def _event(self, kind, **fields):
        rec = {"t": round(self._wall(), 3), "kind": kind, **fields}
        line = json.dumps(rec, ensure_ascii=False, default=str)
        self._pub_event.publish(String(data=line))
        if self._jsonl:
            self._jsonl.write(line + "\n")
            self._jsonl.flush()

    def _report(self):
        wall = self._wall()
        with self._lock:
            rtf = self._sim.rtf
            sim_ok = self._sim.valid()
        ctrl = self._params[CTRL_NODE]
        gt_on = ctrl.get("control.pn.use_ground_truth")
        lost_to = ctrl.get("mission.track_target_lost_timeout_sec")
        age_to = ctrl.get("control.pn.max_measurement_age_sec")
        cam_age = self._cam_age.summary()
        det_proc = self._det_proc.summary()
        det_age = self._det_age.summary()
        tgt_gap = self._tgt_gap.summary()
        compute = self._det_compute.summary()
        # 영상 수신→검출 수신(벽시계) 중 검출 노드가 실제로 계산한 시간을 빼면
        # 나머지는 큐 대기(이전 프레임 처리 중) + DDS 전송이다.
        queue_s = (det_proc["mean"] - compute["mean"]) if (det_proc and compute) else None
        cam_sim_hz = self._cam_rate_sim.rate(self._cam_gap.last or 0.0) if self._cam_gap.last else 0.0
        gt_age = (wall - self._gt_last_wall) if self._gt_last_wall else None
        zero_now = (wall - self._zero_thr_since) if self._zero_thr_since else 0.0
        summary = {
            "t": round(wall, 3), "kind": "summary", "state": self._state,
            "armed": self._armed, "manual_out": self._manual,
            "rtf": round(rtf, 3) if rtf else None, "sim_clock": sim_ok,
            "camera": {"hz_wall": round(self._cam_rate.rate(wall), 2),
                       "hz_sim": round(cam_sim_hz, 2),
                       "received": self._cam_gap.received, "missing": self._cam_gap.missing,
                       "loss_ratio": round(self._cam_gap.loss_ratio(), 4),
                       "max_gap_sim_s": round(self._cam_gap.max_gap, 3),
                       "age_at_rx_sim_s": cam_age},
            "detection": {"hz": round(self._det_rate.rate(wall), 2),
                          "target_hz": round(self._det_tgt_rate.rate(wall), 2),
                          "raw_hz": round(self._raw_rate.rate(wall), 2),
                          "raw_target_hz": round(self._raw_tgt_rate.rate(wall), 2),
                          "frames_skipped": self._det_gap.missing,
                          "skip_ratio": round(self._det_gap.loss_ratio(), 4),
                          "proc_latency_wall_s": det_proc, "age_at_rx_sim_s": det_age,
                          "compute_s": compute, "yolo_s": self._det_yolo.summary(),
                          "queue_transport_mean_s": (round(queue_s, 3)
                                                     if queue_s is not None else None),
                          "target_gap_wall_s": tgt_gap,
                          "since_last_target_s": (round(wall - self._last_tgt_wall, 2)
                                                  if self._last_tgt_wall else None),
                          "last": self._last_det},
            "attitude_hz": round(self._att_rate.rate(wall), 2),
            "gt": {"use_ground_truth": gt_on, "topic_hz": round(self._gt_rate.rate(wall), 2),
                   "age_s": round(gt_age, 2) if gt_age is not None else None,
                   "pn_debug": dict(self._pn)},
            "control_timeouts": {"track_target_lost_sec": lost_to, "pn_max_age_sec": age_to},
            "zero_thrust": {"events": self._zero_thr_events,
                            "total_sec": round(self._zero_thr_total + zero_now, 2)},
            "transitions": {f"{a}->{b}": n for (a, b), n in self._transitions.counts.items()},
            "rail": self._attitude_check(),
        }
        line = json.dumps(summary, ensure_ascii=False, default=str)
        self._pub_summary.publish(String(data=line))
        if self._jsonl:
            self._jsonl.write(line + "\n")
            self._jsonl.flush()

        gt_txt = ("GT=?" if gt_on is None else
                  f"GT={'ON' if gt_on else 'OFF'} gt_topic {self._gt_rate.rate(wall):.1f}Hz")
        self.get_logger().info(
            f"[diag] RTF {rtf if rtf is None else round(rtf, 2)} | "
            f"cam {self._cam_rate.rate(wall):.1f}Hz(wall) {cam_sim_hz:.1f}Hz(sim) "
            f"loss {100 * self._cam_gap.loss_ratio():.1f}% age p95 {_fmt(cam_age, 'p95')} | "
            f"det {self._det_rate.rate(wall):.1f}Hz tgt {self._det_tgt_rate.rate(wall):.1f}Hz "
            f"skip {100 * self._det_gap.loss_ratio():.0f}% proc {_fmt(det_proc)}/p95 "
            f"{_fmt(det_proc, 'p95')} (compute {_fmt(compute)}, queue+tx "
            f"{'n/a' if queue_s is None else f'{queue_s * 1000:.0f}ms'}) age(sim) {_fmt(det_age)} | "
            f"tgt gap max {_fmt(tgt_gap, 'max', 1.0, 's')} "
            f"(lost {'?' if lost_to is None else lost_to}s, pn_age {'?' if age_to is None else age_to}s) | "
            f"att {self._att_rate.rate(wall):.0f}Hz | {gt_txt} | state {self._state}")
        if gt_on and self._gt_rate.rate(wall) <= 0.0:
            self.get_logger().warn(
                "[GT] use_ground_truth=true 인데 sim/guidance_ground_truth 가 안 온다 → "
                "PN 갱신이 전부 건너뛰어진다(표적이 숨김 상태이거나 referee 미기동?)")
        if lost_to and tgt_gap and tgt_gap["max"] > 0.5 * float(lost_to):
            self.get_logger().warn(
                f"[det] 표적 검출 간격 최대 {tgt_gap['max']:.2f}s 가 TRACK 소실 timeout "
                f"{lost_to}s 의 절반을 넘었다 — 검출 주기가 상태머신 여유를 잠식")
        rail = summary["rail"]
        if rail and "err_rpy_deg" in rail and self._launcher_status.startswith("장전됨") \
                and max(abs(v) for v in rail["err_rpy_deg"]) > 10.0:
            self.get_logger().warn(
                f"[att] 장착 상태 PX4 자세가 발사기 기하와 다르다: 기대 {rail['expected_rpy_deg']} "
                f"실제 {rail['px4_rpy_deg']} (EKF heading/자력계 확인)")
        self._tgt_gap.clear()
        self._det_proc.clear()
        self._det_compute.clear()
        self._det_yolo.clear()
        self._cam_age.clear()
        self._det_age.clear()

    def destroy_node(self):
        if self._jsonl:
            self._jsonl.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = AngledLaunchMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
