#!/usr/bin/env python3
"""
angled_stability_test.py — 기울어진 초기 자세 비충돌 SITL 시험 드라이버  [arms_sim]

arms_sitl.launch.py(gui:=false) 가 떠 있는 상태에서 가상 조종기 대신 operator/input 을 내고,
결과를 JSONL 로 남긴다. 표적을 향한 유도 비행은 하지 않는다(비충돌).

  P1 발사관 위 자세   : PX4 자세 vs 발사관 기하(기대값) vs Gazebo 실제 자세(gz pose → NED/FRD).
                        EKF 재정렬(launcher 자동 요청) 전후 오차와 정렬 소요 시간.
  P2 자동 모드 안전   : release_enabled=false(분리 금지)로 발사관에 물린 채 SEARCH→LOCK→TRACK.
                        표적을 화면 밖으로 옮겨 TRACK→IDLE 수동 인계, hold 구간 명령(추력 0 여부),
                        무장 유지, ARM OFF disarm 을 기록.
  P3 수동 이탈 비행   : 표적은 멀리 치우고 수동(Stabilized) 모드로 ARM → 스로틀로 분리 →
                        스틱 중앙(수평·호버)을 유지. 자세 수렴 시간, 최대 기울기, 고도·수평 변화.

사용:  ros2 run arms_sim angled_stability_test --ros-args -p elev:=50.0 -p out:=/tmp/x.jsonl
"""

import json
import math
import os
import signal
import threading
import time

os.environ.setdefault("PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION", "python")

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rcl_interfaces.msg import Parameter as ParamMsg, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from rosgraph_msgs.msg import Clock
from std_msgs.msg import Bool, String
from arms_msgs.msg import (AttitudeTelemetry, DetectionArray, FlightCommand, GpsTelemetry,
                           IoStatus, MissionState, OperatorInput, PnDebug)

from arms_sim import launcher_geometry as G

try:
    from gz.transport13 import Node as _GzNode
    from gz.msgs10.pose_v_pb2 import Pose_V as _GzPoseV
    _gz = _GzNode()
except Exception:   # pragma: no cover - gz 없는 환경
    _gz = None

LAUNCHER = "/arms/arms_launcher"
REFEREE = "/arms/balloon_referee"


def _pv(v):
    pv = ParameterValue()
    if isinstance(v, bool):
        pv.type, pv.bool_value = ParameterType.PARAMETER_BOOL, v
    else:
        pv.type, pv.double_value = ParameterType.PARAMETER_DOUBLE, float(v)
    return pv


class StabilityTest(Node):
    def __init__(self):
        super().__init__("angled_stability_test")
        self.declare_parameter("elev", 90.0)
        self.declare_parameter("az", 0.0)
        self.declare_parameter("out", "")
        self.declare_parameter("drone_model", "arms_drone_1")
        self.declare_parameter("world", "arms_sitl")
        self.declare_parameter("phases", "P1,P2,P3")
        self.declare_parameter("flight_sim_sec", 12.0)
        # PA 단계용: 런치는 start_elev(기본 90°)로 띄우고, 정렬 뒤 elev 로 기울인다(패널 조작과 같음).
        self.declare_parameter("start_elev", -1.0)
        self.elev = float(self.get_parameter("elev").value)
        self.az = float(self.get_parameter("az").value)
        self.drone = str(self.get_parameter("drone_model").value)
        out = os.path.expanduser(str(self.get_parameter("out").value) or
                                 f"~/arms_diag_logs/stability_elev{self.elev:.0f}.jsonl")
        os.makedirs(os.path.dirname(out), exist_ok=True)
        self._out = open(out, "a", encoding="utf-8")
        self.get_logger().info(f"결과 파일: {out}")

        self.lock = threading.Lock()
        self.sim = None
        self.io = None
        self.att = None
        self.gps = None
        self.gps0 = None
        self.state = None
        self.armed = False           # control 의 CH5 요청 (MissionState.armed)
        self.fc_armed = False        # PX4 실제 무장 (vehicle/armed)
        self.manual_out = False
        self.cmd = None
        self.pn = None
        self.launcher_status = ""
        self.det_target_t = None
        self.truth = None            # (x,y,z, roll,pitch,yaw PX4 규약)
        self.transitions = []
        self.cmd_log = []
        # operator 입력 (드라이버가 바꾼다)
        self.op = dict(roll=0.0, pitch=0.0, throttle=-1.0, yaw=0.0,
                       arm=False, manual=False, launch=False)

        ns = "/arms/"
        self.create_subscription(Clock, "/clock", self._on_clock, 10)
        self.create_subscription(IoStatus, ns + "io/status", lambda m: setattr(self, "io", m), 10)
        self.create_subscription(AttitudeTelemetry, ns + "vehicle/attitude", self._on_att,
                                 qos_profile_sensor_data)
        self.create_subscription(GpsTelemetry, ns + "vehicle/gps", self._on_gps, 10)
        self.create_subscription(MissionState, ns + "mission/state", self._on_state, 10)
        self.create_subscription(FlightCommand, ns + "control/flight_command", self._on_cmd, 50)
        self.create_subscription(PnDebug, ns + "control/pn_debug", lambda m: setattr(self, "pn", m), 10)
        self.create_subscription(String, ns + "sim/launcher/status", self._on_launcher, 10)
        self.create_subscription(Bool, ns + "vehicle/armed",
                                 lambda m: setattr(self, "fc_armed", bool(m.data)), 10)
        self.create_subscription(DetectionArray, ns + "perception/detections", self._on_det, 10)
        self._pub_op = self.create_publisher(OperatorInput, ns + "operator/input", 10)
        self._set = {n: self.create_client(SetParameters, f"{n}/set_parameters")
                     for n in (LAUNCHER, REFEREE)}
        self.create_timer(0.02, self._tick_op)
        if _gz is not None:
            _gz.subscribe(_GzPoseV, f"/world/{self.get_parameter('world').value}/dynamic_pose/info",
                          self._on_pose)

    # ── 입력 ─────────────────────────────────────────────────────────────
    def _on_clock(self, m):
        self.sim = m.clock.sec + m.clock.nanosec * 1e-9

    def _on_att(self, m):
        self.att = (m.roll_deg, m.pitch_deg, m.yaw_deg)

    def _on_gps(self, m):
        if m.valid_fix:
            self.gps = (m.latitude_deg, m.longitude_deg, m.altitude_m)

    def _on_state(self, m):
        with self.lock:
            if self.state is not None and m.state != self.state:
                self.transitions.append({"wall": time.time(), "sim": self.sim,
                                         "prev": self.state, "new": m.state,
                                         "armed": m.armed, "manual_out": m.manual_mode})
            self.state, self.armed, self.manual_out = m.state, m.armed, m.manual_mode

    def _on_cmd(self, m):
        with self.lock:
            self.cmd = m
            self.cmd_log.append((time.time(), self.sim, self.state, m))
            if len(self.cmd_log) > 20000:
                del self.cmd_log[:5000]

    def _on_launcher(self, m):
        self.launcher_status = m.data

    def _on_det(self, m):
        if m.detections:
            self.det_target_t = time.time()

    def _on_pose(self, msg):
        for p in msg.pose:
            if p.name == self.drone:
                q = (p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w)
                self.truth = (p.position.x, p.position.y, p.position.z,
                              *G.enu_flu_quat_to_px4_euler_deg(q))
                return

    def _tick_op(self):
        o = self.op
        m = OperatorInput()
        m.roll, m.pitch, m.throttle, m.yaw = o["roll"], o["pitch"], o["throttle"], o["yaw"]
        m.arm_switch, m.manual_mode, m.launch_request = o["arm"], o["manual"], o["launch"]
        self._pub_op.publish(m)

    # ── 도우미 ───────────────────────────────────────────────────────────
    def rec(self, kind, **kw):
        line = json.dumps({"t": round(time.time(), 3), "sim": self.sim, "kind": kind,
                           "elev": self.elev, **kw}, ensure_ascii=False, default=str)
        self._out.write(line + "\n")
        self._out.flush()
        self.get_logger().info(line[:400])

    def set_params(self, node, values, timeout=10.0):
        cli = self._set[node]
        if not cli.wait_for_service(timeout_sec=timeout):
            return False
        fut = cli.call_async(SetParameters.Request(
            parameters=[ParamMsg(name=k, value=_pv(v)) for k, v in values.items()]))
        t0 = time.time()
        while not fut.done() and time.time() - t0 < timeout:
            time.sleep(0.05)
        return fut.done() and all(r.successful for r in fut.result().results)

    def wait(self, pred, timeout, step=0.05):
        t0 = time.time()
        while time.time() - t0 < timeout and rclpy.ok():
            if pred():
                return True
            time.sleep(step)
        return pred()

    def wait_sim(self, sim_sec, wall_cap):
        s0, t0 = self.sim, time.time()
        while time.time() - t0 < wall_cap and rclpy.ok():
            if self.sim is not None and s0 is not None and self.sim - s0 >= sim_sec:
                return True
            time.sleep(0.05)
        return False

    def expected(self):
        return G.expected_px4_euler_deg(self.elev, self.az)

    def att_errors(self):
        exp = self.expected()
        out = {"expected": [round(v, 2) for v in exp]}
        if self.att:
            out["px4"] = [round(v, 2) for v in self.att]
            out["px4_minus_expected"] = [round(G.angle_difference_deg(a, e), 2)
                                         for a, e in zip(self.att, exp)]
        if self.truth:
            tr = self.truth[3:]
            out["gz_truth"] = [round(v, 2) for v in tr]
            out["gz_minus_expected"] = [round(G.angle_difference_deg(a, e), 2)
                                        for a, e in zip(tr, exp)]
            if self.att:
                out["px4_minus_gz"] = [round(G.angle_difference_deg(a, t), 2)
                                       for a, t in zip(self.att, tr)]
            out["gz_pos"] = [round(v, 3) for v in self.truth[:3]]
        return out

    def toggle_arm(self, manual):
        self.op.update(manual=manual, arm=False, throttle=-1.0, roll=0.0, pitch=0.0, yaw=0.0)
        time.sleep(0.6)
        self.op["arm"] = True

    # ── 시험 단계 ─────────────────────────────────────────────────────────
    def p0_ready(self):
        start = float(self.get_parameter("start_elev").value)
        e0 = start if start >= 0.0 else self.elev
        ok = self.wait(lambda: self.io is not None and self.io.vehicle_telemetry_connected
                       and self.launcher_status.startswith("장전됨")
                       and f"앙각 {e0:.0f}°" in self.launcher_status
                       and self.att is not None and self.sim is not None, 300.0)
        self.rec("ready", ok=ok, launcher=self.launcher_status)
        return ok

    def p1_rail(self):
        first = self.att_errors()
        self.rec("rail_first", **first, launcher=self.launcher_status)
        t0, s0 = time.time(), self.sim
        aligned_at = None
        while time.time() - t0 < 90.0 and rclpy.ok():
            if "자세정렬 OK" in self.launcher_status and aligned_at is None:
                aligned_at = (time.time() - t0, (self.sim or 0) - (s0 or 0))
            if aligned_at and time.time() - t0 > aligned_at[0] + 8.0:
                break
            time.sleep(0.5)
        final = self.att_errors()
        self.rec("rail_final", **final, launcher=self.launcher_status,
                 aligned_after_wall_s=aligned_at and round(aligned_at[0], 1),
                 aligned_after_sim_s=aligned_at and round(aligned_at[1], 1))
        return aligned_at is not None

    def pa_tilt_after_alignment(self):
        """부팅·정렬 뒤 발사관을 기울인다(패널과 같은 파라미터 경로). 재정렬을 끈 채로 오차를
        재고(문제 재현), 켠 뒤 수렴을 잰다(수정 검증)."""
        start = float(self.get_parameter("start_elev").value)
        aligned0 = self.wait(lambda: "자세정렬 OK" in self.launcher_status, 90.0)
        self.rec("pa_start", start_elev=start, aligned_at_start=aligned0, **self.att_errors())
        self.set_params(LAUNCHER, {"realign_estimator": False})
        self.set_params(LAUNCHER, {"elevation_deg": self.elev, "azimuth_deg": self.az})
        self.wait(lambda: self.launcher_status.startswith("장전됨")
                  and f"앙각 {self.elev:.0f}°" in self.launcher_status, 60.0)
        t_tilt = self.sim
        series = []
        while self.sim is not None and self.sim - t_tilt < 15.0 and rclpy.ok():
            e = self.att_errors()
            if "px4_minus_expected" in e:
                series.append((round(self.sim - t_tilt, 2), e["px4_minus_expected"]))
            time.sleep(0.5)
        worst = max((max(abs(v) for v in err) for _, err in series), default=None)
        self.rec("pa_no_realign", after_tilt_sim_s=15.0, worst_err_deg=worst,
                 first=series[:1], last=series[-1:], **self.att_errors())
        self.set_params(LAUNCHER, {"realign_estimator": True})
        t_on = self.sim
        ok = self.wait(lambda: "자세정렬 OK" in self.launcher_status, 120.0)
        self.rec("pa_realign", ok=ok, align_sim_s=(self.sim - t_on) if ok else None,
                 **self.att_errors(), launcher=self.launcher_status)
        return ok

    def p2_auto_safety(self):
        self.set_params(LAUNCHER, {"release_enabled": False})
        tgt = G.boresight_point(self.elev, self.az, 25.0)
        self.set_params(REFEREE, {"enabled": False, "pos_x": round(tgt[0], 2),
                                  "pos_y": round(tgt[1], 2), "alt": round(tgt[2], 2)})
        seen = self.wait(lambda: self.det_target_t and time.time() - self.det_target_t < 1.0, 60.0)
        self.rec("p2_target_on_boresight", target=tgt, detected=bool(seen))
        self.toggle_arm(manual=False)
        if not self.wait(lambda: self.state == "LOCK", 90.0):
            self.rec("p2_result", ok=False, why=f"LOCK 미도달 (state={self.state})")
            self.op["arm"] = False
            return False
        self.op["launch"] = True
        ok = self.wait(lambda: self.state == "TRACK", 10.0)
        self.op["launch"] = False
        if not ok:
            self.rec("p2_result", ok=False, why="TRACK 미진입")
            self.op["arm"] = False
            return False
        self.wait_sim(2.0, 30.0)
        with self.lock:
            track_cmds = [m for _, _, s, m in self.cmd_log[-400:] if s == "TRACK"]
        active = sum(1 for m in track_cmds if m.throttle > 1e-3)
        self.rec("p2_track", ticks=len(track_cmds), nonzero_thrust=active,
                 pn_active=bool(self.pn and self.pn.command_active),
                 launcher=self.launcher_status)
        # 표적을 화면 밖으로 → 표적 소실
        self.set_params(REFEREE, {"pos_x": -60.0, "pos_y": 60.0, "alt": 5.0})
        lost_wall, lost_sim = time.time(), self.sim
        ok = self.wait(lambda: self.state == "IDLE", 120.0)
        idle_wall, idle_sim = time.time(), self.sim
        time.sleep(0.5)
        with self.lock:
            hold = [(sm, m) for w, sm, s, m in self.cmd_log
                    if lost_wall <= w <= idle_wall and s == "TRACK"]
            trans = list(self.transitions[-6:])
        zero = sum(1 for _, m in hold if m.throttle <= 1e-3 and not m.manual_mode)
        thr = sorted({round(m.throttle, 3) for _, m in hold})
        c = self.cmd
        self.rec("p2_result", ok=ok, lost_to_idle_sim_s=(idle_sim - lost_sim) if ok else None,
                 lost_to_idle_wall_s=round(idle_wall - lost_wall, 2),
                 hold_ticks=len(hold), hold_zero_thrust_ticks=zero, hold_throttles=thr[:10],
                 after_manual=bool(c and c.manual_mode), after_arm=bool(c and c.arm),
                 after_rtl=bool(c and c.rtl_request), transitions=trans)
        self.op["arm"] = False
        disarmed = self.wait(lambda: self.cmd is not None and not self.cmd.arm, 10.0)
        self.rec("p2_disarm", ok=disarmed)
        self.set_params(LAUNCHER, {"release_enabled": True})
        return ok

    def p3_manual_flight(self):
        self.set_params(REFEREE, {"enabled": False, "pos_x": -80.0, "pos_y": 80.0, "alt": 5.0})
        self.set_params(LAUNCHER, {"release_enabled": True})
        aligned = self.wait(lambda: "자세정렬 OK" in self.launcher_status, 90.0)
        self.rec("p3_pre", aligned=aligned, **self.att_errors(), launcher=self.launcher_status)
        self.toggle_arm(manual=True)
        if not self.wait(lambda: self.armed and self.fc_armed, 30.0):
            self.rec("p3_result", ok=False,
                     why=f"ARM 실패 (요청={self.armed}, PX4 실제={self.fc_armed})")
            return False
        self.wait_sim(2.0, 30.0)             # PX4 spool-up(시뮬 1 s) 이후에 스로틀
        self.op["throttle"] = 0.25           # 호버 이상 → 발사기 수동 분리 조건
        released = self.wait(lambda: self.launcher_status.startswith("발사됨"), 30.0)
        rel_sim, rel_wall = self.sim, time.time()
        rel_truth = self.truth
        self.op["throttle"] = 0.0            # 스틱 중앙 = PX4 Stabilized 호버, 롤/피치 0 = 수평
        if not released:
            self.rec("p3_result", ok=False, why=f"분리 안 됨: {self.launcher_status}")
            self.op["arm"] = False
            return False
        samples = []
        dur = float(self.get_parameter("flight_sim_sec").value)
        while self.sim - rel_sim < dur and time.time() - rel_wall < dur * 15 and rclpy.ok():
            if self.truth and self.att:
                samples.append({"sim": round(self.sim - rel_sim, 3),
                                "px4": [round(v, 2) for v in self.att],
                                "gz": [round(v, 3) for v in self.truth]})
            time.sleep(0.1)
        res = self._flight_metrics(samples, rel_truth)
        self.rec("p3_result", ok=True, release_truth=rel_truth, **res, samples=samples[::5])
        self.op.update(throttle=-0.4)
        self.wait_sim(3.0, 40.0)
        self.op["arm"] = False
        self.wait(lambda: self.cmd is not None and not self.cmd.arm, 10.0)
        return True

    @staticmethod
    def _flight_metrics(samples, rel_truth):
        if not samples:
            return {"n": 0}

        def settle(idx):   # |roll|,|pitch| 가 5° 안으로 들어와 끝까지 머문 첫 시각
            t_set = None
            for s in samples:
                r, p = s[idx][0 if idx == "px4" else 3], s[idx][1 if idx == "px4" else 4]
                if abs(r) < 5.0 and abs(p) < 5.0:
                    t_set = t_set if t_set is not None else s["sim"]
                else:
                    t_set = None
            return t_set

        late = [s for s in samples if s["sim"] > 3.0] or samples
        # 분리 1 s 뒤(스틱 중앙=MPC_THR_HOVER) z(t) 2차 맞춤 → 수직 가속도. 0 이면 그 추력이 호버.
        fit = [(s["sim"], s["gz"][2]) for s in samples if 1.0 <= s["sim"] <= 6.0]
        a_z = None
        if len(fit) >= 8:
            ts = [t for t, _ in fit]
            # 최소제곱 z = c0 + c1 t + c2 t^2  (정규방정식)
            S = [[sum(t ** (i + j) for t in ts) for j in range(3)] for i in range(3)]
            b = [sum(z * t ** i for t, z in fit) for i in range(3)]
            for i in range(3):            # 가우스 소거
                piv = S[i][i]
                for j in range(i, 3):
                    S[i][j] /= piv
                b[i] /= piv
                for k in range(3):
                    if k != i:
                        f = S[k][i]
                        for j in range(i, 3):
                            S[k][j] -= f * S[i][j]
                        b[k] -= f * b[i]
            a_z = 2.0 * b[2]

        gz0 = rel_truth or samples[0]["gz"]
        last = samples[-1]["gz"]
        return {
            "n": len(samples),
            "settle_px4_sim_s": settle("px4"),
            "settle_gz_sim_s": settle("gz"),
            "max_tilt_gz_after3s_deg": round(max(max(abs(s["gz"][3]), abs(s["gz"][4]))
                                                 for s in late), 2),
            "max_tilt_px4_after3s_deg": round(max(max(abs(s["px4"][0]), abs(s["px4"][1]))
                                                  for s in late), 2),
            "px4_minus_gz_max_deg_after3s": round(max(
                max(abs(G.angle_difference_deg(s["px4"][i], s["gz"][3 + i])) for i in range(2))
                for s in late), 2),
            "alt_change_m": round(last[2] - gz0[2], 3),
            "min_alt_m": round(min(s["gz"][2] for s in samples), 3),
            "horiz_disp_m": round(math.hypot(last[0] - gz0[0], last[1] - gz0[1]), 3),
            "final_gz_rpy": last[3:],
            "vertical_accel_center_stick_mps2": None if a_z is None else round(a_z, 3),
        }

    def run(self):
        phases = [p.strip() for p in str(self.get_parameter("phases").value).split(",")]
        self.rec("start", phases=phases, expected=self.expected())
        if not self.p0_ready():
            self.rec("end", ok=False, why="준비 안 됨")
            return
        results = {}
        for p in phases:
            fn = {"PA": self.pa_tilt_after_alignment, "P1": self.p1_rail,
                  "P2": self.p2_auto_safety, "P3": self.p3_manual_flight}[p]
            try:
                results[p] = bool(fn())
            except Exception as e:   # 시험 드라이버 예외도 기록하고 다음 단계로
                results[p] = False
                self.rec("error", phase=p, error=repr(e))
        self.rec("end", ok=all(results.values()), results=results)


def main(args=None):
    # SIGTERM 도 KeyboardInterrupt 로 → finally 에서 disarm 후 종료 (시험 래퍼 정리용)
    signal.signal(signal.SIGTERM, lambda *_: (_ for _ in ()).throw(KeyboardInterrupt()))
    rclpy.init(args=args)
    node = StabilityTest()
    ex = MultiThreadedExecutor(num_threads=2)
    ex.add_node(node)
    threading.Thread(target=ex.spin, daemon=True).start()
    try:
        node.run()
    except KeyboardInterrupt:
        node.rec("end", ok=False, why="중단됨")
    finally:
        node.op["arm"] = False
        time.sleep(0.5)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
