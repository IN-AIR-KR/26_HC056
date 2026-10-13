#!/usr/bin/env python3
"""
panel.py — A.R.M.S. SITL 튜닝/심판 콘솔 (tkinter GUI)  [arms_sim]

SITL 전용 개발 콘솔. 운용자 입력은 발행하지 않는다 (그건 가상 조종기 =
arms_operator 노드 담당). 여기서 하는 일은 전부 파라미터/서비스 세팅뿐:
  - 미션 상태 표시 (IDLE/SEARCH/LOCK/TRACK/RTL) — 읽기전용 구독
  - Roll/Pitch 부호, PID, τ, 추력, PN 게인 등 → arms_control_node 파라미터
  - 표적(풍선/드론) 비행·정지·고도·속도 → referee 파라미터
  - RESET → Gazebo world reset + PX4 SITL 재시작 (Gazebo 프로세스는 유지)

파라미터 쓰기는 ParamBus 하나로 순서대로 처리하고, 매번 노드에서 다시 읽어
표시한다(패널 표시 = 노드 실제 값). 결과는 상단 상태줄에 ✓/✗ 로 뜬다.

실행: ros2 run arms_sim panel
"""

import queue
import threading
import time
import tkinter as tk

import rclpy
from rclpy.node import Node
from arms_msgs.msg import MissionState
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import GetParameters, SetParameters
from std_msgs.msg import Empty, String
from std_srvs.srv import Trigger

from arms_sim import launcher_geometry as G

CTRL_NODE    = "/arms/arms_control_node"
REFEREE_NODE = "/arms/balloon_referee"
LAUNCHER_NODE = "/arms/arms_launcher"   # 견착 발사기 (arms_sim/launcher.py)

WORLD = "arms_sitl"
DRONE = "arms_drone_0"
BALL  = "target_ball"

STATE_COLOR = {
    "IDLE":   "#888888",
    "SEARCH": "#f5a623",
    "LOCK":   "#f57c00",
    "TRACK":  "#d0021b",
    "RTL":    "#2979ff",
}

def group(parent, title):
    """제목 달린 묶음 상자. tkinter 의 기본 그룹 위젯이 LabelFrame 이다.

    패널이 커지면서 슬라이더가 한 줄로 주욱 늘어서 어느 게 어느 계통인지
    읽히지 않았다. 여기서 묶는 기준은 '무엇을 튜닝하느냐'가 아니라
    '어느 노드의 파라미터냐'다 — 제어기(arms_control), 심판/표적(referee).
    잘못된 노드에 값을 쏘면 조용히 아무 일도 안 일어나므로, 그 경계가 눈에
    보이는 편이 낫다.
    """
    f = tk.LabelFrame(parent, text=title, fg="#9fc3e8", bg="#1e1e1e",
                      font=("Arial", 10, "bold"), bd=1, relief="groove",
                      labelanchor="nw", padx=8, pady=6)
    f.pack(fill="x", pady=(8, 0))
    return f


# ---------------------------------------------------------------------------
# 파라미터 버스 — 패널 → 노드 쓰기/읽기를 한 줄로 세운다
# ---------------------------------------------------------------------------
#
# 예전 방식의 문제 (2026-10-11 실제로 겪음):
#   버튼/슬라이더마다 스레드를 새로 띄워 `ros2 param set` CLI 를 병렬 실행했고
#   결과는 DEVNULL 로 버렸다. 그래서
#     1) 순서 보장이 없다: 유도 방식을 0→1→2 로 빠르게 누르면 "set 2" 가 먼저
#        끝나고 "set 1" 이 나중에 끝나 노드엔 1(PN)이 남는다.
#     2) 성공 확인이 없다: 버튼은 누르는 즉시 IBVS 로 바뀌어 보이는데 노드는 PN.
#     3) CLI 는 호출마다 수 초 걸려(노트북 부하 시 더) 위 레이스가 잘 터진다.
#   → 패널은 IBVS 를 보여주는데 제어기는 PN 으로 비행했다.
#
# 지금 방식:
#   - CLI 대신 노드의 파라미터 서비스(get/set_parameters)를 패널 노드가 직접 부른다.
#   - 쓰기는 작업 큐 하나 + 워커 스레드 하나로 '누른 순서대로' 한 번에 하나씩.
#   - 매 쓰기마다: 현재 타입 조회 → 그 타입으로 변환해 set → 결과 확인 → 다시 읽기.
#   - 다시 읽은 값(=노드의 진짜 값)으로 위젯을 갱신하고 성공/실패를 상태줄에 띄운다.

_SVC_TIMEOUT = 3.0   # 서비스 대기/응답 한도 [s]


def _pv_to_str(pv):
    """ParameterValue → 위젯 반영용 문자열 (예전 CLI 출력과 같은 꼴)."""
    t = pv.type
    if t == ParameterType.PARAMETER_BOOL:
        return "True" if pv.bool_value else "False"
    if t == ParameterType.PARAMETER_INTEGER:
        return str(pv.integer_value)
    if t == ParameterType.PARAMETER_DOUBLE:
        return str(pv.double_value)
    if t == ParameterType.PARAMETER_STRING:
        return pv.string_value
    return None


def _to_pv(value, ptype):
    """value 를 노드에 선언된 타입(ptype)에 맞춘 ParameterValue 로 만든다.

    타입이 틀리면 노드가 set 을 거부한다(예: int 파라미터에 2.0). 그래서
    보내기 전에 현재 값의 타입을 읽어와 거기에 맞춘다.
    """
    pv = ParameterValue(type=ptype)
    if ptype == ParameterType.PARAMETER_BOOL:
        pv.bool_value = (value if isinstance(value, bool)
                         else str(value).strip().lower() in ("true", "1", "yes", "on"))
    elif ptype == ParameterType.PARAMETER_INTEGER:
        pv.integer_value = int(round(float(value)))
    elif ptype == ParameterType.PARAMETER_DOUBLE:
        pv.double_value = float(value)
    elif ptype == ParameterType.PARAMETER_STRING:
        pv.string_value = str(value)
    else:
        raise ValueError(f"지원하지 않는 파라미터 타입 {ptype}")
    return pv


class ParamBus:
    """노드 파라미터를 순서대로·확인하며 쓰고, 읽어온 진짜 값을 돌려준다.

    on_result(node, name, ok, value_str, msg, more_pending) 는 워커 스레드에서
    호출된다 — 위젯은 호출측이 메인 스레드로 넘겨서 건드릴 것.
      value_str    : 쓰기 후 다시 읽은 노드의 실제 값 (못 읽으면 None)
      more_pending : 같은 파라미터에 대한 쓰기가 큐에 더 남았는지. True 면 중간값이니
                     위젯을 되돌리지 말 것(연타 시 표시가 출렁이지 않게).
    """

    def __init__(self, node, target_nodes, on_result):
        self._node = node
        self._on_result = on_result
        # 클라이언트는 메인 스레드에서 미리 만든다(실행 중 노드에 추가하지 않게).
        self._get = {n: node.create_client(GetParameters, f"{n}/get_parameters")
                     for n in target_nodes}
        self._set = {n: node.create_client(SetParameters, f"{n}/set_parameters")
                     for n in target_nodes}
        self._call_lock = threading.Lock()   # 시작 시 pull 워커와 쓰기 워커가 겹치지 않게
        # 쓰기 큐는 '키' 단위로 합친다(coalesce): 같은 파라미터에 값이 연달아 오면
        # 아직 안 보낸 옛 값은 버리고 마지막 값만 보낸다. 슬라이더를 드래그해도 큐가
        # 밀리지 않고, 연타해도 최종값 = 마지막 입력이 보장된다.
        self._q = queue.Queue()              # 처리 대기 키 (처음 들어온 순서)
        self._latest = {}                    # (node, name) → 아직 안 보낸 최신 값
        self._latest_lock = threading.Lock()
        self._types = {}                     # (node, name) → 선언 타입 캐시
        threading.Thread(target=self._run, daemon=True).start()

    # ---- 공개 API ----------------------------------------------------------
    def set(self, node, name, value):
        """쓰기 예약(비차단). 하나씩 순서대로 처리되고, 같은 키는 마지막 값만 남는다."""
        key = (node, name)
        with self._latest_lock:
            queued = key in self._latest
            self._latest[key] = value
        if not queued:
            self._q.put(key)

    def get(self, node, name):
        """노드의 현재 값을 ParameterValue 로 읽는다(차단). 실패 시 None."""
        client = self._get.get(node)
        if client is None:
            return None
        req = GetParameters.Request()
        req.names = [name]
        res = self._call(client, req)
        if res is None or not res.values:
            return None
        pv = res.values[0]
        if pv.type == ParameterType.PARAMETER_NOT_SET:
            return None
        return pv

    def get_str(self, node, name):
        pv = self.get(node, name)
        return None if pv is None else _pv_to_str(pv)

    # ---- 내부 --------------------------------------------------------------
    def _call(self, client, req):
        with self._call_lock:
            if not client.wait_for_service(timeout_sec=_SVC_TIMEOUT):
                return None
            fut = client.call_async(req)
            deadline = time.monotonic() + _SVC_TIMEOUT
            while not fut.done():               # 응답은 rclpy.spin 스레드가 채운다
                if time.monotonic() > deadline:
                    try:
                        client.remove_pending_request(fut)
                    except Exception:
                        pass
                    return None
                time.sleep(0.005)
            try:
                return fut.result()
            except Exception:
                return None

    def _run(self):
        while True:
            key = self._q.get()
            node, name = key
            with self._latest_lock:
                value = self._latest.pop(key)
            ok, msg = False, ""
            try:
                ptype = self._types.get(key)
                if ptype is None:
                    cur = self.get(node, name)
                    ptype = None if cur is None else cur.type
                    if ptype is not None:
                        self._types[key] = ptype
                if ptype is None:
                    msg = "노드 응답 없음 또는 파라미터 없음"
                else:
                    req = SetParameters.Request()
                    req.parameters = [Parameter(name=name, value=_to_pv(value, ptype))]
                    res = self._call(self._set[node], req)
                    if res is None or not res.results:
                        msg = "set 응답 없음(타임아웃)"
                        self._types.pop(key, None)    # 노드 재기동 등 → 다음엔 다시 조회
                    elif not res.results[0].successful:
                        msg = res.results[0].reason or "노드가 거부"
                        self._types.pop(key, None)
                    else:
                        ok = True
            except (ValueError, TypeError) as e:
                msg = f"값 변환 실패: {value!r} ({e})"
            # 성공이든 실패든 노드의 '진짜 값'을 다시 읽어 돌려준다.
            back = self.get_str(node, name)
            with self._latest_lock:
                more = key in self._latest           # 처리하는 사이 새 값이 들어왔나
            try:
                self._on_result(node, name, ok, back, msg, more)
            except Exception as e:
                print(f"[panel] 결과 처리 실패: {e}")


_BUS = None   # PanelGUI 가 만든다. 아래 두 함수는 기존 호출부 호환용 래퍼.


def ros_param_set(name, value):
    """arms_control_node 파라미터 쓰기 (순차·확인)."""
    _BUS.set(CTRL_NODE, name, value)


def ros_param_set_node(node, name, value):
    """임의 노드 파라미터 쓰기 (순차·확인)."""
    _BUS.set(node, name, value)


class PanelNode(Node):
    """콘솔 노드 — 미션 상태만 구독(표시용). 운용자 입력은 발행하지 않는다."""

    def __init__(self, q):
        super().__init__("arms_panel_node")
        self.q = q
        self.create_subscription(MissionState, "mission/state", self._cb_state, 10)
        self.create_subscription(String, "sim/launcher/status", self._cb_state, 10)
        self.reset_client = self.create_client(Trigger, "sim/reset")
        self.reload_pub = self.create_publisher(Empty, "sim/launcher/reload_manual", 10)

    def _cb_state(self, msg):
        self.q.put(msg)


# ---------------------------------------------------------------------------
# 메인 컨트롤 패널
# ---------------------------------------------------------------------------

class PanelGUI:
    def __init__(self, root, node, q):
        self.root = root
        self.node = node
        self.q = q
        self.roll_sign  = 1.0
        self.pitch_sign = 1.0
        # (노드, 파라미터, 반영함수) — _pull_from_nodes 가 시작할 때 읽어 채운다.
        self._pullables = []
        self._applyq = queue.Queue()      # 워커 → 메인 루프 (_poll 이 비운다)
        self._reset_in_progress = False

        # 파라미터 쓰기/읽기 버스 (순차 + 결과 확인 + 다시 읽기)
        global _BUS
        self._bus = ParamBus(
            node, [CTRL_NODE, REFEREE_NODE, LAUNCHER_NODE],
            on_result=lambda *r: self._applyq.put((self._on_param_result, r)))
        _BUS = self._bus

        root.title("A.R.M.S. Control Panel")
        root.configure(bg="#1e1e1e")
        # 크기를 지정하지 않는다 = tkinter 가 내용 자연 크기로 맞춘다.
        # 예전엔 geometry("612x766") 로 박아뒀는데, 자연 크기는 폰트/DPI 에 따라
        # 달라져서 개발 머신이 바뀌면 그대로 어긋난다(실제로 이 머신에선 805 가
        # 필요해 아래 39px 가 잘리고 있었다). 위젯을 늘리든 줄이든 여기는
        # 손댈 필요가 없다.

        # ── 상단: 미션 상태 (전체 폭) ──────────────────────────────────────
        top = tk.Frame(root, bg="#1e1e1e")
        top.pack(fill="x", padx=12, pady=(10, 4))

        tk.Label(top, text="MISSION STATE", fg="#aaaaaa", bg="#1e1e1e",
                 font=("Arial", 11)).pack()
        self.state_lbl = tk.Label(top, text="—", fg="white", bg="#888888",
                                   font=("Arial", 24, "bold"), width=14)
        self.state_lbl.pack(pady=3, ipady=6)
        self.info_lbl = tk.Label(top, text="error: -, -\nlock: 0.0s",
                                 fg="#dddddd", bg="#1e1e1e", font=("Consolas", 10))
        self.info_lbl.pack()
        self.reset_btn = tk.Button(
            top, text="실험 RESET (PX4 + 월드)", width=24,
            font=("Arial", 10, "bold"), bg="#b71c1c", fg="white",
            activebackground="#e53935", command=self.reset_episode)
        self.reset_btn.pack(pady=(7, 0))
        # 파라미터 적용 결과 상태줄 — 노드가 실제로 받아들였는지(다시 읽은 값)를 보여준다.
        self.param_lbl = tk.Label(top, text="파라미터: 노드에서 읽는 중…",
                                  fg="#dddddd", bg="#1e1e1e", font=("Consolas", 10),
                                  wraplength=560, justify="center")
        self.param_lbl.pack(pady=(6, 0))
        # LAUNCH/arm/RTL/mode는 가상 조종기(arms_operator) 소관 — operator/input 발행자는
        # 조종기 하나뿐이어야 한다(두 노드가 동시에 쏘면 레이스). 여긴 튜닝/심판만.

        # ── 2열 레이아웃 ────────────────────────────────────────────────────
        cols = tk.Frame(root, bg="#1e1e1e")
        cols.pack(fill="both", expand=True, padx=(12, 4), pady=4)

        # 가운데 구분선 없음. LabelFrame 테두리가 이미 그룹 경계를 그리므로
        # 세로줄까지 있으면 선이 두 겹이 된다.
        left = tk.Frame(cols, bg="#1e1e1e")
        left.grid(row=0, column=0, sticky="nw", padx=(0, 12))

        right = tk.Frame(cols, bg="#1e1e1e")
        right.grid(row=0, column=1, sticky="nw")

        # 검출(arms_detection) 그룹은 없다. 그 노드는 도커 컨테이너로 돌고
        # YOLO on/off 는 ROS 파라미터가 아니라 기동 시 ARMS_MODEL 환경변수로
        # 정해진다 — 패널에서 누를 수 있는 값이 애초에 없다.

        # ── [제어] 축 부호 + PID ────────────────────────────────────────────
        g_pid = group(left, "제어 PID  ·  arms_control")
        sfrm = tk.Frame(g_pid, bg="#1e1e1e")
        sfrm.pack(pady=(0, 2))
        self.roll_btn = tk.Button(sfrm, text="Roll sign: +", width=13, command=self.flip_roll)
        self.roll_btn.grid(row=0, column=0, padx=3)
        self.pitch_btn = tk.Button(sfrm, text="Pitch sign: +", width=13, command=self.flip_pitch)
        self.pitch_btn.grid(row=0, column=1, padx=3)
        # 부호도 노드 값을 거울처럼 보여준다(예전엔 패널 시작값 + 고정이라 노드가 - 여도 + 로 보였다).
        self._track(CTRL_NODE, "control.roll_sign", self._show_roll_sign)
        self._track(CTRL_NODE, "control.pitch_sign", self._show_pitch_sign)

        def make_pid_group(parent, title, defaults, params):
            """(P, I, D) 입력 행 + 적용 버튼. returns (ep, ei, ed) Entry widgets."""
            tk.Label(parent, text=title, fg="#aaaaaa", bg="#1e1e1e",
                     font=("Arial", 10, "bold")).pack(anchor="w", pady=(8, 2))
            row = tk.Frame(parent, bg="#1e1e1e")
            row.pack(anchor="w")
            entries = []
            for col, (lbl, val) in enumerate(zip(["P", "I", "D"], defaults)):
                tk.Label(row, text=lbl, fg="white", bg="#1e1e1e",
                         width=2).grid(row=0, column=col * 2, padx=(4, 0))
                e = tk.Entry(row, width=7, bg="#2e2e2e", fg="white",
                             insertbackground="white", relief="flat")
                e.insert(0, str(val))
                e.grid(row=0, column=col * 2 + 1, padx=(2, 6))
                entries.append(e)
            ep, ei, ed = entries

            def apply():
                try:
                    ros_param_set(params[0], float(ep.get()))
                    ros_param_set(params[1], float(ei.get()))
                    ros_param_set(params[2], float(ed.get()))
                except ValueError:
                    pass

            tk.Button(row, text="적용", bg="#455a64", fg="white",
                      activebackground="#607d8b", relief="flat",
                      command=apply).grid(row=0, column=6, padx=(2, 0))
            for widget, pname in zip(entries, params):
                self._track(CTRL_NODE, pname,
                            lambda v, w=widget: (w.delete(0, "end"), w.insert(0, v)))
            return ep, ei, ed

        self.roll_kp, self.roll_ki, self.roll_kd = make_pid_group(
            g_pid, "Roll PID  [px→deg/s]", [455, 0.0, 3.5],
            ["control.roll_pid.kp", "control.roll_pid.ki", "control.roll_pid.kd"])

        self.pitch_kp, self.pitch_ki, self.pitch_kd = make_pid_group(
            g_pid, "Pitch PID  [px→deg/s]", [455, 0.0, 3.5],
            ["control.pitch_pid.kp", "control.pitch_pid.ki", "control.pitch_pid.kd"])

        # ── [제어] 추력 ─────────────────────────────────────────────────────
        g_thr = group(left, "추력  ·  arms_control")
        tk.Label(g_thr, text="상승 추력 (track_throttle)", fg="#aaaaaa",
                 bg="#1e1e1e", font=("Arial", 9)).pack(anchor="w")
        thr_row = tk.Frame(g_thr, bg="#1e1e1e")
        thr_row.pack(anchor="w")
        self.thr_entry = tk.Entry(thr_row, width=7, bg="#2e2e2e", fg="white",
                                  insertbackground="white", relief="flat")
        self.thr_entry.insert(0, "0.85")   # yaml track_throttle 과 일치 (1.05kg)
        self.thr_entry.grid(row=0, column=0, padx=(4, 6))
        self._track(CTRL_NODE, "control.track_throttle",
                    lambda v: (self.thr_entry.delete(0, "end"),
                               self.thr_entry.insert(0, v)))
        tk.Button(thr_row, text="적용", bg="#455a64", fg="white",
                  activebackground="#607d8b", relief="flat",
                  command=lambda: ros_param_set(
                      "control.track_throttle", float(self.thr_entry.get()))
                  ).grid(row=0, column=1)

        # ── [발사기] launcher ───────────────────────────────────────────────
        #   슬라이더를 드래그하는 동안 실시간으로 노드에 보낸다(같은 값은 합쳐짐).
        #   발사기 노드가 일정 속도로 따라 돌리므로 드론·카메라 화면이 부드럽게 움직인다.
        g_lch = group(left, "견착 발사기  ·  launcher")
        self.lch_status = tk.Label(g_lch, text="발사기: 노드 대기 중…", fg="white",
                                   bg="#455a64", font=("Arial", 10, "bold"), width=30)
        self.lch_status.pack(pady=(0, 4), ipady=3)
        self._lch_drag = set()   # 지금 드래그 중인 슬라이더 (그동안 노드 값으로 되돌리지 않음)
        self._lch_sent = {}      # pname → (마지막으로 보낸 값, 보낸 시각)

        def lch_slider(label, lo, hi, init, pname):
            sc = tk.Scale(g_lch, from_=lo, to=hi, resolution=1, orient="horizontal",
                          length=220, bg="#1e1e1e", fg="white", label=label,
                          highlightthickness=0, troughcolor="#444")
            sc.set(init)
            sc.pack(pady=(2, 0))

            def send_now():
                v = float(sc.get())
                self._lch_sent[pname] = (v, time.monotonic())
                ros_param_set_node(LAUNCHER_NODE, pname, v)

            def send(_e=None):
                # 위젯 바인딩이 Scale 클래스 바인딩보다 먼저 돌아서 지금 읽으면 한 박자
                # 늦은 값이다 → 슬라이더가 값을 갱신한 '다음'(idle)에 읽어서 보낸다.
                sc.after_idle(send_now)

            def press(_e):
                self._lch_drag.add(pname)

            def release(_e):
                self._lch_drag.discard(pname)
                send()

            sc.bind("<ButtonPress-1>", press)
            sc.bind("<B1-Motion>", send)
            sc.bind("<ButtonRelease-1>", release)

            def show(v):
                if pname in self._lch_drag:
                    return
                sent = self._lch_sent.get(pname)
                # 방금 보낸 값과 다른 '옛 결과'가 늦게 도착하면 무시(슬라이더가 튕겨 돌아가지 않게).
                if sent and abs(float(v) - sent[0]) > 0.5 and time.monotonic() - sent[1] < 1.5:
                    return
                sc.set(float(v))
            self._track(LAUNCHER_NODE, pname, show)
            return sc

        self.lch_elev = lch_slider("앙각 [deg]  (90=수직, 0=수평)", 0, 90, 90, "elevation_deg")
        self.lch_az = lch_slider("방위 [deg]  (0=+X, 반시계 +)", -180, 180, 0, "azimuth_deg")

        prow = tk.Frame(g_lch, bg="#1e1e1e")
        prow.pack(pady=(4, 0))
        for deg in (90, 60, 50, 40, 30):
            tk.Button(prow, text=f"{deg}°", width=4, bg="#37474f", fg="white",
                      relief="flat",
                      command=lambda d=deg: (
                          self.lch_elev.set(d),
                          self._lch_sent.__setitem__("elevation_deg", (float(d), time.monotonic())),
                          ros_param_set_node(LAUNCHER_NODE, "elevation_deg", float(d)))
                      ).pack(side="left", padx=2)
        tk.Button(g_lch, text="재장전 (DISARM 상태에서만)", bg="#5d4037", fg="white",
                  relief="flat", command=lambda: self.node.reload_pub.publish(Empty())
                  ).pack(pady=(6, 0), fill="x")

        # ── [표적] referee ──────────────────────────────────────────────────
        g_tgt = group(right, "표적  ·  referee")

        # 표적 종류 토글 (풍선 ⇄ 드론) → referee 'target' 파라미터. 선택된 것만 비행.
        self.target_is_drone = False   # 기본: 풍선
        self.target_btn = tk.Button(g_tgt, text="종류: 풍선", width=20,
                                    font=("Arial", 10, "bold"), bg="#8d6e63", fg="white",
                                    command=self.toggle_target)
        self.target_btn.pack(pady=(0, 6))
        self._track(REFEREE_NODE, "target", self._show_target)

        bbtn = tk.Frame(g_tgt, bg="#1e1e1e")
        bbtn.pack(pady=2)
        self.ball_fly_btn = tk.Button(bbtn, text="비행 시작", width=12,
                                      font=("Arial", 10, "bold"),
                                      command=self.ball_start)
        self.ball_fly_btn.grid(row=0, column=0, padx=3, pady=2)
        self.ball_stop_btn = tk.Button(bbtn, text="정지", width=8,
                                       command=self.ball_stop)
        self.ball_stop_btn.grid(row=0, column=1, padx=3, pady=2)

        self.ball_state_lbl = tk.Label(g_tgt, text="상태: ? (버튼으로 설정)",
                                       fg="white", bg="#555555",
                                       font=("Arial", 10, "bold"), width=26)
        self.ball_state_lbl.pack(pady=(2, 6), ipady=4)
        self._refresh_ball_btns()
        self._track(REFEREE_NODE, "enabled", self._show_enabled)

        # ── 공(표적) 위치 X / Y / Z ─────────────────────────────────────────
        #   발사기를 기울이면 수직 기준으로 세워둔 공이 카메라 밖으로 나간다 → 패널에서
        #   공을 직접 옮기거나 [발사기 조준선에 놓기] 로 조준선 위에 올린다.
        #   드래그 중에도 실시간 전송(같은 값은 ParamBus 가 합침), Gazebo·UI 카메라·rviz 에 바로 반영.
        self._tgt_drag = set()
        self._tgt_sent = {}

        def tgt_slider(label, lo, hi, init, pname, res):
            sc = tk.Scale(g_tgt, from_=lo, to=hi, resolution=res, orient="horizontal",
                          length=180, bg="#1e1e1e", fg="white", label=label,
                          highlightthickness=0, troughcolor="#444")
            sc.set(init)
            sc.pack(pady=(2, 0))

            def send_now():
                v = float(sc.get())
                self._tgt_sent[pname] = (v, time.monotonic())
                ros_param_set_node(REFEREE_NODE, pname, v)

            def send(_e=None):
                sc.after_idle(send_now)   # 슬라이더 값이 갱신된 '다음'에 읽는다

            def press(_e):
                self._tgt_drag.add(pname)

            def release(_e):
                self._tgt_drag.discard(pname)
                send()

            sc.bind("<ButtonPress-1>", press)
            sc.bind("<B1-Motion>", send)
            sc.bind("<ButtonRelease-1>", release)

            def show(v):
                if pname in self._tgt_drag:
                    return
                sent = self._tgt_sent.get(pname)
                if sent and abs(float(v) - sent[0]) > 0.3 and time.monotonic() - sent[1] < 1.5:
                    return
                sc.set(float(v))
            self._track(REFEREE_NODE, pname, show)
            return sc

        self.tgt_x = tgt_slider("공 X [m]  (+X = 방위 0° 방향)", -100, 100, 5, "pos_x", 0.5)
        self.tgt_y = tgt_slider("공 Y [m]  (+Y = 방위 90° 방향)", -100, 100, -5, "pos_y", 0.5)
        self.alt = tgt_slider("공 Z = 타겟 고도 [m]", 5, 100, 30, "alt", 1)
        tk.Button(g_tgt, text="🎯 발사기 조준선에 놓기 (현재 고도 유지)", bg="#00695c", fg="white",
                  relief="flat", command=self.place_on_launcher_axis
                  ).pack(pady=(6, 0), fill="x")

        # 타겟 속도 슬라이더 (근접/먼거리 통일 — 단일 속도)
        self.ball_speed = tk.Scale(g_tgt, from_=0.2, to=40, resolution=0.2, orient="horizontal",
                                   length=180, bg="#1e1e1e", fg="white", label="타겟 속도 [m/s]",
                                   highlightthickness=0, troughcolor="#444")
        self.ball_speed.set(1.6)
        self.ball_speed.pack(pady=(2, 0))
        self._track(REFEREE_NODE, "speed",
                    lambda v: self.ball_speed.set(float(v)))
        self.ball_speed.bind("<ButtonRelease-1>",
                             lambda e: ros_param_set_node(REFEREE_NODE, "speed", float(self.ball_speed.get())))

        # ── [유도] arms_control ─────────────────────────────────────────────
        g_guid = group(right, "유도  ·  arms_control")
        # 0=기본 추적(각속도 PID 추종), 1=PN(비례항법). 기본은 기본 추적.
        self._guidance_mode = 0
        self.guid_btn = tk.Button(g_guid, text="방식: 기본 추적", font=("Arial", 11, "bold"),
                                  bg="#37474f", fg="white", activebackground="#546e7a",
                                  command=self.toggle_guidance)
        self.guid_btn.pack(pady=(0, 4), fill="x")
        self._track(CTRL_NODE, "control.guidance_mode",
                    lambda v: self._set_guidance(int(float(v))))

        # 예측 조준(lead) — 움직이는 공의 미래 위치를 겨냥 (control.lead_gain)
        self.lead = tk.Scale(g_guid, from_=0.0, to=1.5, resolution=0.05, orient="horizontal",
                             length=180, bg="#1e1e1e", fg="white", label="예측 조준 lead (기본 추적)",
                             highlightthickness=0, troughcolor="#444")
        self.lead.set(0.0)
        self.lead.pack(pady=(2, 0))
        self._track(CTRL_NODE, "control.lead_gain",
                    lambda v: self.lead.set(float(v)))
        self.lead.bind("<ButtonRelease-1>",
                       lambda e: ros_param_set("control.lead_gain", float(self.lead.get())))

        self.pn_nav = tk.Scale(g_guid, from_=0, to=525, resolution=5, orient="horizontal",
                               length=180, bg="#1e1e1e", fg="white", label="PN 항법이득 (PN 일때)",
                               highlightthickness=0, troughcolor="#444")
        self.pn_nav.set(175)
        self.pn_nav.pack(pady=(2, 0))
        self._track(CTRL_NODE, "control.pn_nav_gain",
                    lambda v: self.pn_nav.set(float(v)))
        self.pn_nav.bind("<ButtonRelease-1>",
                         lambda e: ros_param_set("control.pn_nav_gain", float(self.pn_nav.get())))
        self.pn_center = tk.Scale(g_guid, from_=0.0, to=350.0, resolution=0.5, orient="horizontal",
                                  length=180, bg="#1e1e1e", fg="white", label="PN 중심유지 (PN 일때)",
                                  highlightthickness=0, troughcolor="#444")
        self.pn_center.set(52.5)
        self.pn_center.pack(pady=(2, 0))
        self._track(CTRL_NODE, "control.pn_center_gain",
                    lambda v: self.pn_center.set(float(v)))
        self.pn_center.bind("<ButtonRelease-1>",
                            lambda e: ros_param_set("control.pn_center_gain", float(self.pn_center.get())))

        self.ibvs_gain = tk.Scale(g_guid, from_=0.0, to=300.0, resolution=5.0, orient="horizontal",
                                  length=180, bg="#1e1e1e", fg="white",
                                  label="IBVS 배리어항 세기 (ibvs_gain, IBVS 일때)",
                                  highlightthickness=0, troughcolor="#444")
        self.ibvs_gain.set(50.0)
        self.ibvs_gain.pack(pady=(2, 0))
        self._track(CTRL_NODE, "control.ibvs_gain",
                    lambda v: self.ibvs_gain.set(float(v)))
        self.ibvs_gain.bind("<ButtonRelease-1>",
                            lambda e: ros_param_set("control.ibvs_gain", float(self.ibvs_gain.get())))

        self.ibvs_center = tk.Scale(g_guid, from_=0.0, to=150.0, resolution=1.0, orient="horizontal",
                                    length=180, bg="#1e1e1e", fg="white",
                                    label="IBVS 중앙유지 (ibvs_center_gain, IBVS 일때)",
                                    highlightthickness=0, troughcolor="#444")
        self.ibvs_center.set(35.0)
        self.ibvs_center.pack(pady=(2, 0))
        self._track(CTRL_NODE, "control.ibvs_center_gain",
                    lambda v: self.ibvs_center.set(float(v)))
        self.ibvs_center.bind("<ButtonRelease-1>",
                              lambda e: ros_param_set("control.ibvs_center_gain", float(self.ibvs_center.get())))

        self.ibvs_los_rate = tk.Scale(g_guid, from_=0.0, to=200.0, resolution=1.0, orient="horizontal",
                                      length=180, bg="#1e1e1e", fg="white",
                                      label="IBVS 리드항 (ibvs_los_rate_gain, IBVS 일때)",
                                      highlightthickness=0, troughcolor="#444")
        self.ibvs_los_rate.set(10.0)
        self.ibvs_los_rate.pack(pady=(2, 0))
        self._track(CTRL_NODE, "control.ibvs_los_rate_gain",
                    lambda v: self.ibvs_los_rate.set(float(v)))
        self.ibvs_los_rate.bind(
            "<ButtonRelease-1>",
            lambda e: ros_param_set("control.ibvs_los_rate_gain", float(self.ibvs_los_rate.get())))

        # 충돌 판정 그룹은 없다. 명중은 Gazebo 접촉 센서가 실제 충돌 형상으로
        # 판정하므로 조절할 값이 없고(예전 hit_radius 는 "빗나가도 명중"을 만들 수
        # 있는 노브였다), 발사 τ 는 control_params.yaml 에서 잡는다.

        self._pull_from_nodes()
        self._poll()

    def _track(self, node, param, apply_fn):
        """시작할 때 노드에서 읽어와 위젯에 반영할 항목을 등록한다."""
        self._pullables.append((node, param, apply_fn))

    def _pull_from_nodes(self):
        """실제 노드 값을 읽어 화면을 맞춘다. 노드가 진실이고 패널은 거울이다.

        예전에는 반대로 패널이 자기 하드코딩 값을 노드로 밀었다(_push_defaults).
        그러면 control_params.yaml 을 고쳐도 패널은 옛 숫자를 그대로 보여주고,
        심지어 '적용' 을 누르는 순간 yaml 값을 옛 숫자로 되돌려버린다.
        실제로 kp 를 1055 로 올려도 패널은 455 를 그리고 있었다.

        노드가 아직 안 떴을 수 있으니 몇 번 재시도한다. tkinter 는 스레드 안전이
        아니라서 위젯을 워커 스레드에서 직접 못 건드리고, root.after 조차
        메인 스레드 전용이다("main thread is not in main loop"). 그래서 결과를
        큐에 넣고 이미 도는 _poll 루프가 꺼내 반영한다.
        """
        def worker():
            pending = list(self._pullables)
            for attempt in range(6):
                if not pending:
                    break
                still = []
                for node, param, apply_fn in pending:
                    raw = self._bus.get_str(node, param)
                    if raw is None:
                        still.append((node, param, apply_fn))
                        continue
                    self._applyq.put((apply_fn, raw))
                pending = still
                if pending:
                    time.sleep(1.0)
            if pending:
                names = ", ".join(f"{n}/{p}" for n, p, _ in pending[:3])
                more = f" 외 {len(pending) - 3}개" if len(pending) > 3 else ""
                text = f"⚠ 노드에서 못 읽음 (표시값 신뢰 X): {names}{more}"
                print(f"[panel] {text}")
                self._applyq.put((self._show_param_status, (text, "#ff8a65")))
            else:
                self._applyq.put((self._show_param_status,
                                  ("파라미터: 노드 값과 동기화됨", "#81c784")))
        threading.Thread(target=worker, daemon=True).start()

    # ---- 파라미터 쓰기 결과 (메인 스레드) ---------------------------------
    def _show_param_status(self, arg):
        text, color = arg
        self.param_lbl.config(text=text, fg=color)

    def _on_param_result(self, r):
        """ParamBus 결과: 노드에서 다시 읽은 '진짜 값'으로 위젯을 맞춘다."""
        node, name, ok, value, msg, more_pending = r
        short = (name.split(".")[-1] if node == CTRL_NODE
                 else f"launcher.{name}" if node == LAUNCHER_NODE
                 else f"referee.{name}")
        if value is not None and not more_pending:
            for n, p, fn in self._pullables:
                if n == node and p == name:
                    self._safe_apply(fn, value)
        if ok:
            self._show_param_status((f"✓ {short} = {value}", "#81c784"))
        else:
            self._show_param_status(
                (f"✗ {short} 적용 실패: {msg}  (노드 현재값: {value})", "#ff5252"))
            print(f"[panel] {node} {name} 적용 실패: {msg} (현재값 {value})")

    @staticmethod
    def _safe_apply(apply_fn, raw):
        try:
            apply_fn(raw)
        except (ValueError, TypeError) as e:
            print(f"[panel] 파라미터 반영 실패: {raw!r} ({e})")

    # 버튼 동작 원칙: 누르면 '의도' 를 즉시 표시(… 표시)하고 큐에 쓰기를 넣는다.
    #   쓰기가 끝나면 _on_param_result 가 노드에서 다시 읽은 값으로 덮어쓴다
    #   = 최종 표시는 항상 노드의 실제 값.
    def flip_roll(self):
        self.roll_sign *= -1.0
        ros_param_set("control.roll_sign", self.roll_sign)
        self.roll_btn.config(text=f"Roll sign: {'+' if self.roll_sign > 0 else '-'} …")

    def flip_pitch(self):
        self.pitch_sign *= -1.0
        ros_param_set("control.pitch_sign", self.pitch_sign)
        self.pitch_btn.config(text=f"Pitch sign: {'+' if self.pitch_sign > 0 else '-'} …")

    def _show_roll_sign(self, v):
        self.roll_sign = 1.0 if float(v) > 0 else -1.0
        self.roll_btn.config(text=f"Roll sign: {'+' if self.roll_sign > 0 else '-'}")

    def _show_pitch_sign(self, v):
        self.pitch_sign = 1.0 if float(v) > 0 else -1.0
        self.pitch_btn.config(text=f"Pitch sign: {'+' if self.pitch_sign > 0 else '-'}")

    # ---- 표적 종류 전환 (풍선 ↔ 드론) → referee 'target' 파라미터 ----
    def toggle_target(self):
        self.target_is_drone = not self.target_is_drone
        if self.target_is_drone:
            self.target_btn.config(text="종류: 드론 …", bg="#37474f")
            ros_param_set_node(REFEREE_NODE, "target", "drone")
        else:
            self.target_btn.config(text="종류: 풍선 …", bg="#8d6e63")
            ros_param_set_node(REFEREE_NODE, "target", "balloon")

    def _show_target(self, v):
        self.target_is_drone = (str(v).strip().lower() == "drone")
        if self.target_is_drone:
            self.target_btn.config(text="종류: 드론", bg="#37474f")
        else:
            self.target_btn.config(text="종류: 풍선", bg="#8d6e63")

    def _show_enabled(self, v):
        self.ball_flying = str(v).strip().lower() in ("true", "1")
        self._refresh_ball_btns()

    # ---- 유도 방식 전환 (기본 추적 → PN → IBVS → ...) ----
    _GUID_LABELS = {
        0: ("방식: 기본 추적",      "#37474f"),
        1: ("방식: PN(비례항법)",   "#6a1b9a"),
        2: ("방식: IBVS(공선유도)", "#00695c"),
    }

    def _apply_guid_label(self, pending=False):
        text, color = self._GUID_LABELS.get(self._guidance_mode, self._GUID_LABELS[0])
        self.guid_btn.config(text=text + (" …" if pending else ""), bg=color)

    def _set_guidance(self, mode):
        """버튼 표시만 갱신 (노드로 쓰지 않는다 — 노드에서 읽어온 값 반영용)."""
        self._guidance_mode = mode if mode in self._GUID_LABELS else 0
        self._apply_guid_label()

    def toggle_guidance(self):
        self._guidance_mode = (self._guidance_mode + 1) % 3
        ros_param_set("control.guidance_mode", self._guidance_mode)
        self._apply_guid_label(pending=True)   # 노드 확인 전까지 '…' 표시

    def place_on_launcher_axis(self):
        """발사기 앙각/방위를 노드에서 읽어, 조준선(튜브 축)과 현재 고도가 만나는 점에 공을 놓는다.

        드론 카메라 광축 = 기체 +Z = 튜브 축이라 공이 화면 정중앙에 온다.
        앙각이 너무 낮아(≈수평) 고도와 안 만나면 축을 따라 30m 앞에 놓고 고도도 같이 맞춘다.
        """
        def worker():
            el = self._bus.get_str(LAUNCHER_NODE, "elevation_deg")
            az = self._bus.get_str(LAUNCHER_NODE, "azimuth_deg")
            if el is None or az is None:
                self._applyq.put((self._show_param_status,
                                  ("✗ 발사기 각도를 못 읽음 (launcher 노드 확인)", "#ff5252")))
                return
            el, az = float(el), float(az)
            d = G.axis(el, az)
            alt = float(self.alt.get())
            if d[2] > 0.09 and alt > G.PIVOT_Z:          # 앙각 ≳ 5°: 현재 고도에서 만난다
                s = (alt - G.PIVOT_Z) / d[2]
            else:                                        # 거의 수평: 30m 앞, 고도도 맞춤
                s = 30.0
                alt = max(5.0, G.PIVOT_Z + s * d[2])
                ros_param_set_node(REFEREE_NODE, "alt", float(round(alt)))
            x = round(G.PIVOT_X + s * d[0], 1)
            y = round(G.PIVOT_Y + s * d[1], 1)
            ros_param_set_node(REFEREE_NODE, "pos_x", x)
            ros_param_set_node(REFEREE_NODE, "pos_y", y)
            self._applyq.put((self._show_param_status,
                              (f"🎯 조준선에 공 배치: ({x:+.1f}, {y:+.1f}, {alt:.0f}m) "
                               f"· 앙각 {el:.0f}° 방위 {az:.0f}° · 거리 {s:.0f}m", "#81c784")))
        threading.Thread(target=worker, daemon=True).start()

    def ball_start(self):
        # enabled=true → referee 가 정지했던 자리(t)에서 재개
        ros_param_set_node(REFEREE_NODE, "enabled", True)
        self.ball_flying = True
        self._refresh_ball_btns()

    def ball_stop(self):
        ros_param_set_node(REFEREE_NODE, "enabled", False)
        self.ball_flying = False
        self._refresh_ball_btns()

    def _refresh_ball_btns(self):
        flying = getattr(self, "ball_flying", None)
        if flying is True:
            self.ball_state_lbl.config(text="타겟 상태: 비행 중 (움직임)", bg="#9c27b0")
            self.ball_fly_btn.config(bg="#9c27b0", fg="white", relief="sunken")
            self.ball_stop_btn.config(bg="#333333", fg="#cccccc", relief="raised")
        elif flying is False:
            self.ball_state_lbl.config(text="타겟 상태: 정지 (제자리, 재개가능)", bg="#2e7d32")
            self.ball_fly_btn.config(bg="#333333", fg="#cccccc", relief="raised")
            self.ball_stop_btn.config(bg="#2e7d32", fg="white", relief="sunken")
        else:
            self.ball_state_lbl.config(text="타겟 상태: ? (버튼을 눌러 설정)", bg="#555555")
            self.ball_fly_btn.config(bg="#9c27b0", fg="white", relief="raised")
            self.ball_stop_btn.config(bg="#333333", fg="#cccccc", relief="raised")

    def reset_episode(self):
        """Gazebo 월드와 PX4 상태를 함께 새 실험 상태로 되돌린다."""
        if self._reset_in_progress:
            return
        if not self.node.reset_client.wait_for_service(timeout_sec=0.2):
            self.info_lbl.config(text="RESET 실패: sitl_reset_manager 미기동")
            return
        self._reset_in_progress = True
        self.reset_btn.config(state="disabled", text="RESET 중…")
        future = self.node.reset_client.call_async(Trigger.Request())

        def done(f):
            try:
                response = f.result()
                text = ("RESET 완료: PX4 연결 대기" if response.success
                        else f"RESET 실패: {response.message}")
            except Exception as exc:
                text = f"RESET 실패: {exc}"
            self._applyq.put((self._finish_reset, text))
        future.add_done_callback(done)

    def _finish_reset(self, text):
        self._reset_in_progress = False
        self.reset_btn.config(state="normal", text="실험 RESET (PX4 + 월드)")
        self.info_lbl.config(text=text)

    def _poll(self):
        # 노드에서 읽어온 파라미터를 메인 스레드에서 위젯에 반영
        try:
            while True:
                apply_fn, raw = self._applyq.get_nowait()
                self._safe_apply(apply_fn, raw)
        except queue.Empty:
            pass
        try:
            while True:
                msg = self.q.get_nowait()
                if isinstance(msg, String):
                    launched = msg.data.startswith("발사됨") or msg.data.startswith("분리됨")
                    self.lch_status.config(
                        text=msg.data,
                        bg="#b71c1c" if launched else
                           ("#2e7d32" if msg.data.startswith("장전됨") else "#455a64"))
                elif isinstance(msg, MissionState):
                    color = STATE_COLOR.get(msg.state, "#888888")
                    self.state_lbl.config(text=msg.state or "—", bg=color)
                    self.info_lbl.config(
                        text=f"error: {msg.error_x:+.2f}, {msg.error_y:+.2f}\n"
                             f"lock: {msg.lock_elapsed_sec:.1f}s")
        except queue.Empty:
            pass
        self.state_lbl.after(50, self._poll)


def main(args=None):
    rclpy.init(args=args)
    q = queue.Queue()
    node = PanelNode(q)
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()

    root = tk.Tk()
    PanelGUI(root, node, q)

    try:
        root.mainloop()
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
