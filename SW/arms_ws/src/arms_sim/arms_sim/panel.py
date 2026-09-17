#!/usr/bin/env python3
"""
panel.py — A.R.M.S. SITL 튜닝/심판 콘솔 (tkinter GUI)  [arms_sim]

SITL 전용 개발 콘솔. /arms/command 는 발행하지 않는다 (그건 가상 조종기 =
arms_command 노드 담당). 여기서 하는 일은 전부 파라미터/서비스 세팅뿐:
  - 미션 상태 표시 (IDLE/SEARCH/LOCK/TRACK/FIRE/RTL) — 읽기전용 구독
  - Roll/Pitch 부호, PID, τ, 추력, PN 게인 등 → arms_control_node 파라미터
  - 표적(풍선/드론) 비행·정지·고도·속도 → referee 파라미터
  - RESET → Gazebo world reset + PX4 SITL 재시작 (Gazebo 프로세스는 유지)

실행: ros2 run arms_sim panel
"""

import queue
import subprocess
import threading
import time
import tkinter as tk

import rclpy
from rclpy.node import Node
from arms_msgs.msg import MissionState
from std_srvs.srv import Trigger

CTRL_NODE    = "/arms_control_node"
REFEREE_NODE = "/balloon_referee"

WORLD = "arms_sitl"
DRONE = "arms_drone_0"
BALL  = "target_ball"

STATE_COLOR = {
    "IDLE":   "#888888",
    "SEARCH": "#f5a623",
    "LOCK":   "#f57c00",
    "TRACK":  "#d0021b",
    "FIRE":   "#ff1744",
    "RTL":    "#2979ff",
}

def _bg(fn):
    threading.Thread(target=fn, daemon=True).start()


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


def ros_param_set(name, value):
    _bg(lambda: subprocess.run(
        ["ros2", "param", "set", CTRL_NODE, name, str(value)],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL))


def ros_param_set_node(node, name, value):
    _bg(lambda: subprocess.run(
        ["ros2", "param", "set", node, name, str(value)],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL))


def ros_param_get(node, name, timeout=4.0):
    """노드에서 파라미터 값을 읽어 문자열로 돌려준다. 실패하면 None.

    출력 형식이 "Double value is: 455.0" 같은 꼴이라 접두어를 떼어낸다.
    (--hide-type 은 배포판마다 있는지 달라서 의존하지 않는다.)
    """
    try:
        r = subprocess.run(["ros2", "param", "get", node, name],
                           capture_output=True, text=True, timeout=timeout)
    except (subprocess.TimeoutExpired, OSError):
        return None
    if r.returncode != 0:
        return None
    out = r.stdout.strip()
    if " value is: " in out:
        out = out.split(" value is: ", 1)[1]
    elif " values are: " in out:
        out = out.split(" values are: ", 1)[1]
    return out.strip() or None


class PanelNode(Node):
    """콘솔 노드 — 미션 상태만 구독(표시용). /arms/command 는 발행하지 않는다."""

    def __init__(self, q):
        super().__init__("arms_panel_node")
        self.q = q
        self.create_subscription(MissionState, "/arms/mission_state", self._cb_state, 10)
        self.reset_client = self.create_client(Trigger, "/arms/sitl/reset")

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
        # LAUNCH/arm/kill/mode 는 가상 조종기(arms_command)로 이동 — /arms/command 발행자는
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

        # ── [표적] referee ──────────────────────────────────────────────────
        g_tgt = group(right, "표적  ·  referee")

        # 표적 종류 토글 (풍선 ⇄ 드론) → referee 'target' 파라미터. 선택된 것만 비행.
        self.target_is_drone = False   # 기본: 풍선
        self.target_btn = tk.Button(g_tgt, text="종류: 풍선", width=20,
                                    font=("Arial", 10, "bold"), bg="#8d6e63", fg="white",
                                    command=self.toggle_target)
        self.target_btn.pack(pady=(0, 6))

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

        self.alt = tk.Scale(g_tgt, from_=5, to=100, resolution=1, orient="horizontal",
                            length=180, bg="#1e1e1e", fg="white", label="타겟 고도 [m]",
                            highlightthickness=0, troughcolor="#444")
        self.alt.set(42)
        self.alt.pack(pady=(2, 0))
        self._track(REFEREE_NODE, "alt",
                    lambda v: self.alt.set(float(v)))
        self.alt.bind("<ButtonRelease-1>",
                      lambda e: ros_param_set_node(REFEREE_NODE, "alt", float(self.alt.get())))

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
                    raw = ros_param_get(node, param)
                    if raw is None:
                        still.append((node, param, apply_fn))
                        continue
                    self._applyq.put((apply_fn, raw))
                pending = still
                if pending:
                    time.sleep(1.0)
            if pending:
                names = ", ".join(f"{n}{p}" for n, p, _ in pending[:3])
                print(f"[panel] 파라미터를 못 읽음 (노드 미기동?): {names}"
                      f"{' 외 %d개' % (len(pending) - 3) if len(pending) > 3 else ''}")
        threading.Thread(target=worker, daemon=True).start()

    @staticmethod
    def _safe_apply(apply_fn, raw):
        try:
            apply_fn(raw)
        except (ValueError, TypeError) as e:
            print(f"[panel] 파라미터 반영 실패: {raw!r} ({e})")

    def flip_roll(self):
        self.roll_sign *= -1.0
        ros_param_set("control.roll_sign", self.roll_sign)
        self.roll_btn.config(text=f"Roll sign: {'+' if self.roll_sign > 0 else '-'}")

    def flip_pitch(self):
        self.pitch_sign *= -1.0
        ros_param_set("control.pitch_sign", self.pitch_sign)
        self.pitch_btn.config(text=f"Pitch sign: {'+' if self.pitch_sign > 0 else '-'}")

    # ---- 표적 종류 전환 (풍선 ↔ 드론) → referee 'target' 파라미터 ----
    def toggle_target(self):
        self.target_is_drone = not self.target_is_drone
        if self.target_is_drone:
            self.target_btn.config(text="종류: 드론", bg="#37474f")
            ros_param_set_node(REFEREE_NODE, "target", "drone")
        else:
            self.target_btn.config(text="종류: 풍선", bg="#8d6e63")
            ros_param_set_node(REFEREE_NODE, "target", "balloon")

    # ---- 유도 방식 전환 (기본 추적 ↔ PN) ----
    def _set_guidance(self, mode):
        """버튼 표시만 갱신 (노드로 쓰지 않는다 — 읽어온 값 반영용)."""
        self._guidance_mode = 1 if mode == 1 else 0
        if self._guidance_mode == 1:
            self.guid_btn.config(text="방식: PN(비례항법)", bg="#6a1b9a")
        else:
            self.guid_btn.config(text="방식: 기본 추적", bg="#37474f")

    def toggle_guidance(self):
        self._guidance_mode = 1 if self._guidance_mode == 0 else 0
        ros_param_set("control.guidance_mode", self._guidance_mode)
        if self._guidance_mode == 1:
            self.guid_btn.config(text="방식: PN(비례항법)", bg="#6a1b9a")
        else:
            self.guid_btn.config(text="방식: 기본 추적", bg="#37474f")

    def ball_start(self):
        # enabled=true → referee 가 정지했던 자리(t)에서 재개
        ros_param_set_node(REFEREE_NODE, "enabled", "true")
        self.ball_flying = True
        self._refresh_ball_btns()

    def ball_stop(self):
        ros_param_set_node(REFEREE_NODE, "enabled", "false")
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
                if isinstance(msg, MissionState):
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
