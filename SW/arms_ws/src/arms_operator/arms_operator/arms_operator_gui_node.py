#!/usr/bin/env python3
"""
arms_operator_gui_node.py — A.R.M.S. SITL 가상 운용자 입력 (tkinter GUI)

실기체의 물리 조종기(ESP32 → arms_operator_hardware_node)에 대응하는 SITL 쌍둥이.
오직 operator/input (OperatorInput)만 발행한다:
  - 스틱 드래그 → roll/pitch/throttle/yaw
  - MODE/RTL/ARM 버튼 → manual_mode/rtl_switch/arm_switch
  - LAUNCH 버튼 → launch_request (순간 눌림)

튜닝/심판 콘솔은 arms_sim 패키지의 panel 노드로 분리됨.
(발행자는 환경별로 하나뿐 — 같은 토픽에 두 입력을 동시에 연결하지 않는다.)

실행: ros2 run arms_operator arms_operator_gui_node
"""

import math
import threading
import tkinter as tk

import rclpy
from rclpy.node import Node
from arms_msgs.msg import OperatorInput

# 스틱 캔버스 크기 (크게)
STICK_SIZE = 120
STICK_R    = 52
DOT_R      = 12


class ControllerNode(Node):
    """가상 조종기 노드 — 상대 토픽 operator/input만 발행."""

    def __init__(self):
        super().__init__("arms_operator_gui_node")
        self._input_pub = self.create_publisher(OperatorInput, "operator/input", 10)
        self._axes = [0.0, 0.0, 0.0, 0.0]
        # arm, manual mode, launch, RTL 상태를 GUI에서 유지한다.
        self._buttons = [0, 0, 0, 0, 0, 0]
        self.create_timer(0.05, self._publish_input)  # 20 Hz

    def _publish_input(self):
        msg = OperatorInput()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.yaw, msg.throttle, msg.roll, msg.pitch = self._axes
        msg.arm_switch = bool(self._buttons[1])
        msg.manual_mode = bool(self._buttons[2])
        msg.launch_request = bool(self._buttons[3])
        msg.rtl_switch = bool(self._buttons[4])
        msg.input_lost = False
        self._input_pub.publish(msg)

    def set_axis(self, idx, value):
        self._axes[idx] = float(value)

    def toggle_button(self, idx):
        self._buttons[idx] ^= 1
        return self._buttons[idx]

    def set_button(self, idx, val):
        self._buttons[idx] = 1 if val else 0
        return self._buttons[idx]

    def request_launch(self):
        self._buttons[3] = 1

    def release_launch(self):
        self._buttons[3] = 0


class ControllerGUI:
    """가상 조종기 창 — 스틱 2개 + 모드/RTL/암 버튼 + LAUNCH."""

    def __init__(self, root, node):
        self.root = root
        self.node = node

        root.title("RC CONTROLLER")
        root.configure(bg="#1e1e1e")
        # 내용 자연 높이(reqheight)가 387 px 이다. 560 이던 시절에는 소제목이
        # 있었고, 그걸 뺀 뒤로는 LAUNCH 아래로 187 px 가 빈 채였다.
        root.geometry("460x390")

        tk.Label(root, text="RC CONTROLLER", fg="#dddddd", bg="#1e1e1e",
                 font=("Arial", 22, "bold")).pack(pady=(14, 6))

        # ── 스틱 2개 ──────────────────────────────────────────────────────────
        stick_frm = tk.Frame(root, bg="#1e1e1e")
        stick_frm.pack(pady=12)

        # 왼쪽 스틱 (ax0=X=yaw, ax1=Y=throttle)
        #   throttle 초기값 = 최하단(-1.0). 실제 조종기와 같은 관례이고, 켜자마자
        #   ARM 이 된다 — arms_control_node 의 pre-arm 확인이
        #   joy_axes_[1] <= prearm_throttle_max_(-0.85) 를 요구하기 때문이다.
        #
        #   ★ 대가: 자동→수동 인계가 위험하다. 수동 모드의 PX4 는 Altitude 라
        #     스틱 중앙이 "고도 유지"인데, 최하단에서 인계받으면 그 순간 하강 명령이
        #     된다. 비행 중 수동으로 넘길 거면 넘기기 전에 스틱을 중앙으로 올려둘 것.
        #     (예전엔 이걸 피하려고 중앙에서 시작했는데, 그러면 매번 ARM 전에
        #      throttle 을 한 번 바닥까지 내려야 해서 "ARM DENIED" 를 자주 봤다.)
        self._thr_hold = -1.0
        self.node.set_axis(1, -1.0)
        lfrm = tk.Frame(stick_frm, bg="#1e1e1e")
        lfrm.grid(row=0, column=0, padx=24)
        tk.Label(lfrm, text="L-STICK", fg="#999999", bg="#1e1e1e",
                 font=("Arial", 11, "bold")).pack()
        self.l_canvas = tk.Canvas(lfrm, width=STICK_SIZE, height=STICK_SIZE,
                                  bg="#2a2a2a", highlightthickness=0, cursor="crosshair")
        self.l_canvas.pack()
        tk.Label(lfrm, text="yaw, throttle", fg="#666666", bg="#1e1e1e",
                 font=("Arial", 8)).pack()
        self._draw_stick_bg(self.l_canvas)
        self.l_dot = self.l_canvas.create_oval(
            *self._dot_coords(0, self._thr_hold), fill="#e53935", outline="")
        self._bind_throttle_stick(self.l_canvas, self.l_dot, 0, 1)

        # 오른쪽 스틱 (ax2=X=roll, ax3=Y=pitch)
        rfrm = tk.Frame(stick_frm, bg="#1e1e1e")
        rfrm.grid(row=0, column=1, padx=24)
        tk.Label(rfrm, text="R-STICK", fg="#999999", bg="#1e1e1e",
                 font=("Arial", 11, "bold")).pack()
        self.r_canvas = tk.Canvas(rfrm, width=STICK_SIZE, height=STICK_SIZE,
                                  bg="#2a2a2a", highlightthickness=0, cursor="crosshair")
        self.r_canvas.pack()
        tk.Label(rfrm, text="roll, pitch", fg="#666666", bg="#1e1e1e",
                 font=("Arial", 8)).pack()
        self._draw_stick_bg(self.r_canvas)
        self.r_dot = self.r_canvas.create_oval(
            *self._dot_coords(0, 0), fill="#e53935", outline="")
        self._bind_stick(self.r_canvas, self.r_dot, 2, 3)

        # ── 스위치 + LAUNCH ───────────────────────────────────────────────────
        #   한 grid 에 같이 넣는다. 3개 버튼과 그 아래 LAUNCH 의 폭을 맞추려면
        #   같은 컬럼 격자를 공유해야 한다(uniform + columnspan). 버튼에 width= 를
        #   주면 글자수 기준이라 폭이 제각각 되므로 쓰지 않고 sticky="ew" 로 늘린다.
        btn_frm = tk.Frame(root, bg="#1e1e1e")
        btn_frm.pack(pady=(8, 14), fill="x", padx=28)
        for c in range(3):
            btn_frm.grid_columnconfigure(c, weight=1, uniform="sw")

        # 순서: 모드 / RTL(GPS Rescue) / 암-디스암
        self.mode_btn = tk.Button(btn_frm, text="모드: 자동", height=2,
                                  font=("Arial", 14, "bold"), bg="#444444", fg="#cccccc",
                                  activebackground="#2e7d32", command=self._toggle_mode)
        self.mode_btn.grid(row=0, column=0, padx=(0, 3), sticky="ew")
        self.rtl_btn = tk.Button(btn_frm, text="RTL", height=2,
                                  font=("Arial", 14, "bold"), bg="#444444", fg="#cccccc",
                                  activebackground="#f57c00", command=self._toggle_rtl)
        self.rtl_btn.grid(row=0, column=1, padx=3, sticky="ew")
        self.arm_btn = tk.Button(btn_frm, text="DISARM", height=2,
                                 font=("Arial", 14, "bold"), bg="#444444", fg="#cccccc",
                                 activebackground="#1565c0", command=self._toggle_arm)
        self.arm_btn.grid(row=0, column=2, padx=(3, 0), sticky="ew")

        self.launch_btn = tk.Button(btn_frm, text="LAUNCH", height=2,
                                    font=("Arial", 17, "bold"), bg="#d0021b", fg="white",
                                    activebackground="#ff1744", command=self._on_launch)
        self.launch_btn.grid(row=1, column=0, columnspan=3, pady=(6, 0), sticky="ew")

    # ---- 스위치 핸들러 (토글 + 색/텍스트 갱신) ----
    def _toggle_rtl(self):
        on = self.node.toggle_button(4)
        self.rtl_btn.config(text="■ RTL ACTIVE" if on else "RTL",
                            bg="#e65100" if on else "#444444",
                            fg="white" if on else "#cccccc")

    def _toggle_arm(self):
        on = self.node.toggle_button(1)
        self.arm_btn.config(text="● ARM" if on else "DISARM",
                            bg="#d0021b" if on else "#444444",
                            fg="white" if on else "#cccccc")

    def _toggle_mode(self):
        on = self.node.toggle_button(2)
        self.mode_btn.config(text="모드: 수동" if on else "모드: 자동",
                             bg="#1565c0" if on else "#444444",
                             fg="white" if on else "#cccccc")

    def _on_launch(self):
        # 순간 눌림: buttons[3]=1 잠깐 → 300ms 후 해제 (arms_control LOCK→TRACK 트리거)
        self.node.request_launch()
        self.root.after(300, self.node.release_launch)

    # ---- 스틱 바인딩/그리기 ----
    def _bind_throttle_stick(self, canvas, dot_id, ax_x, ay_thr):
        """왼쪽 스틱: 가로(yaw)는 놓으면 중앙 복귀, 세로(throttle)는 그 자리 유지."""
        def _move(event):
            cx = cy = STICK_SIZE // 2
            dx = max(-1.0, min(1.0, (event.x - cx) / STICK_R))
            dy = max(-1.0, min(1.0, (cy - event.y) / STICK_R))
            self.node.set_axis(ax_x, dx)
            self.node.set_axis(ay_thr, dy)
            self._thr_hold = dy
            canvas.coords(dot_id, *self._dot_coords(dx, dy))

        def _release(event):
            self.node.set_axis(ax_x, 0.0)   # yaw 만 중앙 복귀
            canvas.coords(dot_id, *self._dot_coords(0.0, self._thr_hold))

        canvas.bind("<Button-1>", _move)
        canvas.bind("<B1-Motion>", _move)
        canvas.bind("<ButtonRelease-1>", _release)

    def _bind_stick(self, canvas, dot_id, ax_idx, ay_idx):
        """오른쪽 스틱: 놓으면 중앙 복귀."""
        def _move(event):
            cx = cy = STICK_SIZE // 2
            dx = (event.x - cx) / STICK_R
            dy = (cy - event.y) / STICK_R
            mag = math.hypot(dx, dy)
            if mag > 1.0:
                dx /= mag
                dy /= mag
            self.node.set_axis(ax_idx, dx)
            self.node.set_axis(ay_idx, dy)
            canvas.coords(dot_id, *self._dot_coords(dx, dy))

        def _release(event):
            self.node.set_axis(ax_idx, 0.0)
            self.node.set_axis(ay_idx, 0.0)
            canvas.coords(dot_id, *self._dot_coords(0.0, 0.0))

        canvas.bind("<Button-1>", _move)
        canvas.bind("<B1-Motion>", _move)
        canvas.bind("<ButtonRelease-1>", _release)

    def _draw_stick_bg(self, canvas):
        cx = cy = STICK_SIZE // 2
        r = STICK_R
        canvas.create_oval(cx - r, cy - r, cx + r, cy + r,
                           outline="#555555", width=1, fill="#1a1a1a")
        canvas.create_line(cx - r, cy, cx + r, cy, fill="#444444")
        canvas.create_line(cx, cy - r, cx, cy + r, fill="#444444")

    def _dot_coords(self, ax, ay):
        cx = cy = STICK_SIZE // 2
        x = cx + ax * STICK_R
        y = cy - ay * STICK_R
        return x - DOT_R, y - DOT_R, x + DOT_R, y + DOT_R


def main(args=None):
    rclpy.init(args=args)
    node = ControllerNode()
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()

    root = tk.Tk()
    ControllerGUI(root, node)
    try:
        root.mainloop()
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
