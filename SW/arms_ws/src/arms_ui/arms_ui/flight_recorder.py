#!/usr/bin/env python3
"""비행 로그 녹화기 (auto_save:=true 일 때 arms.launch.py / arms_sitl.launch.py 가 기동).

조종기 ARM 스위치를 올리면(DOWN→UP) record_topics(UI/원본 영상 + 검출 raw/필터/진단 + IBVS 디버그
+ 제어·CRSF·GPS·배터리 + 상태/조종)를 rosbag 으로 녹화하기 시작하고, 내리면 멈추고 저장한다.
모드(auto/manual)·상태머신·kill 과는 무관하게 ARM 스위치 하나로만 결정한다.
저장 포맷은 sqlite3(항상 사용 가능). mcap 은 젯슨에 플러그인이 없어 쓰지 않는다.
  · 시작은 상승엣지에서만 → 녹화기가 뜰 때 이미 ARM 이 위였다면 한 번 내렸다 올려야 녹화된다.
  · 저장 위치: ~/arms_flight_logs_ibvs/<YYYYmmdd_HHMMSS>/ (비행 시작 일시). 이 폴더 안에
    bag/(rosbag) + record.log(녹화 프로세스 로그) + manifest.json + control_params.yaml 사본을 함께 담는다.
  · detections_raw = 칼만 적용 전 원시 검출 → 추후 KF 파라미터 오프라인 재현/튜닝용.
  · tag 파라미터를 주면 폴더명 뒤에 붙는다 (SITL: <일시>_sitl).

/arms/operator/input의 arm_switch 상승/하강 엣지로 녹화를 시작·종료한다.
"""
import os
import json
import signal
import shutil
import subprocess
from datetime import datetime
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from arms_msgs.msg import OperatorInput


class FlightRecorder(Node):
    def require_parameter(self, name):
        self.declare_parameter(name)
        value = self.get_parameter(name).value
        if value is None:
            raise RuntimeError(
                f"Required parameter '{name}' is missing for "
                f"{self.get_fully_qualified_name()}")
        return value

    def __init__(self):
        super().__init__("flight_recorder")
        # 녹화할 토픽 목록. 영상(UI) + 검출 raw/필터/진단 + CRSF 송·수신 + 상태/조종.
        #   detections_raw = 칼만 적용 전 원시 측정값(추후 KF 튜닝 재현용).
        #   ibvs_debug=유도 각 항(오차/LOS각속도/z1·배리어/항별 기여/스케줄링/명령·추력) 스냅샷.
        #   crsf_tx=송신 채널, crsf_rx=수신 통합 텔레메트리, attitude=기체 자세,
        #   battery=배터리 전용 토픽이다.
        #   대부분 구독자 있을 때만 발행되는 토픽이라, 이 녹화기가 구독하면 자동으로 켜진다.
        self.require_parameter("record_topics")
        self.require_parameter("save_dir")
        # 저장 포맷. 젯슨엔 mcap 플러그인이 없어 sqlite3 로 고정(항상 사용 가능).
        self.require_parameter("storage")
        # 폴더명 접미사 (<일시>_<tag>). 실기체 기록과 구분용 — SITL 런치는 "sitl".
        self.require_parameter("tag")
        # 런치가 실제로 해석한 인자와 사용한 제어 설정의 원본 경로. 비행 시작 시
        # manifest.json 및 control_params.yaml 사본으로 남긴다.
        self.require_parameter("manifest.control_params_path")
        for name in (
                "px4", "px4_dir", "rviz", "detection", "det_model", "auto_save",
                "crsf_port", "start_detection", "model", "fullscreen", "debug", "cv_debug"):
            self.require_parameter(f"manifest.arg_{name}")
        self._topics = list(self.get_parameter("record_topics").value)
        self._save_dir = os.path.expanduser(self.get_parameter("save_dir").value)
        os.makedirs(self._save_dir, exist_ok=True)

        self._proc = None        # 실행 중인 'ros2 bag record' 프로세스
        self._flight_dir = None  # 이번 비행 폴더 (~/arms_flight_logs_ibvs/<일시>/)
        self._bag_dir = None     # 그 안의 rosbag (<일시>/bag)
        self._log_path = None    # 자식 stdout/stderr 로그 파일 경로
        self._log_fh = None
        self._health_timer = None
        self._arm = 0            # 직전 ARM 스위치 (command buttons[1]) — 상승엣지 판정용

        best = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                          history=HistoryPolicy.KEEP_LAST, depth=1)
        self.create_subscription(OperatorInput, "/arms/operator/input", self._cb_cmd, best)
        self.get_logger().info(
            "flight_recorder ready — ARM 스위치 올리면 녹화 시작, "
            f"내리면 종료. 저장: {self._save_dir}")

    def _recording(self) -> bool:
        return self._proc is not None and self._proc.poll() is None

    def _cb_cmd(self, msg: OperatorInput):
        arm = int(msg.arm_switch)
        if arm and not self._arm and not self._recording():
            self._start()                      # ARM 올림 → 녹화 시작
        elif not arm and self._recording():
            self._stop("ARM 스위치 OFF")        # ARM 내림 → 녹화 종료·저장
        self._arm = arm

    def _start(self):
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        tag = str(self.get_parameter("tag").value).strip()
        if tag:
            ts += f"_{tag}"
        # 이번 비행 결과를 한 폴더에 모은다: <일시>/bag(rosbag) + <일시>/record.log.
        self._flight_dir = os.path.join(self._save_dir, ts)
        os.makedirs(self._flight_dir, exist_ok=True)
        self._bag_dir = os.path.join(self._flight_dir, "bag")   # ros2 bag record 가 새로 만든다
        self._log_path = os.path.join(self._flight_dir, "record.log")
        self._write_manifest()
        storage = str(self.get_parameter("storage").value).strip().lower() or "sqlite3"
        # ros2 bag record 는 -o 경로가 없어야 새로 만든다. 그룹세션으로 띄워 SIGINT 를 그룹째 보낸다.
        cmd = ["ros2", "bag", "record", "--storage", storage, "-o", self._bag_dir, *self._topics]
        try:
            self._log_fh = open(self._log_path, "wb")
            self._proc = subprocess.Popen(
                cmd, start_new_session=True,
                stdout=self._log_fh, stderr=subprocess.STDOUT)
            self.get_logger().info(
                f"[REC] 녹화 시작 → {self._flight_dir}  (storage={storage}, {len(self._topics)}개 토픽)")
            # 플러그인 부재 등으로 곧바로 죽는지 1초 뒤 1회 점검(조용한 실패 방지).
            self._health_timer = self.create_timer(1.0, self._check_started)
        except Exception as e:
            self.get_logger().error(f"[REC] 녹화 시작 실패: {e}")
            self._close_log()
            self._proc = None

    def _git_commit(self):
        """실행 환경에서 저장소 HEAD를 찾는다. 실패해도 녹화는 계속한다."""
        config_path = str(self.get_parameter("manifest.control_params_path").value).strip()
        candidates = [Path.cwd(), Path(__file__).resolve()]
        arms_sw = os.environ.get("ARMS_SW", "").strip()
        if arms_sw:
            candidates.insert(0, Path(arms_sw))
        if config_path:
            candidates.append(Path(config_path).resolve())
        for candidate in candidates:
            start = candidate if candidate.is_dir() else candidate.parent
            for directory in (start, *start.parents):
                try:
                    result = subprocess.run(
                        ["git", "-C", str(directory), "rev-parse", "HEAD"],
                        capture_output=True, text=True, timeout=2, check=False)
                    if result.returncode == 0:
                        return result.stdout.strip()
                except (OSError, subprocess.TimeoutExpired):
                    break
        return "unavailable"

    def _write_manifest(self):
        """각 비행 폴더에 재현에 필요한 실행 메타데이터와 제어 설정을 저장한다."""
        args = {}
        for name in (
                "px4", "px4_dir", "rviz", "detection", "det_model", "auto_save",
                "crsf_port", "start_detection", "model", "fullscreen", "debug", "cv_debug"):
            value = str(self.get_parameter(f"manifest.arg_{name}").value)
            if value:
                args[name] = value
        config_source = str(self.get_parameter("manifest.control_params_path").value).strip()
        config_copy = None
        if config_source and os.path.isfile(config_source):
            try:
                config_copy = "control_params.yaml"
                shutil.copy2(config_source, os.path.join(self._flight_dir, config_copy))
            except OSError as e:
                self.get_logger().warn(f"[REC] control_params.yaml 사본 저장 실패: {e}")
                config_copy = None
        manifest = {
            "started_at": datetime.now().isoformat(timespec="seconds"),
            "git_commit": self._git_commit(),
            "launch_arguments": args,
            "control_params_source": config_source or None,
            "control_params_copy": config_copy,
            "record_topics": self._topics,
        }
        try:
            with open(os.path.join(self._flight_dir, "manifest.json"), "w", encoding="utf-8") as f:
                json.dump(manifest, f, ensure_ascii=False, indent=2)
                f.write("\n")
        except OSError as e:
            self.get_logger().warn(f"[REC] manifest 저장 실패: {e}")

    def _check_started(self):
        if self._health_timer is not None:
            self._health_timer.cancel()
            self._health_timer = None
        if self._proc is not None and self._proc.poll() is not None:
            tail = ""
            try:
                with open(self._log_path, "r", errors="replace") as f:
                    tail = f.read()[-600:].strip()
            except Exception:
                pass
            self.get_logger().error(
                "[REC] 녹화가 즉시 종료됨 — bag record 실패. 저장 안 됨. "
                f"마지막 로그:\n{tail}")
            self._close_log()
            self._proc = None

    def _close_log(self):
        if self._log_fh is not None:
            try:
                self._log_fh.close()
            except Exception:
                pass
            self._log_fh = None

    def _stop(self, reason: str):
        if self._health_timer is not None:
            self._health_timer.cancel()
            self._health_timer = None
        if self._proc is None:
            self._close_log()
            return
        try:
            # ros2 bag record 는 자식 프로세스를 두므로 그룹째 SIGINT (bag 정상 마감/인덱싱).
            os.killpg(os.getpgid(self._proc.pid), signal.SIGINT)
            self._proc.wait(timeout=10)
        except Exception as e:
            self.get_logger().warn(f"[REC] 종료 처리 예외({e}) → 강제 종료")
            try:
                self._proc.kill()
            except Exception:
                pass
        self.get_logger().info(f"[REC] 녹화 종료 ({reason}) → {self._flight_dir}")
        self._close_log()
        self._proc = None

    def destroy_node(self):
        if self._recording():
            self._stop("노드 종료")   # 스택 종료 시에도 bag 을 정상 마감
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = FlightRecorder()
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
