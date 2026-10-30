"""
diag_stats.py — 각도 발사 SITL 진단용 순수 계산 도우미  [arms_sim]

ROS 의존이 없어 단위 테스트(test/test_diag_stats.py)로 직접 검증한다.
angled_launch_monitor.py 가 이것들로 FPS·누락·지연을 집계한다.

시간축 약속 (이 스택은 use_sim_time 을 안 쓴다)
  - header.stamp(카메라/검출) : Gazebo 시뮬레이션 시간. ros_gz_bridge 가 gz 이미지 시각을 넣고,
                                 검출 노드는 입력 영상 header 를 그대로 복사한다.
  - 수신 시각                 : 각 노드의 벽시계(ROS system clock).
  두 시계는 RTF(실시간 배율)만큼 서로 다르게 흐르므로 직접 빼면 안 된다. SimClock 이
  gz WorldStatistics(sim_time ↔ 벽시계 수신시각)로 '지금의 시뮬 시각'을 추정해 준다.
"""

import math
from collections import deque


class RateWindow:
    """최근 window_sec 동안의 이벤트 수로 주기[Hz]를 잰다."""

    def __init__(self, window_sec=2.0):
        self.window = float(window_sec)
        self._t = deque()

    def add(self, t):
        self._t.append(t)
        self._trim(t)

    def _trim(self, now):
        while self._t and now - self._t[0] > self.window:
            self._t.popleft()

    def rate(self, now):
        self._trim(now)
        if len(self._t) < 2:
            return 0.0
        span = max(now - self._t[0], self._t[-1] - self._t[0])
        return (len(self._t) - 1) / span if span > 1e-9 else 0.0

    def count(self, now):
        self._trim(now)
        return len(self._t)


class WindowStats:
    """최근 N개 표본의 min/mean/p95/max."""

    def __init__(self, maxlen=600):
        self._v = deque(maxlen=maxlen)

    def add(self, v):
        if v is not None and math.isfinite(v):
            self._v.append(float(v))

    def __len__(self):
        return len(self._v)

    def summary(self):
        if not self._v:
            return None
        s = sorted(self._v)
        p95 = s[min(len(s) - 1, int(math.ceil(0.95 * len(s))) - 1)]
        return {"n": len(s), "min": s[0], "mean": sum(s) / len(s), "p95": p95, "max": s[-1]}

    def clear(self):
        self._v.clear()


class StampGapTracker:
    """header.stamp 시퀀스에서 누락 프레임과 최대 간격을 센다.

    nominal_period 는 센서 설정 주기(카메라 60 Hz → 1/60 s). 간격이 그 1.5배를
    넘으면 (간격/주기 - 1) 프레임이 빠진 것으로 본다. 시각이 되돌아가면(RESET 등)
    누락이 아니라 재시작으로 보고 기준만 다시 잡는다.
    """

    def __init__(self, nominal_period):
        self.period = float(nominal_period)
        self.reset()

    def reset(self):
        self.last = None
        self.received = 0
        self.missing = 0
        self.restarts = 0
        self.duplicates = 0
        self.max_gap = 0.0

    def add(self, stamp):
        self.received += 1
        if self.last is None:
            self.last = stamp
            return 0
        gap = stamp - self.last
        if gap < -1e-6:
            self.restarts += 1
            self.last = stamp
            return 0
        if gap <= 1e-6:
            self.duplicates += 1
            return 0
        self.last = stamp
        self.max_gap = max(self.max_gap, gap)
        if gap > 1.5 * self.period:
            lost = int(round(gap / self.period)) - 1
            self.missing += lost
            return lost
        return 0

    def loss_ratio(self):
        total = self.received + self.missing
        return self.missing / total if total else 0.0


class SimClock:
    """gz WorldStatistics 표본(sim_time, 벽시계 수신시각, RTF)으로 현재 시뮬 시각을 추정한다.

    WorldStatistics 는 ~5 Hz 로 오므로 마지막 표본에서 RTF 로 외삽한다. 오차는
    대략 (표본 간격 × RTF 변동) 수준으로, 수십 ms 진단에는 충분하다.
    """

    def __init__(self):
        self.sim = None
        self.wall = None
        self.rtf = None
        self._prev = None

    def update(self, sim_time, wall_time, rtf=None):
        if self._prev is not None and wall_time - self._prev[1] > 1e-3 and sim_time >= self._prev[0]:
            measured = (sim_time - self._prev[0]) / (wall_time - self._prev[1])
            self.rtf = measured if rtf is None or not math.isfinite(rtf) or rtf <= 0 else rtf
        elif rtf is not None and math.isfinite(rtf) and rtf > 0:
            self.rtf = rtf
        if self._prev is not None and sim_time < self._prev[0]:
            self.rtf = rtf if rtf and rtf > 0 else None   # 월드 리셋: 기준 재설정
        self._prev = (sim_time, wall_time)
        self.sim, self.wall = sim_time, wall_time

    def valid(self):
        return self.sim is not None

    def now(self, wall_now):
        if self.sim is None:
            return None
        rtf = self.rtf if self.rtf is not None else 1.0
        return self.sim + max(0.0, wall_now - self.wall) * rtf


class StampMatcher:
    """영상 header.stamp → 그 영상을 받은 벽시계 시각. 검출이 같은 stamp 로 오면 짝지어
    '영상 수신 → 검출 수신' 처리 지연을 계산한다(둘 다 같은 노드의 벽시계라 뺄 수 있다)."""

    def __init__(self, maxlen=240):
        self._keys = deque()
        self._vals = {}
        self.maxlen = maxlen

    def add(self, stamp_ns, wall):
        if stamp_ns in self._vals:
            return
        self._keys.append(stamp_ns)
        self._vals[stamp_ns] = wall
        while len(self._keys) > self.maxlen:
            self._vals.pop(self._keys.popleft(), None)

    def pop(self, stamp_ns):
        return self._vals.pop(stamp_ns, None)

    def clear(self):
        self._keys.clear()
        self._vals.clear()


class TransitionLog:
    """미션 상태 전이 기록 + 비정상 전이 분류."""

    # control 이 정상적으로 만드는 전이. 나머지는 '비정상' 으로 표시만 한다(판정은 사람이).
    NORMAL = {
        ("IDLE", "SEARCH"), ("SEARCH", "LOCK"), ("LOCK", "TRACK"),
        ("LOCK", "SEARCH"), ("SEARCH", "IDLE"), ("LOCK", "IDLE"),
        ("TRACK", "RTL"), ("RTL", "IDLE"), ("SEARCH", "RTL"), ("LOCK", "RTL"),
        ("TRACK", "IDLE"),
    }

    def __init__(self, maxlen=200):
        self.events = deque(maxlen=maxlen)
        self.counts = {}

    def add(self, t, prev, new, info=""):
        key = (prev, new)
        self.counts[key] = self.counts.get(key, 0) + 1
        abnormal = key not in self.NORMAL
        self.events.append((t, prev, new, abnormal, info))
        return abnormal

