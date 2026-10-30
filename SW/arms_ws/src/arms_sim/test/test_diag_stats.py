"""diag_stats 집계 도우미 검증 (ROS 불필요)."""
import pytest

from arms_sim.diag_stats import (RateWindow, SimClock, StampGapTracker, StampMatcher,
                                 TransitionLog, WindowStats)


def test_rate_window():
    r = RateWindow(window_sec=2.0)
    for i in range(61):
        r.add(i / 30.0)            # 30 Hz, 2 s
    assert r.rate(2.0) == pytest.approx(30.0, rel=0.02)
    assert r.rate(10.0) == 0.0     # 오래된 표본은 버린다


def test_stamp_gap_tracker_counts_missing_frames():
    g = StampGapTracker(1.0 / 60.0)
    stamps = [i / 60.0 for i in range(10)] + [13 / 60.0, 14 / 60.0]   # 10,11,12 누락
    for s in stamps:
        g.add(s)
    assert g.received == 12
    assert g.missing == 3
    assert g.max_gap == pytest.approx(4 / 60.0)
    assert g.loss_ratio() == pytest.approx(3 / 15)


def test_stamp_gap_tracker_restart_and_duplicates():
    g = StampGapTracker(0.1)
    for s in (5.0, 5.1, 5.1, 0.0, 0.1):
        g.add(s)
    assert g.duplicates == 1
    assert g.restarts == 1
    assert g.missing == 0


def test_sim_clock_extrapolates_with_rtf():
    c = SimClock()
    assert c.now(0.0) is None
    c.update(10.0, 100.0, 0.5)
    c.update(10.1, 100.2, 0.5)
    assert c.rtf == pytest.approx(0.5)
    assert c.now(101.2) == pytest.approx(10.6)


def test_sim_clock_measures_rtf_when_reported_invalid():
    c = SimClock()
    c.update(0.0, 0.0, 0.0)
    c.update(1.0, 4.0, 0.0)
    assert c.rtf == pytest.approx(0.25)


def test_window_stats_and_matcher():
    w = WindowStats()
    for v in range(1, 101):
        w.add(v / 1000.0)
    s = w.summary()
    assert s["n"] == 100 and s["min"] == pytest.approx(0.001)
    assert s["p95"] == pytest.approx(0.095) and s["max"] == pytest.approx(0.1)
    m = StampMatcher(maxlen=2)
    m.add(1, 10.0)
    m.add(2, 11.0)
    m.add(3, 12.0)
    assert m.pop(1) is None          # 오래된 키는 밀려난다
    assert m.pop(3) == 12.0 and m.pop(3) is None


def test_transition_log_flags_unexpected():
    t = TransitionLog()
    assert not t.add(0.0, "LOCK", "TRACK")
    assert not t.add(1.0, "TRACK", "IDLE")
    assert t.add(2.0, "IDLE", "TRACK")      # IDLE 에서 곧장 TRACK 은 비정상
    assert t.counts[("LOCK", "TRACK")] == 1
