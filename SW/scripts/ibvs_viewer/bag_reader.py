#!/usr/bin/env python3
"""ROS 없이 rosbag2(sqlite3) 를 읽는 헬퍼 — ibvs_viewer 전용.

rosbag2_py 대신 sqlite3 로 직접 열고, 역직렬화는 순수 파이썬 `rosbags` 로 한다.
→ 맥/노트북(ROS 미설치)에서도 젯슨에서 가져온 bag 을 바로 볼 수 있다.
  · metadata.yaml 이 없는(녹화 강제 종료) bag 도 .db3 만 있으면 읽힌다.
  · bag_0.db3, bag_1.db3 … 분할 bag 은 이어붙인다.
  · arms_msgs 커스텀 타입은 레포의 arms_ws/src/arms_msgs/msg/*.msg 를 등록해 푼다.
    (녹화 이후 .msg 필드가 바뀌었으면 그 토픽은 역직렬화 실패로 건너뛴다.)
"""
import math
import os
import sqlite3
from dataclasses import fields, is_dataclass
from pathlib import Path

import numpy as np

MSG_DIR = Path(__file__).resolve().parents[2] / "arms_ws" / "src" / "arms_msgs" / "msg"

# 영상은 그래프 대상이 아니고 무거우므로 역직렬화하지 않는다(프레임 색인만 만든다).
SKIP_TYPES = {"sensor_msgs/msg/CompressedImage", "sensor_msgs/msg/Image"}
VIDEO_TYPE = "sensor_msgs/msg/CompressedImage"
VIDEO_TOPIC = "/arms/ui_image/compressed"   # 우선 사용할 화면 영상 토픽
# 평탄화에서 버릴 필드 (시각은 bag 수신시각으로 통일, MultiArray layout 은 잡음).
SKIP_FIELDS = {"header", "layout"}
MAX_ARRAY = 16

_typestore = None


def typestore():
    """ROS2 Humble 기본 타입 + arms_msgs 커스텀 타입이 등록된 typestore (1회 생성)."""
    global _typestore
    if _typestore is None:
        from rosbags.typesys import Stores, get_types_from_msg, get_typestore
        ts = get_typestore(Stores.ROS2_HUMBLE)
        add = {}
        for f in sorted(MSG_DIR.glob("*.msg")):
            add.update(get_types_from_msg(f.read_text(), f"arms_msgs/msg/{f.stem}"))
        ts.register(add)
        _typestore = ts
    return _typestore


def find_db3(record: Path) -> list:
    """기록 폴더 안(<일시>/bag/ 까지 2단계)의 .db3 파일들 (분할 순서대로)."""
    if record.is_file():
        return [record] if record.suffix == ".db3" else []
    try:
        out = sorted(record.glob("*.db3")) or sorted(record.glob("*/*.db3"))
    except OSError:
        return []
    return [p for p in out if p.is_file()]


def is_record(d: Path, max_subdirs: int = 8) -> bool:
    """폴더 선택 화면용 빠른 판정 — .db3 가 바로 안이나 하위 폴더 몇 개 안에 있는가.
    (~/Library 같은 큰 폴더에서 find_db3 의 전체 glob 은 수 초 걸린다.)"""
    try:
        subs = []
        with os.scandir(d) as it:
            for e in it:
                if e.name.endswith(".db3") and e.is_file():
                    return True
                if len(subs) < max_subdirs and e.is_dir() and not e.name.startswith("."):
                    subs.append(e.path)
        for s in subs:
            with os.scandir(s) as it:
                if any(e.name.endswith(".db3") for e in it):
                    return True
    except OSError:
        pass
    return False


def _connect(db: Path):
    # 읽기 전용 — 녹화 중인 bag 을 열어도 건드리지 않는다.
    return sqlite3.connect(f"file:{db}?mode=ro", uri=True)


def record_summary(record: Path) -> dict:
    """사이드바용 요약: 길이[s], 토픽별 메시지 수, 크기. 역직렬화 없이 SQL 만 쓴다."""
    dbs = find_db3(record)
    counts, t_min, t_max, size = {}, None, None, 0
    for db in dbs:
        size += db.stat().st_size
        try:
            con = _connect(db)
            rows = con.execute(
                "SELECT t.name, COUNT(*), MIN(m.timestamp), MAX(m.timestamp) "
                "FROM messages m JOIN topics t ON m.topic_id = t.id GROUP BY t.name").fetchall()
            con.close()
        except sqlite3.Error:
            continue
        for name, n, lo, hi in rows:
            counts[name] = counts.get(name, 0) + n
            t_min = lo if t_min is None else min(t_min, lo)
            t_max = hi if t_max is None else max(t_max, hi)
    return {
        "start_ns": t_min,
        "duration": (t_max - t_min) / 1e9 if t_min is not None else 0.0,
        "counts": counts,
        "size": size,
        "n_files": len(dbs),
    }


def _scalar(v):
    """JSON 에 담을 스칼라로. NaN/inf → None, bool → 0/1."""
    if isinstance(v, (bool, np.bool_)):
        return int(v)
    if isinstance(v, (int, np.integer)):
        return int(v)
    if isinstance(v, (float, np.floating)):
        f = float(v)
        return f if math.isfinite(f) else None
    return v


def flatten(msg, prefix: str = "") -> dict:
    """rosbags 메시지(dataclass) → {열이름: 스칼라|문자열}.

    중첩 메시지는 prefix_ 로 재귀, 원시 배열은 name_0.. 로 전개(최대 16),
    메시지 배열은 개수(name_count) + 첫 원소(name_0_*) 만 남긴다.
    """
    out = {}
    for f in fields(msg):
        name = f.name
        if name in SKIP_FIELDS or name.startswith("__"):   # rosbags 내부 필드(__msgtype__)
            continue
        val = getattr(msg, name)
        key = prefix + name
        if is_dataclass(val):
            out.update(flatten(val, key + "_"))
        elif isinstance(val, str):
            out[key] = val
        elif isinstance(val, (np.ndarray, list, tuple)):
            seq = list(val)
            if seq and is_dataclass(seq[0]):
                out[key + "_count"] = len(seq)
                out.update(flatten(seq[0], key + "_0_"))
            elif not seq and isinstance(val, list):
                out[key + "_count"] = 0
            else:
                for i, v in enumerate(seq[:MAX_ARRAY]):
                    out[f"{key}_{i}"] = _scalar(v)
        else:
            out[key] = _scalar(val)
    return out


def load_record(record: Path) -> dict:
    """기록 하나를 토픽별 열(column) 데이터로 읽는다.

    반환:
      {"t0_ns", "duration", "errors": [...],
       "topics": {토픽: {"type", "t": [초...],
                         "fields": {열: [수치|None...]},
                         "strings": {열: [문자열...]}}}}
    t 는 기록 전체의 첫 메시지(영상 포함) 기준 상대초 — 모든 토픽이 같은 시간축을 쓴다.
    """
    ts = typestore()
    dbs = find_db3(record)
    if not dbs:
        raise FileNotFoundError(f".db3 가 없음: {record}")

    rows = {}      # 토픽 → [(t_ns, flat dict)]
    types = {}
    errors = {}
    frames = {}    # 영상 토픽 → [(t_ns, db 경로, message id)]  (디코드 없이 색인만)
    t0 = None
    for db in dbs:
        con = _connect(db)
        lo = con.execute("SELECT MIN(timestamp) FROM messages").fetchone()[0]
        if lo is not None:
            t0 = lo if t0 is None else min(t0, lo)
        topics = {tid: (name, ty) for tid, name, ty in
                  con.execute("SELECT id, name, type FROM topics")}
        for tid, (name, ty) in topics.items():
            if ty == VIDEO_TYPE:
                frames.setdefault(name, []).extend(
                    (t_ns, str(db), mid) for mid, t_ns in con.execute(
                        "SELECT id, timestamp FROM messages WHERE topic_id = ?", (tid,)))
        want = [tid for tid, (_, ty) in topics.items() if ty not in SKIP_TYPES]
        if not want:
            con.close()
            continue
        q = ("SELECT topic_id, timestamp, data FROM messages WHERE topic_id IN (%s) "
             "ORDER BY timestamp" % ",".join("?" * len(want)))
        for tid, t_ns, data in con.execute(q, want):
            name, ty = topics[tid]
            if name in errors:
                continue
            try:
                flat = flatten(ts.deserialize_cdr(bytes(data), ty))
            except Exception as e:  # 타입 미등록 / .msg 정의 불일치
                errors[name] = f"{name} ({ty}): {type(e).__name__}: {e}"
                rows.pop(name, None)
                continue
            types[name] = ty
            rows.setdefault(name, []).append((t_ns, flat))
        con.close()

    if t0 is None:
        t0 = 0
    out_topics = {}
    t_end = t0
    for name, items in rows.items():
        # 행마다 키가 다를 수 있어(검출 0/1개 등) 키 합집합으로 열을 만든다.
        keys = []
        seen = set()
        for _, flat in items:
            for k in flat:
                if k not in seen:
                    seen.add(k)
                    keys.append(k)
        num, strs = {}, {}
        for k in keys:
            col = [flat.get(k) for _, flat in items]
            if any(isinstance(v, str) for v in col):
                strs[k] = ["" if v is None else str(v) for v in col]
            else:
                num[k] = col
        t_end = max(t_end, items[-1][0])
        out_topics[name] = {
            "type": types[name],
            "t": [round((t - t0) / 1e9, 6) for t, _ in items],
            "fields": num,
            "strings": strs,
        }
    # CompressedImage는 UI/원본 등 여러 개일 수 있다. UI를 기본으로 쓰되, 각 소스를
    # 독립 색인해 브라우저에서 바꿔 볼 수 있게 한다.
    videos, frame_indexes = {}, {}
    for name, items in frames.items():
        fr = sorted(items)
        if not fr:
            continue
        t_end = max(t_end, fr[-1][0])
        videos[name] = {"topic": name, "t": [round((t - t0) / 1e9, 6) for t, _, _ in fr]}
        frame_indexes[name] = [(db, mid) for _, db, mid in fr]
    vname = VIDEO_TOPIC if VIDEO_TOPIC in videos else next(iter(videos), None)
    video = videos.get(vname)
    return {
        "t0_ns": t0,
        "duration": (t_end - t0) / 1e9,
        "topics": out_topics,
        "video": video,              # 기본 영상 {"topic", "t": [초...]} 또는 None
        "videos": videos,            # 모든 CompressedImage 소스
        "errors": list(errors.values()),
        "_frames": frame_indexes,    # 서버 전용(JSON 에 안 나감): 토픽별 i번째 (db, id)
    }


def read_frame(db: str, msg_id: int):
    """색인된 프레임 1장 → (이미지 바이트, mimetype)."""
    con = _connect(Path(db))
    try:
        row = con.execute(
            "SELECT m.data, t.type FROM messages m JOIN topics t ON m.topic_id = t.id "
            "WHERE m.id = ?", (msg_id,)).fetchone()
    finally:
        con.close()
    if row is None:
        raise KeyError(msg_id)
    msg = typestore().deserialize_cdr(bytes(row[0]), row[1])
    fmt = (msg.format or "").lower()
    mime = "image/png" if "png" in fmt else "image/jpeg"
    return bytes(msg.data), mime
