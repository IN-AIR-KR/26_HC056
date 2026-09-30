#!/usr/bin/env python3
"""IBVS 비행 로그 웹뷰어 (Flask + uPlot).

auto_save 로 녹화한 ~/arms_flight_logs_ibvs/<일시>/ 기록들을 브라우저에서 본다.
ROS 없이 동작한다(sqlite3 + rosbags 로 직접 읽음) → 노트북에 bag 폴더만 복사해 와도 된다.

사용:
    pip install -r SW/scripts/ibvs_viewer/requirements.txt
    python3 SW/scripts/ibvs_viewer/server.py                  # 브라우저에서 데이터 폴더 선택
    python3 SW/scripts/ibvs_viewer/server.py --data ~/arms_flight_logs_ibvs
    python3 SW/scripts/ibvs_viewer/server.py --host 0.0.0.0   # 젯슨에서 띄우고 노트북으로 접속
"""
import argparse
import os
import threading
import webbrowser
from collections import OrderedDict
from datetime import datetime
from pathlib import Path

from flask import Flask, abort, jsonify, request, send_from_directory

import bag_reader

HERE = Path(__file__).resolve().parent
DEFAULT_DIR = Path("~/arms_flight_logs_ibvs").expanduser()

app = Flask(__name__, static_folder=None)
state = {"root": None}                 # 현재 데이터 폴더 (None = 아직 선택 안 함)
_cache = OrderedDict()                 # (경로, mtime, 크기) → 로드 결과. 최근 4개만 유지
_cache_lock = threading.Lock()


def _root() -> Path:
    if state["root"] is None:
        abort(409, "데이터 폴더가 선택되지 않음")
    return state["root"]


def _record_path(rec_id: str) -> Path:
    """사이드바 id(데이터 폴더 기준 상대경로) → 실제 경로. 폴더 밖 접근 차단."""
    root = _root()
    p = (root / rec_id).resolve() if rec_id not in ("", ".") else root
    if p != root and root not in p.parents:
        abort(400, "데이터 폴더 밖 경로")
    if not p.exists():
        abort(404, f"기록 없음: {rec_id}")
    return p


def _records(root: Path) -> list:
    """데이터 폴더 안의 기록(.db3 를 가진 폴더) 목록. 폴더 자체가 기록이어도 된다."""
    cands = []
    if bag_reader.find_db3(root):
        cands.append(root)
    for d in root.iterdir():
        if d.is_dir() and not d.name.startswith(".") and bag_reader.find_db3(d):
            cands.append(d)
    out = []
    for d in cands:
        s = bag_reader.record_summary(d)
        out.append({
            "id": "." if d == root else d.relative_to(root).as_posix(),
            "name": d.name,
            "start": (datetime.fromtimestamp(s["start_ns"] / 1e9).isoformat(timespec="seconds")
                      if s["start_ns"] else None),
            "duration": s["duration"],
            "size": s["size"],
            "ibvs_count": s["counts"].get("/arms/ibvs_debug", 0),
            "counts": s["counts"],
        })
    out.sort(key=lambda r: r["name"], reverse=True)   # <YYYYmmdd_HHMMSS> → 최신이 위
    return out


# ---------------------------------------------------------------- pages
@app.get("/")
def index():
    return send_from_directory(HERE / "static", "index.html")


@app.get("/static/<path:name>")
def static_files(name):
    return send_from_directory(HERE / "static", name)


# ---------------------------------------------------------------- api
@app.get("/api/root")
def get_root():
    return jsonify({
        "root": str(state["root"]) if state["root"] else None,
        "default": str(DEFAULT_DIR),
    })


@app.post("/api/root")
def set_root():
    p = Path(str((request.get_json(silent=True) or {}).get("path", ""))).expanduser()
    if not p.is_dir():
        return jsonify({"error": f"폴더가 아님: {p}"}), 400
    state["root"] = p.resolve()
    return jsonify({"root": str(state["root"])})


@app.get("/api/browse")
def browse():
    """폴더 선택 화면용: 하위 폴더 목록 (+ 각 폴더가 기록 몇 개를 담고 있는지)."""
    p = Path(request.args.get("path") or DEFAULT_DIR).expanduser()
    if not p.is_dir():
        p = Path.home()
    p = p.resolve()
    dirs = []
    try:
        for d in sorted(p.iterdir(), key=lambda x: x.name.lower()):
            if d.is_dir() and not d.name.startswith("."):
                dirs.append({"name": d.name, "path": str(d),
                             "is_record": bag_reader.is_record(d)})
    except PermissionError:
        pass
    return jsonify({"path": str(p), "parent": str(p.parent) if p.parent != p else None,
                    "dirs": dirs})


@app.get("/api/records")
def records():
    root = _root()
    return jsonify({"root": str(root), "records": _records(root)})


@app.get("/api/record")
def record():
    p = _record_path(request.args.get("id", ""))
    dbs = bag_reader.find_db3(p)
    key = (str(p), tuple((str(d), d.stat().st_mtime_ns, d.stat().st_size) for d in dbs))
    with _cache_lock:
        if key in _cache:
            _cache.move_to_end(key)
            return jsonify(_cache[key])
    try:
        data = bag_reader.load_record(p)
    except FileNotFoundError as e:
        return jsonify({"error": str(e)}), 404
    data["name"] = p.name
    with _cache_lock:
        _cache[key] = data
        while len(_cache) > 4:
            _cache.popitem(last=False)
    return jsonify(data)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--data", default="",
                    help="데이터 폴더 (생략하면 브라우저에서 선택)")
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=5050)
    ap.add_argument("--no-browser", action="store_true", help="브라우저 자동 열기 끔")
    args = ap.parse_args()

    if args.data:
        p = Path(args.data).expanduser()
        if not p.is_dir():
            ap.error(f"폴더가 아님: {p}")
        state["root"] = p.resolve()

    bag_reader.typestore()   # arms_msgs .msg 등록 오류를 기동 시점에 바로 드러낸다
    url = f"http://{'127.0.0.1' if args.host in ('0.0.0.0', '::') else args.host}:{args.port}/"
    print(f"IBVS 뷰어: {url}  (데이터 폴더: {state['root'] or '브라우저에서 선택'})")
    if not args.no_browser and not os.environ.get("SSH_CONNECTION"):
        threading.Timer(0.8, lambda: webbrowser.open(url)).start()
    app.run(host=args.host, port=args.port, threaded=True)


if __name__ == "__main__":
    main()
