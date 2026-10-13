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
import json
import os
import shutil
import subprocess
import tempfile
import threading
import webbrowser
from collections import OrderedDict
from datetime import datetime
from pathlib import Path

from flask import Flask, Response, abort, jsonify, request, send_file, send_from_directory

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
            "ibvs_count": s["counts"].get("/arms/control/ibvs_debug", 0),
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


def _load_cached(p: Path) -> dict:
    """기록 로드(최근 4개 캐시). 캐시 키에 db 파일 mtime/크기를 넣어 녹화 중 갱신도 반영."""
    dbs = bag_reader.find_db3(p)
    key = (str(p), tuple((str(d), d.stat().st_mtime_ns, d.stat().st_size) for d in dbs))
    with _cache_lock:
        if key in _cache:
            _cache.move_to_end(key)
            return _cache[key]
    data = bag_reader.load_record(p)
    data["name"] = p.name
    with _cache_lock:
        _cache[key] = data
        while len(_cache) > 4:
            _cache.popitem(last=False)
    return data


def _manifest_path(record: Path) -> Path:
    """비행 폴더의 manifest 위치. bag/ 또는 단일 db3 선택도 허용한다."""
    base = record.parent if record.is_file() else record
    if base.name == "bag":
        base = base.parent
    return base / "manifest.json"


def _read_manifest(record: Path) -> dict:
    path = _manifest_path(record)
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
        return data if isinstance(data, dict) else {}
    except (OSError, json.JSONDecodeError):
        return {}


def _write_manifest(record: Path, data: dict):
    path = _manifest_path(record)
    path.parent.mkdir(parents=True, exist_ok=True)
    temp = path.with_suffix(".json.tmp")
    temp.write_text(json.dumps(data, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    temp.replace(path)


def _mp4_name(record: Path, topic: str) -> str:
    """다운로드 파일명. 토픽별 UI/원본 영상을 구분한다."""
    source = "ui" if topic == bag_reader.VIDEO_TOPIC else "raw"
    return f"{record.name}_{source}.mp4"


def _export_mp4(record: Path, topic: str) -> Path:
    """색인된 JPEG 프레임을 ffmpeg에 직접 전달해 임시 H.264 MP4로 만든다."""
    if shutil.which("ffmpeg") is None:
        abort(503, "ffmpeg가 없습니다. Ubuntu에서 'sudo apt install ffmpeg'를 설치하세요.")
    data = _load_cached(record)
    frames = data.get("_frames", {}).get(topic)
    times = (data.get("videos", {}).get(topic) or {}).get("t")
    if not frames or not times:
        abort(404, "선택한 compressed 영상 토픽이 없습니다")
    if len(frames) != len(times):
        abort(500, "영상 프레임 색인이 일치하지 않습니다")

    span = times[-1] - times[0] if len(times) > 1 else 0.0
    fps = (len(times) - 1) / span if span > 1e-6 else 15.0
    fps = min(max(fps, 1.0), 120.0)
    fd, name = tempfile.mkstemp(prefix="ibvs_viewer_", suffix=".mp4")
    os.close(fd)
    output = Path(name)
    command = [
        "ffmpeg", "-y", "-hide_banner", "-loglevel", "error",
        # rosbag의 auto_save 영상은 JPEG compressed이다. image2pipe만으로는 일부 ffmpeg가
        # 입력 코덱을 추론하지 못하므로 mjpeg를 명시한다.
        "-f", "image2pipe", "-c:v", "mjpeg", "-framerate", f"{fps:.4f}", "-i", "-",
        "-vf", "pad=ceil(iw/2)*2:ceil(ih/2)*2",
        "-c:v", "libx264", "-pix_fmt", "yuv420p", "-movflags", "+faststart",
        str(output),
    ]
    proc = subprocess.Popen(command, stdin=subprocess.PIPE, stderr=subprocess.PIPE)
    try:
        for db, mid in frames:
            image, _mime = bag_reader.read_frame(db, mid)
            proc.stdin.write(image)
        proc.stdin.close()
        stderr = proc.stderr.read().decode("utf-8", "replace")
        if proc.wait() != 0:
            abort(500, f"ffmpeg MP4 변환 실패: {stderr.strip() or 'unknown error'}")
    except Exception:
        if proc.stdin and not proc.stdin.closed:
            proc.stdin.close()
        proc.kill()
        proc.wait()
        output.unlink(missing_ok=True)
        raise
    finally:
        if proc.stderr:
            proc.stderr.close()
    return output


@app.get("/api/frame")
def frame():
    """화면 영상 i번째 프레임(JPEG). 뷰어가 재생/커서 위치에 맞춰 한 장씩 요청한다."""
    p = _record_path(request.args.get("id", ""))
    try:
        idx = int(request.args.get("i", "0"))
        data = _load_cached(p)
        topic = request.args.get("topic") or (data.get("video") or {}).get("topic")
        db, mid = data["_frames"][topic][idx]
        img, mime = bag_reader.read_frame(db, mid)
    except (ValueError, IndexError, KeyError):
        abort(404)
    return Response(img, mimetype=mime, headers={"Cache-Control": "private, max-age=3600"})


@app.get("/api/mp4")
def download_mp4():
    """선택한 compressed 영상 토픽을 요청 시 H.264 MP4로 만들어 다운로드한다."""
    record = _record_path(request.args.get("id", ""))
    data = _load_cached(record)
    topic = request.args.get("topic") or (data.get("video") or {}).get("topic")
    if topic not in data.get("_frames", {}):
        abort(404, "선택한 compressed 영상 토픽이 없습니다")
    output = _export_mp4(record, topic)
    response = send_file(output, mimetype="video/mp4", as_attachment=True,
                         download_name=_mp4_name(record, topic), max_age=0)
    response.call_on_close(lambda: output.unlink(missing_ok=True))
    return response


@app.get("/api/record")
def record():
    p = _record_path(request.args.get("id", ""))
    try:
        data = _load_cached(p)
    except FileNotFoundError as e:
        return jsonify({"error": str(e)}), 404
    payload = {k: v for k, v in data.items() if not k.startswith("_")}
    payload["manifest"] = _read_manifest(p)
    return jsonify(payload)


@app.post("/api/manifest/balloon")
def save_balloon():
    body = request.get_json(silent=True) or {}
    p = _record_path(str(body.get("id", "")))
    balloon = body.get("balloon")
    if not isinstance(balloon, dict):
        return jsonify({"error": "balloon 객체가 필요함"}), 400
    try:
        clean = {
            "latitude_deg": float(balloon["latitude_deg"]),
            "longitude_deg": float(balloon["longitude_deg"]),
            "altitude_m": float(balloon["altitude_m"]),
        }
    except (KeyError, TypeError, ValueError):
        return jsonify({"error": "풍선 위도·경도·고도는 숫자여야 함"}), 400
    manifest = _read_manifest(p)
    manifest["balloon"] = clean
    _write_manifest(p, manifest)
    return jsonify({"balloon": clean})


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
