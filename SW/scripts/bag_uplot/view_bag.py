#!/usr/bin/env python3
"""auto_save rosbag을 CSV/JSON으로 변환하고 uPlot 웹 뷰어를 연다."""

import argparse
import csv
import json
import shutil
import sys
import threading
from pathlib import Path

from flask import Flask, jsonify, render_template, send_from_directory

HERE = Path(__file__).resolve().parent
BAG_VIS = HERE.parent / "bag_visualize"
sys.path.insert(0, str(BAG_VIS))

STATE_CODES = {"IDLE": 0, "SEARCH": 1, "LOCK": 2, "TRACK": 3, "FIRE": 4, "RTL": 5}


def number(value):
    value = value.strip()
    if not value:
        return None
    if value.lower() == "true":
        return 1.0
    if value.lower() == "false":
        return 0.0
    try:
        v = float(value)
        return v if v == v and abs(v) != float("inf") else None
    except ValueError:
        return None


def csv_to_topic(path: Path):
    with path.open(newline="") as f:
        rows = list(csv.DictReader(f))
    if not rows or "t_rel" not in rows[0]:
        return None

    columns = {"t_rel": [number(row.get("t_rel", "")) for row in rows]}
    for name in rows[0]:
        if name in {"t_ns", "t_rel"} or name.startswith("header_stamp"):
            continue
        values = [number(row.get(name, "")) for row in rows]
        if any(v is not None for v in values):
            columns[name] = values

    # mission state 문자열은 숫자 상태축도 같이 만든다.
    if "state" in rows[0]:
        values = [STATE_CODES.get(row.get("state", "")) for row in rows]
        if any(v is not None for v in values):
            columns["state_code"] = values

    if len(columns) == 1:
        return None
    return {"name": "/" + path.stem.replace("_", "/", 1), "file": path.name,
            "samples": len(rows), "columns": columns}


def export_data(csv_dir: Path, out_dir: Path):
    topics = []
    for path in sorted(csv_dir.glob("*.csv")):
        topic = csv_to_topic(path)
        if topic:
            topics.append(topic)
    payload = {"state_codes": STATE_CODES, "topics": topics}
    out_dir.mkdir(parents=True, exist_ok=True)
    (out_dir / "data.json").write_text(
        json.dumps(payload, ensure_ascii=False, separators=(",", ":")), encoding="utf-8")
    return topics


def discover_flights(source: Path):
    """비행 하나 또는 로그 루트에서 (정상 기록, 사용 불가 기록)를 찾는다."""
    from _common import resolve_bag

    # auto_save 비행 폴더는 바로 아래에 bag/이 있다. bag 자체나 CSV 폴더도 단일 입력이다.
    if (source / "bag").is_dir() or (source / "metadata.yaml").is_file() \
            or any(source.glob("*.db3")) or any(source.glob("*.mcap")) \
            or any(source.glob("*.csv")):
        return [(source.name, source)], []

    flights = []
    unavailable = []
    for child in source.iterdir() if source.is_dir() else []:
        if not child.is_dir() or child.name.endswith("_uplot"):
            continue
        try:
            resolve_bag(str(child))
        except FileNotFoundError as exc:
            unavailable.append((child.name, str(exc)))
            continue
        flights.append((child.name, child))
    flights.sort(key=lambda item: item[1].stat().st_mtime, reverse=True)
    unavailable.sort(reverse=True)
    return flights, unavailable


def source_mtime(path: Path) -> float:
    """캐시 무효화 기준. bag/CSV 내용이 바뀌면 data.json보다 최신이 된다."""
    candidates = [path]
    if path.is_dir():
        candidates.extend(
            p for p in path.rglob("*")
            if p.is_file() and p.suffix in {".db3", ".mcap", ".yaml", ".csv"})
    return max((p.stat().st_mtime for p in candidates), default=0.0)


def convert_flight(name: str, flight_source: Path, out: Path):
    """선택한 비행 하나만 JSON으로 변환한다. 변경되지 않은 bag은 캐시를 재사용한다."""
    from bag_to_csv import bag_to_csv

    flight_out = out / "flights" / name
    data_path = flight_out / "data.json"
    if data_path.is_file() and data_path.stat().st_mtime >= source_mtime(flight_source):
        return data_path

    # 생성 캐시만 지운다. 원본 arms_flight_log는 절대 건드리지 않는다.
    if flight_out.exists():
        shutil.rmtree(flight_out)
    if flight_source.is_dir() and any(flight_source.glob("*.csv")):
        csv_dir = flight_source
    else:
        csv_dir = flight_out / "csv"
        written = bag_to_csv(str(flight_source), out_dir=str(csv_dir))
        if not written:
            raise RuntimeError("변환 가능한 수치 토픽이 없음")
    topics = export_data(csv_dir, flight_out)
    if not topics:
        raise RuntimeError("그래프로 표시할 수치 토픽이 없음")
    return data_path


def serve_flask(source: Path, out: Path, port: int):
    app = Flask(__name__)
    conversion_lock = threading.Lock()

    def current_catalog():
        flights, unavailable = discover_flights(source)
        items = []
        for name, flight_source in flights:
            cached = out / "flights" / name / "data.json"
            topic_count = None
            if cached.is_file():
                try:
                    topic_count = len(json.loads(cached.read_text(encoding="utf-8")).get("topics", []))
                except Exception:
                    pass
            items.append({"name": name, "source": str(flight_source),
                          "data": f"/api/flights/{name}/data", "topics": topic_count})
        for name, reason in unavailable:
            items.append({"name": name, "topics": 0, "error": reason})
        items.sort(key=lambda item: item["name"], reverse=True)
        return items

    @app.after_request
    def disable_cache(response):
        response.headers["Cache-Control"] = "no-store, no-cache, must-revalidate"
        response.headers["Pragma"] = "no-cache"
        response.headers["Expires"] = "0"
        return response

    @app.get("/")
    def index():
        return render_template("index.html")

    @app.get("/api/flights")
    def flights_api():
        return jsonify({"flights": current_catalog()})

    @app.get("/api/flights/<name>/data")
    def flight_data_api(name):
        flights, unavailable = discover_flights(source)
        registry = {flight_name: path for flight_name, path in flights}
        if name not in registry:
            reasons = {flight_name: reason for flight_name, reason in unavailable}
            return jsonify({"error": reasons.get(name, "기록을 찾을 수 없음")}), 404
        try:
            with conversion_lock:
                data_path = convert_flight(name, registry[name], out)
            return send_from_directory(data_path.parent, data_path.name)
        except Exception as exc:
            return jsonify({"error": f"{type(exc).__name__}: {exc}"}), 422

    app.run(host="127.0.0.1", port=port, debug=False, use_reloader=False, threaded=True)


def main():
    parser = argparse.ArgumentParser(description="A.R.M.S. auto_save uPlot 로그 뷰어")
    parser.add_argument("path", help="<일시>/, bag/, 또는 이미 변환된 csv/ 폴더")
    parser.add_argument("--outdir", default="", help="출력 폴더(기본: <입력>_uplot)")
    parser.add_argument("--port", type=int, default=8765)
    args = parser.parse_args()

    source = Path(args.path).expanduser().resolve()
    if not source.exists():
        raise SystemExit(f"로그 경로가 존재하지 않습니다: {source}")
    out = Path(args.outdir).expanduser().resolve() if args.outdir else Path(str(source) + "_uplot")
    out.mkdir(parents=True, exist_ok=True)

    serve_flask(source, out, args.port)


if __name__ == "__main__":
    main()
