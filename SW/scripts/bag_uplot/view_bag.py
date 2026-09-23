#!/usr/bin/env python3
"""auto_save rosbag을 CSV/JSON으로 변환하고 uPlot 웹 뷰어를 연다."""

import argparse
import csv
import json
import shutil
import sys
import webbrowser
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

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
    (out_dir / "data.json").write_text(
        json.dumps(payload, ensure_ascii=False, separators=(",", ":")), encoding="utf-8")
    for asset in ("index.html", "app.js", "style.css"):
        shutil.copy2(HERE / asset, out_dir / asset)
    return topics


def main():
    parser = argparse.ArgumentParser(description="A.R.M.S. auto_save uPlot 로그 뷰어")
    parser.add_argument("path", help="<일시>/, bag/, 또는 이미 변환된 csv/ 폴더")
    parser.add_argument("--outdir", default="", help="출력 폴더(기본: <입력>_uplot)")
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument("--no-browser", action="store_true")
    parser.add_argument("--export-only", action="store_true")
    args = parser.parse_args()

    source = Path(args.path).expanduser().resolve()
    out = Path(args.outdir).expanduser().resolve() if args.outdir else Path(str(source) + "_uplot")
    out.mkdir(parents=True, exist_ok=True)

    if source.is_dir() and any(source.glob("*.csv")):
        csv_dir = source
    else:
        from bag_to_csv import bag_to_csv
        csv_dir = out / "csv"
        written = bag_to_csv(str(source), out_dir=str(csv_dir))
        if not written:
            raise SystemExit("변환 가능한 데이터 토픽이 없습니다.")

    topics = export_data(csv_dir, out)
    print(f"[uplot] {len(topics)}개 토픽 → {out / 'index.html'}")
    if args.export_only:
        return

    handler = lambda *a, **kw: SimpleHTTPRequestHandler(*a, directory=str(out), **kw)
    server = ThreadingHTTPServer(("127.0.0.1", args.port), handler)
    url = f"http://127.0.0.1:{args.port}/"
    print(f"[uplot] {url}  (종료: Ctrl-C)")
    if not args.no_browser:
        webbrowser.open(url)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()


if __name__ == "__main__":
    main()
