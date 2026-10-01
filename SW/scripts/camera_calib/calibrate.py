#!/usr/bin/env python3
"""체커보드 카메라 캘리브레이션 + 보정 전/후 비교 뷰어 (OpenCV, ROS 불필요).

결과는 arms_control_node 의 control.camera.* 형식(hw_overrides.yaml 에 그대로 붙여넣는 블록)으로
저장한다. IBVS(mode 2)가 픽셀 → tan 좌표 변환에 이 값을 쓴다.

사용:
    python3 calibrate.py                         # 실기체 캡처 동글(by-id), 720x480, 칸 9x6 체커보드(안쪽 코너 8x5)
    python3 calibrate.py --device 0 --cols 8 --rows 5 --square 0.03
    python3 calibrate.py --model fisheye         # 광각이 심하면 equidistant(fisheye) 모델
    python3 calibrate.py --load SW/arms_ws/src/arms_control/config/hw_overrides.yaml   # 현재 적용값 보기
    python3 calibrate.py --images output/20261001_120000/images  # 저장해 둔 사진으로 재계산

키:
    SPACE  캡처 (체커보드가 검출된 프레임만)     d  마지막 캡처 삭제
    k      캘리 계산 + 저장                      l  캘리 파일 다시 불러오기
    u      보정 ON/OFF                           s  나란히 보기(원본 | 보정) ON/OFF
    a      보정 시 화면 범위 전환(검은 테두리 포함 전체 ↔ 유효 영역 크롭)
    g      격자선 ON/OFF (보정 후 직선이 곧게 펴지는지 확인)
    q/ESC  종료

※ 체커보드 cols/rows 는 '안쪽 코너' 개수다 = 칸 수 − 1 (칸 9x6 보드 → 8x5, 기본).
※ 스택(arms_video)이 카메라를 쓰고 있으면 열리지 않는다 — 스택을 내리고 실행.
"""
import argparse
import glob
import os
import sys
from datetime import datetime

import cv2
import numpy as np
import yaml

OUT_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "output")   # gitignore 됨
DEFAULT_DEVICE = "/dev/v4l/by-id/usb-MACROSIL_AV_TO_USB2.0-video-index0"
MIN_CAPTURES = 10
WIN = "camera_calib"


# ---------------------------------------------------------------- 캘리 계산 / 입출력
def find_corners(gray, pattern, fast=False):
    """체커보드 안쪽 코너 검출. fast=True 는 미리보기용(빠름, 서브픽셀 정밀화 생략)."""
    flags = cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE
    if fast:
        flags |= cv2.CALIB_CB_FAST_CHECK
    ok, corners = cv2.findChessboardCorners(gray, pattern, flags)
    if not ok:
        return None
    if not fast:
        crit = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 1e-3)
        corners = cv2.cornerSubPix(gray, corners, (5, 5), (-1, -1), crit)
    return corners


def _fisheye_flag(name):
    """fisheye 플래그 상수 — OpenCV 4.x 는 cv2.fisheye.X, 5.x 는 cv2.X 로 옮겨졌다."""
    for v in (getattr(cv2.fisheye, name, None), getattr(cv2, "fisheye_" + name, None), getattr(cv2, name, None)):
        if v is not None:
            return v
    raise AttributeError(name)


def board_points(pattern, square):
    cols, rows = pattern
    objp = np.zeros((rows * cols, 3), np.float64)
    objp[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2) * square
    return objp


def calibrate(corners_list, size, pattern, square, model):
    """→ dict(K, D, model, rms, per_view, size). corners_list: [Nx1x2] 리스트."""
    objp = board_points(pattern, square)
    w, h = size
    if model == "fisheye":
        # (1, N, 3)/(1, N, 2) 모양이 OpenCV 4.x/5.x 모두에서 받아진다((N,1,3) 은 5.x 에서 에러).
        obj = [objp.reshape(1, -1, 3) for _ in corners_list]
        img = [c.reshape(1, -1, 2).astype(np.float64) for c in corners_list]
        # CALIB_CHECK_COND 는 나쁜 1장 때문에 전체를 실패시키므로 쓰지 않는다(장별 오차로 대신 확인).
        flags = _fisheye_flag("CALIB_RECOMPUTE_EXTRINSIC") | _fisheye_flag("CALIB_FIX_SKEW")
        crit = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 1e-6)
        rms, K, D, rvecs, tvecs = cv2.fisheye.calibrate(
            obj, img, (w, h), np.zeros((3, 3)), np.zeros((4, 1)), flags=flags, criteria=crit)
        per_view = []
        for o, im, r, t in zip(obj, img, rvecs, tvecs):
            proj, _ = cv2.fisheye.projectPoints(o, r, t, K, D)
            per_view.append(float(np.sqrt(np.mean(np.sum((proj.reshape(-1, 2) - im.reshape(-1, 2)) ** 2, axis=1)))))
        D = D.reshape(-1)[:4]
    else:
        obj = [objp.astype(np.float32) for _ in corners_list]
        img = [c.astype(np.float32) for c in corners_list]
        rms, K, D, rvecs, tvecs = cv2.calibrateCamera(obj, img, (w, h), None, None)
        per_view = []
        for o, im, r, t in zip(obj, img, rvecs, tvecs):
            proj, _ = cv2.projectPoints(o, r, t, K, D)
            per_view.append(float(np.sqrt(np.mean(np.sum((proj.reshape(-1, 2) - im.reshape(-1, 2)) ** 2, axis=1)))))
        D = D.reshape(-1)[:5]
    return {"K": K, "D": np.asarray(D, float), "model": model, "rms": float(rms),
            "per_view": per_view, "size": (w, h)}


# ---------------------------------------------------------------- 노드 호환 검사
def _node_undistort(xd, yd, D, fisheye):
    """arms_control_node.cpp undistort_normalized() 를 그대로 옮긴 것."""
    if fisheye:
        rd = np.hypot(xd, yd)
        if rd < 1e-9:
            return xd, yd
        th = rd
        for _ in range(10):
            t2 = th * th; t4 = t2 * t2; t6 = t4 * t2; t8 = t4 * t4
            f = th * (1 + D[0] * t2 + D[1] * t4 + D[2] * t6 + D[3] * t8) - rd
            df = 1 + 3 * D[0] * t2 + 5 * D[1] * t4 + 7 * D[2] * t6 + 9 * D[3] * t8
            if abs(df) < 1e-9:
                break
            th -= f / df
        th = min(max(th, 0.0), 1.5533)
        s = np.tan(th) / rd
        return xd * s, yd * s
    x, y = xd, yd
    for _ in range(20):
        r2 = x * x + y * y
        rad = 1 + ((D[4] * r2 + D[1]) * r2 + D[0]) * r2
        if rad < 0.1:
            break
        dx = 2 * D[2] * x * y + D[3] * (r2 + 2 * x * x)
        dy = D[2] * (r2 + 2 * y * y) + 2 * D[3] * x * y
        x, y = (xd - dx) / rad, (yd - dy) / rad
    return x, y


def _distort(x, y, D, fisheye):
    if fisheye:
        r = np.hypot(x, y)
        if r < 1e-12:
            return x, y
        th = np.arctan(r)
        t2 = th * th
        thd = th * (1 + D[0] * t2 + D[1] * t2 ** 2 + D[2] * t2 ** 3 + D[3] * t2 ** 4)
        return x * thd / r, y * thd / r
    r2 = x * x + y * y
    rad = 1 + ((D[4] * r2 + D[1]) * r2 + D[0]) * r2
    return (x * rad + 2 * D[2] * x * y + D[3] * (r2 + 2 * x * x),
            y * rad + D[2] * (r2 + 2 * y * y) + 2 * D[3] * x * y)


def node_check(calib, step=10, tol_px=0.5):
    """화면 전체 격자에서 노드의 왜곡 역보정이 수렴하는지 검사.
    역보정 결과를 다시 왜곡시켜 원래 픽셀과 tol_px 이상 어긋나면 '실패'(= 그 위치에선
    노드의 x_n,y_n 이 틀림 → IBVS 오차·z1 이 엉뚱해진다). → (실패 비율, 최대 오차 px, 실패 예시 픽셀)"""
    K, D = calib["K"], np.pad(calib["D"], (0, 5))[:5]
    fe = calib["model"] == "fisheye"
    w, h = calib["size"]
    fails, worst, example = 0, 0.0, None
    pts = [(u, v) for v in range(0, h + 1, step) for u in range(0, w + 1, step)]
    for u, v in pts:
        xd = (u - K[0, 2]) / K[0, 0]
        yd = (v - K[1, 2]) / K[1, 1]
        x, y = _node_undistort(xd, yd, D, fe)
        rx, ry = _distort(x, y, D, fe)
        err = float(np.hypot((rx - xd) * K[0, 0], (ry - yd) * K[1, 1]))
        if not np.isfinite(err) or err > tol_px:
            fails += 1
            if example is None or err > worst:
                example = (u, v)
        if np.isfinite(err):
            worst = max(worst, err)
    return fails / len(pts), worst, example


def report_node_check(calib):
    frac, worst, ex = node_check(calib)
    if frac == 0:
        print(f"[calib] 노드 호환 OK — 화면 전체에서 왜곡 역보정 수렴 (최대 {worst:.3f}px)")
        return "node check OK"
    print(f"[calib] ⚠ 노드 호환 경고 — 화면의 {frac*100:.1f}% 에서 arms_control_node 의 왜곡 역보정이 "
          f"수렴하지 않음 (예: 픽셀 {ex}, 최대 {worst:.1f}px). 그 위치에선 IBVS 오차가 틀린다.")
    if calib["model"] != "fisheye":
        print("[calib]   → 왜곡이 강한 광각이면 --model fisheye 로 다시 캘리할 것을 권장.")
    return f"node check: {frac*100:.1f}% of image FAILS (see terminal)"


def hfov_deg(calib):
    w, _ = calib["size"]
    return float(np.degrees(2 * np.arctan(w / 2 / calib["K"][0, 0])))


def to_param_block(calib, n_images, pattern, square):
    """hw_overrides.yaml 의 control.camera 블록과 같은 구조(dict)."""
    K, D = calib["K"], calib["D"]
    w, h = calib["size"]
    return {
        "arms_control_node": {"ros__parameters": {"control": {"camera": {
            "calibrated": True,
            "image_width": int(w),
            "image_height": int(h),
            # 실수형 유지(정수로 쓰면 ROS 파라미터 타입 불일치로 노드 기동 실패)
            "fx": round(float(K[0, 0]), 4),
            "fy": round(float(K[1, 1]), 4),
            "cx": round(float(K[0, 2]), 4),
            "cy": round(float(K[1, 2]), 4),
            "distortion_model": "equidistant" if calib["model"] == "fisheye" else "plumb_bob",
            "distortion": [round(float(v), 6) for v in D],
        }}}},
        "_info": {
            "rms_px": round(calib["rms"], 4),
            "images": n_images,
            "board": f"{pattern[0]}x{pattern[1]} inner corners, square {square} m",
            "hfov_deg": round(hfov_deg(calib), 2),
            "created": datetime.now().isoformat(timespec="seconds"),
        },
    }


class _FloatDumper(yaml.SafeDumper):
    pass


def _float_rep(dumper, v):   # 360 → 360.0 처럼 항상 소수점 유지
    s = repr(float(v))
    return dumper.represent_scalar("tag:yaml.org,2002:float", s)


_FloatDumper.add_representer(float, _float_rep)


def save_yaml(path, block):
    info = block.pop("_info")
    head = "# camera_calib/calibrate.py 결과 — 아래 camera: 블록을 hw_overrides.yaml 의 control.camera 에 붙여넣는다.\n"
    head += "".join(f"#   {k}: {v}\n" for k, v in info.items())
    with open(path, "w") as f:
        f.write(head)
        yaml.dump(block, f, Dumper=_FloatDumper, sort_keys=False, default_flow_style=None)
    block["_info"] = info


def load_yaml(path):
    """camera 블록을 찾아 읽는다. 이 스크립트 결과 / hw_overrides.yaml 둘 다 된다."""
    with open(os.path.expanduser(path)) as f:
        data = yaml.safe_load(f)

    def find(d):
        if isinstance(d, dict):
            if "fx" in d and "distortion" in d:
                return d
            for v in d.values():
                r = find(v)
                if r is not None:
                    return r
        return None

    cam = find(data)
    if cam is None:
        raise ValueError(f"camera 블록(fx, distortion)을 찾지 못함: {path}")
    K = np.array([[cam["fx"], 0, cam["cx"]], [0, cam["fy"], cam["cy"]], [0, 0, 1]], float)
    model = "fisheye" if cam.get("distortion_model") == "equidistant" else "plumb_bob"
    return {"K": K, "D": np.array(cam["distortion"], float), "model": model, "rms": None,
            "per_view": [], "size": (int(cam["image_width"]), int(cam["image_height"]))}


# ---------------------------------------------------------------- 보정 맵
class Undistorter:
    def __init__(self):
        self._calib = None
        self.alpha = 0.0      # 0 = 유효 영역만(크롭), 1 = 원본 전체(검은 테두리)
        self._key = None
        self._maps = None

    @property
    def calib(self):
        return self._calib

    @calib.setter
    def calib(self, c):       # 캘리가 바뀌면 보정 맵을 다시 만든다
        self._calib = c
        self._key = None

    def remap(self, frame):
        h, w = frame.shape[:2]
        c = self.calib
        key = (w, h, self.alpha)
        if key != self._key:
            K, D = c["K"].copy(), c["D"]
            cw, ch = c["size"]
            if (cw, ch) != (w, h):          # 캘리 해상도와 다르면 K 를 비례 스케일
                K[0] *= w / cw
                K[1] *= h / ch
            if c["model"] == "fisheye":
                newK = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(
                    K, D[:4].reshape(4, 1), (w, h), np.eye(3), balance=self.alpha)
                m1, m2 = cv2.fisheye.initUndistortRectifyMap(
                    K, D[:4].reshape(4, 1), np.eye(3), newK, (w, h), cv2.CV_16SC2)
            else:
                newK, _ = cv2.getOptimalNewCameraMatrix(K, D, (w, h), self.alpha, (w, h))
                m1, m2 = cv2.initUndistortRectifyMap(K, D, None, newK, (w, h), cv2.CV_16SC2)
            self._maps, self._key = (m1, m2), key
        return cv2.remap(frame, *self._maps, cv2.INTER_LINEAR)


# ---------------------------------------------------------------- 화면
def draw_grid(img, step=60):
    h, w = img.shape[:2]
    for x in range(0, w, step):
        cv2.line(img, (x, 0), (x, h), (0, 255, 255), 1)
    for y in range(0, h, step):
        cv2.line(img, (0, y), (w, y), (0, 255, 255), 1)


def put_lines(img, lines, org=(8, 20), color=(255, 255, 255)):
    x, y = org
    for ln in lines:
        cv2.putText(img, ln, (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 3, cv2.LINE_AA)
        cv2.putText(img, ln, (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1, cv2.LINE_AA)
        y += 18


def open_camera(dev, w, h, required=True):
    src = int(dev) if str(dev).isdigit() else dev
    cap = cv2.VideoCapture(src, cv2.CAP_V4L2) if sys.platform.startswith("linux") else cv2.VideoCapture(src)
    if not cap.isOpened():
        cap = cv2.VideoCapture(src)
    if not cap.isOpened():
        if not required:
            return None
        sys.exit(f"카메라를 열 수 없음: {dev}  (스택이 쓰고 있으면 내리고, --device 확인)")
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, w)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, h)
    return cap


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--device", default=DEFAULT_DEVICE, help="카메라 (인덱스 또는 경로)")
    ap.add_argument("--width", type=int, default=720)
    ap.add_argument("--height", type=int, default=480)
    ap.add_argument("--cols", type=int, default=8, help="체커보드 안쪽 코너 가로 개수 (칸 수 − 1)")
    ap.add_argument("--rows", type=int, default=5, help="체커보드 안쪽 코너 세로 개수 (칸 수 − 1)")
    ap.add_argument("--square", type=float, default=0.025, help="칸 한 변 [m] (내부파라미터엔 영향 없음)")
    ap.add_argument("--model", choices=["plumb_bob", "fisheye"], default="plumb_bob")
    ap.add_argument("--out", default=OUT_DIR, help="저장 상위 폴더 (<일시>/ 하위에 저장, 기본 이 스크립트 옆 output/)")
    ap.add_argument("--load", default="", help="시작 시 불러올 캘리 yaml (결과 파일 또는 hw_overrides.yaml)")
    ap.add_argument("--images", default="", help="저장해 둔 캡처 폴더로 바로 캘리 계산")
    args = ap.parse_args()

    pattern = (args.cols, args.rows)
    session = os.path.join(os.path.expanduser(args.out), datetime.now().strftime("%Y%m%d_%H%M%S"))
    img_dir = os.path.join(session, "images")
    out_yaml = os.path.join(session, "camera.yaml")

    captures = []            # [(frame, corners)]
    und = Undistorter()
    load_path = args.load
    msg = ""                 # 화면 하단 상태줄 (putText 는 한글을 못 그려서 영문, 자세한 건 터미널)

    node_line = ""           # 노드 호환 검사 결과 (화면 고정 표시)

    def notify(screen, term=None):
        nonlocal msg
        msg = screen
        print("[calib]", term or screen)

    def set_calib(c):
        nonlocal node_line
        und.calib = c
        node_line = report_node_check(c)

    def do_calibrate():
        nonlocal load_path
        if len(captures) < MIN_CAPTURES:
            notify(f"need >= {MIN_CAPTURES} captures (now {len(captures)})",
                   f"캡처 {len(captures)}장 — 최소 {MIN_CAPTURES}장 필요")
            return
        h, w = captures[0][0].shape[:2]
        try:
            c = calibrate([cc for _, cc in captures], (w, h), pattern, args.square, args.model)
        except cv2.error as e:
            notify("calibration FAILED (see terminal)",
                   f"캘리 실패 — 나쁜 캡처(흐림/일부만 보임)가 섞였을 수 있음. d 로 지우고 다시: {e}")
            return
        os.makedirs(img_dir, exist_ok=True)
        for i, (f, _) in enumerate(captures):
            cv2.imwrite(os.path.join(img_dir, f"{i:03d}.png"), f)
        block = to_param_block(c, len(captures), pattern, args.square)
        save_yaml(out_yaml, block)
        load_path = out_yaml
        notify(f"calibrated: RMS {c['rms']:.3f}px  hfov {hfov_deg(c):.1f}deg  saved",
               f"RMS {c['rms']:.4f} px  ({len(captures)}장, model={args.model})")
        set_calib(c)
        worst = sorted(enumerate(c["per_view"]), key=lambda t: -t[1])[:5]
        print("[calib] 오차 큰 캡처:", ", ".join(f"#{i} {e:.3f}px" for i, e in worst))
        print(f"[calib] 저장: {out_yaml}\n")
        with open(out_yaml) as f:
            print(f.read())

    # ---- 저장된 사진으로 바로 계산
    if args.images:
        files = sorted(glob.glob(os.path.join(os.path.expanduser(args.images), "*.png"))
                       + glob.glob(os.path.join(os.path.expanduser(args.images), "*.jpg")))
        for fp in files:
            f = cv2.imread(fp)
            cc = find_corners(cv2.cvtColor(f, cv2.COLOR_BGR2GRAY), pattern)
            if cc is not None:
                captures.append((f, cc))
        print(f"[calib] {args.images}: {len(files)}장 중 {len(captures)}장 검출")
        do_calibrate()

    if load_path and und.calib is None:
        notify("loaded calibration", f"불러옴: {load_path}")
        set_calib(load_yaml(load_path))

    cap = open_camera(args.device, args.width, args.height, required=not args.images)
    if cap is None:   # --images 로 계산만 하고, 카메라가 없으면 비교 화면 없이 종료
        print("[calib] 카메라 없음 → 비교 화면 없이 종료 (결과는 위 경로에 저장됨)")
        return
    show_und, side, grid = und.calib is not None, False, False
    cv2.namedWindow(WIN, cv2.WINDOW_NORMAL)
    warned = False

    while True:
        ok, frame = cap.read()
        if not ok:
            if cv2.waitKey(30) & 0xFF in (ord("q"), 27):
                break
            continue
        h, w = frame.shape[:2]
        if (w, h) != (args.width, args.height) and not warned:
            print(f"[calib] 주의: 실제 해상도 {w}x{h} (요청 {args.width}x{args.height}). "
                  "캘리는 실제 해상도 기준으로 저장된다.")
            warned = True

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        live = find_corners(gray, pattern, fast=True)

        # 원본 뷰: 검출 코너 + 지금까지 캡처한 코너 분포(화면 구석구석 덮였는지 확인용)
        raw = frame.copy()
        for _, cc in captures:
            for p in cc.reshape(-1, 2)[::3]:
                cv2.circle(raw, (int(p[0]), int(p[1])), 2, (255, 128, 0), -1)
        if live is not None:
            cv2.drawChessboardCorners(raw, pattern, live, True)

        if und.calib is not None and (show_und or side):
            fixed = und.remap(frame)
        else:
            fixed = None

        if side and fixed is not None:
            left, right = raw, fixed.copy()
            if grid:
                draw_grid(left)
                draw_grid(right)
            put_lines(left, ["ORIGINAL"], (8, h - 10), (0, 200, 255))
            put_lines(right, ["UNDISTORTED"], (8, h - 10), (0, 255, 0))
            view = np.hstack([left, right])
        else:
            view = fixed.copy() if (show_und and fixed is not None) else raw
            if grid:
                draw_grid(view)

        mode = ("SIDE" if side else ("UNDIST ON" if show_und else "UNDIST OFF")) if und.calib else "NO CALIB"
        info = [
            f"{w}x{h}  board {pattern[0]}x{pattern[1]}  model {args.model}",
            f"captures {len(captures)}  board {'DETECTED' if live is not None else '-'}  view {mode}"
            f"  crop {'all' if und.alpha >= 1 else 'valid'}",
            "SPACE cap  d undo  k calib  l load  u undist  s side  a crop  g grid  q quit",
        ]
        if und.calib is not None:
            c = und.calib
            rms = f"  rms {c['rms']:.3f}" if c.get("rms") is not None else ""
            info.append(f"fx {c['K'][0,0]:.1f} fy {c['K'][1,1]:.1f} cx {c['K'][0,2]:.1f} "
                        f"cy {c['K'][1,2]:.1f}  hfov {hfov_deg(c):.1f}{rms}")
        if node_line:
            info.append(node_line)
        put_lines(view, info, color=(0, 255, 0) if live is not None else (255, 255, 255))
        if msg:
            put_lines(view, [msg], (8, view.shape[0] - 30), (0, 255, 255))
        cv2.imshow(WIN, view)

        key = cv2.waitKey(1) & 0xFF
        if key in (ord("q"), 27):
            break
        elif key == ord(" "):
            cc = find_corners(gray, pattern)
            if cc is None:
                notify("board NOT detected - not captured", "체커보드 미검출 — 캡처 안 함")
            else:
                captures.append((frame.copy(), cc))
                notify(f"captured #{len(captures)}", f"캡처 {len(captures)}장")
        elif key == ord("d") and captures:
            captures.pop()
            notify(f"removed last -> {len(captures)}", f"마지막 캡처 삭제 → {len(captures)}장")
        elif key == ord("k"):
            do_calibrate()
            show_und = und.calib is not None
        elif key == ord("l"):
            if not load_path:
                notify("nothing to load", "불러올 파일 없음 (--load 로 지정하거나 k 로 먼저 계산)")
            else:
                try:
                    c = load_yaml(load_path)
                    notify("loaded calibration", f"불러옴: {load_path}")
                    set_calib(c)
                    show_und = True
                except Exception as e:
                    notify("load FAILED (see terminal)", f"불러오기 실패: {e}")
        elif key == ord("u"):
            show_und = not show_und
            side = False
        elif key == ord("s"):
            side = not side
        elif key == ord("a"):
            und.alpha = 0.0 if und.alpha >= 1 else 1.0
        elif key == ord("g"):
            grid = not grid

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
