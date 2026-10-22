/* Flight Analyzer — uPlot 차트, 커서/줌 연동, 기록 목록 사이드바. */
"use strict";

const T = "/arms/control/diagnostics";
const R = "/arms/io/crsf/rx";
const M = "/arms/mission/state";
const C = "/arms/control/debug";
const G = "/arms/vehicle/gps";
const B = "/arms/vehicle/battery";
const CMD = "/arms/operator/input";
const TX = "/arms/io/crsf/tx";
const A = "/arms/vehicle/attitude";
const D = "/arms/perception/detections";
const DR = "/arms/perception/detections_raw";
const DS = "/arms/perception/status";
const IO = "/arms/io/status";
const PN = "/arms/control/pn_debug";
const PN_GT = "/arms/sim/guidance_ground_truth";
const RATE = "__flight_analyzer_rates";
const RATE_WINDOW_SEC = 1.0;
const RATE_STEP_SEC = 0.1;

// ---------------------------------------------------------------- 필드 수식
// 범례는 필드명 그대로. 수식은 패널 위 표와 범례 툴팁에 나온다.
const FIELD_INFO = {
  raw_err_x: "bbox_cx/W − 0.5   (±0.5 = 화면 끝)",
  raw_err_y: "bbox_cy/H − 0.5",
  filt_err_x: "LPF(tan(raw_err_x))   (α = error_lpf_alpha, tan image coordinate)",
  filt_err_y: "LPF(tan(raw_err_y))   (camera y tan coordinate)",
  emag: "√(deadzone(filt_err_x)² + deadzone(filt_err_y − aim_above_offset)²)",
  err_dot_x: "LPF(Δfilt_err_x/dt), clamp ±err_dot_clamp",
  err_dot_y: "LPF(Δfilt_err_y/dt), clamp ±err_dot_clamp",
  x_n: "tan 단위 카메라 x 좌표",
  y_n: "tan 단위 카메라 y 좌표",
  err_angle_x: "atan(x_n) · 180/π   [deg]   (좌우 오차각, + = 오른쪽)",
  err_angle_y: "atan(y_n) · 180/π   [deg]   (상하 오차각, + = 아래)",
  guidance_x: "PID roll 출력 [deg/s] (부호 전)",
  guidance_y: "PID pitch 출력 [deg/s] (부호 전)",
  roll_rate_cmd: "roll_sign · PID 출력 · range_gain_scale   [deg/s]",
  pitch_rate_cmd: "pitch_sign · PID 출력 · range_gain_scale   [deg/s]",
  roll_rate_out: "clamp(roll_rate_cmd, ±max_cmd_rate_dps)   [deg/s] = 송신값",
  pitch_rate_out: "clamp(pitch_rate_cmd, ±max_cmd_rate_dps)   [deg/s] = 송신값",
  closeness: "clamp((loom_size − near_size_start)/(near_size_full − near_size_start), 0, 1)",
  range_gain_scale: "1 − closeness·(1 − min_gain_scale)",
  range_throttle_scale: "1 − closeness·(1 − min_throttle_scale)",
  tau_ease: "clamp((tau − tau_ease_full)/(tau_ease_start − tau_ease_full), tau_ease_min_scale, 1)",
  center_q: "clamp(1 − emag/pursuit_gate, 0, 1)   (pursuit_center_boost=false → 1)",
  loom_size: "EMA(max(bbox_w, bbox_h), loom_size_alpha)",
  loom_rate: "EMA(Δloom_size/dt, loom_rate_alpha)",
  tau: "loom_size / loom_rate   [s]   (loom_rate ≤ 0 → 9.99)",
  throttle_up: "hover + (track − hover)·range_throttle_scale",
  thrust: "hover + (throttle_up − hover)·center_q·tau_ease",
  cmd_active: "state ∈ {TRACK, FIRE}",
  filter_active: "state ∈ {LOCK, TRACK, FIRE}",
  detection_held: "검출 끊김 → 마지막 오차 유지",
  rate_saturated: "|rate_cmd| > max_cmd_rate_dps",
  manual_out: "수동 모드 or RTL (스틱 passthrough)",
  dt: "제어 틱 간격 [ms]",
  roll_deg: "CRSF 텔레메트리 roll [deg]",
  pitch_deg: "CRSF 텔레메트리 pitch [deg]",
  yaw_deg: "CRSF 텔레메트리 yaw [deg]",
  satellites: "GPS 수신 위성 수",
  video_hz: "1초 이동 윈도우의 원본 카메라 compressed 영상 수신율 [Hz]",
  ui_hz: "1초 이동 윈도우의 UI overlay compressed 영상 수신율 [Hz]",
  detection_hz: "1초 이동 윈도우의 detection 메시지 수신율 [Hz]",
  control_hz: "1초 이동 윈도우의 control debug 메시지 수신율 [Hz]",
  tx_uart_write_hz: "최근 1초 UART write 성공 CRSF 채널 프레임율 [Hz]",
  rx_uart_valid_frame_hz: "최근 1초 UART 수신·CRC 통과 CRSF 프레임율 [Hz]",
  estimated_los_rate_x: "영상·자세로 추정한 LOS 각속도 NED north [rad/s]",
  estimated_los_rate_y: "영상·자세로 추정한 LOS 각속도 NED east [rad/s]",
  estimated_los_rate_z: "영상·자세로 추정한 LOS 각속도 NED down [rad/s]",
  applied_los_rate_x: "PN 식에 실제 입력한 LOS 각속도 NED north [rad/s]",
  applied_los_rate_y: "PN 식에 실제 입력한 LOS 각속도 NED east [rad/s]",
  applied_los_rate_z: "PN 식에 실제 입력한 LOS 각속도 NED down [rad/s]",
  estimated_closing_speed_mps: "bbox scale로 추정한 접근속도 [m/s]",
  applied_closing_speed_mps: "PN 식에 실제 입력한 접근속도 [m/s]",
  closing_speed_mps: "Gazebo world pose 기준 접근속도 [m/s]",
  los_rate_x: "Gazebo world pose 기준 LOS 각속도 NED north [rad/s]",
  los_rate_y: "Gazebo world pose 기준 LOS 각속도 NED east [rad/s]",
  los_rate_z: "Gazebo world pose 기준 LOS 각속도 NED down [rad/s]",
  range_m: "bbox scale 기반 거리 추정 [m]",
  acceleration_pn_x: "PN 횡가속도 NED north [m/s²]",
  acceleration_pn_y: "PN 횡가속도 NED east [m/s²]",
  acceleration_pn_z: "PN 횡가속도 NED down [m/s²]",
  command_roll_deg: "FC에 보낼 PN 목표 roll [deg]",
  command_pitch_deg: "FC 규약 변환 전 PN 목표 pitch [deg]",
  command_throttle: "PN 추력 명령 [0..1]",
};

// 기록에 없는 중간값은 기록된 필드로 계산한다. 입력 중 null 이면 null.
const DERIVED = {
  err_angle_x: { deps: ["x_n"], fn: (x) => Math.atan(x) * 180 / Math.PI },
  err_angle_y: { deps: ["y_n"], fn: (y) => Math.atan(y) * 180 / Math.PI },
};

// ---------------------------------------------------------------- 일반 비행 정보 (제어 세부 그래프보다 위에 배치)
const GENERAL_PANELS = [
  { title: "Stream rate", note: "1.0 s message rate · CRSF is measured at the UART boundary",
    series: [
      { t: RATE, f: "video_hz", label: "video (raw) [Hz]" },
      { t: RATE, f: "ui_hz", label: "UI overlay [Hz]" },
      { t: RATE, f: "detection_hz", label: "detection [Hz]" },
      { t: RATE, f: "control_hz", label: "control [Hz]" },
      { t: IO, f: "tx_uart_write_hz", label: "CRSF TX UART [Hz]" },
      { t: IO, f: "rx_uart_valid_frame_hz", label: "CRSF RX UART [Hz]" },
    ] },
  { title: "Detection", note: "raw / filtered target count and selected target confidence",
    series: [
      { t: DR, f: "detections_count", label: "raw count" }, { t: D, f: "detections_count", label: "filtered count" },
      { t: D, f: "detections_0_confidence", label: "target confidence", scale: "y2" },
    ] },
  { title: "Detector diagnostics", note: "YOLO / HSV confidence and processing latency",
    series: [
      { t: DS, f: "data_0", label: "YOLO confidence" }, { t: DS, f: "data_1", label: "HSV confidence" },
      { t: DS, f: "data_10", label: "total [ms]", scale: "y2" },
    ] },
  { title: "Mission / target error", note: "state, lock, normalized image error",
    series: [
      { t: M, f: "error_x", label: "error_x" }, { t: M, f: "error_y", label: "error_y" },
      { t: M, f: "lock_elapsed_sec", label: "lock elapsed [s]", scale: "y2" },
      { t: M, f: "manual_mode", label: "mode (0=auto, 1=manual)", step: true, scale: "y2" },
    ] },
  { title: "Operator input", note: "sticks and AUTO/MANUAL / ARM / RTL switch inputs",
    series: [
      { t: CMD, f: "yaw", label: "yaw" }, { t: CMD, f: "throttle", label: "throttle" },
      { t: CMD, f: "roll", label: "roll" }, { t: CMD, f: "pitch", label: "pitch" },
      { t: CMD, f: "manual_mode", label: "mode (0=auto, 1=manual)", step: true },
      { t: CMD, f: "arm_switch", label: "arm switch", step: true },
      { t: CMD, f: "rtl_switch", label: "RTL switch", step: true },
    ] },
  { title: "Control output", note: "final command before CRSF normalization",
    series: [
      { t: C, f: "x", label: "roll rate [deg/s]" }, { t: C, f: "y", label: "pitch rate [deg/s]" },
      { t: C, f: "z", label: "thrust [0..1]", scale: "y2" },
    ] },
  { title: "PN closing speed", note: "solid: image-scale estimate · dashed: Gazebo world-pose ground truth · PN input is the value actually used",
    series: [
      { t: PN, f: "estimated_closing_speed_mps", label: "estimate [m/s]" },
      { t: PN, f: "applied_closing_speed_mps", label: "PN input [m/s]", color: "#fbbf24" },
      { t: PN_GT, f: "closing_speed_mps", label: "Gazebo GT [m/s]", color: "#f87171", dash: [5, 3] },
    ] },
  { title: "PN LOS rate", note: "solid: image + attitude estimate · dashed: Gazebo world-pose ground truth · NED [rad/s]",
    series: [
      { t: PN, f: "estimated_los_rate_x", label: "estimate north", color: "#60a5fa" },
      { t: PN, f: "estimated_los_rate_y", label: "estimate east", color: "#4ade80" },
      { t: PN, f: "estimated_los_rate_z", label: "estimate down", color: "#c084fc" },
      { t: PN_GT, f: "los_rate_x", label: "GT north", color: "#60a5fa", dash: [5, 3] },
      { t: PN_GT, f: "los_rate_y", label: "GT east", color: "#4ade80", dash: [5, 3] },
      { t: PN_GT, f: "los_rate_z", label: "GT down", color: "#c084fc", dash: [5, 3] },
    ] },
  { title: "PN command", note: "desired attitude / PN lateral acceleration / thrust",
    series: [
      { t: PN, f: "command_roll_deg", label: "roll [deg]" },
      { t: PN, f: "command_pitch_deg", label: "pitch [deg]" },
      { t: PN, f: "acceleration_pn_x", label: "aPN north [m/s²]", color: "#4ade80", dash: [5, 3], scale: "y2" },
      { t: PN, f: "acceleration_pn_y", label: "aPN east [m/s²]", color: "#c084fc", dash: [5, 3], scale: "y2" },
      { t: PN, f: "command_throttle", label: "throttle", color: "#fbbf24", scale: "y2" },
    ] },
  { title: "Attitude", note: "source-independent vehicle roll / pitch / yaw",
    series: [
      { t: A, f: "roll_deg", label: "roll [deg]" }, { t: A, f: "pitch_deg", label: "pitch [deg]" },
      { t: A, f: "yaw_deg", label: "yaw [deg]" },
    ] },
  { title: "GPS position", note: "latitude / longitude",
    series: [
      { t: G, f: "latitude_deg", label: "latitude" }, { t: G, f: "longitude_deg", label: "longitude", scale: "y2" },
    ] },
  { title: "GPS motion", note: "altitude / ground speed / course",
    series: [
      { t: G, f: "altitude_m", label: "altitude [m]" },
      { t: G, f: "ground_speed_mps", label: "ground speed [m/s]" },
      { t: G, f: "ground_course_deg", label: "course [deg]", scale: "y2" },
    ] },
  { title: "GPS satellites", note: "visible satellite count",
    series: [
      { t: G, f: "satellites", label: "satellites", step: true },
    ] },
  { title: "Battery", note: "pack voltage / current / remaining",
    series: [
      { t: B, f: "voltage_v", label: "voltage [V]" }, { t: B, f: "current_a", label: "current [A]" },
      { t: B, f: "battery_remaining_pct", label: "remaining [%]", scale: "y2" },
    ] },
  { title: "CRSF link", note: "uplink telemetry received from ELRS",
    series: [
      { t: R, f: "uplink_rssi_dbm", label: "uplink RSSI [dBm]" },
      { t: R, f: "uplink_lq", label: "uplink LQ [%]" }, { t: R, f: "uplink_snr_db", label: "uplink SNR [dB]" },
    ] },
  { title: "CRSF output", note: "channels sent to FC (CH6: auto=ACRO, manual/RTL=self-level)",
    series: [
      { t: TX, f: "data_0", label: "roll" }, { t: TX, f: "data_1", label: "pitch" },
      { t: TX, f: "data_2", label: "throttle" }, { t: TX, f: "data_4", label: "arm" },
      { t: TX, f: "data_5", label: "flight mode CH6 (172=auto, 1811=manual/RTL)" },
      { t: TX, f: "data_6", label: "rtl" },
    ] },
];

// ---------------------------------------------------------------- 패널 정의
// series: {t: 토픽(기본 T), f: 필드(또는 DERIVED 이름), scale: "y"|"y2", dash, width, step, show,
//          mul/add: 표시용 변환(범례에는 원값 표시)}
const PANELS = [
  { title: "Error", note: "dashed: raw screen-normalized ±0.5 · solid: filtered camera tan coordinate",
    series: [
      { f: "raw_err_x", dash: [4, 3], width: 1 }, { f: "filt_err_x" },
      { f: "raw_err_y", dash: [4, 3], width: 1 }, { f: "filt_err_y" },
    ] },
  { title: "Error angle", note: "[deg] · filtered camera tan-coordinate control input",
    series: [{ f: "err_angle_x" }, { f: "err_angle_y" }] },
  { title: "Lead rate", note: "[tan/s] · LPF of filtered-error derivative used by time-based lead",
    series: [{ f: "err_dot_x" }, { f: "err_dot_y" }] },
  { title: "Command vs attitude", note: "왼축 [deg/s] · 오른축 source-independent attitude [deg]",
    series: [
      { f: "roll_rate_cmd", dash: [4, 3], width: 1 }, { f: "roll_rate_out" },
      { f: "pitch_rate_cmd", dash: [4, 3], width: 1 }, { f: "pitch_rate_out" },
      { t: A, f: "roll_deg", scale: "y2", dash: [2, 3] }, { t: A, f: "pitch_deg", scale: "y2", dash: [2, 3] },
    ] },
  { title: "Scheduling scales", note: "0~1",
    series: [
      { f: "closeness" }, { f: "range_gain_scale" }, { f: "range_throttle_scale" },
      { f: "tau_ease" }, { f: "center_q" },
    ] },
  { title: "Looming / τ", note: "왼축 loom_size·loom_rate · 오른축 tau[s]",
    series: [
      { f: "loom_size" }, { f: "loom_rate" },
      { f: "tau", scale: "y2" },
    ] },
  { title: "Thrust", note: "0~1",
    series: [{ f: "thrust", width: 2 }, { f: "throttle_up", dash: [4, 3] }, { f: "emag", width: 1 }] },
  { title: "Flags", note: "0/1", flags: true,
    series: ["cmd_active", "filter_active", "detection_held", "rate_saturated", "manual_out"]
      .map((f, i) => ({ f, step: true, mul: 0.8, add: i })) },
  { title: "Control loop", note: "dt [ms]",
    series: [{ f: "dt", mul: 1000, width: 1 }] },
];

const PALETTE = ["#60a5fa", "#f87171", "#4ade80", "#fbbf24", "#c084fc", "#22d3ee", "#f472b6", "#a3e635"];
const STATE_COLORS = {
  SEARCH: "rgba(148,163,184,0.10)",
  LOCK: "rgba(234,179,8,0.13)",
  TRACK: "rgba(34,197,94,0.11)",
  FIRE: "rgba(239,68,68,0.18)",
  RTL: "rgba(59,130,246,0.13)",
};
const CHART_H = 210;

// ---------------------------------------------------------------- 상태
let rec = null;            // 현재 기록 데이터
let charts = [];           // uPlot 인스턴스
let stateSegs = [];        // [{t0, t1, state}]
let syncing = false;       // 줌 전파 재진입 방지
let records = [];
let currentId = null;
let customPanels = loadLS("flight_analyzer.custom", []);
let showHelp = loadLS("flight_analyzer.help", true);   // 필터 칩의 수식 표시
let layout = loadLS("flight_analyzer.layout", "graphs");  // "graphs" | "split"(영상 + 그래프)
let vmode = loadLS("flight_analyzer.vmode", "play");      // "play"(영상 재생) | "follow"(그래프 커서 따라감)
let sidebarCollapsed = loadLS("flight_analyzer.sidebar_collapsed", false);
// 영상 재생 상태
const V = { t: 0, playing: false, speed: 1, shown: -1, want: -1, loading: false, lastTs: 0, lastCursor: 0, topic: null };

const $ = (id) => document.getElementById(id);

function loadLS(k, d) { try { return JSON.parse(localStorage.getItem(k)) ?? d; } catch { return d; } }
function saveLS(k, v) { try { localStorage.setItem(k, JSON.stringify(v)); } catch { /* 무시 */ } }

async function api(path, opts) {
  const r = await fetch(path, opts);
  const j = await r.json().catch(() => ({}));
  if (!r.ok) throw new Error(j.error || `${r.status} ${r.statusText}`);
  return j;
}

function fmtDur(s) {
  if (!isFinite(s)) return "—";
  const m = Math.floor(s / 60);
  return m ? `${m}분 ${(s % 60).toFixed(0)}초` : `${s.toFixed(1)}초`;
}
function fmtSize(b) {
  if (b > 1e9) return (b / 1e9).toFixed(1) + " GB";
  if (b > 1e6) return (b / 1e6).toFixed(1) + " MB";
  return (b / 1e3).toFixed(0) + " KB";
}
function fmtVal(v) {
  if (v == null || !isFinite(v)) return "—";
  const a = Math.abs(v);
  if (a !== 0 && (a < 1e-3 || a >= 1e5)) return v.toExponential(2);
  return a >= 100 ? v.toFixed(1) : a >= 10 ? v.toFixed(2) : v.toFixed(4);
}
function cssVar(n) { return getComputedStyle(document.documentElement).getPropertyValue(n).trim(); }

// ---------------------------------------------------------------- 기록 목록
async function loadRecords() {
  const j = await api("/api/records");
  records = j.records;
  renderRecords();
  const want = decodeURIComponent(location.hash.slice(1));
  if (want && records.some((r) => r.id === want) && want !== currentId) openRecord(want);
}
function renderRecords() {
  const q = $("rec-filter").value.trim().toLowerCase();
  const ul = $("rec-list");
  ul.innerHTML = "";
  const list = records.filter((r) => !q || r.name.toLowerCase().includes(q));
  if (!list.length) {
    ul.innerHTML = `<li class="empty">${records.length ? "검색 결과 없음" : "이 폴더에 기록(.db3)이 없음"}</li>`;
    return;
  }
  for (const r of list) {
    const li = document.createElement("li");
    li.className = (r.id === currentId ? "active " : "") + (r.control_count ? "" : "no-control-debug");
    li.innerHTML = `<div class="rec-name"></div><div class="rec-sub"></div>`;
    li.children[0].textContent = r.name;
    const sub = li.children[1];
    const parts = [
      r.start ? r.start.replace("T", " ").slice(5, 16) : null,
      fmtDur(r.duration), fmtSize(r.size),
    ].filter(Boolean);
    for (const p of parts) { const s = document.createElement("span"); s.textContent = p; sub.appendChild(s); }
    const s = document.createElement("span");
    if (r.control_count) s.textContent = `control ${r.control_count}`;
    else { s.textContent = "control diagnostics 없음"; s.className = "warn"; }
    sub.appendChild(s);
    li.onclick = () => openRecord(r.id);
    ul.appendChild(li);
  }
}

// ---------------------------------------------------------------- 기록 열기
async function openRecord(id) {
  currentId = id;
  history.replaceState(null, "", "#" + encodeURIComponent(id));
  renderRecords();
  $("rec-title").textContent = records.find((r) => r.id === id)?.name ?? id;
  $("rec-meta").textContent = "불러오는 중…";
  $("errors").innerHTML = "";
  try {
    rec = await api("/api/record?id=" + encodeURIComponent(id));
  } catch (e) {
    $("rec-meta").textContent = "";
    $("errors").innerHTML = `<div></div>`;
    $("errors").firstChild.textContent = "불러오기 실패: " + e.message;
    return;
  }
  if (currentId !== id) return;   // 로딩 중 다른 기록을 눌렀음
  const topics = Object.keys(rec.topics);
  $("rec-meta").textContent =
    `${fmtDur(rec.duration)} · 토픽 ${topics.length}개 · control diagnostics ${rec.topics[T]?.t.length ?? 0}개`;
  $("errors").innerHTML = "";
  for (const e of rec.errors || []) {
    const d = document.createElement("div");
    d.textContent = "역직렬화 실패(건너뜀): " + e;
    $("errors").appendChild(d);
  }
  if (!rec.topics[T]) {
    const d = document.createElement("div");
    d.textContent = "이 기록에는 /arms/control/diagnostics 가 없습니다. 다른 토픽은 아래 '패널 추가'로 볼 수 있습니다.";
    $("errors").appendChild(d);
  }
  stateSegs = buildStateSegs();
  renderStateLegend();
  buildStreamRates(rec.video?.topic);
  fillFieldList();
  renderPanels();
  initScene();
  buildScenePath();
  loadBalloon();
  resetVideo();
  updateScene(0);
}

function buildStateSegs() {
  const src = rec.topics[T]?.strings?.state ? rec.topics[T] : rec.topics["/arms/mission/state"];
  const st = src?.strings?.state;
  if (!st) return [];
  const segs = [];
  let cur = null;
  for (let i = 0; i < st.length; i++) {
    if (!cur || cur.state !== st[i]) {
      if (cur) cur.t1 = src.t[i];
      cur = { state: st[i], t0: src.t[i], t1: src.t[i] };
      segs.push(cur);
    }
  }
  if (cur) cur.t1 = src.t[src.t.length - 1];
  return segs;
}
function stateAt(t) {
  let lo = 0, hi = stateSegs.length - 1, ans = null;
  while (lo <= hi) {
    const m = (lo + hi) >> 1;
    if (stateSegs[m].t0 <= t) { ans = stateSegs[m]; lo = m + 1; } else hi = m - 1;
  }
  return ans?.state ?? "";
}

// ---------------------------------------------------------------- GPS/CRSF 동기화 3D 뷰
// 좌표는 기록 첫 GPS를 원점으로 한 ENU 근사: X=east, Y=up, Z=north [m].
const S3 = { renderer: null, scene: null, camera: null, drone: null, balloon: null,
  path: null, origin: null, gps: null, resize: null, target: null,
  radius: 30, theta: 0, phi: 1.1, pointer: null,
  focus: loadLS("flight_analyzer.scene_focus", "drone") };
const deg = (v) => (v || 0) * Math.PI / 180;

function nearestAt(d, field, t) {
  const col = d?.fields?.[field];
  if (!col || !d.t?.length) return null;
  let lo = 0, hi = d.t.length - 1;
  while (lo < hi) { const m = (lo + hi) >> 1; if (d.t[m] < t) lo = m + 1; else hi = m; }
  const i = lo > 0 && Math.abs(d.t[lo - 1] - t) < Math.abs(d.t[lo] - t) ? lo - 1 : lo;
  return col[i] == null ? null : Number(col[i]);
}
function enu(lat, lon, alt) {
  if (!S3.origin || !isFinite(lat) || !isFinite(lon) || !isFinite(alt)) return null;
  const r = Math.PI / 180, o = S3.origin;
  return new THREE.Vector3(
    (lon - o.lon) * 111320 * Math.cos(o.lat * r),
    alt - o.alt,
    (lat - o.lat) * 110540,
  );
}
function gpsAt(t) {
  const g = rec?.topics?.[G];
  if (!g) return null;
  return enu(nearestAt(g, "latitude_deg", t), nearestAt(g, "longitude_deg", t), nearestAt(g, "altitude_m", t));
}
function applySceneCamera() {
  if (!S3.camera || !S3.target) return;
  const sinPhi = Math.sin(S3.phi);
  S3.camera.position.set(
    S3.target.x + S3.radius * sinPhi * Math.sin(S3.theta),
    S3.target.y + S3.radius * Math.cos(S3.phi),
    S3.target.z + S3.radius * sinPhi * Math.cos(S3.theta),
  );
  S3.camera.lookAt(S3.target);
}
function setSceneCamera(target, radius = null) {
  if (!S3.camera) return;
  S3.target = target.clone();
  const offset = S3.camera.position.clone().sub(S3.target);
  S3.radius = radius ?? Math.max(1, offset.length());
  S3.theta = Math.atan2(offset.x, offset.z);
  S3.phi = Math.acos(Math.max(-.98, Math.min(.98, offset.y / Math.max(.001, offset.length()))));
  if (radius != null) { S3.theta = Math.PI / 4; S3.phi = 1.05; }
  applySceneCamera();
}
function updateSceneFocusControls() {
  document.querySelectorAll("#scene-focus button").forEach((b) => b.classList.toggle("on", b.dataset.focus === S3.focus));
}
function setSceneFocus(focus) {
  S3.focus = focus === "origin" ? "origin" : "drone";
  saveLS("flight_analyzer.scene_focus", S3.focus);
  updateSceneFocusControls();
  if (!window.THREE || !S3.target) return;
  if (S3.focus === "origin") S3.target.set(0, 0, 0);
  else if (S3.drone?.visible) S3.target.copy(S3.drone.position);
  applySceneCamera(); renderScene();
}
function initScene() {
  const host = $("scene-canvas");
  if (!window.THREE || !host) return;
  if (S3.renderer) { S3.renderer.dispose(); host.innerHTML = ""; }
  const width = Math.max(1, host.clientWidth), height = Math.max(1, host.clientHeight);
  S3.renderer = new THREE.WebGLRenderer({ antialias: true });
  S3.renderer.setPixelRatio(Math.min(devicePixelRatio, 2));
  S3.renderer.setSize(width, height); host.appendChild(S3.renderer.domElement);
  S3.scene = new THREE.Scene(); S3.scene.background = new THREE.Color(0x090c10);
  S3.camera = new THREE.PerspectiveCamera(45, width / height, 0.1, 50000);
  S3.camera.position.set(20, 16, 22); S3.camera.lookAt(0, 0, 0);
  setSceneCamera(new THREE.Vector3(), null);
  S3.scene.add(new THREE.HemisphereLight(0xbfd7ff, 0x202a32, 2.2));
  const grid = new THREE.GridHelper(40, 20, 0x506070, 0x25303a); S3.scene.add(grid);
  const worldAxes = new THREE.AxesHelper(4); S3.scene.add(worldAxes);
  S3.drone = new THREE.Group();
  S3.drone.add(new THREE.Mesh(new THREE.SphereGeometry(.55, 20, 14), new THREE.MeshStandardMaterial({ color: 0x60a5fa })));
  const axes = [[0xff5555, new THREE.Vector3(1, 0, 0)], [0x55dd77, new THREE.Vector3(0, 1, 0)], [0x55aaff, new THREE.Vector3(0, 0, 1)]];
  axes.forEach(([color, dir]) => S3.drone.add(new THREE.ArrowHelper(dir, new THREE.Vector3(), 1.5, color, .3, .18)));
  S3.scene.add(S3.drone);
  S3.balloon = new THREE.Mesh(new THREE.SphereGeometry(.45, 18, 14), new THREE.MeshStandardMaterial({ color: 0xef4444 }));
  S3.balloon.visible = false; S3.scene.add(S3.balloon);
  const canvas = S3.renderer.domElement;
  canvas.addEventListener("pointerdown", (e) => {
    S3.pointer = { id: e.pointerId, x: e.clientX, y: e.clientY };
    canvas.setPointerCapture(e.pointerId);
  });
  canvas.addEventListener("pointermove", (e) => {
    if (!S3.pointer || e.pointerId !== S3.pointer.id) return;
    const dx = e.clientX - S3.pointer.x, dy = e.clientY - S3.pointer.y;
    S3.pointer.x = e.clientX; S3.pointer.y = e.clientY;
    S3.theta -= dx * 0.012;
    S3.phi = Math.max(0.08, Math.min(Math.PI - 0.08, S3.phi - dy * 0.012));
    applySceneCamera(); renderScene();
  });
  const releasePointer = (e) => {
    if (S3.pointer?.id !== e.pointerId) return;
    S3.pointer = null;
    if (canvas.hasPointerCapture(e.pointerId)) canvas.releasePointerCapture(e.pointerId);
  };
  canvas.addEventListener("pointerup", releasePointer);
  canvas.addEventListener("pointercancel", releasePointer);
  canvas.addEventListener("wheel", (e) => {
    e.preventDefault();
    // 지수 스케일이라 트랙패드/마우스 휠 모두 자연스럽게 동작한다.
    S3.radius = Math.max(1, Math.min(50000, S3.radius * Math.exp(e.deltaY * 0.001)));
    applySceneCamera(); renderScene();
  }, { passive: false });
  S3.resize?.disconnect();
  S3.resize = new ResizeObserver(() => { if (!S3.renderer) return; const w = Math.max(1, host.clientWidth), h = Math.max(1, host.clientHeight); S3.camera.aspect = w / h; S3.camera.updateProjectionMatrix(); S3.renderer.setSize(w, h); renderScene(); });
  S3.resize.observe(host);
}
function buildScenePath() {
  const gps = rec?.topics?.[G]; S3.gps = gps || null; S3.origin = null;
  if (!window.THREE || !S3.scene) return;
  if (!gps?.t?.length) return;
  const lat = gps.fields.latitude_deg, lon = gps.fields.longitude_deg, alt = gps.fields.altitude_m;
  if (!lat || !lon || !alt) return;
  const i0 = lat.findIndex((v, i) => v != null && lon[i] != null && alt[i] != null &&
    isFinite(v) && isFinite(lon[i]) && isFinite(alt[i]));
  if (i0 < 0) return;
  S3.origin = { lat: Number(lat[i0]), lon: Number(lon[i0]), alt: Number(alt[i0]) };
  const pts = [];
  for (let i = 0; i < gps.t.length; i++) { const p = enu(lat[i], lon[i], alt[i]); if (p) pts.push(p); }
  if (pts.length > 1) {
    S3.path?.removeFromParent();
    S3.path = new THREE.Line(new THREE.BufferGeometry().setFromPoints(pts), new THREE.LineBasicMaterial({ color: 0x60a5fa }));
    S3.scene.add(S3.path);
    const box = new THREE.Box3().setFromPoints(pts); const center = box.getCenter(new THREE.Vector3());
    const span = Math.max(8, box.getSize(new THREE.Vector3()).length());
    setSceneCamera(center, span * 1.45);
  }
}
function setBalloonFromInputs() {
  if (!S3.balloon) return;
  const lat = parseFloat($("balloon-lat").value), lon = parseFloat($("balloon-lon").value), alt = parseFloat($("balloon-alt").value);
  const p = enu(lat, lon, alt); S3.balloon.visible = !!p; if (p) S3.balloon.position.copy(p); renderScene();
}
function loadBalloon() {
  const b = rec?.manifest?.balloon || {};
  $("balloon-lat").value = b.latitude_deg ?? ""; $("balloon-lon").value = b.longitude_deg ?? ""; $("balloon-alt").value = b.altitude_m ?? "";
  setBalloonFromInputs();
}
function updateScene(t) {
  if (!S3.scene || !rec) return;
  const pos = gpsAt(t); const att = rec.topics[A];
  if (pos) {
    S3.drone.visible = true; S3.drone.position.copy(pos);
    // Source-independent attitude: yaw=heading, roll/pitch=body tilt.
    const roll = nearestAt(att, "roll_deg", t), pitch = nearestAt(att, "pitch_deg", t), yaw = nearestAt(att, "yaw_deg", t);
    S3.drone.rotation.order = "YXZ";
    S3.drone.rotation.set(deg(pitch), -deg(yaw), deg(roll));
    if (S3.focus === "drone") { S3.target.copy(pos); applySceneCamera(); }
    const a = (v) => v == null ? "—" : v.toFixed(0) + "°";
    $("scene-info").textContent = `t=${t.toFixed(2)}s  E ${pos.x.toFixed(1)} N ${pos.z.toFixed(1)} U ${pos.y.toFixed(1)}m · R ${a(roll)} P ${a(pitch)} Y ${a(yaw)}`;
  } else {
    S3.drone.visible = false; $("scene-info").textContent = "GPS 위치 없음";
  }
  renderScene();
}
function renderScene() { if (S3.renderer && S3.scene && S3.camera) S3.renderer.render(S3.scene, S3.camera); }
async function saveBalloon() {
  const balloon = { latitude_deg: parseFloat($("balloon-lat").value), longitude_deg: parseFloat($("balloon-lon").value), altitude_m: parseFloat($("balloon-alt").value) };
  if (Object.values(balloon).some((v) => !isFinite(v))) { $("scene-info").textContent = "풍선 위도·경도·고도를 입력하세요"; return; }
  try {
    const j = await api("/api/manifest/balloon", { method: "POST", headers: { "Content-Type": "application/json" }, body: JSON.stringify({ id: currentId, balloon }) });
    rec.manifest = rec.manifest || {}; rec.manifest.balloon = j.balloon; setBalloonFromInputs();
  } catch (e) { $("scene-info").textContent = "풍선 저장 실패: " + e.message; }
}
function renderStateLegend() {
  const el = $("state-legend");
  el.innerHTML = "";
  if (!stateSegs.length) return;
  const present = new Set(stateSegs.map((s) => s.state));
  for (const [k, c] of Object.entries(STATE_COLORS)) {
    if (!present.has(k)) continue;
    const s = document.createElement("span");
    s.innerHTML = `<i style="background:${c.replace(/[\d.]+\)$/, "0.55)")}"></i>${k}`;
    el.appendChild(s);
  }
}

function fillFieldList() {
  const dl = $("field-list");
  dl.innerHTML = "";
  for (const [tp, d] of Object.entries(rec.topics)) {
    const extra = tp === T ? Object.keys(DERIVED) : [];
    for (const f of [...Object.keys(d.fields), ...extra]) {
      const o = document.createElement("option");
      o.value = `${tp} ${f}`;
      dl.appendChild(o);
    }
  }
  $("custom-add").hidden = false;
}

/* 각 스트림의 bag 기록 시각으로부터 1초 이동 윈도우 메시지율을 만든다.
   header stamp 대신 bag timestamp를 쓰므로, 실제로 recorder가 받은 빈도를 보여 준다. */
function rollingRate(times, grid) {
  if (!times?.length) return null;
  const out = [];
  let first = 0, after = 0;
  for (const t of grid) {
    while (after < times.length && times[after] <= t) after++;
    while (first < after && times[first] <= t - RATE_WINDOW_SEC) first++;
    out.push((after - first) / RATE_WINDOW_SEC);
  }
  return out;
}

const RAW_VIDEO_TOPIC = "/arms/camera/image_raw/compressed";
const UI_VIDEO_TOPIC = "/arms/ui/image/compressed";

function buildStreamRates() {
  if (!rec) return;
  const duration = rec.duration || 0;
  const grid = [];
  for (let t = 0; t <= duration + 1e-9; t += RATE_STEP_SEC) grid.push(Number(t.toFixed(6)));
  if (!grid.length) grid.push(0);
  const detection = rec.topics[D] || rec.topics[DR];
  const control = rec.topics[C] || rec.topics[T];
  const fields = {
    video_hz: rollingRate(rec.videos?.[RAW_VIDEO_TOPIC]?.t, grid),
    ui_hz: rollingRate(rec.videos?.[UI_VIDEO_TOPIC]?.t, grid),
    detection_hz: rollingRate(detection?.t, grid),
    control_hz: rollingRate(control?.t, grid),
  };
  // 없는 스트림은 null 열로 남겨 패널에 존재하지 않는 선으로 표시한다.
  for (const [name, values] of Object.entries(fields)) {
    if (!values) fields[name] = Array(grid.length).fill(null);
  }
  rec.topics[RATE] = { type: "viewer/StreamRates", t: grid, fields, strings: {} };
}

// ---------------------------------------------------------------- 차트
function destroyCharts() {
  charts.forEach((c) => c.destroy());
  charts = [];
}

function renderPanels() {
  destroyCharts();
  $("empty").hidden = true;
  const general = $("general-panels");
  const control = $("control-panels");
  general.innerHTML = "";
  control.innerHTML = "";
  for (const p of GENERAL_PANELS) if (p.series.some((s) => fieldCol(s.t || T, s.f))) general.appendChild(makePanel(p));
  for (const p of [...PANELS, ...customPanels.map((p, i) => ({ ...p, custom: i }))]) {
    control.appendChild(makePanel(p));
  }
}

function makePanel(p) {
  const det = document.createElement("section");
  det.className = "panel";
  const head = document.createElement("div");
  head.className = "panel-head";
  const title = document.createElement("div");
  title.className = "panel-title";
  title.textContent = p.title;
  const note = document.createElement("div");
  note.className = "note";
  note.textContent = p.note || "";
  head.append(title, note);
  if (p.custom != null) {
    const x = document.createElement("button");
    x.className = "btn small panel-delete";
    x.textContent = "삭제";
    x.onclick = () => {
      customPanels.splice(p.custom, 1);
      saveLS("flight_analyzer.custom", customPanels);
      renderPanels();
    };
    head.appendChild(x);
  }
  det.appendChild(head);
  const body = document.createElement("div");
  det.appendChild(body);

  const built = buildData(p);
  if (!built) {
    body.innerHTML = '<div class="missing">이 기록에 해당 필드가 없음</div>';
    return det;
  }
  const help = makeHelp(p, built);
  body.appendChild(help);
  const chartEl = document.createElement("div");
  body.appendChild(chartEl);
  const chart = makeChart(p, built, chartEl);
  bindToggles(p, built.specs, help, chart);
  return det;
}

function seriesName(s, multiTopic) {
  if (s.label) return s.label;
  return multiTopic && s.t !== T ? `${s.t.split("/").pop()}.${s.f}` : s.f;
}
function fieldCol(tp, f) {
  const d = rec.topics[tp];
  if (!d) return null;
  if (d.fields[f]) return d.fields[f];
  const dv = DERIVED[f];
  if (!dv || dv.deps.some((k) => !d.fields[k])) return null;
  const cols = dv.deps.map((k) => d.fields[k]);
  return d.t.map((_, i) => {
    const args = cols.map((c) => c[i]);
    return args.some((v) => v == null) ? null : dv.fn(...args);
  });
}
function seriesColor(s, i) { return s.color || PALETTE[i % PALETTE.length]; }

/* 패널 위 필터 칩: [체크박스][필드명][수식] 을 작은 버튼으로 나열. 칩 클릭 = 선 켜기/끄기. */
function makeHelp(p, { specs, multiTopic }) {
  const box = document.createElement("div");
  box.className = "help";
  specs.forEach((s, i) => {
    const chip = document.createElement("label");
    chip.className = "chip";
    chip.innerHTML = `<input type="checkbox" class="cb"><span class="nm mono"></span><span class="ds mono"></span>`;
    chip.querySelector(".cb").checked = seriesShown(p, s);
    chip.querySelector(".nm").textContent = seriesName(s, multiTopic) + (s.scale === "y2" ? " (R)" : "");
    chip.querySelector(".ds").textContent = (FIELD_INFO[s.f] ?? "") + (DERIVED[s.f] ? " ⚙" : "");
    chip.title = `${s.f} = ${FIELD_INFO[s.f] ?? ""}${DERIVED[s.f] ? "  (뷰어 계산)" : ""}`;
    chip.style.setProperty("--series", seriesColor(s, i));
    box.appendChild(chip);
  });
  return box;
}

/* 선 표시 여부 — 패널·필드별로 브라우저에 기억. 기본은 패널 정의의 show(없으면 켬). */
const showKey = (p, s) => `flight_analyzer.show.${p.title}.${s.t || T}.${s.f}`;
function seriesShown(p, s) { return loadLS(showKey(p, s), s.show ?? true); }

/* 수식표 체크박스 ↔ 차트 선 표시 연동 (범례 클릭으로 바꿔도 체크박스가 따라간다). */
function bindToggles(p, specs, box, u) {
  const cbs = box.querySelectorAll(".cb");
  cbs.forEach((cb, i) => {
    cb.onchange = () => u.setSeries(i + 1, { show: cb.checked });
  });
  u.hooks.setSeries = u.hooks.setSeries || [];
  u.hooks.setSeries.push((u, idx, opts) => {
    if (idx == null || opts.show == null || !cbs[idx - 1]) return;
    cbs[idx - 1].checked = opts.show;
    saveLS(showKey(p, specs[idx - 1]), opts.show);
  });
}

/* 패널의 시리즈들을 하나의 x축으로 정렬. 토픽이 여러 개면 uPlot.join 으로 합친다. */
function buildData(p) {
  const groups = new Map();   // topic → [spec]
  for (const s of p.series) {
    const tp = s.t || T;
    if (!fieldCol(tp, s.f)) continue;
    if (!groups.has(tp)) groups.set(tp, []);
    groups.get(tp).push(s);
  }
  if (!groups.size) return null;
  const tables = [], specs = [];
  for (const [tp, ss] of groups) {
    const d = rec.topics[tp];
    const cols = ss.map((s) => {
      const src = fieldCol(tp, s.f);
      if (s.mul == null && s.add == null) return src;
      const mul = s.mul ?? 1, add = s.add ?? 0;
      return src.map((v) => (v == null ? null : v * mul + add));
    });
    tables.push([d.t, ...cols]);
    specs.push(...ss.map((s) => ({ ...s, t: tp })));
  }
  const data = tables.length === 1
    ? tables[0]
    : uPlot.join(tables, tables.map((t) => t.slice(1).map(() => 1)));   // 1 = 원래 null(끊김)은 유지
  return { data, specs, multiTopic: groups.size > 1 };
}

function chartWidth() {
  return Math.max(300, $("graphs").clientWidth - 22);
}

function makeChart(p, { data, specs, multiTopic }, el) {
  const axisStroke = cssVar("--axis"), grid = cssVar("--grid");
  const hasY2 = specs.some((s) => s.scale === "y2");
  const series = [{ label: "t[s]", value: (u, v) => (v == null ? "—" : v.toFixed(3)) }];
  specs.forEach((s, i) => {
    const mul = s.mul ?? 1, add = s.add ?? 0;
    series.push({
      label: seriesName(s, multiTopic),
      scale: s.scale || "y",
      stroke: seriesColor(s, i),
      width: s.width ?? 1.5,
      dash: s.dash,
      show: seriesShown(p, s),
      spanGaps: multiTopic,
      paths: s.step ? uPlot.paths.stepped({ align: 1 }) : undefined,
      points: { show: false },
      value: (u, v) => (v == null ? "—" : p.flags ? String(Math.round((v - add) / mul)) : fmtVal((v - add) / mul)),
    });
  });
  const dur = rec.duration || 1;
  const axes = [
    { stroke: axisStroke, grid: { stroke: grid }, ticks: { stroke: grid } },
    p.flags
      ? { stroke: axisStroke, grid: { show: false }, ticks: { show: false }, size: 110,
          splits: () => specs.map((s) => (s.add ?? 0) + 0.4),   // 각 플래그 줄 가운데에 이름
          values: () => specs.map((s) => seriesName(s, false)) }
      : { stroke: axisStroke, grid: { stroke: grid }, ticks: { stroke: grid }, size: 60 },
  ];
  if (hasY2) axes.push({ scale: "y2", side: 1, stroke: axisStroke, grid: { show: false }, size: 50 });

  const opts = {
    width: chartWidth(),
    height: CHART_H,
    series,
    axes,
    scales: {
      x: { time: false, auto: false, range: (u, mn, mx) => [mn ?? 0, mx ?? dur] },
      ...(p.flags ? { y: { auto: false, range: [-0.2, p.series.length] } } : {}),
    },
    legend: { live: true },
    cursor: {
      sync: { key: "arms", setSeries: false },
      drag: { x: true, y: false, setScale: true },
      bind: { dblclick: () => () => { resetZoom(); return null; } },
      points: { size: 6 },
    },
    plugins: [statePlugin()],
    hooks: {
      setScale: [(u, key) => {
        if (key !== "x" || syncing) return;
        syncing = true;
        const { min, max } = u.scales.x;
        for (const c of charts) if (c !== u) c.setScale("x", { min, max });
        syncing = false;
      }],
      setCursor: [(u) => {
        const { left, idx } = u.cursor;
        if (left == null || left < 0 || idx == null) return;
        const t = u.posToVal(left, "x");
        if (vmode === "follow") {
          if (videoOn()) showFrameAt(t);
          else updateScene(t);
        }
      }],
    },
  };
  const u = new uPlot(opts, data, el);
  // 범례 이름에 마우스를 올리면 설명이 뜨게 한다
  u.root.querySelectorAll(".u-legend tr.u-series").forEach((tr, i) => {
    const s = specs[i - 1];
    if (s) tr.title = `${s.f} = ${FIELD_INFO[s.f] ?? ""}`;
  });
  // Play 모드에서 그래프를 클릭하면 영상이 그 시각으로 이동한다(드래그 확대와는 별개).
  let downX = null;
  u.over.addEventListener("mousedown", (e) => { downX = e.clientX; });
  u.over.addEventListener("mouseup", (e) => {
    if (downX == null || Math.abs(e.clientX - downX) > 3) return;
    if (videoOn() && vmode === "play" && u.cursor.left >= 0) seekVideo(u.posToVal(u.cursor.left, "x"));
  });
  u.setScale("x", { min: 0, max: dur });
  // 방금 만든 차트를 다른 차트들의 현재 줌에 맞춘다
  if (charts.length) {
    const { min, max } = charts[0].scales.x;
    u.setScale("x", { min, max });
  }
  charts.push(u);
  return u;
}

/* 상태머신 상태(LOCK/TRACK/FIRE…)를 배경 띠로 칠한다. */
function statePlugin() {
  return {
    hooks: {
      drawClear: [(u) => {
        if (!stateSegs.length) return;
        const { ctx } = u;
        const { left, top, width, height } = u.bbox;
        ctx.save();
        for (const s of stateSegs) {
          const c = STATE_COLORS[s.state];
          if (!c) continue;
          const x0 = Math.max(u.valToPos(s.t0, "x", true), left);
          const x1 = Math.min(u.valToPos(s.t1, "x", true), left + width);
          if (x1 <= x0) continue;
          ctx.fillStyle = c;
          ctx.fillRect(x0, top, x1 - x0, height);
        }
        ctx.restore();
      }],
    },
  };
}

function resetZoom() {
  if (!rec) return;
  syncing = true;
  for (const c of charts) c.setScale("x", { min: 0, max: rec.duration || 1 });
  syncing = false;
}

function addCustom() {
  const raw = $("custom-input").value.trim();
  if (!raw || !rec) return;
  const series = [];
  for (const part of raw.split(",")) {
    const bits = part.trim().split(/\s+/);
    if (!bits[0]) continue;
    const [tp, f] = bits.length >= 2 ? bits : [T, bits[0]];
    if (!fieldCol(tp, f)) {
      alert(`필드 없음: ${tp} ${f}`);
      return;
    }
    series.push({ t: tp, f });
  }
  if (!series.length) return;
  customPanels.push({ title: "Custom: " + series.map((s) => s.f).join(", "), series });
  saveLS("flight_analyzer.custom", customPanels);
  $("custom-input").value = "";
  renderPanels();
  $("control-panels").lastElementChild?.scrollIntoView({ behavior: "smooth", block: "center" });
}

// ---------------------------------------------------------------- 화면 영상
const videoData = () => rec?.videos?.[V.topic] || rec?.video || null;
const hasVideo = () => !!(videoData() && videoData().t.length);
const videoOn = () => layout === "split" && hasVideo();

function frameIndexAt(t) {          // t 이하인 마지막 프레임
  const ts = videoData().t;
  let lo = 0, hi = ts.length - 1, ans = 0;
  while (lo <= hi) {
    const m = (lo + hi) >> 1;
    if (ts[m] <= t) { ans = m; lo = m + 1; } else hi = m - 1;
  }
  return ans;
}

/* 프레임을 한 장씩 받아 바꾼다. 받는 중에 다른 요청이 오면 마지막 것만 남긴다(밀림 방지). */
function showFrameAt(t) {
  if (!hasVideo()) return;
  updateScene(t);
  V.want = frameIndexAt(t);
  updateVideoInfo(t);
  pumpFrame();
}
function pumpFrame() {
  if (V.loading || V.want === V.shown || V.want < 0) return;
  const i = V.want, id = currentId;
  V.loading = true;
  const img = new Image();
  img.onload = () => {
    V.loading = false;
    if (id !== currentId) return;
    $("v-img").src = img.src;
    V.shown = i;
    pumpFrame();
  };
  img.onerror = () => { V.loading = false; V.shown = i; pumpFrame(); };
  img.src = `/api/frame?id=${encodeURIComponent(id)}&topic=${encodeURIComponent(V.topic || "")}&i=${i}`;
}
function updateVideoInfo(t) {
  if (!hasVideo()) return;
  const st = stateAt(t);
  $("v-info").textContent =
    `t = ${t.toFixed(2)} s · frame ${V.want + 1}/${videoData().t.length}${st ? " · " + st : ""}`;
}

/* 재생 위치를 모든 그래프의 커서로 표시한다.
 * uPlot의 cursor sync는 포인터 이벤트는 전파하지만, setCursor()로 넣은 재생 위치는
 * 다른 차트로 전파하지 않는다. 따라서 각 차트의 동일한 시간 좌표에 직접 둔다. */
function cursorToTime(t) {
  const first = charts[0];
  if (!first) return;
  const { min, max } = first.scales.x;
  if (t < min || t > max) return;
  for (const u of charts) {
    u.setCursor({
      left: u.valToPos(t, "x"),
      top: u.bbox.height / devicePixelRatio / 2,
    });
  }
}

function seekVideo(t) {
  if (!hasVideo()) return;
  V.t = Math.min(Math.max(t, 0), rec.duration);
  $("v-seek").value = V.t;
  showFrameAt(V.t);
  cursorToTime(V.t);
}
function setPlaying(on) {
  V.playing = on && hasVideo();
  $("v-play").textContent = V.playing ? "❚❚" : "▶";
  if (V.playing) {
    if (V.t >= rec.duration - 1e-3) V.t = 0;
    V.lastTs = performance.now();
    requestAnimationFrame(tick);
  }
}
function tick(now) {
  if (!V.playing || !videoOn() || vmode !== "play") { setPlaying(false); return; }
  V.t += (now - V.lastTs) / 1000 * V.speed;
  V.lastTs = now;
  if (V.t >= rec.duration) { V.t = rec.duration; setPlaying(false); }
  $("v-seek").value = V.t;
  showFrameAt(V.t);
  if (now - V.lastCursor > 50) { cursorToTime(V.t); V.lastCursor = now; }   // 그래프 커서는 20Hz
  if (V.playing) requestAnimationFrame(tick);
}

function resetVideo() {
  setPlaying(false);
  V.t = 0; V.shown = -1; V.want = -1;
  V.topic = rec?.video?.topic || Object.keys(rec?.videos || {})[0] || null;
  const source = $("v-source");
  source.innerHTML = "";
  for (const name of Object.keys(rec?.videos || {})) {
    const option = document.createElement("option");
    option.value = option.textContent = name;
    option.selected = name === V.topic;
    source.appendChild(option);
  }
  source.hidden = source.options.length < 2;
  updateMp4Download();
  $("v-img").removeAttribute("src");
  $("v-info").textContent = "";
  const ok = hasVideo();
  $("v-seek").max = rec ? (rec.duration || 1) : 1;
  $("v-seek").value = 0;
  $("v-empty").textContent = !rec ? "기록을 선택하세요"
    : ok ? "" : "이 기록에는 compressed 영상이 없습니다";
  $("v-play").disabled = $("v-seek").disabled = !ok;
  if (ok && layout === "split") showFrameAt(0);
}

function updateMp4Download() {
  const link = $("v-download");
  const ok = hasVideo() && currentId;
  link.hidden = !ok;
  if (ok) link.href = `/api/mp4?id=${encodeURIComponent(currentId)}&topic=${encodeURIComponent(V.topic || "")}`;
}

function applyLayout() {
  document.body.classList.toggle("split", layout === "split");
  document.body.classList.toggle("follow", vmode === "follow");
  $("video-pane").hidden = layout !== "split";
  document.querySelectorAll("#layout button").forEach((b) => b.classList.toggle("on", b.dataset.l === layout));
  document.querySelectorAll("#vmode button").forEach((b) => b.classList.toggle("on", b.dataset.m === vmode));
  if (layout !== "split" || vmode !== "play") setPlaying(false);
  if (videoOn() && V.shown < 0) showFrameAt(V.t);
}

function applySidebar() {
  document.body.classList.toggle("sidebar-collapsed", sidebarCollapsed);
  $("btn-sidebar").setAttribute("aria-expanded", String(!sidebarCollapsed));
}

// ---------------------------------------------------------------- 시작
function bind() {
  $("btn-sidebar").onclick = () => {
    sidebarCollapsed = !sidebarCollapsed;
    saveLS("flight_analyzer.sidebar_collapsed", sidebarCollapsed);
    applySidebar();
  };
  document.querySelectorAll("#layout button").forEach((b) => b.onclick = () => {
    layout = b.dataset.l; saveLS("flight_analyzer.layout", layout); applyLayout();
  });
  document.querySelectorAll("#vmode button").forEach((b) => b.onclick = () => {
    vmode = b.dataset.m; saveLS("flight_analyzer.vmode", vmode); applyLayout();
    if (vmode === "play") {
      if (videoOn()) showFrameAt(V.t);
      else updateScene(V.t);
    }
  });
  $("v-play").onclick = () => setPlaying(!V.playing);
  $("v-seek").oninput = () => { setPlaying(false); seekVideo(parseFloat($("v-seek").value)); };
  $("v-speed").onchange = () => { V.speed = parseFloat($("v-speed").value); };
  $("v-source").onchange = () => {
    setPlaying(false); V.topic = $("v-source").value; V.shown = V.want = -1;
    buildStreamRates(V.topic); renderPanels();
    updateMp4Download(); showFrameAt(V.t);
  };
  document.querySelectorAll("#scene-focus button").forEach((b) => { b.onclick = () => setSceneFocus(b.dataset.focus); });
  updateSceneFocusControls();
  ["balloon-lat", "balloon-lon", "balloon-alt"].forEach((id) => { $(id).onchange = setBalloonFromInputs; });
  $("balloon-save").onclick = saveBalloon;
  document.addEventListener("keydown", (e) => {   // Space = 재생/일시정지 (입력창 밖에서만)
    if (e.code !== "Space" || !videoOn() || vmode !== "play") return;
    if (/^(INPUT|SELECT|TEXTAREA|BUTTON)$/.test(document.activeElement?.tagName)) return;
    e.preventDefault();
    setPlaying(!V.playing);
  });
  applyLayout();
  resetVideo();
  $("btn-refresh").onclick = () => loadRecords();
  $("rec-filter").oninput = renderRecords;
  const applyHelp = () => {
    document.body.classList.toggle("hide-help", !showHelp);
    $("btn-help").textContent = showHelp ? "수식 숨기기" : "수식 보기";
  };
  $("btn-help").onclick = () => { showHelp = !showHelp; saveLS("flight_analyzer.help", showHelp); applyHelp(); };
  applyHelp();
  applySidebar();
  $("btn-custom").onclick = addCustom;
  $("custom-input").onkeydown = (e) => { if (e.key === "Enter") addCustom(); };
  window.addEventListener("hashchange", () => {
    const id = decodeURIComponent(location.hash.slice(1));
    if (id && id !== currentId && records.some((r) => r.id === id)) openRecord(id);
  });
  let raf = 0;
  new ResizeObserver(() => {
    cancelAnimationFrame(raf);
    raf = requestAnimationFrame(() => {
      const w = chartWidth();
      for (const c of charts) if (c.width !== w) c.setSize({ width: w, height: CHART_H });
    });
  }).observe($("graphs"));
}

(async function init() {
  bind();
  loadRecords();
})();
