/* IBVS 로그 뷰어 — uPlot 차트, 커서/줌 연동, 기록 목록 사이드바. */
"use strict";

const T = "/arms/ibvs_debug";
const R = "/arms/crsf_rx";

// ---------------------------------------------------------------- 패널 정의
// series: {t: 토픽(기본 T), f: 필드, label, scale: "y"|"y2", dash, width, step,
//          mul/add: 표시용 변환(범례에는 원값 표시)}
const PANELS = [
  { title: "① 오차", note: "정규화 픽셀 ±0.5 · raw = 상태머신 원시, filt = LPF(IBVS 입력)",
    series: [
      { f: "raw_err_x", dash: [4, 3], width: 1 }, { f: "filt_err_x" },
      { f: "raw_err_y", dash: [4, 3], width: 1 }, { f: "filt_err_y" },
    ] },
  { title: "② LOS각속도", note: "[1/s] · losf_dot = alpha-beta, los_rate = pn_los_clamp 후(IBVS 입력)",
    series: [
      { f: "losf_x_dot" }, { f: "los_rate_x", dash: [4, 3] },
      { f: "losf_y_dot" }, { f: "los_rate_y", dash: [4, 3] },
      { f: "err_dot_x", width: 1, show: false }, { f: "err_dot_y", width: 1, show: false },
    ] },
  { title: "③ 공선 오차 z1 / 이탈각", note: "z1 = 1−cos θ · kb 에 닿으면 배리어 발산 · 오른축 θ[deg]",
    series: [
      { f: "z1_raw", dash: [4, 3], width: 1 }, { f: "z1" }, { f: "kb", dash: [2, 4], width: 1 },
      { f: "theta_deg", scale: "y2" },
    ] },
  { title: "③ 배리어", note: "z1 / (kb² − z1²)",
    series: [{ f: "barrier" }] },
  { title: "④ Roll 항별 기여", note: "[deg/s] · 부호·스케줄링 전 항 → guidance → cmd(스케줄 후) → out(clamp 후 송신값)",
    series: [
      { f: "barrier_term_x", width: 1 }, { f: "center_term_x", width: 1 }, { f: "los_rate_term_x", width: 1 },
      { f: "guidance_x", width: 2 }, { f: "roll_rate_cmd", dash: [4, 3] }, { f: "roll_rate_out", width: 2 },
    ] },
  { title: "④ Pitch 항별 기여", note: "[deg/s]",
    series: [
      { f: "barrier_term_y", width: 1 }, { f: "center_term_y", width: 1 }, { f: "los_rate_term_y", width: 1 },
      { f: "guidance_y", width: 2 }, { f: "pitch_rate_cmd", dash: [4, 3] }, { f: "pitch_rate_out", width: 2 },
    ] },
  { title: "⑥ 명령 vs 기체 자세", note: "왼축 = 송신 각속도[deg/s] · 오른축 = CRSF 텔레메트리 자세[deg]",
    series: [
      { f: "roll_rate_out" }, { f: "pitch_rate_out" },
      { t: R, f: "roll_deg", scale: "y2", dash: [4, 3] }, { t: R, f: "pitch_deg", scale: "y2", dash: [4, 3] },
    ] },
  { title: "⑤ 스케줄링 배율", note: "0~1",
    series: [
      { f: "closeness" }, { f: "range_gain_scale" }, { f: "range_throttle_scale" },
      { f: "tau_ease" }, { f: "center_q" },
    ] },
  { title: "⑤ Looming / τ", note: "왼축 = bbox 크기·팽창률 · 오른축 = τ[s] (9.99 = 미접근)",
    series: [
      { f: "loom_size" }, { f: "loom_rate" },
      { f: "tau", scale: "y2" },
    ] },
  { title: "⑦ 추력", note: "0~1 · emag = |filt_err|",
    series: [{ f: "thrust", width: 2 }, { f: "throttle_up", dash: [4, 3] }, { f: "emag", width: 1 }] },
  { title: "플래그", note: "각 줄 = 0/1 (위로 올라가면 1)", flags: true,
    series: ["cmd_active", "filter_active", "detection_held", "z1_clamped", "rate_saturated", "manual_out"]
      .map((f, i) => ({ f, step: true, mul: 0.8, add: i })) },
  { title: "게인 / 틱 간격", note: "왼축 = 게인 · 오른축 = dt[ms]",
    series: [
      { f: "ibvs_gain" }, { f: "ibvs_center_gain" }, { f: "ibvs_los_rate_gain" },
      { f: "dt", label: "dt[ms]", scale: "y2", mul: 1000, width: 1 },
    ] },
];

const PALETTE = ["#2563eb", "#dc2626", "#16a34a", "#d97706", "#7c3aed", "#0891b2", "#db2777", "#65a30d"];
const STATE_COLORS = {
  SEARCH: "rgba(148,163,184,0.14)",
  LOCK: "rgba(234,179,8,0.16)",
  TRACK: "rgba(34,197,94,0.14)",
  FIRE: "rgba(239,68,68,0.20)",
  RTL: "rgba(59,130,246,0.14)",
};
const CHART_H = 210;

// ---------------------------------------------------------------- 상태
let rec = null;            // 현재 기록 데이터
let charts = [];           // uPlot 인스턴스
let stateSegs = [];        // [{t0, t1, state}]
let syncing = false;       // 줌 전파 재진입 방지
let records = [];
let currentId = null;
let customPanels = loadLS("ibvs_viewer.custom", []);

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

// ---------------------------------------------------------------- 폴더 선택
async function openPicker(start) {
  $("picker").hidden = false;
  $("pk-cancel").hidden = !$("root-path").textContent;
  await browse(start);
}
async function browse(path) {
  let j;
  try {
    j = await api("/api/browse?path=" + encodeURIComponent(path || ""));
  } catch (e) { $("pk-info").textContent = e.message; return; }
  $("pk-path").value = j.path;
  $("pk-up").disabled = !j.parent;
  $("pk-up").onclick = () => browse(j.parent);
  const nRec = j.dirs.filter((d) => d.is_record).length;
  $("pk-info").textContent = nRec
    ? `이 폴더에 기록 ${nRec}개 — "이 폴더 사용" 을 누르세요.`
    : "하위 폴더를 눌러 이동하세요.";
  const ul = $("pk-list");
  ul.innerHTML = "";
  if (!j.dirs.length) ul.innerHTML = '<li class="muted">(하위 폴더 없음)</li>';
  for (const d of j.dirs) {
    const li = document.createElement("li");
    li.innerHTML = `<span>📁</span><span class="mono"></span>${d.is_record ? '<span class="badge">기록</span>' : ""}`;
    li.children[1].textContent = d.name;
    li.onclick = () => browse(d.path);
    ul.appendChild(li);
  }
}
async function useFolder() {
  try {
    await api("/api/root", {
      method: "POST", headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ path: $("pk-path").value }),
    });
  } catch (e) { $("pk-info").textContent = e.message; return; }
  $("picker").hidden = true;
  await loadRecords();
}

// ---------------------------------------------------------------- 기록 목록
async function loadRecords() {
  const j = await api("/api/records");
  $("root-path").textContent = j.root;
  $("root-path").title = j.root;
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
    li.className = (r.id === currentId ? "active " : "") + (r.ibvs_count ? "" : "no-ibvs");
    li.innerHTML = `<div class="rec-name"></div><div class="rec-sub"></div>`;
    li.children[0].textContent = r.name;
    const sub = li.children[1];
    const parts = [
      r.start ? r.start.replace("T", " ").slice(5, 16) : null,
      fmtDur(r.duration), fmtSize(r.size),
    ].filter(Boolean);
    for (const p of parts) { const s = document.createElement("span"); s.textContent = p; sub.appendChild(s); }
    const s = document.createElement("span");
    if (r.ibvs_count) s.textContent = `ibvs ${r.ibvs_count}`;
    else { s.textContent = "ibvs_debug 없음"; s.className = "warn"; }
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
    `${fmtDur(rec.duration)} · 토픽 ${topics.length}개 · ibvs_debug ${rec.topics[T]?.t.length ?? 0}개`;
  $("errors").innerHTML = "";
  for (const e of rec.errors || []) {
    const d = document.createElement("div");
    d.textContent = "역직렬화 실패(건너뜀): " + e;
    $("errors").appendChild(d);
  }
  if (!rec.topics[T]) {
    const d = document.createElement("div");
    d.textContent = "이 기록에는 /arms/ibvs_debug 가 없습니다 (IbvsDebug 추가 전 녹화). 다른 토픽은 아래 '패널 추가' 로 볼 수 있습니다.";
    $("errors").appendChild(d);
  }
  stateSegs = buildStateSegs();
  renderStateLegend();
  fillFieldList();
  renderPanels();
}

function buildStateSegs() {
  const src = rec.topics[T]?.strings?.state ? rec.topics[T] : rec.topics["/arms/mission_state"];
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
    for (const f of Object.keys(d.fields)) {
      const o = document.createElement("option");
      o.value = `${tp} ${f}`;
      dl.appendChild(o);
    }
  }
  $("custom-add").hidden = false;
}

// ---------------------------------------------------------------- 차트
function destroyCharts() {
  charts.forEach((c) => c.destroy());
  charts = [];
}

function renderPanels() {
  destroyCharts();
  $("empty").hidden = true;
  const root = $("panels");
  root.innerHTML = "";
  const all = [...PANELS, ...customPanels.map((p, i) => ({ ...p, custom: i }))];
  for (const p of all) root.appendChild(makePanel(p));
}

function makePanel(p) {
  const det = document.createElement("details");
  det.className = "panel";
  const closedKey = "ibvs_viewer.closed." + p.title;
  det.open = !loadLS(closedKey, false);
  const sum = document.createElement("summary");
  sum.innerHTML = `<span></span><span class="note"></span>`;
  sum.children[0].textContent = p.title;
  sum.children[1].textContent = p.note || "";
  if (p.custom != null) {
    const x = document.createElement("button");
    x.className = "btn small close";
    x.textContent = "삭제";
    x.onclick = (e) => {
      e.preventDefault();
      customPanels.splice(p.custom, 1);
      saveLS("ibvs_viewer.custom", customPanels);
      renderPanels();
    };
    sum.appendChild(x);
  }
  det.appendChild(sum);
  const body = document.createElement("div");
  det.appendChild(body);

  const built = buildData(p);
  if (!built) {
    body.innerHTML = '<div class="missing">이 기록에 해당 필드가 없음</div>';
    return det;
  }
  const chart = makeChart(p, built, body);
  det.addEventListener("toggle", () => {
    saveLS(closedKey, !det.open);
    if (det.open) chart.setSize({ width: chartWidth(), height: CHART_H });
  });
  return det;
}

/* 패널의 시리즈들을 하나의 x축으로 정렬. 토픽이 여러 개면 uPlot.join 으로 합친다. */
function buildData(p) {
  const groups = new Map();   // topic → [spec]
  for (const s of p.series) {
    const tp = s.t || T;
    const col = rec.topics[tp]?.fields?.[s.f];
    if (!col) continue;
    if (!groups.has(tp)) groups.set(tp, []);
    groups.get(tp).push(s);
  }
  if (!groups.size) return null;
  const tables = [], specs = [];
  for (const [tp, ss] of groups) {
    const d = rec.topics[tp];
    const cols = ss.map((s) => {
      const src = d.fields[s.f];
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
  return Math.max(300, $("panels").clientWidth - 22);
}

function makeChart(p, { data, specs, multiTopic }, el) {
  const axisStroke = cssVar("--axis"), grid = cssVar("--grid");
  const hasY2 = specs.some((s) => s.scale === "y2");
  const series = [{ label: "t[s]", value: (u, v) => (v == null ? "—" : v.toFixed(3)) }];
  specs.forEach((s, i) => {
    const mul = s.mul ?? 1, add = s.add ?? 0;
    const name = s.label || (multiTopic && s.t !== T ? `${s.t.split("/").pop()}.${s.f}` : s.f);
    series.push({
      label: name,
      scale: s.scale || "y",
      stroke: s.color || PALETTE[i % PALETTE.length],
      width: s.width ?? 1.5,
      dash: s.dash,
      show: s.show ?? true,
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
          values: () => specs.map((s) => s.f) }
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
        const st = stateAt(t);
        $("readout").textContent = `t = ${t.toFixed(3)} s${st ? "  ·  " + st : ""}`;
      }],
    },
  };
  const u = new uPlot(opts, data, el);
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
    if (!rec.topics[tp]?.fields?.[f]) {
      alert(`필드 없음: ${tp} ${f}`);
      return;
    }
    series.push({ t: tp, f });
  }
  if (!series.length) return;
  customPanels.push({ title: "사용자 패널: " + series.map((s) => s.f).join(", "), series });
  saveLS("ibvs_viewer.custom", customPanels);
  $("custom-input").value = "";
  renderPanels();
  $("panels").lastElementChild?.scrollIntoView({ behavior: "smooth", block: "center" });
}

// ---------------------------------------------------------------- 시작
function bind() {
  $("btn-change-root").onclick = () => openPicker($("root-path").textContent);
  $("btn-refresh").onclick = () => loadRecords();
  $("rec-filter").oninput = renderRecords;
  $("btn-reset").onclick = resetZoom;
  $("btn-custom").onclick = addCustom;
  $("custom-input").onkeydown = (e) => { if (e.key === "Enter") addCustom(); };
  $("pk-go").onclick = () => browse($("pk-path").value);
  $("pk-path").onkeydown = (e) => { if (e.key === "Enter") browse($("pk-path").value); };
  $("pk-use").onclick = useFolder;
  $("pk-cancel").onclick = () => { $("picker").hidden = true; };
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
  }).observe($("panels"));
}

(async function init() {
  bind();
  const j = await api("/api/root");
  if (!j.root) openPicker(j.default);   // 시작 시 데이터 폴더 선택
  else loadRecords();
})();
