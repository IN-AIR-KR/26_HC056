const COLORS = ["#55b8ff", "#ff9f43", "#55d98b", "#ef6f9a", "#b48cff", "#f3d65c", "#65d7dc", "#f07c60"];
const preferred = /(^valid$|state_code|error_[xy]|los_rate_[xyz]|range_m|closing_speed|pn_accel_[xyz]|desired_(roll|pitch)|actual_(roll|pitch)|(roll|pitch)_cmd_deg|throttle|voltage|battery|detections_0_(x_center|y_center|width|height))/;
// 모든 plot이 같은 bag 기준 t_rel 축을 쓰므로 uPlot cursor sync group 하나로 묶는다.
// setSeries=false: 한 그래프에서 가까운 series를 고른 상태까지 다른 그래프에 강요하지 않고,
// 시간 위치/세로 crosshair와 각 그래프 legend 값만 함께 갱신한다.
const CURSOR_SYNC = {key: "arms-flight-time", setSeries: false};
const plotHandles = [];
let loadSequence = 0;

function buildCard(topic) {
  const names = Object.keys(topic.columns).filter(n => n !== "t_rel");
  const active = new Set(names.filter(n => preferred.test(n)).slice(0, 8));
  if (!active.size) names.slice(0, 4).forEach(n => active.add(n));
  const card = document.createElement("section"); card.className = "card";
  const head = document.createElement("div"); head.className = "card-head";
  head.innerHTML = `<h2>${topic.name}</h2><span class="meta">${topic.samples.toLocaleString()} samples</span>`;
  const toggles = document.createElement("div"); toggles.className = "toggles"; head.appendChild(toggles);
  const plotEl = document.createElement("div"); plotEl.className = "plot";
  card.append(head, plotEl);

  const data = [topic.columns.t_rel, ...names.map(n => topic.columns[n])];
  const series = [{label: "t [s]"}, ...names.map((n, i) => ({
    label: n, stroke: COLORS[i % COLORS.length], width: 1.4, show: active.has(n), spanGaps: true,
    points: {show: false}
  }))];
  const plot = new uPlot({width: 900, height: 280, series,
    axes: [{stroke: "#667381", grid: {stroke: "#151d26"}, ticks: {stroke: "#202a35"}},
           {stroke: "#667381", grid: {stroke: "#151d26"}, ticks: {stroke: "#202a35"}}],
    scales: {x: {time: false}},
    cursor: {
      sync: CURSOR_SYNC,
      drag: {x: true, y: false, setScale: true}
    }}, data, plotEl);

  names.forEach((name, i) => {
    const label = document.createElement("label");
    label.innerHTML = `<input type="checkbox" ${active.has(name) ? "checked" : ""}>${name}`;
    label.querySelector("input").addEventListener("change", e => plot.setSeries(i + 1, {show: e.target.checked}));
    toggles.appendChild(label);
  });
  const resize = () => plot.setSize({width: Math.max(320, plotEl.clientWidth - 16), height: 280});
  const observer = new ResizeObserver(resize); observer.observe(plotEl); resize();
  plotHandles.push({plot, observer});
  return card;
}

function clearPlots() {
  plotHandles.splice(0).forEach(({plot, observer}) => {
    observer.disconnect();
    plot.destroy();
  });
}

function renderPayload(payload, flight) {
  clearPlots();
  const main = document.querySelector("#plots"); main.innerHTML = "";
  const samples = payload.topics.reduce((n, t) => n + t.samples, 0);
  document.querySelector("#summary").textContent = `${payload.topics.length} topics · ${samples.toLocaleString()} rows`;
  document.querySelector("#current-flight").textContent = flight.name;
  payload.topics.forEach(t => main.appendChild(buildCard(t)));
}

function showError(err) {
  clearPlots();
  document.querySelector("#plots").innerHTML = `<div class="error">로드 실패: ${err.message}<br>HTTP 서버로 열었는지 확인하세요.</div>`;
}

async function selectFlight(flight, button) {
  const sequence = ++loadSequence;
  document.querySelectorAll(".flight-item").forEach(el => el.classList.toggle("active", el === button));
  if (flight.error) {
    clearPlots();
    document.querySelector("#current-flight").textContent = flight.name;
    document.querySelector("#summary").textContent = "unavailable";
    const main = document.querySelector("#plots"); main.innerHTML = "";
    const error = document.createElement("div"); error.className = "error";
    error.textContent = `이 기록을 열 수 없습니다: ${flight.error}`;
    main.appendChild(error);
    return;
  }
  document.querySelector("#plots").innerHTML = `<div class="loading">${flight.name} 읽는 중…</div>`;
  try {
    const response = await fetch(flight.data, {cache: "no-store"});
    if (!response.ok) {
      let detail = response.statusText;
      try { detail = (await response.json()).error || detail; } catch (_) {}
      throw new Error(detail);
    }
    const payload = await response.json();
    if (sequence === loadSequence) {
      flight.topics = payload.topics.length;
      button.querySelector(".flight-meta").textContent = `${flight.topics} topics`;
      renderPayload(payload, flight);
    }
  } catch (err) {
    if (sequence === loadSequence) showError(err);
  }
}

document.querySelector("#sidebar-toggle").addEventListener("click", () => {
  document.body.classList.toggle("sidebar-collapsed");
});

async function fetchCatalog() {
  const response = await fetch("/api/flights", {cache: "no-store"});
  if (!response.ok) throw new Error(response.statusText);
  return response.json();
}

async function loadFlightList() {
  const list = document.querySelector("#flight-list");
  list.innerHTML = `<div class="loading small">기록 찾는 중…</div>`;
  try {
    const manifest = await fetchCatalog();
    const list = document.querySelector("#flight-list");
    list.innerHTML = "";
    let firstAvailable = null;
    manifest.flights.forEach(flight => {
      const button = document.createElement("button");
      button.type = "button";
      button.className = `flight-item${flight.error ? " unavailable" : ""}`;
      const name = document.createElement("span"); name.className = "flight-name"; name.textContent = flight.name;
      const meta = document.createElement("span"); meta.className = "flight-meta";
      meta.textContent = flight.error ? "사용 불가 · 클릭해서 원인 보기"
        : flight.topics == null ? "클릭해서 불러오기" : `${flight.topics} topics`;
      button.append(name, meta);
      button.addEventListener("click", () => selectFlight(flight, button));
      list.appendChild(button);
      if (!flight.error && firstAvailable === null) firstAvailable = [flight, button];
    });
    if (!manifest.flights.length) list.innerHTML = `<div class="error small">기록이 없습니다.</div>`;
    else if (firstAvailable) selectFlight(...firstAvailable);
  } catch (err) {
    showError(err);
  }
}

document.querySelector("#flight-refresh").addEventListener("click", loadFlightList);
loadFlightList();
