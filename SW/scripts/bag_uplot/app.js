const COLORS = ["#55b8ff", "#ff9f43", "#55d98b", "#ef6f9a", "#b48cff", "#f3d65c", "#65d7dc", "#f07c60"];
const preferred = /(^valid$|state_code|error_[xy]|los_rate_[xyz]|range_m|closing_speed|pn_accel_[xyz]|desired_(roll|pitch)|actual_(roll|pitch)|(roll|pitch)_cmd_deg|throttle|voltage|battery|detections_0_(x_center|y_center|width|height))/;

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
    axes: [{stroke: "#8290a0", grid: {stroke: "#222d39"}},
           {stroke: "#8290a0", grid: {stroke: "#222d39"}}],
    scales: {x: {time: false}}, cursor: {drag: {x: true, y: false, setScale: true}}}, data, plotEl);

  names.forEach((name, i) => {
    const label = document.createElement("label");
    label.innerHTML = `<input type="checkbox" ${active.has(name) ? "checked" : ""}>${name}`;
    label.querySelector("input").addEventListener("change", e => plot.setSeries(i + 1, {show: e.target.checked}));
    toggles.appendChild(label);
  });
  const resize = () => plot.setSize({width: Math.max(320, plotEl.clientWidth - 16), height: 280});
  new ResizeObserver(resize).observe(plotEl); resize();
  return card;
}

fetch("data.json").then(r => { if (!r.ok) throw new Error(r.statusText); return r.json(); }).then(payload => {
  const main = document.querySelector("#plots"); main.innerHTML = "";
  const samples = payload.topics.reduce((n, t) => n + t.samples, 0);
  document.querySelector("#summary").textContent = `${payload.topics.length} topics · ${samples.toLocaleString()} rows`;
  payload.topics.forEach(t => main.appendChild(buildCard(t)));
}).catch(err => {
  document.querySelector("#plots").innerHTML = `<div class="error">로드 실패: ${err.message}<br>HTTP 서버로 열었는지 확인하세요.</div>`;
});
