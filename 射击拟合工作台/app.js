(() => {
  "use strict";

  const fit = globalThis.ShotFit;
  const byId = id => document.getElementById(id);
  const clamp = (value, low, high) => Math.min(high, Math.max(low, value));
  const format = (value, digits = 1) => Number.isFinite(value) ? value.toFixed(digits) : "—";
  const signed = (value, digits = 1) => Number.isFinite(value) ? (value > 0 ? "+" : "") + value.toFixed(digits) : "—";
  const pointKey = point => point.id + "@" + point.x + "," + point.y;
  const storageKey = "biobuzz-shot-fit-workbench-v1";
  const initialGeometry = globalThis.initialShotGeometry;
  const initial = fit.parse(globalThis.initialShotRecords, initialGeometry);
  const field = byId("field");
  const anglePlot = byId("angle-profile");
  const rpmPlot = byId("rpm-profile");
  const query = { x: 0.5, y: 2.5 };
  let points = initial.points;
  let baselinePoints = points.map(point => ({ ...point }));
  let settings = { ...fit.defaults };
  let geometry = initialGeometry;
  let selected = null;
  let model, stats, rpmLow, rpmHigh, fitFrame, fieldBounds;
  let blockedTiles = new Set();
  let recalAnchors = [];
  let feedbackTimer;

  function checkedSettings(source = {}) {
    const value = (key, low, high) => Number.isFinite(source[key]) ? clamp(source[key], low, high) : fit.defaults[key];
    return {
      angleOrder: Math.round(value("angleOrder", -1, 2)),
      rpmOrder: Math.round(value("rpmOrder", 0, 3)),
      ridge: value("ridge", 0.001, 10),
      angleTrim: value("angleTrim", -45, 45),
      rpmTrim: value("rpmTrim", -1000, 1000),
      robust: typeof source.robust === "boolean" ? source.robust : fit.defaults.robust,
      directionalRpm: typeof source.directionalRpm === "boolean" ? source.directionalRpm : fit.defaults.directionalRpm
    };
  }

  function normalizedPoint(point) {
    return {
      ...point,
      blocked: Boolean(point.blocked),
      inactive: Boolean(point.inactive),
      weight: Number.isFinite(point.weight) ? point.weight : 1
    };
  }

  function applyProject(project) {
    if (![1, 2].includes(project?.version) || !Array.isArray(project.points) ||
        project.points.length > 1200 || !project.points.every(fit.validPoint)) {
      throw new Error("方案中缺少有效的 06 测点。");
    }
    points = project.points.map(normalizedPoint);
    if (Array.isArray(project.baselinePoints) && project.baselinePoints.length <= 1200 &&
        project.baselinePoints.every(fit.validPoint)) {
      baselinePoints = project.baselinePoints.map(normalizedPoint);
    } else if (points.some(point => fit.usable(point) && /[1-4a-h]$/.test(point.id))) {
      baselinePoints = points.map(point => ({ ...point }));
    }
    settings = checkedSettings(project.settings || {});
    geometry = fit.checkedGeometry(project.geometry || initialGeometry);
    selected = null;
  }

  try {
    const previous = localStorage.getItem(storageKey);
    if (previous) applyProject(JSON.parse(previous));
  } catch {
    // An unavailable browser store does not prevent portable JSON files from working.
  }

  function message(text) {
    const feedback = byId("feedback");
    feedback.textContent = text;
    feedback.hidden = false;
    clearTimeout(feedbackTimer);
    feedbackTimer = setTimeout(() => { feedback.hidden = true; }, 3800);
  }

  function syncControls() {
    for (const key of ["angleOrder", "rpmOrder", "angleTrim", "rpmTrim"]) byId(key).value = settings[key];
    for (const key of ["robust", "directionalRpm"]) byId(key).checked = Boolean(settings[key]);
    byId("ridge").value = Math.log10(settings.ridge);
    byId("ridge-value").textContent = "λ " + settings.ridge.toPrecision(3);
  }

  function polynomial(poly) {
    if (!poly.terms.length) return "0";
    return poly.terms.map(([p, q], index) => {
      const coefficient = poly.coefficients[index];
      if (Math.abs(coefficient) < 0.00005 && index > 0) return "";
      const term = (p ? p === 1 ? "u" : "u^" + p : "") + (q ? q === 1 ? "v" : "v^" + q : "");
      return (coefficient < 0 ? " − " : index ? " + " : "") + format(Math.abs(coefficient), 4) + term;
    }).join("").trimStart();
  }

  function equationText() {
    if (!model) return "至少需要一个参与拟合的实测点。";
    const [right, forward] = model.goal;
    const trim = value => value === 0 ? "" : value > 0 ? " + " + value : " − " + Math.abs(value);
    const offset = Number.isFinite(model.goalOffsetDeg) ? model.goalOffsetDeg : 0;
    const offsetTerm = Math.abs(offset) >= 0.0005
      ? (offset > 0 ? " + " : " − ") + format(Math.abs(offset), 3)
      : "";
    const polyExpr = model.angular.terms.length ? " + P" : "";
    const [x0, sx] = model.angular.x, [y0, sy] = model.angular.y;
    const [r0, sr] = model.speed.radius;
    const radial = model.speed.coefficients.map((value, i) =>
      (value < 0 ? " − " : i > 0 ? " + " : "") + format(Math.abs(value), 3) + (i === 0 ? "" : i === 1 ? "d" : "d^" + i)).join("");
    const [ac, as] = model.speed.directional || [0, 0];
    const hasDir = Math.abs(ac) >= 0.05 || Math.abs(as) >= 0.05;
    const dirExpr = hasDir
      ? (ac >= 0 ? " + " : " − ") + format(Math.abs(ac), 1) + "cos β" +
        (as >= 0 ? " + " : " − ") + format(Math.abs(as), 1) + "sin β"
      : "";
    return "θ = s × (atan2(x − " + format(right, 5) + ", " + format(forward, 5) + " − y) × 180/π" +
      offsetTerm + polyExpr + ")" + trim(settings.angleTrim) +
      (model.angular.terms.length
        ? "\nP = " + polynomial(model.angular) +
          "\nu = (x − " + format(x0, 4) + ")/" + format(sx, 4) + ", v = (y − " + format(y0, 4) + ")/" + format(sy, 4)
        : "") +
      "\nRPM = " + radial + dirExpr + trim(settings.rpmTrim) +
      "\nd = (r − " + format(r0, 4) + ")/" + format(sr, 4) +
      (hasDir ? "，β = atan2(x − " + format(right, 3) + ", " + format(forward, 3) + " − y)" : "");
  }

  function updateFit(persist = false) {
    model = fit.fit(points, settings, geometry);
    stats = fit.diagnostics(points, model, settings);
    const measuredRpm = points.filter(fit.usable).map(point => point.rpm);
    rpmLow = measuredRpm.length ? Math.min(...measuredRpm) : 2900;
    rpmHigh = measuredRpm.length ? Math.max(...measuredRpm) : 3500;
    blockedTiles = new Set();
    const realCenters = new Set(points.filter(point => point.id[2] === "0")
      .map(point => (Number(point.id[1]) - 1) + "," + (Number(point.id[0]) - 1)));
    for (const point of points.filter(point => point.id[2] === "0" && point.blocked)) {
      const x = Number(point.id[1]) - 1, y = Number(point.id[0]) - 1;
      blockedTiles.add(x + "," + y);
      if (byId("mirror").checked && !realCenters.has((5 - x) + "," + y)) blockedTiles.add((5 - x) + "," + y);
    }
    byId("point-count").textContent = (model?.count ?? 0) + " 点";
    byId("record-count").textContent = points.length + " 条记录 · " + points.filter(point => point.blocked).length + " 不可达";
    byId("angle-error").textContent = stats ? format(stats.inside.angle, 2) + "°" : "—";
    byId("rpm-error").textContent = stats ? format(stats.inside.rpm) + " RPM" : "—";
    byId("angle-validation").textContent = "留一验证 " + (stats?.validation
      ? format(stats.validation.angle, 2) + "° · 中位 " + format(stats.validationMedian?.angle, 2) + "°"
      : "—");
    byId("rpm-validation").textContent = "留一验证 " + (stats?.validation
      ? format(stats.validation.rpm, 0) + " RPM · 中位 " + format(stats.validationMedian?.rpm, 0)
      : "—");
    byId("equations").textContent = equationText();
    byId("rpm-low").textContent = format(rpmLow, 0);
    byId("rpm-high").textContent = format(rpmHigh, 0);
    byId("ridge-value").textContent = "λ " + settings.ridge.toPrecision(3);
    renderTable();
    renderSelection();
    draw();
    if (persist) {
      try {
        localStorage.setItem(storageKey, JSON.stringify({ version: 2, points, baselinePoints, settings, geometry }));
      } catch { /* file export is still available */ }
    }
  }

  function scheduleFit() {
    if (fitFrame) cancelAnimationFrame(fitFrame);
    fitFrame = requestAnimationFrame(() => { fitFrame = null; updateFit(true); });
  }

  function canvasState(canvas) {
    const width = canvas.clientWidth, height = canvas.clientHeight;
    const density = Math.min(window.devicePixelRatio || 1, 3);
    if (canvas.width !== Math.round(width * density) || canvas.height !== Math.round(height * density)) {
      canvas.width = Math.round(width * density);
      canvas.height = Math.round(height * density);
    }
    const ctx = canvas.getContext("2d");
    ctx.setTransform(density, 0, 0, density, 0, 0);
    ctx.clearRect(0, 0, width, height);
    ctx.font = '12px Bahnschrift, "Microsoft YaHei UI", sans-serif';
    return { ctx, width, height };
  }

  function segment(ctx, x1, y1, x2, y2, color, width = 1) {
    ctx.strokeStyle = color;
    ctx.lineWidth = width;
    ctx.beginPath();
    ctx.moveTo(x1, y1);
    ctx.lineTo(x2, y2);
    ctx.stroke();
  }

  function circle(ctx, x, y, radius, color, ring) {
    ctx.beginPath();
    ctx.arc(x, y, radius, 0, 2 * Math.PI);
    if (color) { ctx.fillStyle = color; ctx.fill(); }
    if (ring) { ctx.strokeStyle = ring; ctx.lineWidth = 1.4; ctx.stroke(); }
  }

  function speedColor(rpm) {
    const t = rpmHigh === rpmLow ? 0.5 : clamp((rpm - rpmLow) / (rpmHigh - rpmLow), 0, 1);
    const palette = [[99, 156, 243], [99, 221, 205], [243, 192, 118]];
    const segment = t < 0.5 ? 0 : 1;
    const blend = t * 2 - segment;
    return "rgb(" + palette[segment].map((value, index) =>
      Math.round(value + blend * (palette[segment + 1][index] - value))).join(",") + ")";
  }

  function arrow(ctx, x, y, angle, length, color) {
    const radians = angle * Math.PI / 180;
    const dx = -Math.sin(radians), dy = -Math.cos(radians);
    const endX = x + length * dx, endY = y + length * dy;
    segment(ctx, x - dx * length * 0.27, y - dy * length * 0.27, endX, endY, color, 1.6);
    ctx.fillStyle = color;
    ctx.beginPath();
    ctx.moveTo(endX, endY);
    ctx.lineTo(endX - dx * 5 + dy * 2.5, endY - dy * 5 - dx * 2.5);
    ctx.lineTo(endX - dx * 5 - dy * 2.5, endY - dy * 5 + dx * 2.5);
    ctx.closePath();
    ctx.fill();
  }

  function available(x, y) {
    return !blockedTiles.has(Math.min(5, Math.floor(x)) + "," + Math.min(5, Math.floor(y)));
  }

  function mapEnabled(x) {
    return byId("mirror").checked || x < 3;
  }

  function drawField() {
    const { ctx, width, height } = canvasState(field);
    const size = Math.min(width - 60, height - 47);
    if (size <= 0) return;
    const left = (width - size) / 2 + 5, top = 17, unit = size / 6;
    const sx = x => left + unit * x, sy = y => top + size - unit * y;
    fieldBounds = { sx, sy, left, top, size, unit };
    ctx.fillStyle = "#192630";
    ctx.fillRect(left, top, size, size);
    ctx.save();
    ctx.beginPath(); ctx.rect(left, top, size, size); ctx.clip();
    for (const tile of blockedTiles) {
      const [x, y] = tile.split(",").map(Number), px = sx(x), py = sy(y + 1);
      ctx.fillStyle = "#372932";
      ctx.fillRect(px, py, unit, unit);
      ctx.save(); ctx.beginPath(); ctx.rect(px, py, unit, unit); ctx.clip();
      for (let i = -unit; i < unit * 2; i += 10) segment(ctx, px + i, py + unit, px + i + unit, py, "#60444c");
      ctx.restore();
    }
    if (model?.hull.length >= 3) {
      for (const mirror of byId("mirror").checked ? [false, true] : [false]) {
        ctx.beginPath();
        model.hull.forEach((point, index) => {
          const x = sx(mirror ? 6 - point.x : point.x), y = sy(point.y);
          if (index === 0) ctx.moveTo(x, y);
          else ctx.lineTo(x, y);
        });
        ctx.closePath();
        ctx.fillStyle = "#57dabc20"; ctx.fill();
        ctx.setLineDash([4, 3]); ctx.strokeStyle = "#67c5b1"; ctx.stroke(); ctx.setLineDash([]);
      }
    }
    for (let i = 0; i <= 6; i++) {
      segment(ctx, sx(i), top, sx(i), sy(0), "#53636c");
      segment(ctx, left, sy(i), sx(6), sy(i), "#53636c");
    }
    ctx.setLineDash([5, 4]);
    segment(ctx, sx(3), top, sx(3), sy(0), "#9daebc");
    ctx.setLineDash([]);
    if (model) {
      const includeExtrapolation = byId("extrapolate").checked;
      for (let col = 0; col < 6; col++) {
        for (const offset of [0.13, 0.43, 0.57, 0.87]) {
          const x = col + offset;
          if (!mapEnabled(x)) continue;
          for (let y = 0.19; y < 6; y += 0.315) {
            const supported = fit.supported(model, x, y);
            if ((!supported && !includeExtrapolation) || (!available(x, y) && !includeExtrapolation)) continue;
            const prediction = fit.predict(model, x, y, settings);
            ctx.globalAlpha = supported ? 0.92 : 0.30;
            arrow(ctx, sx(x), sy(y), prediction.angle, Math.min(size * 0.025, 12), speedColor(prediction.rpm));
          }
        }
      }
      ctx.globalAlpha = 1;
    }
    for (const point of points) {
      if (point.angle === null) continue;
      const isSelected = point === selected;
      circle(ctx, sx(point.x), sy(point.y), isSelected ? 5.8 : 3.1,
        fit.usable(point) ? "#70e6cd" : "#a88d98", isSelected ? "#fff1c8" : "#13252c");
    }
    ctx.setLineDash([3, 5]);
    segment(ctx, left, sy(query.y), sx(6), sy(query.y), "#92a9c26c");
    segment(ctx, sx(query.x), top, sx(query.x), sy(0), "#92a9c26c");
    ctx.setLineDash([]);
    const probe = fit.predict(model, query.x, query.y, settings);
    if (probe && mapEnabled(query.x)) arrow(ctx, sx(query.x), sy(query.y), probe.angle, 21, "#ffd093");
    circle(ctx, sx(query.x), sy(query.y), 5.5, null, "#ffce8c");
    ctx.restore();
    ctx.strokeStyle = "#687c89"; ctx.lineWidth = 1; ctx.strokeRect(left, top, size, size);
    ctx.fillStyle = "#aebfca"; ctx.textAlign = "center";
    for (let i = 0; i <= 6; i++) {
      ctx.fillText(i, sx(i), sy(0) + 17);
      ctx.fillText(i, left - 14, sy(i) + 4);
    }
    ctx.fillText("右 · 格", left + size / 2, sy(0) + 29);
    ctx.fillText("前", left - 25, top + 28);
  }

  function drawProfile(canvas, key) {
    const { ctx, width, height } = canvasState(canvas);
    const left = 54, top = 13, bottom = height - 31, plotWidth = width - left - 18;
    const px = y => left + plotWidth * y / 6;
    const mirrored = query.x >= 3;
    const withinSide = point => (point.x >= 3) === mirrored;
    const nearby = points.filter(point => fit.usable(point) && withinSide(point) && Math.abs(point.x - query.x) <= 0.18);
    const series = model && mapEnabled(query.x) ? Array.from({ length: 121 }, (_, index) => {
      const y = index * 6 / 120;
      return { y, value: fit.predict(model, query.x, y, settings)[key] };
    }) : [];
    const ticks = series.map(point => point.value).concat(nearby.map(point => point[key]));
    const lo = Math.min(...ticks), hi = Math.max(...ticks);
    let step = key === "rpm" ? 50 : 15;
    while (Number.isFinite(hi - lo) && (hi - lo) / step > 6) step *= 2;
    const min = ticks.length ? Math.floor(lo / step) * step : 0;
    const max = ticks.length ? Math.max(min + step, Math.ceil(hi / step) * step) : step;
    const py = value => bottom - (value - min) / (max - min) * (bottom - top);
    ctx.fillStyle = "#9cb0be";
    ctx.textAlign = "right";
    for (let value = min; value <= max; value += step) {
      segment(ctx, left, py(value), width - 18, py(value), "#304351");
      ctx.fillText(format(value, 0), left - 7, py(value) + 4);
    }
    ctx.textAlign = "center";
    for (let y = 0; y <= 6; y++) {
      segment(ctx, px(y), top, px(y), bottom, "#273946");
      ctx.fillText(y, px(y), bottom + 15);
    }
    ctx.fillText("前 · 格", left + plotWidth / 2, height - 3);
    for (let i = 1; i < series.length; i++) {
      const previous = series[i - 1], next = series[i];
      if (key === "angle" && Math.abs(previous.value - next.value) > 180) continue;
      ctx.setLineDash(fit.supported(model, query.x, next.y) ? [] : [3, 3]);
      segment(ctx, px(previous.y), py(previous.value), px(next.y), py(next.value), "#80b8ff", 2);
    }
    ctx.setLineDash([]);
    for (const point of nearby) circle(ctx, px(point.y), py(point[key]), 4, "#64dfc3", "#101a21");
    segment(ctx, px(query.y), top, px(query.y), bottom, "#ffd1a385");
    if (series.length) circle(ctx, px(query.y), py(fit.predict(model, query.x, query.y, settings)[key]), 4, "#ffd093", "#101a21");
    canvas.onclick = event => {
      const x = event.clientX - canvas.getBoundingClientRect().left;
      if (x >= left && x <= width - 18) setQuery(query.x, (x - left) * 6 / plotWidth);
    };
  }

  function draw() {
    drawField();
    drawProfile(anglePlot, "angle");
    drawProfile(rpmPlot, "rpm");
    const probe = mapEnabled(query.x) ? fit.predict(model, query.x, query.y, settings) : null;
    const support = fit.supported(model, query.x, query.y);
    const output = byId("probe");
    output.dataset.outside = !support || !available(query.x, query.y);
    output.textContent = probe ? format(probe.angle, 2) + "°  ·  " + format(probe.rpm, 0) + " RPM  ·  " +
      (!available(query.x, query.y) ? "不可达格" : support ? "测点包络内" : "外推") : "此位置暂无预测";
    byId("profile-location").textContent = "右 " + format(query.x, 3) + " 格 · 虚线为包络外 · 散点取邻近实测";
  }

  function setQuery(x, y) {
    if (!Number.isFinite(x) || !Number.isFinite(y)) return;
    query.x = clamp(x, 0, 6); query.y = clamp(y, 0, 6);
    byId("query-x").value = format(query.x, 3); byId("query-y").value = format(query.y, 3);
    draw();
  }

  function selectPoint(point) {
    selected = point;
    if (point) setQuery(point.x, point.y);
    else draw();
    byId("edit-angle").value = point?.angle ?? "";
    byId("edit-rpm").value = point?.rpm ?? "";
    byId("edit-weight").value = point?.weight ?? 1;
    renderSelection();
    renderTable();
  }

  function renderSelection() {
    byId("clear-selection").hidden = !selected;
    byId("edit-form").hidden = !selected || selected.angle === null || selected.blocked || selected.inactive;
    byId("selection-title").textContent = selected ? selected.id + " · " + fit.pointName(selected.id) : "测点微调";
    const residual = selected && fit.error(selected, model, settings);
    byId("selection-hint").textContent = residual ? "实测减拟合 " + signed(residual.angle, 2) + "° / " +
      signed(residual.rpm, 0) + " RPM" + (selected.blocked || selected.inactive ? " · 不参与拟合" : " · 权重 0 可排除")
      : selected ? "此点没有保存射向和转速。" : "点地图或下方列表调整测点，列表按射向偏差排序。";
    const shortcuts = byId("residuals");
    shortcuts.replaceChildren();
    if (selected || !model) return;
    const largest = points.filter(fit.usable).map(point => ({ point, residual: fit.error(point, model, settings) }))
      .sort((a, b) => Math.abs(b.residual.angle) - Math.abs(a.residual.angle)).slice(0, 3);
    for (const { point, residual: delta } of largest) {
      const button = document.createElement("button");
      const name = document.createElement("span"), values = document.createElement("span");
      button.type = "button";
      name.textContent = point.id + " " + fit.pointName(point.id);
      values.textContent = signed(delta.angle, 2) + "° / " + signed(delta.rpm, 0) + " RPM";
      button.append(name, values);
      button.addEventListener("click", () => selectPoint(point));
      shortcuts.append(button);
    }
  }

  function renderTable() {
    const fragment = document.createDocumentFragment();
    for (const point of points) {
      if (point.blocked && !byId("show-blocked").checked) continue;
      const residual = fit.error(point, model, settings);
      const tr = document.createElement("tr");
      tr.dataset.selected = point === selected;
      tr.dataset.blocked = point.blocked;
      const cell = document.createElement("td"), button = document.createElement("button");
      button.type = "button";
      button.textContent = point.id + " " + fit.pointName(point.id);
      button.addEventListener("click", () => {
        selectPoint(point);
        byId("selection").scrollIntoView({ block: "nearest" });
      });
      cell.append(button); tr.append(cell);
      const values = [
        format(point.x, 3) + " / " + format(point.y, 3), format(point.angle, 2), format(point.rpm, 0),
        signed(residual?.angle, 2), signed(residual?.rpm, 0),
        point.blocked ? "不可达" : point.inactive ? "已停用" : point.angle === null ? "未测" : format(point.weight, 2)
      ];
      for (const [index, value] of values.entries()) {
        const td = document.createElement("td");
        td.textContent = value;
        if ((index === 3 && Math.abs(residual?.angle) >= 5) || (index === 4 && Math.abs(residual?.rpm) >= 100)) {
          td.className = "large-residual";
        }
        tr.append(td);
      }
      fragment.append(tr);
    }
    byId("records").replaceChildren(fragment);
  }

  function download(name, text, type) {
    const url = URL.createObjectURL(new Blob([text], { type }));
    const link = document.createElement("a");
    link.href = url; link.download = name;
    document.body.append(link); link.click(); link.remove();
    setTimeout(() => URL.revokeObjectURL(url), 500);
  }

  function saveCsv() {
    const rows = ["id,right_tile,forward_tile,angle_deg,rpm,weight,blocked,inactive,fit_angle_deg,fit_rpm,error_deg,error_rpm"];
    for (const point of points) {
      const guess = fit.predict(model, point.x, point.y, settings), residual = fit.error(point, model, settings);
      const numeric = value => Number.isFinite(value) ? value.toFixed(6) : "";
      rows.push([point.id, numeric(point.x), numeric(point.y), point.angle ?? "", point.rpm ?? "",
        point.weight, Number(point.blocked), Number(point.inactive), numeric(guess?.angle), numeric(guess?.rpm),
        numeric(residual?.angle), numeric(residual?.rpm)].join(","));
    }
    download("06-measurements.csv", rows.join("\r\n") + "\r\n", "text/csv;charset=utf-8");
  }

  for (const key of ["angleOrder", "rpmOrder", "angleTrim", "rpmTrim"]) {
    byId(key).addEventListener("input", event => {
      if (event.target.value === "") return;
      settings = checkedSettings({ ...settings, [key]: Number(event.target.value) });
      scheduleFit();
    });
  }
  byId("ridge").addEventListener("input", event => {
    settings.ridge = 10 ** Number(event.target.value);
    scheduleFit();
  });
  for (const key of ["robust", "directionalRpm"]) {
    byId(key).addEventListener("change", event => {
      settings = checkedSettings({ ...settings, [key]: event.target.checked });
      updateFit(true);
    });
  }
  byId("auto-tune").addEventListener("click", () => {
    const tuned = fit.autoTune(points, settings, geometry);
    if (!tuned) return message("至少需要 4 个有效实测点才能自动寻优。");
    settings = checkedSettings(tuned.settings);
    syncControls();
    updateFit(true);
    const v = tuned.stats?.validation;
    message(v
      ? "已自动寻优：留一验证 " + format(v.angle, 2) + "° / " + format(v.rpm, 0) + " RPM。"
      : "已应用自动寻优参数。");
  });
  byId("reset-controls").addEventListener("click", () => { settings = { ...fit.defaults }; syncControls(); updateFit(true); });
  byId("mirror").addEventListener("change", () => updateFit());
  byId("extrapolate").addEventListener("change", draw);
  byId("show-blocked").addEventListener("change", renderTable);
  byId("query-x").addEventListener("change", event => setQuery(event.target.valueAsNumber, query.y));
  byId("query-y").addEventListener("change", event => setQuery(query.x, event.target.valueAsNumber));
  byId("clear-selection").addEventListener("click", () => selectPoint(null));
  byId("edit-form").addEventListener("submit", event => {
    event.preventDefault();
    if (!selected || !event.target.reportValidity()) return;
    selected.angle = byId("edit-angle").valueAsNumber;
    selected.rpm = byId("edit-rpm").valueAsNumber;
    selected.weight = byId("edit-weight").valueAsNumber;
    updateFit(true);
  });
  field.addEventListener("click", event => {
    if (!fieldBounds) return;
    const position = field.getBoundingClientRect(), mx = event.clientX - position.left, my = event.clientY - position.top;
    const { left, top, size, unit, sx, sy } = fieldBounds;
    if (mx < left || mx > left + size || my < top || my > top + size) return;
    const hits = points.filter(point => point.angle !== null).map(point =>
      ({ point, distance: Math.hypot(sx(point.x) - mx, sy(point.y) - my) })).sort((a, b) => a.distance - b.distance);
    if (hits[0]?.distance <= 8) selectPoint(hits[0].point);
    else setQuery((mx - left) / unit, (top + size - my) / unit);
  });
  byId("csv").addEventListener("click", saveCsv);
  byId("save").addEventListener("click", () => {
    if (!model) return message("至少需要一个参与拟合的实测点。");
    download("shot_model.json", JSON.stringify({
      version: 2, kind: "biobuzz-shot-model", points, baselinePoints, settings, geometry, model, equation: equationText(), stats
    }, null, 2), "application/json");
  });
  byId("copy-equations").addEventListener("click", async () => {
    const text = equationText() +
      "\nx/y 为云台转轴坐标。右半场先用 x → 6 − x，再令 s = −1；左半场 s = 1。u、v、d 限于 ±3。";
    try {
      await navigator.clipboard.writeText(text);
      message("方程已复制。");
    } catch {
      const selection = window.getSelection(), range = document.createRange();
      range.selectNodeContents(byId("equations"));
      selection.removeAllRanges(); selection.addRange(range);
      message("已选中方程，可按 Ctrl+C 复制。");
    }
  });

  function parseInputRecords(text, fallbackGeometry) {
    if (text.startsWith("{")) {
      const project = JSON.parse(text);
      if (![1, 2].includes(project?.version) || !Array.isArray(project.points) ||
          project.points.length > 1200 || !project.points.every(fit.validPoint)) {
        throw new Error("方案中缺少有效的 06 测点。");
      }
      return {
        incomingPoints: project.points.map(normalizedPoint),
        incomingGeometry: project.geometry ? fit.checkedGeometry(project.geometry) : fallbackGeometry,
        project
      };
    }
    const parsed = fit.parse(text, fallbackGeometry);
    if (parsed.errors.length) throw new Error("这些点位无法识别 " + parsed.errors.slice(0, 4).join("、"));
    if (!parsed.points.length) throw new Error("请粘贴至少一条 07 点位，例如 110:-4675/3100。");
    return {
      incomingPoints: parsed.points.map(normalizedPoint),
      incomingGeometry: fallbackGeometry,
      project: null
    };
  }

  function smartSelectAnchors(incomingUsable) {
    const baseMap = new Map(baselinePoints.filter(fit.usable).map(point => [point.id, point]));
    const changed = incomingUsable.filter(point => {
      const ref = baseMap.get(point.id);
      return !ref || Math.abs(fit.wrap(point.angle - ref.angle)) > 0.05 || Math.abs(point.rpm - ref.rpm) > 1;
    });
    if (incomingUsable.length <= 8) return incomingUsable;
    if (changed.length >= 1 && changed.length <= 12) return changed;
    const centers = incomingUsable.filter(point => point.id[2] === "0");
    return centers.length ? centers : incomingUsable;
  }

  function getRecalOptions() {
    return {
      strategy: byId("recal-strategy").value,
      angleMode: byId("recal-angle-mode").value,
      rpmOrder: Number(byId("recal-rpm-order").value)
    };
  }

  const dialog = byId("import-dialog");
  byId("open-import").addEventListener("click", () => { byId("import-error").textContent = ""; dialog.showModal(); });
  byId("close-import").addEventListener("click", () => dialog.close());
  byId("import-file").addEventListener("change", async event => {
    const file = event.target.files[0];
    if (file) {
      byId("import-text").value = await file.text();
      event.target.value = "";
    }
  });
  byId("import-form").addEventListener("submit", event => {
    event.preventDefault();
    const text = byId("import-text").value.trim();
    const asRecalibrate = byId("import-recalibrate").checked;
    try {
      let notice;
      if (asRecalibrate) {
        const { incomingPoints, incomingGeometry } = parseInputRecords(text, geometry);
        const usableIncoming = incomingPoints.filter(fit.usable);
        if (!usableIncoming.length) throw new Error("导入数据中没有可用于少点校准的有效实测点。");
        const anchorSubset = smartSelectAnchors(usableIncoming);
        const result = fit.recalibrate(baselinePoints, anchorSubset, getRecalOptions(), settings, incomingGeometry);
        if (!result) throw new Error("少点迁移校准失败，请检查当前基准与锚点数据。");
        geometry = incomingGeometry;
        points = result.points.map(point => ({ ...point }));
        notice = "已用 " + result.anchorCount + " 个新角度锚点校准全部 " + result.calibratedCount +
          " 个精测点（平均 ΔRPM " + signed(result.summary.meanDeltaRpm, 0) + "）。";
      } else if (text.startsWith("{")) {
        applyProject(JSON.parse(text));
        notice = "已导入 " + points.length + " 条记录。";
      } else {
        const parsed = fit.parse(text, geometry);
        if (parsed.errors.length) throw new Error("这些点位无法识别 " + parsed.errors.slice(0, 4).join("、"));
        if (!parsed.points.length) throw new Error("请粘贴至少一条 07 点位，例如 110:-4675/3100。");
        const merged = new Map(points.map(point => [pointKey(point), point]));
        for (const point of parsed.points) merged.set(pointKey(point), point);
        points = [...merged.values()].sort((a, b) => a.id.localeCompare(b.id));
        notice = "已导入 " + parsed.points.length + " 条记录。";
      }
      selectPoint(null);
      syncControls();
      updateFit(true);
      byId("import-text").value = "";
      byId("import-recalibrate").checked = false;
      dialog.close();
      message(notice);
    } catch (error) {
      byId("import-error").textContent = error.message;
    }
  });

  const recalDialog = byId("recalibrate-dialog");

  function renderRecalBaselineInfo() {
    const usableBase = baselinePoints.filter(fit.usable);
    const cornerCount = usableBase.filter(point => point.id[2] !== "0").length;
    byId("recal-baseline-info").textContent =
      "当前 24 点基准：" + usableBase.length + " 个可用点（含 " + cornerCount + " 个精测角点）";
  }

  function buildRecalAnchors(preferredIds = null, overridePoints = null) {
    const baseById = new Map(baselinePoints.filter(fit.usable).map(point => [point.id, point]));
    const currentById = new Map(points.filter(fit.usable).map(point => [point.id, point]));
    const overrideById = overridePoints
      ? new Map(overridePoints.filter(fit.usable).map(point => [point.id, point]))
      : new Map();
    const ids = [];
    const seen = new Set();
    for (const point of baselinePoints) {
      if (fit.usable(point) && point.id[2] === "0" && !seen.has(point.id)) {
        seen.add(point.id);
        ids.push(point.id);
      }
    }
    if (overridePoints) {
      for (const point of overridePoints) {
        if (fit.usable(point) && !seen.has(point.id)) {
          seen.add(point.id);
          ids.push(point.id);
        }
      }
    }
    for (const point of points) {
      if (fit.usable(point) && !seen.has(point.id)) {
        const ref = baseById.get(point.id);
        const modified = !ref || Math.abs(fit.wrap(point.angle - ref.angle)) > 0.05 || Math.abs(point.rpm - ref.rpm) > 1;
        if (point.id[2] === "0" || modified) {
          seen.add(point.id);
          ids.push(point.id);
        }
      }
    }
    ids.sort((a, b) => a.localeCompare(b));
    recalAnchors = ids.map(id => {
      const ref = baseById.get(id);
      const ov = overrideById.get(id);
      const cur = currentById.get(id);
      const source = ov || cur || ref;
      return {
        id,
        name: fit.pointName(id),
        x: source.x,
        y: source.y,
        weight: source.weight || 1,
        oldAngle: ref ? ref.angle : null,
        oldRpm: ref ? ref.rpm : null,
        newAngle: Number(source.angle.toFixed(2)),
        newRpm: Math.round(source.rpm),
        checked: false
      };
    });
    if (preferredIds) {
      recalAnchors.forEach(row => { row.checked = preferredIds.has(row.id); });
    } else {
      const changed = recalAnchors.filter(row => row.oldAngle !== null &&
        (Math.abs(fit.wrap(row.newAngle - row.oldAngle)) > 0.05 || Math.abs(row.newRpm - row.oldRpm) > 1));
      const useChanged = changed.length > 0 && changed.length < recalAnchors.length;
      recalAnchors.forEach(row => {
        row.checked = useChanged ? changed.includes(row) : row.id[2] === "0";
      });
    }
    renderRecalAnchors();
    updateRecalPreview();
  }

  function anchorDeltaText(row) {
    if (row.oldAngle === null) return "新增锚点";
    if (!Number.isFinite(row.newAngle) || !Number.isFinite(row.newRpm)) return "—";
    return signed(fit.wrap(row.newAngle - row.oldAngle), 2) + "° / " + signed(row.newRpm - row.oldRpm, 0);
  }

  function renderRecalAnchors() {
    const fragment = document.createDocumentFragment();
    for (const row of recalAnchors) {
      const tr = document.createElement("tr");
      const checkTd = document.createElement("td");
      const check = document.createElement("input");
      check.type = "checkbox";
      check.checked = row.checked;
      check.ariaLabel = "将 " + row.id + " 作为锚点";
      checkTd.append(check);

      const idTd = document.createElement("td");
      idTd.textContent = row.id + " " + row.name;

      const posTd = document.createElement("td");
      posTd.textContent = format(row.x, 3) + " / " + format(row.y, 3);

      const oldTd = document.createElement("td");
      oldTd.textContent = row.oldAngle === null ? "无基准" : format(row.oldAngle, 2) + "° / " + format(row.oldRpm, 0);

      const angleTd = document.createElement("td");
      const angleInput = document.createElement("input");
      angleInput.type = "number";
      angleInput.min = "-180";
      angleInput.max = "180";
      angleInput.step = "0.1";
      angleInput.value = Number.isFinite(row.newAngle) ? row.newAngle : "";
      angleTd.append(angleInput);

      const rpmTd = document.createElement("td");
      const rpmInput = document.createElement("input");
      rpmInput.type = "number";
      rpmInput.min = "1";
      rpmInput.max = "20000";
      rpmInput.step = "25";
      rpmInput.value = Number.isFinite(row.newRpm) ? row.newRpm : "";
      rpmTd.append(rpmInput);

      const deltaTd = document.createElement("td");
      deltaTd.className = "delta-badge";
      deltaTd.textContent = anchorDeltaText(row);
      row.deltaTd = deltaTd;

      check.addEventListener("change", () => {
        row.checked = check.checked;
        updateRecalPreview();
      });
      angleInput.addEventListener("input", () => {
        row.newAngle = angleInput.value === "" ? NaN : angleInput.valueAsNumber;
        if (Number.isFinite(row.newAngle) && row.oldAngle !== null && Math.abs(fit.wrap(row.newAngle - row.oldAngle)) > 0.05) {
          row.checked = true;
          check.checked = true;
        }
        deltaTd.textContent = anchorDeltaText(row);
        updateRecalPreview();
      });
      rpmInput.addEventListener("input", () => {
        row.newRpm = rpmInput.value === "" ? NaN : rpmInput.valueAsNumber;
        if (Number.isFinite(row.newRpm) && row.oldRpm !== null && Math.abs(row.newRpm - row.oldRpm) > 1) {
          row.checked = true;
          check.checked = true;
        }
        deltaTd.textContent = anchorDeltaText(row);
        updateRecalPreview();
      });

      tr.append(checkTd, idTd, posTd, oldTd, angleTd, rpmTd, deltaTd);
      fragment.append(tr);
    }
    byId("recal-anchors-body").replaceChildren(fragment);
  }

  function activeAnchorPoints() {
    const valid = [];
    for (const row of recalAnchors) {
      if (!row.checked) continue;
      const point = {
        id: row.id, x: row.x, y: row.y,
        angle: row.newAngle, rpm: row.newRpm,
        blocked: false, inactive: false, weight: row.weight || 1
      };
      if (fit.validPoint(point) && fit.usable(point)) valid.push(point);
    }
    return valid;
  }

  function updateRecalPreview() {
    const anchors = activeAnchorPoints();
    const errorEl = byId("recal-error");
    const applyBtn = byId("recal-apply");
    const previewBody = byId("recal-preview-body");
    if (!anchors.length) {
      errorEl.textContent = "请至少勾选 1 个有效的新角度实测锚点，并填入射向与 RPM。";
      applyBtn.disabled = true;
      byId("recal-summary").textContent = "等待有效锚点…";
      previewBody.replaceChildren();
      return;
    }
    const result = fit.recalibrate(baselinePoints, anchors, getRecalOptions(), settings, geometry);
    if (!result) {
      errorEl.textContent = "当前基准数据中缺少可迁移的实测点。";
      applyBtn.disabled = true;
      byId("recal-summary").textContent = "无法计算迁移结果。";
      previewBody.replaceChildren();
      return;
    }
    errorEl.textContent = "";
    applyBtn.disabled = false;
    const s = result.summary;
    const diagById = new Map((result.anchorDiagnostics || []).map(item => [item.id, item]));
    for (const row of recalAnchors) {
      if (!row.deltaTd) continue;
      const diag = row.checked ? diagById.get(row.id) : null;
      const baseText = anchorDeltaText(row);
      if (diag?.outlier) {
        row.deltaTd.className = "delta-badge warn";
        row.deltaTd.textContent = baseText + "（降权 w=" + format(Math.min(diag.angleWeight, diag.rpmWeight), 2) + "）";
      } else {
        row.deltaTd.className = "delta-badge";
        row.deltaTd.textContent = baseText;
      }
    }
    const [r0, sr] = s.rNorm;
    const rpmExpr = s.rpmCoeffs.map((value, i) =>
      (value < 0 ? " − " : i > 0 ? " + " : "") + format(Math.abs(value), 1) + (i === 0 ? "" : i === 1 ? "d" : "d^" + i)).join("");
    const [dgx, dgy] = s.goalShift || [0, 0];
    const angleText = s.angleMode === "keep" ? "保持原射向不变"
      : s.effAngleOrder === 0 ? "射向统一平移 " + signed(s.meanDeltaAngle, 2) + "°"
      : "2D MAP 目标偏移 (" + signed(dgx, 3) + ", " + signed(dgy, 3) + ") 格 · 零位 " + signed(s.deltaGoalOffsetDeg, 2) + "°";
    const outlierNote = s.outlierIds?.length
      ? "\n⚠ 抗噪降权锚点：" + s.outlierIds.join("、") + "（偏离群体趋势，Huber 已自动降低影响）"
      : "";
    byId("recal-summary").textContent =
      "锚点：" + result.anchorCount + " 点 → 待更新全部测点：" + result.calibratedCount + " 点" +
      (s.useCornerModelAsBaseline ? "（检测到格心已覆盖：已自动改用 24 点角点拟合面作为旧基准）" : "") +
      "\n加权平均偏移：射向 " + signed(s.meanDeltaAngle, 2) + "° · 转速 " + signed(s.meanDeltaRpm, 0) + " RPM" +
      "（近端 " + signed(s.nearDeltaRpm, 0) + " → 远端 " + signed(s.farDeltaRpm, 0) + " RPM）" +
      "\nΔRPM = " + rpmExpr + "  (d = (r − " + format(r0, 3) + ")/" + format(sr, 3) + ")  |  " + angleText + outlierNote;

    const fragment = document.createDocumentFragment();
    for (const row of result.comparisons) {
      const tr = document.createElement("tr");
      const values = [
        row.id + " " + row.name,
        format(row.x, 3) + " / " + format(row.y, 3),
        row.oldAngle === null ? "—" : format(row.oldAngle, 2) + "° / " + format(row.oldRpm, 0),
        format(row.newAngle, 2) + "° / " + format(row.newRpm, 0),
        row.dAngle === null ? "新增" : signed(row.dAngle, 2) + "° / " + signed(row.dRpm, 0),
        row.source
      ];
      values.forEach((value, idx) => {
        const td = document.createElement("td");
        td.textContent = value;
        if (idx === 4) {
          td.className = "delta-badge" + (Math.abs(row.dRpm || 0) >= 400 || Math.abs(row.dAngle || 0) >= 8 ? " warn" : "");
        }
        tr.append(td);
      });
      fragment.append(tr);
    }
    previewBody.replaceChildren(fragment);
  }

  function applyRecalPaste(text) {
    const errorEl = byId("recal-error");
    try {
      const { incomingPoints, incomingGeometry } = parseInputRecords(text.trim(), geometry);
      const usableIncoming = incomingPoints.filter(fit.usable);
      if (!usableIncoming.length) throw new Error("未找到可用的实测点。");
      geometry = incomingGeometry;
      const chosen = smartSelectAnchors(usableIncoming);
      buildRecalAnchors(new Set(chosen.map(point => point.id)), usableIncoming);
      errorEl.textContent = "";
      message("已解析 " + usableIncoming.length + " 个测点，自动选中 " + chosen.length + " 个作为新角度锚点。");
    } catch (error) {
      errorEl.textContent = error.message;
    }
  }

  byId("open-recalibrate").addEventListener("click", () => {
    byId("recal-error").textContent = "";
    renderRecalBaselineInfo();
    buildRecalAnchors();
    recalDialog.showModal();
  });
  byId("close-recalibrate").addEventListener("click", () => recalDialog.close());
  byId("recal-cancel").addEventListener("click", () => recalDialog.close());
  byId("recal-extract-centers").addEventListener("click", () => buildRecalAnchors(new Set(
    recalAnchors.filter(row => row.id[2] === "0").map(row => row.id)
  )));
  byId("recal-set-baseline").addEventListener("click", () => {
    baselinePoints = points.map(point => ({ ...point }));
    updateFit(true);
    renderRecalBaselineInfo();
    buildRecalAnchors();
    message("已将当前表格数据锁定为 24 点迁移基准。");
  });
  byId("recal-restore-baseline").addEventListener("click", () => {
    points = baselinePoints.map(point => ({ ...point }));
    selectPoint(null);
    updateFit(true);
    buildRecalAnchors();
    message("已还原为 24 点迁移基准。");
  });
  byId("recal-parse-btn").addEventListener("click", () => applyRecalPaste(byId("recal-paste").value));
  byId("recal-file").addEventListener("change", async event => {
    const file = event.target.files[0];
    if (!file) return;
    const text = await file.text();
    byId("recal-paste").value = text;
    event.target.value = "";
    applyRecalPaste(text);
  });
  for (const id of ["recal-strategy", "recal-angle-mode", "recal-rpm-order"]) {
    byId(id).addEventListener("change", updateRecalPreview);
  }
  byId("recal-apply").addEventListener("click", () => {
    const anchors = activeAnchorPoints();
    const result = fit.recalibrate(baselinePoints, anchors, getRecalOptions(), settings, geometry);
    if (!result) {
      byId("recal-error").textContent = "无法完成迁移校准，请检查锚点与基准数据。";
      return;
    }
    points = result.points.map(point => ({ ...point }));
    selectPoint(null);
    updateFit(true);
    recalDialog.close();
    message("已用 " + result.anchorCount + " 个新角度锚点校准全部 " + result.calibratedCount +
      " 个精测点（平均 ΔRPM " + signed(result.summary.meanDeltaRpm, 0) + "）。");
  });

  if (!("closedBy" in HTMLDialogElement.prototype)) {
    const outside = (e, d) => {
      const r = d.getBoundingClientRect();
      return e.clientX < r.left || e.clientX > r.right || e.clientY < r.top || e.clientY > r.bottom;
    };
    for (const d of document.querySelectorAll('dialog[closedby="any"]')) {
      let pressedOutside = false;
      d.addEventListener("pointerdown", e => { pressedOutside = outside(e, d); });
      d.addEventListener("pointerup", e => { if (pressedOutside && outside(e, d)) d.close(); });
    }
  }

  syncControls();
  updateFit();
  new ResizeObserver(draw).observe(field);
})();
