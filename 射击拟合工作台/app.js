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
  let settings = { ...fit.defaults };
  let geometry = initialGeometry;
  let selected = null;
  let model, stats, rpmLow, rpmHigh, fitFrame, fieldBounds;
  let blockedTiles = new Set();
  let feedbackTimer;

  function checkedSettings(source = {}) {
    const value = (key, low, high) => Number.isFinite(source[key]) ? clamp(source[key], low, high) : fit.defaults[key];
    return {
      angleOrder: Math.round(value("angleOrder", -1, 2)),
      rpmOrder: Math.round(value("rpmOrder", 0, 3)),
      ridge: value("ridge", 0.001, 10),
      angleTrim: value("angleTrim", -45, 45),
      rpmTrim: value("rpmTrim", -1000, 1000)
    };
  }

  function applyProject(project) {
    if (![1, 2].includes(project?.version) || !Array.isArray(project.points) ||
        project.points.length > 1200 || !project.points.every(fit.validPoint)) {
      throw new Error("方案中缺少有效的 06 测点。");
    }
    points = project.points.map(point => ({ ...point, blocked: Boolean(point.blocked), inactive: Boolean(point.inactive) }));
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
    const [x0, sx] = model.angular.x, [y0, sy] = model.angular.y;
    const [r0, sr] = model.speed.radius;
    const radial = model.speed.coefficients.map((value, i) =>
      (value < 0 ? " − " : i > 0 ? " + " : "") + format(Math.abs(value), 3) + (i === 0 ? "" : i === 1 ? "d" : "d^" + i)).join("");
    return "θ = s × (atan2(x − " + format(right, 5) + ", " + format(forward, 5) + " − y) × 180/π + P)" +
      trim(settings.angleTrim) + "\nP = " + polynomial(model.angular) +
      "\nu = (x − " + format(x0, 4) + ")/" + format(sx, 4) + ", v = (y − " + format(y0, 4) + ")/" + format(sy, 4) +
      "\nRPM = " + radial + trim(settings.rpmTrim) +
      "\nd = (到目标的距离 − " + format(r0, 4) + ")/" + format(sr, 4);
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
    byId("angle-validation").textContent = "留一验证 " + (stats?.validation ? format(stats.validation.angle, 2) + "°" : "—");
    byId("rpm-validation").textContent = "留一验证 " + (stats?.validation ? format(stats.validation.rpm, 0) + " RPM" : "—");
    byId("equations").textContent = equationText();
    byId("rpm-low").textContent = format(rpmLow, 0);
    byId("rpm-high").textContent = format(rpmHigh, 0);
    byId("ridge-value").textContent = "λ " + settings.ridge.toPrecision(3);
    renderTable();
    renderSelection();
    draw();
    if (persist) {
      try { localStorage.setItem(storageKey, JSON.stringify({ version: 2, points, settings, geometry })); } catch { /* file export is still available */ }
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
      version: 2, kind: "biobuzz-shot-model", points, settings, geometry, model, equation: equationText(), stats
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
    try {
      let count;
      if (text.startsWith("{")) {
        applyProject(JSON.parse(text)); count = points.length;
      } else {
        const parsed = fit.parse(text, geometry);
        if (parsed.errors.length) throw new Error("这些点位无法识别 " + parsed.errors.slice(0, 4).join("、"));
        if (!parsed.points.length) throw new Error("请粘贴至少一条 07 点位，例如 110:-4675/3100。");
        const merged = new Map(points.map(point => [pointKey(point), point]));
        for (const point of parsed.points) merged.set(pointKey(point), point);
        points = [...merged.values()].sort((a, b) => a.id.localeCompare(b.id));
        count = parsed.points.length;
      }
      selectPoint(null);
      syncControls();
      updateFit(true);
      byId("import-text").value = "";
      dialog.close();
      message("已导入 " + count + " 条记录。");
    } catch (error) {
      byId("import-error").textContent = error.message;
    }
  });

  syncControls();
  updateFit();
  new ResizeObserver(draw).observe(field);
})();
