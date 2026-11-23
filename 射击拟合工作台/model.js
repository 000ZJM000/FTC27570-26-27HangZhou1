(function () {
  "use strict";

  const RAD = Math.PI / 180;
  const powers = [[0, 0], [1, 0], [0, 1], [2, 0], [1, 1], [0, 2], [3, 0], [2, 1], [1, 2], [0, 3]];
  const defaults = { angleOrder: 1, rpmOrder: 2, ridge: 0.18, angleTrim: 0, rpmTrim: 0 };
  const codeNames = ["格心", "左后", "右后", "右前", "左前"];
  const defaultGeometry = { tileMm: 609.6, turretRightMm: 0, turretForwardMm: 0 };

  function wrap(angle) {
    return ((angle + 180) % 360 + 360) % 360 - 180;
  }

  function pointName(id) {
    return codeNames[Number(id[2])] || "旧精测 " + id[2];
  }

  function parse(text, geometry = defaultGeometry) {
    const token = /^([1-6][1-6][0-4a-h])(?:@(-?\d+(?:\.\d+)?),(-?\d+(?:\.\d+)?))?:(?:(-?\d+)\/(\d+(?:\.\d+)?)|(\?))([!~]*)$/u;
    const points = new Map();
    const errors = [];
    for (const word of text.replace(/[−–]/g, "-").split(/\s+/).filter(Boolean)) {
      // Copying the whole 07 page can include its navigation header.
      if (!word.includes(":")) continue;
      const match = word.match(token);
      if (!match) {
        errors.push(word);
        continue;
      }
      const [, id, right, forward, angle, rpm, absent, flags] = match;
      const [row, column, code] = id;
      const edges = ["leftEdgeMm", "rightEdgeMm", "rearEdgeMm", "frontEdgeMm"];
      const knownCorners = edges.every(edge => Number.isFinite(geometry[edge]));
      if (right === undefined && code !== "0" && (!knownCorners || !/[1-4]/.test(code))) {
        errors.push(word);
        continue;
      }
      const cornerX = code === "0" ? 0.5 : /[14]/.test(code) ? geometry.leftEdgeMm / geometry.tileMm
        : 1 - geometry.rightEdgeMm / geometry.tileMm;
      const cornerY = code === "0" ? 0.5 : /[12]/.test(code) ? geometry.rearEdgeMm / geometry.tileMm
        : 1 - geometry.frontEdgeMm / geometry.tileMm;
      const x = right === undefined ? Number(column) - 1 + cornerX : Number(right);
      const y = forward === undefined ? Number(row) - 1 + cornerY : Number(forward);
      if (!(x >= 0 && x <= 6 && y >= 0 && y <= 6) ||
          (!absent && !(Math.abs(Number(angle)) <= 18000 && Number(rpm) > 0 && Number(rpm) <= 20000))) {
        errors.push(word);
        continue;
      }
      points.set(id + "@" + x + "," + y, {
        id, x, y, angle: absent ? null : Number(angle) / 100, rpm: absent ? null : Number(rpm),
        blocked: flags.includes("!"), inactive: flags.includes("~"), weight: 1
      });
    }
    return { points: [...points.values()], errors };
  }

  function validPoint(point) {
    return point && /^[1-6][1-6][0-4a-h]$/.test(point.id) &&
      Number.isFinite(point.x) && point.x >= 0 && point.x <= 6 &&
      Number.isFinite(point.y) && point.y >= 0 && point.y <= 6 &&
      ((point.angle === null && point.rpm === null) ||
        (Number.isFinite(point.angle) && Math.abs(point.angle) <= 180 &&
         Number.isFinite(point.rpm) && point.rpm > 0 && point.rpm <= 20000)) &&
      Number.isFinite(point.weight) && point.weight >= 0 && point.weight <= 5;
  }

  function usable(point) {
    return point.angle !== null && !point.blocked && !point.inactive && point.weight > 0;
  }

  function fitPoints(points) {
    const cellsWithCorners = new Set(points.filter(point => /[1-4]$/.test(point.id) && !point.inactive &&
      (point.angle !== null || point.blocked)).map(point => point.id.slice(0, 2)));
    return points.filter(point => usable(point) &&
      (!/[a-h]$/.test(point.id) || !cellsWithCorners.has(point.id.slice(0, 2))));
  }

  function checkedGeometry(geometry) {
    if (!geometry || !Number.isFinite(geometry.tileMm) || geometry.tileMm <= 0 ||
        !Number.isFinite(geometry.turretRightMm) || !Number.isFinite(geometry.turretForwardMm)) return defaultGeometry;
    return geometry;
  }

  function solve(matrix, vector) {
    const size = vector.length;
    const rows = matrix.map((row, index) => row.concat(vector[index]));
    for (let column = 0; column < size; column++) {
      let pivot = column;
      for (let row = column + 1; row < size; row++) {
        if (Math.abs(rows[row][column]) > Math.abs(rows[pivot][column])) pivot = row;
      }
      if (Math.abs(rows[pivot][column]) < 1e-12) return null;
      [rows[pivot], rows[column]] = [rows[column], rows[pivot]];
      const divisor = rows[column][column];
      for (let j = column; j <= size; j++) rows[column][j] /= divisor;
      for (let row = 0; row < size; row++) {
        if (row === column) continue;
        const factor = rows[row][column];
        for (let j = column; j <= size; j++) rows[row][j] -= factor * rows[column][j];
      }
    }
    return rows.map(row => row[size]);
  }

  function leastSquares(points, features, observed, penalty, prior) {
    const size = penalty.length;
    const matrix = penalty.map((diagonal, index) =>
      penalty.map((_, column) => column === index ? diagonal : 0));
    const vector = penalty.map((diagonal, index) => diagonal * (prior ? prior[index] : 0));
    for (const point of points) {
      const values = features(point);
      const target = observed(point);
      for (let i = 0; i < size; i++) {
        vector[i] += point.weight * values[i] * target;
        for (let j = 0; j < size; j++) matrix[i][j] += point.weight * values[i] * values[j];
      }
    }
    return solve(matrix, vector);
  }

  function bearing(x, y, goal) {
    return Math.atan2(x - goal[0], goal[1] - y) / RAD;
  }

  function fitGoal(points) {
    const features = point => [Math.cos(point.angle * RAD), Math.sin(point.angle * RAD)];
    let goal = leastSquares(points, features, point => {
      const [c, s] = features(point);
      return c * point.x + s * point.y;
    }, [0.018, 0.018], [2, 2.5]);
    const bound = values => values.map(value => Math.min(6, Math.max(0, value)));
    goal = bound(goal);
    const loss = candidate => points.reduce((sum, point) =>
      sum + point.weight * wrap(point.angle - bearing(point.x, point.y, candidate)) ** 2, 0);
    for (let step = 0; step < 16; step++) {
      const correction = leastSquares(points, point => {
        const dx = goal[0] - point.x, dy = goal[1] - point.y;
        const distanceSquared = Math.max(dx * dx + dy * dy, 0.01);
        return [-dy / distanceSquared, dx / distanceSquared];
      }, point => RAD * wrap(point.angle - bearing(point.x, point.y, goal)), [0.025, 0.025]);
      if (!correction || Math.hypot(...correction) < 1e-7) break;
      const previousLoss = loss(goal);
      let improved = false;
      for (let fraction = 1; fraction >= 0.03125; fraction /= 2) {
        const trial = bound(goal.map((value, index) => value + fraction * correction[index]));
        if (loss(trial) < previousLoss) {
          goal = trial;
          improved = true;
          break;
        }
      }
      if (!improved) break;
    }
    return goal;
  }

  function polynomialTerms(order) {
    return powers.filter(([xPower, yPower]) => xPower + yPower <= order);
  }

  function normalization(points, key) {
    const total = points.reduce((sum, point) => sum + point.weight, 0);
    const mean = points.reduce((sum, point) => sum + point.weight * point[key], 0) / total;
    const square = points.reduce((sum, point) => sum + point.weight * point[key] ** 2, 0) / total;
    return [mean, Math.sqrt(Math.max(0.0625, square - mean * mean))];
  }

  const normalized = (value, [center, scale]) => Math.max(-3, Math.min(3, (value - center) / scale));

  function basis(x, y, poly) {
    const u = normalized(x, poly.x), v = normalized(y, poly.y);
    return poly.terms.map(([p, q]) => u ** p * v ** q);
  }

  function evaluate(poly, x, y) {
    return basis(x, y, poly).reduce((sum, term, index) => sum + term * poly.coefficients[index], 0);
  }

  function fit(points, settings = defaults, inputGeometry = defaultGeometry) {
    const geometry = checkedGeometry(inputGeometry);
    const participating = fitPoints(points).map(point => {
      const right = point.x >= 3;
      const pivotX = point.x + geometry.turretRightMm / geometry.tileMm;
      return { ...point, x: right ? 6 - pivotX : pivotX, y: point.y + geometry.turretForwardMm / geometry.tileMm,
        angle: right ? -point.angle : point.angle, chassisX: right ? 6 - point.x : point.x, chassisY: point.y };
    });
    if (participating.length === 0) return null;
    const goal = fitGoal(participating);
    const ridge = Math.max(0.00001, settings.ridge);
    const terms = polynomialTerms(settings.angleOrder);
    const angular = { terms, x: normalization(participating, "x"), y: normalization(participating, "y"), coefficients: [] };
    if (terms.length) {
      angular.coefficients = leastSquares(participating, point => basis(point.x, point.y, angular),
        point => wrap(point.angle - bearing(point.x, point.y, goal)),
        terms.map(([p, q]) => p + q === 0 ? 1e-9 : ridge * (p + q) ** 2));
    }
    const radial = participating.map(point => ({ ...point, r: Math.hypot(point.x - goal[0], point.y - goal[1]) }));
    const radius = normalization(radial, "r");
    const speedPowers = Array.from({ length: settings.rpmOrder + 1 }, (_, power) => power);
    const speed = { radius, coefficients: leastSquares(radial,
      point => speedPowers.map(power => normalized(point.r, radius) ** power), point => point.rpm,
      speedPowers.map(power => power === 0 ? 1e-9 : ridge * power ** 2)) };
    return { goal, angular, speed, angleTrim: settings.angleTrim, rpmTrim: settings.rpmTrim, geometry,
      hull: hull(participating.map(point => ({ x: point.chassisX, y: point.chassisY }))), count: participating.length };
  }

  function predict(model, x, y) {
    if (!model) return null;
    const sign = x < 3 ? 1 : -1;
    const pivotX = x + model.geometry.turretRightMm / model.geometry.tileMm;
    const pivotY = y + model.geometry.turretForwardMm / model.geometry.tileMm;
    const localX = sign === 1 ? pivotX : 6 - pivotX;
    const angle = bearing(localX, pivotY, model.goal) + evaluate(model.angular, localX, pivotY);
    const r = normalized(Math.hypot(localX - model.goal[0], pivotY - model.goal[1]), model.speed.radius);
    const rpm = model.speed.coefficients.reduceRight((sum, term) => sum * r + term, 0) + model.rpmTrim;
    return { angle: wrap(sign * angle + model.angleTrim), rpm: Math.max(0, Math.min(20000, rpm)) };
  }

  function error(point, model, settings) {
    if (point.angle === null || !model) return null;
    const fitted = predict(model, point.x, point.y, settings);
    return { angle: wrap(point.angle - fitted.angle), rpm: point.rpm - fitted.rpm };
  }

  function diagnostics(points, model, settings) {
    const measured = fitPoints(points);
    if (!model || !measured.length) return null;
    const weight = measured.reduce((sum, point) => sum + point.weight, 0);
    const rmse = values => ({
      angle: Math.sqrt(values.reduce((sum, value, i) => sum + measured[i].weight * value.angle ** 2, 0) / weight),
      rpm: Math.sqrt(values.reduce((sum, value, i) => sum + measured[i].weight * value.rpm ** 2, 0) / weight)
    });
    const inside = rmse(measured.map(point => error(point, model, settings)));
    const validation = measured.length < 4 ? null : rmse(measured.map((point, index) =>
      error(point, fit(measured.filter((_, other) => index !== other), settings, model.geometry), settings)));
    return { inside, validation };
  }

  function cross(origin, left, right) {
    return (left.x - origin.x) * (right.y - origin.y) - (left.y - origin.y) * (right.x - origin.x);
  }

  function hull(points) {
    const sorted = [...points].sort((a, b) => a.x - b.x || a.y - b.y);
    if (sorted.length < 3) return sorted;
    const chain = list => {
      const edges = [];
      for (const point of list) {
        while (edges.length > 1 && cross(edges.at(-2), edges.at(-1), point) <= 0) edges.pop();
        edges.push(point);
      }
      return edges.slice(0, -1);
    };
    return [...chain(sorted), ...chain(sorted.reverse())];
  }

  function supported(model, x, y) {
    if (!model || model.hull.length < 3) return false;
    const local = { x: x < 3 ? x : 6 - x, y };
    return model.hull.every((point, index, points) =>
      cross(point, points[(index + 1) % points.length], local) >= -1e-8);
  }

  function canonicalPivot(point, geometry, goal) {
    const right = point.x >= 3, sign = right ? -1 : 1;
    const pivotX = point.x + geometry.turretRightMm / geometry.tileMm;
    const pivotY = point.y + geometry.turretForwardMm / geometry.tileMm;
    const localX = right ? 6 - pivotX : pivotX;
    const localAngle = point.angle === null ? null : right ? -point.angle : point.angle;
    const r = goal ? Math.hypot(localX - goal[0], pivotY - goal[1]) : 0;
    return { right, sign, pivotX, pivotY, localX, localAngle, r };
  }

  function recalibrate(baselinePoints, anchorPoints, options = {}, settings = defaults, inputGeometry = defaultGeometry) {
    const strategy = options.strategy || "local-anchor";
    const angleMode = options.angleMode || "auto";
    const rpmOrder = Number.isFinite(options.rpmOrder) ? Math.max(0, Math.min(2, Math.round(options.rpmOrder))) : 2;
    const usableBase = (baselinePoints || []).filter(usable);
    const usableAnchors = (anchorPoints || []).filter(usable);
    if (!usableAnchors.length || !usableBase.length) return null;

    const geometry = checkedGeometry(inputGeometry);
    const baseModel = fit(baselinePoints, settings, geometry);
    const anchorById = new Map(usableAnchors.map(point => [point.id, point]));
    const baseById = new Map(usableBase.map(point => [point.id, point]));
    const nonAnchorBase = usableBase.filter(point => !anchorById.has(point.id));
    const nonAnchorModel = nonAnchorBase.length >= 3 ? fit(nonAnchorBase, settings, geometry) : null;

    const matchedCount = usableAnchors.filter(a => baseById.has(a.id)).length;
    const allMatchedIdentical = matchedCount > 0 && usableAnchors.every(a => {
      const b = baseById.get(a.id);
      return !b || (Math.abs(wrap(a.angle - b.angle)) < 1e-6 && Math.abs(a.rpm - b.rpm) < 1e-6);
    });
    const useCornerModelAsBaseline = allMatchedIdentical && Boolean(nonAnchorModel);
    const refModel = useCornerModelAsBaseline ? nonAnchorModel : (baseModel || fit(usableAnchors, settings, geometry));
    const goal = refModel ? refModel.goal : [2, 2.5];

    const anchorData = usableAnchors.map(a => {
      const ca = canonicalPivot(a, geometry, goal);
      const b = !useCornerModelAsBaseline ? baseById.get(a.id) : null;
      let refLocalAngle, refRpm;
      if (b && (b.x >= 3) === (a.x >= 3)) {
        const cb = canonicalPivot(b, geometry, goal);
        refLocalAngle = cb.localAngle;
        refRpm = b.rpm;
      } else if (refModel) {
        const pred = predict(refModel, a.x, a.y);
        refLocalAngle = ca.sign * wrap(pred.angle - refModel.angleTrim);
        refRpm = pred.rpm;
      } else {
        refLocalAngle = ca.localAngle;
        refRpm = a.rpm;
      }
      const dAngle = wrap(ca.localAngle - refLocalAngle);
      const dRpm = a.rpm - refRpm;
      return { anchor: a, ...ca, refLocalAngle, refRpm, dAngle, dRpm, weight: a.weight > 0 ? a.weight : 1 };
    });

    const rNorm = normalization(anchorData, "r");
    const rValues = anchorData.map(d => d.r);
    const rMin = Math.min(...rValues), rMax = Math.max(...rValues), rSpan = rMax - rMin;
    const effRpmOrder = anchorData.length <= 1 || rSpan < 0.35 ? 0
      : anchorData.length <= 3 || rSpan < 1.1 ? Math.min(1, rpmOrder)
      : Math.min(2, rpmOrder, anchorData.length - 1);
    const rpmPowers = Array.from({ length: effRpmOrder + 1 }, (_, i) => i);
    const rpmPenalties = [1e-9, 0.12, 0.48].slice(0, effRpmOrder + 1);
    const rpmCoeffs = leastSquares(anchorData, d => rpmPowers.map(p => normalized(d.r, rNorm) ** p),
      d => d.dRpm, rpmPenalties) || [0];
    const evalDeltaRpm = r => rpmCoeffs.reduceRight((sum, c) => sum * normalized(r, rNorm) + c, 0);

    const totalWeight = anchorData.reduce((s, d) => s + d.weight, 0);
    const meanDeltaAngle = anchorData.reduce((s, d) => s + d.weight * d.dAngle, 0) / totalWeight;
    const meanDeltaRpm = anchorData.reduce((s, d) => s + d.weight * d.dRpm, 0) / totalWeight;

    let effAngleOrder, angleCoeffs, evalDeltaAngle;
    if (angleMode === "keep") {
      effAngleOrder = -1;
      angleCoeffs = [0];
      evalDeltaAngle = () => 0;
    } else if (angleMode === "offset" || anchorData.length <= 2 || rSpan < 0.5) {
      effAngleOrder = 0;
      angleCoeffs = [meanDeltaAngle];
      evalDeltaAngle = () => meanDeltaAngle;
    } else {
      effAngleOrder = 1;
      angleCoeffs = leastSquares(anchorData, d => [1, normalized(d.r, rNorm)],
        d => d.dAngle, [1e-9, 0.25]) || [meanDeltaAngle, 0];
      evalDeltaAngle = r => angleCoeffs[0] + (angleCoeffs[1] || 0) * normalized(r, rNorm);
    }

    const anchorModel = strategy === "residual-stack"
      ? fit(usableAnchors, {
        ...settings,
        angleOrder: Math.min(1, Math.max(0, settings.angleOrder)),
        rpmOrder: Math.min(2, settings.rpmOrder)
      }, geometry)
      : null;

    const anchorDataById = new Map(anchorData.map(d => [d.anchor.id, d]));
    const existingIds = new Set(baselinePoints.map(p => p.id));
    const comparisons = [];
    const calibratedPoints = baselinePoints.map(p => {
      if (p.angle === null || p.blocked || p.inactive) return { ...p };
      const cp = canonicalPivot(p, geometry, goal);
      const direct = anchorDataById.get(p.id);
      let newAngle, newRpm, source;
      if (direct && (direct.anchor.x >= 3) === (p.x >= 3) && strategy !== "global-curve") {
        newRpm = Math.max(1, Math.min(20000, Math.round(direct.anchor.rpm)));
        newAngle = angleMode === "keep" ? p.angle
          : angleMode === "offset" ? Number(wrap(p.angle + cp.sign * evalDeltaAngle(cp.r)).toFixed(2))
          : Number(direct.anchor.angle.toFixed(2));
        source = "新锚点直写";
      } else if (strategy === "residual-stack" && anchorModel && refModel) {
        const oldPred = predict(refModel, p.x, p.y), newPred = predict(anchorModel, p.x, p.y);
        const resAngle = wrap(p.angle - oldPred.angle), resRpm = p.rpm - oldPred.rpm;
        newRpm = Math.max(1, Math.min(20000, Math.round(newPred.rpm + resRpm)));
        newAngle = angleMode === "keep" ? p.angle
          : angleMode === "offset" ? Number(wrap(p.angle + cp.sign * evalDeltaAngle(cp.r)).toFixed(2))
          : Number(wrap(newPred.angle + resAngle).toFixed(2));
        source = "新曲线+旧残差";
      } else if (strategy === "local-anchor") {
        const centerId = p.id.slice(0, 2) + "0";
        const cellAnchor = anchorDataById.get(centerId);
        if (cellAnchor && cellAnchor.right === cp.right) {
          const dRpm = cellAnchor.dRpm + (evalDeltaRpm(cp.r) - evalDeltaRpm(cellAnchor.r));
          const dAngleLocal = angleMode === "keep" ? 0
            : angleMode === "offset" ? evalDeltaAngle(cp.r)
            : cellAnchor.dAngle + (evalDeltaAngle(cp.r) - evalDeltaAngle(cellAnchor.r));
          newRpm = Math.max(1, Math.min(20000, Math.round(p.rpm + dRpm)));
          newAngle = Number(wrap(p.angle + cp.sign * dAngleLocal).toFixed(2));
          source = "同格锚定+径向";
        } else {
          newRpm = Math.max(1, Math.min(20000, Math.round(p.rpm + evalDeltaRpm(cp.r))));
          newAngle = Number(wrap(p.angle + cp.sign * evalDeltaAngle(cp.r)).toFixed(2));
          source = "径向曲线迁移";
        }
      } else {
        newRpm = Math.max(1, Math.min(20000, Math.round(p.rpm + evalDeltaRpm(cp.r))));
        newAngle = Number(wrap(p.angle + cp.sign * evalDeltaAngle(cp.r)).toFixed(2));
        source = direct ? "全局曲线(锚点)" : "全局曲线迁移";
      }
      comparisons.push({
        id: p.id, name: pointName(p.id), x: p.x, y: p.y,
        oldAngle: p.angle, oldRpm: p.rpm, newAngle, newRpm,
        dAngle: wrap(newAngle - p.angle), dRpm: newRpm - p.rpm, source
      });
      return { ...p, angle: newAngle, rpm: newRpm };
    });

    for (const a of usableAnchors) {
      if (existingIds.has(a.id)) continue;
      const newAngle = Number(a.angle.toFixed(2));
      const newRpm = Math.max(1, Math.min(20000, Math.round(a.rpm)));
      calibratedPoints.push({ ...a, angle: newAngle, rpm: newRpm });
      comparisons.push({
        id: a.id, name: pointName(a.id), x: a.x, y: a.y,
        oldAngle: null, oldRpm: null, newAngle, newRpm,
        dAngle: null, dRpm: null, source: "新增锚点"
      });
    }
    calibratedPoints.sort((a, b) => a.id.localeCompare(b.id));

    return {
      points: calibratedPoints,
      comparisons,
      anchorCount: usableAnchors.length,
      calibratedCount: comparisons.length,
      summary: {
        meanDeltaAngle,
        meanDeltaRpm,
        nearDeltaRpm: evalDeltaRpm(rMin),
        farDeltaRpm: evalDeltaRpm(rMax),
        rMin,
        rMax,
        rNorm,
        rpmCoeffs,
        angleCoeffs,
        effRpmOrder,
        effAngleOrder,
        strategy,
        angleMode,
        useCornerModelAsBaseline
      }
    };
  }

  const api = { defaults, defaultGeometry, checkedGeometry, parse, validPoint, usable, pointName, wrap, fit, predict, error, diagnostics, supported, recalibrate };
  globalThis.ShotFit = api;
  if (typeof module === "object" && module.exports) module.exports = api;
})();
