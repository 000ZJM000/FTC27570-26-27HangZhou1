(function () {
  "use strict";

  const RAD = Math.PI / 180;
  const powers = [[0, 0], [1, 0], [0, 1], [2, 0], [1, 1], [0, 2], [3, 0], [2, 1], [1, 2], [0, 3]];
  const defaults = {
    angleOrder: -1,
    rpmOrder: 2,
    ridge: 0.18,
    angleTrim: 0,
    rpmTrim: 0,
    robust: true,
    directionalRpm: true
  };
  const codeNames = ["格心", "左后", "右后", "右前", "左前"];
  const defaultGeometry = { tileMm: 609.6, turretRightMm: 0, turretForwardMm: 0 };

  function clamp(value, low, high) {
    return Math.min(high, Math.max(low, value));
  }

  function wrap(angle) {
    return ((angle + 180) % 360 + 360) % 360 - 180;
  }

  function median(values) {
    if (!values.length) return 0;
    const sorted = [...values].sort((a, b) => a - b);
    const mid = Math.floor(sorted.length / 2);
    return sorted.length % 2 ? sorted[mid] : 0.5 * (sorted[mid - 1] + sorted[mid]);
  }

  function huberDelta(residuals, floorScale) {
    if (!residuals.length) return 1.345 * floorScale;
    const mad = median(residuals.map(Math.abs));
    return 1.345 * Math.max(floorScale, 1.4826 * mad);
  }

  function huberWeight(error, delta) {
    const abs = Math.abs(error);
    return abs <= delta ? 1 : delta / abs;
  }

  function huberLoss(error, delta) {
    const abs = Math.abs(error);
    return abs <= delta ? 0.5 * abs * abs : delta * (abs - 0.5 * delta);
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

  function leastSquares(points, features, observed, penalty, prior = null, extraWeights = null) {
    const size = penalty.length;
    const matrix = penalty.map((diagonal, index) =>
      penalty.map((_, column) => column === index ? diagonal : 0));
    const vector = penalty.map((diagonal, index) => diagonal * (prior ? prior[index] : 0));
    for (let k = 0; k < points.length; k++) {
      const point = points[k];
      const w = point.weight * (extraWeights ? extraWeights[k] : 1);
      const values = features(point);
      const target = observed(point);
      for (let i = 0; i < size; i++) {
        vector[i] += w * values[i] * target;
        for (let j = 0; j < size; j++) matrix[i][j] += w * values[i] * values[j];
      }
    }
    return solve(matrix, vector);
  }

  function fitHuberLinear(points, features, observed, penalty, floorScale, robust = true, initialCenter = null) {
    const useRobust = robust && points.length >= 4;
    let hw = null;
    if (useRobust && initialCenter !== null) {
      const res0 = points.map(p => observed(p) - initialCenter);
      const d0 = huberDelta(res0, floorScale);
      hw = res0.map(e => huberWeight(e, d0));
    }
    let coeffs = null;
    const outer = useRobust ? 6 : 1;
    let delta = 1e9;
    for (let iter = 0; iter < outer; iter++) {
      coeffs = leastSquares(points, features, observed, penalty, null, hw);
      if (!coeffs) break;
      const res = points.map(p => {
        const phi = features(p);
        let pred = 0;
        for (let i = 0; i < phi.length; i++) pred += phi[i] * coeffs[i];
        return observed(p) - pred;
      });
      if (!useRobust) break;
      delta = huberDelta(res, floorScale);
      hw = res.map(e => huberWeight(e, delta));
    }
    const finalWeights = useRobust && coeffs
      ? points.map(p => {
        const phi = features(p);
        let pred = 0;
        for (let i = 0; i < phi.length; i++) pred += phi[i] * coeffs[i];
        return huberWeight(observed(p) - pred, delta);
      })
      : points.map(() => 1);
    return { coefficients: coeffs, weights: finalWeights };
  }

  function bearing(x, y, goal) {
    return Math.atan2(x - goal[0], goal[1] - y) / RAD;
  }

  function fitGoal(points, options = {}) {
    const priorX = Number.isFinite(options.priorX) ? options.priorX : 2.0;
    const priorY = Number.isFinite(options.priorY) ? options.priorY : 2.5;
    const priorTheta = Number.isFinite(options.priorTheta) ? options.priorTheta : 0.0;
    const robust = options.robust !== false;
    const goalPenalty = Number.isFinite(options.goalPenalty) ? options.goalPenalty : 9.0;
    const thetaPenalty = Number.isFinite(options.thetaPenalty) ? options.thetaPenalty : 0.01;
    const floorScale = Number.isFinite(options.huberFloor) ? options.huberFloor : 1.8;
    const observed = options.observed || (point => point.angle);

    let gx, gy;
    if (Array.isArray(options.seedGoal) && options.seedGoal.length === 2) {
      gx = clamp(options.seedGoal[0], 0, 6);
      gy = clamp(options.seedGoal[1], 0, 6);
    } else {
      const features = point => [Math.cos(observed(point) * RAD), Math.sin(observed(point) * RAD)];
      const seed = leastSquares(points, features, point => {
        const [c, s] = features(point);
        return c * point.x + s * point.y;
      }, [0.018, 0.018], [priorX, priorY]);
      gx = seed ? clamp(seed[0], 0, 6) : priorX;
      gy = seed ? clamp(seed[1], 0, 6) : priorY;
    }

    let th0 = clamp(median(points.map(p => wrap(observed(p) - bearing(p.x, p.y, [gx, gy])))), -45, 45);
    const useRobust = robust && points.length >= 4;
    const outerSteps = useRobust ? 4 : 1;
    let finalDelta = 1e9;

    for (let irls = 0; irls < outerSteps; irls++) {
      const initRes = points.map(p => wrap(observed(p) - (bearing(p.x, p.y, [gx, gy]) + th0)));
      const delta = useRobust ? huberDelta(initRes, floorScale) : 1e9;
      finalDelta = delta;

      const objective = (cx, cy, ct) => {
        let sum = 0.5 * goalPenalty * ((cx - priorX) ** 2 + (cy - priorY) ** 2) +
          0.5 * thetaPenalty * (ct - priorTheta) ** 2;
        for (const p of points) {
          const e = wrap(observed(p) - (bearing(p.x, p.y, [cx, cy]) + ct));
          sum += p.weight * (useRobust ? huberLoss(e, delta) : 0.5 * e * e);
        }
        return sum;
      };

      for (let step = 0; step < 10; step++) {
        const H = [
          [goalPenalty + 1e-3, 0, 0],
          [0, goalPenalty + 1e-3, 0],
          [0, 0, thetaPenalty + 1e-3]
        ];
        const g = [
          -goalPenalty * (gx - priorX),
          -goalPenalty * (gy - priorY),
          -thetaPenalty * (th0 - priorTheta)
        ];
        for (const p of points) {
          const e = wrap(observed(p) - (bearing(p.x, p.y, [gx, gy]) + th0));
          const hw = useRobust ? huberWeight(e, delta) : 1;
          const w = p.weight * hw;
          const dx = gx - p.x, dy = gy - p.y;
          const d2 = Math.max(dx * dx + dy * dy, 0.01);
          const J = [(-dy / d2) / RAD, (dx / d2) / RAD, 1];
          for (let i = 0; i < 3; i++) {
            g[i] += w * J[i] * e;
            for (let j = 0; j < 3; j++) H[i][j] += w * J[i] * J[j];
          }
        }
        const corr = solve(H, g);
        if (!corr || Math.hypot(corr[0], corr[1], corr[2] * 0.05) < 1e-7) break;
        const currentLoss = objective(gx, gy, th0);
        let improved = false;
        for (let fraction = 1; fraction >= 0.03125; fraction /= 2) {
          const nx = clamp(gx + fraction * corr[0], 0, 6);
          const ny = clamp(gy + fraction * corr[1], 0, 6);
          const nt = clamp(th0 + fraction * corr[2], -45, 45);
          if (objective(nx, ny, nt) < currentLoss) {
            gx = nx;
            gy = ny;
            th0 = nt;
            improved = true;
            break;
          }
        }
        if (!improved) break;
      }
    }

    const weights = points.map(p => {
      if (!useRobust) return 1;
      const e = wrap(observed(p) - (bearing(p.x, p.y, [gx, gy]) + th0));
      return huberWeight(e, finalDelta);
    });
    return { goal: [gx, gy], goalOffsetDeg: th0, weights };
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
    if (!poly || !poly.terms || !poly.terms.length) return 0;
    return basis(x, y, poly).reduce((sum, term, index) => sum + term * poly.coefficients[index], 0);
  }

  function toCanonicalList(points, geometry) {
    return fitPoints(points).map(point => {
      const right = point.x >= 3;
      const pivotX = point.x + geometry.turretRightMm / geometry.tileMm;
      return {
        ...point,
        x: right ? 6 - pivotX : pivotX,
        y: point.y + geometry.turretForwardMm / geometry.tileMm,
        angle: right ? -point.angle : point.angle,
        chassisX: right ? 6 - point.x : point.x,
        chassisY: point.y
      };
    });
  }

  function fitFromCanonical(participating, settings, geometry, precomputedGoal = null) {
    const robust = settings.robust !== false;
    const { goal, goalOffsetDeg } = precomputedGoal || fitGoal(participating, { robust });
    const ridge = Math.max(0.00001, Number.isFinite(settings.ridge) ? settings.ridge : defaults.ridge);
    const angleOrder = Number.isFinite(settings.angleOrder) ? Math.round(settings.angleOrder) : defaults.angleOrder;
    const rpmOrder = Number.isFinite(settings.rpmOrder) ? Math.round(settings.rpmOrder) : defaults.rpmOrder;
    const terms = polynomialTerms(angleOrder);
    const angular = { terms, x: normalization(participating, "x"), y: normalization(participating, "y"), coefficients: [] };
    if (terms.length) {
      const fitA = fitHuberLinear(
        participating,
        point => basis(point.x, point.y, angular),
        point => wrap(point.angle - (bearing(point.x, point.y, goal) + goalOffsetDeg)),
        terms.map(([p, q]) => p + q === 0 ? 1e-9 : ridge * (p + q) ** 2),
        1.8,
        robust
      );
      angular.coefficients = fitA.coefficients || terms.map(() => 0);
    }

    const radial = participating.map(point => {
      const r = Math.hypot(point.x - goal[0], point.y - goal[1]);
      const beta = Math.atan2(point.x - goal[0], goal[1] - point.y);
      return { ...point, r, cb: Math.cos(beta), sb: Math.sin(beta) };
    });
    const radius = normalization(radial, "r");
    const useDir = settings.directionalRpm !== false && participating.length >= 8;
    const speedPowers = Array.from({ length: rpmOrder + 1 }, (_, power) => power);
    const speedFeatures = point => {
      const d = normalized(point.r, radius);
      const base = speedPowers.map(power => d ** power);
      return useDir ? base.concat([point.cb, point.sb]) : base;
    };
    const dirPen = Math.max(1.0, ridge * 4.0);
    const speedPenalty = speedPowers.map(power => power === 0 ? 1e-9 : ridge * power ** 2)
      .concat(useDir ? [dirPen, dirPen] : []);
    const rpmMedian = median(radial.map(p => p.rpm));
    const fitR = fitHuberLinear(radial, speedFeatures, point => point.rpm, speedPenalty, 35.0, robust, rpmMedian);
    const rawCoeffs = fitR.coefficients || speedPowers.map(() => 0).concat(useDir ? [0, 0] : []);
    const speed = {
      radius,
      coefficients: rawCoeffs.slice(0, rpmOrder + 1),
      directional: useDir ? rawCoeffs.slice(rpmOrder + 1, rpmOrder + 3) : [0, 0]
    };

    return {
      goal,
      goalOffsetDeg,
      angular,
      speed,
      angleTrim: Number.isFinite(settings.angleTrim) ? settings.angleTrim : 0,
      rpmTrim: Number.isFinite(settings.rpmTrim) ? settings.rpmTrim : 0,
      geometry,
      hull: hull(participating.map(point => ({ x: point.chassisX, y: point.chassisY }))),
      count: participating.length
    };
  }

  function fit(points, settings = defaults, inputGeometry = defaultGeometry) {
    const merged = { ...defaults, ...settings };
    const geometry = checkedGeometry(inputGeometry);
    const participating = toCanonicalList(points, geometry);
    if (participating.length === 0) return null;
    return fitFromCanonical(participating, merged, geometry);
  }

  function predict(model, x, y) {
    if (!model) return null;
    const sign = x < 3 ? 1 : -1;
    const pivotX = x + model.geometry.turretRightMm / model.geometry.tileMm;
    const pivotY = y + model.geometry.turretForwardMm / model.geometry.tileMm;
    const localX = sign === 1 ? pivotX : 6 - pivotX;
    const offsetDeg = Number.isFinite(model.goalOffsetDeg) ? model.goalOffsetDeg : 0;
    const angle = bearing(localX, pivotY, model.goal) + offsetDeg + evaluate(model.angular, localX, pivotY);
    const radius = Math.hypot(localX - model.goal[0], pivotY - model.goal[1]);
    const r = normalized(radius, model.speed.radius);
    const polyRpm = model.speed.coefficients.reduceRight((sum, term) => sum * r + term, 0);
    const dir = model.speed.directional;
    let dirRpm = 0;
    if (Array.isArray(dir) && dir.length === 2) {
      const beta = Math.atan2(localX - model.goal[0], model.goal[1] - pivotY);
      dirRpm = dir[0] * Math.cos(beta) + dir[1] * Math.sin(beta);
    }
    const rpm = polyRpm + dirRpm + (model.rpmTrim || 0);
    return { angle: wrap(sign * angle + (model.angleTrim || 0)), rpm: Math.max(0, Math.min(20000, rpm)) };
  }

  function error(point, model, settings) {
    if (point.angle === null || !model) return null;
    const fitted = predict(model, point.x, point.y, settings);
    return { angle: wrap(point.angle - fitted.angle), rpm: point.rpm - fitted.rpm };
  }

  function diagnostics(points, model, settings = defaults) {
    const merged = { ...defaults, ...settings };
    const measured = fitPoints(points);
    if (!model || !measured.length) return null;
    const weight = measured.reduce((sum, point) => sum + point.weight, 0);
    const rmse = values => ({
      angle: Math.sqrt(values.reduce((sum, value, i) => sum + measured[i].weight * value.angle ** 2, 0) / weight),
      rpm: Math.sqrt(values.reduce((sum, value, i) => sum + measured[i].weight * value.rpm ** 2, 0) / weight)
    });
    const medAbs = values => ({
      angle: median(values.map(v => Math.abs(v.angle))),
      rpm: median(values.map(v => Math.abs(v.rpm)))
    });
    const insideErrs = measured.map(point => error(point, model, merged));
    const inside = rmse(insideErrs);
    const insideMedian = medAbs(insideErrs);
    let validation = null;
    let validationMedian = null;
    if (measured.length >= 4) {
      const looErrs = measured.map((point, index) =>
        error(point, fit(measured.filter((_, other) => other !== index), merged, model.geometry), merged));
      validation = rmse(looErrs);
      validationMedian = medAbs(looErrs);
    }
    return { inside, validation, insideMedian, validationMedian };
  }

  function autoTune(points, baseSettings = defaults, inputGeometry = defaultGeometry) {
    const geometry = checkedGeometry(inputGeometry);
    const measured = fitPoints(points);
    if (measured.length < 4) return null;
    const merged = { ...defaults, ...baseSettings };
    const robust = merged.robust !== false;
    const canonical = toCanonicalList(measured, geometry);
    const n = canonical.length;
    const folds = canonical.map((heldOut, idx) => {
      const train = canonical.filter((_, j) => j !== idx);
      const goalFit = fitGoal(train, { robust });
      return { raw: measured[idx], train, goalFit };
    });

    const totalWeight = measured.reduce((s, p) => s + p.weight, 0);
    const angleOrders = n >= 16 ? [-1, 0, 1, 2] : [-1, 0, 1];
    const rpmOrders = n >= 14 ? [1, 2, 3] : n >= 6 ? [1, 2] : [0, 1];
    const ridges = [0.06, 0.18, 0.5, 1.5];
    const dirOptions = n >= 8 ? [true, false] : [false];

    const angleLoo = new Map();
    for (const angleOrder of angleOrders) {
      for (const ridge of ridges) {
        let sq = 0;
        for (const fold of folds) {
          const m = fitFromCanonical(fold.train, { ...merged, angleOrder, rpmOrder: 0, ridge, directionalRpm: false },
            geometry, fold.goalFit);
          const pred = predict(m, fold.raw.x, fold.raw.y);
          const e = wrap(fold.raw.angle - pred.angle);
          sq += fold.raw.weight * e * e;
        }
        angleLoo.set(angleOrder + "|" + ridge, Math.sqrt(sq / totalWeight));
      }
    }

    const rpmLoo = new Map();
    for (const rpmOrder of rpmOrders) {
      for (const ridge of ridges) {
        for (const directionalRpm of dirOptions) {
          let sq = 0;
          for (const fold of folds) {
            const m = fitFromCanonical(fold.train, { ...merged, angleOrder: -1, rpmOrder, ridge, directionalRpm },
              geometry, fold.goalFit);
            const pred = predict(m, fold.raw.x, fold.raw.y);
            const e = fold.raw.rpm - pred.rpm;
            sq += fold.raw.weight * e * e;
          }
          rpmLoo.set(rpmOrder + "|" + ridge + "|" + directionalRpm, Math.sqrt(sq / totalWeight));
        }
      }
    }

    let best = null;
    for (const ridge of ridges) {
      for (const angleOrder of angleOrders) {
        const errA = angleLoo.get(angleOrder + "|" + ridge);
        for (const rpmOrder of rpmOrders) {
          for (const directionalRpm of dirOptions) {
            const errR = rpmLoo.get(rpmOrder + "|" + ridge + "|" + directionalRpm);
            const complexity = (angleOrder + 1) * 0.002 + rpmOrder * 0.002 + (directionalRpm ? 0.001 : 0);
            const score = (errA / 1.5) ** 2 + (errR / 45) ** 2 + complexity;
            if (!best || score < best.score) {
              best = {
                score,
                angleOrder,
                rpmOrder,
                ridge,
                directionalRpm,
                errA,
                errR
              };
            }
          }
        }
      }
    }

    const tunedSettings = {
      ...merged,
      angleOrder: best.angleOrder,
      rpmOrder: best.rpmOrder,
      ridge: best.ridge,
      directionalRpm: best.directionalRpm
    };
    const tunedModel = fit(points, tunedSettings, geometry);
    const tunedStats = diagnostics(points, tunedModel, tunedSettings);
    return { settings: tunedSettings, stats: tunedStats, score: best.score };
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
    const mergedSettings = { ...defaults, ...settings };
    const strategy = options.strategy || "bayesian-map";
    const angleMode = options.angleMode || "auto";
    const rpmOrder = Number.isFinite(options.rpmOrder) ? Math.max(0, Math.min(2, Math.round(options.rpmOrder))) : 2;
    const usableBase = (baselinePoints || []).filter(usable);
    const usableAnchors = (anchorPoints || []).filter(usable);
    if (!usableAnchors.length || !usableBase.length) return null;

    const geometry = checkedGeometry(inputGeometry);
    const baseModel = fit(baselinePoints, mergedSettings, geometry);
    const anchorById = new Map(usableAnchors.map(point => [point.id, point]));
    const baseById = new Map(usableBase.map(point => [point.id, point]));
    const nonAnchorBase = usableBase.filter(point => !anchorById.has(point.id));
    const nonAnchorModel = nonAnchorBase.length >= 3 ? fit(nonAnchorBase, mergedSettings, geometry) : null;

    const matchedCount = usableAnchors.filter(a => baseById.has(a.id)).length;
    const allMatchedIdentical = matchedCount > 0 && usableAnchors.every(a => {
      const b = baseById.get(a.id);
      return !b || (Math.abs(wrap(a.angle - b.angle)) < 1e-6 && Math.abs(a.rpm - b.rpm) < 1e-6);
    });
    const useCornerModelAsBaseline = allMatchedIdentical && Boolean(nonAnchorModel);
    const refModel = useCornerModelAsBaseline ? nonAnchorModel : (baseModel || fit(usableAnchors, mergedSettings, geometry));
    if (!refModel) return null;

    const goal = refModel.goal;
    const baseOffset0 = Number.isFinite(refModel.goalOffsetDeg) ? refModel.goalOffsetDeg : 0;

    // Anchor shifts are ALWAYS computed relative to the smooth refModel surface (never raw noisy single points).
    const anchorData = usableAnchors.map(a => {
      const ca = canonicalPivot(a, geometry, goal);
      const pred = predict(refModel, a.x, a.y);
      const oldPoly = evaluate(refModel.angular, ca.localX, ca.pivotY);
      const refLocalAngle = ca.sign * wrap(pred.angle - (refModel.angleTrim || 0));
      const refRpm = pred.rpm - (refModel.rpmTrim || 0);
      const dAngle = wrap(ca.localAngle - refLocalAngle);
      const dRpm = a.rpm - refRpm;
      return {
        anchor: a,
        ...ca,
        x: ca.localX,
        y: ca.pivotY,
        oldPoly,
        targetGeomAngle: wrap(ca.localAngle - oldPoly),
        refLocalAngle,
        refRpm,
        dAngle,
        dRpm,
        weight: a.weight > 0 ? a.weight : 1
      };
    });

    const rNorm = normalization(anchorData, "r");
    const rValues = anchorData.map(d => d.r);
    const rMin = Math.min(...rValues), rMax = Math.max(...rValues), rSpan = rMax - rMin;
    const effRpmOrder = anchorData.length <= 1 || rSpan < 0.35 ? 0
      : anchorData.length <= 3 || rSpan < 1.1 ? Math.min(1, rpmOrder)
      : Math.min(2, rpmOrder, anchorData.length - 1);
    const rpmPowers = Array.from({ length: effRpmOrder + 1 }, (_, i) => i);
    const rpmPenalties = [1e-9, 0.40, 1.60].slice(0, effRpmOrder + 1);
    const medianRpmShift = median(anchorData.map(d => d.dRpm));
    const rpmFit = fitHuberLinear(
      anchorData,
      d => rpmPowers.map(p => normalized(d.r, rNorm) ** p),
      d => d.dRpm,
      rpmPenalties,
      30.0,
      true,
      medianRpmShift
    );
    const rpmCoeffs = rpmFit.coefficients || [medianRpmShift];
    const rpmWeights = rpmFit.weights;
    const evalDeltaRpm = r => rpmCoeffs.reduceRight((sum, c) => sum * normalized(r, rNorm) + c, 0);

    let effAngleOrder, angleCoeffs, evalDeltaAngle, angleWeights;
    let newGoal = [...goal], newGoalOffsetDeg = baseOffset0;
    if (angleMode === "keep") {
      effAngleOrder = -1;
      angleCoeffs = [0];
      evalDeltaAngle = () => 0;
      angleWeights = anchorData.map(() => 1);
    } else if (angleMode === "offset" || anchorData.length <= 1) {
      effAngleOrder = 0;
      const medianAngleShift = median(anchorData.map(d => d.dAngle));
      const angFit = fitHuberLinear(anchorData, () => [1], d => d.dAngle, [1e-9], 1.5, true, medianAngleShift);
      const theta0 = angFit.coefficients ? angFit.coefficients[0] : medianAngleShift;
      angleCoeffs = [theta0];
      angleWeights = angFit.weights;
      newGoalOffsetDeg = wrap(baseOffset0 + theta0);
      evalDeltaAngle = () => theta0;
    } else {
      effAngleOrder = 2; // 2D Bayesian MAP geometry (gx, gy, goalOffsetDeg)
      const priorGoal = anchorData.length <= 4 ? 600.0 : 350.0;
      const mapGoal = fitGoal(anchorData, {
        priorX: goal[0],
        priorY: goal[1],
        priorTheta: baseOffset0,
        seedGoal: goal,
        goalPenalty: priorGoal,
        thetaPenalty: 0.01,
        huberFloor: 1.5,
        robust: true,
        observed: d => d.targetGeomAngle
      });
      newGoal = mapGoal.goal;
      newGoalOffsetDeg = mapGoal.goalOffsetDeg;
      angleWeights = mapGoal.weights;
      const dTh0 = wrap(newGoalOffsetDeg - baseOffset0);
      angleCoeffs = [dTh0, newGoal[0] - goal[0], newGoal[1] - goal[1]];
      evalDeltaAngle = (lx, py) => wrap(
        (bearing(lx, py, newGoal) + newGoalOffsetDeg) - (bearing(lx, py, goal) + baseOffset0)
      );
    }

    const totalAWeight = anchorData.reduce((s, d, i) => s + d.weight * angleWeights[i], 0) || 1;
    const totalRWeight = anchorData.reduce((s, d, i) => s + d.weight * rpmWeights[i], 0) || 1;
    const meanDeltaAngle = anchorData.reduce((s, d, i) => s + d.weight * angleWeights[i] * d.dAngle, 0) / totalAWeight;
    const meanDeltaRpm = anchorData.reduce((s, d, i) => s + d.weight * rpmWeights[i] * d.dRpm, 0) / totalRWeight;

    const anchorDiagnostics = anchorData.map((d, i) => ({
      id: d.anchor.id,
      dAngle: d.dAngle,
      dRpm: d.dRpm,
      angleWeight: angleWeights[i],
      rpmWeight: rpmWeights[i],
      outlier: angleWeights[i] < 0.7 || rpmWeights[i] < 0.7
    }));
    const anchorDiagById = new Map(anchorDiagnostics.map(item => [item.id, item]));
    const outlierIds = anchorDiagnostics.filter(d => d.outlier).map(d => d.id);

    const anchorModel = strategy === "residual-stack"
      ? fit(usableAnchors, {
        ...mergedSettings,
        angleOrder: Math.min(1, Math.max(-1, mergedSettings.angleOrder)),
        rpmOrder: Math.min(2, mergedSettings.rpmOrder)
      }, geometry)
      : null;

    const anchorDataById = new Map(anchorData.map(d => [d.anchor.id, d]));
    const existingIds = new Set(baselinePoints.map(p => p.id));
    const comparisons = [];
    const calibratedPoints = baselinePoints.map(p => {
      if (p.angle === null || p.blocked || p.inactive) return { ...p };
      const cp = canonicalPivot(p, geometry, goal);
      const predBase = predict(refModel, p.x, p.y);
      const oldSurfLocal = cp.sign * wrap(predBase.angle - (refModel.angleTrim || 0));
      const oldSurfRpm = predBase.rpm - (refModel.rpmTrim || 0);
      const oldResAngle = wrap(cp.localAngle - oldSurfLocal);
      const oldResRpm = p.rpm - oldSurfRpm;
      const dAng = evalDeltaAngle(cp.localX, cp.pivotY);
      const dSpd = evalDeltaRpm(cp.r);
      const newSurfLocal = oldSurfLocal + dAng;
      const newSurfRpm = oldSurfRpm + dSpd;
      const direct = anchorDataById.get(p.id);
      const sameHalf = direct && (direct.anchor.x >= 3) === (p.x >= 3);

      let newAngle, newRpm, source;
      if (strategy === "bayesian-map") {
        if (sameHalf) {
          const eR = direct.anchor.rpm - newSurfRpm;
          const kR = 0.45 * Math.min(1, 35 / Math.max(1e-6, Math.abs(eR)));
          newRpm = clamp(Math.round(newSurfRpm + kR * eR), 1, 20000);
          if (angleMode === "keep") {
            newAngle = p.angle;
          } else if (angleMode === "offset") {
            newAngle = Number(wrap(cp.sign * (oldSurfLocal + dAng + 0.35 * clamp(oldResAngle, -2, 2))).toFixed(2));
          } else {
            const eA = wrap(direct.localAngle - newSurfLocal);
            const kA = 0.45 * Math.min(1, 1.5 / Math.max(1e-6, Math.abs(eA)));
            newAngle = Number(wrap(cp.sign * (newSurfLocal + kA * eA)).toFixed(2));
          }
          const diag = anchorDiagById.get(p.id);
          source = diag?.outlier ? "锚点抗噪收缩" : "锚点贝叶斯融合";
        } else {
          newRpm = clamp(Math.round(newSurfRpm + 0.35 * clamp(oldResRpm, -45, 45)), 1, 20000);
          newAngle = angleMode === "keep"
            ? p.angle
            : Number(wrap(cp.sign * (newSurfLocal + 0.35 * clamp(oldResAngle, -2, 2))).toFixed(2));
          source = "贝叶斯MAP迁移";
        }
      } else if (sameHalf && strategy !== "global-curve") {
        newRpm = clamp(Math.round(direct.anchor.rpm), 1, 20000);
        newAngle = angleMode === "keep" ? p.angle
          : angleMode === "offset" ? Number(wrap(p.angle + cp.sign * dAng).toFixed(2))
          : Number(direct.anchor.angle.toFixed(2));
        source = "新锚点直写";
      } else if (strategy === "residual-stack" && anchorModel && refModel) {
        const oldPred = predict(refModel, p.x, p.y), newPred = predict(anchorModel, p.x, p.y);
        const resAngle = wrap(p.angle - oldPred.angle), resRpm = p.rpm - oldPred.rpm;
        newRpm = clamp(Math.round(newPred.rpm + resRpm), 1, 20000);
        newAngle = angleMode === "keep" ? p.angle
          : angleMode === "offset" ? Number(wrap(p.angle + cp.sign * dAng).toFixed(2))
          : Number(wrap(newPred.angle + resAngle).toFixed(2));
        source = "新曲线+旧残差";
      } else if (strategy === "local-anchor") {
        const centerId = p.id.slice(0, 2) + "0";
        const cellAnchor = anchorDataById.get(centerId);
        if (cellAnchor && cellAnchor.right === cp.right) {
          const cellDiag = anchorDiagById.get(centerId);
          const localResRpm = clamp(cellAnchor.dRpm - evalDeltaRpm(cellAnchor.r), -50, 50);
          const localResAng = clamp(
            wrap(cellAnchor.dAngle - evalDeltaAngle(cellAnchor.localX, cellAnchor.pivotY)), -2, 2);
          const kR = 0.45 * (cellDiag ? cellDiag.rpmWeight : 1);
          const kA = 0.45 * (cellDiag ? cellDiag.angleWeight : 1);
          newRpm = clamp(Math.round(p.rpm + dSpd + kR * localResRpm), 1, 20000);
          const totalDeltaA = angleMode === "keep" ? 0 : angleMode === "offset" ? dAng : dAng + kA * localResAng;
          newAngle = Number(wrap(p.angle + cp.sign * totalDeltaA).toFixed(2));
          source = "同格抗噪锚定+径向";
        } else {
          newRpm = clamp(Math.round(p.rpm + dSpd), 1, 20000);
          newAngle = Number(wrap(p.angle + cp.sign * dAng).toFixed(2));
          source = "径向曲线迁移";
        }
      } else {
        newRpm = clamp(Math.round(p.rpm + dSpd), 1, 20000);
        newAngle = Number(wrap(p.angle + cp.sign * dAng).toFixed(2));
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
      let newAngle, newRpm, source;
      if (strategy === "bayesian-map") {
        const ca = canonicalPivot(a, geometry, goal);
        const predBase = predict(refModel, a.x, a.y);
        const oldSurfLocal = ca.sign * wrap(predBase.angle - (refModel.angleTrim || 0));
        const oldSurfRpm = predBase.rpm - (refModel.rpmTrim || 0);
        const newSurfLocal = oldSurfLocal + evalDeltaAngle(ca.localX, ca.pivotY);
        const newSurfRpm = oldSurfRpm + evalDeltaRpm(ca.r);
        const eR = a.rpm - newSurfRpm;
        const kR = 0.45 * Math.min(1, 35 / Math.max(1e-6, Math.abs(eR)));
        newRpm = clamp(Math.round(newSurfRpm + kR * eR), 1, 20000);
        const eA = wrap(ca.localAngle - newSurfLocal);
        const kA = 0.45 * Math.min(1, 1.5 / Math.max(1e-6, Math.abs(eA)));
        newAngle = angleMode === "offset" ? Number(wrap(ca.sign * newSurfLocal).toFixed(2))
          : Number(wrap(ca.sign * (newSurfLocal + kA * eA)).toFixed(2));
        source = "新增锚点(贝叶斯)";
      } else {
        newAngle = Number(a.angle.toFixed(2));
        newRpm = clamp(Math.round(a.rpm), 1, 20000);
        source = "新增锚点";
      }
      calibratedPoints.push({ ...a, angle: newAngle, rpm: newRpm });
      comparisons.push({
        id: a.id, name: pointName(a.id), x: a.x, y: a.y,
        oldAngle: null, oldRpm: null, newAngle, newRpm,
        dAngle: null, dRpm: null, source
      });
    }
    calibratedPoints.sort((a, b) => a.id.localeCompare(b.id));

    return {
      points: calibratedPoints,
      comparisons,
      anchorDiagnostics,
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
        useCornerModelAsBaseline,
        oldGoal: goal,
        newGoal,
        goalShift: [newGoal[0] - goal[0], newGoal[1] - goal[1]],
        deltaGoalOffsetDeg: wrap(newGoalOffsetDeg - baseOffset0),
        outlierIds
      }
    };
  }

  const api = {
    defaults,
    defaultGeometry,
    checkedGeometry,
    parse,
    validPoint,
    usable,
    pointName,
    wrap,
    fit,
    predict,
    error,
    diagnostics,
    autoTune,
    supported,
    recalibrate
  };
  globalThis.ShotFit = api;
  if (typeof module === "object" && module.exports) module.exports = api;
})();
