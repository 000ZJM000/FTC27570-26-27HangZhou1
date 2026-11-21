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

  const api = { defaults, defaultGeometry, checkedGeometry, parse, validPoint, usable, pointName, wrap, fit, predict, error, diagnostics, supported };
  globalThis.ShotFit = api;
  if (typeof module === "object" && module.exports) module.exports = api;
})();
