(function () {
  "use strict";

  const TILE = 609.6;
  const RAD = Math.PI / 180;
  const powers = [[0, 0], [1, 0], [0, 1], [2, 0], [1, 1], [0, 2], [3, 0], [2, 1], [1, 2], [0, 3]];
  const defaults = { angleOrder: 1, rpmOrder: 2, ridge: 0.18, angleTrim: 0, rpmTrim: 0 };
  const codeNames = ["格心", "左后", "右后", "右前", "左前"];

  function wrap(angle) {
    return ((angle + 180) % 360 + 360) % 360 - 180;
  }

  function pointName(id) {
    return codeNames[Number(id[2])] || "旧精测 " + id[2];
  }

  function parse(text) {
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
      const halfWidth = 219.3 / TILE;
      const rear = 177.47468 / TILE;
      const front = 1 - (440.47468 - 177.47468) / TILE;
      const positions = [[0.5, 0.5], [halfWidth, rear], [1 - halfWidth, rear],
        [1 - halfWidth, front], [halfWidth, front]];
      if (right === undefined && !positions[code]) {
        errors.push(word);
        continue;
      }
      const x = right === undefined ? Number(column) - 1 + positions[code][0] : Number(right);
      const y = forward === undefined ? Number(row) - 1 + positions[code][1] : Number(forward);
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

  function canonical(point) {
    return point.x < 3 ? point : { ...point, x: 6 - point.x, angle: -point.angle };
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

  function basis(x, y, terms) {
    const u = x - 0.5, v = (y - 3) / 3;
    return terms.map(([p, q]) => u ** p * v ** q);
  }

  function fitPolynomial(points, order, target, ridge) {
    const terms = polynomialTerms(order);
    const penalty = terms.map(([p, q]) => p + q === 0 ? 1e-9 : ridge * (p + q) ** 2);
    const coefficients = leastSquares(points, point => basis(point.x, point.y, terms), target, penalty);
    return { terms, coefficients };
  }

  function evaluate(poly, x, y) {
    return basis(x, y, poly.terms).reduce((sum, term, index) => sum + term * poly.coefficients[index], 0);
  }

  function fit(points, settings = defaults) {
    const participating = points.filter(usable).map(canonical);
    if (participating.length === 0) return null;
    const goal = fitGoal(participating);
    const ridge = Math.max(0.00001, settings.ridge);
    const angular = settings.angleOrder < 0 ? { terms: [], coefficients: [] }
      : fitPolynomial(participating, settings.angleOrder, point =>
        wrap(point.angle - bearing(point.x, point.y, goal)), ridge);
    const speed = fitPolynomial(participating, settings.rpmOrder, point => point.rpm, ridge);
    return { goal, angular, speed, hull: hull(participating), count: participating.length };
  }

  function predict(model, x, y, settings = defaults) {
    if (!model) return null;
    const sign = x < 3 ? 1 : -1;
    const localX = x < 3 ? x : 6 - x;
    const angle = bearing(localX, y, model.goal) + evaluate(model.angular, localX, y) + settings.angleTrim;
    return { angle: wrap(sign * angle), rpm: evaluate(model.speed, localX, y) + settings.rpmTrim };
  }

  function error(point, model, settings) {
    if (point.angle === null || !model) return null;
    const fitted = predict(model, point.x, point.y, settings);
    return { angle: wrap(point.angle - fitted.angle), rpm: point.rpm - fitted.rpm };
  }

  function diagnostics(points, model, settings) {
    const measured = points.filter(usable);
    if (!model || !measured.length) return null;
    const rmse = values => ({
      angle: Math.sqrt(values.reduce((sum, value) => sum + value.angle ** 2, 0) / values.length),
      rpm: Math.sqrt(values.reduce((sum, value) => sum + value.rpm ** 2, 0) / values.length)
    });
    const inside = rmse(measured.map(point => error(point, model, settings)));
    const validation = measured.length < 4 ? null : rmse(measured.map((point, index) =>
      error(point, fit(measured.filter((_, other) => index !== other), settings), settings)));
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

  const api = { defaults, parse, validPoint, usable, pointName, wrap, fit, predict, error, diagnostics, supported };
  globalThis.ShotFit = api;
  if (typeof module === "object" && module.exports) module.exports = api;
})();
