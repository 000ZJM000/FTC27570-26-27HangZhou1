package org.firstinspires.ftc.teamcode.biobuzz.calibration;

import com.qualcomm.robotcore.util.RobotLog;
import org.firstinspires.ftc.teamcode.biobuzz.core.Maths;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Comparator;
import java.util.List;
import java.util.Random;
import java.util.function.BooleanSupplier;

/** Constrained Bayesian search using separate Gaussian processes for score and overshoot margin. */
final class BayesianPidSearch {
    static final int MIN_ROUNDS = 10, MAX_ROUNDS = 16;
    static final int HISTORY_STRIDE = 6, HISTORY_ROWS = 40;
    static final int VERIFICATION_ROUNDS = 2;
    private static final int FINALISTS = 4, RECORD_PATIENCE = 5, WARMUP_OBSERVATIONS = 5;
    private final double[] min, max, scale, safeRecoveryCenter;
    private final Random random = new Random(27270);
    private final List<Observation> data = new ArrayList<>();
    private final List<Review> reviews = new ArrayList<>();
    private int historyCount;

    interface Trial {
        Result test(double[] gains, boolean verification, int pattern) throws InterruptedException;
    }

    static final class Result {
        final double cost, margin, steadyError;
        final boolean settled;
        final String reason;

        Result(double cost, double margin, boolean settled) {
            this(cost, margin, settled, "");
        }

        Result(double cost, double margin, boolean settled, String reason) {
            this(cost, margin, settled, reason, Double.NaN);
        }

        Result(double cost, double margin, boolean settled, String reason, double steadyError) {
            this.cost = cost;
            this.margin = margin;
            this.settled = settled;
            this.reason = reason;
            this.steadyError = steadyError;
        }

        static Result unavailable(String reason) {
            return new Result(Double.NaN, Double.NaN, false, reason);
        }
    }

    static final class Review {
        final double[] gains;
        final int passed;
        final double score, steadyError, measuredScore;
        final String firstFailure;

        Review(double[] gains, int passed, double score, String firstFailure, double steadyError, double measuredScore) {
            this.gains = gains;
            this.passed = passed;
            this.score = score;
            this.firstFailure = firstFailure;
            this.steadyError = steadyError;
            this.measuredScore = measuredScore;
        }

        boolean qualified() { return passed == VERIFICATION_ROUNDS && Double.isFinite(score); }
    }

    BayesianPidSearch(double[] min, double[] max, double[] scale) {
        this(min, max, scale, new double[]{0.22, 0.12, 0.72});
    }

    BayesianPidSearch(double[] min, double[] max, double[] scale, double[] safeRecoveryCenter) {
        this.min = min.clone();
        this.max = max.clone();
        this.scale = scale.clone();
        this.safeRecoveryCenter = safeRecoveryCenter.clone();
    }

    List<Review> reviews() { return new ArrayList<>(reviews); }
    Review incumbent() { return reviews.isEmpty() ? null : reviews.get(0); }
    Review bestCandidate() {
        return reviews.stream().skip(1).filter(Review::qualified)
                .min(Comparator.comparingDouble(row -> row.score)).orElse(null);
    }
    Review bestFallbackCandidate() {
        Review best = null;
        for (int i = 1; i < reviews.size(); i++) {
            Review row = reviews.get(i);
            if (!Double.isFinite(row.measuredScore) || row.measuredScore >= 8.5) continue;
            if (best == null || row.passed > best.passed
                    || (row.passed == best.passed && row.measuredScore < best.measuredScore)) {
                best = row;
            }
        }
        return best;
    }
    int searchCount() { return data.size() - historyCount; }
    int historyCount() { return historyCount; }
    double bestSearchCost() {
        return data.stream().skip(historyCount).mapToDouble(o -> o.result.cost).min().orElse(Double.NaN);
    }
    double bestQualifiedSearchCost() {
        return data.stream().skip(historyCount).filter(o -> o.result.settled && o.result.margin >= 0)
                .mapToDouble(o -> o.result.cost).min().orElse(Double.NaN);
    }

    void tune(double[] existing, Trial trial, BooleanSupplier cancelled, double[] history)
            throws InterruptedException {
        loadHistory(history);
        double[] baseline = normalize(existing), center = baseline;
        double radius = 0.26, best = Double.POSITIVE_INFINITY, record = Double.POSITIVE_INFINITY;
        for (Observation row : data) {
            if (row.result.settled && row.result.margin >= 0 && row.result.cost < best) {
                best = row.result.cost;
                center = row.x;
            }
        }
        int stagnant = 0, lastRecord = 0, gpRounds = 0;
        boolean resample = false;
        for (int round = 0, attempts = 0; round < MAX_ROUNDS && attempts < MAX_ROUNDS + 12
                && !cancelled.getAsBoolean(); attempts++) {
            boolean trySafeSeed = attempts == 1 && !data.isEmpty()
                    && (!data.get(data.size() - 1).result.settled || data.get(data.size() - 1).result.margin < 0)
                    && distance(safeRecoveryCenter, baseline) >= 0.04;
            boolean usedGp = attempts > 0 && !trySafeSeed && !resample && data.size() >= WARMUP_OBSERVATIONS
                    && data.stream().anyMatch(o -> o.result.margin >= 0);
            double[] x = attempts == 0 ? baseline : trySafeSeed ? safeRecoveryCenter.clone()
                    : resample ? sampleNovel(center, radius)
                    : propose(center, radius, round, cancelled);
            if (cancelled.getAsBoolean()) return;
            double[] testedGains = gains(x);
            Result score = trial.test(testedGains, false, attempts % 6);
            RobotLog.ii("BioBuzzCalibration",
                    "PID search round %d (attempt %d, gp=%b): P=%.3f I=%.3f D=%.3f cost=%.3f margin=%+.3f settled=%b reason=%s",
                    round + 1, attempts + 1, usedGp, testedGains[0], testedGains[1], testedGains[2],
                    score == null ? Double.NaN : score.cost, score == null ? Double.NaN : score.margin,
                    score != null && score.settled, score == null ? "cancelled" : score.reason);
            resample = !measured(score);
            if (resample) continue;
            data.add(new Observation(x, score));
            round++;
            if (usedGp) gpRounds++;
            if (score.settled && score.margin >= 0 && score.cost < record) {
                record = score.cost;
                lastRecord = round;
            }
            if (score.settled && score.margin >= 0 && score.cost < best - 0.015) {
                best = score.cost;
                center = x;
                stagnant = 0;
                if (round >= WARMUP_OBSERVATIONS + 2) radius = Math.min(radius, 0.20);
            } else if (++stagnant >= 2) {
                radius = Math.max(0.06, radius * 0.72);
                stagnant = 0;
            }
            if (round >= MIN_ROUNDS && gpRounds >= 4 && Double.isFinite(record)
                    && (round - lastRecord >= 4 || radius <= 0.08)) {
                break;
            }
        }
        if (cancelled.getAsBoolean() || searchCount() < MIN_ROUNDS) return;

        List<double[]> finalists = new ArrayList<>();
        finalists.add(baseline);
        List<Observation> rankedCurrent = new ArrayList<>(data.subList(historyCount, data.size()));
        rankedCurrent.sort(Comparator.comparingDouble(o -> o.result.cost));
        for (Observation o : rankedCurrent) {
            if (!o.result.settled || o.result.margin < 0) continue;
            if (isDistinct(finalists, o.x)) finalists.add(o.x);
            if (finalists.size() == FINALISTS) break;
        }
        if (finalists.size() < FINALISTS && historyCount > 0) {
            List<Observation> rankedHistory = new ArrayList<>(data.subList(0, historyCount));
            rankedHistory.sort(Comparator.comparingDouble(o -> o.result.cost));
            for (Observation o : rankedHistory) {
                if (!o.result.settled || o.result.margin < 0) continue;
                if (isDistinct(finalists, o.x)) finalists.add(o.x);
                if (finalists.size() == FINALISTS) break;
            }
        }
        if (finalists.size() < FINALISTS && data.size() >= WARMUP_OBSERVATIONS) {
            double[] gpOptimum = proposeFinalist(finalists, cancelled);
            if (gpOptimum != null && isDistinct(finalists, gpOptimum)) finalists.add(gpOptimum);
        }
        if (finalists.size() == 1 && !rankedCurrent.isEmpty()) {
            Observation safest = null;
            for (Observation o : rankedCurrent) {
                if (!isDistinct(finalists, o.x)) continue;
                if (safest == null || o.result.margin > safest.result.margin + 1e-6
                        || (Math.abs(o.result.margin - safest.result.margin) <= 1e-6
                                && o.result.cost < safest.result.cost)) {
                    safest = o;
                }
            }
            if (safest != null) finalists.add(safest.x);
        }
        double[][] costs = new double[finalists.size()][VERIFICATION_ROUNDS];
        double[] steadyErrors = new double[finalists.size()];
        Arrays.fill(steadyErrors, Double.NaN);
        double[] measuredSum = new double[finalists.size()];
        int[] measuredCount = new int[finalists.size()];
        int[] passed = new int[finalists.size()];
        String[] firstFailure = new String[finalists.size()];
        for (int repeat = 0; repeat < VERIFICATION_ROUNDS; repeat++) {
            for (int i = 0; i < finalists.size(); i++) {
                if (cancelled.getAsBoolean()) return;
                int j = (i + repeat) % finalists.size();
                double[] candidate = j == 0 ? existing.clone() : gains(finalists.get(j));
                Result result = trial.test(candidate, true, repeat);
                if (!measured(result) && !cancelled.getAsBoolean()) {
                    Result retry = trial.test(candidate, true, repeat);
                    if (retry != null) result = retry;
                }
                if (measured(result)) {
                    measuredSum[j] += result.cost;
                    measuredCount[j]++;
                    if (Double.isFinite(result.steadyError)) {
                        steadyErrors[j] = Double.isFinite(steadyErrors[j])
                                ? Math.max(steadyErrors[j], result.steadyError) : result.steadyError;
                    }
                }
                boolean pass = measured(result) && result.settled && result.margin >= 0;
                costs[j][repeat] = pass ? result.cost : Double.POSITIVE_INFINITY;
                if (pass) {
                    passed[j]++;
                } else if (firstFailure[j] == null) {
                    String reason = result != null && !result.reason.isEmpty() ? result.reason
                            : !measured(result) ? "无有效读数"
                            : result.margin < 0 ? "超调超限" : "未停稳";
                    firstFailure[j] = "第 " + (repeat + 1) + " 组 " + reason;
                }
                RobotLog.ii("BioBuzzCalibration",
                        "PID verify finalist %d round %d: P=%.3f I=%.3f D=%.3f pass=%b cost=%.3f margin=%+.3f steady=%.4f reason=%s",
                        j, repeat + 1, candidate[0], candidate[1], candidate[2], pass,
                        result == null ? Double.NaN : result.cost,
                        result == null ? Double.NaN : result.margin,
                        result == null ? Double.NaN : result.steadyError,
                        firstFailure[j] != null ? firstFailure[j] : "ok");
            }
        }
        for (int i = 0; i < costs.length; i++) {
            double value = passed[i] == VERIFICATION_ROUNDS ? validationScore(costs[i]) : Double.POSITIVE_INFINITY;
            double measuredScore = measuredCount[i] > 0 ? measuredSum[i] / measuredCount[i] : Double.POSITIVE_INFINITY;
            reviews.add(new Review(i == 0 ? existing.clone() : gains(finalists.get(i)), passed[i], value,
                    firstFailure[i], steadyErrors[i], measuredScore));
        }
    }

    private void loadHistory(double[] history) {
        for (int i = 0; i + HISTORY_STRIDE <= history.length; i += HISTORY_STRIDE) {
            double[] old = {history[i], history[i + 1], history[i + 2]};
            boolean inBounds = true;
            for (int axis = 0; axis < 3; axis++) inBounds &= old[axis] >= min[axis] && old[axis] <= max[axis];
            Result remembered = new Result(history[i + 3], history[i + 4], history[i + 5] == 1);
            if (inBounds && measured(remembered)) data.add(new Observation(normalize(old), remembered));
        }
        historyCount = data.size();
    }

    double[] history() {
        int rows = Math.min(HISTORY_ROWS, data.size());
        double[] result = new double[rows * HISTORY_STRIDE];
        for (int i = 0; i < rows; i++) {
            Observation row = data.get(data.size() - rows + i);
            int j = i * HISTORY_STRIDE;
            System.arraycopy(gains(row.x), 0, result, j, 3);
            result[j + 3] = row.result.cost;
            result[j + 4] = row.result.margin;
            result[j + 5] = row.result.settled ? 1 : 0;
        }
        return result;
    }

    private static boolean measured(Result result) {
        return result != null && Maths.finite(result.cost, result.margin);
    }

    private double[] propose(double[] center, double radius, int round, BooleanSupplier cancelled) {
        if (data.size() < WARMUP_OBSERVATIONS) {
            double[] x = center.clone();
            int step = Math.max(0, data.size() - 1);
            if (step == 0) x[0] = Maths.clip(x[0] + 0.16, 0, 1);
            else if (step == 1) x[1] = Maths.clip(x[1] + 0.16, 0, 1);
            else if (step == 2) x[2] = Maths.clip(x[2] + 0.18, 0, 1);
            else {
                x[0] = Maths.clip(x[0] - 0.16, 0, 1);
                x[2] = Maths.clip(x[2] + 0.16, 0, 1);
            }
            if (distance(x, center) > 0.03 && nearestDistance(x) >= 0.028) return x;
            return sampleNovel(center, 0.22);
        }
        if (data.stream().noneMatch(o -> o.result.margin >= 0)) {
            return sampleNovel(safeRecoveryCenter, 0.18);
        }
        double progress = Maths.clip((round - WARMUP_OBSERVATIONS)
                / (double) Math.max(1, MAX_ROUNDS - WARMUP_OBSERVATIONS), 0, 1);
        boolean earlyExplore = round == WARMUP_OBSERVATIONS + 1;
        double beta = earlyExplore ? 1.15 : (0.65 * (1.0 - progress) + 0.05 * progress);
        double safetyBeta = earlyExplore ? 0.45 : 0.70;
        double minSep = progress >= 0.5 ? 0.015 : 0.024;
        GaussianProcess objective = GaussianProcess.fit(data, false);
        GaussianProcess safety = GaussianProcess.fit(data, true);
        double[] winner = null;
        double bestAcquisition = Double.POSITIVE_INFINITY;
        double[] fallback = null;
        double bestFallback = Double.NEGATIVE_INFINITY;
        double[] work = new double[data.size()];
        for (int i = 0; i < 4096; i++) {
            if (i % 64 == 0 && cancelled.getAsBoolean()) break;
            double[] x = sample(center, i < 2048 ? Math.max(0.04, 0.45 * radius) : radius);
            double nearest = Double.POSITIVE_INFINITY;
            boolean nearViolation = false;
            for (Observation o : data) {
                double separation = distance(x, o.x);
                nearest = Math.min(nearest, separation);
                nearViolation |= o.result.margin < -0.05 && separation < 0.06;
            }
            if (nearest < minSep) continue;
            double[] limit = safety.predict(x, work);
            double[] cost = objective.predict(x, work);
            if (nearest >= Math.max(0.02, minSep)) {
                double feasibility = limit[0] + 0.5 * limit[1] - 0.15 * cost[0];
                if (feasibility > bestFallback) {
                    bestFallback = feasibility;
                    fallback = x;
                }
            }
            if (nearViolation) continue;
            if (limit[0] - safetyBeta * limit[1] < 0) continue;
            double acquisition = cost[0] - beta * cost[1];
            if (acquisition < bestAcquisition) {
                bestAcquisition = acquisition;
                winner = x;
            }
        }
        if (winner != null) return winner;
        return fallback != null ? fallback : sampleNovel(safeRecoveryCenter, 0.25);
    }

    private double[] proposeFinalist(List<double[]> finalists, BooleanSupplier cancelled) {
        GaussianProcess objective = GaussianProcess.fit(data, false);
        GaussianProcess safety = GaussianProcess.fit(data, true);
        double[] winner = null;
        double bestCost = Double.POSITIVE_INFINITY;
        double[] work = new double[data.size()];
        for (int i = 0; i < 2048; i++) {
            if (i % 64 == 0 && cancelled.getAsBoolean()) break;
            double[] x = {random.nextDouble(), random.nextDouble(), random.nextDouble()};
            if (!isDistinct(finalists, x)) continue;
            double[] limit = safety.predict(x, work);
            if (limit[0] < 0.02) continue;
            double[] cost = objective.predict(x, work);
            if (cost[0] < bestCost) {
                bestCost = cost[0];
                winner = x;
            }
        }
        return winner;
    }

    private double[] sampleNovel(double[] center, double radius) {
        double[] best = sample(center, radius);
        double bestNearest = nearestDistance(best);
        if (bestNearest >= 0.04) return best;
        for (int attempt = 1; attempt < 32; attempt++) {
            double[] candidate = sample(center, radius);
            double nearest = nearestDistance(candidate);
            if (nearest >= 0.04) return candidate;
            if (nearest > bestNearest) {
                bestNearest = nearest;
                best = candidate;
            }
        }
        return best;
    }

    private double nearestDistance(double[] x) {
        double nearest = Double.POSITIVE_INFINITY;
        for (Observation o : data) nearest = Math.min(nearest, distance(x, o.x));
        return nearest;
    }

    private static boolean isDistinct(List<double[]> existing, double[] candidate) {
        return existing.stream().noneMatch(x -> distance(x, candidate) <= 0.035);
    }

    private double[] sample(double[] center, double radius) {
        double[] x = new double[3];
        for (int i = 0; i < x.length; i++) {
            double lo = Math.max(0, center[i] - radius), hi = Math.min(1, center[i] + radius);
            x[i] = lo + random.nextDouble() * (hi - lo);
        }
        return x;
    }

    private static double validationScore(double[] scores) {
        return 0.65 * Maths.median(scores, scores.length)
                + 0.35 * Arrays.stream(scores).max().orElse(Double.POSITIVE_INFINITY);
    }

    private double[] normalize(double[] gains) {
        double[] x = new double[3];
        for (int i = 0; i < x.length; i++) {
            double denom = Math.max(1e-6, scale[i]);
            x[i] = Math.log1p((Maths.clip(gains[i], min[i], max[i]) - min[i]) / denom)
                    / Math.log1p((max[i] - min[i]) / denom);
        }
        return x;
    }

    private double[] gains(double[] x) {
        double[] gains = new double[3];
        for (int i = 0; i < x.length; i++) {
            double denom = Math.max(1e-6, scale[i]);
            gains[i] = min[i] + denom * Math.expm1(x[i] * Math.log1p((max[i] - min[i]) / denom));
        }
        return gains;
    }

    private static double distance(double[] a, double[] b) {
        return Math.sqrt(square(a[0] - b[0]) + square(a[1] - b[1]) + square(a[2] - b[2]));
    }

    private static double square(double value) { return value * value; }

    private static final class Observation {
        final double[] x;
        final Result result;

        Observation(double[] x, Result result) {
            this.x = x;
            this.result = result;
        }
    }

    private static final class GaussianProcess {
        final List<Observation> data;
        final double mean, deviation, length, noise, loss;
        final double[][] lower;
        final double[] alpha;

        static GaussianProcess fit(List<Observation> data, boolean constraint) {
            double[] y = data.stream().mapToDouble(o -> constraint ? o.result.margin : o.result.cost).toArray();
            double mean = Arrays.stream(y).average().orElse(0);
            double deviation = Math.max(constraint ? 0.12 : 0.10,
                    Math.sqrt(Arrays.stream(y).map(v -> square(v - mean)).average().orElse(0)));
            for (int i = 0; i < y.length; i++) y[i] = (y[i] - mean) / deviation;
            GaussianProcess best = null;
            for (double length : new double[]{0.10, 0.18, 0.30, 0.48, 0.75}) {
                for (double noise : new double[]{0.015, 0.04, 0.12}) {
                    GaussianProcess model = new GaussianProcess(data, y, mean, deviation, length, noise);
                    if (best == null || model.loss < best.loss) best = model;
                }
            }
            return best;
        }

        GaussianProcess(List<Observation> data, double[] y, double mean, double deviation, double length,
                        double noise) {
            this.data = data;
            this.mean = mean;
            this.deviation = deviation;
            this.length = length;
            this.noise = noise;
            lower = new double[data.size()][data.size()];
            double likelihood = 0;
            for (int i = 0; i < data.size(); i++) {
                for (int j = 0; j <= i; j++) {
                    double value = i == j ? 1 + noise : kernel(data.get(i).x, data.get(j).x);
                    for (int k = 0; k < j; k++) value -= lower[i][k] * lower[j][k];
                    lower[i][j] = i == j ? Math.sqrt(value) : value / lower[j][j];
                }
                likelihood += Math.log(lower[i][i]);
            }
            alpha = y.clone();
            forward(alpha);
            for (double value : alpha) likelihood += 0.5 * value * value;
            loss = likelihood;
            for (int i = data.size() - 1; i >= 0; i--) {
                for (int j = i + 1; j < data.size(); j++) alpha[i] -= lower[j][i] * alpha[j];
                alpha[i] /= lower[i][i];
            }
        }

        double[] predict(double[] x, double[] work) {
            double value = 0;
            for (int i = 0; i < data.size(); i++) {
                work[i] = kernel(x, data.get(i).x);
                value += alpha[i] * work[i];
            }
            forward(work);
            double variance = 1 + noise * 0.25;
            for (int i = 0; i < data.size(); i++) variance -= square(work[i]);
            return new double[]{mean + deviation * value, deviation * Math.sqrt(Math.max(0, variance))};
        }

        private void forward(double[] values) {
            for (int i = 0; i < data.size(); i++) {
                for (int j = 0; j < i; j++) values[i] -= lower[i][j] * values[j];
                values[i] /= lower[i][i];
            }
        }

        private double kernel(double[] a, double[] b) {
            double d = Math.sqrt(5) * distance(a, b) / length;
            return (1 + d + d * d / 3) * Math.exp(-d);
        }
    }
}
