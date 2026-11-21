package org.firstinspires.ftc.teamcode.biobuzz.calibration;

import org.firstinspires.ftc.teamcode.biobuzz.CalibrationStore;
import org.firstinspires.ftc.teamcode.biobuzz.Drive;
import org.firstinspires.ftc.teamcode.biobuzz.Maths;
import org.firstinspires.ftc.teamcode.biobuzz.PinpointPose;
import org.firstinspires.ftc.teamcode.biobuzz.Turret;
import org.firstinspires.ftc.teamcode.biobuzz.TurretPlant;

import java.util.Locale;

final class TurretPidTuning {
    private static final double HOLD = 0.12;
    private static final double[] VERIFICATION_CENTERS = {0.27, 0.39};
    private static final int[] SEARCH_POSITIONS = {0, 2, 4, 5, 3, 1};
    private final CalibrationOp op;
    private int trials;
    private BayesianPidSearch completed;
    private String outcome = "未验收";
    private String rewindOutcome = "未运行";
    private double peakPower;

    TurretPidTuning(CalibrationOp op) {
        this.op = op;
    }

    boolean tune(Turret turret, Drive drive, PinpointPose pose) throws InterruptedException {
        double[] current = turret.pidGains();
        TurretPlant plant = TurretPlant.load(op.store);
        double[] seed = plant.pidSeed(plant.baseline());
        double[] min = {
                Math.min(0.55 * seed[0], 0.85 * current[0]),
                Math.min(0.40 * seed[1], 0.85 * current[1]),
                Math.min(0.30 * seed[2], 0.85 * current[2])
        };
        double[] max = {
                Math.max(1.65 * seed[0], 1.15 * current[0]),
                Math.max(1.85 * seed[1], 1.15 * current[1]),
                Math.max(1.75 * seed[2] + 0.04, 1.15 * current[2] + 0.01)
        };
        BayesianPidSearch search = new BayesianPidSearch(min, max,
                new double[]{seed[0], seed[1], Math.max(0.03, seed[2])});
        double[] positioningGains = drive == null ? current : seed;
        completed = search;
        boolean saved = false;
        try {
            double[] history = op.store.turretStale() ? new double[0]
                    : op.store.readHistory(CalibrationStore.TURRET_PID_HISTORY, BayesianPidSearch.HISTORY_STRIDE,
                            BayesianPidSearch.HISTORY_ROWS);
            search.tune(current,
                    (gains, verification, pattern) -> trial(turret, positioningGains, gains, verification, pattern),
                    () -> !active(), history);
            if (!active()) {
                outcome = "云台 PID 测量已停止";
                return false;
            }
            if (search.searchCount() > 0) op.save(CalibrationStore.TURRET_PID_HISTORY, search.history());

            BayesianPidSearch.Review best = search.bestCandidate();
            BayesianPidSearch.Review original = search.incumbent();
            if (original != null && original.qualified() && (best == null || original.score <= best.score)) best = original;
            if (best == null) {
                outcome = "本轮缺少通过留出复测的 PID";
                return false;
            }
            boolean full = drive != null;
            turret.setPidGains(best.gains);
            if (TurretRewindCheck.supported(turret)) {
                TurretRewindCheck check = new TurretRewindCheck(op, turret);
                rewindOutcome = full ? check.run(drive, pose) : check.run(true);
                if (!check.passed() || !active()) {
                    outcome = "双向回转未通过，保留原 PID";
                    return false;
                }
            } else rewindOutcome = "行程不足 270°，按双组阶跃验收";
            turret.stop();
            if (!full && best != original && !op.choosePid(search, best, () -> op.telemetry.addData("回转", rewindOutcome))) {
                outcome = "保留旧 PID";
                return false;
            }
            saved = active() && op.store.saveTurretPid(best.gains, best.steadyError);
            outcome = saved ? String.format(Locale.ROOT, "%s，验收代价 %.3f，最大末端误差 %.2f°",
                    best == original ? "原 PID 通过复测" : "已保存新 12V PID",
                    best.score, Math.toDegrees(best.steadyError)) : "PID 未写入";
            op.telemetry.log().add("%s，P/I/D %.3f / %.3f / %.3f", outcome, best.gains[0], best.gains[1], best.gains[2]);
            return saved;
        } finally {
            turret.stop();
            if (!saved) turret.setPidGains(current);
        }
    }

    void showResults() {
        op.telemetry.addData("云台 PID", outcome);
        op.telemetry.addData("回转验收", rewindOutcome);
        if (completed == null) return;
        op.telemetry.addData("PID 搜索", "历史 %d / 本轮 %d，最低 %s",
                completed.historyCount(), completed.searchCount(), costText(completed.bestSearchCost()));
        op.telemetry.addData("PID 动作峰值", "%.2f V", 12 * peakPower);
        op.showPidReview("进入校准时", completed.incumbent());
        op.showPidReview("最佳新候选", completed.bestCandidate());
    }

    void quickCheck(Turret turret) throws InterruptedException {
        double[] gains = turret.pidGains();
        op.motionDeadline = Maths.time() + 30;
        try {
            double residual = 0;
            for (int group = 0; group < VERIFICATION_CENTERS.length; group++) {
                double center = turret.span() * (group == 0 ? 0.30 : 0.70);
                if (!TurretMotion.go(op, turret, center, Math.toRadians(2.2))) {
                    outcome = "快检起始角就位失败";
                    return;
                }
                double step = (group == 0 ? 1 : -1) * Math.min(Math.toRadians(35), turret.span() * 0.19);
                for (double target : new double[]{center + step, center}) {
                    TurretStability.Result check = TurretStability.pidStep(op, turret, target, "PID 快检", false, false);
                    if (!active() || check.unsafe || check.readFailed || !Double.isFinite(check.response.settlingTime(HOLD))) {
                        outcome = "快检阶跃未通过";
                        return;
                    }
                    residual = Math.max(residual, Math.abs(check.endError));
                }
            }
            if (TurretRewindCheck.supported(turret)) {
                TurretRewindCheck check = new TurretRewindCheck(op, turret);
                rewindOutcome = check.run(false);
                if (!check.passed()) {
                    outcome = "快检回转未通过，" + rewindOutcome;
                    return;
                }
            } else rewindOutcome = "行程不足 270°，仅验收阶跃";
            outcome = active() && op.store.saveTurretPid(gains, residual)
                    ? String.format(Locale.ROOT, "快检通过，末端误差 %.2f°，%s", Math.toDegrees(residual), rewindOutcome)
                    : "快检结束，参数未写入";
        } finally {
            op.motionDeadline = Double.POSITIVE_INFINITY;
        }
    }

    private static String costText(double cost) {
        return Double.isFinite(cost) ? String.format(Locale.US, "%.3f", cost) : "无";
    }

    private BayesianPidSearch.Result trial(Turret turret, double[] positioningGains, double[] gains,
                                           boolean verification, int pattern) throws InterruptedException {
        double span = turret.span();
        if (!verification) pattern = SEARCH_POSITIONS[pattern];
        double sign = pattern % 2 == 0 ? 1 : -1;
        double band = verification ? VERIFICATION_CENTERS[pattern] : 0.24 + 0.055 * (pattern / 2);
        double center = span * (sign > 0 ? band : 1 - band);
        turret.setPidGains(positioningGains);
        if (!TurretMotion.go(op, turret, center, Math.toRadians(2.2))) {
            return BayesianPidSearch.Result.unavailable(String.format(Locale.US, "起始角 %.1f° 就位失败",
                    Math.toDegrees(center)));
        }
        turret.setPidGains(gains);
        trials++;
        double small = Math.min(Math.toRadians(18), span * 0.13);
        double large = Math.min(Math.toRadians(105), span * 0.29);
        double[] targets = {center - sign * small, center, center + sign * large, center};
        double total = 0, worst = 0, margin = 0.34, steadyError = 0;
        boolean settled = true;
        String firstFailure = "";
        for (int i = 0; i < targets.length; i++) {
            double target = targets[i];
            if (!active()) return null;
            double change = target - turret.angle;
            String step = String.format(Locale.US, "动作 %d（%+.1f°）", i + 1, Math.toDegrees(change));
            if (Math.abs(change) < Math.toRadians(3)) {
                total += 4;
                worst = Math.max(worst, 4);
                settled = false;
                if (firstFailure.isEmpty()) firstFailure = step + " 未形成有效阶跃";
                continue;
            }
            TurretStability.Result run = TurretStability.pidStep(op, turret, target,
                    (verification ? "云台 PID 复测 " : "云台 PID 搜索 ") + trials, i >= 2, verification);
            peakPower = Math.max(peakPower, run.peakPower);
            steadyError = Math.max(steadyError, Math.abs(run.endError));
            if (!active()) return null;
            if (run.readFailed) return BayesianPidSearch.Result.unavailable(step + " 编码器或电机读数中断");
            if (run.unsafe) return new BayesianPidSearch.Result(10, -0.6, false,
                    step + (run.limitReached ? " 触发安全行程" : " 超调超过 8°"));
            StepScore response = run.response;
            boolean arrived = Double.isFinite(response.settlingTime(HOLD));
            margin = Math.min(margin, 0.34 - response.overshoot());
            if (firstFailure.isEmpty()) {
                if (response.overshoot() > 0.34) {
                    firstFailure = String.format(Locale.US, "%s 超调 %.0f%%（上限 34%%）",
                            step, response.overshoot() * 100);
                } else if (!arrived) {
                    firstFailure = String.format(Locale.US, "%s 未停稳，末端误差 %+.1f°，速度 %+.0f°/s", step,
                            Math.toDegrees(run.endError), Math.toDegrees(run.endSpeed));
                }
            }
            double cost = Math.min(9, arrived ? response.scoreWithSettlingTime(HOLD) : 2.6 + response.score());
            total += cost;
            worst = Math.max(worst, cost);
            settled &= arrived;
        }
        double score = 0.70 * total / targets.length + 0.30 * worst;
        op.telemetry.log().add("云台第 %d 轮 %.3f，超调裕量 %+.2f%s",
                trials, score, margin, settled && margin >= 0 ? "，停稳" : "");
        return new BayesianPidSearch.Result(score, margin, settled, firstFailure, steadyError);
    }

    private boolean active() { return op.opModeIsActive() && !op.motionAborted(); }
}
