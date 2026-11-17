package org.firstinspires.ftc.teamcode.biobuzz.calibration;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.biobuzz.CalibrationStore;
import org.firstinspires.ftc.teamcode.biobuzz.Turret;
import org.firstinspires.ftc.teamcode.biobuzz.TurretPlant;

import java.util.List;
import java.util.Locale;

@TeleOp(name = "05 自测云台 PID 与回转", group = "BIOBUZZ 自测")
public final class TurretPidCalibration extends CalibrationOp {
    private static final double HOLD = 0.12;
    private static final double[] VERIFICATION_CENTERS = {0.27, 0.35, 0.24, 0.39};
    private int trials;
    private BayesianPidSearch completed;
    private String outcome;
    private String rewindOutcome = "未运行";
    private double peakPower;

    @Override String intro() { return "单独拟合云台 PID，使用已保存的行程、摩擦和刹车模型。"; }

    @Override void calibrate() throws InterruptedException {
        Turret turret = new Turret(hardwareMap, store);
        try {
            if (!turret.calibrated()) {
                telemetry.log().add("尚无云台方向和行程，请先运行 02。");
                return;
            }
            turret.teachingMode(true);
            while (confirm("把云台摆到最右标记，停稳后按 A 记零位。")) {
                turret.homeAtRight();
                if (turret.encoderReady()) break;
            }
            if (!active() || !turret.encoderReady()) return;
            turret.teachingMode(false);
            if (!confirm("A 开始 20–32 轮搜索及留出复测。停滞会提前收尾。")) return;
            TurretPlant plant = TurretPlant.load(store);
            double[] seed = plant.pidSeed(plant.baseline());
            double[] current = store.read(CalibrationStore.TURRET_PID, seed);
            BayesianPidSearch search = new BayesianPidSearch(
                    new double[]{Math.max(4.0, 1.6 * current[0]), Math.max(0.80, 1.6 * current[1]),
                            Math.max(0.32, 1.8 * current[2])},
                    new double[]{0.9, 0.10, 0.07});
            BayesianPidSearch.Selection best = search.tune(current,
                    (gains, verification, pattern) -> trial(turret, current, gains, verification, pattern),
                    () -> !active(),
                    store.readHistory(CalibrationStore.TURRET_PID_HISTORY, BayesianPidSearch.HISTORY_STRIDE,
                            BayesianPidSearch.HISTORY_ROWS));
            completed = search;
            if (active() && search.searchCount() > 0) {
                save(CalibrationStore.TURRET_PID_HISTORY, search.history());
            }
            double[] selected = current;
            if (best != null && active()) {
                boolean saved = save(CalibrationStore.TURRET_PID, best.gains);
                if (saved) selected = best.gains;
                outcome = saved ? String.format(Locale.US, "已保存，复测代价 %.3f", best.score)
                        : "复测通过，写入失败";
                telemetry.log().add("%s，P/I/D %.3f / %.3f / %.3f",
                        outcome, best.gains[0], best.gains[1], best.gains[2]);
            } else if (active()) {
                outcome = search.searchCount() < BayesianPidSearch.MIN_ROUNDS ? "有效搜索轮数不足，保留当前 PID"
                        : "没有一组参数全部通过复测，保留当前 PID";
                telemetry.log().add("%s", outcome);
            }
            if (active()) {
                turret.setPidGains(selected);
                rewindOutcome = new TurretRewindCheck(this, turret).run();
            }
        } finally {
            turret.teachingMode(false);
        }
    }

    @Override void showResults() {
        if (completed == null) return;
        telemetry.addData("结果", outcome == null ? "中止" : outcome);
        telemetry.addData("搜索", "历史 %d / 本轮 %d，最低 %s，合格最低 %s",
                completed.historyCount(), completed.searchCount(), costText(completed.bestSearchCost()),
                costText(completed.bestQualifiedSearchCost()));
        telemetry.addData("PID 动作实测峰值", "%.2f", peakPower);
        telemetry.addData("PID 回转", rewindOutcome);
        List<BayesianPidSearch.Review> results = completed.reviews();
        for (int i = 0; i < results.size(); i++) {
            BayesianPidSearch.Review row = results.get(i);
            String title = i == 0 ? "原 PID" : "候选 " + i;
            telemetry.addData(title, "%d/4 组通过，%s", row.passed,
                    row.firstFailure == null ? "复测代价 " + costText(row.score) : row.firstFailure);
            telemetry.addData(title + " P/I/D", "%.3f / %.3f / %.3f", row.gains[0], row.gains[1], row.gains[2]);
        }
    }

    private static String costText(double cost) {
        return Double.isFinite(cost) ? String.format(Locale.US, "%.3f", cost) : "无";
    }

    private BayesianPidSearch.Result trial(Turret turret, double[] current, double[] gains,
                                           boolean verification, int pattern) throws InterruptedException {
        double span = turret.span();
        double sign = pattern % 2 == 0 ? 1 : -1;
        double band = verification ? VERIFICATION_CENTERS[pattern] : 0.24 + 0.055 * (pattern / 2);
        double center = span * (sign > 0 ? band : 1 - band);
        turret.setPidGains(current);
        if (!TurretMotion.go(this, turret, center, Math.toRadians(2.2))) {
            return BayesianPidSearch.Result.unavailable(String.format(Locale.US, "起始角 %.1f° 就位失败",
                    Math.toDegrees(center)));
        }
        turret.setPidGains(gains);
        trials++;
        double small = Math.min(Math.toRadians(18), span * 0.13);
        double large = Math.min(Math.toRadians(105), span * 0.29);
        double[] targets = {center - sign * small, center, center + sign * large, center};
        double total = 0, worst = 0, margin = 0.34;
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
            TurretStability.Result run = TurretStability.pidStep(this, turret, target,
                    (verification ? "云台 PID 复测 " : "云台 PID 搜索 ") + trials, i >= 2);
            peakPower = Math.max(peakPower, run.peakPower);
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
        telemetry.log().add("云台第 %d 轮 %.3f，超调裕量 %+.2f%s",
                trials, score, margin, settled && margin >= 0 ? "，停稳" : "");
        return new BayesianPidSearch.Result(score, margin, settled, firstFailure);
    }

    private boolean active() { return opModeIsActive() && !motionAborted(); }
}
